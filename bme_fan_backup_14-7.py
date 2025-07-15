#!/usr/bin/env python3

import time
import numpy as np
import json
import threading
import signal
import smbus2
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from gpiozero import CPUTemperature



# === Config ===
BME280_ADDR     = 0x76
ADC128D818_ADDR = 0x1D
I2C_BUS         = 1

FAN_PIN         = 12
PWM_FREQ        = 100

MQTT_BROKER     = "localhost"
MQTT_PORT       = 1883
MQTT_TOPIC_PUB  = "sensor/data"
MQTT_TOPIC_SUB  = "fan/control"

# === GPIO Setup ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT)
pwm = GPIO.PWM(FAN_PIN, PWM_FREQ)
pwm.start(10)

fan_duty = 10
fan_lock = threading.Lock()
stop_event = threading.Event()

# === CPU Temperature ===
cpu = CPUTemperature()

def get_cpu_temperature():
    return cpu.temperature

# === BME280 Sensor ===
class BME280:
    def __init__(self, address=BME280_ADDR, bus_no=I2C_BUS):
        self.address = address
        self.bus = smbus2.SMBus(bus_no)
        self._load_calibration()
        self.bus.write_byte_data(address, 0xF2, 0x01)
        self.bus.write_byte_data(address, 0xF4, 0x27)
        self.bus.write_byte_data(address, 0xF5, 0xA0)

    def _signed(self, val):
        return val - 65536 if val & 0x8000 else val

    def _load_calibration(self):
        c1 = self.bus.read_i2c_block_data(self.address, 0x88, 24)
        c2 = self.bus.read_i2c_block_data(self.address, 0xA1, 1)
        c3 = self.bus.read_i2c_block_data(self.address, 0xE1, 7)
        self.dig_T1 = c1[0] | (c1[1] << 8)
        self.dig_T2 = self._signed(c1[2] | (c1[3] << 8))
        self.dig_T3 = self._signed(c1[4] | (c1[5] << 8))
        self.dig_P1 = c1[6] | (c1[7] << 8)
        self.dig_P2 = self._signed(c1[8] | (c1[9] << 8))
        self.dig_P3 = self._signed(c1[10] | (c1[11] << 8))
        self.dig_P4 = self._signed(c1[12] | (c1[13] << 8))
        self.dig_P5 = self._signed(c1[14] | (c1[15] << 8))
        self.dig_P6 = self._signed(c1[16] | (c1[17] << 8))
        self.dig_P7 = self._signed(c1[18] | (c1[19] << 8))
        self.dig_P8 = self._signed(c1[20] | (c1[21] << 8))
        self.dig_P9 = self._signed(c1[22] | (c1[23] << 8))
        self.dig_H1 = c2[0]
        self.dig_H2 = self._signed(c3[0] | (c3[1] << 8))
        self.dig_H3 = c3[2]
        self.dig_H4 = (c3[3] << 4) | (c3[4] & 0x0F)
        self.dig_H5 = (c3[5] << 4) | (c3[4] >> 4)
        self.dig_H6 = self._signed(c3[6])

    def read_raw(self):
        d = self.bus.read_i2c_block_data(self.address, 0xF7, 8)
        pres = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4)
        temp = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)
        hum = (d[6] << 8) | d[7]
        return temp, pres, hum

    def read_compensated(self):
        adc_T, adc_P, adc_H = self.read_raw()
        v1 = (((adc_T >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
        v2 = (((((adc_T >> 4) - self.dig_T1) ** 2) >> 12) * self.dig_T3) >> 14
        t_fine = v1 + v2
        T = (t_fine * 5 + 128) >> 8

        v1 = (t_fine >> 1) - 64000
        v2 = (((v1 >> 2) * (v1 >> 2)) >> 11) * self.dig_P6
        v2 += (v1 * self.dig_P5) << 1
        v2 = (v2 >> 2) + (self.dig_P4 << 16)
        v1 = (((self.dig_P3 * ((v1 >> 2) ** 2 >> 13)) >> 3) + ((self.dig_P2 * v1) >> 1)) >> 18
        v1 = ((32768 + v1) * self.dig_P1) >> 15
        pressure = 0 if v1 == 0 else ((1048576 - adc_P) - (v2 >> 12)) * 3125 // v1
        pressure /= 100

        h = t_fine - 76800
        h = (((((adc_H << 14) - (self.dig_H4 << 20) - (self.dig_H5 * h)) + 16384) >> 15) *
             (((((((h * self.dig_H6) >> 10) * (((h * self.dig_H3) >> 11) + 327680)) >> 10) + 2097152) *
               self.dig_H2 + 8192) >> 14))
        h -= (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
        h = max(0, min(h, 419430400))
        humidity = (h >> 12) / 1024.0

        return T / 100.0, pressure, humidity

# === ADC128D818 ===
class ADC128D818:
    def __init__(self, address=ADC128D818_ADDR, bus_no=I2C_BUS, chnls=[7]):
        self.address = address
        self.bus = smbus2.SMBus(bus_no)
        while (self.bus.read_byte_data(address, 0x0C) & 0x02) == 0x02:
            continue
        self.bus.write_byte_data(address, 0x00, 0x80)        # reset config
        self.bus.write_byte_data(address, 0x0B, 0b0000_0011)        # external ref.
        #elf.bus.write_byte_data(address, 0x07, 0x00) #llow power conversion
        for c in chnls:
            mask = ~(1<<c)
            self.bus.write_byte_data(address, 0x08, 0b1111_1111&mask)
        
        self.bus.write_byte_data(address, 0x00, 0x09)        # 

    def read_channel(self, ch):
        if not 0 <= ch <= 7:
            raise ValueError("Channel must be 0-7")
        self.bus.write_byte_data(self.address, 0x09, 0x01)
        #while (self.bus.read_byte_data(address, 0x0C) & 0x01) == 0x01:
         #   continue
        time.sleep(0.7)
            
        reg = 0x20 + ch
        value = self.bus.read_word_data(self.address, reg)

        low_byte = value & (0b0000_0000_1111_1111)
        high_byte = (value & (0b1111_1111_0000_0000))>>8
        
        clean_value = low_byte << 8 | (high_byte)
        

        return clean_value >> 4

# === MQTT Setup ===
mqtt_client = mqtt.Client()
mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)

def on_connect(client, userdata, flags, rc):
    print("MQTT connected.")
    client.subscribe(MQTT_TOPIC_SUB)

def on_message(client, userdata, msg):
    global fan_duty
    try:
        val = int(msg.payload.decode())
        if 0 <= val <= 100:
            with fan_lock:
                fan_duty = val
                pwm.ChangeDutyCycle(val)
                print(f"Fan set to {val}% via MQTT")
        else:
            print("Invalid fan value via MQTT")
    except Exception as e:
        print(f"MQTT msg error: {e}")

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# === Data Publisher ===
def sensor_loop():
    sensor = BME280()
    time.sleep(0.033)
    adc = ADC128D818()

    while not stop_event.is_set():
        
        try:
            temp, pres, hum = sensor.read_compensated()
            adc7 = adc.read_channel(7)
            cpu_temp = get_cpu_temperature()
            #temp_adc7 = adc_to _temp(adc7)
            

            with fan_lock:
                duty = fan_duty

            payload = {
                "temperature": temp,
                "pressure": pres,
                "humidity": hum,
                "adc7": adc7,
                #"adc_temp": temp_adc7,
                "cpu_temp": cpu_temp,
                "fan": duty
            }

            mqtt_client.publish(MQTT_TOPIC_PUB, json.dumps(payload))
            print(f"Sent: {payload}")

        except Exception as e:
            print(f"Error reading sensors: {e}")

        time.sleep(3)

def handle_shutdown(signum, frame):
    stop_event.set()

# === Run ===


def main():
    
    signal.signal(signal.SIGTERM, handle_shutdown)
    signal.signal(signal.SIGINT, handle_shutdown)
    # Start sensor_loop() in a daemon thread
    sensor_thread = threading.Thread(
        target=sensor_loop,
        name="SensorThread",
        daemon=True
    )
    sensor_thread.start()
    
    stop_event.wait()

    # Give sensor thread a moment to clean up
    sensor_thread.join(timeout=1)
    pwm.stop()
    GPIO.cleanup()
    mqtt_client.loop_stop()
    print("Shutdown complete.")

if __name__ == "__main__":
    main()


