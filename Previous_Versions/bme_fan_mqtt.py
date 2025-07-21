#!/usr/bin/env python3

import time
import json
import threading
import signal
import logging
import math

import smbus2
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from scipy.interpolate import CubicSpline
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

SAMPLE_INTERVAL = 1.0    # seconds between sensor publishes

# === Logging Setup (step 6) ===
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger(__name__)

# === Shutdown Event ===
stop_event = threading.Event()
def _handle_signal(sig, frame):
    log.info("Signal %s received; shutting down…", sig)
    stop_event.set()

signal.signal(signal.SIGINT,  _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)

# === GPIO / PWM ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT)
pwm = GPIO.PWM(FAN_PIN, PWM_FREQ)
pwm.start(10)   # default 10%

fan_duty   = 10
fan_lock   = threading.Lock()

# === MQTT Setup & Handlers (step 2 error logging) ===
mqtt_client = mqtt.Client()
mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        log.info("MQTT connected (rc=%s)", rc)
        client.subscribe(MQTT_TOPIC_SUB, qos=1)
    else:
        log.error("MQTT connect failed with rc=%s", rc)

def on_message(client, userdata, msg):
    global fan_duty
    raw = msg.payload.decode(errors="ignore")
    try:
        val = int(raw)
        if 0 <= val <= 100:
            with fan_lock:
                fan_duty = val
                pwm.ChangeDutyCycle(val)
            log.info("Fan set to %d%% via MQTT", val)
        else:
            log.warning("MQTT: fan value out of range: %r", raw)
    except ValueError:
        log.error("Invalid payload on %r: %r", msg.topic, raw)

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# === CPU Temperature ===
cpu = CPUTemperature()
def get_cpu_temperature():
    return cpu.temperature


# === PT1000 Convert ADC to Temp ===
a, b, c = 3.9083e-3, -5.775e-7, -4.183e-12
R0 = 1000  # Ω @ 0°C

temps = []
resistances = []
for t in range(-70, 40):
    if t < 0:
        r = R0 * (1 + a*t + b*t**2 + c*(t - 100)*t**3)
    else:
        r = R0 * (1 + a*t + b*t**2)
    temps.append(t)
    resistances.append(r)

spline = CubicSpline(resistances, temps)

def adc_to_temp(adc_values):
    """Convert raw ADC code to °C using the PT1000 spline."""
    # 12‑bit ADC: scale code to resistance in a divider with equal R0
    pt1000_vals={}
    for ch in adc_values:
        if adc_values[ch] == 0:
            pt1000_vals[ch] = -1000.1
        else:
            resistance = R0 * ((2**12) / adc_values[ch] - 1)
            pt1000_vals[ch] = float(spline(resistance))
    return pt1000_vals


# === BME280 Driver  ===
class BME280:
    def __init__(self, address=BME280_ADDR, bus_no=I2C_BUS):
        self.address = address
        self.bus = smbus2.SMBus(bus_no)
        self._load_calibration()
        for reg, val in ((0xF2,0x01),(0xF4,0x27),(0xF5,0xA0)):
            self.bus.write_byte_data(address, reg, val)
        self.t_fine = 0.0

    def _signed(self, val):
        return val - 65536 if val & 0x8000 else val

    def _load_calibration(self):
        c1 = self.bus.read_i2c_block_data(self.address, 0x88, 24)
        c2 = self.bus.read_i2c_block_data(self.address, 0xA1, 1)
        c3 = self.bus.read_i2c_block_data(self.address, 0xE1, 7)
        self.dig_T1 = c1[0] | (c1[1]<<8)
        self.dig_T2 = self._signed(c1[2] | (c1[3]<<8))
        self.dig_T3 = self._signed(c1[4] | (c1[5]<<8))
        self.dig_P1 = c1[6] | (c1[7]<<8)
        self.dig_P2 = self._signed(c1[8] | (c1[9]<<8))
        self.dig_P3 = self._signed(c1[10]| (c1[11]<<8))
        self.dig_P4 = self._signed(c1[12]| (c1[13]<<8))
        self.dig_P5 = self._signed(c1[14]| (c1[15]<<8))
        self.dig_P6 = self._signed(c1[16]| (c1[17]<<8))
        self.dig_P7 = self._signed(c1[18]| (c1[19]<<8))
        self.dig_P8 = self._signed(c1[20]| (c1[21]<<8))
        self.dig_P9 = self._signed(c1[22]| (c1[23]<<8))
        self.dig_H1 = c2[0]
        self.dig_H2 = self._signed(c3[0] | (c3[1]<<8))
        self.dig_H3 = c3[2]
        self.dig_H4 = (c3[3]<<4) | (c3[4]&0x0F)
        self.dig_H5 = (c3[5]<<4) | (c3[4]>>4)
        self.dig_H6 = self._signed(c3[6])

    def read_raw(self):
        d = self.bus.read_i2c_block_data(self.address, 0xF7, 8)
        adc_P = (d[0]<<12)|(d[1]<<4)|(d[2]>>4)
        adc_T = (d[3]<<12)|(d[4]<<4)|(d[5]>>4)
        adc_H = (d[6]<<8)|d[7]
        return adc_T, adc_P, adc_H

    def read_compensated(self):
        adc_T, adc_P, adc_H = self.read_raw()
        var1 = (adc_T/16384.0 - self.dig_T1/1024.0) * self.dig_T2
        var2 = ((adc_T/131072.0 - self.dig_T1/8192.0)**2)*self.dig_T3
        self.t_fine = var1 + var2
        temperature = (var1 + var2)/5120.0

        v1 = self.t_fine/2.0 - 64000.0
        v2 = v1*v1 * (self.dig_P6/32768.0)
        v2 += v1*self.dig_P5*2.0
        v2 = v2/4.0 + self.dig_P4*65536.0
        v1 = ((self.dig_P3*v1*v1/524288.0)+(self.dig_P2*v1))/524288.0
        v1 = (1.0+v1/32768.0)*self.dig_P1
        if v1 == 0.0:
            pressure = 0.0
        else:
            p = (1048576.0 - adc_P - v2/4096.0)*6250.0/v1
            v1 = self.dig_P9*p*p/2147483648.0
            v2 = p*self.dig_P8/32768.0
            p = p + (v1+v2+self.dig_P7)/16.0
            pressure = p

        h = self.t_fine - 76800.0
        h = (adc_H - (self.dig_H4*64.0 + self.dig_H5/16384.0*h)) * \
            (self.dig_H2/65536.0*(1.0+self.dig_H6/67108864.0*h*(1.0+self.dig_H3/67108864.0*h)))
        humidity = max(0.0, min(100.0, h))

        return temperature, pressure/100.0, humidity

# === ADC128D818 Driver ===
class ADC128D818:
    def __init__(self, address=ADC128D818_ADDR, bus_no=I2C_BUS, channels=(7,)):
        self.address = address
        self.bus = smbus2.SMBus(bus_no)
        self.channels = channels
        start = time.time()
        while (self.bus.read_byte_data(address, 0x0C) & 0x02):
            if time.time() - start > 0.5:
                raise RuntimeError("ADC init timeout")
            time.sleep(0.01)
        self.bus.write_byte_data(address, 0x00, 0x80)
        self.bus.write_byte_data(address, 0x0B, 0x03)
        mask = 0xFF & ~sum(1<<c for c in channels)
        self.bus.write_byte_data(address, 0x08, mask)
        self.bus.write_byte_data(address, 0x00, 0x09)



    def read_channels(self):
        self.bus.write_byte_data(self.address, 0x09, 0x01)
        start = time.time()
        while not (self.bus.read_byte_data(self.address, 0x0C) & 0x01):
            if time.time() - start > 0.5:
                raise RuntimeError("ADC read timeout")
            time.sleep(0.01)
        values = {}
        for ch in self.channels:
            raw = self.bus.read_word_data(self.address, 0x20 + ch)
            lo = raw & 0xFF
            hi = (raw >> 8) & 0xFF
            values[ch] = ((lo << 8) | hi) >> 4
        return values

# === Sensor publishing loop (with timing guard) ===
MAX_BME_ERRORS = 5  # step‑1 threshold

def sensor_loop():
    sensor = BME280()
    adc    = ADC128D818(channels=(6,5,7))
    next_time = time.time()
    bme_errors = 0

    while not stop_event.is_set():
        start = time.time()
        try:
            # --- attempt a BME280 read ---
            T, P, H = sensor.read_compensated()
            bme_errors = 0  # reset on success

            
            adc_vals = adc.read_channels()
            pt1000_vals   = adc_to_temp(adc_vals)
            cpu_t   = get_cpu_temperature()
            with fan_lock:
                duty = fan_duty
                
            adc_payload = {f'adc{ch}': adc_vals[ch] for ch in adc_vals}
            pt1000_payload = {f'adc{chnl}_temp': pt1000_vals[chnl] for chnl in pt1000_vals}

            environment = {
                "temperature": T,
                "pressure":    P,
                "humidity":    H,
                "cpu_temp":    cpu_t,
                "fan":         duty
            }
            
            payload = {**environment, **adc_payload, **pt1000_payload}
            mqtt_client.publish(
                MQTT_TOPIC_PUB,
                json.dumps( payload ),
                qos=1, retain=True
            )
            log.info("Sent: %s", payload )

        except Exception as e:
            bme_errors += 1
            log.error("BME280 read #%d failed: %s", bme_errors, e)

            if bme_errors >= MAX_BME_ERRORS:
                log.warning("Reached %d BME errors, re‑initializing sensor", MAX_BME_ERRORS)
                try:
                    sensor = BME280()
                    log.info("BME280 re‑init successful")
                    bme_errors = 0
                except Exception as e2:
                    log.error("BME280 re‑init failed: %s", e2)
            # skip doing ADC/mqtt on this iteration
        finally:
            # maintain exact SAMPLE_INTERVAL spacing, but interruptible
            next_time += SAMPLE_INTERVAL
            while not stop_event.is_set() and time.time() < next_time:
                time.sleep(0.05)

# === Run ===
def main():
    t = threading.Thread(target=sensor_loop, daemon=True)
    t.start()
    stop_event.wait()

    t.join(timeout=1)
    pwm.stop()
    GPIO.cleanup()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    log.info("Shutdown complete.")

if __name__ == "__main__":
    main()
