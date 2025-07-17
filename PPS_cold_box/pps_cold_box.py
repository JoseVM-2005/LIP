#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Fan controller & sensor‐publisher daemon for Raspberry Pi
# – Reads BME280 environmental sensor (T, P, RH)
# – Reads ADC128D818 channels and converts PT1000 resistances to °C
# – Publishes a merged JSON payload via MQTT every SAMPLE_INTERVAL
# – Listens for fan‐speed commands over MQTT and drives PWM on GPIO12
#

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

# === Configuration Constants ===
BME280_ADDR     = 0x76             # I²C address of BME280
ADC128D818_ADDR = 0x1D             # I²C address of ADC128D818
I2C_BUS         = 1                # /dev/i2c-1

FAN_PIN         = 12               # BCM pin for PWM fan control
PWM_FREQ        = 100              # PWM frequency in Hz

MQTT_BROKER     = "localhost"      # MQTT broker hostname
MQTT_PORT       = 1883             # MQTT broker port
MQTT_TOPIC_PUB  = "sensor/data"    # Topic to publish merged sensor data
MQTT_TOPIC_SUB  = "fan/control"    # Topic to receive fan‐speed commands

SAMPLE_INTERVAL = 1.0              # Seconds between sensor publishes
MAX_BME_ERRORS  = 5                # Retries before re‐init BME280

# PT1000 callendar‐van‐dusen coefficients
PT1000_R0 = 1000.0                 # Ω at 0 °C
PT1000_a  = 3.9083e-3
PT1000_b  = -5.775e-7
PT1000_c  = -4.183e-12              # only used below 0 °C

# === Logging Setup ===
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger(__name__)

# === Graceful Shutdown Handling ===
stop_event = threading.Event()

def _handle_signal(sig, frame):
    """
    Catches SIGINT/SIGTERM and notifies threads to clean up.
    """
    log.info("Signal %s received; shutting down…", sig)
    stop_event.set()

signal.signal(signal.SIGINT,  _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)

# === GPIO & PWM Initialization ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)             # use Broadcom pin numbering
GPIO.setup(FAN_PIN, GPIO.OUT)
pwm = GPIO.PWM(FAN_PIN, PWM_FREQ)
pwm.start(10)                      # default 10% duty cycle to understand when PI is ready

# Shared state for fan duty—protected by fan_lock
fan_duty = 10
fan_lock = threading.Lock()

# === MQTT Client Setup ===
mqtt_client = mqtt.Client()
mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)

def on_connect(client, userdata, flags, rc):
    """
    Called upon (re)connection to the MQTT broker.
    Subscribes to fan control topic if all OK.
    """
    if rc == 0:
        log.info("MQTT connected (rc=%s)", rc)
        client.subscribe(MQTT_TOPIC_SUB, qos=1)
    else:
        log.error("MQTT connection failed, rc=%s", rc)

def on_message(client, userdata, msg):
    """
    Handles incoming fan‐speed commands.
    Payload must be an integer 0…100.
    """
    global fan_duty
    raw = msg.payload.decode(errors="ignore")
    try:
        val = int(raw)
        if 0 <= val <= 100:
            with fan_lock:
                fan_duty = val
                pwm.ChangeDutyCycle(val)
            log.info("Fan duty set to %d%% via MQTT", val)
        else:
            log.warning("MQTT duty out of range: %r", raw)
    except ValueError:
        log.error("Invalid MQTT payload on %r: %r", msg.topic, raw)

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# === CPU Temperature Reader ===
cpu = CPUTemperature()
def get_cpu_temperature():
    """Return CPU temperature in °C via gpiozero."""
    return cpu.temperature

# === Build PT1000 Callendar–Van Dusen Spline ===
# Precompute lookup table spanning –70…+39 °C, then interpolate
temps = []
resistances = []
for t in range(-70, 40):
    if t < 0:
        # full CVD equation for negative T
        r = PT1000_R0 * (1 + PT1000_a*t + PT1000_b*t**2 + PT1000_c*(t-100)*t**3)
    else:
        # simplified quadratic for T ≥ 0 °C
        r = PT1000_R0 * (1 + PT1000_a*t + PT1000_b*t**2)
    temps.append(t)
    resistances.append(r)

# CubicSpline: maps resistance → temperature
spline = CubicSpline(resistances, temps)

def adc_to_temp(adc_values):
    """
    Convert raw 12-bit ADC readings (dict ch→code) into °C.
    Assumes PT1000 in series with identical PT1000_R0 forms a divider.
    Returns dict channel→temperature.
    """
    result = {}
    for ch, code in adc_values.items():
        if code == 0:
            result[ch] = math.nan
        else:
            # R_PT = R0 * (4095/code – 1)
            resistance = PT1000_R0 * ((2**12)/code - 1)
            # spline interpolates to °C
            result[ch] = float(spline(resistance))
    return result

# === BME280 Driver ===
class BME280:
    def __init__(self, address=BME280_ADDR, bus_no=I2C_BUS):
        self.address = address
        self.bus = smbus2.SMBus(bus_no)
        self._load_calibration()
        # humidity oversample ×1, pressure/temp ×1, normal mode
        for reg, val in ((0xF2, 0x01), (0xF4, 0x27), (0xF5, 0xA0)):
            self.bus.write_byte_data(address, reg, val)
        self.t_fine = 0.0

    def _signed(self, val):
        """Convert 16‑bit unsigned to signed."""
        return val - 65536 if val & 0x8000 else val

    def _load_calibration(self):
        """
        Read the 24+1+7 bytes of calibration registers and unpack
        temperature, pressure, and humidity coefficients (dig_T*, dig_P*, dig_H*).
        """
        c1 = self.bus.read_i2c_block_data(self.address, 0x88, 24)
        c2 = self.bus.read_i2c_block_data(self.address, 0xA1, 1)
        c3 = self.bus.read_i2c_block_data(self.address, 0xE1, 7)

        # Temperature calibration
        self.dig_T1 = c1[0] | (c1[1] << 8)
        self.dig_T2 = self._signed(c1[2] | (c1[3] << 8))
        self.dig_T3 = self._signed(c1[4] | (c1[5] << 8))
        # Pressure calibration
        self.dig_P1 = c1[6] | (c1[7] << 8)
        self.dig_P2 = self._signed(c1[8] | (c1[9] << 8))
        self.dig_P3 = self._signed(c1[10]| (c1[11]<< 8))
        self.dig_P4 = self._signed(c1[12]| (c1[13]<< 8))
        self.dig_P5 = self._signed(c1[14]| (c1[15]<< 8))
        self.dig_P6 = self._signed(c1[16]| (c1[17]<< 8))
        self.dig_P7 = self._signed(c1[18]| (c1[19]<< 8))
        self.dig_P8 = self._signed(c1[20]| (c1[21]<< 8))
        self.dig_P9 = self._signed(c1[22]| (c1[23]<< 8))
        # Humidity calibration
        self.dig_H1 = c2[0]
        self.dig_H2 = self._signed(c3[0] | (c3[1] << 8))
        self.dig_H3 = c3[2]
        e4, e5, e6 = c3[3], c3[4], c3[5]
        self.dig_H4 = (e4 << 4) | (e5 & 0x0F)
        self.dig_H5 = (e6 << 4) | (e5 >> 4)
        self.dig_H6 = self._signed(c3[6])

    def read_raw(self):
        """
        Pulls 8 data bytes from 0xF7–0xFE:
         – 20‑bit pressure: [0],[1],[2]
         – 20‑bit temperature: [3],[4],[5]
         – 16‑bit humidity: [6],[7]
        Returns (adc_T, adc_P, adc_H).
        """
        d = self.bus.read_i2c_block_data(self.address, 0xF7, 8)
        adc_P = (d[0] << 12) | (d[1] << 4)  | (d[2] >> 4)
        adc_T = (d[3] << 12) | (d[4] << 4)  | (d[5] >> 4)
        adc_H = (d[6] << 8)  | d[7]
        return adc_T, adc_P, adc_H

    def read_compensated(self):
        """
        Runs the Bosch‑provided double‑precision compensation formulas
        (rev 1.1) to turn raw ADC codes into human units.
        Returns (temperature °C, pressure hPa, humidity %RH).
        """
        adc_T, adc_P, adc_H = self.read_raw()

        # Temperature
        v1 = (adc_T/16384.0 - self.dig_T1/1024.0) * self.dig_T2
        v2 = ((adc_T/131072.0 - self.dig_T1/8192.0)**2) * self.dig_T3
        self.t_fine  = v1 + v2
        temperature = (v1 + v2) / 5120.0

        # Pressure
        var1 = self.t_fine/2.0 - 64000.0
        var2 = var1*var1*(self.dig_P6/32768.0)
        var2 += var1*self.dig_P5*2.0
        var2 = var2/4.0 + self.dig_P4*65536.0
        var1 = ((self.dig_P3*var1*var1/524288.0) + (self.dig_P2*var1)) / 524288.0
        var1 = (1.0 + var1/32768.0) * self.dig_P1
        if var1 == 0.0:
            pressure = 0.0  # avoid div0
        else:
            p = (1048576.0 - adc_P - var2/4096.0)*6250.0/var1
            v1 = (self.dig_P9 * (p*p/2147483648.0))
            v2 = (p*self.dig_P8/32768.0)
            p  = p + (v1+v2+self.dig_P7)/16.0
            pressure = p

        # Humidity
        h = self.t_fine - 76800.0
        h = (adc_H - (self.dig_H4*64.0 + self.dig_H5/16384.0*h)) \
            * (self.dig_H2/65536.0*(1.0 + self.dig_H6/67108864.0*h \
            *(1.0 + self.dig_H3/67108864.0*h)))
        humidity = max(0.0, min(100.0, h))

        return temperature, pressure/100.0, humidity

# === ADC128D818 Driver ===
class ADC128D818:
    def __init__(self, address=ADC128D818_ADDR, bus_no=I2C_BUS, channels=(7,)):
        """
        channels: tuple of 0–7
        After power‑up, poll STATUS bit until ready, configure external ref,
        mask off all channels except those in `channels`, then start conversions.
        """
        self.address  = address
        self.bus      = smbus2.SMBus(bus_no)
        self.channels = channels

        # wait for power‑up ready flag (STATUS bit 1 == 0)
        start = time.time()
        while self.bus.read_byte_data(address, 0x0C) & 0x02:
            if time.time() - start > 0.5:
                raise RuntimeError("ADC init timeout")
            time.sleep(0.01)

        # 0x00: reset + start, 0x0B: external ref enable
        self.bus.write_byte_data(address, 0x00, 0x80)
        self.bus.write_byte_data(address, 0x0B, 0x03)

        # 0x08: channel‐mask register (1=powered down). invert our list.
        mask = 0xFF & ~sum(1 << c for c in channels)
        self.bus.write_byte_data(address, 0x08, mask)

        # kick off continuous conversions
        self.bus.write_byte_data(address, 0x00, 0x09)

    def read_channels(self):
        """
        Trigger one-shot conversion on all enabled channels,
        wait for DRDY bit, then read each channel’s two‑byte result.
        Returns dict ch→12‑bit code.
        """
        # 0x09 bit0 = conversion start
        self.bus.write_byte_data(self.address, 0x09, 0x01)

        # wait for DRDY (STATUS bit0 == 1)
        start = time.time()
        while not (self.bus.read_byte_data(self.address, 0x0C) & 0x01):
            if time.time() - start > 0.5:
                raise RuntimeError("ADC read timeout")
            time.sleep(0.01)

        out = {}
        for ch in self.channels:
            raw = self.bus.read_word_data(self.address, 0x20 + ch)
            lo  = raw & 0xFF
            hi  = (raw >> 8) & 0xFF
            # combine hi/lo then right‑shift to get 12 bits
            out[ch] = ((lo << 8) | hi) >> 4
        return out

# === Sensor‐publishing Loop ===
def sensor_loop():
    """
    Runs in its own thread:
    – every SAMPLE_INTERVAL seconds
    – read BME280 → (T,P,H)
    – read ADC128D818 → dict ch→code → PT1000 °C
    – read CPU temp
    – snapshot fan_duty under lock
    – publish merged JSON to MQTT_TOPIC_PUB
    – on repeated BME280 failures, auto‑reinitialize
    """
    sensor    = BME280()
    adc       = ADC128D818(channels=(6, 7))
    next_time = time.time()
    bme_errors = 0

    while not stop_event.is_set():
        try:
            # 1) environmental read
            T, P, H = sensor.read_compensated()
            bme_errors = 0

            # 2) ADC & PT1000 conversion
            raw_codes    = adc.read_channels()
            pt1000_temps = adc_to_temp(raw_codes)

            # 3) CPU temp & fan snapshot
            cpu_t = get_cpu_temperature()
            with fan_lock:
                duty = fan_duty

            # 4) build payload
            env  = dict(temperature=T, pressure=P, humidity=H,
                        cpu_temp=cpu_t, fan=duty)
            adc_p  = {f"adc{ch}": code       for ch, code in raw_codes.items()}
            tmp_p  = {f"adc{ch}_temp": tmp   for ch, tmp  in pt1000_temps.items()}
            payload = {**env, **adc_p, **tmp_p}

            # 5) publish & log
            mqtt_client.publish(MQTT_TOPIC_PUB, json.dumps(payload),
                                qos=1, retain=True)
            log.info("Published payload: %s", payload)

        except Exception as exc:
            # track and recover from BME280 errors
            bme_errors += 1
            log.error("BME280 read #%d failed: %s", bme_errors, exc)
            if bme_errors >= MAX_BME_ERRORS:
                log.warning("Re‑initializing BME280 after %d errors", MAX_BME_ERRORS)
                try:
                    sensor = BME280()
                    bme_errors = 0
                    log.info("BME280 re‑init succeeded")
                except Exception as exc2:
                    log.error("BME280 re‑init failed: %s", exc2)

        finally:
            # throttle loop to exactly SAMPLE_INTERVAL seconds,
            # but wake early if stop_event is set
            next_time += SAMPLE_INTERVAL
            while not stop_event.is_set() and time.time() < next_time:
                time.sleep(0.01)

def main():
    """
    Start the sensor thread, then block until a shutdown signal arrives.
    Clean up GPIO, MQTT, and threads before exit.
    """
    thr = threading.Thread(target=sensor_loop, daemon=True)
    thr.start()

    # wait for Ctrl‑C or systemd stop
    stop_event.wait()

    # cleanup
    thr.join(timeout=1)
    pwm.stop()
    GPIO.cleanup()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    log.info("Shutdown complete.")

if __name__ == "__main__":
    main()
