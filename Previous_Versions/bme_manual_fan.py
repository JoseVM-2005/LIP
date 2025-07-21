#!/usr/bin/env python3
"""
bme_manual_logger.py

Continuously logs BME280 sensor data once per second (integer‐exact compensation),
and lets the user manually control a PWM fan on GPIO12 via simple terminal commands:
  - 0–100    : set fan duty
  - status   : show most recent reading
  - exit     : quit

Includes clean shutdown, thread-safe DB/state updates, console warnings, and sensor auto-reinit.
"""

import threading
import time
import sqlite3
import logging
import signal
import smbus
import RPi.GPIO as GPIO
from datetime import datetime
from logging.handlers import RotatingFileHandler

# === Configuration ===
I2C_ADDRESS    = 0x76
I2C_BUS        = 1
FAN_PIN        = 12        # BCM GPIO12 (pin 32)
PWM_FREQ       = 1000      # Hz
DB_FILE        = 'bme_readings.db'
LOG_FILE       = 'bme_manual_logger.log'
SAMPLE_RATE    = 1.0       # seconds per sample
I2C_RETRIES    = 3
ERROR_BACKOFF  = 0.5       # seconds between retries
REINIT_LIMIT   = 5         # re-instantiate sensor after this many failures

# === Shutdown Event ===
stop_event = threading.Event()
def _handle_signal(sig, frame):
    stop_event.set()
signal.signal(signal.SIGINT, _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)

# === Logging ===
logger = logging.getLogger("bme_logger")
logger.setLevel(logging.INFO)

fh = RotatingFileHandler(LOG_FILE, maxBytes=1_000_000, backupCount=3)
fh.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(message)s'))
logger.addHandler(fh)

ch = logging.StreamHandler()
ch.setLevel(logging.WARNING)
ch.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(message)s'))
logger.addHandler(ch)

# === GPIO/PWM Init ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT)
pwm = GPIO.PWM(FAN_PIN, PWM_FREQ)
pwm.start(0)

# === SQLite Init (thread-safe) ===
conn = sqlite3.connect(DB_FILE, check_same_thread=False)
conn.execute('''
    CREATE TABLE IF NOT EXISTS readings (
      id    INTEGER PRIMARY KEY AUTOINCREMENT,
      ts    TEXT,
      temp  REAL,
      pres  REAL,
      hum   REAL,
      fan   INTEGER,
      adc7  INTEGER
    )
''')
conn.commit()

# === Shared State ===
fan_duty     = 0
fan_lock     = threading.Lock()
last_reading = (None, None, None, None, None)  # (ts, temp, pres, hum, adc7)
reading_lock = threading.Lock()

# === ADC128D818 Driver === #
class ADC128D818:
    def __init__(self, address=0x1D, bus_no=I2C_BUS):
        self.address = address
        self.bus = smbus.SMBus(bus_no)
        self._init_device()

    def _init_device(self):
        self.bus.write_byte_data(self.address, 0x00, 0x01)  # Reset config
        self.bus.write_byte_data(self.address, 0x02, 0x80)  # Enable only channel 7 (bit 7)
        self.bus.write_byte_data(self.address, 0x03, 0x01)  # Enable single-ended inputs
        self.bus.write_byte_data(self.address, 0x00, 0x81)  # Start conversions (bit 7 = start, bit 0 = config)

    def read_channel(self, ch):
        if not 0 <= ch <= 7:
            raise ValueError("Channel must be 0–7")
        reg = 0x20 + ch
        raw = self.bus.read_word_data(self.address, reg)
        return (raw >> 8) | ((raw & 0xFF) << 8) >> 4  # Convert to 12-bit MSB-first

# === BME280 Driver === #
# (Unchanged - truncated here for brevity)
# You can keep your original BME280 driver implementation unchanged.

# === Acquisition Thread ===
def acquisition_loop():
    global last_reading
    sensor = BME280()
    adc = ADC128D818()
    consecutive_errors = 0

    while not stop_event.is_set():
        start = time.time()

        for _ in range(I2C_RETRIES):
            try:
                t, p, h = sensor.read_compensated()
                adc7 = adc.read_channel(7)
                consecutive_errors = 0
                break
            except OSError as e:
                consecutive_errors += 1
                logger.warning(f"I²C read error ({consecutive_errors}): {e}")
                time.sleep(ERROR_BACKOFF)
        else:
            logger.error("All I²C retries failed—skipping sample")
            if consecutive_errors >= REINIT_LIMIT:
                logger.warning("Reinitializing BME280 driver due to repeated failures")
                sensor = BME280()
                adc = ADC128D818()
                consecutive_errors = 0
            continue

        with fan_lock:
            duty = fan_duty
        pwm.ChangeDutyCycle(duty)

        ts = datetime.utcnow().isoformat()
        with reading_lock:
            last_reading = (ts, t, p, h, adc7)

        try:
            with conn:
                conn.execute(
                    "INSERT INTO readings (ts,temp,pres,hum,fan,adc7) VALUES (?,?,?,?,?,?)",
                    (ts, t, p, h, duty, adc7)
                )
        except Exception as e:
            logger.error(f"DB write failed: {e}")

        logger.info(f"T={t:.2f}°C P={p:.2f}hPa H={h:.2f}% Fan={duty}% ADC7={adc7}")

        elapsed = time.time() - start
        time.sleep(max(0, SAMPLE_RATE - elapsed))

# === User Input Loop ===
def user_input_loop():
    global fan_duty
    print("Enter 0–100 to set fan, 'status' to view last reading, 'exit' to quit.")
    while not stop_event.is_set():
        try:
            ui = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break

        if ui == "exit":
            stop_event.set()
            break

        if ui == "status":
            with reading_lock:
                ts, t, p, h, adc7 = last_reading
            if t is None:
                print("No data yet—please wait.")
            else:
                print(f"[{ts}] T={t:.2f}°C P={p:.2f}hPa H={h:.2f}% Fan={fan_duty}% ADC7={adc7}")
            continue

        try:
            val = int(ui)
            if 0 <= val <= 100:
                with fan_lock:
                    fan_duty = val
                print(f"Fan duty set to {val}%")
            else:
                print("Please enter a value between 0–100.")
        except ValueError:
            print("Invalid input. Use 0–100, 'status', or 'exit'.")

# === Main ===
def main():
    t = threading.Thread(target=acquisition_loop, daemon=True)
    t.start()
    user_input_loop()
    t.join()
    pwm.stop()
    GPIO.cleanup()
    conn.close()
    print("Shutdown complete.")

if __name__ == "__main__":
    main()
