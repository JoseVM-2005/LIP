#!/usr/bin/env python3
"""
Simple InfluxDB → pandas loader.

Usage:
  python3 influx_reader.py START_TIMESTAMP END_TIMESTAMP

Example:
  python3 influx_reader.py 2025-07-01T00:00:00Z 2025-07-02T00:00:00Z
"""

import argparse
import logging
import sys

import pandas as pd
from influxdb import InfluxDBClient
from influxdb.exceptions import InfluxDBClientError

# ── USER CONFIG ────────────────────────────────────────────────────────────────
HOST        = "localhost"
PORT        = 8086
DATABASE    = "sensor_data"
MEASUREMENT = "environment"

EXPORT_CSV   = False    # ← set True to emit CSV
EXPORT_JSON  = False    # ← set True to emit JSON

CSV_OUTFILE  = "output.csv"
JSON_OUTFILE = "output.json"
# ────────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Load InfluxDB time-series points into a Pandas DataFrame"
    )
    p.add_argument(
        "start",
        help="RFC3339 start (e.g. 2025-07-01T00:00:00Z)"
    )
    p.add_argument(
        "end",
        help="RFC3339 end (e.g. 2025-07-02T00:00:00Z)"
    )
    return p.parse_args()

def main():
    # ——— Parse & validate inputs —————————————————————————————
    args = parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s"
    )
    log = logging.getLogger(__name__)

    try:
        t0 = pd.to_datetime(args.start)
        t1 = pd.to_datetime(args.end)
    except ValueError as e:
        log.error("Bad timestamp format: %s", e)
        sys.exit(1)

    if t1 < t0:
        log.error("End time %s precedes start time %s", t1, t0)
        sys.exit(1)

    log.info(
        "Querying InfluxDB '%s' @ %s:%d, measurement '%s'",
        DATABASE, HOST, PORT, MEASUREMENT
    )
    log.info("Time window: %s → %s", t0, t1)

    # ——— Connect & fetch ——————————————————————————————————————
     #Adapt as necessary
    #q = (
    #    f'SELECT time, temperature, humidity FROM "{MEASUREMENT}" '
    #    f"WHERE time >= '{start}' AND time <= '{end}'"
    #)

    client = InfluxDBClient(host=HOST, port=PORT, database=DATABASE)
    q = (
        f'SELECT * FROM "{MEASUREMENT}" '
        f"WHERE time >= '{args.start}' AND time <= '{args.end}'"
    )
    try:
        res = client.query(q)
    except InfluxDBClientError as e:
        log.error("InfluxDB query error: %s", e)
        sys.exit(1)
    except Exception as e:
        log.error("Unexpected error: %s", e)
        sys.exit(1)

    pts = list(res.get_points(measurement=MEASUREMENT))
    if not pts:
        log.warning("No data in the specified range.")
        client.close()
        sys.exit(0)

    # ——— Build DataFrame ——————————————————————————————————————
    df = pd.DataFrame(pts)
    df["time"] = pd.to_datetime(df["time"])
    df.set_index("time", inplace=True)
    log.info("Loaded %d records.", len(df))

    # ——— Export or print ——————————————————————————————————————
    if EXPORT_CSV:
        df.to_csv(CSV_OUTFILE)
        log.info("CSV exported to %s", CSV_OUTFILE)

    if EXPORT_JSON:
        df.to_json(JSON_OUTFILE, date_format="iso")
        log.info("JSON exported to %s", JSON_OUTFILE)

    if not EXPORT_CSV and not EXPORT_JSON:
        # Try a prettier table if tabulate’s installed
        try:
            print(df.to_markdown())
        except Exception:
            print(df.to_string())

    client.close()

if __name__ == "__main__":
    main()
