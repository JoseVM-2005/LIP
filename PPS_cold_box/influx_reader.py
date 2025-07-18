#!/usr/bin/env python3
"""
Simple InfluxDB → pandas loader.

Usage:
  python3 influx_reader.py START_TIMESTAMP END_TIMESTAMP

Example:
  python3 influx_reader.py 2025-07-01T00:00:00Z 2025-07-02T00:00:00Z
"""
import sys
import pandas as pd
from influxdb import InfluxDBClient

# ── CONFIGURE HERE ─────────────────────────────────────────────────────────────
HOST        = "localhost"
PORT        = 8086
DATABASE    = "sensor_data"
MEASUREMENT = "environment"
# ────────────────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} START_TIMESTAMP END_TIMESTAMP")
        sys.exit(1)

    start, end = sys.argv[1], sys.argv[2]

    # Connect
    client = InfluxDBClient(host=HOST, port=PORT, database=DATABASE)


    # Build & run query
    #Adapt as necessary
    #q = (
    #    f'SELECT time, temperature, humidity FROM "{MEASUREMENT}" '
    #    f"WHERE time >= '{start}' AND time <= '{end}'"
    #)

    q = f"SELECT * FROM \"{MEASUREMENT}\" " \
        f"WHERE time >= '{start}' AND time <= '{end}'"
    res = client.query(q)

    # Extract points
    pts = list(res.get_points(measurement=MEASUREMENT))
    if not pts:
        print("No data in that range.")
        return

    # Build dataframe
    df = pd.DataFrame(pts)
    df["time"] = pd.to_datetime(df["time"])
    df.set_index("time", inplace=True)

    # Show/Extract as file
    print(df)
    #df.to_csv("out.csv")


if __name__ == "__main__":
    main()
