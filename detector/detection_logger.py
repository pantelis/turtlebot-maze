#!/usr/bin/env python3
"""
Persistent detection logger for dataset generation.

Subscribes to the Zenoh key carrying YOLOv8 detection results (default:
tb/detections) and appends timestamped JSONL records to a file.

Each line is a self-contained JSON record:
  {"ts": 1740000000.123, "iso": "2026-02-20T10:00:00.123Z", "detections": [...]}

Usage:
  python3 detection_logger.py                          # defaults
  python3 detection_logger.py --output /data/detections/run1.jsonl
  python3 detection_logger.py --connect tcp/localhost:7447
"""

import argparse
import json
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import zenoh


def parse_args():
    parser = argparse.ArgumentParser(description="Zenoh detection logger")
    parser.add_argument(
        "-e",
        "--connect",
        type=str,
        default="",
        help="Zenoh endpoint (empty = multicast/peer)",
    )
    parser.add_argument(
        "--key",
        type=str,
        default="tb/detections",
        help="Zenoh key expression to subscribe to",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="/data/detections/detections.jsonl",
        help="Output JSONL file path",
    )
    parser.add_argument(
        "--flush-every",
        type=int,
        default=10,
        help="Flush output file every N records (0 = every record)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    record_count = 0

    # Open Zenoh session
    conf = zenoh.Config()
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

    session = zenoh.open(conf)

    outfile = open(output_path, "a", buffering=1)  # line-buffered
    print(f"Logging detections from '{args.key}' → {output_path}", flush=True)

    def on_detection(sample):
        nonlocal record_count

        now = time.time()
        iso = datetime.fromtimestamp(now, tz=timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%S.%f"
        )[:-3] + "Z"

        try:
            detections = json.loads(bytes(sample.payload))
        except json.JSONDecodeError as exc:
            print(f"JSON parse error: {exc}", file=sys.stderr)
            return

        record = {"ts": now, "iso": iso, "detections": detections}
        outfile.write(json.dumps(record) + "\n")

        record_count += 1
        if args.flush_every == 0 or record_count % args.flush_every == 0:
            outfile.flush()

        if detections:
            classes = [d["class"] for d in detections]
            print(f"[{iso}] {classes}", flush=True)

    sub = session.declare_subscriber(args.key, on_detection)

    def shutdown(sig, frame):
        print(f"\nShutting down — logged {record_count} records to {output_path}")
        sub.undeclare()
        outfile.flush()
        outfile.close()
        session.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("Logger running. Press Ctrl+C to stop.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
