#!/usr/bin/env python3
"""
Generic Zenoh → JSONL logger for dataset generation and message traceability.

Subscribes to any Zenoh key expression and appends timestamped JSONL records
to a file.  Works for any JSON payload (detection results, SLAM poses, etc.).

Each output line is a self-contained JSON record:
  {"ts": 1740000000.123, "iso": "2026-02-20T10:00:00.123Z",
   "key": "tb/detections", "payload": <parsed JSON or raw string>}

Usage:
  # Object detections
  python3 zenoh_logger.py --key tb/detections \
      --output /data/detections/detections.jsonl

  # SLAM poses (run two instances for pose + status)
  python3 zenoh_logger.py --key tb/slam/pose \
      --output /data/slam/poses.jsonl
  python3 zenoh_logger.py --key tb/slam/status \
      --output /data/slam/status.jsonl

  # Custom router
  python3 zenoh_logger.py --key tb/detections --connect tcp/localhost:7447
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
    parser = argparse.ArgumentParser(description="Generic Zenoh → JSONL logger")
    parser.add_argument(
        "-e",
        "--connect",
        type=str,
        default="",
        help="Zenoh endpoint (empty = peer/multicast)",
    )
    parser.add_argument(
        "--key",
        type=str,
        required=True,
        help="Zenoh key expression to subscribe to (e.g. tb/detections)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        required=True,
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

    conf = zenoh.Config()
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

    session = zenoh.open(conf)
    outfile = open(output_path, "a", buffering=1)  # line-buffered

    print(f"Logging '{args.key}' → {output_path}", flush=True)

    def on_sample(sample):
        nonlocal record_count

        now = time.time()
        iso = (
            datetime.fromtimestamp(now, tz=timezone.utc).strftime(
                "%Y-%m-%dT%H:%M:%S.%f"
            )[:-3]
            + "Z"
        )

        raw = bytes(sample.payload)
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            payload = raw.decode(errors="replace")

        record = {
            "ts": now,
            "iso": iso,
            "key": str(sample.key_expr),
            "payload": payload,
        }
        outfile.write(json.dumps(record) + "\n")

        record_count += 1
        if args.flush_every == 0 or record_count % args.flush_every == 0:
            outfile.flush()

        print(f"[{iso}] {sample.key_expr} #{record_count}", flush=True)

    sub = session.declare_subscriber(args.key, on_sample)

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
