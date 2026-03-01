#!/usr/bin/env python3
"""
Query and export detection records.

Two modes:
  --source zenoh   Query the live Zenoh storage (in-session, in-memory backend).
                   Requires the zenoh-router to be running with the storage
                   configured (zenoh/zenoh-storage.json5).

  --source jsonl   Read from a recorded JSONL file produced by detection_logger.py.
                   Supports optional time-range filtering.

Output formats:
  --format json    Pretty-printed JSON array (default)
  --format jsonl   One JSON record per line
  --format csv     CSV: timestamp,iso,class,confidence,x1,y1,x2,y2
  --format summary Class frequency summary

Usage examples:
  # Query live Zenoh storage
  python3 query_detections.py --source zenoh --connect tcp/localhost:7447

  # Read JSONL, filter by time range, export CSV
  python3 query_detections.py --source jsonl \
      --input /data/detections/detections.jsonl \
      --after 2026-02-20T10:00:00 --before 2026-02-20T10:10:00 \
      --format csv --output run1.csv

  # Show detection class summary from JSONL
  python3 query_detections.py --source jsonl \
      --input /data/detections/detections.jsonl --format summary
"""

import argparse
import csv
import json
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import zenoh


def parse_args():
    parser = argparse.ArgumentParser(description="Query and export detection records")
    parser.add_argument(
        "--source",
        choices=["zenoh", "jsonl"],
        default="jsonl",
        help="Data source: live Zenoh storage or recorded JSONL file",
    )
    # Zenoh options
    parser.add_argument(
        "-e",
        "--connect",
        type=str,
        default="",
        help="Zenoh endpoint for --source zenoh",
    )
    parser.add_argument(
        "--key",
        type=str,
        default="tb/detections",
        help="Zenoh key expression to query",
    )
    # JSONL options
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        default="/data/detections/detections.jsonl",
        help="Input JSONL file for --source jsonl",
    )
    # Time range
    parser.add_argument(
        "--after",
        type=str,
        default="",
        help="Include records after this ISO 8601 timestamp",
    )
    parser.add_argument(
        "--before",
        type=str,
        default="",
        help="Include records before this ISO 8601 timestamp",
    )
    # Output
    parser.add_argument(
        "--format",
        choices=["json", "jsonl", "csv", "summary"],
        default="json",
        help="Output format",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="",
        help="Output file (default: stdout)",
    )
    return parser.parse_args()


def _parse_ts(iso: str) -> float:
    """Parse ISO 8601 string to Unix timestamp."""
    for fmt in ("%Y-%m-%dT%H:%M:%S", "%Y-%m-%dT%H:%M:%SZ", "%Y-%m-%d"):
        try:
            dt = datetime.strptime(iso, fmt).replace(tzinfo=timezone.utc)
            return dt.timestamp()
        except ValueError:
            continue
    raise ValueError(f"Cannot parse timestamp: {iso!r}")


def fetch_from_zenoh(args) -> list:
    """Query the in-memory Zenoh storage using get()."""
    conf = zenoh.Config()
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

    session = zenoh.open(conf)
    records = []

    replies = session.get(args.key)
    for reply in replies:
        if reply.ok is not None:
            sample = reply.ok
            ts = time.time()
            try:
                detections = json.loads(bytes(sample.payload))
            except json.JSONDecodeError:
                continue
            records.append({"ts": ts, "iso": sample.key_expr, "detections": detections})

    session.close()
    return records


def fetch_from_jsonl(args) -> list:
    """Read records from a JSONL file, applying optional time filters."""
    path = Path(args.input)
    if not path.exists():
        print(f"Error: file not found: {path}", file=sys.stderr)
        sys.exit(1)

    ts_after = _parse_ts(args.after) if args.after else None
    ts_before = _parse_ts(args.before) if args.before else None

    records = []
    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                print(f"Line {lineno}: JSON error: {exc}", file=sys.stderr)
                continue
            ts = record.get("ts", 0)
            if ts_after and ts < ts_after:
                continue
            if ts_before and ts > ts_before:
                continue
            records.append(record)

    return records


def output_json(records, outfile):
    json.dump(records, outfile, indent=2)
    outfile.write("\n")


def output_jsonl(records, outfile):
    for r in records:
        outfile.write(json.dumps(r) + "\n")


def output_csv(records, outfile):
    writer = csv.writer(outfile)
    writer.writerow(["timestamp", "iso", "class", "confidence", "x1", "y1", "x2", "y2"])
    for r in records:
        ts = r.get("ts", "")
        iso = r.get("iso", "")
        for det in r.get("detections", []):
            bbox = det.get("bbox", [None, None, None, None])
            writer.writerow(
                [
                    ts,
                    iso,
                    det.get("class", ""),
                    det.get("confidence", ""),
                    *bbox,
                ]
            )


def output_summary(records, outfile):
    class_counts: Counter = Counter()
    total_frames = len(records)
    frames_with_detections = 0

    for r in records:
        dets = r.get("detections", [])
        if dets:
            frames_with_detections += 1
        for d in dets:
            class_counts[d.get("class", "unknown")] += 1

    outfile.write(f"Total frames recorded : {total_frames}\n")
    outfile.write(f"Frames with detections: {frames_with_detections}\n")
    outfile.write(f"Total detections      : {sum(class_counts.values())}\n\n")
    outfile.write("Detection class counts:\n")
    for cls, count in class_counts.most_common():
        outfile.write(f"  {cls:<30} {count}\n")


def main():
    import time  # local import so it's available in fetch_from_zenoh closure

    args = parse_args()

    if args.source == "zenoh":
        records = fetch_from_zenoh(args)
    else:
        records = fetch_from_jsonl(args)

    outfile = open(args.output, "w") if args.output else sys.stdout

    try:
        if args.format == "json":
            output_json(records, outfile)
        elif args.format == "jsonl":
            output_jsonl(records, outfile)
        elif args.format == "csv":
            output_csv(records, outfile)
        elif args.format == "summary":
            output_summary(records, outfile)
    finally:
        if args.output:
            outfile.close()

    if args.output:
        print(f"Wrote {len(records)} records to {args.output}")


if __name__ == "__main__":
    main()
