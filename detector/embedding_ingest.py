#!/usr/bin/env python3
"""
Zenoh-to-pgvector ingest worker.

Subscribes to tb/detections on the Zenoh router, decodes CLIP embeddings
from the JSON payload, and writes them to the pgvector PostgreSQL service.
"""

import argparse
import base64
import json
import time

import numpy as np
import psycopg2
import zenoh


def decode_embedding(b64_str: str) -> list[float]:
    """Decode a base64-encoded float32 vector to a Python list."""
    raw = base64.b64decode(b64_str)
    return np.frombuffer(raw, dtype=np.float32).tolist()


def main():
    parser = argparse.ArgumentParser(description="Zenoh → pgvector ingest worker")
    parser.add_argument(
        "-e",
        "--connect",
        type=str,
        default="tcp/localhost:7447",
        help="Zenoh endpoint to connect to",
    )
    parser.add_argument(
        "--key",
        type=str,
        default="tb/detections",
        help="Zenoh key to subscribe to",
    )
    parser.add_argument(
        "--pg-host", type=str, default="localhost", help="PostgreSQL host"
    )
    parser.add_argument(
        "--pg-port", type=int, default=5436, help="PostgreSQL port"
    )
    parser.add_argument(
        "--pg-db", type=str, default="vectordb", help="PostgreSQL database"
    )
    parser.add_argument(
        "--pg-user", type=str, default="postgres", help="PostgreSQL user"
    )
    parser.add_argument(
        "--pg-password", type=str, default="postgres", help="PostgreSQL password"
    )
    args = parser.parse_args()

    # Connect to PostgreSQL
    conn = psycopg2.connect(
        host=args.pg_host,
        port=args.pg_port,
        dbname=args.pg_db,
        user=args.pg_user,
        password=args.pg_password,
    )
    conn.autocommit = True
    cur = conn.cursor()
    print(f"Connected to PostgreSQL: {args.pg_host}:{args.pg_port}/{args.pg_db}")

    insert_count = 0

    def detection_callback(sample):
        nonlocal insert_count
        try:
            msg = json.loads(sample.payload.to_bytes())
        except Exception as e:
            print(f"JSON decode error: {e}")
            return

        # Handle envelope format: {keyframe_id, timestamp, map_x, map_y, map_yaw, detections: [...]}
        if isinstance(msg, dict) and "detections" in msg:
            detections = msg["detections"]
            kf_id = msg.get("keyframe_id")
            run_id = msg.get("run_id")
            map_x = msg.get("map_x")
            map_y = msg.get("map_y")
            map_yaw = msg.get("map_yaw")
        elif isinstance(msg, list):
            # Legacy flat list format
            detections = msg
            kf_id = None
            run_id = None
            map_x = map_y = map_yaw = None
        else:
            return

        for det in detections:
            embedding_b64 = det.get("embedding")
            if embedding_b64 is None:
                continue

            embedding = decode_embedding(embedding_b64)
            det_id = det.get("det_id")
            cur.execute(
                """INSERT INTO detection_embeddings
                   (run_id, det_id, keyframe_id, class_name, confidence, bbox,
                    map_x, map_y, map_yaw, embedding_model, embedding)
                   VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                   ON CONFLICT (det_id) DO NOTHING""",
                (
                    run_id,
                    det_id,
                    kf_id,
                    det["class"],
                    det["confidence"],
                    det["bbox"],
                    map_x,
                    map_y,
                    map_yaw,
                    det.get("embedding_model"),
                    str(embedding),
                ),
            )
            insert_count += 1

        if detections:
            classes = [d["class"] for d in detections if d.get("embedding")]
            if classes:
                print(
                    f"KF#{kf_id} [{run_id}] Ingested {len(classes)} embeddings "
                    f"(total: {insert_count}): {classes}"
                )

    # Open Zenoh session
    conf = zenoh.Config()
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

    session = zenoh.open(conf)
    sub = session.declare_subscriber(args.key, detection_callback)
    print(f"Subscribed to: {args.key}")
    print("Ingest worker running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        sub.undeclare()
        session.close()
        cur.close()
        conn.close()
        print(f"Shutdown. Total embeddings ingested: {insert_count}")


if __name__ == "__main__":
    main()
