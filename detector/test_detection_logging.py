#!/usr/bin/env python3
"""
Integration tests for zenoh_logger.py and query_detections.py.

Tests run entirely on the host (no Docker, no Zenoh router) using Zenoh
in peer mode.  A publisher thread sends fake payloads; the logger runs as a
subprocess and writes JSONL; the query script is exercised against both the
written file and hand-crafted fixtures.

Covers both use cases driven by the open issues:
  - Object detection (turtlebot-maze-es3): tb/detections
  - SLAM pose recording (turtlebot-maze-m3n): tb/slam/pose, tb/slam/status

Run with:
    pytest detector/test_detection_logging.py -v           # from project root
    python3 detector/test_detection_logging.py             # direct
"""

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
import zenoh

DETECTOR_DIR = Path(__file__).parent
LOGGER_SCRIPT = DETECTOR_DIR / "zenoh_logger.py"
QUERY_SCRIPT = DETECTOR_DIR / "query_detections.py"

# ---------------------------------------------------------------------------
# Sample payloads
# ---------------------------------------------------------------------------

DETECTION_PAYLOADS = [
    [{"class": "cup", "confidence": 0.92, "bbox": [10.0, 20.0, 80.0, 90.0]}],
    [
        {"class": "bottle", "confidence": 0.75, "bbox": [5.0, 5.0, 50.0, 60.0]},
        {"class": "cup", "confidence": 0.65, "bbox": [100.0, 100.0, 150.0, 180.0]},
    ],
    [],  # frame with no detections
    [{"class": "suitcase", "confidence": 0.88, "bbox": [200.0, 10.0, 300.0, 200.0]}],
]

# 4x4 pose matrix (row-major, translation in last column)
_POSE_MAT = [1, 0, 0, 0.1,  0, 1, 0, 0.2,  0, 0, 1, 0.05]
SLAM_POSE_PAYLOADS = [
    {"pose": _POSE_MAT, "frame": i, "tracking": "tracking"}
    for i in range(4)
]
SLAM_STATUS_PAYLOADS = [
    {"status": "tracking", "frame": i, "num_keyframes": i * 2}
    for i in range(3)
]

DETECTION_KEY = "tb/detections/test"
SLAM_POSE_KEY = "tb/slam/pose/test"
SLAM_STATUS_KEY = "tb/slam/status/test"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def publish(key: str, payloads: list, delay: float = 0.15):
    """Open a Zenoh peer session, publish each payload as JSON, then close."""
    conf = zenoh.Config()
    session = zenoh.open(conf)
    pub = session.declare_publisher(key)
    for payload in payloads:
        pub.put(json.dumps(payload).encode())
        time.sleep(delay)
    pub.undeclare()
    session.close()


def start_logger(output_path: Path, key: str) -> subprocess.Popen:
    return subprocess.Popen(
        [
            sys.executable,
            str(LOGGER_SCRIPT),
            "--key", key,
            "--output", str(output_path),
            "--flush-every", "1",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def stop_logger(proc: subprocess.Popen, timeout: float = 5.0):
    proc.send_signal(signal.SIGTERM)
    proc.wait(timeout=timeout)


def read_jsonl(path: Path) -> list:
    records = []
    if not path.exists():
        return records
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def run_query(*args) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(QUERY_SCRIPT), *args],
        capture_output=True,
        text=True,
    )


# ---------------------------------------------------------------------------
# Shared schema assertions
# ---------------------------------------------------------------------------


def assert_record_schema(record: dict, expected_key: str):
    """Every JSONL record must have ts, iso, key, payload fields."""
    assert "ts" in record, f"Missing 'ts': {record}"
    assert "iso" in record, f"Missing 'iso': {record}"
    assert "key" in record, f"Missing 'key': {record}"
    assert "payload" in record, f"Missing 'payload': {record}"
    assert isinstance(record["ts"], float), "'ts' must be float"
    assert isinstance(record["iso"], str), "'iso' must be str"
    assert record["key"] == expected_key, f"key mismatch: {record['key']} != {expected_key}"


# ---------------------------------------------------------------------------
# TestZenohLogger — generic logger behaviour (parametrized over both use cases)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "key,payloads",
    [
        (DETECTION_KEY, DETECTION_PAYLOADS),
        (SLAM_POSE_KEY, SLAM_POSE_PAYLOADS),
        (SLAM_STATUS_KEY, SLAM_STATUS_PAYLOADS),
    ],
    ids=["detections", "slam_pose", "slam_status"],
)
class TestZenohLogger:
    """End-to-end: logger subprocess receives Zenoh publications → JSONL."""

    def test_writes_correct_record_count(self, tmp_path, key, payloads):
        out = tmp_path / "out.jsonl"
        proc = start_logger(out, key)
        try:
            time.sleep(1.0)
            publish(key, payloads)
            time.sleep(0.5)
        finally:
            stop_logger(proc)

        records = read_jsonl(out)
        assert len(records) == len(payloads), (
            f"Expected {len(payloads)} records, got {len(records)}\n"
            f"stdout: {proc.stdout.read().decode()}\n"
            f"stderr: {proc.stderr.read().decode()}"
        )

    def test_record_schema(self, tmp_path, key, payloads):
        out = tmp_path / "out.jsonl"
        proc = start_logger(out, key)
        try:
            time.sleep(1.0)
            publish(key, payloads[:2])
            time.sleep(0.5)
        finally:
            stop_logger(proc)

        records = read_jsonl(out)
        assert len(records) >= 1, "No records written"
        for rec in records:
            assert_record_schema(rec, key)

    def test_timestamps_monotonic(self, tmp_path, key, payloads):
        out = tmp_path / "out.jsonl"
        proc = start_logger(out, key)
        try:
            time.sleep(1.0)
            publish(key, payloads, delay=0.2)
            time.sleep(0.5)
        finally:
            stop_logger(proc)

        records = read_jsonl(out)
        timestamps = [r["ts"] for r in records]
        assert timestamps == sorted(timestamps), "Timestamps not monotonically increasing"

    def test_payload_roundtrip(self, tmp_path, key, payloads):
        """Payload must survive the JSON → JSONL → JSON roundtrip intact."""
        out = tmp_path / "out.jsonl"
        proc = start_logger(out, key)
        try:
            time.sleep(1.0)
            publish(key, [payloads[0]])
            time.sleep(0.5)
        finally:
            stop_logger(proc)

        records = read_jsonl(out)
        assert len(records) >= 1
        assert records[0]["payload"] == payloads[0]


# ---------------------------------------------------------------------------
# TestDetectionPayload — detection-specific field validation
# ---------------------------------------------------------------------------


class TestDetectionPayload:
    def test_detection_fields_present(self, tmp_path):
        out = tmp_path / "out.jsonl"
        proc = start_logger(out, DETECTION_KEY)
        try:
            time.sleep(1.0)
            publish(DETECTION_KEY, [DETECTION_PAYLOADS[0]])
            time.sleep(0.5)
        finally:
            stop_logger(proc)

        records = read_jsonl(out)
        assert len(records) >= 1
        det = records[0]["payload"][0]
        assert det["class"] == "cup"
        assert 0.0 <= det["confidence"] <= 1.0
        assert len(det["bbox"]) == 4


# ---------------------------------------------------------------------------
# TestSlamPayload — SLAM-specific field validation
# ---------------------------------------------------------------------------


class TestSlamPayload:
    def test_pose_matrix_shape(self, tmp_path):
        out = tmp_path / "out.jsonl"
        proc = start_logger(out, SLAM_POSE_KEY)
        try:
            time.sleep(1.0)
            publish(SLAM_POSE_KEY, [SLAM_POSE_PAYLOADS[0]])
            time.sleep(0.5)
        finally:
            stop_logger(proc)

        records = read_jsonl(out)
        assert len(records) >= 1
        payload = records[0]["payload"]
        assert "pose" in payload, f"No 'pose' in SLAM record: {payload}"
        assert len(payload["pose"]) == 12, "Pose matrix must have 12 elements (3x4)"

    def test_status_fields(self, tmp_path):
        out = tmp_path / "out.jsonl"
        proc = start_logger(out, SLAM_STATUS_KEY)
        try:
            time.sleep(1.0)
            publish(SLAM_STATUS_KEY, [SLAM_STATUS_PAYLOADS[0]])
            time.sleep(0.5)
        finally:
            stop_logger(proc)

        records = read_jsonl(out)
        assert len(records) >= 1
        payload = records[0]["payload"]
        assert "status" in payload, f"No 'status' field in SLAM status record: {payload}"


# ---------------------------------------------------------------------------
# TestQueryDetections — query_detections.py with a pre-built JSONL fixture
# ---------------------------------------------------------------------------

# Fixture uses the new schema: ts, iso, key, payload.
# ts values are derived from the ISO strings so time-range filters work correctly.
from datetime import datetime, timezone as _tz

_ISO_BASE = "2026-02-20T10:00:00Z"
_BASE_TS = datetime.fromisoformat(_ISO_BASE.replace("Z", "+00:00")).timestamp()


@pytest.fixture
def detection_fixture(tmp_path) -> Path:
    path = tmp_path / "detections.jsonl"
    with open(path, "w") as f:
        for i, dets in enumerate(DETECTION_PAYLOADS):
            ts = _BASE_TS + i * 10
            iso = f"2026-02-20T10:00:{i * 10:02d}.000Z"
            record = {
                "ts": ts,
                "iso": iso,
                "key": "tb/detections",
                "payload": dets,
            }
            f.write(json.dumps(record) + "\n")
    return path


class TestQueryDetections:
    def test_json_output(self, detection_fixture):
        r = run_query("--source", "jsonl", "--input", str(detection_fixture), "--format", "json")
        assert r.returncode == 0, r.stderr
        data = json.loads(r.stdout)
        assert len(data) == len(DETECTION_PAYLOADS)

    def test_jsonl_output(self, detection_fixture):
        r = run_query("--source", "jsonl", "--input", str(detection_fixture), "--format", "jsonl")
        assert r.returncode == 0, r.stderr
        lines = [l for l in r.stdout.strip().splitlines() if l]
        assert len(lines) == len(DETECTION_PAYLOADS)
        for line in lines:
            json.loads(line)

    def test_csv_output(self, detection_fixture):
        r = run_query("--source", "jsonl", "--input", str(detection_fixture), "--format", "csv")
        assert r.returncode == 0, r.stderr
        lines = r.stdout.strip().splitlines()
        assert lines[0].startswith("timestamp,iso,class")
        expected_rows = sum(len(d) for d in DETECTION_PAYLOADS)
        assert len(lines) - 1 == expected_rows

    def test_summary_output(self, detection_fixture):
        r = run_query("--source", "jsonl", "--input", str(detection_fixture), "--format", "summary")
        assert r.returncode == 0, r.stderr
        assert "cup" in r.stdout
        assert "bottle" in r.stdout
        assert "suitcase" in r.stdout

    def test_time_filter_after(self, detection_fixture):
        r = run_query(
            "--source", "jsonl", "--input", str(detection_fixture),
            "--after", "2026-02-20T10:00:15", "--format", "json",
        )
        assert r.returncode == 0, r.stderr
        # Records at +0s and +10s are before T10:00:15; +20s and +30s are after
        assert len(json.loads(r.stdout)) == 2

    def test_time_filter_before(self, detection_fixture):
        r = run_query(
            "--source", "jsonl", "--input", str(detection_fixture),
            "--before", "2026-02-20T10:00:15", "--format", "json",
        )
        assert r.returncode == 0, r.stderr
        assert len(json.loads(r.stdout)) == 2

    def test_missing_input_file(self):
        r = run_query("--source", "jsonl", "--input", "/nonexistent/detections.jsonl", "--format", "json")
        assert r.returncode != 0


# ---------------------------------------------------------------------------
# TestZenohStorageConfig — validate the JSON5 config file
# ---------------------------------------------------------------------------


class TestZenohStorageConfig:
    def test_config_file_exists(self):
        config = DETECTOR_DIR.parent / "zenoh" / "zenoh-storage.json5"
        assert config.exists(), f"Config file not found: {config}"

    def test_config_has_storage_manager_section(self):
        """Config must declare the storage_manager plugin (not just reference keys)."""
        config = DETECTOR_DIR.parent / "zenoh" / "zenoh-storage.json5"
        text = config.read_text()
        assert "storage_manager" in text
        assert "volumes" in text
        assert "storages" in text
        assert "memory" in text

    def test_config_has_detection_storage(self):
        config = DETECTOR_DIR.parent / "zenoh" / "zenoh-storage.json5"
        text = config.read_text()
        assert "tb/detections" in text

    def test_config_has_slam_storages(self):
        config = DETECTOR_DIR.parent / "zenoh" / "zenoh-storage.json5"
        text = config.read_text()
        assert "tb/slam/pose" in text
        assert "tb/slam/status" in text


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
