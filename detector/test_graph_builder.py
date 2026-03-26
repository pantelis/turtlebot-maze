"""Integration tests for GraphBuilder.

These tests require a live Apache AGE database at localhost:5435.
Run the container with:

    docker compose up -d age

then execute:

    cd detector && python -m pytest test_graph_builder.py -v -p no:dash
"""

from __future__ import annotations

import base64
import sys
from pathlib import Path

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Ensure the project root is on sys.path so local packages (zenoh, etc.) are
# importable when pytest is invoked from inside the detector/ sub-directory.
# ---------------------------------------------------------------------------
_PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

try:
    import psycopg2

    _PSYCOPG2_OK = True
except ImportError:
    _PSYCOPG2_OK = False

try:
    from graph_builder import GraphBuilder  # noqa: E402

    _GRAPH_BUILDER_OK = True
except ImportError as _e:
    _GRAPH_BUILDER_OK = False
    _GRAPH_BUILDER_IMPORT_ERR = str(_e)

# ---------------------------------------------------------------------------
# Connection probe — used by skipif
# ---------------------------------------------------------------------------

_AGE_PARAMS = dict(
    host="localhost",
    port=5435,
    dbname="graphdb",
    user="postgres",
    password="postgres",
)


def _age_available() -> bool:
    if not _PSYCOPG2_OK:
        return False
    try:
        conn = psycopg2.connect(connect_timeout=3, **_AGE_PARAMS)
        conn.close()
        return True
    except Exception:
        return False


_AGE_UNAVAILABLE = not _age_available()
_SKIP_REASON = (
    "AGE database not available at localhost:5435 "
    "(start with: docker compose up -d age)"
    if _PSYCOPG2_OK
    else "psycopg2 not installed"
)

if not _GRAPH_BUILDER_OK:
    _AGE_UNAVAILABLE = True
    _SKIP_REASON = f"graph_builder import failed: {_GRAPH_BUILDER_IMPORT_ERR}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_detection(run_id, kf_id, idx, class_name, conf=0.8, seed=None):
    """Create a synthetic detection dict with random embedding."""
    if seed is None:
        seed = hash(f"{run_id}_{kf_id}_{idx}") % (2**31)
    rng = np.random.RandomState(seed)
    emb = rng.randn(512).astype(np.float32)
    emb = emb / np.linalg.norm(emb)
    return {
        "det_id": f"{run_id}_kf{kf_id}_d{idx}",
        "class": class_name,
        "confidence": conf,
        "bbox": [10.0, 20.0, 100.0, 200.0],
        "embedding": base64.b64encode(emb.tobytes()).decode(),
        "embedding_dim": 512,
        "embedding_model": "test",
    }


def _make_envelope(run_id, kf_id, x, y, yaw, detections):
    return {
        "run_id": run_id,
        "keyframe_id": kf_id,
        "timestamp": 1000.0 + kf_id,
        "map_x": x,
        "map_y": y,
        "map_yaw": yaw,
        "detections": detections,
    }


def _cypher_count(cur, label):
    """Count nodes with a given label."""
    cur.execute(
        f"SELECT * FROM cypher('semantic_map', $$ MATCH (n:{label}) RETURN count(n) $$) AS (c agtype)"
    )
    row = cur.fetchone()
    return int(str(row[0]))


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def age_conn():
    """Provide a live AGE connection, cleaning the graph before each test."""
    conn = psycopg2.connect(**_AGE_PARAMS)
    conn.autocommit = True
    cur = conn.cursor()
    cur.execute("LOAD 'age'")
    cur.execute('SET search_path = ag_catalog, "$user", public')

    # Ensure the graph exists
    cur.execute(
        "SELECT count(*) FROM ag_catalog.ag_graph WHERE name = 'semantic_map'"
    )
    if cur.fetchone()[0] == 0:
        cur.execute("SELECT create_graph('semantic_map')")

    # Clean before test
    cur.execute(
        "SELECT * FROM cypher('semantic_map', $$ MATCH (n) DETACH DELETE n $$) AS (result agtype)"
    )
    cur.close()

    yield conn

    conn.close()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.skipif(_AGE_UNAVAILABLE, reason=_SKIP_REASON)
def test_single_keyframe_creates_run_and_keyframe(age_conn):
    """Processing one envelope must create exactly 1 Run and 1 Keyframe node."""
    builder = GraphBuilder(age_conn, epsilon=1.5, min_samples=3)
    envelope = _make_envelope("run_001", 0, 0.0, 0.0, 0.0, [])
    builder.process_envelope(envelope)

    cur = age_conn.cursor()
    assert _cypher_count(cur, "Run") == 1, "Expected 1 Run node"
    assert _cypher_count(cur, "Keyframe") == 1, "Expected 1 Keyframe node"
    cur.close()


@pytest.mark.skipif(_AGE_UNAVAILABLE, reason=_SKIP_REASON)
def test_observation_created_for_detection(age_conn):
    """One envelope with one detection must produce exactly 1 Observation node."""
    builder = GraphBuilder(age_conn, epsilon=1.5, min_samples=3)
    det = _make_detection("run_002", 0, 0, "chair")
    envelope = _make_envelope("run_002", 0, 0.0, 0.0, 0.0, [det])
    builder.process_envelope(envelope)

    cur = age_conn.cursor()
    assert _cypher_count(cur, "Observation") == 1, "Expected 1 Observation node"
    cur.close()


@pytest.mark.skipif(_AGE_UNAVAILABLE, reason=_SKIP_REASON)
def test_three_keyframes_form_place(age_conn):
    """Three keyframes within epsilon distance must collapse into 1 Place.

    With min_samples=3 and epsilon=1.5, three points at (0,0), (0.5,0), (1.0,0)
    are all within 1.5 m of each other and together satisfy the core-point
    condition, so DBSCAN must assign them to a single cluster / Place.
    """
    builder = GraphBuilder(age_conn, epsilon=1.5, min_samples=3)
    positions = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
    for kf_id, (x, y) in enumerate(positions):
        envelope = _make_envelope("run_003", kf_id, x, y, 0.0, [])
        builder.process_envelope(envelope)

    cur = age_conn.cursor()
    assert _cypher_count(cur, "Place") == 1, "Expected exactly 1 Place node"
    cur.close()


@pytest.mark.skipif(_AGE_UNAVAILABLE, reason=_SKIP_REASON)
def test_idempotent_replay(age_conn):
    """Processing the same envelope twice must not duplicate Keyframe nodes."""
    builder = GraphBuilder(age_conn, epsilon=1.5, min_samples=3)
    envelope = _make_envelope("run_004", 0, 0.0, 0.0, 0.0, [])

    builder.process_envelope(envelope)
    builder.process_envelope(envelope)  # replay — should be skipped

    cur = age_conn.cursor()
    assert _cypher_count(cur, "Keyframe") == 1, "Idempotent replay created duplicate Keyframe"
    cur.close()
