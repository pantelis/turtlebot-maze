# Semantic Graph Builder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an incremental semantic graph from keyframe detections using online DBSCAN for place construction and CLIP-based landmark fusion, stored in Apache AGE.

**Architecture:** A `graph-builder` Zenoh subscriber writes graph nodes/edges to AGE as detections arrive. Online DBSCAN clusters keyframe poses into Places with merge support. Landmark fusion merges repeated observations into Object nodes using embedding similarity (primary) and spatial proximity (secondary). The `det_id` field joins pgvector embeddings to AGE graph nodes.

**Tech Stack:** Python 3.11, psycopg2 (AGE SQL via Cypher), eclipse-zenoh, numpy, docker-compose

**Spec:** `docs/superpowers/specs/2026-03-24-semantic-graph-builder-design.md`

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `detector/online_dbscan.py` | Create | Online DBSCAN with merge support |
| `detector/landmark_fusion.py` | Create | Object landmark fusion using embedding similarity |
| `detector/graph_builder.py` | Create | Main service: Zenoh sub, AGE writer, orchestration |
| `detector/test_online_dbscan.py` | Create | Unit tests for clustering |
| `detector/test_landmark_fusion.py` | Create | Unit tests for fusion |
| `detector/test_graph_builder.py` | Create | Integration tests against AGE |
| `detector/object_detector.py` | Modify | Add `run_id` and `det_id` to envelope |
| `detector/embedding_ingest.py` | Modify | Write `run_id` and `det_id` to pgvector |
| `vector-init.sql` | Modify | Add `run_id`, `det_id` columns |
| `docker-compose.yaml` | Modify | Add `graph-builder` service |
| `detector/requirements.txt` | Modify | Add `psycopg2-binary` (already present) |

---

### Task 1: Add `run_id` and `det_id` to detector envelope

The detector must emit identity fields so downstream services can join across databases.

**Files:**
- Modify: `detector/object_detector.py` (envelope section, ~line 367-380)

- [ ] **Step 1: Add `run_id` generation at startup**

In `main()`, after argument parsing, before model loading, generate a run_id:

```python
import socket
from datetime import datetime, timezone

run_id = f"{socket.gethostname()}-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')}"
print(f"Run ID: {run_id}")
```

Also add a CLI argument `--run-id` to allow override:

```python
parser.add_argument(
    "--run-id",
    type=str,
    default="",
    help="Run identifier (auto-generated if empty)",
)
```

If `args.run_id` is provided, use it instead of the auto-generated one.

- [ ] **Step 2: Add `det_id` per detection and `run_id` to envelope**

In the envelope construction (around line 367-380), change:

```python
# Before (current):
envelope = {
    "keyframe_id": keyframe_id,
    "timestamp": now,
    **pose_data,
    "detections": detections,
}

# After:
for idx, det in enumerate(detections):
    det["det_id"] = f"{run_id}_kf{keyframe_id}_d{idx}"

envelope = {
    "run_id": run_id,
    "keyframe_id": keyframe_id,
    "timestamp": now,
    **pose_data,
    "detections": detections,
}
```

- [ ] **Step 3: Verify detector starts and emits new fields**

```bash
docker compose up -d --force-recreate detector
sleep 15
docker logs turtlebot-maze-detector-1 2>&1 | grep "Run ID"
```

Expected: `Run ID: <hostname>-<timestamp>`

- [ ] **Step 4: Commit**

```bash
git add detector/object_detector.py
git commit -m "feat(detector): add run_id and det_id to detection envelope"
```

---

### Task 2: Update pgvector schema and ingest worker for identity fields

**Files:**
- Modify: `vector-init.sql`
- Modify: `detector/embedding_ingest.py` (~line 79-110)

- [ ] **Step 1: Add columns to vector-init.sql**

```sql
CREATE TABLE IF NOT EXISTS detection_embeddings (
  det_pk bigserial PRIMARY KEY,
  run_id text,
  det_id text UNIQUE,
  keyframe_id integer,
  class_name text NOT NULL,
  confidence real NOT NULL,
  bbox real[] NOT NULL,
  map_x real,
  map_y real,
  map_yaw real,
  embedding_model text,
  embedding vector(512),
  ingested_at timestamptz NOT NULL DEFAULT now()
);
```

- [ ] **Step 2: Update embedding_ingest.py to write new fields**

In `detection_callback`, extract `run_id` and `det_id` from the envelope:

```python
run_id = msg.get("run_id")
```

Per detection:

```python
det_id = det.get("det_id")
```

Update the INSERT statement:

```python
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
```

The `ON CONFLICT (det_id) DO NOTHING` provides idempotency.

- [ ] **Step 3: Recreate the table in running database**

```bash
docker exec pgvector-postgres psql -U postgres -d vectordb -c "
DROP TABLE IF EXISTS detection_embeddings;
CREATE TABLE detection_embeddings (
  det_pk bigserial PRIMARY KEY,
  run_id text,
  det_id text UNIQUE,
  keyframe_id integer,
  class_name text NOT NULL,
  confidence real NOT NULL,
  bbox real[] NOT NULL,
  map_x real,
  map_y real,
  map_yaw real,
  embedding_model text,
  embedding vector(512),
  ingested_at timestamptz NOT NULL DEFAULT now()
);"
```

- [ ] **Step 4: Restart ingest and verify**

```bash
docker compose restart embedding-ingest
sleep 5
docker logs turtlebot-maze-embedding-ingest-1 --tail 5
```

- [ ] **Step 5: Commit**

```bash
git add vector-init.sql detector/embedding_ingest.py
git commit -m "feat(ingest): add run_id and det_id to pgvector schema and ingest"
```

---

### Task 3: Implement online DBSCAN

**Files:**
- Create: `detector/online_dbscan.py`
- Create: `detector/test_online_dbscan.py`

- [ ] **Step 1: Write failing tests**

Create `detector/test_online_dbscan.py`:

```python
"""Tests for online DBSCAN place clustering."""
import pytest
from online_dbscan import OnlineDBSCAN


class TestSingleCluster:
    def test_three_close_points_form_cluster(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        db.add_point("r1", 1, 0.0, 0.0)
        db.add_point("r1", 2, 0.5, 0.0)
        db.add_point("r1", 3, 1.0, 0.0)
        # All three within epsilon, all core → one cluster
        labels = db.get_labels()
        place_ids = set(labels.values())
        assert -1 not in place_ids
        assert len(place_ids) == 1

    def test_two_points_are_noise(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        db.add_point("r1", 1, 0.0, 0.0)
        db.add_point("r1", 2, 0.5, 0.0)
        labels = db.get_labels()
        assert all(v == -1 for v in labels.values())


class TestMerge:
    def test_bridge_point_merges_two_clusters(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        # Cluster A
        db.add_point("r1", 1, 0.0, 0.0)
        db.add_point("r1", 2, 0.5, 0.0)
        db.add_point("r1", 3, 1.0, 0.0)
        # Cluster B (separate)
        db.add_point("r1", 4, 3.0, 0.0)
        db.add_point("r1", 5, 3.5, 0.0)
        db.add_point("r1", 6, 4.0, 0.0)
        labels_before = db.get_labels()
        place_ids_before = {v for v in labels_before.values() if v != -1}
        assert len(place_ids_before) == 2

        # Bridge point between clusters
        db.add_point("r1", 7, 2.0, 0.0)
        labels_after = db.get_labels()
        place_ids_after = {v for v in labels_after.values() if v != -1}
        assert len(place_ids_after) == 1  # merged


class TestFarPoints:
    def test_distant_points_separate_clusters(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        # Cluster A
        for i in range(3):
            db.add_point("r1", i, float(i) * 0.5, 0.0)
        # Cluster B far away
        for i in range(3, 6):
            db.add_point("r1", i, 100.0 + float(i) * 0.5, 0.0)
        labels = db.get_labels()
        place_ids = {v for v in labels.values() if v != -1}
        assert len(place_ids) == 2


class TestCentroid:
    def test_centroid_is_mean_of_members(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        db.add_point("r1", 1, 0.0, 0.0)
        db.add_point("r1", 2, 1.0, 0.0)
        db.add_point("r1", 3, 2.0, 0.0)
        labels = db.get_labels()
        place_id = next(v for v in labels.values() if v != -1)
        cx, cy = db.get_centroid(place_id)
        assert abs(cx - 1.0) < 0.01
        assert abs(cy - 0.0) < 0.01


class TestGetPlaceId:
    def test_returns_place_for_assigned_point(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        db.add_point("r1", 1, 0.0, 0.0)
        db.add_point("r1", 2, 0.5, 0.0)
        db.add_point("r1", 3, 1.0, 0.0)
        place = db.get_place_for_keyframe("r1", 2)
        assert place is not None and place != -1

    def test_returns_none_for_noise(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        db.add_point("r1", 1, 0.0, 0.0)
        place = db.get_place_for_keyframe("r1", 1)
        assert place is None or place == -1
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd detector && python -m pytest test_online_dbscan.py -v
```

Expected: `ModuleNotFoundError: No module named 'online_dbscan'`

- [ ] **Step 3: Implement `online_dbscan.py`**

Create `detector/online_dbscan.py`:

```python
"""Online DBSCAN for incremental place construction.

Supports adding points one at a time, with automatic cluster
creation, extension, and merge when a new point bridges two
existing clusters.
"""

import math
from dataclasses import dataclass, field


@dataclass
class PoseEntry:
    run_id: str
    keyframe_id: int
    x: float
    y: float


class OnlineDBSCAN:
    def __init__(self, epsilon: float = 1.5, min_samples: int = 3):
        self.epsilon = epsilon
        self.min_samples = min_samples

        self._poses: list[PoseEntry] = []
        self._neighbors: dict[int, set[int]] = {}
        self._labels: dict[int, str | int] = {}  # pose_idx -> place_id or -1
        self._core: set[int] = set()
        self._next_place_counter: int = 0
        self._run_id: str = ""

        # Reverse lookup: (run_id, keyframe_id) -> pose index
        self._kf_index: dict[tuple[str, int], int] = {}

        # Place membership: place_id -> set of pose indices
        self._place_members: dict[str, set[int]] = {}

    def _make_place_id(self) -> str:
        pid = f"{self._run_id}_p{self._next_place_counter}"
        self._next_place_counter += 1
        return pid

    def _distance(self, i: int, j: int) -> float:
        a, b = self._poses[i], self._poses[j]
        dx = a.x - b.x
        dy = a.y - b.y
        return math.sqrt(dx * dx + dy * dy)

    def add_point(self, run_id: str, keyframe_id: int, x: float, y: float) -> dict:
        """Add a point and return clustering result.

        Returns dict with keys:
            place_id: str or None (if noise)
            merged: list of place_ids that were absorbed (empty if no merge)
            new_place: bool (True if a new place was created)
            changed: set of pose indices whose labels changed
        """
        self._run_id = run_id
        i = len(self._poses)
        self._poses.append(PoseEntry(run_id, keyframe_id, x, y))
        self._kf_index[(run_id, keyframe_id)] = i

        # Build neighbor set (includes self)
        self._neighbors[i] = {i}
        for j in range(len(self._poses) - 1):
            if self._distance(i, j) <= self.epsilon:
                self._neighbors[i].add(j)
                self._neighbors[j].add(i)

        # Recompute core status for affected points
        affected = set(self._neighbors[i])
        old_core = self._core.copy()
        for idx in affected:
            if len(self._neighbors[idx]) >= self.min_samples:
                self._core.add(idx)
            else:
                self._core.discard(idx)

        # Determine cluster assignment
        core_neighbor_places = set()
        for j in self._neighbors[i]:
            if j in self._core and j != i:
                label = self._labels.get(j, -1)
                if label != -1:
                    core_neighbor_places.add(label)

        result = {"place_id": None, "merged": [], "new_place": False, "changed": set()}

        if i in self._core and not core_neighbor_places:
            # New cluster
            pid = self._make_place_id()
            self._labels[i] = pid
            self._place_members[pid] = {i}
            result["place_id"] = pid
            result["new_place"] = True
        elif len(core_neighbor_places) == 1:
            pid = next(iter(core_neighbor_places))
            self._labels[i] = pid
            self._place_members.setdefault(pid, set()).add(i)
            result["place_id"] = pid
        elif len(core_neighbor_places) > 1:
            # Merge: keep lowest place_id
            sorted_pids = sorted(core_neighbor_places)
            survivor = sorted_pids[0]
            absorbed = sorted_pids[1:]
            self._labels[i] = survivor
            self._place_members.setdefault(survivor, set()).add(i)
            for dead_pid in absorbed:
                for idx in self._place_members.get(dead_pid, set()):
                    self._labels[idx] = survivor
                    self._place_members[survivor].add(idx)
                    result["changed"].add(idx)
                del self._place_members[dead_pid]
            result["place_id"] = survivor
            result["merged"] = absorbed
        else:
            self._labels[i] = -1

        # Promote noise neighbors that are now border points
        for j in self._neighbors[i]:
            if j == i:
                continue
            if self._labels.get(j, -1) == -1:
                # Check if j is neighbor of any core point
                for k in self._neighbors[j]:
                    if k in self._core:
                        label_k = self._labels.get(k, -1)
                        if label_k != -1:
                            self._labels[j] = label_k
                            self._place_members.setdefault(label_k, set()).add(j)
                            result["changed"].add(j)
                            break

        # Also check if i was noise but now should be a border point
        if self._labels.get(i, -1) == -1 and i not in self._core:
            for j in self._neighbors[i]:
                if j in self._core:
                    label_j = self._labels.get(j, -1)
                    if label_j != -1:
                        self._labels[i] = label_j
                        self._place_members.setdefault(label_j, set()).add(i)
                        result["place_id"] = label_j
                        break

        return result

    def get_labels(self) -> dict[int, str | int]:
        """Return {pose_index: place_id} mapping. -1 means noise."""
        return dict(self._labels)

    def get_place_for_keyframe(self, run_id: str, keyframe_id: int) -> str | None:
        """Return place_id for a keyframe, or None if noise."""
        idx = self._kf_index.get((run_id, keyframe_id))
        if idx is None:
            return None
        label = self._labels.get(idx, -1)
        return None if label == -1 else label

    def get_centroid(self, place_id: str) -> tuple[float, float]:
        """Return (cx, cy) for a place."""
        members = self._place_members.get(place_id, set())
        if not members:
            return (0.0, 0.0)
        xs = [self._poses[i].x for i in members]
        ys = [self._poses[i].y for i in members]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def get_place_keyframe_count(self, place_id: str) -> int:
        return len(self._place_members.get(place_id, set()))

    def get_all_place_ids(self) -> set[str]:
        return set(self._place_members.keys())
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd detector && python -m pytest test_online_dbscan.py -v
```

Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
git add detector/online_dbscan.py detector/test_online_dbscan.py
git commit -m "feat: implement online DBSCAN with merge support"
```

---

### Task 4: Implement landmark fusion

**Files:**
- Create: `detector/landmark_fusion.py`
- Create: `detector/test_landmark_fusion.py`

- [ ] **Step 1: Write failing tests**

Create `detector/test_landmark_fusion.py`:

```python
"""Tests for object landmark fusion."""
import numpy as np
import pytest
from landmark_fusion import LandmarkFusion


def _rand_embedding(seed: int = 0) -> np.ndarray:
    rng = np.random.RandomState(seed)
    v = rng.randn(512).astype(np.float32)
    return v / np.linalg.norm(v)


class TestNewLandmark:
    def test_first_observation_creates_landmark(self):
        fuse = LandmarkFusion(spatial_threshold=3.0, similarity_threshold=0.7)
        emb = _rand_embedding(0)
        result = fuse.fuse("cup", 1.0, 2.0, emb, 100.0)
        assert result["is_new"]
        assert result["object_id"] is not None

    def test_different_class_creates_separate_landmark(self):
        fuse = LandmarkFusion(spatial_threshold=3.0, similarity_threshold=0.7)
        emb = _rand_embedding(0)
        r1 = fuse.fuse("cup", 1.0, 2.0, emb, 100.0)
        r2 = fuse.fuse("bottle", 1.0, 2.0, emb, 101.0)
        assert r1["object_id"] != r2["object_id"]


class TestMerge:
    def test_same_embedding_same_location_merges(self):
        fuse = LandmarkFusion(spatial_threshold=3.0, similarity_threshold=0.7)
        emb = _rand_embedding(0)
        r1 = fuse.fuse("cup", 1.0, 2.0, emb, 100.0)
        r2 = fuse.fuse("cup", 1.2, 2.1, emb, 101.0)
        assert not r2["is_new"]
        assert r2["object_id"] == r1["object_id"]
        assert r2["obs_count"] == 2

    def test_similar_embedding_far_location_no_merge(self):
        fuse = LandmarkFusion(spatial_threshold=3.0, similarity_threshold=0.7)
        emb = _rand_embedding(0)
        r1 = fuse.fuse("cup", 0.0, 0.0, emb, 100.0)
        r2 = fuse.fuse("cup", 100.0, 100.0, emb, 101.0)
        assert r2["is_new"]
        assert r2["object_id"] != r1["object_id"]

    def test_different_embedding_same_location_no_merge(self):
        fuse = LandmarkFusion(spatial_threshold=3.0, similarity_threshold=0.7)
        emb1 = _rand_embedding(0)
        emb2 = _rand_embedding(999)
        # Verify embeddings are dissimilar
        sim = np.dot(emb1, emb2)
        assert sim < 0.7
        r1 = fuse.fuse("cup", 1.0, 2.0, emb1, 100.0)
        r2 = fuse.fuse("cup", 1.0, 2.0, emb2, 101.0)
        assert r2["is_new"]


class TestGetLandmark:
    def test_get_returns_correct_state(self):
        fuse = LandmarkFusion(spatial_threshold=3.0, similarity_threshold=0.7)
        emb = _rand_embedding(0)
        r1 = fuse.fuse("cup", 1.0, 2.0, emb, 100.0)
        lm = fuse.get_landmark(r1["object_id"])
        assert lm is not None
        assert lm["class_name"] == "cup"
        assert lm["obs_count"] == 1
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd detector && python -m pytest test_landmark_fusion.py -v
```

Expected: `ModuleNotFoundError: No module named 'landmark_fusion'`

- [ ] **Step 3: Implement `landmark_fusion.py`**

Create `detector/landmark_fusion.py`:

```python
"""Object landmark fusion using CLIP embedding similarity.

Merges repeated observations of the same physical object into a single
landmark. Embedding cosine similarity is the primary discriminator;
spatial distance (robot pose) is a secondary filter.
"""

import math
import uuid

import numpy as np


class LandmarkFusion:
    def __init__(
        self,
        spatial_threshold: float = 3.0,
        similarity_threshold: float = 0.7,
    ):
        self.spatial_threshold = spatial_threshold
        self.similarity_threshold = similarity_threshold
        self._landmarks: dict[str, dict] = {}  # object_id -> landmark state

    def fuse(
        self,
        class_name: str,
        robot_x: float,
        robot_y: float,
        embedding: np.ndarray,
        timestamp: float,
    ) -> dict:
        """Fuse an observation into existing landmarks or create new.

        Returns dict with keys:
            object_id: str
            is_new: bool
            similarity: float (cosine sim to matched landmark, 0.0 if new)
            obs_count: int
        """
        best_id = None
        best_sim = -1.0

        for oid, lm in self._landmarks.items():
            if lm["class_name"] != class_name:
                continue
            sim = float(np.dot(embedding, lm["mean_embedding"]))
            if sim < self.similarity_threshold:
                continue
            dx = robot_x - lm["mean_x"]
            dy = robot_y - lm["mean_y"]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.spatial_threshold:
                continue
            if sim > best_sim:
                best_sim = sim
                best_id = oid

        if best_id is not None:
            lm = self._landmarks[best_id]
            n = lm["obs_count"]
            lm["mean_x"] = (lm["mean_x"] * n + robot_x) / (n + 1)
            lm["mean_y"] = (lm["mean_y"] * n + robot_y) / (n + 1)
            new_emb = (lm["mean_embedding"] * n + embedding) / (n + 1)
            norm = np.linalg.norm(new_emb)
            if norm > 0:
                new_emb = new_emb / norm
            lm["mean_embedding"] = new_emb
            lm["obs_count"] = n + 1
            lm["last_seen"] = timestamp
            return {
                "object_id": best_id,
                "is_new": False,
                "similarity": best_sim,
                "obs_count": lm["obs_count"],
            }

        oid = f"obj-{uuid.uuid4().hex[:8]}"
        self._landmarks[oid] = {
            "object_id": oid,
            "class_name": class_name,
            "mean_x": robot_x,
            "mean_y": robot_y,
            "mean_embedding": embedding.copy(),
            "obs_count": 1,
            "first_seen": timestamp,
            "last_seen": timestamp,
        }
        return {
            "object_id": oid,
            "is_new": True,
            "similarity": 0.0,
            "obs_count": 1,
        }

    def get_landmark(self, object_id: str) -> dict | None:
        lm = self._landmarks.get(object_id)
        if lm is None:
            return None
        return {
            "object_id": lm["object_id"],
            "class_name": lm["class_name"],
            "mean_x": lm["mean_x"],
            "mean_y": lm["mean_y"],
            "obs_count": lm["obs_count"],
            "first_seen": lm["first_seen"],
            "last_seen": lm["last_seen"],
        }

    def get_all_landmarks(self) -> list[dict]:
        return [self.get_landmark(oid) for oid in self._landmarks]
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd detector && python -m pytest test_landmark_fusion.py -v
```

Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
git add detector/landmark_fusion.py detector/test_landmark_fusion.py
git commit -m "feat: implement landmark fusion with embedding similarity"
```

---

### Task 5: Implement graph builder service

**Files:**
- Create: `detector/graph_builder.py`

- [ ] **Step 1: Create `graph_builder.py`**

This is the main service. It subscribes to `tb/detections`, uses `OnlineDBSCAN` and `LandmarkFusion`, and writes to AGE via psycopg2. Due to AGE's Cypher-via-SQL interface, all graph writes go through `SELECT * FROM cypher(...)`.

Create `detector/graph_builder.py` with:
- Argument parsing (Zenoh connect, AGE host/port/db, pgvector host/port/db, epsilon, min_samples, thresholds)
- AGE connection with `LOAD 'age'` and `SET search_path` per connection
- Helper function `cypher(cur, query, params=None)` that wraps `SELECT * FROM cypher('semantic_map', $$ ... $$)`
- `detection_callback` that:
  1. Parses the envelope
  2. Creates Run node (idempotent)
  3. Creates Keyframe + Pose nodes, HAS_KEYFRAME + HAS_POSE edges
  4. Calls `dbscan.add_point()` for place assignment
  5. Creates IN_PLACE edge, Place node (if new), handles merges
  6. For each detection: creates Observation, calls `fusion.fuse()`, creates OBSERVES + LOCATED_IN edges
  7. Updates place adjacency

The full implementation is ~250 lines. Key AGE/Cypher patterns:

```python
def _cypher(cur, query):
    """Execute a Cypher query against the semantic_map graph."""
    cur.execute(
        f"SELECT * FROM cypher('semantic_map', $${query}$$) AS (result agtype)"
    )
    return cur.fetchall()

def _create_run(cur, run_id, timestamp):
    _cypher(cur, f"""
        MERGE (r:Run {{run_id: '{run_id}'}})
        ON CREATE SET r.started_at = {timestamp}
        RETURN r
    """)
```

- [ ] **Step 2: Test manually against running AGE**

```bash
docker exec age-postgres psql -U postgres -d graphdb -c "
LOAD 'age';
SET search_path = ag_catalog, \"\$user\", public;
SELECT * FROM cypher('semantic_map', \$\$
  MATCH (n) RETURN count(n)
\$\$) AS (count agtype);
"
```

Expected: count = 0 (empty graph)

- [ ] **Step 3: Commit**

```bash
git add detector/graph_builder.py
git commit -m "feat: implement graph builder service with AGE integration"
```

---

### Task 6: Add graph-builder to docker-compose

**Files:**
- Modify: `docker-compose.yaml`

- [ ] **Step 1: Add service definition**

Add after the `embedding-ingest` service:

```yaml
  # Semantic graph builder (writes property graph to Apache AGE)
  graph-builder:
    build:
      context: .
      dockerfile: docker/Dockerfile.torch.gpu
    network_mode: host
    ipc: host
    environment:
      - PYTHONUNBUFFERED=1
    volumes:
      - ./detector:/app
    working_dir: /app
    command: >
      python graph_builder.py
      --connect tcp/localhost:7447
      --key tb/detections
      --age-host localhost
      --age-port 5435
      --age-db graphdb
      --epsilon 1.5
      --min-samples 3
      --spatial-threshold 3.0
      --similarity-threshold 0.7
    depends_on:
      - age
      - zenoh-router
```

- [ ] **Step 2: Bring up and verify**

```bash
docker compose up -d graph-builder
sleep 10
docker logs turtlebot-maze-graph-builder-1 --tail 10
```

Expected: Connection messages, "Subscribed to: tb/detections", "Graph builder running."

- [ ] **Step 3: Commit**

```bash
git add docker-compose.yaml
git commit -m "feat: add graph-builder service to docker-compose"
```

---

### Task 7: Integration test with live data

**Files:**
- Create: `detector/test_graph_builder.py`

- [ ] **Step 1: Write integration test**

Create `detector/test_graph_builder.py` that:
1. Connects to AGE
2. Sends synthetic keyframe envelopes directly to the graph builder's callback
3. Verifies nodes and edges exist in the graph

```python
"""Integration tests for graph builder against AGE."""
import pytest
import psycopg2
import numpy as np
import base64
from graph_builder import GraphBuilder


@pytest.fixture
def age_conn():
    conn = psycopg2.connect(
        host="localhost", port=5435, dbname="graphdb",
        user="postgres", password="postgres"
    )
    conn.autocommit = True
    cur = conn.cursor()
    cur.execute("LOAD 'age'")
    cur.execute('SET search_path = ag_catalog, "$user", public')
    # Clean graph
    try:
        cur.execute(
            "SELECT * FROM cypher('semantic_map', $$ MATCH (n) DETACH DELETE n $$) AS (r agtype)"
        )
    except Exception:
        pass
    yield conn
    conn.close()


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


def _make_detection(run_id, kf_id, idx, class_name, conf=0.8):
    emb = np.random.RandomState(hash(f"{run_id}_{kf_id}_{idx}") % 2**31).randn(512).astype(np.float32)
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


class TestGraphCreation:
    def test_single_keyframe_creates_nodes(self, age_conn):
        builder = GraphBuilder(age_conn=age_conn, epsilon=1.5, min_samples=3)
        det = _make_detection("test-run", 1, 0, "cup")
        env = _make_envelope("test-run", 1, 1.0, 2.0, 0.0, [det])
        builder.process_envelope(env)

        cur = age_conn.cursor()
        cur.execute(
            "SELECT * FROM cypher('semantic_map', $$ MATCH (k:Keyframe) RETURN count(k) $$) AS (c agtype)"
        )
        count = cur.fetchone()[0]
        assert str(count) == "1"
```

- [ ] **Step 2: Run integration test (requires AGE container running)**

```bash
cd detector && python -m pytest test_graph_builder.py -v
```

- [ ] **Step 3: Commit**

```bash
git add detector/test_graph_builder.py
git commit -m "test: add integration tests for graph builder"
```

---

### Task 8: End-to-end verification

- [ ] **Step 1: Bring up full stack**

```bash
docker compose up -d demo-world-enhanced zenoh-router zenoh-bridge \
  detector embedding-ingest graph-builder vector age
```

- [ ] **Step 2: Drive robot and verify graph population**

Move robot using ros-mcp-server (with 5s warmup), then check:

```bash
docker exec age-postgres psql -U postgres -d graphdb -c "
LOAD 'age';
SET search_path = ag_catalog, \"\$user\", public;
SELECT * FROM cypher('semantic_map', \$\$
  MATCH (r:Run)-[:HAS_KEYFRAME]->(k:Keyframe)-[:IN_PLACE]->(p:Place)
  RETURN r.run_id, count(k), count(DISTINCT p)
\$\$) AS (run agtype, keyframes agtype, places agtype);
"
```

Expected: Non-zero keyframes and places

- [ ] **Step 3: Verify pgvector has det_id**

```bash
docker exec pgvector-postgres psql -U postgres -d vectordb -c "
SELECT det_id, run_id, class_name, map_x FROM detection_embeddings LIMIT 3;
"
```

Expected: Rows with populated det_id and run_id

- [ ] **Step 4: Commit all remaining changes**

```bash
git add -A
git commit -m "feat: semantic graph builder end-to-end integration"
```
