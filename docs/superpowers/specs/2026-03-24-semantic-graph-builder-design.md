# Semantic Graph Builder Design

## Overview

A new `graph-builder` service that subscribes to `tb/detections` on Zenoh and incrementally constructs a semantic property graph in Apache AGE. The graph links Runs, Keyframes, Poses, Observations, Objects (landmarks), and Places. Place construction uses online DBSCAN to handle cluster merges (required for house environments with multiple entry points per room). Object landmark fusion merges repeated observations of the same physical object using spatial proximity and CLIP embedding similarity.

## Context

The detection pipeline already produces keyframe-gated detections with CLIP embeddings and map-frame poses, published as JSON envelopes to `tb/detections`. The `embedding-ingest` service writes these to pgvector. This design adds a parallel service that builds the graph representation required for semantic re-localization (assignment-3).

### Current infrastructure

| Service | Database | Port | Contains |
|---|---|---|---|
| `vector` (pgvector) | `vectordb` | 5436 | `detection_embeddings` table with CLIP vectors and poses |
| `age` (Apache AGE) | `graphdb` | 5435 | `semantic_map` graph (empty) |

### Data flowing through `tb/detections`

```json
{
  "run_id": "run-a-20260324-1300",
  "keyframe_id": 42,
  "timestamp": 1711281234.567,
  "map_x": 2.14,
  "map_y": 0.83,
  "map_yaw": 0.785,
  "detections": [
    {
      "det_id": "run-a-20260324-1300_kf42_d0",
      "class": "cup",
      "confidence": 0.87,
      "bbox": [120.0, 80.0, 250.0, 310.0],
      "embedding": "<base64 512 x float32>",
      "embedding_dim": 512,
      "embedding_model": "ViT-B-32/laion2b_s34b_b79k"
    }
  ]
}
```

### Identity scheme

Every entity needs a globally unique, stable identifier that survives restarts and is shared across pgvector and AGE:

| Entity | ID format | Example |
|---|---|---|
| Run | `run_id` (user-supplied or auto-generated at detector start) | `run-a-20260324-1300` |
| Keyframe | `(run_id, keyframe_id)` | `("run-a-20260324-1300", 42)` |
| Observation | `det_id` = `{run_id}_kf{keyframe_id}_d{detection_idx}` | `run-a-20260324-1300_kf42_d0` |
| Object | `object_id` (auto-generated UUID at creation) | `obj-a1b2c3` |
| Place | `place_id` (auto-generated, prefixed with run_id) | `run-a-20260324-1300_p3` |

The `det_id` is the join key between pgvector and AGE. Both `embedding-ingest` and `graph-builder` receive the same `det_id` from the Zenoh payload. pgvector stores it as a column; AGE stores it as an Observation node property. The re-localization query joins on `det_id`.

The detector must include `run_id` in the envelope (generated once at startup, e.g. from hostname + ISO timestamp) and `det_id` per detection (composed from `run_id`, `keyframe_id`, and detection index within the keyframe).

Keyframe IDs are scoped to a run: `(run_id, keyframe_id)` is unique, but `keyframe_id` alone may repeat across runs or detector restarts.

## Architecture

```
tb/detections (Zenoh)
       |
       v
 graph-builder.py
       |
       +---> AGE (graphdb:5435)     -- graph nodes and edges
       +---> pgvector (vectordb:5436) -- read embeddings for landmark fusion
```

The graph-builder is a standalone Docker service, separate from embedding-ingest. Each service has one primary write target and a single clear purpose:
- `embedding-ingest` writes vectors to pgvector (including `det_id` as a column for join queries)
- `graph-builder` writes graph structure to AGE (including `det_id` on Observation nodes)

Both subscribe to `tb/detections` independently. The duplicated Zenoh subscription is trivial overhead.

### pgvector schema change

The `detection_embeddings` table must add `det_id` and `run_id` columns:

```sql
ALTER TABLE detection_embeddings
  ADD COLUMN run_id text,
  ADD COLUMN det_id text UNIQUE;
```

The `embedding-ingest` worker must be updated to write these fields from the Zenoh payload. The `UNIQUE` constraint on `det_id` provides idempotency (duplicate inserts are rejected).

## Graph Schema

### Node types

| Node | Properties |
|---|---|
| Run | `run_id`, `started_at` |
| Keyframe | `run_id`, `keyframe_id`, `timestamp` |
| Pose | `map_x`, `map_y`, `map_yaw`, `run_id`, `keyframe_id` |
| Observation | `det_id`, `class_name`, `confidence`, `bbox`, `run_id`, `keyframe_id` |
| Object | `object_id`, `class_name`, `mean_x`, `mean_y`, `mean_embedding_b64`, `obs_count`, `first_seen`, `last_seen` |
| Place | `place_id`, `centroid_x`, `centroid_y`, `keyframe_count` |

`det_id` on Observation is the shared join key with the `detection_embeddings` table in pgvector. Both services receive it from the same Zenoh payload.

### Edge types

| Edge | From | To | Properties |
|---|---|---|---|
| HAS_KEYFRAME | Run | Keyframe | |
| HAS_POSE | Keyframe | Pose | |
| HAS_OBSERVATION | Keyframe | Observation | |
| OBSERVES | Observation | Object | `similarity` (cosine to landmark mean) |
| LOCATED_IN | Object | Place | |
| IN_PLACE | Keyframe | Place | |
| ADJACENT_TO | Place | Place | `shared_transitions` (count of consecutive keyframes across the two places) |

The `IN_PLACE` edge from Keyframe to Place is part of the schema (not an implementation afterthought). It records which place each keyframe was assigned to by the online DBSCAN clustering.

## Online DBSCAN for Place Construction

### Why online DBSCAN

The robot operates in a house environment where the same room is entered from multiple doorways. Leader clustering (assign to nearest centroid within radius) would create separate places for each entry path that should eventually merge into one room. Online DBSCAN handles this by merging clusters when a new point bridges two existing groups.

### Parameters

| Parameter | Value | Rationale |
|---|---|---|
| `epsilon` | 1.5 m | Two poses within 1.5 m can belong to the same place; roughly room-scale |
| `min_samples` | 3 | A place needs at least 3 keyframes to form; fewer are noise until more arrive |

### Data structures (in memory)

- `poses`: ordered list of `(run_id, keyframe_id, x, y)` for all keyframes seen
- `neighbors`: dict mapping each pose index to its set of neighbor indices within epsilon (includes self)
- `labels`: dict mapping each pose index to a `place_id` (-1 for noise)
- `core_points`: set of pose indices that have >= min_samples neighbors (including self)
- `next_place_id`: monotonic counter, prefixed with run_id

### Algorithm (per new keyframe)

1. Append `(run_id, keyframe_id, x, y)` to poses list, get index `i`
2. Compute Euclidean distance from pose `i` to all existing poses; build `neighbors[i]` = set of indices within epsilon, **including `i` itself** (distance 0 is within epsilon)
3. For each existing pose `j` in `neighbors[i]` where `j != i`, add `i` to `neighbors[j]`
4. Recompute core status for `i` and all affected `j`: core if `len(neighbors[idx]) >= min_samples`. Following standard DBSCAN, the point itself is counted in its neighbor set, so with `min_samples=3` a point needs 2 other points within epsilon to become core.
5. Determine cluster assignment:
   - Collect the set of distinct `place_id` values among core neighbors of `i` (excluding -1)
   - If empty and `i` is core: create new place, assign `next_place_id`, increment counter
   - If empty and `i` is not core: assign -1 (noise)
   - If exactly one place_id: assign `i` to that place
   - If multiple place_ids: **merge** — pick the lowest place_id as survivor, reassign all points in absorbed clusters to the survivor
6. Check if any previously-noise neighbors of `i` have become border points (neighbor of a core point) and assign them to the appropriate cluster
7. Update AGE graph:
   - Create or update Place node with recalculated centroid (mean of all member poses)
   - On merge: delete absorbed Place node, redirect all LOCATED_IN edges to survivor, recalculate ADJACENT_TO edges
   - Create Keyframe→Place relationship (not in the original schema but needed for place assignment tracking; alternatively infer from Keyframe→Pose + Pose position)

### Centroid maintenance

Place centroids are running averages of member pose positions:

```
centroid_x = mean(x for all poses in cluster)
centroid_y = mean(y for all poses in cluster)
```

Updated incrementally: when adding a point, `new_centroid = old_centroid + (new_point - old_centroid) / new_count`. On merge, recompute from both sets.

## Object Landmark Fusion

### Purpose

The same physical cup is detected in multiple keyframes as the robot passes it. Landmark fusion merges these observations into a single Object node.

### Observation position

The Zenoh payload includes the robot's map-frame pose `(map_x, map_y, map_yaw)` but not the object's world position. Without depth data or raycasting, we cannot compute the object's true map coordinates from a 2D bounding box alone.

The observation position used for fusion is therefore the **robot pose at the time of observation**, not the object's position. This means multiple distinct objects seen from the same keyframe share the same spatial coordinates. Spatial distance alone cannot distinguish them. The fusion algorithm relies on **embedding similarity as the primary discriminator** and spatial distance as a secondary filter to avoid merging visually similar objects that are far apart in the environment.

The landmark's `mean_x`, `mean_y` converges toward the centroid of the robot poses from which the object was observed, not the object's actual position. This is an approximation that works when the robot observes an object from a consistent region (e.g. always from one side of a room). For more precise object localization, depth integration would be needed (out of scope for this design).

### Parameters

| Parameter | Value | Rationale |
|---|---|---|
| `spatial_threshold` | 3.0 m | Maximum distance between observation robot pose and landmark mean robot pose. Wider than originally specified because this is robot-to-robot distance, not object-to-object. |
| `similarity_threshold` | 0.7 | Minimum cosine similarity between observation embedding and landmark mean embedding. This is the primary merge criterion. |

### Data structures (in memory)

Per-landmark index entry:
- `object_id`: unique identifier (UUID)
- `class_name`: COCO class label
- `mean_x`, `mean_y`: running average of observer robot poses
- `mean_embedding`: running average of L2-normalized embeddings (re-normalized after each update)
- `obs_count`: number of merged observations
- `first_seen`, `last_seen`: timestamps

### Algorithm (per observation)

1. Filter landmarks to those matching `class_name`
2. For each candidate landmark:
   - Compute cosine similarity: `sim = dot(obs_embedding, mean_embedding)` (both L2-normalized)
   - If `sim < similarity_threshold`, skip (primary filter)
   - Compute Euclidean distance: `dist = sqrt((robot_x - mean_x)^2 + (robot_y - mean_y)^2)`
   - If `dist > spatial_threshold`, skip (secondary filter: same-looking objects in different rooms)
3. Among candidates passing both filters, pick highest similarity
4. If match found: merge
   - Update `mean_x`, `mean_y` with running average of robot poses
   - Update `mean_embedding`: `new_mean = normalize((old_mean * old_count + obs_embedding) / (old_count + 1))`
   - Increment `obs_count`, update `last_seen`
   - Create OBSERVES edge from Observation to existing Object node
5. If no match: create new Object node
   - Create OBSERVES edge from Observation to new Object node
   - Determine which Place this Object belongs to (based on observation's keyframe place assignment)
   - Create LOCATED_IN edge from Object to Place

### Object-Place assignment

An Object is LOCATED_IN the Place where the majority of its observations were made. When a new observation is merged into an existing landmark, check if the observation's place differs from the Object's current place. If the majority shifts, update the LOCATED_IN edge.

## Place Adjacency

Two places are adjacent if the robot transitioned between them during the run. Detected by:

- The graph-builder maintains `last_keyframe_place` in memory (the place_id of the most recently processed keyframe within the current run)
- When a new keyframe is assigned to a place that differs from `last_keyframe_place`, an ADJACENT_TO edge is created (or its `shared_transitions` count is incremented) between the two places
- This is purely sequential within a single run. Keyframe IDs are scoped to `(run_id, keyframe_id)`, so adjacency is never computed across runs

Adjacency is recalculated when:
- A new keyframe is assigned to a place that differs from `last_keyframe_place`
- A cluster merge happens: the absorbed place's ADJACENT_TO edges are transferred to the survivor; if both the survivor and absorbed place had edges to the same third place, the edges are merged (sum of `shared_transitions`)

## Docker Service

New entry in `docker-compose.yaml`:

```yaml
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
    --pg-host localhost
    --pg-port 5436
    --pg-db vectordb
    --epsilon 1.5
    --min-samples 3
    --spatial-threshold 1.0
    --similarity-threshold 0.7
  depends_on:
    - age
    - vector
    - zenoh-router
```

## File Structure

```
detector/
  graph_builder.py       # Main service: Zenoh subscriber + AGE writer
  online_dbscan.py       # Online DBSCAN implementation
  landmark_fusion.py     # Object landmark fusion logic
  embedding_ingest.py    # Existing: pgvector writer (unchanged)
  object_detector.py     # Existing: YOLO + CLIP detector (unchanged)
```

## Re-localization Query (downstream consumer)

The graph enables the re-localization query from assignment-3:

1. Robot in Run B captures crops, computes CLIP embeddings
2. KNN search in pgvector finds top-k similar stored embeddings, returning their `det_id` values and cosine distances
3. For each `det_id`, query the AGE graph: match the Observation node with that `det_id`, traverse OBSERVES edge to its Object node, then LOCATED_IN edge to its Place node
4. Aggregate similarity scores per Place, rank top-3
5. Output pose hypothesis from the winning Place's centroid

The join key is `det_id`, which is present in both the pgvector `detection_embeddings.det_id` column and the AGE Observation node's `det_id` property. Both are populated from the same Zenoh payload field.

```sql
-- Step 2: KNN in pgvector
SELECT det_id, embedding <=> $query_vec AS distance
FROM detection_embeddings
ORDER BY distance LIMIT 10;

-- Step 3: Graph traversal in AGE (per det_id from step 2)
SELECT * FROM cypher('semantic_map', $$
  MATCH (obs:Observation {det_id: $det_id})-[:OBSERVES]->(obj:Object)-[:LOCATED_IN]->(p:Place)
  RETURN p.place_id, p.centroid_x, p.centroid_y, obj.class_name
$$) AS (place_id agtype, cx agtype, cy agtype, class agtype);
```

This query is not part of the graph-builder service. It is a separate script that reads from both databases.

## Bootstrap and Idempotency

All in-memory state (DBSCAN poses/neighbors/labels, landmark index) is ephemeral. On container restart, the graph-builder must reconstruct its state from the AGE graph before processing new messages.

### Startup bootstrap sequence

1. Connect to AGE and pgvector
2. Query all existing Keyframe nodes with their Pose properties for the current run, ordered by keyframe_id
3. Rebuild the DBSCAN state by replaying poses in order (same algorithm as live processing, but without writing to AGE since the graph already has the data)
4. Query all existing Object nodes with their properties; rebuild the landmark index
5. Query all Place nodes with centroid and member counts; verify consistency with rebuilt DBSCAN state
6. Record the highest `(run_id, keyframe_id)` seen — only process Zenoh messages with higher keyframe_ids

### Idempotency

Before creating any node, check if a node with the same identity key already exists:
- Keyframe: `(run_id, keyframe_id)`
- Observation: `det_id`
- Object: `object_id`
- Place: `place_id`

If the node exists, skip creation. This makes the service safe to restart at any point — it replays its state from AGE, then resumes from where it left off.

### Run scoping

In-memory state is scoped to a single `run_id`. If the graph-builder receives a message with a different `run_id` than the current one, it flushes in-memory state and bootstraps from the new run's existing graph data (if any).

## Error Handling

- If AGE is unreachable at startup, retry with backoff (graph-builder depends_on age)
- If a keyframe arrives with no pose data (map_x/map_y null), skip place assignment and landmark fusion for that keyframe's observations; still create Keyframe, Pose (with nulls), and Observation nodes
- If CLIP embedding is missing from a detection, create Observation and skip landmark fusion (no embedding to compare)
- On cluster merge, use a database transaction to atomically update all affected labels and edges
- On duplicate detection (det_id already exists in AGE), skip silently (idempotent)

## Testing Strategy

- Unit tests for `online_dbscan.py`: known point configurations, verify cluster assignments and merges
- Unit tests for `landmark_fusion.py`: same-object merge, different-object separation, edge cases (identical position different class)
- Integration test: feed a sequence of synthetic keyframe envelopes, verify graph structure in AGE matches expected nodes and edges
