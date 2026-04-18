"""Graph builder service: Zenoh subscriber -> Apache AGE semantic property graph.

Listens to ``tb/detections`` envelopes, clusters keyframes into places via
OnlineDBSCAN, fuses object detections via LandmarkFusion, and writes the
resulting semantic map into an Apache AGE (PostgreSQL + Cypher) database.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import struct
import time

import numpy as np
import psycopg2
import zenoh

from landmark_fusion import LandmarkFusion
from online_dbscan import OnlineDBSCAN

LOG = logging.getLogger("graph_builder")

GRAPH_NAME = "semantic_map"


# ---------------------------------------------------------------------------
# Cypher helper
# ---------------------------------------------------------------------------


def _cypher(cur, query: str) -> list:
    """Execute a Cypher query through the AGE SQL wrapper and return rows."""
    sql = (
        f"SELECT * FROM cypher('{GRAPH_NAME}', $$ {query} $$) AS (result agtype)"
    )
    cur.execute(sql)
    return cur.fetchall()


# ---------------------------------------------------------------------------
# AGE connection setup
# ---------------------------------------------------------------------------


def _connect_age(host: str, port: int, dbname: str, user: str, password: str):
    """Open a psycopg2 connection and initialise AGE extensions."""
    conn = psycopg2.connect(
        host=host,
        port=port,
        dbname=dbname,
        user=user,
        password=password,
    )
    conn.autocommit = True
    cur = conn.cursor()
    cur.execute("LOAD 'age'")
    cur.execute('SET search_path = ag_catalog, "$user", public')
    # Ensure the graph exists
    cur.execute(
        "SELECT count(*) FROM ag_catalog.ag_graph WHERE name = %s",
        (GRAPH_NAME,),
    )
    if cur.fetchone()[0] == 0:
        cur.execute("SELECT create_graph(%s)", (GRAPH_NAME,))
    return conn


# ---------------------------------------------------------------------------
# Embedding helpers
# ---------------------------------------------------------------------------


def _decode_embedding(b64: str, dim: int) -> np.ndarray:
    """Decode a base64 encoded float32 embedding to a numpy array."""
    raw = base64.b64decode(b64)
    return np.array(struct.unpack(f"<{dim}f", raw), dtype=np.float32)


def _encode_embedding(arr: np.ndarray) -> str:
    """Encode a numpy float32 array to base64."""
    return base64.b64encode(arr.astype(np.float32).tobytes()).decode("ascii")


# ---------------------------------------------------------------------------
# GraphBuilder
# ---------------------------------------------------------------------------


class GraphBuilder:
    """Stateful processor that turns detection envelopes into AGE graph ops."""

    def __init__(
        self,
        conn,
        epsilon: float = 1.5,
        min_samples: int = 3,
        spatial_threshold: float = 3.0,
        similarity_threshold: float = 0.7,
    ) -> None:
        self._conn = conn
        self._dbscan = OnlineDBSCAN(epsilon=epsilon, min_samples=min_samples)
        self._fusion = LandmarkFusion(
            spatial_threshold=spatial_threshold,
            similarity_threshold=similarity_threshold,
        )
        self._current_run_id: str | None = None
        self._last_keyframe_place: str | None = None
        self._highest_keyframe_id: int = -1

    # ------------------------------------------------------------------
    # Main processing entry point
    # ------------------------------------------------------------------

    def process_envelope(self, envelope: dict) -> None:
        """Process one detection envelope and update the graph."""
        run_id = envelope["run_id"]
        keyframe_id = envelope["keyframe_id"]
        timestamp = envelope["timestamp"]
        map_x = envelope["map_x"]
        map_y = envelope["map_y"]
        map_yaw = envelope["map_yaw"]
        detections = envelope.get("detections", [])

        # Idempotency guard
        if keyframe_id <= self._highest_keyframe_id:
            LOG.debug(
                "Skipping keyframe %d (already processed up to %d)",
                keyframe_id,
                self._highest_keyframe_id,
            )
            return

        # Track run transitions
        if run_id != self._current_run_id:
            self._current_run_id = run_id
            self._last_keyframe_place = None

        cur = self._conn.cursor()

        # (a) MERGE Run node
        _cypher(cur, f"MERGE (r:Run {{run_id: '{run_id}'}})")

        # (b) MERGE Keyframe node + HAS_KEYFRAME edge
        _cypher(
            cur,
            f"MERGE (kf:Keyframe {{keyframe_id: {keyframe_id}, run_id: '{run_id}'}})",
        )
        _cypher(
            cur,
            f"MATCH (r:Run {{run_id: '{run_id}'}}), "
            f"(kf:Keyframe {{keyframe_id: {keyframe_id}, run_id: '{run_id}'}}) "
            f"MERGE (r)-[:HAS_KEYFRAME]->(kf)",
        )

        # (c) CREATE Pose node + HAS_POSE edge
        _cypher(
            cur,
            f"MATCH (kf:Keyframe {{keyframe_id: {keyframe_id}, run_id: '{run_id}'}}) "
            f"CREATE (p:Pose {{x: {map_x}, y: {map_y}, yaw: {map_yaw}, "
            f"timestamp: {timestamp}}})"
            f"CREATE (kf)-[:HAS_POSE]->(p)",
        )

        # (d) Cluster into places via DBSCAN
        result = self._dbscan.add_point(run_id, keyframe_id, map_x, map_y)
        place_id = result["place_id"]
        new_place = result["new_place"]
        merged = result["merged"]

        # (e) If new place: CREATE Place node
        if new_place and place_id is not None:
            cx, cy = self._dbscan.get_centroid(place_id)
            _cypher(
                cur,
                f"CREATE (:Place {{place_id: '{place_id}', "
                f"centroid_x: {cx}, centroid_y: {cy}}})",
            )

        # (f) Handle merges: redirect edges from dead places to surviving
        if merged and place_id is not None:
            for dead_id in merged:
                # Redirect IN_PLACE edges
                _cypher(
                    cur,
                    f"MATCH (kf)-[e:IN_PLACE]->(dead:Place {{place_id: '{dead_id}'}}) "
                    f"MATCH (surv:Place {{place_id: '{place_id}'}}) "
                    f"CREATE (kf)-[:IN_PLACE]->(surv) "
                    f"DELETE e",
                )
                # Redirect LOCATED_IN edges
                _cypher(
                    cur,
                    f"MATCH (obj)-[e:LOCATED_IN]->(dead:Place {{place_id: '{dead_id}'}}) "
                    f"MATCH (surv:Place {{place_id: '{place_id}'}}) "
                    f"CREATE (obj)-[:LOCATED_IN]->(surv) "
                    f"DELETE e",
                )
                # Redirect ADJACENT_TO edges (both directions)
                _cypher(
                    cur,
                    f"MATCH (dead:Place {{place_id: '{dead_id}'}})-[e:ADJACENT_TO]->(other:Place) "
                    f"WHERE other.place_id <> '{place_id}' "
                    f"MATCH (surv:Place {{place_id: '{place_id}'}}) "
                    f"MERGE (surv)-[:ADJACENT_TO]->(other) "
                    f"DELETE e",
                )
                _cypher(
                    cur,
                    f"MATCH (other:Place)-[e:ADJACENT_TO]->(dead:Place {{place_id: '{dead_id}'}}) "
                    f"WHERE other.place_id <> '{place_id}' "
                    f"MATCH (surv:Place {{place_id: '{place_id}'}}) "
                    f"MERGE (other)-[:ADJACENT_TO]->(surv) "
                    f"DELETE e",
                )
                # Delete the dead place node
                _cypher(
                    cur,
                    f"MATCH (dead:Place {{place_id: '{dead_id}'}}) "
                    f"DETACH DELETE dead",
                )
            # Update surviving place centroid
            cx, cy = self._dbscan.get_centroid(place_id)
            _cypher(
                cur,
                f"MATCH (p:Place {{place_id: '{place_id}'}}) "
                f"SET p.centroid_x = {cx}, p.centroid_y = {cy}",
            )

        # (g) If place is assigned (not noise), update centroid and link keyframe
        if place_id is not None:
            # Update centroid on the Place node
            if not new_place and not merged:
                cx, cy = self._dbscan.get_centroid(place_id)
                _cypher(
                    cur,
                    f"MATCH (p:Place {{place_id: '{place_id}'}}) "
                    f"SET p.centroid_x = {cx}, p.centroid_y = {cy}",
                )

            # CREATE IN_PLACE edge
            _cypher(
                cur,
                f"MATCH (kf:Keyframe {{keyframe_id: {keyframe_id}, run_id: '{run_id}'}}), "
                f"(p:Place {{place_id: '{place_id}'}}) "
                f"CREATE (kf)-[:IN_PLACE]->(p)",
            )

            # (h) Adjacency: if place differs from last keyframe's place
            if (
                self._last_keyframe_place is not None
                and self._last_keyframe_place != place_id
            ):
                _cypher(
                    cur,
                    f"MATCH (a:Place {{place_id: '{self._last_keyframe_place}'}}), "
                    f"(b:Place {{place_id: '{place_id}'}}) "
                    f"MERGE (a)-[:ADJACENT_TO]->(b)",
                )

            self._last_keyframe_place = place_id

        # (i) Process detections
        for det in detections:
            det_id = det["det_id"]
            class_name = det["class"]
            confidence = det["confidence"]
            bbox_str = str(det["bbox"])
            embedding_b64 = det.get("embedding")
            embedding_dim = det.get("embedding_dim", 512)

            # CREATE Observation node + HAS_OBSERVATION edge
            _cypher(
                cur,
                f"MATCH (kf:Keyframe {{keyframe_id: {keyframe_id}, run_id: '{run_id}'}}) "
                f"CREATE (obs:Observation {{det_id: '{det_id}', "
                f"class: '{class_name}', confidence: {confidence}, "
                f"bbox: '{bbox_str}'}}) "
                f"CREATE (kf)-[:HAS_OBSERVATION]->(obs)",
            )

            if embedding_b64 is None:
                continue

            # Decode embedding and fuse
            embedding = _decode_embedding(embedding_b64, embedding_dim)
            fusion_result = self._fusion.fuse(
                class_name=class_name,
                robot_x=map_x,
                robot_y=map_y,
                embedding=embedding,
                timestamp=timestamp,
            )
            object_id = fusion_result["object_id"]
            is_new = fusion_result["is_new"]

            if is_new:
                # CREATE Object node
                mean_emb_b64 = _encode_embedding(embedding)
                _cypher(
                    cur,
                    f"CREATE (:Object {{object_id: '{object_id}', "
                    f"class: '{class_name}', "
                    f"mean_embedding_b64: '{mean_emb_b64}', "
                    f"obs_count: 1}})",
                )
                # CREATE LOCATED_IN edge to current place (if assigned)
                if place_id is not None:
                    _cypher(
                        cur,
                        f"MATCH (o:Object {{object_id: '{object_id}'}}), "
                        f"(p:Place {{place_id: '{place_id}'}}) "
                        f"CREATE (o)-[:LOCATED_IN]->(p)",
                    )
            else:
                # Update existing Object node with new obs_count and embedding
                lm = self._fusion.get_landmark(object_id)
                if lm is not None:
                    mean_emb_b64 = _encode_embedding(
                        self._fusion._landmarks[object_id].mean_embedding
                    )
                    _cypher(
                        cur,
                        f"MATCH (o:Object {{object_id: '{object_id}'}}) "
                        f"SET o.obs_count = {lm['obs_count']}, "
                        f"o.mean_embedding_b64 = '{mean_emb_b64}'",
                    )

            # CREATE OBSERVES edge (Observation -> Object)
            _cypher(
                cur,
                f"MATCH (obs:Observation {{det_id: '{det_id}'}}), "
                f"(o:Object {{object_id: '{object_id}'}}) "
                f"CREATE (obs)-[:OBSERVES]->(o)",
            )

        # Update bookkeeping
        self._highest_keyframe_id = keyframe_id
        cur.close()
        LOG.info(
            "Processed keyframe %d (run=%s, place=%s, detections=%d)",
            keyframe_id,
            run_id,
            place_id,
            len(detections),
        )


# ---------------------------------------------------------------------------
# Zenoh callback & main
# ---------------------------------------------------------------------------


def _make_callback(builder: GraphBuilder):
    """Return a Zenoh subscriber callback that feeds the builder."""

    def _on_sample(sample: zenoh.Sample):
        try:
            payload = sample.payload.to_bytes().decode("utf-8")
            envelope = json.loads(payload)
            builder.process_envelope(envelope)
        except Exception:
            LOG.exception("Error processing detection envelope")

    return _on_sample


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Semantic graph builder: Zenoh detections -> Apache AGE"
    )
    parser.add_argument(
        "--connect",
        type=str,
        default=None,
        help="Zenoh connect endpoint (e.g. tcp/localhost:7447)",
    )
    parser.add_argument(
        "--key",
        type=str,
        default="tb/detections",
        help="Zenoh key expression to subscribe to",
    )
    parser.add_argument("--age-host", type=str, default="localhost")
    parser.add_argument("--age-port", type=int, default=5435)
    parser.add_argument("--age-db", type=str, default="graphdb")
    parser.add_argument("--age-user", type=str, default="postgres")
    parser.add_argument("--age-password", type=str, default="postgres")
    parser.add_argument(
        "--epsilon",
        type=float,
        default=1.5,
        help="DBSCAN epsilon (neighbourhood radius)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=3,
        help="DBSCAN min_samples for core point",
    )
    parser.add_argument(
        "--spatial-threshold",
        type=float,
        default=3.0,
        help="Landmark fusion spatial distance threshold",
    )
    parser.add_argument(
        "--similarity-threshold",
        type=float,
        default=0.7,
        help="Landmark fusion cosine similarity threshold",
    )
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    args = parse_args()

    # Connect to AGE
    LOG.info(
        "Connecting to AGE at %s:%d/%s", args.age_host, args.age_port, args.age_db
    )
    conn = _connect_age(
        host=args.age_host,
        port=args.age_port,
        dbname=args.age_db,
        user=args.age_user,
        password=args.age_password,
    )
    LOG.info("AGE connection established, graph '%s' ready", GRAPH_NAME)

    # Build the processor
    builder = GraphBuilder(
        conn=conn,
        epsilon=args.epsilon,
        min_samples=args.min_samples,
        spatial_threshold=args.spatial_threshold,
        similarity_threshold=args.similarity_threshold,
    )

    # Open Zenoh session
    zenoh_cfg = zenoh.Config()
    if args.connect:
        zenoh_cfg.insert_json5("connect/endpoints", json.dumps([args.connect]))

    LOG.info("Opening Zenoh session (connect=%s)", args.connect)
    session = zenoh.open(zenoh_cfg)

    LOG.info("Subscribing to '%s'", args.key)
    _sub = session.declare_subscriber(args.key, _make_callback(builder))

    LOG.info("Graph builder running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        LOG.info("Shutting down...")
    finally:
        session.close()
        conn.close()


if __name__ == "__main__":
    main()
