"""Landmark fusion using CLIP embedding cosine similarity and spatial distance."""

from __future__ import annotations

import uuid
from typing import Optional

import numpy as np


class LandmarkFusion:
    """Fuse multi-view object detections into persistent landmarks.

    Primary discriminator: cosine similarity between CLIP embeddings.
    Secondary discriminator: Euclidean distance between robot poses.

    Both thresholds must be satisfied for two observations to merge.
    """

    def __init__(
        self,
        spatial_threshold: float = 3.0,
        similarity_threshold: float = 0.7,
    ) -> None:
        self._spatial_threshold = spatial_threshold
        self._similarity_threshold = similarity_threshold
        # Internal store: object_id -> _Landmark
        self._landmarks: dict[str, _Landmark] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def fuse(
        self,
        class_name: str,
        robot_x: float,
        robot_y: float,
        embedding: np.ndarray,
        timestamp: float,
    ) -> dict:
        """Incorporate a new detection and return fusion metadata.

        Returns
        -------
        dict with keys:
            object_id  : str
            is_new     : bool
            similarity : float  (1.0 for new landmarks)
            obs_count  : int
        """
        emb = _normalize(embedding)
        best_id, best_sim = self._find_best_match(class_name, robot_x, robot_y, emb)

        if best_id is not None:
            lm = self._landmarks[best_id]
            lm.update(robot_x, robot_y, emb, timestamp)
            return {
                "object_id": best_id,
                "is_new": False,
                "similarity": float(best_sim),
                "obs_count": lm.obs_count,
            }

        # No match — create a new landmark
        new_id = f"obj-{uuid.uuid4().hex[:8]}"
        self._landmarks[new_id] = _Landmark(
            object_id=new_id,
            class_name=class_name,
            x=robot_x,
            y=robot_y,
            embedding=emb,
            timestamp=timestamp,
        )
        return {
            "object_id": new_id,
            "is_new": True,
            "similarity": 1.0,
            "obs_count": 1,
        }

    def get_landmark(self, object_id: str) -> Optional[dict]:
        """Return the current state of a landmark, or None if not found."""
        lm = self._landmarks.get(object_id)
        if lm is None:
            return None
        return lm.to_dict()

    def get_all_landmarks(self) -> list[dict]:
        """Return a list of all landmark state dicts."""
        return [lm.to_dict() for lm in self._landmarks.values()]

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _find_best_match(
        self,
        class_name: str,
        robot_x: float,
        robot_y: float,
        emb: np.ndarray,
    ) -> tuple[Optional[str], float]:
        """Return (object_id, similarity) of the best matching landmark.

        Returns (None, 0.0) when no candidate passes both thresholds.
        """
        best_id: Optional[str] = None
        best_sim: float = -1.0

        for oid, lm in self._landmarks.items():
            if lm.class_name != class_name:
                continue

            # Spatial filter (robot-pose distance)
            dist = np.hypot(robot_x - lm.mean_x, robot_y - lm.mean_y)
            if dist > self._spatial_threshold:
                continue

            # Embedding similarity
            sim = float(np.dot(emb, lm.mean_embedding))
            if sim < self._similarity_threshold:
                continue

            if sim > best_sim:
                best_sim = sim
                best_id = oid

        return best_id, best_sim


# ---------------------------------------------------------------------------
# Internal data class
# ---------------------------------------------------------------------------


class _Landmark:
    """Mutable state for a single fused landmark."""

    __slots__ = (
        "object_id",
        "class_name",
        "mean_x",
        "mean_y",
        "mean_embedding",
        "obs_count",
        "first_seen",
        "last_seen",
    )

    def __init__(
        self,
        object_id: str,
        class_name: str,
        x: float,
        y: float,
        embedding: np.ndarray,
        timestamp: float,
    ) -> None:
        self.object_id = object_id
        self.class_name = class_name
        self.mean_x = x
        self.mean_y = y
        self.mean_embedding: np.ndarray = embedding.copy()
        self.obs_count = 1
        self.first_seen = timestamp
        self.last_seen = timestamp

    def update(
        self,
        x: float,
        y: float,
        embedding: np.ndarray,
        timestamp: float,
    ) -> None:
        """Incorporate a new observation via incremental running averages."""
        n = self.obs_count

        # Incremental mean for position
        self.mean_x = (self.mean_x * n + x) / (n + 1)
        self.mean_y = (self.mean_y * n + y) / (n + 1)

        # Incremental mean for embedding, then re-normalise
        raw = (self.mean_embedding * n + embedding) / (n + 1)
        self.mean_embedding = _normalize(raw)

        self.obs_count = n + 1
        self.last_seen = timestamp

    def to_dict(self) -> dict:
        return {
            "object_id": self.object_id,
            "class_name": self.class_name,
            "mean_x": self.mean_x,
            "mean_y": self.mean_y,
            "obs_count": self.obs_count,
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
        }


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------


def _normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    if norm < 1e-12:
        return v
    return v / norm
