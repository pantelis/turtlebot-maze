"""Tests for LandmarkFusion — run BEFORE implementation to verify TDD red phase."""

import numpy as np
import pytest

from landmark_fusion import LandmarkFusion


def _rand_embedding(seed: int = 0) -> np.ndarray:
    rng = np.random.RandomState(seed)
    v = rng.randn(512).astype(np.float32)
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------------------
# Basic creation
# ---------------------------------------------------------------------------


def test_first_observation_creates_landmark():
    fusion = LandmarkFusion()
    emb = _rand_embedding(seed=0)
    result = fusion.fuse("chair", 1.0, 2.0, emb, timestamp=0.0)

    assert result["is_new"] is True
    assert isinstance(result["object_id"], str)
    assert result["obs_count"] == 1


# ---------------------------------------------------------------------------
# Class isolation
# ---------------------------------------------------------------------------


def test_different_class_creates_separate_landmark():
    fusion = LandmarkFusion()
    emb = _rand_embedding(seed=0)

    r1 = fusion.fuse("chair", 1.0, 1.0, emb, timestamp=0.0)
    r2 = fusion.fuse("table", 1.0, 1.0, emb, timestamp=1.0)

    assert r1["object_id"] != r2["object_id"]
    assert r2["is_new"] is True
    assert len(fusion.get_all_landmarks()) == 2


# ---------------------------------------------------------------------------
# Merge path
# ---------------------------------------------------------------------------


def test_same_embedding_same_location_merges():
    fusion = LandmarkFusion()
    emb = _rand_embedding(seed=0)

    r1 = fusion.fuse("chair", 1.0, 1.0, emb, timestamp=0.0)
    r2 = fusion.fuse("chair", 1.1, 0.9, emb, timestamp=1.0)

    assert r1["object_id"] == r2["object_id"]
    assert r2["is_new"] is False
    assert r2["obs_count"] == 2
    assert len(fusion.get_all_landmarks()) == 1


# ---------------------------------------------------------------------------
# Spatial guard
# ---------------------------------------------------------------------------


def test_similar_embedding_far_location_no_merge():
    """Same embedding but 100 m apart — spatial threshold blocks merge."""
    fusion = LandmarkFusion(spatial_threshold=3.0, similarity_threshold=0.7)
    emb = _rand_embedding(seed=0)

    r1 = fusion.fuse("chair", 0.0, 0.0, emb, timestamp=0.0)
    r2 = fusion.fuse("chair", 100.0, 0.0, emb, timestamp=1.0)

    assert r1["object_id"] != r2["object_id"]
    assert r2["is_new"] is True
    assert len(fusion.get_all_landmarks()) == 2


# ---------------------------------------------------------------------------
# Similarity guard
# ---------------------------------------------------------------------------


def test_different_embedding_same_location_no_merge():
    """Different random embeddings at same location — similarity blocks merge."""
    fusion = LandmarkFusion(spatial_threshold=3.0, similarity_threshold=0.7)

    emb_a = _rand_embedding(seed=0)
    emb_b = _rand_embedding(seed=99)

    # Verify the two embeddings are genuinely dissimilar
    sim = float(np.dot(emb_a, emb_b))
    assert sim < 0.7, f"Test setup: embeddings are unexpectedly similar ({sim:.3f})"

    r1 = fusion.fuse("chair", 1.0, 1.0, emb_a, timestamp=0.0)
    r2 = fusion.fuse("chair", 1.0, 1.0, emb_b, timestamp=1.0)

    assert r1["object_id"] != r2["object_id"]
    assert r2["is_new"] is True
    assert len(fusion.get_all_landmarks()) == 2


# ---------------------------------------------------------------------------
# State retrieval
# ---------------------------------------------------------------------------


def test_get_returns_correct_state():
    fusion = LandmarkFusion()
    emb = _rand_embedding(seed=0)

    r1 = fusion.fuse("bottle", 5.0, 3.0, emb, timestamp=10.0)
    fusion.fuse("bottle", 5.2, 3.1, emb, timestamp=20.0)

    lm = fusion.get_landmark(r1["object_id"])

    assert lm is not None
    assert lm["class_name"] == "bottle"
    assert lm["obs_count"] == 2
    assert lm["object_id"] == r1["object_id"]
    # Position should be the mean of the two observations
    assert abs(lm["mean_x"] - 5.1) < 1e-5
    assert abs(lm["mean_y"] - 3.05) < 1e-5
    assert lm["first_seen"] == pytest.approx(10.0)
    assert lm["last_seen"] == pytest.approx(20.0)
