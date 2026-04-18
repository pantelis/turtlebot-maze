"""Tests for OnlineDBSCAN incremental place clustering."""
import pytest
from online_dbscan import OnlineDBSCAN


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _add_points(db, run_id, points):
    """Add a list of (keyframe_id, x, y) tuples; return list of results."""
    results = []
    for kf_id, x, y in points:
        results.append(db.add_point(run_id, kf_id, x, y))
    return results


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestThreeClosePointsFormCluster:
    """3 points within epsilon → they should form a single cluster."""

    def test_three_close_points_form_cluster(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        pts = [(0, 0.0, 0.0), (1, 0.5, 0.0), (2, 1.0, 0.0)]
        _add_points(db, "r1", pts)

        labels = db.get_labels()
        place_ids = [v for v in labels.values() if v != -1]
        assert len(place_ids) == 3, "All three points should be clustered"
        assert len(set(place_ids)) == 1, "All three points should share one place_id"


class TestTwoPointsAreNoise:
    """Only 2 points with min_samples=3 → both remain noise."""

    def test_two_points_are_noise(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        _add_points(db, "r1", [(0, 0.0, 0.0), (1, 0.5, 0.0)])

        labels = db.get_labels()
        assert all(v == -1 for v in labels.values()), "Both points should be noise"


class TestBridgePointMergesTwoClusters:
    """Two separate clusters of 3, then a bridge point → 1 cluster."""

    def test_bridge_point_merges_two_clusters(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)

        # Cluster A centred around (0, 0)
        _add_points(db, "r1", [
            (0, 0.0, 0.0),
            (1, 0.5, 0.0),
            (2, 0.0, 0.5),
        ])

        # Cluster B centred around (2.5, 0)
        _add_points(db, "r1", [
            (3, 2.5, 0.0),
            (4, 3.0, 0.0),
            (5, 2.5, 0.5),
        ])

        labels_before = db.get_labels()
        cluster_ids_before = set(v for v in labels_before.values() if v != -1)
        assert len(cluster_ids_before) == 2, "Should have two distinct clusters before bridge"

        # Bridge point equidistant between clusters; distance to each side ~1.25
        result = db.add_point("r1", 6, 1.25, 0.0)

        labels_after = db.get_labels()
        cluster_ids_after = set(v for v in labels_after.values() if v != -1)
        assert len(cluster_ids_after) == 1, "Bridge should merge clusters into one"
        assert result["merged"] or result["new_place"] or result["place_id"] is not None


class TestDistantPointsSeparateClusters:
    """Two groups far apart → 2 separate clusters."""

    def test_distant_points_separate_clusters(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)

        # Group 1 near origin
        _add_points(db, "r1", [
            (0, 0.0, 0.0),
            (1, 0.5, 0.0),
            (2, 0.0, 0.5),
        ])

        # Group 2 far away
        _add_points(db, "r1", [
            (3, 10.0, 10.0),
            (4, 10.5, 10.0),
            (5, 10.0, 10.5),
        ])

        labels = db.get_labels()
        cluster_ids = set(v for v in labels.values() if v != -1)
        assert len(cluster_ids) == 2, "Should produce exactly 2 distinct clusters"

        # Confirm no inter-group connectivity
        place_group1 = db.get_place_for_keyframe("r1", 0)
        place_group2 = db.get_place_for_keyframe("r1", 3)
        assert place_group1 != place_group2


class TestCentroidIsMeanOfMembers:
    """Centroid should equal (mean_x, mean_y) of cluster members."""

    def test_centroid_is_mean_of_members(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        pts = [(0, 0.0, 0.0), (1, 1.0, 0.0), (2, 0.0, 1.0)]
        _add_points(db, "r1", pts)

        place_id = db.get_place_for_keyframe("r1", 0)
        assert place_id is not None

        cx, cy = db.get_centroid(place_id)
        expected_x = (0.0 + 1.0 + 0.0) / 3
        expected_y = (0.0 + 0.0 + 1.0) / 3
        assert abs(cx - expected_x) < 1e-9
        assert abs(cy - expected_y) < 1e-9


class TestReturnsPlaceForAssignedPoint:
    """`get_place_for_keyframe` returns a non-None place_id for clustered point."""

    def test_returns_place_for_assigned_point(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        _add_points(db, "run42", [(0, 0.0, 0.0), (1, 0.5, 0.0), (2, 1.0, 0.0)])

        place = db.get_place_for_keyframe("run42", 1)
        assert place is not None
        assert isinstance(place, str)
        assert "run42" in place


class TestReturnsNoneForNoise:
    """Noise point should return None (or -1) from `get_place_for_keyframe`."""

    def test_returns_none_for_noise(self):
        db = OnlineDBSCAN(epsilon=1.5, min_samples=3)
        # Only add 2 points — they stay noise
        _add_points(db, "r1", [(0, 0.0, 0.0), (1, 0.5, 0.0)])

        place = db.get_place_for_keyframe("r1", 0)
        # Spec says None or -1
        assert place is None or place == -1
