"""Online DBSCAN for incremental place construction.

As keyframe poses (x, y) arrive one at a time, this algorithm clusters them
into "places" (spatial regions).  Cluster merges are handled when a new point
bridges two existing clusters.

Place IDs have the format: ``{run_id}_p{counter}``
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Set, Tuple


class OnlineDBSCAN:
    """Incremental DBSCAN that processes one pose at a time.

    Parameters
    ----------
    epsilon:
        Neighbourhood radius (inclusive).
    min_samples:
        Minimum number of points in the neighbourhood (including self) for a
        point to be considered a *core* point.
    """

    def __init__(self, epsilon: float = 1.5, min_samples: int = 3) -> None:
        self.epsilon = epsilon
        self.min_samples = min_samples

        # ---- per-point data ------------------------------------------------
        # pose_index → (run_id, keyframe_id, x, y)
        self._poses: List[Tuple[str, int, float, float]] = []

        # pose_index → set of pose_indices within epsilon (self included)
        self._neighbors: List[Set[int]] = []

        # pose_index → place_id string  OR  -1 (noise)
        self._labels: List[str | int] = []

        # ---- per-place data ------------------------------------------------
        # place_id → set of pose_indices
        self._place_members: Dict[str, Set[int]] = {}

        # place_id → (sum_x, sum_y, count) for O(1) centroid updates
        self._place_sums: Dict[str, Tuple[float, float, int]] = {}

        # ---- lookup helpers ------------------------------------------------
        # (run_id, keyframe_id) → pose_index
        self._kf_index: Dict[Tuple[str, int], int] = {}

        # global place counter for ID generation
        self._place_counter: int = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add_point(
        self, run_id: str, keyframe_id: int, x: float, y: float
    ) -> dict:
        """Add one keyframe pose and update cluster assignments.

        Returns
        -------
        dict with keys:
            place_id  : str | None — place assigned to this point
            merged    : list[str]  — place_ids that were consumed by a merge
            new_place : bool       — True if a brand-new place was created
            changed   : set[int]   — pose indices whose label changed
        """
        idx = len(self._poses)
        self._poses.append((run_id, keyframe_id, x, y))
        self._labels.append(-1)
        self._kf_index[(run_id, keyframe_id)] = idx

        # ------------------------------------------------------------------
        # Step 1: compute neighbours within epsilon (self always included)
        # ------------------------------------------------------------------
        my_neighbors: Set[int] = set()
        for other_idx, (_, _, ox, oy) in enumerate(self._poses):
            if math.dist((x, y), (ox, oy)) <= self.epsilon:
                my_neighbors.add(other_idx)

        self._neighbors.append(my_neighbors)

        # ------------------------------------------------------------------
        # Step 2: update neighbour sets of all affected existing points
        # ------------------------------------------------------------------
        for other_idx in my_neighbors:
            if other_idx != idx:
                self._neighbors[other_idx].add(idx)

        # ------------------------------------------------------------------
        # Step 3: recompute core status for new point + its neighbourhood
        # ------------------------------------------------------------------
        # (core status is implicit: a point is core iff
        #  len(self._neighbors[i]) >= min_samples)

        # ------------------------------------------------------------------
        # Step 4: determine cluster assignment for the new point
        # ------------------------------------------------------------------
        changed: Set[int] = set()
        merged: List[str] = []
        new_place = False

        # Collect the place_ids of core neighbours (excluding self if noise)
        core_neighbor_places: Set[str] = set()
        for nb_idx in my_neighbors:
            if nb_idx == idx:
                continue
            if len(self._neighbors[nb_idx]) >= self.min_samples:
                lbl = self._labels[nb_idx]
                if isinstance(lbl, str):
                    core_neighbor_places.add(lbl)

        self_is_core = len(my_neighbors) >= self.min_samples

        if core_neighbor_places:
            # Join the surviving cluster (lowest place_id lexicographically
            # keeps the canonical name so tests that check IDs are stable)
            surviving = min(core_neighbor_places)
            to_merge = core_neighbor_places - {surviving}

            if to_merge:
                # Merge all other clusters into surviving
                for dead_id in to_merge:
                    self._merge_into(dead_id, surviving, changed)
                merged = list(to_merge)

            # Assign the new point to the surviving cluster
            self._assign(idx, surviving, changed)

        elif self_is_core:
            # No cluster nearby — spawn a new place
            new_id = self._new_place_id(run_id)
            self._place_members[new_id] = set()
            self._place_sums[new_id] = (0.0, 0.0, 0)
            self._assign(idx, new_id, changed)
            new_place = True

        # else: remains noise (-1)

        # ------------------------------------------------------------------
        # Step 5: promote noise neighbours that are now border points
        # (they are within epsilon of a core point → assign to its cluster)
        # ------------------------------------------------------------------
        if self_is_core:
            my_place = self._labels[idx]
            if isinstance(my_place, str):
                for nb_idx in my_neighbors:
                    if nb_idx == idx:
                        continue
                    if self._labels[nb_idx] == -1:
                        self._assign(nb_idx, my_place, changed)

        # Also: existing core points that gained idx as a neighbour might now
        # be core for the first time, potentially promoting their own noise
        # neighbours.  We iterate once to handle this transitively.
        for nb_idx in list(my_neighbors):
            if nb_idx == idx:
                continue
            if len(self._neighbors[nb_idx]) >= self.min_samples:
                nb_place = self._labels[nb_idx]
                if isinstance(nb_place, str):
                    for nb2_idx in self._neighbors[nb_idx]:
                        if self._labels[nb2_idx] == -1:
                            self._assign(nb2_idx, nb_place, changed)

        place_id = self._labels[idx]
        return {
            "place_id": place_id if isinstance(place_id, str) else None,
            "merged": merged,
            "new_place": new_place,
            "changed": changed,
        }

    def get_labels(self) -> Dict[int, str | int]:
        """Return mapping pose_index → place_id (or -1 for noise)."""
        return dict(enumerate(self._labels))

    def get_place_for_keyframe(
        self, run_id: str, keyframe_id: int
    ) -> Optional[str]:
        """Return place_id for the given keyframe, or None if noise."""
        key = (run_id, keyframe_id)
        if key not in self._kf_index:
            return None
        lbl = self._labels[self._kf_index[key]]
        return lbl if isinstance(lbl, str) else None

    def get_centroid(self, place_id: str) -> Tuple[float, float]:
        """Return (mean_x, mean_y) of all points in the place."""
        sx, sy, count = self._place_sums[place_id]
        return sx / count, sy / count

    def get_place_keyframe_count(self, place_id: str) -> int:
        """Return number of keyframes assigned to this place."""
        return len(self._place_members[place_id])

    def get_all_place_ids(self) -> Set[str]:
        """Return all active place IDs."""
        return set(self._place_members.keys())

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _new_place_id(self, run_id: str) -> str:
        pid = f"{run_id}_p{self._place_counter}"
        self._place_counter += 1
        return pid

    def _assign(self, idx: int, place_id: str, changed: Set[int]) -> None:
        """Assign pose *idx* to *place_id*, updating accounting."""
        old_label = self._labels[idx]

        # Remove from old place if applicable
        if isinstance(old_label, str) and old_label != place_id:
            if idx in self._place_members.get(old_label, set()):
                self._place_members[old_label].discard(idx)
                _, _, x, y = self._poses[idx]
                sx, sy, c = self._place_sums[old_label]
                self._place_sums[old_label] = (sx - x, sy - y, c - 1)

        if old_label == place_id:
            return  # already assigned; nothing to do

        self._labels[idx] = place_id
        changed.add(idx)

        _, _, x, y = self._poses[idx]
        self._place_members[place_id].add(idx)
        sx, sy, c = self._place_sums[place_id]
        self._place_sums[place_id] = (sx + x, sy + y, c + 1)

    def _merge_into(
        self, dead_id: str, surviving_id: str, changed: Set[int]
    ) -> None:
        """Reassign all members of *dead_id* to *surviving_id*."""
        members = list(self._place_members.get(dead_id, set()))
        for m_idx in members:
            self._assign(m_idx, surviving_id, changed)
        # Clean up dead place
        self._place_members.pop(dead_id, None)
        self._place_sums.pop(dead_id, None)
