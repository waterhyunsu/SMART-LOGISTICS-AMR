from copy import deepcopy
import json
from pathlib import Path
import unittest

from route_optimizer.cost_snapshot import build_snapshot


class CostSnapshotTest(unittest.TestCase):
    def setUp(self):
        fixture = Path(__file__).with_name("nav2_cost_snapshot_fixture.json")
        document = json.loads(fixture.read_text(encoding="utf-8"))
        self.metadata = document["metadata"]
        self.results = document["results"]

    def test_directions_have_independent_costs(self):
        segments = build_snapshot(self.metadata, self.results)["segments"]
        self.assertEqual(
            [(item["start"], item["goal"], item["distance_m"]) for item in segments],
            [("A", "B", 7.0), ("B", "A", 5.0)],
        )

    def test_failed_or_empty_path_is_unreachable(self):
        for succeeded, error_code in ((False, 1), (True, 0)):
            with self.subTest(succeeded=succeeded, error_code=error_code):
                self.results[0].update(
                    succeeded=succeeded, error_code=error_code, poses=[]
                )
                segment = build_snapshot(self.metadata, self.results)["segments"][0]
                self.assertFalse(segment["reachable"])
                self.assertIsNone(segment["distance_m"])
                self.assertTrue(segment["failure_reason"])

    def test_revision_is_stable_and_changes_with_inputs(self):
        revision = build_snapshot(self.metadata, self.results)["cost_revision"]
        reordered = build_snapshot(self.metadata, reversed(self.results))
        self.assertEqual(revision, reordered["cost_revision"])

        for key in ("map", "waypoints", "planner", "global_costmap"):
            with self.subTest(key=key):
                metadata = deepcopy(self.metadata)
                if key == "waypoints":
                    metadata[key]["A"][0] = 1
                else:
                    metadata[key]["revision_marker"] = "changed"
                updated = build_snapshot(metadata, self.results)
                self.assertNotEqual(revision, updated["cost_revision"])

        self.results[0]["poses"] = [[0, 0], [3, 4]]
        updated = build_snapshot(self.metadata, self.results)
        self.assertNotEqual(revision, updated["cost_revision"])

    def test_missing_reverse_result_prevents_snapshot(self):
        with self.assertRaisesRegex(ValueError, "Missing directed results"):
            build_snapshot(self.metadata, self.results[:1])

    def test_snapshot_preserves_waypoints_after_source_changes(self):
        snapshot = build_snapshot(self.metadata, self.results)
        original_waypoints = deepcopy(snapshot["metadata"]["waypoints"])
        original_revision = snapshot["cost_revision"]

        self.metadata["waypoints"]["A"][0] = 1.0

        self.assertEqual(snapshot["metadata"]["waypoints"], original_waypoints)
        self.assertEqual(snapshot["cost_revision"], original_revision)

        updated = build_snapshot(self.metadata, self.results)
        self.assertEqual(updated["metadata"]["waypoints"]["A"][0], 1.0)
        self.assertNotEqual(updated["cost_revision"], original_revision)