"""Build and load directed cost snapshots from ComputePathToPose results."""

import argparse
from copy import deepcopy
from hashlib import sha256
from itertools import pairwise, permutations
import json
from math import hypot, isfinite
from pathlib import Path


def path_length(poses):
    """Sum the distances between consecutive XY positions in meters."""
    if not poses:
        raise ValueError("Empty path")
    if any(not isfinite(value) for point in poses for value in point):
        raise ValueError("Non-finite path coordinate")

    length = sum(
        hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in pairwise(poses)
    )
    if not isfinite(length):
        raise ValueError("Non-finite path length")
    return float(length)


def build_snapshot(metadata, results):
    for key in ("map", "waypoints", "planner", "global_costmap"):
        if not metadata.get(key):
            raise ValueError(f"Missing snapshot metadata: {key}")

    expected = set(permutations(metadata["waypoints"], 2))
    segments = {}

    for result in results:
        pair = (result["start"], result["goal"])
        if pair not in expected or pair in segments:
            raise ValueError(f"Unexpected or duplicate directed pair: {pair}")

        segment = {
            "start": pair[0],
            "goal": pair[1],
            "reachable": False,
            "distance_m": None,
        }

        succeeded = result["succeeded"] and result["error_code"] == 0
        if succeeded and result["poses"]:
            segment["distance_m"] = path_length(result["poses"])
            segment["reachable"] = True
        else:
            segment["failure_reason"] = result.get("failure_reason") or (
                "Empty path" if succeeded
                else f"Planner did not return a path (error_code={result['error_code']})"
            )

        segments[pair] = segment

    missing = expected - segments.keys()
    if missing:
        raise ValueError(f"Missing directed results: {sorted(missing)}")

    snapshot = {
        "schema_version": 1,
        "metadata": deepcopy(metadata),
        "segments": [segments[pair] for pair in sorted(segments)],
    }
    return {**snapshot, "cost_revision": snapshot_revision(snapshot)}


def snapshot_revision(snapshot):
    canonical = json.dumps(
        snapshot, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    revision = sha256(canonical.encode("utf-8")).hexdigest()
    return f"nav2-v1-{revision}"


def load_snapshot(path):
    """Read a saved snapshot and return its revision, waypoint IDs, and costs."""
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    revision = document.pop("cost_revision")

    if document["schema_version"] != 1:
        raise ValueError("Unsupported cost snapshot schema version")
    if revision != snapshot_revision(document):
        raise ValueError("Cost revision does not match snapshot contents")

    waypoints = frozenset(document["metadata"]["waypoints"])
    expected = set(permutations(waypoints, 2))
    seen = set()

    # Two tasks at the same waypoint need no movement between them.
    costs = {(waypoint, waypoint): 0.0 for waypoint in waypoints}

    for segment in document["segments"]:
        pair = (segment["start"], segment["goal"])
        if pair not in expected or pair in seen:
            raise ValueError(f"Unexpected or duplicate directed pair: {pair}")
        seen.add(pair)

        if segment["reachable"]:
            distance = segment["distance_m"]
            if (
                type(distance) not in (int, float)
                or not isfinite(distance)
                or distance < 0
            ):
                raise ValueError(f"Invalid distance for directed pair: {pair}")
            costs[pair] = distance

    if seen != expected:
        raise ValueError("Missing directed costs in snapshot")

    return revision, waypoints, costs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", type=Path)
    args = parser.parse_args()
    document = json.loads(args.recordings.read_text(encoding="utf-8"))
    snapshot = build_snapshot(document["metadata"], document["results"])
    print(json.dumps(snapshot, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()