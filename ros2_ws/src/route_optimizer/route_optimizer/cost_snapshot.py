"""Build a directed cost snapshot from recorded ComputePathToPose results."""

import argparse
from hashlib import sha256
from itertools import pairwise, permutations
import json
from math import hypot, isfinite
from pathlib import Path
from copy import deepcopy


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
    canonical = json.dumps(
        snapshot, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    revision = sha256(canonical.encode("utf-8")).hexdigest()
    return {**snapshot, "cost_revision": f"nav2-v1-{revision}"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", type=Path)
    args = parser.parse_args()
    document = json.loads(args.recordings.read_text(encoding="utf-8"))
    snapshot = build_snapshot(document["metadata"], document["results"])
    print(json.dumps(snapshot, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()