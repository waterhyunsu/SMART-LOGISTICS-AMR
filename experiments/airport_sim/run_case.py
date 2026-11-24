"""Run one isolated five-order trial from HOME, with machine-readable evidence."""

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def stop(group):
    # Only signal process groups started by this trial.
    try:
        os.killpg(group.pid, signal.SIGINT)
        group.wait(timeout=8)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        os.killpg(group.pid, signal.SIGTERM)
        group.wait(timeout=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", choices=("fifo", "optimized"))
    parser.add_argument("results", type=Path)
    args = parser.parse_args()
    if (not os.getenv("GZ_PARTITION")
            or os.getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"
            or not os.getenv("ROS_DOMAIN_ID", "").isdigit()):
        parser.error("Choose an isolated ROS domain and Gazebo partition with LOCALHOST discovery.")
    root = args.results.resolve()
    root.mkdir(parents=True, exist_ok=True)
    prefix = "fifo" if args.policy == "fifo" else "opt"
    result_path = root / (args.policy + ".json")
    snapshot = root / "cost_snapshot.json"
    check = [sys.executable, "-u", str(HERE / "verify_airport.py")]
    with ExitStack() as stack:
        def start(cmd, log):
            stream = stack.enter_context((root / log).open("x"))
            proc = subprocess.Popen(cmd, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            stack.callback(stop, proc)
            return proc

        start(["ros2", "launch", str(HERE / "airport.launch.py")], prefix + ".launch.log")
        subprocess.run([*check, "init"], check=True)
        if not snapshot.exists():
            subprocess.run([*check, "costs", str(snapshot)], check=True)
        else:
            from ament_index_python.packages import get_package_share_directory
            from verify_airport import ASSETS, digest
            import yaml
            meta = json.loads(snapshot.read_text())["metadata"]
            waypoints = yaml.safe_load((ASSETS / "waypoints.yaml").read_text())["waypoints"]
            stock_params = Path(get_package_share_directory("nav2_bringup")) / "params" / "nav2_params.yaml"
            assets = {"yaml_sha256": "airport.yaml", "pgm_sha256": "airport.pgm",
                      "world_xacro_sha256": "airport.sdf.xacro"}
            if (meta["waypoints"] != waypoints
                    or meta["global_costmap"]["params_sha256"] != digest(stock_params)
                    or any(meta["map"][key] != digest(ASSETS / name) for key, name in assets.items())):
                raise ValueError("Costs do not match the airport map, waypoints or Nav2 profile")
        if args.policy == "fifo":
            server = [sys.executable, str(HERE / "fifo_reference.py"), str(snapshot)]
        else:
            server = ["ros2", "run", "route_optimizer", "route_optimizer", "--ros-args",
                      "-p", f"cost_snapshot_path:={snapshot}"]
        start(server, prefix + ".c.log")
        start([sys.executable, "-m", "amr_navigation.gate1_mock", "d", "--ros-args",
               "-p", "use_sim_time:=true", "-p", "wait_for_nav2_before_dispatch:=true",
               "-p", f"waypoints_file:={HERE / 'assets' / 'waypoints.yaml'}"], prefix + ".d.log")
        with (root / (args.policy + ".log")).open("x") as stream:
            subprocess.run([*check, "mission", "multi", str(snapshot), str(result_path)],
                           stdout=stream, stderr=subprocess.STDOUT, check=True)


if __name__ == "__main__":
    main()
