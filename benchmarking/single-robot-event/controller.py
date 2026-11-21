#!/usr/bin/env python3
"""Nuro-SLAM benchmark controller: dry-run planning only."""

import argparse
import json
import shlex
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WS = ROOT.parent

def build_plan(manifest, pipeline_id):
    pipelines = {
        p["id"]: p for p in manifest["pipelines"]
    }

    if pipeline_id not in pipelines:
        raise ValueError(f"Unknown pipeline: {pipeline_id}")

    pipeline = pipelines[pipeline_id]
    files = manifest["dataset"]["files"]

    events = files["events"]["path"]
    camera = files["cmax_camera"]["path"]
    calib = files["optimizer_calibration"]["path"]

    output = ROOT / "runs" / pipeline_id
    stages = []
    warnings = []

    if pipeline["stages"][0] == "CMax":
        port = WS / "ports/cmax_ros2_port"

        stages.append({
            "algorithm": "CMax",
            "working_directory": str(port),
            "ros_overlay": str(port / "install/setup.zsh"),
            "command": [
                "ros2", "launch", "cmax_slam_ros2",
                "offline.launch.py",
                f"events:={events}",
                f"calibration:={camera}",
                "profile:=synthetic",
                f"results:={output / 'frontend'}",
                "rviz:=false",
            ],
            "output": str(output / "frontend"),
        })

    elif pipeline["stages"][0] == "EROAM":
        port = WS / "ports/eroam_ros2_port"

        stages.append({
            "algorithm": "EROAM",
            "working_directory": str(port),
            "ros_overlay": str(port / "install/setup.zsh"),
            "command": [
                "ros2", "launch", "eroam_ros2",
                "offline.launch.py",
                f"events:={events}",
                f"results:={output / 'frontend'}",
                "packet_size:=500",
            ],
            "output": str(output / "frontend"),
        })

        warnings.append(
            "EROAM launcher uses fx=fy=90.0; "
            "Playroom calibration uses approximately 91.40147. "
            "Calibration equivalence is NOT established."
        )

    if len(pipeline["stages"]) > 1:
        backend = pipeline["stages"][1]
        frontend_trajectory = output / "frontend/trajectory.tum"

        if backend == "EMBA":
            exe = (
                WS / "ports/emba_ros2_port/install/"
                "emba_ros2/lib/emba_ros2/emba_cli"
            )

            command = [
                str(exe),
                "--events", events,
                "--calibration", calib,
                "--trajectory", str(frontend_trajectory),
                "--output", str(output / "backend"),
            ]

        elif backend == "EPBA":
            exe = (
                WS / "ports/epba_ros2_port/install/"
                "epba_ros2/lib/epba_ros2/epba_native_cli"
            )

            command = [
                str(exe),
                "--events", events,
                "--calib", calib,
                "--poses", str(frontend_trajectory),
                "--out", str(output / "backend"),
                "--start", "0.006249173",
                "--duration", "0.05",
            ]

            warnings.append(
                "EPBA is a 50 ms research preview, "
                "not a full-sequence optimizer."
            )

        else:
            raise ValueError(f"Unsupported backend: {backend}")

        stages.append({
            "algorithm": backend,
            "working_directory": str(WS),
            "ros_overlay": None,
            "command": command,
            "depends_on": "frontend/trajectory.tum",
            "output": str(output / "backend"),
        })

    if pipeline.get("status") == "requires_validation":
        warnings.append(
            "Pipeline requires validation before execution."
        )

    return {
        "pipeline": pipeline_id,
        "track": pipeline["track"],
        "execution_enabled": False,
        "output_root": str(output),
        "stages": stages,
        "warnings": warnings,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pipeline",
        default="all",
        help="Pipeline ID or 'all'",
    )
    args = parser.parse_args()

    manifest = json.loads(
        (ROOT / "manifest.json").read_text()
    )

    if manifest.get("execution_enabled") is not False:
        raise RuntimeError(
            "Controller requires execution_enabled=false"
        )

    ids = (
        [p["id"] for p in manifest["pipelines"]]
        if args.pipeline == "all"
        else [args.pipeline]
    )

    for pipeline_id in ids:
        plan = build_plan(manifest, pipeline_id)

        print(f"\n=== {pipeline_id} ===")
        print("Track:", plan["track"])
        print("Execution: DISABLED")

        for i, stage in enumerate(plan["stages"], 1):
            print(f"\nStage {i}: {stage['algorithm']}")
            print("Working directory:", stage["working_directory"])
            print(
                "Command:",
                shlex.join(stage["command"])
            )

        for warning in plan["warnings"]:
            print("WARNING:", warning)

    print("\nDRY RUN COMPLETE — NO ALGORITHMS EXECUTED")


if __name__ == "__main__":
    main()
