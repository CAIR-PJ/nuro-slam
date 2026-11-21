#!/usr/bin/env python3
"""Read-only execution prerequisite audit for Nuro-SLAM."""

import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WS = ROOT.parent

sys.path.insert(0, str(ROOT))
from controller import build_plan

manifest = json.loads((ROOT / "manifest.json").read_text())

print("=== STEP 85: EXECUTION PREREQUISITE AUDIT ===")

ros = shutil.which("ros2")
print("\nROS 2 executable:", ros or "NOT FOUND")

results = []

for pipeline in manifest["pipelines"]:
    plan = build_plan(manifest, pipeline["id"])
    print(f"\n=== {plan['pipeline']} ===")

    blocked = []
    deferred = []
    warnings = list(plan["warnings"])

    output = Path(plan["output_root"])

    if output.exists():
        blocked.append(
            f"Output root already exists: {output}"
        )

    for stage in plan["stages"]:
        algorithm = stage["algorithm"]
        command = stage["command"]

        print(f"\nStage: {algorithm}")

        workdir = Path(stage["working_directory"])
        if not workdir.is_dir():
            blocked.append(f"Missing working directory: {workdir}")

        if algorithm in ("CMax", "EROAM"):
            overlay = Path(stage["ros_overlay"])

            if not overlay.is_file():
                blocked.append(f"Missing ROS overlay: {overlay}")

            if ros is None:
                blocked.append("ROS 2 executable unavailable")

            if len(command) < 4 or command[:2] != ["ros2", "launch"]:
                blocked.append("Unexpected ROS launch command")

        else:
            exe = Path(command[0])

            if not exe.is_file():
                blocked.append(f"Missing executable: {exe}")
            elif not exe.stat().st_mode & 0o111:
                blocked.append(f"Executable permission missing: {exe}")

        dependency = stage.get("depends_on")

        if dependency:
            path = output / dependency

            if not path.is_file():
                deferred.append(
                    f"{algorithm} requires generated trajectory: {path}"
                )

    for key, info in manifest["dataset"]["files"].items():
        path = Path(info["path"])

        if not path.is_file():
            blocked.append(f"Missing dataset input {key}: {path}")

    for item in blocked:
        print("BLOCKED:", item)

    for item in deferred:
        print("DEFERRED:", item)

    for item in warnings:
        print("WARNING:", item)

    if blocked:
        status = "BLOCKED"
    elif warnings:
        status = "REVIEW"
    elif deferred:
        status = "DEFERRED"
    else:
        status = "READY"

    print("STATUS:", status)
    results.append((plan["pipeline"], status))

print("\n=== SUMMARY ===")

for name, status in results:
    print(f"{name:15s} {status}")

print("\nExecution enabled:", manifest["execution_enabled"])
print("No algorithms executed")
print("No existing files modified")
print("=== STEP 85 COMPLETE ===")
