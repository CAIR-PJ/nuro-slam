#!/usr/bin/env python3
"""Read-only stage-aware audit of the consolidated EROAM batch."""

import json
import re
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BATCH = ROOT / "runs/consolidated_batch_v1"
RECOVERY = ROOT / "runs/cmax_recovery_v1"
AUDIT = ROOT / "reports/baseline_v1_audit"
OUT = ROOT / "reports/baseline_v1_audit_v2"

NAMES = ("eroam", "eroam_emba", "eroam_epba")
STAGE_COUNTS = {"eroam": 1, "eroam_emba": 2, "eroam_epba": 2}

def read_json(path):
    return json.loads(path.read_text())

def stages_from_record(record):
    """Recognize known stage containers without fabricating statuses."""
    if isinstance(record, list):
        return record
    if isinstance(record, dict):
        for key in ("stages", "stage_results", "results"):
            value = record.get(key)
            if isinstance(value, list):
                return value
        # A single stage may itself be represented by a dictionary.
        if any(k in record for k in (
            "return_code", "exit_code", "status", "success",
            "elapsed_seconds", "duration_seconds"
        )):
            return [record]
    return []

def classify(stage):
    if not isinstance(stage, dict):
        return "REVIEW", "Unrecognized stage format"

    status = str(stage.get("status", "")).strip().lower()
    rc = stage.get("return_code", stage.get("exit_code"))
    success = stage.get("success")

    failure = {"fail", "failed", "error", "timeout", "timed_out"}
    passed = {"pass", "passed", "success", "succeeded", "completed", "ok"}

    if status in failure or success is False:
        return "FAIL", "Explicit failure status"

    if rc is not None:
        if isinstance(rc, bool) or not isinstance(rc, int):
            return "REVIEW", "Non-integer return code"
        if rc != 0:
            return "FAIL", f"Nonzero return code: {rc}"

    if status in passed:
        return "PASS", f"Explicit completion status: {status}"

    if success is True:
        return "PASS", "Explicit success flag"

    if rc == 0:
        return "PASS", "Zero process exit code"

    return "REVIEW", "No explicit success/failure evidence"

def elapsed(stage):
    if not isinstance(stage, dict):
        return None
    for key in ("elapsed_seconds", "duration_seconds", "wall_seconds"):
        value = stage.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return round(float(value), 3)
    return None

def get_pipeline_record(data, name):
    if name in data:
        return data[name], name

    for container in ("pipelines", "results", "pipeline_results"):
        value = data.get(container)
        if isinstance(value, dict) and name in value:
            return value[name], f"{container}.{name}"
        if isinstance(value, list):
            matches = [
                item for item in value
                if isinstance(item, dict)
                and item.get("pipeline", item.get("pipeline_id")) == name
            ]
            if len(matches) == 1:
                return matches[0], f"{container}[pipeline={name}]"

    return None, "not found"

def main():
    manifest = read_json(ROOT / "manifest.json")
    if manifest.get("execution_enabled") is not False:
        raise SystemExit("STOP: global execution guard must be false")

    if OUT.exists():
        raise SystemExit("STOP: audit v2 already exists")

    batch = read_json(BATCH / "batch_results.json")
    old = read_json(AUDIT / "audit.json")
    recovery = read_json(RECOVERY / "results.json")

    if not isinstance(batch, dict) or not isinstance(old, dict):
        raise SystemExit("Invalid source evidence")

    if not isinstance(recovery, dict):
        raise SystemExit("Invalid recovery evidence")

    findings = {}
    warnings = []

    for name in NAMES:
        record, location = get_pipeline_record(batch, name)
        stages = stages_from_record(record)
        expected = STAGE_COUNTS[name]

        evaluations = []
        for index, stage in enumerate(stages, 1):
            status, reason = classify(stage)
            evaluations.append({
                "stage_number": index,
                "status": status,
                "reason": reason,
                "elapsed_seconds": elapsed(stage),
                "raw_stage": stage
            })

        if len(stages) != expected:
            overall = "REVIEW"
            warnings.append(
                f"{name}: expected {expected} stages, found {len(stages)}"
            )
        elif all(x["status"] == "PASS" for x in evaluations):
            overall = "PROCESS_PASS"
        elif any(x["status"] == "FAIL" for x in evaluations):
            overall = "PROCESS_FAIL"
        else:
            overall = "REVIEW"

        if overall != "PROCESS_PASS":
            warnings.append(f"{name}: {overall}")

        findings[name] = {
            "source_record_location": location,
            "expected_stages": expected,
            "found_stages": len(stages),
            "execution_status": overall,
            "stages": evaluations,
            "raw_pipeline_record": record
        }

    # Preserve all earlier artifact/trajectory evidence.
    revised = dict(old)
    revised["title"] = "Neuromorphic SLAM Baseline v1 Audit v2"
    revised["generated_utc"] = datetime.now(
        timezone.utc
    ).isoformat()
    revised["previous_audit"] = str(AUDIT / "audit.json")
    revised["eroam_stage_validation"] = findings
    revised["stage_validation_warnings"] = warnings
    revised["slam_execution_performed"] = False
    revised["global_execution_enabled"] = False

    for row in revised.get("pipelines", []):
        name = row.get("pipeline")
        if name in findings:
            row["execution"] = {
                "status": findings[name]["execution_status"],
                "stage_count": findings[name]["found_stages"],
                "expected_stage_count": findings[name]["expected_stages"],
                "stage_statuses": [
                    s["status"] for s in findings[name]["stages"]
                ],
                "source": findings[name]["source_record_location"]
            }

    lines = [
        "# Neuromorphic SLAM — Evidence Audit v2",
        "",
        "Stage-aware validation of the consolidated EROAM batch.",
        "",
        "| Pipeline | Expected stages | Found | Status | Stage times (s) |",
        "|---|---:|---:|---|---|"
    ]

    for name in NAMES:
        f = findings[name]
        times = [
            str(s["elapsed_seconds"])
            if s["elapsed_seconds"] is not None else "N/A"
            for s in f["stages"]
        ]
        lines.append(
            f"| {name} | {f['expected_stages']} | "
            f"{f['found_stages']} | {f['execution_status']} | "
            f"{', '.join(times) or 'N/A'} |"
        )

    lines += [
        "",
        "## Stage-level findings",
        ""
    ]

    for name in NAMES:
        lines.append(f"### {name}")
        f = findings[name]
        lines.append(
            f"Source record: `{f['source_record_location']}`"
        )
        for stage in f["stages"]:
            lines.append(
                f"- Stage {stage['stage_number']}: "
                f"{stage['status']} — {stage['reason']}"
            )
        lines.append("")

    lines += [
        "## Interpretation",
        "",
        "PROCESS_PASS establishes recorded process-stage completion, "
        "not trajectory accuracy or mapping quality.",
        "",
        "EPBA remains a 50 ms preview, whereas EMBA uses a longer "
        "evaluation interval. Runtime comparisons require matched windows.",
        "",
        "The previous audit's artifact and trajectory inventory "
        "is preserved. No SLAM execution was performed.",
        ""
    ]

    if warnings:
        lines += ["## Unresolved stage warnings", ""]
        lines.extend(f"- {w}" for w in warnings)

    OUT.mkdir(parents=True, exist_ok=False)

    with (OUT / "audit_v2.json").open("x") as f:
        json.dump(revised, f, indent=2)

    with (OUT / "audit_v2.md").open("x") as f:
        f.write("\n".join(lines))

    print("=== STEP 152 — STAGE-AWARE AUDIT ===")
    for name in NAMES:
        f = findings[name]
        print(
            f"{name:13s} "
            f"stages={f['found_stages']}/{f['expected_stages']} "
            f"status={f['execution_status']}"
        )
        for stage in f["stages"]:
            print(
                f"  stage {stage['stage_number']}: "
                f"{stage['status']} "
                f"elapsed={stage['elapsed_seconds']}"
            )

    print("Unresolved stage warnings:", len(warnings))
    print("Report:", OUT / "audit_v2.md")
    print("Evidence:", OUT / "audit_v2.json")
    print("SLAM execution: NONE")
    print("Global guard: UNCHANGED")

if __name__ == "__main__":
    main()
