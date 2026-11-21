#!/usr/bin/env python3
import json
import hashlib
from pathlib import Path

root = Path(__file__).resolve().parent
manifest = json.loads((root / "manifest.json").read_text())

def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

print("=== NURO-SLAM BENCHMARK PREFLIGHT ===")

passed = True

for name, item in manifest["dataset"]["files"].items():
    path = Path(item["path"])
    exists = path.is_file()
    size_ok = exists and path.stat().st_size == item["bytes"]
    hash_ok = size_ok and sha256(path) == item["sha256"]
    ok = exists and size_ok and hash_ok

    print(f"{name}: {'PASS' if ok else 'FAIL'}")
    passed = passed and ok

print("Pipelines:", len(manifest["pipelines"]))
print("Categories:", len(manifest["benchmark_categories"]))
print("Execution enabled:", manifest["execution_enabled"])
print("Overall:", "PASS" if passed else "FAIL")

raise SystemExit(0 if passed else 1)
