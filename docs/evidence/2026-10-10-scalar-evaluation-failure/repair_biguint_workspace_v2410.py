"""Preserve the workspace-root failure and exclude the standalone vendor workspace."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "prepare_biguint_runtime_preview_v2409.py"
    successor = E / "prepare_biguint_runtime_preview_v2411.py"
    failed = E / "biguint-runtime-preparation-v2409"
    record = E / "biguint-workspace-correction-v2410.json"
    assert not successor.exists() and not record.exists()
    result = json.loads((failed / "resolution-result.json").read_text())
    assert not result["complete"] and len(result["runs"]) == 1 and result["runs"][0]["exit_code"] != 0
    assert "multiple workspace roots found" in (failed / "harness.stderr.log").read_text()
    code = parent.read_text()
    anchor = '[workspace]\nresolver = "2"\n'
    assert code.count(anchor) == 1
    code = code.replace(anchor, anchor + 'exclude = ["vendor/num-bigint"]\n')
    for stem in ["biguint-runtime-preparation", "biguint-metadata-runtime-preview", "biguint-metadata-runtime-build"]:
        assert code.count(stem + "-v2409") == 1
        code = code.replace(stem + "-v2409", stem + "-v2411")
    ast.parse(code)
    successor.write_text(code)
    record.write_text(json.dumps({
        "status": "Forked before rerunning resolution; failed v2409 is immutable",
        "repair_runner_sha256": sha(Path(__file__)), "parent_preparation_sha256": sha(parent),
        "failed_resolution_sha256": sha(failed / "resolution-result.json"),
        "failed_stderr_sha256": sha(failed / "harness.stderr.log"),
        "successor": successor.name, "successor_sha256": sha(successor),
        "change": "Add the explicit workspace exclusion for the path dependency's own standalone workspace. Shared dependency sources, runtime harness and requested dependency identities are unchanged.",
        "source_changed": False, "runtime_admitted": False,
    }, indent=2) + "\n")
    print(json.dumps({"successor": successor.name, "workspace_exclusion_added": True}))


if __name__ == "__main__":
    main()
