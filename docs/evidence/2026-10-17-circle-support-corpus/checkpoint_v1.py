"""Admit the independent fixture repair and close cache qualification evidence."""
import ast
import concurrent.futures
import gzip
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parent
ROOT = W.parents[2]
E = ROOT / "docs/evidence/2026-10-17-circle-support-corpus"
PARENT = ROOT / "docs/evidence/2026-10-17-test-storage-recovery"
PREFIX = str(E.relative_to(ROOT)) + "/"
sys.path.insert(0, str(E.parent / "2026-10-16-physical-build-requalification"))
from load_expectations_v1 import load


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def span(raw):
    start = re.search(rb'"current_work"\s*:\s*', raw).end()
    value, length = json.JSONDecoder().raw_decode(raw[start:].decode())
    return start, start + len(raw[start:].decode()[:length].encode()), value


decision = read(W / "decision-v1.json")
base = decision["main_parent_commit"]
assert not E.exists()
assert subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip() == base
subprocess.run(["git", "diff", "--quiet"], cwd=ROOT, check=True)
subprocess.run(["git", "diff", "--cached", "--quiet"], cwd=ROOT, check=True)
previous_path = PARENT / "checkpoint-v1.json"
previous = read(previous_path)
assert all(sha(ROOT / p) == h for p, h in previous["evidence_files_sha256"].items())
assert sha(ROOT / previous["artifact_expectations"]) == previous["artifact_expectations_sha256"]
prior, missing = load(ROOT / previous["artifact_expectations"], allow_unqualified=True)
assert len(prior) == 1182349 and len(missing) == 27093
inherited = previous["inherited_implementation_sha256"] | previous["implementation_sha256"]
extra = previous["extra_unchanged_inputs_sha256"]
dependencies = previous["live_source_dependency_pins_sha256"]
live = previous["restored_native_sha256"] | inherited
assert all(sha(ROOT / p) == h for pins in [live, extra, dependencies] for p, h in pins.items())
changes = decision["implementation_sha256"]
assert len(changes) == 2 and not set(changes).intersection(extra.keys() | dependencies.keys())
with gzip.open(W / "preparation-v1/main-input-hashes.json.gz", "rt") as f:
    main = json.load(f)
with gzip.open(W / "preparation-v1/source-hashes.json.gz", "rt") as f:
    candidate = json.load(f)
assert all(sha(ROOT / p) == h for p, h in main.items())
assert all(sha(W / "candidate-source-v1" / p) == h for p, h in candidate.items())
for name in ["default-corpus-v1", "native-build-v1", "native-corpus-v1", "curve-all-v1",
             "storage-validation-v1", "native-build-v2", "native-corpus-v2"]:
    assert read(W / name / "result.json")["complete"]
assert read(W / "curve-all-v1/result.json")["cargo_target_closed"]
assert read(W / "storage-validation-v1/result.json")["cargo_target_closed"]
assert read(W / "storage-validation-v1/result.json")["command_failures"] == []
assert read(W / "storage-reproduction-v1/result.json")["reproduced_failures"] == 5
assert all(sha(W / "storage-source-v1" / p) == h for p, h in changes.items())
assert all(sha(p) == h for p, h in read(W / "feature-scope-v1.json")["inputs_sha256"].items())
for commands in W.glob("*/commands.json"):
    for row in read(commands):
        for suffix in ["stdout", "stderr", "resources"]:
            if suffix + "_sha256" in row:
                assert sha(commands.parent / (row["name"] + "." + suffix)) == row[suffix + "_sha256"]
for protocol in W.glob("*/protocol.json"):
    record = read(protocol)
    for key in ["input_sha256", "read_only_dependency_sha256"]:
        assert all(sha(p) == h for p, h in record.get(key, {}).items()), (protocol, key)
for p in W.glob("*.py"):
    ast.parse(p.read_text(), filename=str(p))

E.mkdir()
retained, physical_only = {}, []
for p in sorted(W.rglob("*")):
    if not p.is_file() or "__pycache__" in p.parts:
        continue
    rel = p.relative_to(W)
    if any(part.startswith(("candidate-source-", "curve-all-target-", "storage-source-", "storage-lint-target-")) for part in rel.parts):
        continue
    if any(part in ["scratch", "compiler-scratch", "selected-temporary-directory", "runtime-source-mirror"] for part in rel.parts):
        continue
    with p.open("rb") as f:
        magic = f.read(4)
    if magic == b"\x7fELF" or p.stat().st_size > 50_000_000:
        physical_only.append({"path": str(p), "sha256": sha(p), "bytes": p.stat().st_size})
        continue
    dest = E / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dest)
    assert sha(dest) == sha(p)
    retained[str(rel)] = {"original": str(p), "sha256": sha(p)}
for p in E.rglob("*.py"):
    ast.parse(p.read_text(), filename=str(p))
for p, h in changes.items():
    assert (ROOT / p).read_text().split("#[cfg(test)]", 1)[0] == (W / "storage-source-v1" / p).read_text().split("#[cfg(test)]", 1)[0]
    shutil.copy2(W / "storage-source-v1" / p, ROOT / p)
    assert sha(ROOT / p) == h
(E / "applied.patch").write_bytes(subprocess.check_output(["git", "diff", "--binary", "--", *changes], cwd=ROOT))
with (E / "main-format.stdout").open("wb") as out, (E / "main-format.stderr").open("wb") as err:
    fmt = subprocess.run(["cargo", "fmt", "--all", "--", "--check"], cwd=ROOT, stdout=out, stderr=err)
assert fmt.returncode == 0
assert all(sha(ROOT / p) == h for p, h in (main | changes).items())
binary = read(W / "storage-validation-v1/binary.json")
assert sha(binary["path"]) == binary["sha256"]
post_tmp = E / "post-application-temporary-directory"
post_tmp.mkdir()
post_commands = []
for name, selection in [
    ("app", "app::tests::"),
    ("arrow", "arrow_report::tests::writes_readable_arrow_report"),
]:
    command = [binary["path"], selection, "--nocapture", "--test-threads=1"]
    with (E / ("main-" + name + ".stdout")).open("wb") as out, (E / ("main-" + name + ".stderr")).open("wb") as err:
        result = subprocess.run(["timeout", "--kill-after=20s", "60s", *command], cwd=ROOT,
                                env=os.environ | {"TMPDIR": str(post_tmp), "RUST_BACKTRACE": "0"},
                                stdout=out, stderr=err)
    assert result.returncode == 0
    post_commands.append({
        "name": name, "command": command, "exit_code": result.returncode,
        "stdout_sha256": sha(E / ("main-" + name + ".stdout")),
        "stderr_sha256": sha(E / ("main-" + name + ".stderr")),
    })
assert not list(post_tmp.iterdir())
assert all(sha(ROOT / p) == h for p, h in (main | changes).items())
dump(E / "retention-v1.json", {
    "complete": True, "main_parent_commit": base, "production_prefixes_unchanged": True,
    "implementation_sha256": changes, "source_changes_applied": True,
    "source_inputs": len(main), "cache_index_applied": False,
    "retained_evidence_files": retained, "physical_artifacts_not_duplicated_in_git": physical_only,
    "format_exit_code": fmt.returncode, "python_sources_parse": True,
    "post_application_commands": post_commands, "post_application_temporary_directory_cleaned": True,
    "scope": "The independent test-only storage repair is applied. The cache candidate remains isolated, and all earlier admission and rejection decisions remain in force.",
})

ledger = ROOT / "docs/properties-status.json"
raw, before = ledger.read_bytes(), sha(ledger)
assert before == previous["ledger_after_sha256"]
reference = subprocess.check_output(["git", "show", "9025de62:docs/properties-status.json"], cwd=ROOT)
start, end, current = span(raw)
rs, re_, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rs], reference[re_:])
paths, links, directories = [], {}, []
failed = read(W / "storage-reproduction-v1/result.json")
assert all(sha(p) == r["sha256"] for p, r in failed["raw_failed_artifacts"].items())
roots = [W, *[Path(p) for p in failed["temporary_roots"]]]
for root in roots:
    assert root.exists() and not root.is_mount()
    for p in ([root] if root.is_file() else [root, *root.rglob("*")]):
        if p.is_dir():
            directories.append(str(p))
        if p.is_symlink():
            target = p.resolve(strict=True)
            assert target.is_file()
            links[str(p)] = {"link_text": os.readlink(p), "resolved_target": str(target)}
        if p.is_file():
            paths.append(p)
assert len({str(p) for p in paths}) == len(paths)


def inspect(p):
    before_stat, before_link = p.stat(), p.lstat()
    digest = sha(p)
    after_stat, after_link = p.stat(), p.lstat()
    for a, b in [(before_stat, after_stat), (before_link, after_link)]:
        assert all(getattr(a, k) == getattr(b, k) for k in ["st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns"])
    if str(p) in links:
        assert os.readlink(p) == links[str(p)]["link_text"]
        assert str(p.resolve(strict=True)) == links[str(p)]["resolved_target"]
    return str(p), digest


print("Hashing every completed physical path:", len(paths), flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical = dict(pool.map(inspect, paths, buffersize=128))
assert len(physical) == len(paths) and prior.keys().isdisjoint(physical)
expectations = {
    "schema_version": 3, "kind": "admitted test-only fixture storage repair and continued cache corpus; cache remains unadmitted",
    "parent_expectations": previous["artifact_expectations"],
    "parent_expectations_sha256": previous["artifact_expectations_sha256"],
    "inherited_expected_artifact_paths": len(prior), "inherited_missing_artifact_paths": len(missing),
    "added_physical_artifacts_sha256": physical, "new_physical_artifacts_verified": len(physical),
    "local_tool_symlink_targets": links, "physical_roots": [str(p) for p in roots], "directory_inventory": directories,
    "expected_artifact_paths": len(prior) + len(physical), "inherited_artifacts_reverified": False,
    "complete_artifact_audit_passed": False, "historical_preservation_complete": False,
    "scope": "Every new physical file path independently hashed with stable stat/lstat, including the fresh closed all-feature target, source mirrors, compiled tests and raw output. No inode deduplication or whole-history/zero-loss claim.",
}
with gzip.open(E / "artifact-expectations-v1.json.gz", "wt") as f:
    json.dump(expectations, f, sort_keys=True)
current["updated_on"] = "2026-10-17"
current["circle_support_corpus"] = {
    "status": decision["status"], "progress_classification": "progress", "main_parent_commit": base,
    "contract": PREFIX + "contract-v1.md", "decision": PREFIX + "decision-v1.json",
    "progress": PREFIX + "progress-v1.md", "checkpoint": PREFIX + "checkpoint-v1.json",
    "artifact_expectations": PREFIX + "artifact-expectations-v1.json.gz",
    "results": decision["results"], "cache_index_admitted": False, "source_changes_applied": True,
    "implementation_sha256": changes, "production_prefixes_unchanged": True,
    "native_disposition_preserved": True, "pending_native_paths_remaining": 0,
    "full_archived_integration_admitted": False, "native_runtime_admitted": False, "full_plan_complete": False,
    "limitations": decision["limits"], "next": decision["next"],
}
current["circle_support_index"]["continued_corpus"] = PREFIX + "decision-v1.json"
current["circle_support_index"]["status"] = decision["cache_status"]
current["circle_support_index"]["next"] = decision["next"]
updated = raw[:start] + json.dumps(current, indent=2).encode() + raw[end:]
us, ue, _ = span(updated)
assert (updated[:us], updated[ue:]) == (reference[:rs], reference[re_:])
assert {k: v for k, v in json.loads(updated).items() if k != "current_work"} == {
    k: v for k, v in json.loads(reference).items() if k != "current_work"
}
assert sha(ledger) == before
ledger.write_bytes(updated)
files = {str(p.relative_to(ROOT)): sha(p) for p in E.rglob("*") if p.is_file()}
record = {k: previous[k] for k in [
    "rejected_native_sha256", "restored_native_sha256", "removed_added_paths", "candidate_snapshot",
    "native_disposition_pin_scope", "live_native_disposition_overrides_sha256",
    "independently_reimplemented_archived_paths_sha256", "independently_admitted_archived_components_sha256",
]}
record.update({
    "status": decision["status"], "checkpoint_kind": "independent test-only storage repair and continued cache qualification",
    "previous_closure": str(previous_path.relative_to(ROOT)), "previous_closure_sha256": sha(previous_path),
    "evidence_files_sha256": files, "artifact_expectations": PREFIX + "artifact-expectations-v1.json.gz",
    "artifact_expectations_sha256": sha(E / "artifact-expectations-v1.json.gz"), "artifact_expectations_schema": 3,
    "new_physical_artifacts_verified": len(physical), "inherited_expected_artifact_paths": len(prior),
    "inherited_missing_artifact_paths": len(missing), "complete_artifact_audit_passed": False,
    "historical_preservation_complete": False, "implementation_sha256": changes,
    "inherited_implementation_sha256": inherited, "extra_unchanged_inputs_sha256": extra,
    "pending_native_sha256": {}, "live_source_dependency_pins_sha256": dependencies,
    "ledger_before_sha256": before, "ledger_after_sha256": sha(ledger), "only_current_work_changed": True,
    "python_sources_parse": True, "format_exit_code": fmt.returncode,
    "native_disposition_preserved": True, "source_changes_applied": True, "cache_index_admitted": False,
    "production_prefixes_unchanged": True,
    "single_line_kernel_admitted": True, "typed_text_consumer_migration": True,
    "general_exact_kicad_stroke_fidelity_claimed": False, "native_candidate_admitted": False,
    "native_runtime_admitted": False, "full_archived_integration_admitted": False, "full_plan_complete": False,
})
dump(E / "checkpoint-v1.json", record)
print(json.dumps({
    "new_physical_artifacts_verified": len(physical), "only_current_work_changed": True,
    "cache_index_applied": False, "inherited_missing_artifact_paths": len(missing),
}), flush=True)
