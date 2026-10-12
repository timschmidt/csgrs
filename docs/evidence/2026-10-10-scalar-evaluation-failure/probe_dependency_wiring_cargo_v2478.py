"""Exercise the reusable dependency builder through real matched Cargo resolutions."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PROPOSAL = E / "dependency-wiring-proposal-v2476"
OUT = E / "dependency-wiring-cargo-v2478"
WORK = ROOT / "target/dependency-wiring-cargo-v2478"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    result = {}
    for directory, children, names in os.walk(root):
        children[:] = sorted(n for n in children if n not in {"target", "artifacts", "__pycache__"})
        for name in sorted(names):
            path = Path(directory) / name
            assert not path.is_symlink()
            result[str(path.relative_to(root))] = sha(path)
    return result


def main():
    assert not OUT.exists() and not WORK.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    units_path = E / "dependency-wiring-units-v2477"
    units, units_protocol = read(units_path / "result.json"), read(units_path / "protocol.json")
    assert units["gate_passed"]
    assert all(sha(PROPOSAL / n) == h for n, h in units_protocol["proposal_files_sha256"].items())
    base_path = E / "bigint-portable-cargo-v2467"
    base, base_protocol = read(base_path / "result.json"), read(base_path / "protocol.json")
    assert base["gate_passed"]
    old_work = Path(base_protocol["work"])
    assert files(old_work) == base_protocol["work_files_sha256"]
    OUT.mkdir()
    shutil.copytree(old_work, WORK, ignore=shutil.ignore_patterns("target", "artifacts", "__pycache__"))
    vendor = WORK / "vendor/num-bigint"
    manifest = E / "bigint-portable-repackage-v2473/Cargo.toml"
    (vendor / "Cargo.toml").write_bytes(manifest.read_bytes())
    provenance = read(vendor / "verification/provenance.json")
    copied_models = {}
    for name, record in provenance["proof_models"].items():
        original = SOURCE / record["source"]
        assert sha(original) == record["sha256"] == sha(vendor / name)
        relative = record["source"].removeprefix("crates/hyperreal/")
        destination = WORK / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        assert not destination.exists()
        destination.write_bytes(original.read_bytes())
        copied_models[relative] = record["sha256"]
    inputs = files(WORK)
    for name in ["Cargo.toml", "Cargo.lock", "src/lib.rs", "verification/dependencies/Cargo.toml",
                 "verification/dependencies/Cargo.lock", "verification/dependencies/src/lib.rs"]:
        destination = OUT / "inputs" / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((WORK / name).read_bytes())
    tool = Path(read(base_path / "wrapper-config.json")["verus"])
    assert all(sha(tool.parent / n) == h for n, h in base_protocol["tool_sha256"].items())
    environment = {"CARGO_HOME": "/tmp/csgrs-properties-cargo", "CARGO_NET_OFFLINE": "true",
        "CCACHE_DIR": "/tmp/csgrs-properties-ccache", "CARGO_TERM_COLOR": "never", "PYTHONDONTWRITEBYTECODE": "1"}
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    protocol = {"status": "Frozen before invoking the proposed reusable builder with actual Cargo",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "unit_result_sha256": sha(units_path / "result.json"),
        "unit_protocol_sha256": sha(units_path / "protocol.json"), "proposal_files_sha256": units_protocol["proposal_files_sha256"],
        "base_result_sha256": sha(base_path / "result.json"), "base_protocol_sha256": sha(base_path / "protocol.json"),
        "work": str(WORK), "work_files_sha256": inputs, "copied_scalar_model_files": copied_models,
        "manifest_overlay_sha256": sha(manifest), "verus": str(tool), "tool_sha256": base_protocol["tool_sha256"],
        "pin": pin, "environment": environment,
        "operation": "candidate.prepare_dependencies(work, pin, reports, verus_path=tool), then candidate.check_dependency_identity(evidence)",
        "scope": "Use the same matched production/proof Cargo fixture, with the corrected portable package manifest and seventeen byte-identical scalar models at their normal crate-relative paths. The proposed builder itself selects a fresh target generation, configures the compiler wrapper, verifies/compiles the actual dependency, binds the Cargo library and emits import arguments. The driver supplies no ad hoc wrapper or Rust flags. This tests the reusable integration path but does not edit SOURCE, discharge producer premises, admit runtime changes or promote retained coverage."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    sys.path.insert(0, str(PROPOSAL))
    spec = importlib.util.spec_from_file_location("wiring_dependencies_v2478", PROPOSAL / "verus_dependencies.py")
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    saved = os.environ.copy()
    os.environ.update(environment)
    error, evidence, arguments = None, None, None
    start = time.monotonic()
    try:
        arguments, evidence = helper.prepare_dependencies(WORK, pin, OUT / "dependency-reports", verus_path=tool)
        helper.check_dependency_identity(evidence)
    except Exception as caught:
        error = f"{type(caught).__name__}: {caught}"
    finally:
        os.environ.clear()
        os.environ.update(saved)
    generations = sorted((WORK / "target/verification-dependencies").glob("verified-*"))
    assert len(generations) <= 1
    if generations:
        (OUT / "compiler").mkdir()
        for name in ["compiler-config.json", "compiler-invocation.json", "compiler-result.json", "compiler.stdout.json", "compiler.stderr.log"]:
            path = generations[0] / name
            if path.exists():
                (OUT / "compiler" / name).write_bytes(path.read_bytes())
    assert files(WORK) == inputs and files(old_work) == base_protocol["work_files_sha256"]
    assert all(sha(PROPOSAL / n) == h for n, h in units_protocol["proposal_files_sha256"].items())
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert all(sha(tool.parent / n) == h for n, h in base_protocol["tool_sha256"].items())
    compiler_path = OUT / "compiler/compiler-result.json"
    compiler = read(compiler_path) if compiler_path.exists() else None
    result = {"error": error, "seconds": time.monotonic() - start, "gate_passed": error is None,
        "protocol_sha256": sha(OUT / "protocol.json"), "compiler_result": compiler,
        "compiler_result_sha256": sha(compiler_path) if compiler else None,
        "dependency_report_sha256": sha(OUT / "dependency-reports/dependencies.json") if evidence else None,
        "verus_arguments": arguments, "source_unchanged": True, "main_unchanged": True,
        "proposal_unchanged": True, "fixture_unchanged": True, "tooling_unchanged": True,
        "retained_coverage_changed": False, "runtime_admitted": False}
    if error is None:
        assert compiler["gate_passed"] and compiler["verified"] == 453 and compiler["warnings"] == 0
        assert evidence["verified_metadata"]["library_artifacts"] == compiler["library_artifacts"]
        assert len(evidence["packages"]) == len(evidence["bindings"]) == 8
        result["bound_packages"] = len(evidence["packages"])
        result["export_sha256"] = evidence["verified_metadata"]["export_sha256"]
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
