"""Bind a Cargo-built verified BigUint to production and the num facade."""
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
OUT = E / "biguint-cargo-export-v2445"
WORK = ROOT / "target/biguint-cargo-export-v2445"
WRAPPER = E / "biguint_cargo_rustc_wrapper_v2445.py"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    result = {}
    for directory, children, names in os.walk(root):
        children[:] = sorted(n for n in children if n not in {"target", "artifacts", "__pycache__"})
        for name in sorted(names):
            path = Path(directory) / name
            assert not path.is_symlink(), path
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
    prior = E / "biguint-crate-export-v2442"
    prior_protocol = read(prior / "protocol.json")
    prior_result = read(prior / "result.json")
    assert prior_result["exit_code"] == 0 and prior_result["verification_results"]["verified"] == 450
    assert read(E / "biguint-import-v2443/result.json")["complete"]
    old_fixture = read(E / "dependency-binding-fixture-preparation-v2426/result.json")
    fixture = Path(old_fixture["work"]) / "matched"
    old_files = {n.removeprefix("matched/"): h for n, h in old_fixture["work_files_sha256"].items() if n.startswith("matched/")}
    assert files(fixture) == old_files
    OUT.mkdir()
    shutil.copytree(fixture, WORK, ignore=shutil.ignore_patterns("target", "__pycache__"))
    vendor = WORK / "vendor/num-bigint"
    (vendor / "src/lib.rs").write_bytes((prior / "lib.rs").read_bytes())
    assert files(vendor) == prior_protocol["vendor_files_sha256"]
    inputs = files(WORK)
    for name in ["Cargo.toml", "Cargo.lock", "src/lib.rs", "verification/dependencies/Cargo.toml",
                 "verification/dependencies/Cargo.lock", "verification/dependencies/src/lib.rs"]:
        path = OUT / "inputs" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((WORK / name).read_bytes())
    artifacts = WORK / "artifacts"
    artifacts.mkdir()
    compiler_reports = OUT / "compiler"
    compiler_reports.mkdir()
    tool = Path(prior_protocol["command"][0])
    for name, value in prior_protocol["tool_sha256"].items():
        assert sha(tool.parent / name) == value
    config = {"vendor": str(vendor), "reports": str(compiler_reports),
              "export": str(artifacts / "num_bigint.vir"), "verus": str(tool),
              "verus_sha256": sha(tool), "wrapper_sha256": sha(WRAPPER)}
    config_path = OUT / "wrapper-config.json"
    config_path.write_text(json.dumps(config, indent=2) + "\n")
    environment = {"CARGO_HOME": "/tmp/csgrs-properties-cargo", "CARGO_NET_OFFLINE": "true",
                   "CCACHE_DIR": "/tmp/csgrs-properties-ccache", "CARGO_TERM_COLOR": "never",
                   "RUSTC_WRAPPER": str(WRAPPER), "CSGRS_BIGUINT_EXPORT_CONFIG": str(config_path),
                   "RUSTFLAGS": "-D warnings --check-cfg cfg(verus_keep_ghost_body) -L dependency=" + str(tool.parent), "PYTHONDONTWRITEBYTECODE": "1"}
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    helper_path = SOURCE / "crates/hyperreal/scripts/verus_dependencies.py"
    protocol = {"status": "Frozen before Cargo resolution and verified dependency compilation",
        "runner_sha256": sha(Path(__file__)), "wrapper_sha256": sha(WRAPPER),
        "config_sha256": sha(config_path), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "work": str(WORK),
        "work_files_sha256": inputs, "prior_export_protocol_sha256": sha(prior / "protocol.json"),
        "prior_export_result_sha256": sha(prior / "result.json"), "dependency_helper_sha256": sha(helper_path),
        "tool_sha256": prior_protocol["tool_sha256"], "pin": pin, "environment": environment,
        "operation": "prepare_dependencies(work, pin, dependency-reports), followed by check_dependency_identity",
        "scope": "Actual Cargo proof/production resolutions share the same patched dependency path. Cargo builds the complete num dependency graph while the wrapper verifies and compiles only num_bigint with its exact Cargo arguments and exports its contracts. Other compiler commands delegate unchanged. No arithmetic bodies are assumed or production sources edited. This support build does not prove unannotated upstream bodies, discharge caller premises or admit a scalar runtime change."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    spec = importlib.util.spec_from_file_location("qualified_dependencies_v2445", helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    saved = os.environ.copy()
    os.environ.update(environment)
    error, dependency_report = None, None
    start = time.monotonic()
    try:
        arguments, dependency_report = helper.prepare_dependencies(WORK, pin, OUT / "dependency-reports")
        helper.check_dependency_identity(dependency_report)
    except Exception as caught:
        error = f"{type(caught).__name__}: {caught}"
    finally:
        os.environ.clear()
        os.environ.update(saved)
    assert files(WORK) == inputs
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert all(sha(tool.parent / n) == h for n, h in prior_protocol["tool_sha256"].items())
    compiler_result_path = compiler_reports / "compiler-result.json"
    compiler = read(compiler_result_path) if compiler_result_path.exists() else None
    result = {"error": error, "seconds": time.monotonic() - start, "gate_passed": error is None,
              "protocol_sha256": sha(OUT / "protocol.json"), "source_unchanged": True,
              "main_unchanged": True, "fixture_unchanged": True, "tooling_unchanged": True,
              "compiler_result": compiler, "compiler_result_sha256": sha(compiler_result_path) if compiler else None,
              "dependency_report_sha256": sha(OUT / "dependency-reports/dependencies.json") if dependency_report else None,
              "export_sha256": sha(artifacts / "num_bigint.vir") if (artifacts / "num_bigint.vir").exists() else None,
              "retained_coverage_changed": False, "runtime_admitted": False}
    if error is None:
        assert compiler["gate_passed"] and compiler["verification_results"]["verified"] == 450
        assert compiler["warnings"] == 0
        packages = [p for p in dependency_report["packages"] if p["name"] == "num-bigint"]
        assert len(packages) == 1 and packages[0]["source"] is None
        assert Path(packages[0]["manifest_path"]) == vendor / "Cargo.toml"
        assert dependency_report["sources"][packages[0]["id"]] == files(vendor)
        result["verus_arguments"] = arguments
        result["bound_packages"] = len(dependency_report["packages"])
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
