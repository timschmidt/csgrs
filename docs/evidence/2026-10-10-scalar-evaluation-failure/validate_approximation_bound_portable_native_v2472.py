"""Relink the unchanged independent producer oracle to the portable Cargo dependency."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREVIEW = E / "approximation-bound-candidate-v2460"
EXPORT = E / "bigint-portable-cargo-v2467"
OUT = E / "approximation-bound-portable-native-v2472"
WORK = ROOT / "target/approximation-bound-portable-native-v2472"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

CONVERT = """fn observation(bound: BoundInfo) -> crate::Observation {
    match bound {
        BoundInfo::Unknown => crate::Observation::Unknown,
        BoundInfo::Zero => crate::Observation::Zero,
        BoundInfo::NonZero { sign, msd, exact_msd } => crate::Observation::NonZero {
            sign: sign.map(|sign| match sign {
                Sign::Minus => -1,
                Sign::NoSign => 0,
                Sign::Plus => 1,
            }),
            msd,
            exact: exact_msd,
        },
    }
}
"""


def main():
    assert not OUT.exists() and not WORK.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert read(E / "approximation-bound-portable-proof-v2469/result.json")["verification_results"]["verified"] == 737
    assert read(E / "approximation-bound-guards-v2463/result.json")["complete"]
    preview = read(PREVIEW / "protocol.json")
    assert all(sha(PREVIEW / n) == h for n, h in preview["preview_files_sha256"].items())
    export, export_protocol = read(EXPORT / "result.json"), read(EXPORT / "protocol.json")
    dependencies = read(EXPORT / "dependency-reports/dependencies.json")
    assert export["gate_passed"] and export["compiler_result"]["warnings"] == 0
    old_dependencies = read((E / checkpoint["source_record"]).parent / "proof-reports/dependencies.json")
    all_artifacts = old_dependencies["artifacts"] | dependencies["artifacts"]
    assert all(sha(Path(n)) == h for n, h in all_artifacts.items())
    fixture = Path(export_protocol["work"])
    assert all(sha(fixture / n) == h for n, h in export_protocol["work_files_sha256"].items())
    tool = Path(read(EXPORT / "wrapper-config.json")["verus"]).parent
    assert all(sha(tool / n) == h for n, h in export_protocol["tool_sha256"].items())
    harness = E / "approximation-bound-native-harness-v2464.rs"
    instrumentation = E / "approximation-bound-instrumentation-v2464.rs"
    shared = PREVIEW / "candidate/crates/hyperreal/src/computable/node/approximation_bound.rs"
    bounds_path = SOURCE / "crates/hyperreal/src/computable/node/bounds.rs"
    word_path = SOURCE / "crates/hyperreal/src/verified/word.rs"
    bounds, word = bounds_path.read_text(), word_path.read_text()
    enum_start = bounds.index("#[derive(Clone, Copy, Debug, PartialEq, Eq)]\n#[cfg_attr(verus_keep_ghost, verus_verify)]\npub(crate) enum BoundInfo {")
    enum_end = bounds.index("\n#[cfg(verus_keep_ghost)]\nverus! {", enum_start)
    constructor = bounds.index("    fn with_sign_msd(")
    constructor_start = bounds.rfind("    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]", 0, constructor)
    constructor_end = bounds.index("\n    #[cfg(not(verus_keep_ghost))]\n    fn from_rational", constructor)
    word_start = word.index("/// Convert the exact bit-length difference and comparison correction plus an offset to an MSD.")
    word_end = word.index("\n/// Keep a representable upper exponent", word_start)
    original = (PREVIEW / "original-method.rs").read_text()
    assert original.endswith("\n\n")
    excerpts = {"bound_enum": {"source": str(bounds_path), "start": enum_start, "end": enum_end},
        "bound_constructor": {"source": str(bounds_path), "start": constructor_start, "end": constructor_end},
        "word_function": {"source": str(word_path), "start": word_start, "end": word_end}}
    for record in excerpts.values():
        text = Path(record["source"]).read_text()[record["start"]:record["end"]]
        record["excerpt_sha256"] = hashlib.sha256(text.encode()).hexdigest()
    common_bound = "type Precision = i32;\n" + bounds[enum_start:enum_end] + "\nimpl BoundInfo {\n" + bounds[constructor_start:constructor_end] + "}\n"
    modules = ['mod verified { pub(crate) mod word { include!("word.rs"); } }\n']
    inputs = {"harness.rs": harness.read_text(), "instrumentation.rs": instrumentation.read_text(),
        "bound.rs": common_bound, "word.rs": word[word_start:word_end], "convert.rs": CONVERT,
        "original.rs": "impl Computable {\n" + original.removesuffix("\n") + "}\n"}
    for name, dependency, producer, counted in [
        ("candidate_plain", "patched::bigint", str(shared), False),
        ("original_plain", "upstream", "original.rs", False),
        ("candidate_counted", "patched::bigint", str(shared), True),
        ("original_counted", "upstream", "original.rs", True),
    ]:
        imports = "{BigInt as ActualBigInt, BigUint as ActualMagnitude, Sign}" if counted else "{BigInt, Sign}"
        module = f"mod {name} {{\nuse {dependency}::{imports};\n"
        module += 'include!("bound.rs");\nstruct Computable;\n'
        if counted:
            module += 'include!("instrumentation.rs");\n'
        module += "include!(" + json.dumps(producer) + ");\n" + 'include!("convert.rs");\n'
        if counted:
            module += """pub(crate) fn run(precision: i32, value: &ActualBigInt) -> (crate::Observation, crate::Calls) {
    let calls = Counters::default();
    let value = BigInt { value, calls: &calls };
    let result = observation(Computable::bound_from_approx(precision, &value));
    (result, calls.0.get())
}
"""
        else:
            module += """pub(crate) fn run(precision: i32, value: &BigInt) -> crate::Observation {
    observation(Computable::bound_from_approx(precision, value))
}
"""
        modules.append(module + "}\n")
    inputs["modules.rs"] = "\n".join(modules)
    OUT.mkdir()
    (OUT / "inputs").mkdir()
    WORK.mkdir()
    for name, text in inputs.items():
        (WORK / name).write_text(text)
        (OUT / "inputs" / name).write_text(text)
    upstream = [n for n in old_dependencies["artifacts"] if Path(n).name.startswith("libnum_bigint-") and n.endswith(".rlib")]
    assert len(upstream) == 1
    facade = dependencies["verus_arguments"][1]
    assert facade.startswith("num=")
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    binary = WORK / "corpus"
    commands = [
        {"name": "format", "command": ["rustfmt", "--edition", "2024", "--config", "skip_children=true", "--check", str(harness), str(instrumentation), str(shared)]},
        {"name": "compile", "command": ["rustc", "+" + pin["rust_toolchain"], "--edition=2024", "-D", "warnings",
            "--check-cfg", "cfg(verus_keep_ghost)", "--check-cfg", "cfg(verus_keep_ghost_body)",
            "--extern", "patched=" + facade.removeprefix("num="), "--extern", "upstream=" + upstream[0],
            *dependencies["verus_arguments"][-2:], *old_dependencies["verus_arguments"][-2:],
            "-L", "dependency=" + str(tool), str(WORK / "harness.rs"), "-o", str(binary)]},
        {"name": "execute", "command": [str(binary)]},
    ]
    frozen = {"status": "Frozen before native formatting, compilation and one corpus execution",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "commands": commands,
        "work_inputs_sha256": {name: sha(WORK / name) for name in inputs}, "excerpts": excerpts,
        "extraction_rule": "Exact source slices and original method body, wrapped only in isolated types/modules; remove one trailing separator newline after the original method. No other bound/cache code is copied or stubbed.",
        "harness_sha256": sha(harness), "instrumentation_sha256": sha(instrumentation), "shared_source_sha256": sha(shared),
        "preview_protocol_sha256": sha(PREVIEW / "protocol.json"), "cargo_export_result_sha256": sha(EXPORT / "result.json"),
        "dependency_report_sha256": sha(EXPORT / "dependency-reports/dependencies.json"),
        "dependency_artifacts_sha256": all_artifacts, "tool_sha256": export_protocol["tool_sha256"],
        "expected_counts": {"fixtures": "198258", "queries": "2775612", "plain_values": "5551224", "counted_values": "5551224"},
        "scope": "Repeat the unchanged focused corpus after dependency packaging changes, comparing the proposed shared file using the actual self-contained Cargo-exported BigInt and the original exact method using the pinned registry BigInt. Exact BoundInfo, constructor and checked-exponent source excerpts are shared by all wrappers. An independent u32-word endpoint oracle uses increment/decrement and repeated division to derive every expected bound field. The corpus uses all u16 magnitudes, both signs and NoSign, padded and multiword inputs, power-of-two neighbours through 4096 bits, and fourteen precision requests per fixture including i32 extremes and the representable-exponent boundary; duplicates among precision requests are retained. Separate counted adapters delegate to those actual dependencies and compare method-call selection, including lazy trailing-zero scans. This does not prove caller premises, approximation existence or full node behavior, and does not qualify scalar runtime/cost/memory/size gates."}
    (OUT / "protocol.json").write_text(json.dumps(frozen, indent=2) + "\n")
    rows = []
    for job in commands:
        start = time.monotonic()
        run = subprocess.run(job["command"], cwd=ROOT, env=os.environ, capture_output=True, text=True)
        stdout, stderr = OUT / (job["name"] + ".stdout.log"), OUT / (job["name"] + ".stderr.log")
        stdout.write_text(run.stdout)
        stderr.write_text(run.stderr)
        passed = run.returncode == 0 and not run.stderr
        observations = None
        if job["name"] == "execute" and passed:
            observations = dict(item.split("=", 1) for item in run.stdout.split())
            passed = all(observations.get(k) == v for k, v in frozen["expected_counts"].items())
            passed = passed and all(int(observations[k]) > 0 for k in ["unknown", "overflow", "exact", "inexact"])
        row = {"name": job["name"], "exit_code": run.returncode, "seconds": time.monotonic() - start,
            "gate_passed": passed, "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr), "observations": observations}
        rows.append(row)
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        assert all(sha(PREVIEW / n) == h for n, h in preview["preview_files_sha256"].items())
        assert all(sha(fixture / n) == h for n, h in export_protocol["work_files_sha256"].items())
        assert all(sha(WORK / n) == h and sha(OUT / "inputs" / n) == h for n, h in frozen["work_inputs_sha256"].items())
        assert sha(harness) == frozen["harness_sha256"] and sha(instrumentation) == frozen["instrumentation_sha256"]
        assert all(sha(Path(n)) == h for n, h in all_artifacts.items())
        assert all(sha(tool / n) == h for n, h in export_protocol["tool_sha256"].items())
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
            "complete": len(rows) == len(commands) and all(r["gate_passed"] for r in rows),
            "binary_sha256": sha(binary) if binary.exists() else None,
            "source_unchanged": True, "main_unchanged": True, "dependency_inputs_unchanged": True,
            "harness_inputs_unchanged_during_validation": True, "fresh_scalar_runtime_tests": 0,
            "fresh_isolated_producer_corpus_executions": int(job["name"] == "execute"),
            "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
