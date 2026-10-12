"""Inspect preserved integer binaries without retrying timing or changing sources."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "nth-root-enclosure-integer-layout-v2078"
    assert not out.exists()
    initial_path = E / "nth-root-enclosure-initial-cost-review-v2021.json"
    initial = read(initial_path)
    rejection_path = E / "nth-root-enclosure-cost-rejection-v2021.json"
    rejection = read(rejection_path)
    assert not rejection["runtime_retained"] and not rejection["cost_gate_passed"]
    assert rejection["source_record_sha256"] == initial["source_record_sha256"]
    record = E / rejection["source_record"]
    assert sha(record) == rejection["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    previous = read(E / rejection["retained_runtime_checkpoint"])
    retained = json.loads(gzip.decompress((E / previous["source_record"]).read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    reviewed = ["crates/hyperreal/src/computable/node/approximation_queries.rs",
                "crates/hyperreal/src/computable/node/scale.rs",
                "crates/hyperreal/src/computable/node/integer_scale_tests.rs",
                "crates/hyperreal/src/computable/node/evaluation_budget.rs"]
    assert all(sources[name] == retained[name] == sha(SOURCE / name) for name in reviewed)
    assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
    assert all(sha(ROOT / name) == digest for name, digest in guard.items())
    family = initial["families"]["integer"]
    functions = ["probe::main", "<hyperreal::computable::node::Computable>::approx_signal",
                 "hyperreal::computable::node::scale_at_precision_ref"]
    out.mkdir()
    shutil.copyfile(__file__, out / "reproduce.py")
    results, canonical = {}, {}
    for which in ["baseline", "candidate"]:
        binary = Path(family["binaries"][which])
        assert sha(binary) == family["binary_sha256"][which]
        command = ["nm", "-S", "--defined-only", "-C", str(binary)]
        symbols = subprocess.check_output(command, cwd=ROOT, text=True)
        (out / (which + "-symbols.txt")).write_text(symbols)
        entries = {}
        for line in symbols.splitlines():
            parts = line.split(maxsplit=3)
            if len(parts) == 4 and parts[3] in functions:
                assert parts[3] not in entries
                entries[parts[3]] = (int(parts[0], 16), int(parts[1], 16))
        assert set(entries) == set(functions)
        details = {}
        for index, name in enumerate(functions):
            address, size = entries[name]
            command = ["objdump", "-d", "--demangle", "--no-show-raw-insn",
                       f"--start-address={address}", f"--stop-address={address + size}", str(binary)]
            assembly = subprocess.check_output(command, cwd=ROOT, text=True)
            raw_path = out / f"{which}-{index}.asm"
            raw_path.write_text(assembly)
            instructions = []
            for line in assembly.splitlines():
                match = re.match(r"\s*([0-9a-f]+):\s+(.*)", line)
                if not match:
                    continue
                instruction = match[2].split("#", 1)[0].rstrip()
                instruction = re.sub(r"[-+]?0x[0-9a-f]+\(%rip\)", "RIP_ADDRESS(%rip)", instruction)
                def target_name(match):
                    destination = int(match[1], 16)
                    if address <= destination < address + size:
                        return f"LOCAL+0x{destination - address:x}"
                    return "TARGET<" + match[2] + ">"
                instruction = re.sub(r"\b([0-9a-f]+) <([^>]+)>", target_name, instruction)
                instructions.append(f"{int(match[1], 16) - address:04x}: {instruction}")
            normalized = "\n".join(instructions) + "\n"
            normalized_path = out / f"{which}-{index}-normalized.asm"
            normalized_path.write_text(normalized)
            canonical[which, name] = normalized
            details[name] = {"address": address, "size_bytes": size,
                "address_mod_64": address % 64, "instructions": len(instructions),
                "assembly": raw_path.name, "assembly_sha256": sha(raw_path),
                "normalized_assembly": normalized_path.name, "normalized_sha256": sha(normalized_path)}
        results[which] = {"binary": str(binary), "binary_sha256": sha(binary), "functions": details}
    comparisons = {}
    for index, name in enumerate(functions):
        before, after = canonical["baseline", name], canonical["candidate", name]
        diff = out / f"function-{index}.diff"
        diff.write_text("".join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                                  fromfile="baseline", tofile="candidate", n=3)))
        comparisons[name] = {"normalized_instruction_shapes_equal": before == after,
                            "diff": diff.name, "diff_sha256": sha(diff)}
    for name in reviewed:
        destination = out / "inputs" / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((SOURCE / name).read_bytes())
    assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
    assert all(sha(ROOT / name) == digest for name, digest in guard.items())
    result = {
        "status": "Static review only; the failed timing gate remains failed",
        "source_record_sha256": rejection["source_record_sha256"],
        "rejection_record": rejection_path.name, "rejection_record_sha256": sha(rejection_path),
        "initial_cost_record": initial_path.name, "initial_cost_record_sha256": sha(initial_path),
        "reviewed_source_files_sha256": {name: sources[name] for name in reviewed},
        "reviewed_sources_identical_to_retained_v1836": True,
        "source_files_verified": len(sources), "main_guarded_files_verified": len(guard),
        "binaries": results, "comparisons": comparisons, "timed_processes": 0,
        "observations": [
            "Exact integer leaves check cancellation/exhaustion, preserve the exact-zero shortcut, validate precision, charge one evaluation-work unit, then call scale_at_precision_ref. This path precedes all graph validation and node-cache publication.",
            "The wide_gap fixture uses positive/negative 1000001-bit exact integers, precision i32::MAX and 10000 timed queries. In scale_at_precision_ref, the positive precision enters rounded_shift_right, whose gap > n.bits() branch returns zero before any bigint shift or allocation.",
            "The current out-of-line scale helper saves six registers and reserves 88 stack bytes before the gap test. A possible source optimization is to discharge exactly that coarse-zero case in the already-charged exact-integer branch, leaving cancellation, precision validation, work charging and all nonzero-shift behavior in their original order. This hypothesis still needs separate focused evidence and every frozen qualification gate.",
        ],
        "limits": "The normalization removes absolute instruction addresses, RIP displacements/comments, and branch target addresses while retaining instruction offsets and named external targets. Equality describes normalized instruction shape, not byte identity, target/data identity, executed traces, formal semantics or timing causation. This review cannot convert the v2021 failed comparison into a pass. It makes no source edit and performs no timing retry.",
    }
    result["artifacts_sha256"] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob("*")) if path.is_file()}
    (out / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"timed_processes": 0, "comparisons": comparisons,
                      "helper_sizes": {k: v["functions"][functions[2]]["size_bytes"] for k, v in results.items()}}))


if __name__ == "__main__":
    main()
