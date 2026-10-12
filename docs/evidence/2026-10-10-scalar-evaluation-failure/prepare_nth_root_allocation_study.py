"""Freeze a copied-kernel allocation study, separately from public cost gates."""
import hashlib
import json
from pathlib import Path
import textwrap

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())


def main():
    out = E / "nth-root-enclosure-allocation-protocol-v2021.json"
    probe = E / "nth_root_allocation_probe_v2021.rs"
    assert not out.exists() and not probe.exists()
    name = "crates/hyperreal/src/computable/approximation/exp_sqrt.rs"
    archived = E / "nth-root-enclosure-baseline-v2001/inputs" / name
    baseline = read(E / "nth-root-enclosure-baseline-v2001/baseline.json")
    assert sha(archived) == baseline["archived_sources_sha256"][name]
    old = archived.read_text()
    start = "        let lower_power = approximation.magnitude() - BigUint::one();\n"
    end = "\n        let rounding = BigUint::one()"
    assert old.count(start) == 1
    block = start + old.split(start, 1)[1].split(end, 1)[0]
    before = "fn baseline(approximation: &BigInt, degree: u32) -> (BigUint, BigUint) {\n"
    before += textwrap.indent(textwrap.dedent(block), "    ") + "\n    (lower, upper)\n}"
    current = (SOURCE / name).read_text()
    marker = "fn integer_nth_root_bounds("
    assert current.count(marker) == 1
    after = marker + current.split(marker, 1)[1].split("\n}\n", 1)[0] + "\n}"
    template = E / "nth_root_allocation_probe_template.rs"
    text = template.read_text()
    assert text.count("// INSERT_BASELINE") == text.count("// INSERT_CANDIDATE") == 1
    text = text.replace("// INSERT_BASELINE", before).replace("// INSERT_CANDIDATE", after)
    probe.write_text(text)
    proof = read(E / "nth-root-enclosure-proof-validation-v2004.json")
    assert sha(SOURCE / name) == proof["changed_sources_sha256"][name]
    out.write_text(json.dumps({
        "status": "Frozen before building or executing the copied-kernel allocation probe",
        "source_record": proof["source_record"], "source_record_sha256": proof["source_record_sha256"],
        "candidate_file": name, "candidate_file_sha256": sha(SOURCE / name),
        "baseline_file": str(archived.relative_to(E)), "baseline_file_sha256": sha(archived),
        "baseline_block_sha256": hashlib.sha256(block.encode()).hexdigest(),
        "candidate_function_sha256": hashlib.sha256(after.encode()).hexdigest(),
        "template": template.name, "template_sha256": sha(template),
        "probe": probe.name, "probe_sha256": sha(probe), "preparer_sha256": sha(Path(__file__)),
        "degrees": list(range(3, 10)), "root_bit_widths": [16, 128, 1024, "ceil(65568 / degree)"],
        "root_low_bits": 7, "perfect_power_deltas": [-1, 0, 1], "repetitions": 3,
        "fixtures": 84, "paired_observations": 252, "kernel_calls": 504,
        "address_space_mib": 192, "cpu_seconds": [2, 3], "wall_seconds": 4,
        "measurement_fields": ["successful_allocation_calls_including_realloc", "total_requested_bytes_including_full_realloc_size", "successful_reallocation_calls"],
        "acceptance": "Require equal exact endpoints and stable allocation counts across all three repetitions. Report every case and any increases; no timing or whole-process memory inference.",
        "limits": "This copies the exact candidate helper and the archived prior endpoint block (indentation and a tuple-return wrapper only), linking the same num dependency as the production release library. It measures Rust allocation requests during the helper call only. Input construction, output checks, formatting and output destruction are outside the counter window. It does not measure peak/live memory, production call-site behavior, public query allocations, consumed evaluation work or execution time. It cannot pass or replace any original public cost or replay gate, including earlier SIGXCPU failures.",
    }, indent=2) + "\n")
    print(json.dumps({"frozen_allocation_fixtures": 84, "kernel_calls": 504, "measurements_started": False}))


if __name__ == "__main__":
    main()
