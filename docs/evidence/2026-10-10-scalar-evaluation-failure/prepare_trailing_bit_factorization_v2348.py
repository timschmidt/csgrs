"""Correct pinned lemma names while preserving the v2347 diagnostic."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    old_source = E / "trailing-bit-factorization-proposal-v2347.rs"
    old_runner = E / "probe_trailing_bit_factorization_v2347.py"
    source = E / "trailing-bit-factorization-proposal-v2348.rs"
    runner = E / "probe_trailing_bit_factorization_v2348.py"
    record = E / "trailing-bit-factorization-preparation-v2348.json"
    assert not source.exists() and not runner.exists() and not record.exists()
    failed = E / "trailing-bit-factorization-v2347"
    result = json.loads((failed / "result.json").read_text())
    proof = json.loads((failed / "stdout.json").read_text())["verification-results"]
    assert result["exit_code"] == 1 and proof["verified"] == 0 and proof["encountered-error"]
    text = old_source.read_text()
    assert text.count("lemma2_to32();") == 1 and text.count("lemma_div_self(unit as int);") == 2
    text = text.replace("lemma2_to32();", "lemma2_to64();")
    text = text.replace("lemma_div_self(unit as int);", "lemma_div_by_self(unit as int);")
    source.write_text(text)
    code = old_runner.read_text().replace(old_source.name, source.name)
    assert code.count('out = E / "trailing-bit-factorization-v2347"') == 1
    code = code.replace('out = E / "trailing-bit-factorization-v2347"',
                        'out = E / "trailing-bit-factorization-v2348"')
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({"status": "Frozen before corrected standalone probe",
        "preparation_sha256": sha(Path(__file__)), "previous_proposal_sha256": sha(old_source),
        "previous_runner_sha256": sha(old_runner), "failed_result_sha256": sha(failed / "result.json"),
        "failed_stderr_sha256": sha(failed / "stderr.log"), "proposal": source.name,
        "proposal_sha256": sha(source), "runner": runner.name, "runner_sha256": sha(runner),
        "changes": ["Use the existing lemma2_to64 constant-power lemma for both word widths.",
                    "Use the pinned library's lemma_div_by_self name in both all-ones cases."],
        "scope": "No theorem statement, algorithm, fixture, runtime source or admission status changes. The failed name-resolution probe remains immutable; no mathematical verification was completed by that attempt."}, indent=2) + "\n")
    print(json.dumps({"corrected_lemma_names": 3, "source_unchanged": True}))


if __name__ == "__main__":
    main()
