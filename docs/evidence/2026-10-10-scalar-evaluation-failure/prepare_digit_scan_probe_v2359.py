"""Freeze a standalone generic digit-scan proof against the retained toolchain."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    source = E / "digit-scan-model-proposal-v2359.rs"
    parent = E / "probe_trailing_bit_support_v2346.py"
    runner = E / "probe_digit_scan_model_v2359.py"
    record = E / "digit-scan-probe-preparation-v2359.json"
    assert not runner.exists() and not record.exists()
    original = parent.read_text()
    tree = ast.parse(original)
    assignment = next(n for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "PROBE" for t in n.targets))
    lines = original.splitlines(True)
    code = "".join(lines[:assignment.lineno - 1])
    code += 'PROBE = (E / "digit-scan-model-proposal-v2359.rs").read_text()\n'
    code += "".join(lines[assignment.end_lineno:])
    replacements = [
        ('out = E / "trailing-bit-support-v2346"', 'out = E / "digit-scan-model-probe-v2359"'),
        ('typed-api-prototype-v2345-state.json', 'typed-api-prototype-v2358-state.json'),
        ('a75220d56a289d2e83e36a426fb6f1f04b808e684b0889e5d360924408d940b9',
         '5af0c6067447ffcf6f531b7b3cb206b83532225b99f6c8a1f66cbca7698bc991'),
        ('typed-api-prototype-v2343-approximation-metadata-final-verus', 'typed-api-prototype-v2352-trailing-metadata-verus'),
        ('Probe pinned standard-library bit contracts without changing the candidate.',
         'Probe generic digit-scan arithmetic without changing the candidate.'),
        ('Do the existing pinned u32/u64 trailing-ones specifications and actual primitive method contracts establish equality with trailing zeros of the complement?',
         'Do radix-parametric prefix identities and first matching digit premises establish whole-value odd factors for zero and one scans, including empty and all-maximal storage?'),
    ]
    for before, after in replacements:
        assert code.count(before) == 1, before
        code = code.replace(before, after)
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({"status": "Frozen before the generic arithmetic support probe",
        "preparation_sha256": sha(Path(__file__)), "parent_runner": parent.name,
        "parent_runner_sha256": sha(parent), "proposal": source.name, "proposal_sha256": sha(source),
        "runner": runner.name, "runner_sha256": sha(runner),
        "retained_checkpoint": "typed-api-prototype-v2358-state.json",
        "source_changed": False,
        "scope": "The generic prefix lemmas adapt the existing ghost limb-denotation arguments to natural digits and arbitrary power-of-two radix. The scan lemmas take explicit first-match and word-factor premises. No production body, BigUint storage, iterator or machine count is substituted or assumed verified; the model has not yet been integrated into the retained conditional target. Preserve every failed attempt separately."}, indent=2) + "\n")
    print(json.dumps({"frozen_generic_digit_probe": True, "candidate_source_changed": False}))


if __name__ == "__main__":
    main()
