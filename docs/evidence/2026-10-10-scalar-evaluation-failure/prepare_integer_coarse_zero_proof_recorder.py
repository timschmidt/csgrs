"""Prepare a source-bound record for the integer shortcut and unchanged proof scope."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_nth_root_enclosure_proof.py"
    target = E / "record_integer_coarse_zero_proof.py"
    out = E / "integer-coarse-zero-proof-recorder-preparation-v2083.json"
    patch = E / "integer-coarse-zero-proof-recorder-preparation-v2083.patch"
    assert not any(path.exists() for path in [target, out, patch])
    original = parent.read_text()
    versions = {2001: 2079, 2002: 2080, 2003: 2082, 2004: 2083}
    source = re.sub(r"v(\d+)\b", lambda match: "v" + str(versions.get(int(match[1]), int(match[1]))), original)
    source = source.replace("nth-root-enclosure", "integer-coarse-zero")

    def replace(before, after, count=1):
        nonlocal source
        assert source.count(before) == count, (before, source.count(before))
        source = source.replace(before, after)

    replace('prior_path = E / "cache-state-merge-proof-validation-v1926.json"',
            'prior_path = E / "nth-root-enclosure-proof-validation-v2004.json"')
    replace('f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3',
            'e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62')
    replace('    for phase in [focused, proof, acceptance]:',
            '    root_controls = E / "typed-api-prototype-v2081-integer-coarse-zero-root-controls"\n'
            '    for phase in [focused, root_controls, proof, acceptance]:')
    replace('    assert "test result: ok. 3 passed; 0 failed; 0 ignored;" in (focused / "stdout.log").read_text()',
            '    assert "test result: ok. 5 passed; 0 failed; 0 ignored;" in (focused / "stdout.log").read_text()\n'
            '    assert "test result: ok. 3 passed; 0 failed; 0 ignored;" in (root_controls / "stdout.log").read_text()')
    replace('and len(changes) == 8', 'and len(changes) == 9')
    replace('    for name, digest in baseline["inputs_sha256"].items():\n        assert sha(E / name) == digest\n',
            '    for key in ["prior_rejection", "static_review", "prior_proof_record"]:\n'
            '        assert sha(E / baseline[key]) == baseline[key + "_sha256"]\n')
    replace('rejection = E / prior["fresh_rejection_phase"]', 'rejection = E / prior["reused_rejection_phase"]')
    begin = source.index('    kernel_name = "crates/hyperreal/src/computable/approximation/exp_sqrt.rs"\n')
    end = source.index('    diff = []\n', begin)
    check = '''    query_name = "crates/hyperreal/src/computable/node/approximation_queries.rs"
    old_query = (baseline_dir / "inputs" / query_name).read_text()
    new_query = (SOURCE / query_name).read_text()
    inserted = (
        "                // A gap beyond the integer's bit length rounds to zero for\\n"
        "                // either sign. Resolve the scaling helper's same shortcut\\n"
        "                // here, after the precision checks and work charge.\\n"
        "                if p > 0 && u64::from(p as u32) > value.bits() {\\n"
        "                    return Ok(BigInt::zero());\\n"
        "                }\\n")
    assert new_query.count(inserted) == 1 and new_query.replace(inserted, "", 1) == old_query
    prefix = new_query.split(inserted, 1)[0]
    assert prefix.endswith('                if !charge_evaluation_work() {\\n                    return Err(Problem::Exhausted);\\n                }\\n')
    assert old_query.count('let p = checked_approximation_precision(p)?;') == 1
'''
    source = source[:begin] + check + source[end:]
    replace('        original_file = (baseline_dir if name in root_changes else E / "bound-merge-baseline-v1837") / "inputs" / name\n'
            '        assert sha(original_file) == original[name]',
            '        candidates = [directory / "inputs" / name for directory in\n'
            '            [baseline_dir, E / "nth-root-enclosure-baseline-v2001", E / "bound-merge-baseline-v1837"]]\n'
            '        matches = [path for path in candidates if path.is_file() and sha(path) == original[name]]\n'
            '        assert matches, name\n'
            '        original_file = matches[0]')
    replace('One-root enclosure proposal passes focused tests and the unchanged conditional proof target; runtime gates pending',
            'Charged integer coarse-zero proposal passes existing focused tests and unchanged conditional verification; runtime gates pending')
    replace('"changed_since_rejected_selector": sorted(root_changes)', '"changed_since_rejected_proposal": sorted(root_changes)')
    replace('"focused_phase": focused.name, "focused_tests": 3,\n'
            '        "new_test_cases": {"small_integer_enclosures": 7161, "wide_perfect_power_neighbors": 1120},',
            '"focused_phase": focused.name, "focused_tests": 5,\n'
            '        "root_control_phase": root_controls.name, "root_control_tests": 3, "new_tests_added": 0,')
    replace('"reused_rejection_phase": rejection.name, "reused_proof_record": prior_path.name,\n'
            '        "reused_proof_record_sha256": sha(prior_path),',
            '"reused_rejection_phase": rejection.name, "reused_proof_record": prior["reused_proof_record"],\n'
            '        "reused_proof_record_sha256": prior["reused_proof_record_sha256"],\n'
            '        "prior_proposal_proof_record": prior_path.name, "prior_proposal_proof_record_sha256": sha(prior_path),')
    replace('cache-state-merge-cost-rejection-v1943.json', 'nth-root-enclosure-cost-rejection-v2021.json', 2)
    replace('The direct kernel derives floor_root(A - 1) and ceil_root(A + 1) from one floor_root(A + 1) and its power. The prior domain/range/precision/small-sample guards, extreme-precision fallback and final rounding are byte-identical. Two new tests check exact endpoint inequalities and differential endpoint equality; the existing MPFR oracle test is unchanged.',
            'The exact Int branch returns zero for p > 0 and u64(p) > value.bits() after the existing precision checks and successful evaluation-work charge. This is the called scaling helper\'s identical strict predicate and result. All other evaluator bytes, five existing integer tests, root code/tests and proof/guard inputs remain unchanged.')
    replace('The bigint nth-root kernel and its new helper are outside the Verus target. The 651 verified units refresh the existing 98 contracts and 135 model theorems; they do not formally prove this kernel change or complete any of the thirteen open groups.',
            'The changed integer evaluator branch and bigint nth-root helper are outside Verus coverage. The 651 verified units refresh the existing 98 contracts and 135 model theorems; they do not formally prove either runtime change or complete any open group.')
    replace('The combined proposal includes the unretained outlined selector. The v1943 CPU-limit failure, v1857 timing rejection, earlier original-cap failures, integer-control tradeoff and fixed baseline remain explicit. New runtime, cost and replay gates are still required against retained v1836.',
            'The combined proposal includes the unretained outlined selector and one-root kernel. The v2021 wide_gap timing rejection, v1943 CPU-limit failure, v1857 timing rejection, earlier failures/tradeoff and fixed baseline remain unchanged. Fresh broad, cost and replay qualification is required against retained v1836.')
    replace('Exact integer endpoint equality and the existing MPFR cases provide focused finite evidence; all original full-size corpora remain required. No timing, allocation or size improvement is claimed from the source change alone.',
            'Five unchanged integer tests cover signed rounding, million-bit allocation controls, work allowances/retries, retained signs, precision limits and cancellation precedence. Three unchanged root controls pass. This is finite evidence; no performance or size improvement is inferred from the new branch alone. The earlier copied-root allocation study retains only its original extracted-helper scope.')
    begin = source.index('    update = (\n')
    end = source.index('    p4["remaining"] += " " + update\n', begin)
    source = source[:begin] + '''    update = (
        "The v2079-v2083 isolated proposal handles the scaling helper's strict coarse-zero case "
        "directly in the exact-integer evaluator, after the original precision and work checks. "
        "Five unchanged integer tests and three unchanged root controls pass. The other evaluator "
        "bytes, root kernel and test sources are unchanged. Refreshed conditional verification "
        "reports 651 units with zero errors, and ten acceptance tests pass. The 247 negative "
        "guards and four positive baselines are reused under unchanged proof/guard/dependency/tool "
        "identities; the integer branch and bigint root kernel remain outside formal coverage. "
        "The v2021 timing rejection remains failed, and retained runtime stays v1836 pending "
        "fresh broad commands, all original-cap cost controls and replays."
    )
''' + source[end:]
    replace('old = read(E / "cache-state-algebraic-stage-review-v2000.json")["next_priority"]',
            'old = read(E / "nth-root-enclosure-cost-rejection-v2021.json")["next_priority"]')
    replace('The one-root enclosure prototype passes focused tests and refreshed conditional verification.',
            'The charged integer coarse-zero prototype passes existing focused tests and refreshed conditional verification.')
    replace('print(json.dumps({"focused_tests": 3,', 'print(json.dumps({"focused_tests": 5, "root_control_tests": 3,')
    replace('"changed_sources_since_retained": 8', '"changed_sources_since_retained": 9')
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    out.write_text(json.dumps({
        "predecessor": parent.name, "predecessor_sha256": sha(parent),
        "recorder": target.name, "recorder_sha256": sha(target),
        "patch": patch.name, "patch_sha256": sha(patch), "preparer_sha256": sha(Path(__file__)),
        "scope": "Bind the one changed integer branch and unchanged integer/root controls to fresh conditional verification. Audit unchanged proof/guard/tool identities before reusing the v1926 negative controls. Preserve the failed predecessor and all retention gates; change no property/target status.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared": target.name}))


if __name__ == "__main__":
    main()
