"""Fork the full interrupted-order audit for the decoder's baseline CPU failure."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_integer_coarse_zero_capped_failure.py"
    target = E / "record_bound_decoder_capped_failure.py"
    output = E / "bound-decoder-failure-review-preparation-v2247.json"
    patch = E / "bound-decoder-failure-review-preparation-v2247.patch"
    assert all(not path.exists() for path in [target, output, patch])
    original = parent.read_text()
    source = original.replace("integer-coarse-zero", "bound-decoder").replace("integer_coarse_zero", "bound_decoder")
    mapping = {2083: 2171, 2092: 2180, 2093: 2240, 2100: 2247, 2154: 2235, 2156: 2248}
    source = re.sub(r"v(\d+)\b", lambda m: "v" + str(mapping.get(int(m[1]), int(m[1]))), source)
    source = source.replace("bound-decoder-broad-milestone-v2180.json", "bound-decoder-binary-milestone-v2247.json")
    source = source.replace("bound-decoder-replay-protocol-v2101.json", "bound-decoder-measured-replay-protocol-v2247.json")
    source = source.replace("e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62",
                            "80bb17679a0891ff93eba8ed10b9f63c205f3ccb39e35dbb3e187e23bb0b7162")
    for old, new in [("880", "864"), ("881", "865"), ("1348", "1332"), ("1349", "1333"),
                     ("algebraic-26-baseline", "algebraic-18-baseline")]:
        source = source.replace(old, new)

    def replace(before, after, count=1):
        nonlocal source
        assert source.count(before) == count, (before, source.count(before))
        source = source.replace(before, after)

    replace('(52, 48, 1, 3)', '(52, 48, 0, 4)')
    replace('    assert partial[0]["successful_samples"] == {"baseline": 8, "candidate": 8}\n',
            '    assert not partial\n')
    replace('    assert [(row["family"], row["arguments"]) for row in increases] == [("integer", ["small_coarse", "200000"])]',
            '    assert [(row["family"], row["arguments"]) for row in increases] == [\n'
            '        ("integer", ["wide_gap", "10000"]), ("integer", ["wide_fine", "10000"]),\n'
            '        ("integer", ["wide_identity", "10000"]), ("default", ["structural_cancel", "500"])]')
    replace('"unstarted_paired_processes": 55', '"unstarted_paired_processes": 71')
    replace('"observed_partial_fixtures": 1, "unstarted_fixtures": 3',
            '"observed_partial_fixtures": 0, "fixtures_without_successful_samples": 4,\n'
            '        "failed_before_first_success_fixtures": 1, "wholly_unattempted_fixtures": 3')
    replace('All successful processes preserve non-timing output fields and original caps. Partial and unstarted fixtures remain incomplete. A completed wide_gap median improvement does not pass the comparison, and the small_coarse increase remains unresolved because the original run did not complete.',
            'All successful processes preserve non-timing output fields and original caps. Forty-eight fixtures are complete; the failing fixture has no successful paired sample and three further fixtures were never attempted. Four complete fixtures have increases above five percent; they remain unresolved because the original run did not complete. No timing observation passes the failed process gate.')
    start = source.index('    next_priority = (\n')
    end = source.index('    rejection = {\n', start)
    source = source[:start] + '''    next_priority = (
        "Restore the exact retained v1836 source after archiving the decoder's failed cost gate. "
        "Inspect the frozen baseline/candidate executable differences before choosing a next "
        "proof-compatible implementation experiment. Preserve the full-size algebraic CPU "
        "failure and every original fixture/cap; do not repeat unchanged timing runs until "
        "one passes. Producer/existence/atomic, resource and main/native/full-board work "
        "remain open."
    )
''' + source[end:]
    replace('Do not retain the combined integer/root/cache proposal after a baseline capped failure',
            'Do not retain the bound-decoder proposal after a baseline capped failure')
    start = source.index('        "passing_evidence": {')
    end = source.index('        "next_priority": next_priority,', start)
    source = source[:start] + '''        "passing_evidence": {"verified_units": 655, "listed_production_contracts": 98,
            "listed_model_theorems": 135, "fresh_rejected_mutations": 245,
            "reused_rejected_mutations": 0, "fresh_passing_baselines": 4,
            "fresh_acceptance_tests": 10, "broad_commands": 9},
        "proof_record": "bound-decoder-proof-validation-v2171.json",
        "proof_record_sha256": sha(E / "bound-decoder-proof-validation-v2171.json"),
        "limits": review["limits"] + ["The archived conditional proof verifies the actual decoder and five connected field lemmas, but the implementation remains unretained. Producer truth, existence, atomic/caller ordering, all thirteen whole-crate groups, the fixed pre-Verus acceptance baseline and main/native/full-board admission remain open."],
''' + source[end:]
    start = source.index('    update = (\n')
    end = source.index('    p4["remaining"] += " " + update\n', start)
    source = source[:start] + '''    update = (
        "The v2240-v2247 decoder cost comparison stops at algebraic-18-baseline: the "
        "retained v1836 executable hits SIGXCPU under the original 192 MiB, CPU 2/3-second "
        "and four-second wall limits, with no intermediate output. All 468 standalone and "
        "864 paired processes before the failure pass, as do 865 preludes. Forty-eight "
        "fixtures have all nine pairs; the failed fixture has no successful paired sample "
        "and three later fixtures are unattempted. Seventy-one paired processes remain "
        "unstarted. Four complete fixtures exceed the five-percent repeat threshold, but "
        "the incomplete original run prevents repeats. The 655-unit conditional proof, "
        "245 fresh negative guards and nine broad checks remain archived; no replay or "
        "runtime retention follows. The candidate stays v1836 and every property/target "
        "status and main/native/full-board requirement remains unchanged."
    )
''' + source[end:]
    replace('    old = "Next run the frozen cost controls, capped/numerical/public replays and receipt audits before deciding retention."',
            '    old = broad["next_priority"]')
    replace('"complete_fixtures": 48, "partial_fixtures": 1, "unstarted_fixtures": 3,\n'
            '        "unstarted_processes": 55',
            '"complete_fixtures": 48, "partial_fixtures": 0, "fixtures_without_successful_samples": 4,\n'
            '        "wholly_unattempted_fixtures": 3, "unstarted_processes": 71')
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    output.write_text(json.dumps({"status": "Prepared exact order/receipt audit for the observed failed decoder gate",
        "predecessor": parent.name, "predecessor_sha256": sha(parent), "recorder": target.name,
        "recorder_sha256": sha(target), "patch": patch.name, "patch_sha256": sha(patch),
        "scope": "Retain the original pair-order, executable, receipt, cap, CPU-prelude and source guards. Update the actual interruption point and distinguish a fixture attempted without success from fixtures never attempted. Preserve all unresolved timing increases and prohibit retries/replays/retention."}, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "executed": False}))


if __name__ == "__main__":
    main()
