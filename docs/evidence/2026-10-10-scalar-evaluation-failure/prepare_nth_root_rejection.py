"""Fork the repeat-failure recorder, preserving this proposal's exact evidence."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_bound_merge_rejection.py"
    target = E / "record_nth_root_enclosure_rejection.py"
    out = E / "nth-root-enclosure-rejection-preparation-v2021.json"
    patch = E / "nth-root-enclosure-rejection-preparation-v2021.patch"
    assert not any(path.exists() for path in [target, out, patch])
    original = parent.read_text()
    versions = {1840: 2004, 1843: 2007, 1849: 2013, 1850: 2014, 1857: 2021, 1911: 2075, 1913: 2077}
    source = re.sub(r"v(\d+)\b", lambda match: "v" + str(versions.get(int(match[1]), int(match[1]))), original)
    source = source.replace("bound-merge", "nth-root-enclosure")
    source = source.replace("2484", "2160").replace("2016", "1692").replace("1080", "756")

    def replace(before, after, count=1):
        nonlocal source
        assert source.count(before) == count, (before, source.count(before))
        source = source.replace(before, after)

    replace('sum(map(len, repeat["cases"].values())) == 20', 'sum(map(len, repeat["cases"].values())) == 14')
    replace('"family_repeat_fixture_count": 20', '"family_repeat_fixture_count": 14')
    replace('("atan", ("cached_reciprocal_one", "100000")),\n'
            '        ("atan", ("cached_reciprocal_many", "100000"))',
            '("integer", ("wide_gap", "10000"))')
    replace('        "Investigate the two persistent cached-reciprocal atan increases using the preserved "\n'
            '        "binaries and unchanged full-size controls. Any revised extraction needs a new frozen "\n',
            '        "Investigate the prescribed-repeat wide-integer-gap increase using the preserved binaries "\n'
            '        "and unchanged controls. Inspect generated code and the exact-integer fast path before "\n'
            '        "proposing a justified source change. Any revised proposal needs a new frozen "\n')
    replace('Reject the isolated nth-root-enclosure extraction: two prescribed repeat controls exceed the frozen timing threshold',
            'Reject the isolated combined root/cache proposal: a prescribed repeat control exceeds the frozen timing threshold')
    replace('"passing_evidence": {"verified_units": 651, "fresh_rejected_mutants": 246,\n'
            '            "fresh_passing_guard_baselines": 4, "fresh_acceptance_tests": 10,',
            '"passing_evidence": {"verified_units": 651, "fresh_rejected_mutants": 0,\n'
            '            "fresh_passing_guard_baselines": 0, "reused_rejected_mutants": 247,\n'
            '            "reused_passing_guard_baselines": 4, "fresh_acceptance_tests": 10, "focused_tests": 3,')
    replace('The gate rejects this candidate based on the frozen local comparison. Timing does not establish a code-level cause or a uniform regression; integer-family initial increases did not persist in the repeat.',
            'The gate rejects this candidate based on the frozen family-repeat rule. The three initially flagged cases fall below the threshold on repeat, but the unchanged wide_gap control rises 11.175507824761223 percent in that repeat (its initial change was -6.268224844107484 percent). The predeclared rule checks every fixture in each affected family, not only the initially flagged cases. Timing establishes neither a code-level cause nor a uniform regression.')
    replace('The seven probe executables grow by 320-408 bytes, recorded separately from timing. These sizes do not establish whole-board effects.',
            'The seven probe executables shrink by 864-1008 bytes, recorded separately from timing. The copied-kernel allocation study reduces requests in all 84 cases, and the full algebraic fixture has lower observed whole-process wall time. These separate results do not override the failed timing gate or establish whole-board effects.')
    replace('The conditional production proof and nine runtime checks remain valid evidence for their source.',
            'The refreshed conditional cache proof, reused mutation guards and nine runtime checks remain valid evidence for their declared source and scope. The bigint root helper is outside Verus coverage.')
    before = '''    update = (
        "The v2014-v2021 cost gate rejects the nth-root-enclosure extraction. All 2160 cost processes "
        "and 1692 separate CPU preludes pass their original caps and preserve non-timing "
        "observations, but the predeclared family-wide 27-pair repeat leaves cached reciprocal "
        "atan increases of 9.1308 and 7.2137 percent. Integer-family initial increases do not "
        "persist. Probe executables grow 320-408 bytes. All original and repeated observations "
        "remain immutable; the 651-unit proof and nine passing runtime commands do not override "
        "the failed cost gate. Replays and the prepared retention helper were not run. The "
        "retained runtime remains v1836 and all property/target statuses, older failures/tradeoff, "
        "fixed pre-Verus baseline and main/native/full-board obligations remain unchanged."
    )'''
    after = '''    update = (
        "The v2014-v2021 cost gate rejects the combined one-root enclosure and outlined cache "
        "proposal. All 2160 cost processes and 1692 CPU preludes pass their original limits and "
        "preserve every non-timing output. All three initial increases fall below five percent "
        "on the prescribed 27-pair repeat, but another unchanged integer control (wide_gap) "
        "increases 11.1755 percent; its initial change was -6.2682 percent. Every affected-family "
        "control remains part of the frozen gate. All 84 copied-kernel allocation cases use "
        "fewer requests and bytes with identical endpoints; seven probes shrink 864-1008 bytes. "
        "The original algebraic 4/4/0/false/64 fixture passes all paired processes and has "
        "48.0550 percent lower median whole-process wall time, while its timed query loop "
        "changes +1.1863 percent. These observations do not isolate a causal contribution or "
        "override the failed timing gate. All initial/repeated observations and earlier "
        "failures remain immutable. The 651-unit conditional proof, 247 reused mutation "
        "guards and nine broad checks retain their scope; the bigint helper is not formally "
        "verified. Replays and retention were not executed. Retained runtime stays v1836, "
        "with all property/target statuses, fixed pre-Verus baseline and main/native/full-board "
        "obligations unchanged."
    )'''
    replace(before, after)
    replace('The frozen cost comparison rejects the nth-root-enclosure extraction; retain v1836.',
            'The frozen cost comparison rejects the combined root/cache proposal; retain v1836.')
    anchor = '    next_priority = (\n'
    extra = '''    allocation_path = E / "nth-root-enclosure-allocation-study-v2021/summary.json"
    allocation = read(allocation_path)
    resource_path = E / "nth-root-enclosure-resource-review-v2021.json"
    resource = read(resource_path)
    assert allocation["source_record_sha256"] == resource["source_record_sha256"] == expected
    assert allocation["guards_unchanged"] and allocation["counts_stable"] and allocation["exact_endpoints_equal"]
    assert allocation["fixtures"] == 84 and allocation["kernel_calls"] == 504 and not allocation["increases"]
    assert resource["cost_controller_sha256"] == sha(controller_path) and not resource["cost_controller_passed"]
    assert resource["allocation_study_sha256"] == sha(allocation_path)
    for name, digest in allocation["artifacts_sha256"].items():
        assert sha(allocation_path.parent / name) == digest
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(main_guard) == 1167 and all(sha(ROOT / name) == digest for name, digest in main_guard.items())
    assert proof["fresh_rejected_mutants"] == proof["fresh_passing_guard_baselines"] == 0
    assert proof["reused_rejected_mutants"] == 247 and proof["reused_passing_guard_baselines"] == 4
    assert sha(E / proof["prior_failure"]) == proof["prior_failure_sha256"]
    assert not read(E / proof["prior_failure"])["runtime_retained"]
'''
    replace(anchor, extra + anchor)
    replace('        "persistent_increases": persistent,',
            '        "resource_review": resource_path.name, "resource_review_sha256": sha(resource_path),\n'
            '        "allocation_study": str(allocation_path.relative_to(E)), "allocation_study_sha256": sha(allocation_path),\n'
            '        "main_guarded_files_verified": len(main_guard),\n'
            '        "persistent_increases": persistent,')
    replace('for name in [initial_path.name, costs_path.name, out.name]:',
            'for name in [initial_path.name, costs_path.name, resource_path.name, str(allocation_path.relative_to(E)), out.name]:')
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    out.write_text(json.dumps({
        "predecessor": parent.name, "predecessor_sha256": sha(parent),
        "recorder": target.name, "recorder_sha256": sha(target),
        "patch": patch.name, "patch_sha256": sha(patch), "preparer_sha256": sha(Path(__file__)),
        "scope": "Record the failed frozen timing gate, all successful capped processes, the new wide_gap increase, refreshed/reused proof scope, separate allocation/wall/size observations, unchanged main files and unstarted replays. Keep retained v1836 and every older failure.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared_rejection_recorder": target.name}))


if __name__ == "__main__":
    main()
