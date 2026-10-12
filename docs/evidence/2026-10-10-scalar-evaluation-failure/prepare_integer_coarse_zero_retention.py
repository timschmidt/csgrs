"""Prepare retention of the combined candidate, conditional on every frozen gate."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_nth_root_enclosure_retention.py"
    target = E / "record_integer_coarse_zero_retention.py"
    out = E / "integer-coarse-zero-retention-preparation-v2156.json"
    patch = E / "integer-coarse-zero-retention-preparation-v2156.patch"
    assert not any(path.exists() for path in [target, out, patch])
    original = parent.read_text()
    source = re.sub(r"v(\d+)\b", lambda match:
        "v" + str(int(match[1]) + 79) if 2004 <= int(match[1]) <= 2077 else match[0], original)
    source = source.replace("nth-root-enclosure", "integer-coarse-zero").replace("run_nth_root_replays.py", "run_integer_zero_replays.py")
    source = source.replace("f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3",
                            "e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62")

    def replace(before, after, count=1):
        nonlocal source
        assert source.count(before) == count, (before, source.count(before))
        source = source.replace(before, after)

    replace('assert proof["focused_tests"] == 3', 'assert proof["focused_tests"] == 5 and proof["root_control_tests"] == 3')
    replace('["focused_phase", "fresh_verification_phase", "fresh_acceptance_phase"]',
            '["focused_phase", "root_control_phase", "fresh_verification_phase", "fresh_acceptance_phase"]')
    replace('{2005: (1076, 1, 0), 2010: (5860, 3, 33),\n                      2011: (1061, 3, 0), 2012: (1076, 1, 0)}',
            '{2084: (1076, 1, 0), 2089: (5860, 3, 33),\n                      2090: (1061, 3, 0), 2091: (1076, 1, 0)}')
    replace('and len(changed) == 8', 'and len(changed) == 9')
    begin = source.index('    allocation_path = E / "integer-coarse-zero-allocation-study-v2100/summary.json"\n')
    end = source.index('    limits = [\n', begin)
    block = '''    allocation_path = E / "integer-coarse-zero-allocation-review-v2092.json"
    allocation = read(allocation_path)
    assert allocation["source_record_sha256"] == SOURCE_HASH
    assert allocation["public_cases"] == allocation["raw_cases"] == 32
    assert allocation["fresh_budgeted_shortcut_cases"] == allocation["raw_zero_allocation_controls"] == 16
    assert allocation["budgeted_shortcut_allocation_observation"] == {"calls": 1, "bytes": 40, "largest": 40}
    assert allocation["maximum_observed_bytes"] <= 128 and allocation["fresh_root_allocation_measurements"] == 0
    assert sha(E / allocation["focused_phase"] / "stdout.log") == allocation["focused_stdout_sha256"]
    assert sha(E / allocation["focused_phase"] / "stderr.log") == allocation["focused_stderr_sha256"]
    root_study_path = E / allocation["reused_copied_root_study"]
    assert sha(root_study_path) == allocation["reused_copied_root_study_sha256"]
    root_study = read(root_study_path)
    assert root_study["counts_stable"] and root_study["exact_endpoints_equal"]
    assert root_study["fixtures"] == 84 and root_study["kernel_calls"] == 504 and not root_study["increases"]
    for name, digest in root_study["artifacts_sha256"].items():
        assert sha(root_study_path.parent / name) == digest
    assert sha(SOURCE / "crates/hyperreal/src/computable/approximation/exp_sqrt.rs") == allocation["root_kernel_sha256"]
    assert sha(Path(allocation["unchanged_num_dependency"])) == allocation["num_dependency_sha256"]

'''
    source = source[:begin] + block + source[end:]
    replace('The bigint root kernel is outside Verus coverage. Its exact endpoints have focused inequality/differential tests and unchanged public MPFR cases.',
            'The changed integer branch and bigint root kernel are outside Verus coverage. Five unchanged integer tests cover exact signed rounding, precision, cancellation, work accounting and allocations; three unchanged root controls check endpoint inequalities, differential equality and public MPFR cases.')
    replace('The 84-case copied-kernel allocation study records stable counts and identical endpoints across 504 helper calls. It measures allocation requests only, not production call-site behavior, live/peak memory, query work or time. All cases and its limitations remain recorded separately.',
            'The fresh allocation audit records 32 public budgeted and 32 raw integer cases, all within the unchanged 128-byte ceiling. The 16 public coarse-zero cases record one 40-byte allocation with budget setup included; 16 corresponding raw cases allocate nothing. The historical 84-case copied-root study is reused only for its unchanged helper and num dependency, with no fresh root allocation run. Neither study measures live/peak memory or whole-board behavior. The failed zero-allocation audit assumption and its correction remain recorded.')
    replace('The local cost baseline remains v1836. The outlined selector alone failed its original CPU cap at v1943.',
            'The local cost baseline remains v1836. The predecessor root/selector combination failed the wide_gap timing control at v2021; it remains rejected. The outlined selector alone failed its original CPU cap at v1943.')
    replace('Retain the isolated one-root enclosure and conditionally proved outlined cache merge; main admission remains open',
            'Retain the isolated integer shortcut, one-root enclosure and conditionally proved outlined cache merge; main admission remains open')
    replace('"reused_passing_guard_baselines", "focused_tests"]',
            '"reused_passing_guard_baselines", "focused_tests", "root_control_tests", "new_tests_added"]')
    replace('Keep the one-root integer enclosure with identical tested endpoints, together with the same pure BoundCache::merge body included by Verus.',
            'Keep the charged integer coarse-zero branch, the one-root enclosure with identical tested endpoints, and the same pure BoundCache::merge body included by Verus.')
    replace('The kernel change reduces duplicate root computation; it has finite numerical and allocation evidence, not a formal kernel proof.',
            'The integer branch bypasses the scaling helper only after the same precision checks and work charge, under the helper\'s same strict zero predicate. The root change removes duplicate root computation. Both have finite numerical/allocation evidence and remain outside formal runtime coverage.')
    replace('"The v2156 isolated checkpoint retains the one-root enclosure and outlined cached-bound merge "',
            '"The v2156 isolated checkpoint retains the integer coarse-zero branch, one-root enclosure and outlined cached-bound merge "')
    replace('"baseline remain explicit. The bigint kernel is outside Verus; its focused endpoint checks and "\n'
            '        "84-case copied-kernel allocation study remain separate finite evidence. This changes only the isolated retained candidate. "',
            '"baseline remain explicit. The integer branch and bigint kernel remain outside Verus. Five integer "\n'
            '        "tests, three root controls and 64 fresh public/raw allocation cases pass their unchanged "\n'
            '        "contracts. The public coarse-zero call records one 40-byte allocation with budget setup included. The "\n'
            '        "earlier 84-case copied-root study is reused only for its unchanged helper/dependency. "\n'
            '        "This changes only the isolated retained candidate. "')
    replace('"85a24c007460551c9dee5746b68a192cb569b045",',
            '"85a24c007460551c9dee5746b68a192cb569b045", "1c658dee658fb9395b8f9b11f0fd3012fbca513c",')
    replace('        "Continue implementation-connected review of bound producers, independent real existence "',
            '        "Connect the charged integer shortcut to its bit-length producer contracts. "\n'
            '        "Continue implementation-connected review of bound producers, independent real existence "')
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    out.write_text(json.dumps({
        "status": "Prepared only; execution requires all frozen cost/replay gates and their audits to pass",
        "source_record_sha256": "e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62",
        "predecessor": parent.name, "predecessor_sha256": sha(parent),
        "recorder": target.name, "recorder_sha256": sha(target),
        "patch": patch.name, "patch_sha256": sha(patch), "preparer_sha256": sha(Path(__file__)),
        "conditional_retention_checkpoint": "typed-api-prototype-v2156-state.json",
        "retained_runtime_checkpoint": "typed-api-prototype-v1836-state.json",
        "scope": "Preserve every frozen runtime, cost, cap, replay and fact-retention check. Account for nine source changes, five existing integer tests and three existing root controls. Audit fresh public/raw allocation observations, explicitly retain budget-scope allocations and distinguish the reused copied-root study. Preserve all prior failures and every property/target status; preparation neither retains nor admits the runtime.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "executed": False}))


if __name__ == "__main__":
    main()
