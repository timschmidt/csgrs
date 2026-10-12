"""Prepare, without executing, retention checks for the outlined cache selector."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_bound_merge_retention.py"
    output = E / "record_cache_state_merge_retention.py"
    provenance = E / "cache-state-merge-retention-preparation-v1999.json"
    patch = E / "cache-state-merge-retention-preparation-v1999.patch"
    assert all(not path.exists() for path in [output, provenance, patch])
    before = parent.read_text()
    after = before.replace("bound-merge", "cache-state-merge")
    after = after.replace("run_bound_merge_replays.py", "run_cache_state_replays.py")
    versions = {1840: 1926, 1849: 1935, 1857: 1943, 1858: 1944,
                1911: 1997, 1896: 1982, 1912: 1998, 1913: 1999, 1843: 1929}
    for old, new in versions.items():
        assert f"v{old}" in after
        after = after.replace(f"v{old}", f"v{new}")

    def replace(old, new, count=1):
        nonlocal after
        assert after.count(old) == count, (old, after.count(old))
        after = after.replace(old, new)

    replace("2cdd765b340ce6d0a331aeb25b2596639bc262bb18805cb7efeed27aa43143b9",
            "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59")
    replace('proof["fresh_rejected_mutants"] == 246', 'proof["fresh_rejected_mutants"] == 247')
    replace("246 rejected mutations", "247 rejected mutations", 2)
    replace("{1841: (1074, 1, 0), 1846: (5858, 3, 33),\n                      1847: (1059, 3, 0), 1848: (1074, 1, 0)}",
            "{1927: (1074, 1, 0), 1932: (5858, 3, 33),\n                      1933: (1059, 3, 0), 1934: (1074, 1, 0)}")
    replace("BoundInfo::merge", "BoundCache::merge", 2)
    replace('    assert proof["reused_rejected_mutants"] == 0\n',
            '    assert proof["reused_rejected_mutants"] == 0\n'
            '    for name, digest in proof["prior_experiments_sha256"].items():\n'
            '        assert sha(E / name) == digest\n'
            '    rejected = read(E / "bound-merge-cost-rejection-v1857.json")\n'
            '    assert not rejected["runtime_retained"]\n'
            '    for name, diagnostic in proof["compile_only_diagnostics"].items():\n'
            '        assert sha(E / name / "summary.json") == diagnostic["summary_sha256"]\n'
            '        assert diagnostic["timed_processes"] == 0\n')
    replace("The proof includes the actual production BoundCache::merge body.",
            "The proof includes the actual production BoundCache::merge body, including the uninitialized cache case. Ordinary Rust calls that same outlined selector.")
    replace("This extraction has no failed proof attempt; all earlier failed experiments remain immutable.",
            "The outlined extraction has no failed proof attempt. The earlier BoundInfo extraction failed its timing gate; its rejection, the unsupported Option::or probe and the compile-only variants remain immutable.")
    replace("The local cost baseline remains v1836. Older original-cap failures at v1638/v1710, the earlier retained integer-control tradeoff and the fixed pre-Verus acceptance baseline are not erased or reset by this successful comparison.",
            "The local cost baseline remains v1836. Older original-cap failures at v1638/v1710, the rejected BoundInfo extraction's v1857 timing regressions, the earlier retained integer-control tradeoff and the fixed pre-Verus acceptance baseline are not erased or reset by this successful comparison.")
    replace('        "Continue implementation-connected review of bound producers, independent real existence "\n',
            '        "Connect the actual cache-word decoder to the existing model, following cache-word-next-review-v1926.json. "\n'
            '        "Continue implementation-connected review of bound producers, independent real existence "\n')
    replace('        "status": "Retain the isolated proved compatible cached-bound merge; main admission remains open",',
            '        "status": "Retain the isolated proved outlined cache-state merge; main admission remains open",')
    replace('        "production_patch": proof["proposal_patch"], "production_patch_sha256": proof["proposal_patch_sha256"],\n',
            '        "production_patch": proof["proposal_patch"], "production_patch_sha256": proof["proposal_patch_sha256"],\n'
            '        "prior_experiments_sha256": proof["prior_experiments_sha256"],\n'
            '        "compile_only_diagnostics": proof["compile_only_diagnostics"],\n'
            '        "next_production_model_review": "cache-word-next-review-v1926.json",\n'
            '        "next_production_model_review_sha256": sha(E / "cache-word-next-review-v1926.json"),\n')
    replace('"21a45c3ed2b7aee9e5f98d8beaba184b4a8f37f6", "0e865eca8bb98f83137c66fe78d2b9793b1f6557",',
            '"dec6ce44ffc109de6f8f52d2ce253cec1cae5dbf", subprocess.check_output(["git", "log", "-1", "--format=%H", "--", str((E / "cache-state-merge-broad-milestone-v1935.json").relative_to(ROOT))], cwd=ROOT, text=True).strip(),')
    ast.parse(after)
    assert "BoundInfo::merge" not in after and "246" not in after
    assert "run_bound_merge_replays" not in after
    for old in versions:
        # The failed prior extraction remains explicitly cited as v1857.
        assert f"v{old}" not in after or old == 1857
    rejected = json.loads((E / "bound-merge-cost-rejection-v1857.json").read_text())
    assert rejected["runtime_retained"] is False
    output.write_text(after)
    patch.write_text("".join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                               fromfile=parent.name, tofile=output.name)))
    provenance.write_text(json.dumps({
        "status": "Prepared only; execution requires every fresh cost and replay gate to pass",
        "source_record_sha256": "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59",
        "predecessor": parent.name, "predecessor_sha256": sha(parent),
        "recorder": output.name, "recorder_sha256": sha(output),
        "patch": patch.name, "patch_sha256": sha(patch),
        "generator_sha256": sha(Path(__file__)),
        "conditional_retention_checkpoint": "typed-api-prototype-v1999-state.json",
        "retained_runtime_checkpoint": "typed-api-prototype-v1836-state.json",
        "changes": [
            "Bind to the outlined cache-state source, its 247 fresh negative mutations and all new phase identities.",
            "Keep every runtime, original-cap, receipt, fact-retention and conditional-proof requirement from the predecessor, including dynamic family-repeat counts.",
            "Validate and retain the earlier failed extraction and compile-only diagnostics; use the actual BoundCache selector and current evidence commits.",
            "Name the reviewed production decoder gap as the next proof task without claiming it resolved.",
        ],
        "limits": "Preparing this recorder neither admits the runtime nor changes any property/target status. No old helper, completed phase, source file or checkpoint-covered artifact is edited.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared": output.name, "executed": False}))


if __name__ == "__main__":
    main()
