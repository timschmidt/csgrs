"""Fork frozen cost and replay generation for the combined kernel proposal."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "nth-root-enclosure-protocol-generators-v2014.json"
    assert not out.exists()
    forks = {}
    for old, new in [
        ("prepare_cache_state_merge_costs.py", "prepare_nth_root_enclosure_costs.py"),
        ("prepare_cache_state_merge_replays.py", "prepare_nth_root_enclosure_replays.py"),
    ]:
        parent, target = E / old, E / new
        patch = E / (new.removesuffix(".py") + "-v2014.patch")
        assert not target.exists() and not patch.exists()
        original = parent.read_text()
        def version(match):
            number = int(match[1])
            return "v" + str({1924: 2003, 1926: 2004}.get(number,
                number + 78 if 1927 <= number <= 1998 else number))
        source = re.sub(r"v(\d+)\b", version, original)
        source = source.replace("cache-state-merge", "nth-root-enclosure")
        source = source.replace("cache_state_merge", "nth_root_enclosure").replace("cache_state", "nth_root")
        source = source.replace("555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59",
                                "f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3")
        source = source.replace("conditional nth-root-enclosure extraction", "combined nth-root kernel and outlined-cache proposal")
        if new.endswith("costs.py"):
            assert source.count("range(1936, 1943)") == 1
            source = source.replace("range(1936, 1943)", "range(2014, 2021)")
            source = source.replace("Frozen before building or measuring the standalone cost binaries; earlier compile-only diagnostics remain separate and untimed",
                "Frozen before building or measuring cost binaries for this combined proposal; all prior failed comparisons and diagnostics remain separate")
        else:
            for before, after in [("range(1944, 1983)", "range(2022, 2061)"),
                                  ("range(1983, 1996)", "range(2061, 2074)"),
                                  ("[1996, 1997]", "[2074, 2075]")]:
                assert source.count(before) == 1
                source = source.replace(before, after)
            source = source.replace("Frozen before the nine runtime qualification commands and replay execution; earlier compile-only diagnostics remain separate",
                "Frozen before capped, numerical and public replays; broad runtime validation is in progress")
            anchor = '        "domain-predicate-cost-orchestration-v1779.json": "nth-root-enclosure-cost-orchestration-v2020.json",\n'
            assert source.count(anchor) == 1
            old_check = '    assert costs["completed"] and costs["source_record_sha256"] == expected\n'
            new_check = old_check + (
                '    cost_review = read(E / "nth-root-enclosure-cost-review-v2021.json")\n'
                '    controller = read(E / "nth-root-enclosure-cost-validation-v2021.json")\n'
                '    assert controller["complete"] and cost_review["local_timing_and_process_gates_passed"]\n'
                '    assert cost_review["source_record_sha256"] == controller["source_record_sha256"] == expected\n')
            source = source.replace(anchor, anchor + f"        {old_check!r}: {new_check!r},\n")
            source = source.replace("Only frozen identity literals change in the replay, comparison and fact-audit helpers.",
                "Candidate identities change, and replay startup additionally requires the final cost audit and controller to pass.")
        ast.parse(source)
        target.write_text(source)
        patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                                   fromfile=old, tofile=new, n=0)))
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(parent),
            "generator_sha256": sha(target), "patch": patch.name, "patch_sha256": sha(patch)}
    out.write_text(json.dumps({
        "status": "Generators prepared before any new cost or replay execution",
        "source_record_sha256": "f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3",
        "generators": forks, "preparer_sha256": sha(Path(__file__)),
        "limits": "Keep retained v1836, every original full-size fixture and cap, all cost/replay comparisons and the predeclared repeat strategy. The replay runner also checks the final cost gate before starting. The v1943 cap failure and every earlier failure remain unchanged; no new measurements are performed by this preparation.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared_generators": list(forks), "measurements_started": False}))


if __name__ == "__main__":
    main()
