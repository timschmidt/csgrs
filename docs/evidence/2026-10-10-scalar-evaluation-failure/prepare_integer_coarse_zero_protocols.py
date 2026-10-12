"""Freeze unchanged cost/replay gates for the charged integer shortcut proposal."""
import ast
import copy
import difflib
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
EXPECTED = "e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62"


def transform(text):
    text = re.sub(r"v(\d+)\b", lambda match:
        "v" + str(int(match[1]) + 79) if 2003 <= int(match[1]) <= 2077 else match[0], text)
    text = text.replace("nth-root-enclosure", "integer-coarse-zero")
    text = text.replace("nth_root_enclosure", "integer_coarse_zero").replace("nth_root", "integer_zero")
    text = text.replace("f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3", EXPECTED)
    text = text.replace("combined nth-root kernel and outlined-cache proposal", "combined integer shortcut, root kernel and outlined-cache proposal")
    text = text.replace("combined kernel/selector", "combined integer/kernel/selector")
    return text


def main():
    out = E / "integer-coarse-zero-protocol-preparation-v2100.json"
    assert not out.exists()
    assert read(E / "integer-coarse-zero-proof-validation-v2083.json")["source_record_sha256"] == EXPECTED
    assert not (E / "integer-coarse-zero-cost-orchestration-v2099.json").exists()
    assert not (E / "integer-coarse-zero-preconditioned-cost-v2100").exists()
    forks = {}

    def fork(old, new, adjust=None):
        parent, target = E / old, E / new
        patch = E / (new.removesuffix(".py") + "-v2100.patch")
        assert not target.exists() and not patch.exists()
        original = parent.read_text()
        source = transform(original)
        if adjust:
            source = adjust(source)
        ast.parse(source)
        target.write_text(source)
        patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                                   fromfile=old, tofile=new, n=0)))
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(parent),
                      "fork_sha256": sha(target), "patch": patch.name, "patch_sha256": sha(patch)}

    def cost_ranges(source):
        assert source.count("range(2014, 2021)") == 1
        return source.replace("range(2014, 2021)", "range(2093, 2100)")

    def replay_ranges(source):
        for before, after in [("range(2022, 2061)", "range(2101, 2140)"),
                              ("range(2061, 2074)", "range(2140, 2153)"),
                              ("[2074, 2075]", "[2153, 2154]")]:
            assert source.count(before) == 1
            source = source.replace(before, after)
        return source

    fork("prepare_nth_root_enclosure_costs.py", "prepare_integer_coarse_zero_costs.py", cost_ranges)
    fork("prepare_nth_root_enclosure_replays.py", "prepare_integer_coarse_zero_replays.py", replay_ranges)
    for helper in ["prepare_integer_coarse_zero_costs.py", "prepare_integer_coarse_zero_replays.py"]:
        subprocess.run([sys.executable, str(E / helper)], cwd=ROOT, check=True)
    for old in ["record_nth_root_enclosure_initial_costs.py", "prepare_nth_root_enclosure_cost_followup.py",
                "record_nth_root_enclosure_costs.py", "run_nth_root_enclosure_cost_validation.py",
                "record_nth_root_enclosure_cost_milestone.py"]:
        fork(old, old.replace("nth_root_enclosure", "integer_coarse_zero"))
    protocol_path = E / "integer-coarse-zero-cost-protocol-v2093.json"
    assert read(protocol_path)["source_record_sha256"] == EXPECTED
    old_strategy_path = E / "nth-root-enclosure-cost-followup-strategy-v2021.json"
    original = read(old_strategy_path)
    strategy = copy.deepcopy(original)
    strategy.update(source_record_sha256=EXPECTED, cost_protocol=protocol_path.name,
                    cost_protocol_sha256=sha(protocol_path))
    assert all(strategy[key] == original[key] for key in original.keys() -
               {"source_record_sha256", "cost_protocol", "cost_protocol_sha256"})
    assert sum(map(len, strategy["family_fixtures"].values())) == 52 and strategy["repetitions"] == 27
    strategy_path = E / "integer-coarse-zero-cost-followup-strategy-v2100.json"
    assert not strategy_path.exists()
    strategy_path.write_text(json.dumps(strategy, indent=2) + "\n")

    def broad_scope(source):
        before = "The one-root enclosure prototype passes focused tests and refreshed conditional verification. "
        after = "The charged integer coarse-zero prototype passes existing focused tests and refreshed conditional verification. "
        assert source.count(before) == 1
        source = source.replace(before, after)
        before = "The modified bigint kernel is outside Verus coverage."
        assert source.count(before) == 1
        source = source.replace(before, "The modified integer branch and bigint root kernel are outside Verus coverage.")
        anchor = '    assert proof["verified_units"] == 651 and proof["fresh_acceptance_tests"] == 10\n'
        assert source.count(anchor) == 1
        source = source.replace(anchor, anchor + '    assert proof["focused_tests"] == 5 and proof["root_control_tests"] == 3\n')
        return source

    fork("record_nth_root_enclosure_broad_milestone.py", "record_integer_coarse_zero_broad_milestone.py", broad_scope)
    replay_path = E / "integer-coarse-zero-replay-protocol-v2101.json"
    replay = read(replay_path)
    assert replay["source_record_sha256"] == EXPECTED
    assert len(replay["caps"]) == 39 and len(replay["numerical_replays"]) == 13 and len(replay["public_replays"]) == 2
    out.write_text(json.dumps({
        "status": "Frozen before candidate cost and replay execution; broad commands are in progress",
        "source_record_sha256": EXPECTED, "runner_forks": forks, "preparer_sha256": sha(Path(__file__)),
        "cost_protocol": protocol_path.name, "cost_protocol_sha256": sha(protocol_path),
        "repeat_strategy": strategy_path.name, "repeat_strategy_sha256": sha(strategy_path),
        "repeat_strategy_predecessor": old_strategy_path.name, "repeat_strategy_predecessor_sha256": sha(old_strategy_path),
        "replay_protocol": replay_path.name, "replay_protocol_sha256": sha(replay_path),
        "scope": "Keep retained v1836, all 52 full-size cost fixtures, original 192 MiB/CPU 2-3-second/four-second wall limits, nine initial pairs and all-family 27-pair repeats. Keep all 39 caps, 4129 numerical cases, 648 public schedules and every receipt/fact audit; replay startup still requires the final cost gate to pass. No workload, threshold, assertion, precision or time/work budget is weakened. Preserve all earlier failures, including the v2021 integer timing rejection.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared_protocols": [protocol_path.name, replay_path.name],
                      "frozen_repeat_fixtures": 52, "measurements_started": False}))


if __name__ == "__main__":
    main()
