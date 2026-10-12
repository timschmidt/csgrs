"""Prepare a cost milestone that can execute only after all frozen costs pass."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_integer_coarse_zero_cost_milestone.py"
    target = E / "record_bound_decoder_cost_milestone.py"
    record = E / "bound-decoder-cost-milestone-preparation-v2247.json"
    patch = E / "bound-decoder-cost-milestone-preparation-v2247.patch"
    assert all(not path.exists() for path in [target, record, patch])
    original = parent.read_text()
    source = original.replace("integer-coarse-zero", "bound-decoder").replace("v2100", "v2247")
    source = source.replace("bound-decoder-broad-milestone-v2092.json", "bound-decoder-binary-milestone-v2247.json")
    source = source.replace("bound-decoder-replay-protocol-v2101.json", "bound-decoder-measured-replay-protocol-v2247.json")
    source = source.replace("run_integer_zero_replays.py", "run_bound_decoder_replays_v2.py")
    source = source.replace("v2093-v2247", "v2240-v2247")
    source = source.replace("combined integer/kernel/selector source", "proved bound-decoder source")
    source = source.replace("The combined proposal", "The decoder proposal")
    before = ('    old = ("Next run the frozen cost controls, capped/numerical/public replays and receipt audits "\n'
              '           "before deciding retention.")\n')
    assert source.count(before) == 1
    source = source.replace(before, '    old = broad["next_priority"]\n')
    before = ('    after["next_actions"][0] = after["next_actions"][0].replace(old,\n'
              '        "The frozen local cost controls and prescribed repeat rule pass. Next run all "\n'
              '        "capped/numerical/public replays and receipt/fact audits before deciding retention.", 1)\n')
    after = ('    record["next_priority"] = ("The frozen local decoder cost controls and prescribed "\n'
             '        "repeat rule pass. Next run all unchanged capped/numerical/public replays and "\n'
             '        "receipt/fact audits before deciding retention. Preserve all earlier failures, "\n'
             '        "the fixed pre-Verus baseline and open producer/existence/atomic and "\n'
             '        "main/native/full-board requirements.")\n'
             '    after["next_actions"][0] = after["next_actions"][0].replace(old, record["next_priority"], 1)\n')
    assert source.count(before) == 1
    source = source.replace(before, after)
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    record.write_text(json.dumps({"status": "Prepared only; all cost execution and audit gates remain mandatory",
        "predecessor": parent.name, "predecessor_sha256": sha(parent),
        "recorder": target.name, "recorder_sha256": sha(target),
        "patch": patch.name, "patch_sha256": sha(patch),
        "scope": "Only candidate/protocol/ledger identities change. Preserve every cost count, cap, non-timing receipt, prescribed repeat and failure check. The helper neither runs measurements nor records a pass before their completion."}, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "executed": False}))


if __name__ == "__main__":
    main()
