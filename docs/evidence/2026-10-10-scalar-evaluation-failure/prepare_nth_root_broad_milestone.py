"""Prepare a runtime milestone with the actual proof reuse and kernel limits."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_cache_state_merge_broad_milestone.py"
    target = E / "record_nth_root_enclosure_broad_milestone.py"
    out = E / "nth-root-enclosure-broad-milestone-preparation-v2013.json"
    patch = E / "record_nth_root_enclosure_broad_milestone-v2013.patch"
    assert not any(path.exists() for path in [target, out, patch])
    original = parent.read_text()
    def version(match):
        value = int(match[1])
        return "v" + str({1924: 2003, 1926: 2004}.get(value,
            value + 78 if 1927 <= value <= 1998 else value))
    source = re.sub(r"v(\d+)\b", version, original)
    source = source.replace("cache-state-merge", "nth-root-enclosure")
    source = source.replace("cache_state_merge", "nth_root_enclosure").replace("cache_state", "nth_root")
    replacements = {
        "retaining the extraction": "retaining the combined proposal",
        "Source hashes match the fresh conditional proof exactly, so its 651 verified units, 247 rejected mutations, four passing guard baselines and ten acceptance tests still apply only to their declared scope. All thirteen whole-crate groups remain open.":
            "Source hashes match the fresh conditional verification (651 units) and ten fresh acceptance tests. The 247 rejected mutations and four passing guard baselines are reused from v1926 because all 44 proof/guard inputs and verifier identities are unchanged; zero mutations were rerun. The modified bigint kernel is outside Verus coverage. All thirteen whole-crate groups remain open.",
        "The runtime extraction is still a proposal.": "The combined kernel/selector change is still a proposal.",
        "The v2005-v2013 extraction checks pass on the exact v2003 proof source:":
            "The v2005-v2013 broad checks pass on the exact v2003 source snapshot:",
        "1074 all-feature": "1076 all-feature",
        "5858 consumer": "5860 consumer",
        "1059 default-feature": "1061 default-feature",
        "1074 debug": "1076 debug",
        "The nine runtime validation commands pass on the proved source.":
            "The nine runtime validation commands pass on the combined kernel/selector source.",
    }
    for before, after in replacements.items():
        assert source.count(before) == 1, before
        source = source.replace(before, after)
    anchor = '    assert validation["complete"] and len(validation["phases"]) == 9\n'
    assert source.count(anchor) == 1
    source = source.replace(anchor, anchor +
        '    assert proof["verified_units"] == 651 and proof["fresh_acceptance_tests"] == 10\n'
        '    assert proof["fresh_rejected_mutants"] == proof["fresh_passing_guard_baselines"] == 0\n'
        '    assert proof["reused_rejected_mutants"] == 247 and proof["reused_passing_guard_baselines"] == 4\n')
    before = ('    old = ("Next validate its runtime extraction against unchanged scalar/consumer tests, cost "\n'
              '           "controls and capped/numerical/public replays before deciding retention.")\n')
    after = ('    old = ("The one-root enclosure prototype passes focused tests and refreshed conditional verification. "\n'
             '           "Next run the nine broad commands, freeze the new cost/replay protocols, and evaluate the combined "\n'
             '           "proposal against v1836 under every unchanged full-size fixture and original cap. Preserve all "\n'
             '           "earlier failures and do not retain it before all gates pass.")\n')
    assert source.count(before) == 1
    source = source.replace(before, after)
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    out.write_text(json.dumps({
        "predecessor": parent.name, "predecessor_sha256": sha(parent),
        "recorder": target.name, "recorder_sha256": sha(target),
        "patch": patch.name, "patch_sha256": sha(patch), "preparer_sha256": sha(Path(__file__)),
        "scope": "Require all nine broad commands and their receipt audit, account for two new tests, and distinguish refreshed conditional verification from reused mutation guards and the unverified bigint kernel. Preserve every property/target status and leave cost/replay retention pending.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "runtime_validation_required_before_recording": True}))


if __name__ == "__main__":
    main()
