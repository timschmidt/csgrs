"""Prepare unchanged replay-protocol generation for the outlined merge source."""
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    old = E / "prepare_bound_merge_replays.py"
    new = E / "prepare_cache_state_merge_replays.py"
    patch = E / "cache-state-replay-preparer-v1944.patch"
    record = E / "cache-state-replay-preparer-v1944.json"
    assert not any(path.exists() for path in [new, patch, record])
    original = old.read_text()
    source = re.sub(r"v(\d+)\b", lambda match:
        "v" + str(int(match[1]) + 86) if 1838 <= int(match[1]) <= 1912 else match[0], original)
    source = source.replace("bound-merge", "cache-state-merge").replace("_bound_merge_", "_cache_state_")
    for before, after in [("range(1858, 1897)", "range(1944, 1983)"),
                          ("range(1897, 1910)", "range(1983, 1996)"),
                          ("[1910, 1911]", "[1996, 1997]")]:
        assert source.count(before) == 1, before
        source = source.replace(before, after, 1)
    source = source.replace("Frozen before runtime builds and replay execution",
        "Frozen before the nine runtime qualification commands and replay execution; earlier compile-only diagnostics remain separate")
    compile(source, str(new), "exec")
    new.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), source.splitlines(keepends=True),
                    fromfile=old.name, tofile=new.name, n=0)))
    record.write_text(json.dumps({"predecessor": old.name, "predecessor_sha256": sha(old),
        "prepared_generator": new.name, "prepared_generator_sha256": sha(new),
        "fork_patch": patch.name, "fork_patch_sha256": sha(patch),
        "status": "Prepared only; run after the complete fresh proof record exists",
        "scope": "Keep retained v1836 as the replay baseline, with all 39 capped phases, 4129 numerical cases, 648 public schedules, original per-corpus limits, deterministic receipt comparisons and concurrent fact audits. Only candidate phase/runner identities and freeze-status wording change."}, indent=2) + "\n")
    print(json.dumps({"prepared_generator": new.name, "protocol_executed": False}))


if __name__ == "__main__":
    main()
