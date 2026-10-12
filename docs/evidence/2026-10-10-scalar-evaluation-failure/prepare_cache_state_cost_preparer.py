"""Prepare cost-protocol generation while the full proof guard suite runs."""
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    old = E / "prepare_bound_merge_costs.py"
    new = E / "prepare_cache_state_merge_costs.py"
    patch = E / "cache-state-cost-preparer-v1936.patch"
    record = E / "cache-state-cost-preparer-v1936.json"
    assert not any(path.exists() for path in [new, patch, record])
    original = old.read_text()
    source = re.sub(r"v(18(?:3[89]|[45][0-9]))\b", lambda match: "v" + str(int(match[1]) + 86), original)
    source = source.replace("bound-merge", "cache-state-merge").replace("bound_merge", "cache_state_merge")
    source = source.replace("range(1850, 1857)", "range(1936, 1943)")
    source = source.replace("2cdd765b340ce6d0a331aeb25b2596639bc262bb18805cb7efeed27aa43143b9",
                            "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59")
    source = source.replace("Frozen before building or measuring the cache-state-merge runtime extraction",
        "Frozen before building or measuring the standalone cost binaries; earlier compile-only diagnostics remain separate and untimed")
    compile(source, str(new), "exec")
    new.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), source.splitlines(keepends=True),
                    fromfile=old.name, tofile=new.name, n=0)))
    record.write_text(json.dumps({"predecessor": old.name, "predecessor_sha256": sha(old),
        "prepared_generator": new.name, "prepared_generator_sha256": sha(new),
        "fork_patch": patch.name, "fork_patch_sha256": sha(patch),
        "status": "Prepared only; run after the complete fresh proof record exists",
        "scope": "Update only candidate identities and the freeze-status wording. Preserve all original retained-v1836 binaries, fixtures, runner code, process counts and resource limits; earlier diagnostics did not execute the probe."}, indent=2) + "\n")
    print(json.dumps({"prepared_generator": new.name, "protocol_executed": False}))


if __name__ == "__main__":
    main()
