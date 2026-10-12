"""Prepare the unchanged nine runtime commands and scoped recorders for the outlined source."""
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "cache-state-merge-validation-preparation-v1935.json"
    assert not out.exists()
    pairs = [
        ("validate_bound_merge_candidate.py", "validate_cache_state_merge_candidate.py"),
        ("record_bound_merge_validation.py", "record_cache_state_merge_validation.py"),
        ("record_bound_merge_broad_milestone.py", "record_cache_state_merge_broad_milestone.py"),
    ]
    forks = {}
    for old, new in pairs:
        predecessor = E / old
        target = E / new
        patch = E / (new.removesuffix(".py") + "-v1935.patch")
        assert not target.exists() and not patch.exists()
        original = predecessor.read_text()
        source = re.sub(r"v(18(?:3[89]|[45][0-9]))\b", lambda match: "v" + str(int(match[1]) + 86), original)
        source = source.replace("bound-merge", "cache-state-merge")
        source = source.replace("_bound_merge_", "_cache_state_")
        source = source.replace("2cdd765b340ce6d0a331aeb25b2596639bc262bb18805cb7efeed27aa43143b9",
                                "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59")
        source = source.replace("range(1841, 1850)", "range(1927, 1936)")
        source = source.replace("246 rejected mutations", "247 rejected mutations")
        compile(source, str(target), "exec")
        target.write_text(source)
        patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), source.splitlines(keepends=True),
                        fromfile=old, tofile=new, n=0)))
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(predecessor),
            "runner_sha256": sha(target), "fork_patch": patch.name, "fork_patch_sha256": sha(patch)}
    data = {"status": "Prepared only; the fresh proof and complete mutation/acceptance checks must pass first",
        "source_record_sha256": "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59",
        "runner_forks": forks, "phases": list(range(1927, 1936)),
        "scope": "The runtime runner still reads byte-identical commands from retained v1836's nine checks. Preserve counts, ignored names, inherited warnings, strict Clippy, formatting, all source guards and all admission limits. Only source/phase/proof identities and the added invalid-cache mutation count change."}
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"prepared_runners": list(forks), "runtime_commands": 9, "executed": False}))


if __name__ == "__main__":
    main()
