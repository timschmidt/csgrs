"""Fork the unchanged nine-command runtime validation for the integer shortcut."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "integer-coarse-zero-validation-preparation-v2092.json"
    assert not out.exists()
    proof_path = E / "integer-coarse-zero-proof-validation-v2083.json"
    proof = read(proof_path)
    assert proof["focused_tests"] == 5 and proof["root_control_tests"] == 3
    assert proof["verified_units"] == 651 and proof["fresh_acceptance_tests"] == 10
    assert proof["reused_rejected_mutants"] == 247 and proof["fresh_rejected_mutants"] == 0
    forks = {}
    for old, new in [
        ("validate_nth_root_enclosure_candidate.py", "validate_integer_coarse_zero_candidate.py"),
        ("record_nth_root_enclosure_validation.py", "record_integer_coarse_zero_validation.py"),
    ]:
        parent, target = E / old, E / new
        patch = E / (new.removesuffix(".py") + "-v2092.patch")
        assert not target.exists() and not patch.exists()
        original = parent.read_text()
        source = re.sub(r"v(\d+)\b", lambda match:
            "v" + str(int(match[1]) + 79) if 2003 <= int(match[1]) <= 2013 else match[0], original)
        source = source.replace("nth-root-enclosure", "integer-coarse-zero")
        source = source.replace("f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3", proof["source_record_sha256"])
        source = source.replace("combined kernel/selector proposal", "combined integer/kernel/selector proposal")
        if new.startswith("validate_"):
            assert source.count("range(2005, 2014)") == 1
            source = source.replace("range(2005, 2014)", "range(2084, 2093)")
        ast.parse(source)
        target.write_text(source)
        patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                                   fromfile=old, tofile=new, n=0)))
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(parent),
                      "fork_sha256": sha(target), "patch": patch.name, "patch_sha256": sha(patch)}
    out.write_text(json.dumps({
        "status": "Prepared before running the nine broad commands on the charged integer shortcut",
        "source_record_sha256": proof["source_record_sha256"], "proof_record": proof_path.name,
        "proof_record_sha256": sha(proof_path), "runner_forks": forks, "preparer_sha256": sha(Path(__file__)),
        "baseline_commands": "domain-predicate-candidate-validation-v1772.json", "phases": list(range(2084, 2093)),
        "expected_tests": {"scalar": [1076, 1, 0], "stack": [5860, 3, 33],
                           "default": [1061, 3, 0], "debug": [1076, 1, 0]},
        "limits": "No command, fixture, test, test count, warning allowance or ignored case changes. Retain all 108 included formatting paths and both strict Clippy configurations. Source identity and phase names change; broad success does not pass the separate frozen cost/replay gates.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared": list(forks), "commands_changed": False, "tests_added": 0}))


if __name__ == "__main__":
    main()
