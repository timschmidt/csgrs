"""Fork the existing broad-result parser with the restored baseline's test counts."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    old = E / "record_integer_coarse_zero_validation.py"
    new = E / "record_bound_decoder_validation.py"
    output = E / "bound-decoder-validation-recorder-preparation-v2180.json"
    patch = E / "bound-decoder-validation-recorder-preparation-v2180.patch"
    assert not any(p.exists() for p in [new, output, patch])
    source = old.read_text().replace("integer-coarse-zero", "bound-decoder")
    for before, after in {2082: 2169, **{v: v + 88 for v in range(2084, 2093)}}.items():
        source = source.replace("v" + str(before), "v" + str(after))
    for before, after in [(1076, 1074), (5860, 5858), (1061, 1059)]:
        source = source.replace(str(before), str(after))
    source = source.replace("Default feature counts exclude the serde-gated new module. ",
                            "All Rust test bodies and counts match the retained v1836 baseline. ")
    ast.parse(source)
    new.write_text(source)
    patch.write_text("".join(difflib.unified_diff(old.read_text().splitlines(True), source.splitlines(True),
                                                fromfile=old.name, tofile=new.name)))
    output.write_text(json.dumps({"predecessor": old.name, "predecessor_sha256": sha(old),
        "fork": new.name, "fork_sha256": sha(new), "patch": patch.name, "patch_sha256": sha(patch),
        "generator_sha256": sha(Path(__file__)),
        "changes": "Only phase names, baseline-matching counts and the scope sentence change. The existing test parser, ignored-case check, inherited warning comparison and zero-warning Clippy requirement remain unchanged.",
        "expected_counts": {"scalar": 1074, "stack": 5858, "default": 1059, "debug": 1074,
                            "children": [1, 3, 3, 1], "ignored_stack": 33, "inherited_warnings": 14,
                            "included_format_paths": 108}, "executed": False}, indent=2) + "\n")
    print(json.dumps({"prepared": new.name, "executed": False}))


if __name__ == "__main__":
    main()
