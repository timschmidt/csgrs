"""Fork the byte-exact restoration controller for the five decoder proposal files."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "restore_retained_scalar_v2159.py"
    target = E / "restore_retained_decoder_v2248.py"
    output = E / "decoder-restoration-preparation-v2248.json"
    patch = E / "decoder-restoration-preparation-v2248.patch"
    assert all(not p.exists() for p in [target, output, patch])
    original = parent.read_text()
    versions = dict(zip(range(2159, 2166), range(2248, 2255)))
    source = re.sub(r"v(\d+)\b", lambda m: "v" + str(versions.get(int(m[1]), int(m[1]))), original)
    source = source.replace("scalar-retained-restoration", "decoder-retained-restoration")
    source = source.replace("integer-coarse-zero-cost-rejection-v2100.json", "bound-decoder-cost-rejection-v2247.json")
    source = source.replace("integer-coarse-zero-baseline-profile-review-v2158.json", "bound-decoder-binary-comparison-v2181/summary.json")
    before = ('    archives = [E / n / "inputs" for n in ["bound-merge-baseline-v1837",\n'
              '                "nth-root-enclosure-baseline-v2001", "integer-coarse-zero-baseline-v2079"]]\n')
    assert source.count(before) == 1
    source = source.replace(before, '    archives = [E / "bound-decoder-baseline-v2168/inputs"]\n')
    for old, new in [(2160, 2249), (2161, 2250), (2162, 2251), (2163, 2252)]:
        assert source.count(f"({old}, ") == 1
        source = source.replace(f"({old}, ", f"({new}, ")
    source = source.replace('len(names) == 9', 'len(names) == 5')
    source = source.replace('"restored_files": 9', '"restored_files": 5')
    source = source.replace("Restore only these nine isolated files", "Restore only these five isolated files")
    source = source.replace("The unretained merge's additional proofs do not describe restored production code.",
                            "The unretained decoder's additional proofs remain archived and do not describe restored production code.")
    source = source.replace("Continue the actual decoder/model connection from the retained source after restoration verification.",
                            "After restoration verification, inspect the archived executable differences before selecting a proof-compatible next implementation experiment. Preserve the failed original-cap gate; do not retry unchanged timings until one passes.")
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    output.write_text(json.dumps({"status": "Prepared restoration only; restore/validate modes remain separate",
        "predecessor": parent.name, "predecessor_sha256": sha(parent), "helper": target.name,
        "helper_sha256": sha(target), "patch": patch.name, "patch_sha256": sha(patch),
        "scope": "Restore five failed-candidate files from the frozen v2168 baseline, verify all 24746 source and 1167 main files, and run the same six restoration commands. Existing proofs, tests and failed costs remain immutable; no runtime or property completion is inferred."}, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "source_edits_started": False}))


if __name__ == "__main__":
    main()
