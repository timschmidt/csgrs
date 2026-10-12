"""Prepare the restoration audit with decoder-specific scope and preserved failures."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_scalar_retained_restoration.py"
    target = E / "record_decoder_retained_restoration.py"
    record = E / "decoder-restoration-recorder-preparation-v2255.json"
    patch = E / "decoder-restoration-recorder-preparation-v2255.patch"
    assert all(not p.exists() for p in [target, record, patch])
    original = parent.read_text()
    versions = dict(zip(range(2159, 2167), range(2248, 2256)))
    source = re.sub(r"v(\d+)\b", lambda m: "v" + str(versions.get(int(m[1]), int(m[1]))), original)
    source = source.replace("scalar-retained-restoration", "decoder-retained-restoration")
    source = source.replace('len(protocol["restorations"]) == 9', 'len(protocol["restorations"]) == 5')
    source = source.replace('"restored_files": 9', '"restored_files": 5')
    source = source.replace('"integer-coarse-zero-cost-rejection-v2100.json"]:',
                            '"integer-coarse-zero-cost-rejection-v2100.json", "bound-decoder-cost-rejection-v2247.json"]:')
    start = source.index('    next_priority = (\n')
    end = source.index('    record = {', start)
    source = source[:start] + '''    next_priority = (
        "Inspect the archived decoder executable differences before selecting a proof-compatible "
        "implementation experiment from the exact retained v1836 source. Keep the proved but "
        "unretained decoder and every failed original-cap comparison archived. Preserve all "
        "u64 inputs, signs, signed exponents, absent magnitudes and unrelated bits, without "
        "assumed method specifications. Producer validity, independent existence, atomic/caller "
        "ordering, resource work and native/full-board admission remain open; the full "
        "39-property/14-target/crate-future-work plan remains in scope."
    )
''' + source[end:]
    start = source.index('    update = (\n')
    end = source.index('    p4["remaining"] += " " + update\n', start)
    source = source[:start] + '''    update = (
        "The v2248-v2255 restoration returns all 24746 isolated source files to the exact "
        "v1836 checkpoint. Five rejected decoder files and their patch remain archived. "
        "Six fresh commands pass: 1074 scalar units plus one child, release build, workspace "
        "and 108 included-input format checks, 645 conditional proof units and ten acceptance "
        "tests. The 226 negative mutations and four passing baselines are reused under "
        "identical source, dependency and verifier identities. The rejected decoder's "
        "655-unit proof and 245 fresh negative guards remain historical evidence for its "
        "unretained source. Every cost rejection, including the v2247 baseline SIGXCPU, "
        "remains failed. Prior v1836 broad/replay/cost evidence is reused for the identical "
        "retained bytes. Main and all property/target statuses are unchanged; producer, "
        "existence, atomic/caller, resource and native/full-board obligations remain open."
    )
''' + source[end:]
    before = 'read(E / "integer-coarse-zero-baseline-profile-review-v2158.json")["next_priority"]'
    assert source.count(before) == 1
    source = source.replace(before, 'read(E / "bound-decoder-cost-rejection-v2247.json")["next_priority"]')
    source = source.replace('import subprocess\n', 'import subprocess\nimport sys\nsys.dont_write_bytecode = True\n')
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    record.write_text(json.dumps({"status": "Prepared audit; execution still validates every restoration result",
        "predecessor": parent.name, "predecessor_sha256": sha(parent), "recorder": target.name,
        "recorder_sha256": sha(target), "patch": patch.name, "patch_sha256": sha(patch),
        "scope": "Retain the six fresh command, whole-source/main identity, conditional proof, acceptance and reused mutation/dependency checks. Update five-file restoration scope and preserve the decoder's failed cost gate explicitly."}, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "executed": False}))


if __name__ == "__main__":
    main()
