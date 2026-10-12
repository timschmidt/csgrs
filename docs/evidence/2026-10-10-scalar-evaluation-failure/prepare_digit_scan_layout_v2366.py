"""Freeze final formatting of the proof entrypoint and readable guard tuples."""
import ast
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import pprint
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
OUT = E / "digit-scan-layout-v2366"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not OUT.exists()
    record = E / "typed-api-prototype-v2365-digit-scan-verus/source-hashes.json.gz"
    before = json.loads(gzip.decompress(record.read_bytes()))
    assert sha(record) == "7b1277a3edce614e4ee8c4c2ba37fb9330876e0b37c79633921e95051e7db0e5"
    assert all(sha(SOURCE / n) == h for n, h in before.items())
    names = ["crates/hyperreal/verification/lib.rs", "crates/hyperreal/verification/test_digit_scan_rejections.py"]
    OUT.mkdir()
    originals = {n: (SOURCE / n).read_text() for n in names}
    for n in names:
        for part in ["before", "after"]:
            p = OUT / part / n
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(originals[n])
    subprocess.run(["rustfmt", "--edition", "2024", "--config-path", str(SOURCE / "crates/.rustfmt.toml"),
        "--config", "skip_children=true", str(OUT / "after" / names[0])], cwd=ROOT, check=True)
    text = originals[names[1]]
    parsed = ast.parse(text)
    node = next(n for n in parsed.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in n.targets))
    mutations = ast.literal_eval(node.value)
    lines = text.splitlines(True)
    formatted = "".join(lines[:node.lineno - 1]) + "MUTATIONS = [\n"
    for row in mutations:
        formatted += "    " + pprint.pformat(row, width=96).replace("\n", "\n    ") + ",\n"
    formatted += "]\n" + "".join(lines[node.end_lineno:])
    assert ast.dump(ast.parse(formatted)) == ast.dump(parsed)
    (OUT / "after" / names[1]).write_text(formatted)
    patch = []
    hashes = {n: sha(OUT / "after" / n) for n in names}
    for n in names:
        after = (OUT / "after" / n).read_text()
        assert before[n] != hashes[n]
        if n.endswith(".rs"):
            old_declarations = [s for s in originals[n].splitlines() if s.startswith("mod ")]
            new_declarations = [s for s in after.splitlines() if s.startswith("mod ")]
            assert sorted(old_declarations) == sorted(new_declarations)
        patch.extend(difflib.unified_diff(originals[n].splitlines(True), after.splitlines(True),
            fromfile="a/" + n, tofile="b/" + n))
    (OUT / "layout.patch").write_text("".join(patch))
    (OUT / "source-hashes.json.gz").write_bytes(gzip.compress(
        (json.dumps(before | hashes, sort_keys=True) + "\n").encode(), mtime=0))
    protocol = {"status": "Frozen before applying final entrypoint and guard layout",
        "runner_sha256": sha(Path(__file__)), "previous_source_record_sha256": sha(record),
        "before_sha256": {n: before[n] for n in names}, "after_sha256": hashes,
        "patch_sha256": sha(OUT / "layout.patch"),
        "source_record_sha256": sha(OUT / "source-hashes.json.gz"),
        "change": "Sort proof module declarations using rustfmt and format the nineteen mutation tuples over readable lines. Python AST equality is checked; no proof, mutation, production body or old evidence is changed. Rerun the full proof and fresh guards on this final source.",
        "failed_launcher_phase_preserved": "typed-api-prototype-v2364-digit-scan-verus",
        "launcher_failure_reason": "The wrapper runs in SOURCE, so the first relative proof-runner path was unavailable. The v2365 run used the existing absolute runner path and passed 716 units. This is not a verifier failure or a pass credited to v2364."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    subprocess.run(["git", "apply", "--check", str(OUT / "layout.patch")], cwd=SOURCE, check=True)
    for n in names:
        (SOURCE / n).write_bytes((OUT / "after" / n).read_bytes())
    assert all(sha(SOURCE / n) == h for n, h in (before | hashes).items())
    (OUT / "application.json").write_text(json.dumps({"source_record_sha256": protocol["source_record_sha256"],
        "verification_files_formatted": 2, "runtime_source_files_changed": 0,
        "python_ast_unchanged": True, "full_proof_and_guard_reruns_required": True}, indent=2) + "\n")
    print(json.dumps({"formatted_files": 2, "source_record_sha256": protocol["source_record_sha256"]}))


if __name__ == "__main__":
    main()
