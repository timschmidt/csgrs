"""Attempt selected metadata proofs through the real num-bigint crate root."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREVIEW = E / "biguint-metadata-candidate-v2413"
OUT = E / "biguint-crate-export-v2442"
WORK = ROOT / "target/biguint-crate-export-v2442"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists() and not WORK.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    preview = files(PREVIEW)
    assert all(checkpoint["artifacts_sha256"][str((PREVIEW / n).relative_to(E))] == h
               for n, h in preview.items())
    reports = (E / checkpoint["source_record"]).parent / "proof-reports"
    dependencies = read(reports / "dependencies.json")
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    tool = ROOT / "crates/hyperreal/target/verus" / pin["version"] / "verus-x86-linux"
    tooling = read(reports / "tooling.json")["tool_sha256"]
    assert all(sha(tool / n) == h for n, h in tooling.items())
    OUT.mkdir()
    vendor = WORK / "vendor/num-bigint"
    shutil.copytree(PREVIEW / "vendor/num-bigint", vendor)
    library = vendor / "src/lib.rs"
    old = library.read_text()
    marker = "#[macro_use]\nextern crate alloc;"
    assert old.count(marker) == 1
    modules = ["verified", "magnitude_model", "bit_length_model", "digit_scan_model", "word_scan_model"]
    additions = '#![cfg_attr(verus_keep_ghost, feature(proc_macro_hygiene))]\n\n'
    for module in modules:
        relative = "src/verified/mod.rs" if module == "verified" else f"verification/{module}.rs"
        additions += f'#[cfg(verus_keep_ghost)]\n#[path = "{SOURCE / "crates/hyperreal" / relative}"]\nmod {module};\n'
    new = old.replace(marker, additions + "\n" + marker)
    library.write_text(new)
    (OUT / "lib.rs").write_text(new)
    (OUT / "crate-root.patch").write_text("".join(difflib.unified_diff(
        old.splitlines(True), new.splitlines(True), fromfile="v2413/src/lib.rs", tofile="v2442/src/lib.rs")))
    inputs = files(vendor)
    assert set(inputs) == set(files(PREVIEW / "vendor/num-bigint"))
    assert [n for n, h in inputs.items() if sha(PREVIEW / "vendor/num-bigint" / n) != h] == ["src/lib.rs"]
    artifacts = WORK / "artifacts"
    artifacts.mkdir()
    command = [str(tool / "verus"), "--edition=2021", "--crate-type=lib", "--crate-name", "num_bigint",
               "--cfg", 'feature="std"', "--cfg", 'feature="default"', "--no-cheating", "--output-json",
               "--export", str(artifacts / "num_bigint.vir"), "--compile", "--out-dir", str(artifacts)]
    for name in ["num_integer", "num_traits"]:
        candidates = [n for n in dependencies["artifacts"] if Path(n).name.startswith("lib" + name + "-") and n.endswith(".rlib")]
        assert len(candidates) == 1, name
        command += ["--extern", name + "=" + candidates[0]]
    command += dependencies["verus_arguments"][-2:] + [str(library)]
    protocol = {
        "status": "Frozen before attempting real-crate verification and export",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "preview": PREVIEW.name, "preview_files_sha256": preview,
        "vendor": str(vendor), "vendor_files_sha256": inputs,
        "changed_files": ["src/lib.rs"], "archived_library_sha256": sha(OUT / "lib.rs"),
        "crate_root_patch_sha256": sha(OUT / "crate-root.patch"), "command": command,
        "tool_sha256": tooling, "dependency_artifacts_sha256": dependencies["artifacts"],
        "dependency_report_sha256": sha(reports / "dependencies.json"),
        "scope": "Use the full actual num-bigint crate root, BigUint type and inherent methods, adding only ghost imports of existing arithmetic models. Verify annotated metadata methods/helpers for the native width and attempt exporting/compiling their crate. Unannotated upstream bodies remain outside verification. This is a support preview, not production Cargo binding, cross-crate caller proof, arithmetic completeness, runtime admission or a retained coverage increase. No source/main mutation or trust escape hatch is introduced."
    }
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    start = time.monotonic()
    run = subprocess.run(command, cwd=ROOT, env=os.environ | {"PYTHONDONTWRITEBYTECODE": "1"},
                         capture_output=True, text=True)
    (OUT / "stdout.json").write_text(run.stdout)
    (OUT / "stderr.log").write_text(run.stderr)
    assert files(vendor) == inputs and files(PREVIEW) == preview
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert all(sha(tool / n) == h for n, h in tooling.items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    try:
        parsed = json.loads(run.stdout)
    except json.JSONDecodeError:
        parsed = {}
    result = {"exit_code": run.returncode, "seconds": time.monotonic() - start,
        "verification_results": parsed.get("verification-results"),
        "protocol_sha256": sha(OUT / "protocol.json"), "stdout_sha256": sha(OUT / "stdout.json"),
        "stderr_sha256": sha(OUT / "stderr.log"), "emitted_artifacts_sha256": files(artifacts),
        "source_unchanged": True, "main_unchanged": True, "preview_unchanged": True,
        "tooling_unchanged": True, "vendor_unchanged": True,
        "retained_coverage_changed": False, "runtime_admitted": False}
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
