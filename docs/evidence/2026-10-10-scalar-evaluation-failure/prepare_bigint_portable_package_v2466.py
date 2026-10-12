"""Freeze a self-contained dependency package with byte-bound local proof models."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
BASE = E / "bigint-metadata-export-v2450"
OUT = E / "bigint-portable-package-v2466"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    frontier = E / "approximation-bound-frontier-v2465.json"
    assert sha(frontier) == "412a8b592719c35568ae19096e26b9e044907766b340515e4b772ef961258a9d"
    base_protocol, base_result = read(BASE / "protocol.json"), read(BASE / "result.json")
    assert base_result["exit_code"] == 0 and base_result["verification_results"]["verified"] == 453
    original = Path(base_protocol["vendor"])
    assert files(original) == base_protocol["vendor_files_sha256"]
    vendor = OUT / "vendor/num-bigint"
    shutil.copytree(original, vendor)
    copied = {}
    scalar = SOURCE / "crates/hyperreal"
    for path in sorted((scalar / "src/verified").glob("*.rs")):
        copied["verification/verified/" + path.name] = path
    for name in ["magnitude_model", "bit_length_model", "digit_scan_model", "word_scan_model"]:
        copied["verification/" + name + ".rs"] = scalar / "verification" / (name + ".rs")
    assert len(copied) == 17
    for name, path in copied.items():
        destination = vendor / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(path.read_bytes())
        assert sha(destination) == sources[str(path.relative_to(SOURCE))]
    license_path = ROOT / "crates/hyperreal/LICENSE.txt"
    (vendor / "verification/LICENSE-HYPERREAL").write_bytes(license_path.read_bytes())
    root_before = (vendor / "src/lib.rs").read_text()
    root_after = root_before.replace(str(scalar / "src/verified/mod.rs"), "../verification/verified/mod.rs")
    for name in ["magnitude_model", "bit_length_model", "digit_scan_model", "word_scan_model"]:
        root_after = root_after.replace(str(scalar / "verification" / (name + ".rs")), "../verification/" + name + ".rs")
    assert root_after != root_before and str(ROOT) not in root_after
    assert len(re.findall(r'#\[path = "\.\./verification/[^"]+"\]', root_after)) == 5
    (vendor / "src/lib.rs").write_text(root_after)
    manifest_before = (vendor / "Cargo.toml").read_text()
    manifest_after = manifest_before.replace('license = "MIT OR Apache-2.0"', 'license = "Apache-2.0"\npublish = false')
    manifest_after = manifest_after.replace(
        '# Local verification preview; excluded from the production workspace until admitted.',
        '# Local dependency package with self-contained proof inputs; not a registry release.')
    manifest_after = manifest_after.replace("check-cfg = ['cfg(verus_keep_ghost)']", "check-cfg = ['cfg(verus_keep_ghost)', 'cfg(verus_keep_ghost_body)']")
    assert manifest_after != manifest_before
    (vendor / "Cargo.toml").write_text(manifest_after)
    provenance = {
        "upstream": {"package": "num-bigint", "version": "0.4.6", "license": "MIT OR Apache-2.0",
            "archive_sha256": "a5e44f723f1133c9deac646763579fdb3ac745e418f2a7af9cd0c431da1f20b9"},
        "qualified_metadata_preview": "bigint-metadata-export-v2450",
        "qualified_metadata_protocol_sha256": sha(BASE / "protocol.json"),
        "scalar_checkpoint": checkpoint_path.name, "scalar_checkpoint_sha256": sha(checkpoint_path),
        "proof_models": {name: {"source": str(path.relative_to(SOURCE)), "sha256": sha(path)} for name, path in copied.items()},
        "proof_model_license": "Apache-2.0 as declared by the Hyperreal package manifest",
        "proof_model_license_text_sha256": sha(license_path),
        "package_license_selection": "Apache-2.0 covers the copied proof files and selects the Apache option for upstream dual-licensed source; upstream notices and both original license files are preserved.",
        "scope": "All added models are compiled only under verus_keep_ghost. Relative module paths remove the dependency on a specific scalar checkout. Models remain byte copies of the qualified checkpoint, so changes must be reconciled by digest before crediting new proofs. This local package is not published; Cargo workspace patches do not propagate automatically to downstream registry consumers. Original upstream Rust 1.60 metadata is retained but does not itself establish a fresh MSRV pass. Scalar dependency/runtime admission and release integration are separate."
    }
    (vendor / "verification/provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    (OUT / "original").mkdir()
    (OUT / "original/lib.rs").write_text(root_before)
    (OUT / "original/Cargo.toml").write_text(manifest_before)
    changes = []
    for name, before, after in [("src/lib.rs", root_before, root_after), ("Cargo.toml", manifest_before, manifest_after)]:
        changes.append("".join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
            fromfile="a/" + name, tofile="b/" + name, n=0)))
    (OUT / "portability.patch").write_text("".join(changes))
    final_files = files(vendor)
    assert len(final_files) == 74
    assert {n for n in base_protocol["vendor_files_sha256"] if final_files[n] != base_protocol["vendor_files_sha256"][n]} == {"src/lib.rs", "Cargo.toml"}
    assert all((vendor / "src" / path).resolve().is_relative_to(vendor.resolve()) for path in re.findall(r'#\[path = "([^"]+)"\]', root_after))
    assert all((vendor / "src" / path).is_file() for path in re.findall(r'#\[path = "([^"]+)"\]', root_after))
    protocol = {"status": "Self-contained local package frozen before Cargo or proof qualification",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "previous_frontier_sha256": sha(frontier),
        "base_protocol_sha256": sha(BASE / "protocol.json"), "base_result_sha256": sha(BASE / "result.json"),
        "base_vendor_files_sha256": base_protocol["vendor_files_sha256"], "vendor": str(vendor),
        "vendor_files_sha256": final_files, "copied_model_files": provenance["proof_models"],
        "external_license_source": str(license_path), "external_license_sha256": sha(license_path),
        "original_files_sha256": files(OUT / "original"), "portability_patch_sha256": sha(OUT / "portability.patch"),
        "runtime_method_bytes_changed": False, "candidate_sources_changed": False, "main_changed": False,
        "retained_coverage_changed": False, "runtime_admitted": False,
        "scope": provenance["scope"]}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    assert files(original) == base_protocol["vendor_files_sha256"]
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    print(json.dumps({"vendor_files": len(final_files), "copied_ghost_files": len(copied),
        "changed_existing_files": ["Cargo.toml", "src/lib.rs"], "runtime_method_bytes_changed": False,
        "protocol_sha256": sha(OUT / "protocol.json")}))


if __name__ == "__main__":
    main()
