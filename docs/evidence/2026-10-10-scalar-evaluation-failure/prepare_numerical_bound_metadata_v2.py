"""Use the locked crate archive directly when no extracted checksum sidecar exists."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    original = E / "review_numerical_bound_metadata.py"
    target = E / "review_numerical_bound_metadata_v2.py"
    failure = E / "numerical-bound-metadata-reader-failure-v2279.json"
    assert not target.exists() and not failure.exists()
    assert not (E / "numerical-bound-metadata-review-v2279.json").exists()
    text = original.read_text().replace("import tomllib\n", "import tomllib\nimport tarfile\n")
    text = text.replace("numerical-bound-metadata-review-v2279.json", "numerical-bound-metadata-review-v2279-v2.json")
    start = text.index('    checksum = read(dependency / ".cargo-checksum.json")\n')
    end = text.index('    dependency_sections = ', start)
    text = text[:start] + '''    archive = registry / "cache/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6.crate"
    assert sha(archive) == packages[0]["checksum"]
    with tarfile.open(archive, "r:gz") as package:
        for name in ["src/biguint.rs", "src/bigint.rs", "src/lib.rs"]:
            assert package.extractfile("num-bigint-0.4.6/" + name).read() == (dependency / name).read_bytes()
        license_text = package.extractfile("num-bigint-0.4.6/LICENSE-MIT").read().decode()
''' + text[end:]
    text = text.replace('"dependency_files_match_registry_checksums": True,', '"dependency_files_match_locked_archive": True, "dependency_license": license_text,')
    marker = '        "status": "Read-only next-proof design review; no implementation or formal coverage added",\n'
    assert text.count(marker) == 1
    text = text.replace(marker, marker + '''        "reader_failure": "numerical-bound-metadata-reader-failure-v2279.json",
        "reader_failure_sha256": sha(E / "numerical-bound-metadata-reader-failure-v2279.json"),
        "schema_correction": "Compare source bytes directly to the locked, hash-verified crate archive because this extraction has no checksum sidecar. No source, test, proof or admission claim changes.",
''')
    ast.parse(text)
    failure.write_text(json.dumps({"status": "Read-only metadata review stopped before producing a record",
        "runner": original.name, "runner_sha256": sha(original), "exit_code": 1,
        "exception": "FileNotFoundError: extracted num-bigint 0.4.6 has no .cargo-checksum.json",
        "location": "review_numerical_bound_metadata.py line 34",
        "source_changed": False, "new_test_or_cost_processes": 0, "successor": target.name,
        "correction": "Use Cargo.lock's archive checksum and compare the selected extracted source files directly with that archive; also preserve the dependency's MIT license alongside quoted sections."}, indent=2) + "\n")
    target.write_text(text)
    print(json.dumps({"successor": target.name, "preserved_failure": failure.name}))


if __name__ == "__main__":
    main()
