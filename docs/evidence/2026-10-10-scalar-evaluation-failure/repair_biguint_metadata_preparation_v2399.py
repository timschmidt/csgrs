"""Preserve the missing pinned formatter failure and fork a formatter-only repair."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "prepare_biguint_metadata_candidate_v2398.py"
    partial = E / "biguint-metadata-candidate-v2398"
    record = E / "biguint-metadata-preparation-failure-v2399.json"
    successor = E / "prepare_biguint_metadata_candidate_v2400.py"
    assert all(not p.exists() for p in [record, successor, E / "biguint-metadata-candidate-v2400"])
    assert not (partial / "protocol.json").exists()
    assert "'rustfmt' is not installed for the toolchain '1.98.1" in (partial / "format.stderr.log").read_text()
    code = parent.read_text()
    assert code.count('"rustfmt", "+1.98.1",') == 1
    assert code.count("biguint-metadata-candidate-v2398") == 1
    code = code.replace('"rustfmt", "+1.98.1",', '"rustfmt",')
    code = code.replace("biguint-metadata-candidate-v2398", "biguint-metadata-candidate-v2400")
    ast.parse(code)
    successor.write_text(code)
    data = {
        "status": "The v2398 preparation failed at formatting before proof or SOURCE application",
        "preparation_sha256": sha(Path(__file__)), "failed_preparation_sha256": sha(parent),
        "failed_partial_files_sha256": {str(p.relative_to(E)): sha(p) for p in sorted(partial.rglob("*")) if p.is_file()},
        "diagnostic": (partial / "format.stderr.log").read_text(),
        "successor": successor.name, "successor_sha256": sha(successor),
        "repair": "Use the already installed repository rustfmt without a toolchain override. All proposed source bytes, proof contracts, upstream archive and admission requirements are unchanged before formatting.",
        "formatter_version": subprocess.check_output(["rustfmt", "--version"], text=True).strip(),
        "source_changed": False, "retained_coverage_changed": False,
    }
    record.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"preserved_failed_partial_files": len(data["failed_partial_files_sha256"]), "successor": successor.name}))


if __name__ == "__main__":
    main()
