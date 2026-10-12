"""Freeze retained library bytes and candidate compile commands before rebuilding."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    output = E / "bound-decoder-binary-comparison-protocol-v2181.json"
    archive = E / "bound-decoder-retained-library-v2181.rlib.gz"
    assert not output.exists() and not archive.exists()
    baseline_dir = E / "bound-decoder-baseline-v2168"
    baseline = read(baseline_dir / "protocol.json")
    baseline_results = read(baseline_dir / "summary.json")
    assert baseline_results["builds_passed"] == 7
    records = {r["family"]: r for r in baseline_results["results"]}
    expected = "80bb17679a0891ff93eba8ed10b9f63c205f3ccb39e35dbb3e187e23bb0b7162"
    proof = E / "typed-api-prototype-v2169-bound-decoder-verus"
    assert sha(proof / "source-hashes.json.gz") == expected
    result = read(proof / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    commands = []
    hyperreal = {}
    for row in baseline["build_commands"]:
        assert all(sha(Path(n)) == h for n, h in row["libraries_sha256"].items())
        record = records[row["family"]]
        assert record["matches_historical_executable_bytes"]
        assert sha(Path(record["binary"])) == record["binary_sha256"] == row["original_binary_sha256"]
        command = list(row["command"])
        command[command.index("-o") + 1] = str(ROOT / "target/bound-decoder-binary-comparison-v2181" / row["family"])
        commands.append({"family": row["family"], "command": command,
            "probe": row["probe"], "probe_sha256": row["probe_sha256"],
            "baseline_binary": record["binary"], "baseline_binary_sha256": record["binary_sha256"],
            "original_binary": row["original_binary"], "original_binary_sha256": row["original_binary_sha256"],
            "baseline_libraries_sha256": row["libraries_sha256"]})
        for n, h in row["libraries_sha256"].items():
            if Path(n).name.startswith("libhyperreal-"):
                hyperreal[n] = h
    assert len(hyperreal) == 1
    library, digest = next(iter(hyperreal.items()))
    # Production libraries still belong to the restored retained build. The
    # intervening Verus jobs use their own proof dependency artifacts.
    data = Path(library).read_bytes()
    assert hashlib.sha256(data).hexdigest() == digest
    archive.write_bytes(gzip.compress(data, mtime=0))
    protocol = {"status": "Frozen before compiling the decoder's runtime libraries or consumer probes",
        "source_record": str((proof / "source-hashes.json.gz").relative_to(E)), "source_record_sha256": expected,
        "retained_source_record_sha256": baseline["source_record_sha256"],
        "baseline_protocol": "bound-decoder-baseline-v2168/protocol.json", "baseline_protocol_sha256": sha(baseline_dir / "protocol.json"),
        "baseline_summary": "bound-decoder-baseline-v2168/summary.json", "baseline_summary_sha256": sha(baseline_dir / "summary.json"),
        "generator_sha256": sha(Path(__file__)), "retained_library": library, "retained_library_sha256": digest,
        "retained_library_archive": archive.name, "retained_library_archive_sha256": sha(archive),
        "retained_library_bytes": len(data), "build_commands": commands,
        "prerequisite": "All nine bound-decoder runtime commands must complete on the exact proof source before the candidate probe builds.",
        "decision": "Compare complete executable bytes, libraries and sizes. Exact probe binary identity permits reuse of the existing immutable fixture receipts for that executable; a changed executable requires fresh original-cap cost controls. Report any rlib mismatch separately. Historical failures remain failed, and binary identity does not establish whole-crate proof, resource bounds, native/full-board admission or the fixed pre-Verus acceptance gate.",
        "timing_processes_planned": 0, "runtime_retained": False}
    output.write_text(json.dumps(protocol, indent=2) + "\n")
    print(json.dumps({"frozen_builds": 7, "retained_library_bytes": len(data), "timing_processes_planned": 0}))


if __name__ == "__main__":
    main()
