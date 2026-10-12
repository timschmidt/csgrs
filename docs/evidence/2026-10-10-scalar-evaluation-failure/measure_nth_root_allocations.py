"""Measure frozen copied kernels without changing or qualifying public cost runs."""
import gzip
import hashlib
import json
from pathlib import Path
import resource
import shutil
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
TARGET = ROOT / "target/scalar-evaluation-candidate/target"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    protocol_path = E / "nth-root-enclosure-allocation-protocol-v2021.json"
    protocol = read(protocol_path)
    out = E / "nth-root-enclosure-allocation-study-v2021"
    work = ROOT / "target/nth-root-enclosure-allocation-study-v2021"
    assert not out.exists() and not work.exists()
    validation = read(E / "nth-root-enclosure-candidate-validation-v2013.json")
    assert validation["complete"] and len(validation["phases"]) == 9
    assert all(row["result"]["exit_code"] == 0 for row in validation["phases"].values())
    assert validation["source_record_sha256"] == protocol["source_record_sha256"]
    # Run after cost execution has terminated, regardless of whether it passed.
    # The caller must not overlap this separate study with any build or test.
    costs = read(E / "nth-root-enclosure-cost-validation-v2021.json")
    assert costs["source_record_sha256"] == protocol["source_record_sha256"]
    assert costs["complete"] or any(row["exit_code"] != 0 for row in costs["steps"])
    record = E / protocol["source_record"]
    assert sha(record) == protocol["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    for key in ["template", "probe", "baseline_file"]:
        assert sha(E / protocol[key]) == protocol[key + "_sha256"]
    assert sha(SOURCE / protocol["candidate_file"]) == protocol["candidate_file_sha256"]
    deps = TARGET / "release/deps"
    fingerprints = TARGET / "release/.fingerprint"
    libraries = [path for path in deps.glob("libhyperreal-*.rlib")
                 if sha(path) == sha(TARGET / "release/libhyperreal.rlib")]
    assert len(libraries) == 1
    scalar = libraries[0]
    metadata = read(fingerprints / scalar.stem.removeprefix("lib") / "lib-hyperreal.json")
    identity = next(row[3] for row in metadata["deps"] if row[1] == "num")
    matches = []
    for path in deps.glob("libnum-*.rlib"):
        receipt = fingerprints / path.stem.removeprefix("lib") / "lib-num"
        if receipt.is_file() and int.from_bytes(bytes.fromhex(receipt.read_text()), "little") == identity:
            matches.append(path)
    assert len(matches) == 1
    library = matches[0]
    hashes = {str(path): sha(path) for path in [scalar, library, *sorted(deps.glob("libnum_*.rlib"))]}
    out.mkdir()
    work.mkdir()
    shutil.copyfile(__file__, out / "reproduce.py")
    shutil.copyfile(E / protocol["probe"], out / "probe.rs")
    frozen = {path: sha(path) for path in out.iterdir()}

    def verify():
        assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
        assert all(sha(ROOT / name) == digest for name, digest in guard.items())
        assert all(sha(Path(name)) == digest for name, digest in hashes.items())
        assert all(sha(path) == digest for path, digest in frozen.items())

    def caps():
        resource.setrlimit(resource.RLIMIT_AS, (192 * 1024 * 1024,) * 2)
        resource.setrlimit(resource.RLIMIT_CPU, (2, 3))
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

    commands = []

    def run(label, command, limited=False):
        command = list(map(str, command))
        started = time.monotonic()
        stdout, stderr = out / (label + ".stdout"), out / (label + ".stderr")
        timeout, code = False, None
        with stdout.open("wb") as output, stderr.open("wb") as errors:
            try:
                result = subprocess.run(command, cwd=ROOT, stdout=output, stderr=errors,
                    timeout=4 if limited else 120, preexec_fn=caps if limited else None)
                code = result.returncode
            except subprocess.TimeoutExpired:
                timeout = True
        row = {"label": label, "command": command, "exit_code": code,
               "wall_timeout": timeout, "wall_seconds": time.monotonic() - started,
               "wall_seconds_limit": 4 if limited else 120,
               "address_space_mib": 192 if limited else None,
               "cpu_seconds_soft_hard": [2, 3] if limited else None,
               "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        commands.append(row)
        (out / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        if code != 0 or timeout:
            verify()
            (out / "failure.json").write_text(json.dumps(row, indent=2) + "\n")
        assert code == 0 and not timeout, row
        return stdout.read_text()

    verify()
    run("format", ["rustfmt", "--edition", "2024", "--check", out / "probe.rs"])
    run("build", ["rustc", "--edition=2024", "-O", "-D", "warnings", out / "probe.rs",
                  "-L", "dependency=" + str(deps), "--extern", "num=" + str(library), "-o", work / "probe"])
    frozen[work / "probe"] = sha(work / "probe")
    output = run("measure", [work / "probe"], limited=True)
    rows = [json.loads(line) for line in output.splitlines()]
    assert len(rows) == protocol["paired_observations"] == 252
    cases = {}
    for row in rows:
        assert row["endpoints_equal"] and row["repeat"] in range(3)
        key = (row["degree"], row["root_bits"], row["delta"])
        cases.setdefault(key, []).append(row)
    required = {(degree, bits, delta) for degree in range(3, 10)
                for bits in [16, 128, 1024, (65568 + degree - 1) // degree]
                for delta in [-1, 0, 1]}
    assert set(cases) == required and len(cases) == protocol["fixtures"] == 84
    comparisons = []
    for (degree, bits, delta), observations in cases.items():
        assert sorted(row["repeat"] for row in observations) == [0, 1, 2]
        for which in ["baseline", "candidate"]:
            assert len({tuple(row[which]) for row in observations}) == 1
        before, after = observations[0]["baseline"], observations[0]["candidate"]
        comparisons.append({"degree": degree, "root_bits": bits, "delta": delta,
            "sample_bits": observations[0]["sample_bits"], "baseline": before, "candidate": after,
            "change": [a - b for a, b in zip(after, before)]})
    verify()
    result = {
        "status": "Copied-kernel allocation observations completed; public qualification remains separate",
        "source_record_sha256": protocol["source_record_sha256"],
        "protocol": protocol_path.name, "protocol_sha256": sha(protocol_path),
        "libraries_sha256": hashes, "production_num_dependency": str(library),
        "binary_sha256": sha(work / "probe"), "binary_bytes": (work / "probe").stat().st_size,
        "source_files_verified": len(sources), "main_guarded_files_verified": len(guard),
        "guards_unchanged": True, "fixtures": len(cases), "paired_observations": len(rows),
        "kernel_calls": len(rows) * 2, "exact_endpoints_equal": True, "counts_stable": True,
        "measurement_fields": protocol["measurement_fields"], "comparisons": comparisons,
        "increases": [row for row in comparisons if any(value > 0 for value in row["change"])],
        "limits": protocol["limits"],
        "artifacts_sha256": {path.name: sha(path) for path in sorted(out.iterdir()) if path.is_file()},
    }
    (out / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"fixtures": len(cases), "kernel_calls": len(rows) * 2,
                      "endpoints_equal": True, "allocation_increase_cases": len(result["increases"])}))


if __name__ == "__main__":
    main()
