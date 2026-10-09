"""Record scalar-only reconstruction trials without admitting the prototype."""

import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parent
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
artifacts = {}
trials = []
for index in [1, 2, 3]:
    directory = ROOT / f"area-compaction-scalar-trial{index}"
    source = read(directory / "sources.json")
    assert source["unchanged"] and len(source["source_hashes"]) == 1149
    base = ROOT / source["base_archive"]
    assert digest(base) == source["base_archive_sha256"]
    assert digest(directory / "source-delta.tar.gz") == source["delta_archive_sha256"]
    reconstructed = {}
    for path in [base, directory / "source-delta.tar.gz"]:
        with tarfile.open(path) as archive:
            for member in archive.getmembers():
                if member.isfile():
                    reconstructed[member.name] = hashlib.sha256(archive.extractfile(member).read()).hexdigest()
    assert reconstructed == source["source_hashes"]
    status = read(directory / "status.json")
    expected_codes = [0, 0, 101] if index < 3 else [0] * 6
    assert [item["exit_code"] for item in status["commands"]] == expected_codes
    trials.append({"directory": directory.name, "commands": status["commands"]})
    for path in directory.iterdir():
        if path.is_file():
            artifacts[str(path.relative_to(ROOT))] = {"bytes": path.stat().st_size, "sha256": digest(path)}

out = ROOT / "area-compaction-scalar-trial3"
assert "test result: ok. 5 passed; 0 failed; 0 ignored;" in (out / "scalar-tests.log").read_text()
equal = "Decided { value: Equal, certainty: Exact, stage: Structural }"
lines = (out / "probe.log").read_text().splitlines()
scalar = [line for line in lines if line.startswith("scalar_loop\t")]
assert scalar == [f"scalar_loop\t{index}\tcompact=true\tanalytic={equal}" for index in range(40)]
assert f"aggregate\tscalar_compacted\t{equal}" in lines
assert "aggregate\toriginal\tUnknown { needed: RealRefinement, stage: Undecided }" in lines
for name in ["area_scalar_compaction_probe.rs", "reproduce_area_compaction.py", "analyze_area_compaction.py"]:
    path = ROOT / name
    artifacts[name] = {"bytes": path.stat().st_size, "sha256": digest(path)}
record = {
    "decision": "promising prototype; not admitted",
    "previous_diagnostic": "diagnostics-26.json",
    "contract": {
        "owner": "Hyperreal; proposed explicit Real::compact_pi_laurent() -> Option<Real>",
        "input": "Existing scalar expressions; already compact symbolic classes are cloned unchanged. Generic graphs are recognized by the existing Laurent polynomial in pi with rational/radical coefficients.",
        "bounds": "Existing 64 depth and 512 aggregate-node recognition guards; at most eight pi powers and eight radical terms per coefficient. No guard or arithmetic decision cascade changed.",
        "output": "The same exact Real value in a reconstructed expression, retaining outer rational scale and abort signal. No new carrier, cache, numerical refinement, or predicate policy.",
        "failure": "None for opaque remainders or exhausted recognition. Original source expression remains available. General powi can retain an exp/log representation outside this bounded recognizer.",
        "consumer": "Proposed explicit reduction of complete loop integrals before region aggregation; no curve consumer has changed in these trials.",
    },
    "trials": trials,
    "failures_preserved": [
        "Trial 1 did not compile: a Real-level test accessed a Computable-private cache method. The cache assertion was moved into the owning module.",
        "Trial 2 exposed the exp/log integer-power representation outside existing Laurent recognition. The original failure is retained as a typed-None regression; the independent identity matrix additionally constructs direct Laurent products and reciprocals.",
    ],
    "validation": {
        "focused_tests": 5,
        "binomial_identity_rows": 50,
        "identity_and_repeated_compaction_assertions": 100,
        "other_checks": "Rational cancellation, negative scales, attached abort signal before/after activation, unsupported opaque terms, exp/log power frontier, and no root approximation cache population for covered/uncovered Computable examples.",
        "full_fixture": "All forty original area expressions compact and prove equal to independently derived capsule formulas; their sequential sum proves equal to the full analytic sum. The untouched original aggregate remains Unknown.",
        "source_archives_reconstructed_and_verified": 3,
        "source_files_per_trial": 1149,
    },
    "next_step": "Evaluate curve loop-area integration with the full original fixture, independent public contour/region regressions, complete scalar validation, relevant curve consumers, and proportionate cost/allocation/size evidence before retention.",
    "limitations": "Focused prototype evidence only; neither production code nor an API extension is admitted. No timing improvement is claimed. Native-stroke board gate, final validations, report audit and all broader plan work remain open.",
    "artifacts": artifacts,
}
(ROOT / "diagnostics-27.json").write_text(json.dumps(record, indent=2) + "\n")
print("Verified three source archives, five focused tests, and forty exact scalar-compacted areas.")
