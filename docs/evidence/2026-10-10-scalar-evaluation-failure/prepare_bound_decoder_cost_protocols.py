"""Freeze unchanged original-cap cost gates after full binary identity failed."""
import ast
import copy
import difflib
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
EXPECTED = "80bb17679a0891ff93eba8ed10b9f63c205f3ccb39e35dbb3e187e23bb0b7162"
VERSIONS = {2082: 2169, 2083: 2171, 2084: 2172, 2086: 2174, 2092: 2180,
            **dict(zip(range(2093, 2101), range(2240, 2248)))}


def transform(text):
    text = re.sub(r"v(\d+)\b", lambda match:
        "v" + str(VERSIONS.get(int(match[1]), int(match[1]))), text)
    text = text.replace("integer-coarse-zero", "bound-decoder")
    text = text.replace("integer_coarse_zero", "bound_decoder")
    text = text.replace("e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62", EXPECTED)
    text = text.replace("combined integer shortcut, root kernel and outlined-cache proposal", "proved bound-decoder proposal")
    text = text.replace("combined integer/kernel/selector proposal", "proved bound-decoder proposal")
    text = text.replace("combined proposal", "decoder proposal")
    return text


def main():
    output = E / "bound-decoder-cost-protocol-preparation-v2247.json"
    assert not output.exists()
    comparison_path = E / "bound-decoder-binary-comparison-v2181/summary.json"
    comparison = read(comparison_path)
    assert comparison["source_record_sha256"] == EXPECTED and comparison["builds_passed"] == 7
    assert not comparison["all_probe_binaries_identical"]
    assert comparison["new_timing_processes"] == 0
    assert read(E / "bound-decoder-candidate-validation-v2180.json")["complete"]
    assert read(E / "bound-decoder-proof-validation-v2171.json")["source_record_sha256"] == EXPECTED
    forks = {}

    def fork(old, new, adjust=None):
        parent, target = E / old, E / new
        patch = E / (new.removesuffix(".py") + "-v2247.patch")
        assert not target.exists() and not patch.exists()
        original = parent.read_text()
        source = transform(original)
        if adjust:
            source = adjust(source)
        ast.parse(source)
        target.write_text(source)
        patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                                   fromfile=old, tofile=new, n=0)))
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(parent),
                      "fork_sha256": sha(target), "patch": patch.name, "patch_sha256": sha(patch)}

    def cost_ranges(source):
        assert source.count("range(2093, 2100)") == 1
        source = source.replace("range(2093, 2100)", "range(2240, 2247)")
        before = "Frozen before building or measuring cost binaries for this decoder proposal; all prior failed comparisons and diagnostics remain separate"
        after = "Frozen before running fresh standalone or paired cost measurements; the earlier seven-binary static comparison found no complete executable identity, so reuse is unavailable. All prior failed comparisons and diagnostics remain separate"
        assert source.count(before) == 1
        return source.replace(before, after)

    fork("prepare_integer_coarse_zero_costs.py", "prepare_bound_decoder_costs.py", cost_ranges)
    subprocess.run([sys.executable, str(E / "prepare_bound_decoder_costs.py")], cwd=ROOT, check=True)
    for old in ["record_integer_coarse_zero_initial_costs.py", "prepare_integer_coarse_zero_cost_followup.py",
                "record_integer_coarse_zero_costs.py", "run_integer_coarse_zero_cost_validation.py"]:
        fork(old, old.replace("integer_coarse_zero", "bound_decoder"))
    protocol_path = E / "bound-decoder-cost-protocol-v2240.json"
    protocol = read(protocol_path)
    assert protocol["source_record_sha256"] == EXPECTED
    old_strategy_path = E / "integer-coarse-zero-cost-followup-strategy-v2100.json"
    original = read(old_strategy_path)
    strategy = copy.deepcopy(original)
    strategy.update(source_record_sha256=EXPECTED, cost_protocol=protocol_path.name,
                    cost_protocol_sha256=sha(protocol_path))
    assert all(strategy[key] == original[key] for key in original.keys() -
               {"source_record_sha256", "cost_protocol", "cost_protocol_sha256"})
    assert sum(map(len, strategy["family_fixtures"].values())) == 52 and strategy["repetitions"] == 27
    strategy_path = E / "bound-decoder-cost-followup-strategy-v2247.json"
    assert not strategy_path.exists()
    strategy_path.write_text(json.dumps(strategy, indent=2) + "\n")
    qualification = E / "bound-decoder-cost-qualification-v2181.json"
    assert not qualification.exists()
    qualification.write_text(json.dumps({
        "method": "complete-executable-identity", "qualified_for_unchanged_probe_fixtures": False,
        "source_record_sha256": EXPECTED, "binary_comparison": str(comparison_path.relative_to(E)),
        "binary_comparison_sha256": sha(comparison_path),
        "identical_binaries": sum(row["identical_executable_bytes"] for row in comparison["results"]),
        "changed_families": comparison["changed_probe_families"], "new_timing_processes": 0,
        "retained_runtime_checkpoint": protocol["retained_runtime_checkpoint"],
        "runtime_retained": False,
        "decision": "No fixture receipts are reused for the changed executables. The initial identity-gated replay runner cannot start. Fresh original-cap cost measurements and any prescribed family-wide repeats must pass before a separately forked measured-cost replay runner may start. Binary difference alone does not demonstrate a performance regression or its cause.",
    }, indent=2) + "\n")
    output.write_text(json.dumps({
        "status": "Frozen before any fresh decoder cost measurement",
        "source_record_sha256": EXPECTED, "runner_forks": forks, "preparer_sha256": sha(Path(__file__)),
        "cost_protocol": protocol_path.name, "cost_protocol_sha256": sha(protocol_path),
        "repeat_strategy": strategy_path.name, "repeat_strategy_sha256": sha(strategy_path),
        "repeat_strategy_predecessor": old_strategy_path.name, "repeat_strategy_predecessor_sha256": sha(old_strategy_path),
        "identity_qualification": qualification.name, "identity_qualification_sha256": sha(qualification),
        "scope": "Keep retained v1836 and all 52 full-size fixtures, 468 standalone processes, nine initial pairs, original 192 MiB/CPU 2-3-second/four-second wall caps, identical CPU preludes, and the predeclared 27-pair repeat for every fixture in each initially regressing family. No workload, threshold, assertion, precision, time or work budget changes. All previous failures and the fixed pre-Verus baseline remain explicit. No runtime retention follows from preparation.",
    }, indent=2) + "\n")
    print(json.dumps({"cost_protocol": protocol_path.name, "repeat_fixtures": 52,
                      "binary_identity_qualification": False, "fresh_measurements_started": False}))


if __name__ == "__main__":
    main()
