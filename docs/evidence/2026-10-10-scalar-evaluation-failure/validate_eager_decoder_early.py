"""Freeze and run only the eager decoder's formatting, proof and compilation gates."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    baseline = read(E / "eager-bound-decoder-baseline-v2257/protocol.json")
    expected = baseline["source_record_sha256"]
    application = read(E / "eager-bound-decoder-baseline-v2257/application.json")
    assert application["source_record_sha256"] == expected
    old_binary_protocol_path = E / "bound-decoder-binary-comparison-protocol-v2181.json"
    old_binary_protocol = read(old_binary_protocol_path)
    binary_protocol_path = E / "eager-bound-decoder-binary-comparison-protocol-v2262.json"
    comparer_path = E / "compare_eager_decoder_binaries.py"
    controller_path = E / "eager-bound-decoder-early-validation-v2261.json"
    protocol_path = E / "eager-bound-decoder-early-protocol-v2261.json"
    assert all(not p.exists() for p in [binary_protocol_path, comparer_path, controller_path, protocol_path])
    assert sha(Path(old_binary_protocol["retained_library"])) == old_binary_protocol["retained_library_sha256"]
    binary_protocol = copy.deepcopy(old_binary_protocol)
    binary_protocol.update(status="Frozen before the eager-payload runtime build or static probe comparison",
        source_record=baseline["source_record"], source_record_sha256=expected,
        predecessor_protocol=old_binary_protocol_path.name, predecessor_protocol_sha256=sha(old_binary_protocol_path),
        generator_sha256=sha(Path(__file__)),
        prerequisite="All four early format/proof/build commands must pass on the exact candidate source; no broad/runtime retention claim.")
    for row in binary_protocol["build_commands"]:
        command = row["command"]
        position = command.index("-o") + 1
        assert "/target/bound-decoder-binary-comparison-v2181/" in command[position]
        command[position] = command[position].replace("/target/bound-decoder-binary-comparison-v2181/",
            "/target/eager-bound-decoder-binary-comparison-v2262/")
    predecessor = E / "compare_bound_decoder_binaries.py"
    comparer = predecessor.read_text()
    replacements = {
        "bound-decoder-binary-comparison-protocol-v2181.json": binary_protocol_path.name,
        "bound-decoder-candidate-validation-v2180.json": controller_path.name,
        'len(validation["phases"]) == 9': 'len(validation["phases"]) == 4',
        "bound-decoder-binary-comparison-v2181": "eager-bound-decoder-binary-comparison-v2262",
    }
    # Replace the complete protocol filename before its directory-name substring.
    for before, after in replacements.items():
        assert before in comparer
        comparer = comparer.replace(before, after)
    ast.parse(comparer)
    retained = read(E / "domain-predicate-candidate-validation-v1772.json")["phases"]
    commands = []
    for version, previous in [(2258, "typed-api-prototype-v1767-domain-predicate-workspace-fmt"),
                              (2259, "typed-api-prototype-v1768-domain-predicate-included-fmt")]:
        suffix = previous.split("-domain-predicate-", 1)[1]
        commands.append({"phase": f"typed-api-prototype-v{version}-eager-bound-decoder-{suffix}",
            "command": retained[previous]["command"], "baseline_phase": previous})
    proof_phase = "typed-api-prototype-v2260-eager-bound-decoder-verus"
    commands.append({"phase": proof_phase,
        "command": [sys.executable, str(E / "run_candidate_verus.py"), proof_phase]})
    previous = "typed-api-prototype-v1766-domain-predicate-release-build"
    commands.append({"phase": "typed-api-prototype-v2261-eager-bound-decoder-release-build",
        "command": retained[previous]["command"], "baseline_phase": previous})
    binary_protocol_path.write_text(json.dumps(binary_protocol, indent=2) + "\n")
    comparer_path.write_text(comparer)
    protocol_path.write_text(json.dumps({"status": "Frozen before any early validation command",
        "source_record_sha256": expected, "commands": commands, "runner_sha256": sha(Path(__file__)),
        "comparison_protocol": binary_protocol_path.name, "comparison_protocol_sha256": sha(binary_protocol_path),
        "comparison_runner": comparer_path.name, "comparison_runner_sha256": sha(comparer_path),
        "comparison_predecessor": predecessor.name, "comparison_predecessor_sha256": sha(predecessor),
        "comparison_replacements": replacements,
        "limits": "Formatting, conditional proof and compilation only. The changed actual decoder still needs applicable acceptance, mutation, scalar/consumer, resource, replay and fact-retention qualification before retention. Every earlier failed gate remains failed; no current fixture timing is performed."}, indent=2) + "\n")
    phases = {}
    for item in commands:
        phase = E / item["phase"]
        assert not phase.exists()
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase.name,
                        "--exec", *item["command"]], cwd=ROOT, check=True)
        result = read(phase / "result.json")
        phases[phase.name] = {"command": item["command"], "result": result}
        controller_path.write_text(json.dumps({"source_record_sha256": expected,
            "protocol_sha256": sha(protocol_path), "runner_sha256": sha(Path(__file__)),
            "complete": len(phases) == 4 and result["exit_code"] == 0, "phases": phases,
            "scope": "Early static/proof gates only; no workload timings, runtime tests or retention"}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        if phase.name == proof_phase:
            proof = read(phase / "proof-reports/verus.json")["verification-results"]
            assert proof["success"] and proof["verified"] == 655 and proof["errors"] == 0
        print(phase.name, "passed", flush=True)


if __name__ == "__main__":
    main()
