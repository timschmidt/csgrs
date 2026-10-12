"""Fork the dormant identity-gated replay plan to require fresh measured costs."""
import ast
import copy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    predecessor = E / "bound-decoder-replay-protocol-v2182.json"
    original = read(predecessor)
    qualification = read(E / "bound-decoder-cost-qualification-v2181.json")
    assert not qualification["qualified_for_unchanged_probe_fixtures"]
    costs = read(E / "bound-decoder-cost-protocol-v2240.json")
    assert costs["source_record_sha256"] == original["source_record_sha256"] == qualification["source_record_sha256"]
    output = E / "bound-decoder-measured-replay-protocol-v2247.json"
    assert not output.exists() and not (E / "bound-decoder-cost-validation-v2247.json").exists()
    updated = copy.deepcopy(original)
    fork_names = ["run_bound_decoder_replays.py", "compare_bound_decoder_queries.py",
                  "compare_bound_decoder_replays.py", "audit_bound_decoder_fact_retention.py"]
    for name in fork_names:
        parent = E / name
        target = E / name.replace(".py", "_v2.py")
        assert not target.exists() and sha(parent) == original["runner_forks"][name]["fork_sha256"]
        source = parent.read_text()
        assert source.count(predecessor.name) == 1
        source = source.replace(predecessor.name, output.name)
        if name == "run_bound_decoder_replays.py":
            start = source.index('    qualification = read(E / "bound-decoder-cost-qualification-v2181.json")\n')
            end = source.index('    build, unit = ', start)
            source = source[:start] + (
                '    cost_protocol_path = E / "bound-decoder-cost-protocol-v2240.json"\n'
                '    assert sha(cost_protocol_path) == protocol["cost_protocol_sha256"]\n'
                '    costs = read(E / "bound-decoder-cost-review-v2247.json")\n'
                '    controller = read(E / "bound-decoder-cost-validation-v2247.json")\n'
                '    assert controller["complete"] and costs["local_timing_and_process_gates_passed"]\n'
                '    assert costs["source_record_sha256"] == controller["source_record_sha256"] == expected\n'
                '    assert costs["original_process_caps_preserved"]\n'
                '    initial = read(E / costs["initial_record"])\n'
                '    assert sha(E / costs["initial_record"]) == costs["initial_record_sha256"]\n'
                '    assert initial["protocol_sha256"] == sha(cost_protocol_path)\n'
            ) + source[end:]
        ast.parse(source)
        target.write_text(source)
        del updated["runner_forks"][name]
        updated["runner_forks"][target.name] = {
            "predecessor": name, "predecessor_sha256": sha(parent), "fork_sha256": sha(target),
            "change": "Bind the same replay/receipt/fact loops to this protocol; the controller additionally requires the fresh original-cap cost gate and its prescribed repeats to pass.",
        }
    updated.update(
        status="Frozen before fresh measured costs or replays; the original executable-identity route remains unqualified",
        protocol_predecessor=predecessor.name, protocol_predecessor_sha256=sha(predecessor),
        cost_protocol="bound-decoder-cost-protocol-v2240.json",
        cost_protocol_sha256=sha(E / "bound-decoder-cost-protocol-v2240.json"),
        cost_gate={"review": "bound-decoder-cost-review-v2247.json",
                   "controller": "bound-decoder-cost-validation-v2247.json",
                   "requires": "Both complete, source-matched and all local timing/process gates passed"},
        audit_scope="Every cap, numerical/public corpus, assertion, receipt/fact comparison and workload remains unchanged. The earlier identity qualification stays false and its runner remains dormant. These new audit runners require the freshly measured original-cap gate. The public probe still binds its runtime library to the completed binary-comparison library digest.",
    )
    updated["unsatisfied_identity_qualification"] = updated.pop("cost_qualification")
    for key in ["caps", "numerical_replays", "public_replays", "requirements"]:
        assert updated[key] == original[key]
    output.write_text(json.dumps(updated, indent=2) + "\n")
    print(json.dumps({"protocol": output.name, "forked_runners": len(fork_names),
                      "corpora_and_caps_unchanged": True, "measurements_started": False}))


if __name__ == "__main__":
    main()
