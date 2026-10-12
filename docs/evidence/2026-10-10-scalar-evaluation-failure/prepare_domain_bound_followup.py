"""Freeze the partial cost observations and the two independent follow-up jobs."""
import ast
import hashlib
import json
from pathlib import Path
import statistics

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def write_new(name, data):
    path = E / name
    assert not path.exists(), path
    path.write_text(json.dumps(data, indent=2) + "\n")


partial = E / "domain-bound-preconditioned-cost-v1710"
rows = read(partial / "receipts.json")
failure = read(partial / "failed-attempt.json")
assert len(rows) == 864 and failure["name"] == "algebraic-18-baseline"
assert failure["exit_code"] == -24 and not failure["wall_timeout"]
samples = {}
for row in rows:
    family, index, which = row["name"].rsplit("-", 2)
    assert row["exit_code"] == 0
    fixture = tuple(row["command"][1:])
    samples.setdefault((family, fixture), {"baseline": [], "candidate": []})[which].append(
        int(row["receipt"]["elapsed_ns"]))
cases = []
for (family, fixture), values in samples.items():
    assert all(len(v) == 9 for v in values.values())
    medians = {k: statistics.median(v) for k, v in values.items()}
    cases.append({"family": family, "arguments": list(fixture), "elapsed_ns": values,
                  "median_ns": medians,
                  "median_percent_change": 100 * (medians["candidate"] / medians["baseline"] - 1)})
assert len(cases) == 48
increases = [c for c in cases if c["median_percent_change"] > 5]
assert len(increases) == 1 and increases[0]["family"] == "inverse_trig"
assert increases[0]["arguments"] == ["cost", "acos", "1024", "64"]
preludes = read(partial / "prelude-receipts.json")
assert len(preludes) == 865
write_new("domain-bound-partial-cost-v1710.json", {
    "status": "Incomplete original-cap fixed-CPU comparison; preserve all completed observations and the baseline failure",
    "source_record_sha256": read(E / "domain-bound-candidate-caps-v1693.json")["source_record_sha256"],
    "successful_processes": len(rows), "prelude_processes": len(preludes),
    "complete_fixtures": len(cases), "cases": cases, "failed_attempt": failure,
    "original_run_artifacts_sha256": {p.name: sha(p) for p in sorted(partial.iterdir()) if p.is_file()},
})
protocol = {
    "status": "Prepared after preserving the incomplete fixed-CPU run and before these follow-up measurements",
    "source_record_sha256": read(E / "domain-bound-candidate-caps-v1693.json")["source_record_sha256"],
    "previous_failed_run": partial.name, "failure": failure,
    "failure_scope": "The retained v1672 baseline received SIGXCPU. No final receipt was emitted, so the interrupted stage is unobserved. The original process-cap failure remains unresolved.",
    "initial_above_five_percent_increases": increases,
    "unchanged_inputs": "Frozen probes, harnesses, full-size inputs, query counts, checksums and executable hashes remain unchanged. Both versions use the same limits within each job. No original observation is replaced.",
    "jobs": [
        {
            "output": "domain-bound-focused-cost-v1710",
            "prelude": "domain-bound-focused-cpu-prelude-v1710",
            "repetitions": 27, "cpu_seconds": [2, 3], "wall_seconds": 4,
            "selection": {"inverse_trig": [
                ["cost", "acos", "1024", "64"],
                ["cost", "atan", "1024", "64"],
                ["cost", "pi", "1024", "64"],
            ]},
            "scope": "Twenty-seven alternating-order repetitions of the above-five-percent inverse-trig acos cancellation control and the same-width atan and pi controls. Original 192 MiB, CPU 2/3 second and four-second wall limits remain. Initial and follow-up observations are both retained, with no universal speedup claim.",
        },
        {
            "output": "domain-bound-algebraic-cost-v1710",
            "prelude": "domain-bound-algebraic-cpu-prelude-v1710",
            "repetitions": 9, "cpu_seconds": [4, 5], "wall_seconds": 8,
            "selection": read(E / "domain-sign-cost-followup-protocol-v1639.json")["jobs"][1]["selection"],
            "scope": "Separate matched diagnostic for all six unchanged algebraic fixtures with nine alternating-order pairs, 192 MiB, equal CPU 4/5 second and eight-second wall limits. This does not pass or replace the original two-second baseline failure.",
        },
    ],
}
write_new("domain-bound-cost-followup-protocol-v1710.json", protocol)

helper = (E / "replay_domain_bound_cost.py").read_text()


def replace(old, new, count=1):
    global helper
    assert helper.count(old) == count, (old, helper.count(old), count)
    helper = helper.replace(old, new)


replace("import time\n", "import time\nimport sys\n")
replace("OUT = E / 'domain-bound-preconditioned-cost-v1710'", "PROTOCOL = E / 'domain-bound-cost-followup-protocol-v1710.json'\nCONFIG = json.loads(PROTOCOL.read_text())['jobs'][int(sys.argv[1])]\nOUT = E / CONFIG['output']")
replace("resource.setrlimit(resource.RLIMIT_CPU, (2, 3))", "resource.setrlimit(resource.RLIMIT_CPU, tuple(CONFIG['cpu_seconds']))")
replace("ROOT/'target/domain-bound-cost-cpu-prelude-v1710'", "ROOT/'target'/CONFIG['prelude']")
replace("'iterations':50000000,'timed_query_inputs_and_caps_unchanged':True", "'iterations':50000000,'timed_query_inputs_unchanged':True,\n        'process_cpu_seconds':CONFIG['cpu_seconds'],'process_wall_seconds':CONFIG['wall_seconds'],\n        'original_process_caps_preserved':CONFIG['cpu_seconds']==[2,3] and CONFIG['wall_seconds']==4")
replace("    def guard():\n", "    assert sha(source_record) == read(PROTOCOL)['source_record_sha256']\n    protocol_hash = sha(PROTOCOL)\n    def guard():\n")
replace("        assert sha(OUT / 'reproduce.py') == sha(Path(__file__))", "        assert sha(OUT / 'reproduce.py') == sha(Path(__file__))\n        assert sha(PROTOCOL) == protocol_hash")
replace("    for family, old, new in [", "    selection = {family: {tuple(args) for args in fixtures} for family, fixtures in CONFIG['selection'].items()}\n    for family, old, new in [")
replace("        phases = {'baseline': old, 'candidate': new}", "        if family not in selection:\n            continue\n        phases = {'baseline': old, 'candidate': new}")
replace("        assert all(sha(E / old / n) == sha(E / new / n) for n in ['probe.rs', 'reproduce.py'])", "        commands = {which: [row for row in items if tuple(row['command'][1:]) in selection[family]] * (CONFIG['repetitions'] // 9)\n                    for which, items in commands.items()}\n        assert all(len(items) == CONFIG['repetitions'] * len(selection[family]) for items in commands.values())\n        assert all(sha(E / old / n) == sha(E / new / n) for n in ['probe.rs', 'reproduce.py'])")
replace("timeout=4", "timeout=CONFIG['wall_seconds']", 2)
replace("'cpu_seconds_soft_hard': [2, 3]", "'cpu_seconds_soft_hard': CONFIG['cpu_seconds']", 3)
replace("'wall_seconds_limit': 4", "'wall_seconds_limit': CONFIG['wall_seconds']", 3)
replace("assert all(len(v) == 9 for v in values.values())", "assert all(len(v) == CONFIG['repetitions'] for v in values.values())")
replace("assert len(rows) == 936", "assert len(rows) == 2 * CONFIG['repetitions'] * sum(len(fixtures) for fixtures in selection.values())")
old_scope = next(line for line in helper.splitlines() if line.startswith("        'scope': 'Predeclared"))
replace(old_scope, "        'scope': CONFIG['scope'], 'protocol_sha256': protocol_hash, 'configuration': CONFIG,")
ast.parse(helper)
path = E / "replay_domain_bound_followup.py"
assert not path.exists()
path.write_text(helper)
print(json.dumps({"complete_original_fixtures": len(cases), "followup_jobs": len(protocol["jobs"]),
                  "protocol_sha256": sha(E / "domain-bound-cost-followup-protocol-v1710.json"),
                  "runner_sha256": sha(path)}))
