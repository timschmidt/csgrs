"""Frozen reproduction of the original native workload gates."""
import hashlib
import importlib.util
from pathlib import Path
import sys
sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
BASE = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
assert hashlib.sha256(BASE.read_bytes()).hexdigest() == 'f3ee76657ed7c1e8792452ded7c2380c1ad03a41abb312d80f9adf68b4dc6f05'
spec = importlib.util.spec_from_file_location('preserved_serial_runner', BASE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
runner.E = E
original_sources, original_dump = runner.sources, runner.dump

def sources():
    result = original_sources()
    for name in ['benchmarks/support/harness.rs', 'asar.ttf', 'docs/truetype.png', 'docs/hershey_strings.png',
                 'crates/hyperdrc/docs/CPArti FPGA dev board.zip', 'crates/hyperdrc/docs/HVP109A.zip']:
        result[name] = runner.sha(runner.ROOT / name)
    for folder in [E.parent / '2026-10-16-polyhedral-construction', E.parent / '2026-10-16-minkowski-contracts', E.parent / '2026-10-16-batch-winding-contracts', E]:
        for path in folder.rglob('*.rs'):
            result[str(path.relative_to(runner.ROOT))] = runner.sha(path)
    for path in E.iterdir():
        if path.is_file() and path.suffix in {'.py', '.json', '.md'}:
            result[str(path.relative_to(runner.ROOT))] = runner.sha(path)
    return dict(sorted(result.items()))

def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            'wrapper_sha256': runner.sha(Path(__file__)),
            'properties': ['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8', 'P9', 'P17', 'P18', 'P19', 'P20', 'P21', 'P22', 'P23', 'P24', 'P25', 'P27', 'P35', 'P36', 'P37', 'P38', 'P39'],
            'owning_layers': ['Hypercurve exact strokes, retained evidence and certified predicates', 'Hyperdrc native import and checked geometry consumers'],
            'domain': 'Original full CPArti FPGA and HVP109A KiCad fixtures and unchanged runtime assertions; see contract-v1.md.',
            'consumers': ['Original full-board suite', 'Original copper-neck checks', 'Original Gerber smoke suite'],
            'missing_acceptance_test': ['Reproduce both previously failing native runtime gates on the current admitted stack plus unchanged pending 35 files.'],
            'semantic_oracle': 'Original complete test assertions. Runtime reproduction alone does not establish independent geometry semantics or report byte identity.',
            'previous_goal_turn': 'Progress: 5918d85 committed canonical shared-cell triangulation; the pending 35 native files remain unadmitted and unchanged.',
            'limitations': 'Diagnostic reproduction only. Full source review, semantic qualification, library cost gates, release and full plan remain open. External containment is not an internal resource bound. Historical artifact loss remains recorded.',
        })
    original_dump(path, value)
runner.sources, runner.dump = sources, dump
if __name__ == '__main__':
    runner.main()
