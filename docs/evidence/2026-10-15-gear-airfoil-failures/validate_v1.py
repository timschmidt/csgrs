"""Reuse the frozen serial evidence runner with the adapter-specific contract."""
import hashlib
import sys
sys.dont_write_bytecode = True
import importlib.util
from pathlib import Path

E = Path(__file__).resolve().parent
BASE = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
assert hashlib.sha256(BASE.read_bytes()).hexdigest() == 'f3ee76657ed7c1e8792452ded7c2380c1ad03a41abb312d80f9adf68b4dc6f05'
spec = importlib.util.spec_from_file_location('preserved_serial_runner', BASE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
runner.E = E
original_sources = runner.sources

def sources():
    result = original_sources()
    for name in ['benchmarks/support/harness.rs', 'asar.ttf']:
        result[name] = runner.sha(runner.ROOT / name)
    for path in E.glob('*.rs'):
        result[str(path.relative_to(runner.ROOT))] = runner.sha(path)
    return dict(sorted(result.items()))

runner.sources = sources
original_dump = runner.dump


def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            'wrapper_sha256': runner.sha(Path(__file__)),
            'missing_acceptance_test': ['Reproduce invalid/undecided empty results and unchecked capacity panics for all five sampled families.', 'Preserve typed failures and consumed certainty through all three gear solid wrappers.', 'Keep original full-resolution successful source coordinates and independent semantic witnesses.'],
            'artifact_preparation': 'Candidate setup commands create private copy-on-write targets; overlay.json records the immutable E15 lower build and private /dev/shm upper directories. Standalone phases create a fresh directory and pin their external executables. All retained lower files are audited again at closure.',
            'properties': ['P1', 'P5', 'P6', 'P8', 'P9', 'P17', 'P18', 'P19', 'P20', 'P25', 'P27', 'P35', 'P37', 'P39'],
            'owning_layers': ['CSGRS Sampled gear, rack and airfoil construction and solid composition'],
            'domain': 'Preserve existing sampled gear/rack/airfoil sets while exposing validation, predicate, scalar and allocation failures and operation-local certainty. See contract-v1.md.',
            'consumers': ['Public Rust gear/rack/airfoil APIs and gear solid wrappers', 'Readme render example', 'Profile and feature benchmark callers'],
            'semantic_oracle': 'Independent rack area and membership, topology and downstream volume, with exact source-coordinate preservation as a separate check.',
            'limitations': 'External containment only; no internal full-query work/heap-byte/cancellation bound or end-to-end provenance claim. Inherited 27093 missing artifact paths remain recorded. No full-plan or release admission.',
            'previous_goal_turn': 'Progress: f1dab2248 fixes exact Hilbert clipping. Sampled gear/rack/airfoil constructors still swallow failures; reproduce before editing.',

        })
    original_dump(path, value)


runner.dump = dump
if __name__ == '__main__':
    runner.main()
