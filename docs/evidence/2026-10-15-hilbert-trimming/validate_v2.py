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
            'missing_acceptance_test': ['Baseline midpoint selection crosses a hole, drops valid disconnected intervals, and rejects boundary overlap.', 'Native trimming must retain exact source parameters and contacts; string projection must reject only unrepresented endpoints.', 'Preserve original full grids, typed invalid/undecided failures, and policy receipts.'],
            'artifact_preparation': 'Candidate setup commands create private copy-on-write targets; overlay.json records the immutable E14 lower build and private /dev/shm upper directories. Standalone phases create a fresh directory and pin their external executables. All retained lower files are audited again at closure.',
            'properties': ['P1', 'P6', 'P8', 'P9', 'P15', 'P16', 'P17', 'P18', 'P19', 'P20', 'P25', 'P29', 'P35', 'P37', 'P39'],
            'owning_layers': ['CSGRS Hilbert grid construction and native trimming composition'],
            'domain': 'Exact finite Hilbert polyline intersection with closed regions; retain native trim evidence and typed failures. See contract-v1.md.',
            'consumers': ['Public Rust Hilbert APIs', 'Readme render example', 'Profile benchmark caller'],
            'semantic_oracle': 'Independent exact directed segment intervals and contact/parameter replay. Counts and wrapper agreement are supplementary only.',
            'limitations': 'External containment only; no internal full-query work/heap-byte/cancellation bound or end-to-end provenance claim. Inherited 27093 missing artifact paths remain recorded. No full-plan or release admission.',
            'previous_goal_turn': 'Progress: 362e157c1 preserves exact-path construction failures. Hilbert midpoint-only clipping and empty-on-error remain; reproduce before editing.',

        })
    original_dump(path, value)


runner.dump = dump
if __name__ == '__main__':
    runner.main()
