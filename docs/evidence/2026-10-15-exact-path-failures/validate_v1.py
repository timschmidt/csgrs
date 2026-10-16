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
original_dump = runner.dump


def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            'wrapper_sha256': runner.sha(Path(__file__)),
            'properties': ['P6', 'P8', 'P9', 'P16', 'P18', 'P25', 'P30', 'P35', 'P37'],
            'owning_layers': ['CSGRS exact Bezier and B-spline construction grammar'],
            'domain': 'Exact native path and closed Bezier region construction; preserve primary typed failures, native causes and operation-local policy certainty. See contract-v1.md.',
            'consumers': ['Public Rust constructors', 'Readme render example', 'Feature and profile benchmark callers'],
            'semantic_oracle': 'Independent polynomial evaluations and area/membership, plus valid downstream native operations. Native error comparisons establish propagation only.',
            'limitations': 'External containment only; no internal full-query work/heap-byte/cancellation bound or end-to-end provenance claim. Inherited 27093 missing artifact paths remain recorded. No full-plan or release admission.',
            'previous_goal_turn': 'Progress: 958d2fa76 preserves typed mesh Boolean errors with passing affected tests/bindings. Exact path constructors still erase errors or return empty geometry; reproduce before editing.',

        })
    original_dump(path, value)


runner.dump = dump
if __name__ == '__main__':
    runner.main()
