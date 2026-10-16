"""Frozen serial qualification with inherited artifact and source protection."""
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
    for name in ['benchmarks/support/harness.rs', 'asar.ttf', 'docs/truetype.png',
                 'docs/hershey_strings.png',
                 'docs/evidence/2026-10-16-polyhedral-construction/baseline_probe_v1.rs']:
        result[name] = runner.sha(runner.ROOT / name)
    for path in list(E.rglob('*.rs')) + list((E / 'fixtures').glob('*.json')):
        result[str(path.relative_to(runner.ROOT))] = runner.sha(path)
    return dict(sorted(result.items()))


def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            'wrapper_sha256': runner.sha(Path(__file__)),
            'missing_acceptance_test': ['Disconnected and concave represented-set sum without convexification.', 'Unused positions do not change a solid; malformed operands cannot hide behind empty.', 'Signed winding preserves holes, multiplicity and cancellation; typed limits and recovery.'],
            'artifact_preparation': 'Fresh /dev/shm targets or private overlays over immutable builds. Every lower hash is retained and rechecked at closure.',
            'properties': ['P1', 'P4', 'P5', 'P6', 'P8', 'P9', 'P11', 'P17', 'P18', 'P19', 'P20', 'P21', 'P22', 'P24', 'P25', 'P27', 'P28', 'P30', 'P35', 'P36', 'P37', 'P39'],
            'owning_layers': ['Hyperreal bounded constant-square projection and selected eighth-angle radical proofs', 'Hypermesh solid decomposition, winding, hull and arrangement', 'Hyperlattice exact projective intersections', 'CSGRS modeling and canonical empty bridge', 'JavaScript numeric resource-count admission'],
            'boundary_contract': 'wasm-count-contract-v1.md',
            'scalar_contracts': ['root-square-contract-v1.md', 'eighth-angle-contract-v1.md'],
            'domain': 'See contract-v1.md: bounded regularized nonzero-winding solids, including disconnected, concave, nested and qualified inputs. Explicit point-set hull migration.',
            'consumers': ['Native Hypermesh and CSGRS Rust', 'JavaScript/WASM', 'Original mesh/feature benchmark and README fixtures'],
            'semantic_oracle': 'Independent exact volume and witness membership, topology and subsequent Booleans; exact ordered artifact equality separately.',
            'limitations': 'Counts and cooperative checks do not bound native primitive work, scalar growth, heap bytes or wall time. External 1800-second/8-GiB containment, except V8 virtual space. Full release/CI and inherited provenance remain open.',
            'previous_goal_turn': 'adef9a64a committed qualified face/loft repair, 40 passing qualification commands and preserved original artifacts. General Minkowski semantics and broader goal remain open.',
        })
    original_dump(path, value)


runner.sources, runner.dump = sources, dump
if __name__ == '__main__':
    runner.main()
