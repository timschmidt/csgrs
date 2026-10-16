"""Frozen serial batch-winding qualification with inherited input guards."""
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
    for folder in [E.parent / '2026-10-16-minkowski-contracts', E]:
        for path in list(folder.rglob('*.rs')) + list((folder / 'fixtures').glob('*.json')):
            result[str(path.relative_to(runner.ROOT))] = runner.sha(path)
    return dict(sorted(result.items()))


def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            'wrapper_sha256': runner.sha(Path(__file__)),
            'missing_acceptance_test': ['Strict native 26-box and merged-shell union must represent the analytic volume-124 body with its unit cavity.', 'Retain a minimized diagnostic alongside the full original case; preserve coincident/reversed/multiple sheets and shared expression roots.'],
            'artifact_preparation': 'Fresh /dev/shm targets or private overlays over immutable builds; lower hashes rechecked at closure.',
            'properties': ['P4', 'P6', 'P8', 'P9', 'P11', 'P17', 'P18', 'P19', 'P20', 'P21', 'P22', 'P23', 'P24', 'P25', 'P27', 'P35', 'P36', 'P37', 'P39'],
            'owning_layers': ['Hypertri canonical topology-only constrained triangulation', 'Hypermesh coincident-sheet bundling and winding consumers', 'Native Boolean and CSGRS composition consumers'],
            'domain': 'See contract-v1.md and triangulation-contract-v1.md: exact planar straight-line graphs with common protected cells, plus validated bounded closed-PWN integer boxes with coincident sheets, including the full 26-operand and merged-shell forms.',
            'consumers': ['Native Boolean operations and multi-root programs', 'CSGRS composition', 'Affected real bindings and original benchmark consumers'],
            'semantic_oracle': 'Analytic regularized set, exact volume and independent membership witnesses; topology and deterministic artifacts checked separately.',
            'limitations': 'External wall/address-space containment is not an internal total-work or heap bound. Bare-mesh provenance, full CI, release, coverage and formal requirements remain open.',
            'previous_goal_turn': 'Progress: f74a7848 committed qualified Minkowski semantics, exact scalar proofs and typed WASM limits; 636483 available artifacts verified with 27093 inherited missing paths and no new loss.',
        })
    original_dump(path, value)


runner.sources, runner.dump = sources, dump
if __name__ == '__main__':
    runner.main()
