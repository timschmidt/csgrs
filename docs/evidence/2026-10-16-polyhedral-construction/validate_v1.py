"""Use the preserved serial runner under the polyhedral construction contract."""
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
original_sources = runner.sources
original_dump = runner.dump


def sources():
    result = original_sources()
    for name in ['benchmarks/support/harness.rs', 'asar.ttf', 'docs/truetype.png', 'docs/hershey_strings.png']:
        result[name] = runner.sha(runner.ROOT / name)
    for path in E.rglob('*.rs'):
        result[str(path.relative_to(runner.ROOT))] = runner.sha(path)
    return dict(sorted(result.items()))


def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            "wrapper_sha256": runner.sha(Path(__file__)),
            'missing_acceptance_test': ['Reproduce polygon/loft success, failure categories, native causes and downstream topology on 67a3daae8.', 'Compare independently derived set meaning for concave/crossed faces, repeated sections and disconnected Minkowski operands.', 'Retain original tetrahedron, square-section loft and cube-sum benchmark/render workloads.'],
            'artifact_preparation': 'Fresh /dev/shm targets or private copy-on-write overlays over preserved builds; all lower/dependency hashes rechecked at closure.',
            'properties': ['P4', 'P5', 'P6', 'P8', 'P9', 'P10', 'P11', 'P17', 'P18', 'P19', 'P20', 'P21', 'P22', 'P23', 'P24', 'P25', 'P26', 'P27', 'P35', 'P36', 'P37', 'P38', 'P39'],
            'owning_layers': ['Hypertri planar ring validation and triangulation', 'Hypermesh topology, convexity and native mesh operations', 'CSGRS modeling contracts, failure/certainty and composition'],
            'domain': 'See contract-v1.md. Reproduce polygon faces, lofted boundaries and represented-set versus vertex-hull Minkowski semantics before selecting an implementation milestone.',
            'consumers': ['Public native/CSGRS Rust construction APIs', 'Scalar adapters, C/C++/Python/Go and JS/WASM where affected', 'Original feature/mesh benchmarks and README renders'],
            'semantic_oracle': 'Independent exact area, volume and membership, topology and subsequent composition. Deterministic coordinate/index serialization is a separate check.',
            'limitations': 'External containment only. No internal total-work, heap-byte, cancellation, inherited-provenance or full-plan claim. All 27093 inherited missing artifact expectations remain.',
            'previous_goal_turn': 'Progress: 67a3daae8 committed typed distribution failure and native composition; 30 qualification commands and 80 fixed cost rows passed, exact artifacts preserved, no new artifact loss; broader plan and 35 native files remain open.',
        })
    original_dump(path, value)


runner.sources = sources
runner.dump = dump
if __name__ == '__main__':
    runner.main()
