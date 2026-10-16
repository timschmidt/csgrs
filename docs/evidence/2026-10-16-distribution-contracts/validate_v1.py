"""Use the preserved serial runner under the distribution/concatenation contract."""
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
            'missing_acceptance_test': ['Reproduce strict distribution failure becoming empty geometry, erased native causes and malformed-source shortcut acceptance.', 'Exercise index rebasing and capacity overflow without publishing a result or unbounded allocation.', 'Preserve original 8/16/12-copy workloads and the 22-copy feature concatenation, with independent semantics and downstream composition.'],
            'artifact_preparation': 'Fresh /dev/shm targets or private copy-on-write overlays over preserved builds; all lower/dependency hashes rechecked at closure.',
            'properties': ['P4', 'P5', 'P6', 'P8', 'P9', 'P10', 'P11', 'P17', 'P18', 'P19', 'P20', 'P21', 'P22', 'P23', 'P24', 'P25', 'P26', 'P27', 'P35', 'P36', 'P37', 'P38', 'P39'],
            'owning_layers': ['Hypermesh triangle indices, closed-PWN validation and native buffer composition', 'CSGRS distribution layout, public failure/certainty and modeling composition'],
            'domain': 'See contract-v1.md. Concatenation preserves open meshes and winding multiplicity; distribution is regularized set union with canonical-empty handling and validated source topology.',
            'consumers': ['Public native/CSGRS Rust mesh APIs', 'Cache and composition callers', 'Original kernel-comparison and feature benchmark workloads'],
            'semantic_oracle': 'Independent placement, membership, signed volume and topology. Exact coordinate/index serialization is a separate preservation check.',
            'limitations': 'External containment only. No internal total-work, heap-byte, cancellation, inherited-provenance or full-plan claim. Baseline enormous-count children additionally use 512-MiB/10-second containment. All 27093 inherited missing artifact expectations remain.',
            'previous_goal_turn': 'Progress: 00e612307 committed typed text construction, exact native outlines and affected-consumer qualification; broader plan and 35 native files remain open.',
        })
    original_dump(path, value)


runner.sources = sources
runner.dump = dump
if __name__ == '__main__':
    runner.main()
