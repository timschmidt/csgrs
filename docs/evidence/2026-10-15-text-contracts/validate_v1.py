"""Use the preserved serial runner under the text-construction contract."""
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
    for name in ['benchmarks/support/harness.rs', 'asar.ttf']:
        result[name] = runner.sha(runner.ROOT / name)
    for path in E.rglob('*.rs'):
        result[str(path.relative_to(runner.ROOT))] = runner.sha(path)
    return dict(sorted(result.items()))


def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            'wrapper_sha256': runner.sha(Path(__file__)),
            'missing_acceptance_test': [
                'Reproduce malformed/missing glyphs, failed strokes and invalid/undecided scales becoming empty or partial text.',
                'Check exact requested TrueType scale and distinguish sampled conversion from authoritative outline meaning.',
                'Preserve all original full-size README and benchmark completion cases and independent Hershey semantics.',
            ],
            'artifact_preparation': 'Standalone phases use fresh /dev/shm targets and pin external build inputs. Private overlays, when used, record immutable lower builds and separate uppers; closure rechecks all lower hashes.',
            'properties': ['P1', 'P4', 'P5', 'P6', 'P8', 'P9', 'P12', 'P13', 'P14', 'P15', 'P16', 'P17', 'P18', 'P19', 'P20', 'P25', 'P26', 'P27', 'P29', 'P35', 'P36', 'P37', 'P38', 'P39'],
            'owning_layers': ['Native Hypercurve Hershey decoding and strokes', 'CSGRS font-format conversion, layout and modeling composition'],
            'domain': 'Complete typed text construction, exact requested scale and explicit numerical/topological/fidelity contracts. See contract-v1.md; representation migrations must be explicit.',
            'consumers': ['Public native and CSGRS text APIs', 'Dimension annotations', 'README renderer', 'Feature benchmark text cases'],
            'semantic_oracle': 'Independent custom Hershey coordinates, exact advances and Cartesian orientation; distinct exact scale and independently derived synthetic font geometry where applicable. Coordinate/checksum comparisons are separate preservation checks.',
            'limitations': 'External containment only; no complete internal work, heap-byte, cancellation or end-to-end provenance claim. All 27093 inherited missing artifact expectations remain. No full-plan or release admission.',
            'previous_goal_turn': 'Progress: 72cd216cc preserves typed sampled gear/rack/airfoil failures and certainty, with local/downstream validation and fixed cost evidence. Text findings must be reproduced on this revision.',
        })
    original_dump(path, value)


runner.sources = sources
runner.dump = dump
if __name__ == '__main__':
    runner.main()
