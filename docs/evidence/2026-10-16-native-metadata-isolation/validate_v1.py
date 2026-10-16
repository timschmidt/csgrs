"""Run immutable current-revision metadata validation with retained failures."""
import gzip
import hashlib
import importlib.util
import json
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
preparation = json.loads((E / 'preparation-v1.json').read_text())
assert runner.sha(E / 'source-hashes-v1.json.gz') == preparation['source_hashes_sha256']
with gzip.open(E / 'source-hashes-v1.json.gz', 'rt') as stream:
    expected = json.load(stream)
assert original_sources() == expected['main_guard']
for variant, location in preparation['sources'].items():
    source = Path(location)
    actual = {str(p.relative_to(source)): runner.sha(p)
              for p in source.rglob('*') if p.is_file()}
    assert actual == expected[variant]


def sources():
    result = original_sources()
    for name in ['benchmarks/support/harness.rs', 'asar.ttf', 'docs/truetype.png',
                 'docs/hershey_strings.png', 'properties.md', 'docs/admission.md',
                 'crates/hyperdrc/docs/CPArti FPGA dev board.zip',
                 'crates/hyperdrc/docs/HVP109A.zip']:
        result[name] = runner.sha(runner.ROOT / name)
    for location in preparation['sources'].values():
        for path in Path(location).rglob('*'):
            assert not path.is_symlink(), path
            if path.is_file():
                result[str(path)] = runner.sha(path)
    for path in E.iterdir():
        if path.is_file() and path.suffix in {'.py', '.json', '.md', '.rs', '.gz', '.patch', '.log'}:
            result[str(path.relative_to(runner.ROOT))] = runner.sha(path)
    return dict(sorted(result.items()))


def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            'wrapper_sha256': runner.sha(Path(__file__)),
            'preparation_sha256': runner.sha(E / 'preparation-v1.json'),
            'properties': ['P4', 'P6', 'P10', 'P15', 'P17', 'P18', 'P19', 'P20', 'P27', 'P35', 'P36', 'P37', 'P38', 'P39'],
            'owning_layers': ['Hypercurve retained region validation'],
            'domain': 'Existing supported boundary loops and complete native board fixtures; see contract-v1.md.',
            'consumers': ['CurveRegion2 constructors and native arrangement consumers', 'Hyperdrc full original board checks', 'CSGRS modeling composition'],
            'missing_acceptance_test': ['Requalify the isolated conjunction-order candidate on current sources, preserving original tests, policies, fixtures, limits and complete outcomes.'],
            'semantic_oracle': 'Independent exact offsets, duplicate/source-reuse error priorities, retained proof and later refinement assertions. Full test and report equality requirements remain separate from runtime measurements.',
            'previous_goal_turn': 'Progress: 73472622 reproduced both current native runtime failures and retained full profiling and caller diagnostics; no runtime candidate was admitted.',
            'limitations': 'External containment only. Fixed local costs and full report replay are separate gates. No release, full CI, internal heap/work bound or full-plan completion claim.',
        })
    original_dump(path, value)


runner.sources, runner.dump = sources, dump
if __name__ == '__main__':
    runner.main()
