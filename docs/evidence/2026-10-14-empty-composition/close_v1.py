"""Seal completed empty-composition evidence and update only current_work."""
import gzip
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('consumer_validation', E / 'validate_consumers_v1.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
v, ROOT = c.v, c.ROOT


def main():
    out = E / 'final-checks-v1'
    out.mkdir(exist_ok=False)
    with gzip.open(E / 'native-v1/source-hashes.json.gz', 'rt') as f:
        final_sources = json.load(f)
    assert c.sources() == final_sources
    records = {}
    binaries = {}
    for phase, codes in [('baseline-v1', [101]), ('candidate-v1', [0] * 3),
                         ('bindings-v1', [0] * 7), ('native-v1', [0] * 6)]:
        p = E / phase
        record = json.loads((p / 'result.json').read_text())
        assert [r['exit_code'] for r in record['commands']] == codes
        for row in record['commands']:
            for stream in ['stdout', 'stderr']:
                assert v.sha(p / (row['name'] + '.' + stream)) == row[stream + '_sha256']
            assert not row.get('warnings')
        records[phase] = record
        binary_file = p / 'binaries.json'
        binaries.update(json.loads(binary_file.read_text()) if binary_file.exists() else
                        json.loads((p / 'terminal-observation.json').read_text())['executables_sha256'])
    assert all(v.sha(Path(path)) == digest for path, digest in binaries.items())
    native = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert len(native) == 35 and all(v.sha(ROOT / path) == digest for path, digest in native.items())
    changed = ['src/solid.rs', 'crates/hypermesh/src/output.rs', 'readme.md',
               'tests/empty_composition.rs', 'tests/wasm/geometry_certainty.cjs',
               'tests/ffi/empty_composition.c', 'tests/ffi/empty_composition.cpp',
               'tests/ffi/empty_composition.py']
    checks = []
    for name, command in [
        ('workspace-fmt', ['cargo', 'fmt', '--all', '--', '--check']),
        ('diff-whitespace', ['git', 'diff', '--check', '--', *changed]),
    ]:
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            result = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr)
        checks.append({'name': name, 'command': command, 'exit_code': result.returncode,
                       'stdout_sha256': v.sha(out / (name + '.stdout')),
                       'stderr_sha256': v.sha(out / (name + '.stderr'))})
        v.dump(out / 'result.json', checks)
        assert result.returncode == 0
    assert c.sources() == final_sources
    totals = {}
    for name in ['mesh-output-tests', 'mesh-boolean-tests', 'csgrs-all-feature-tests', 'ffi-tests']:
        output = (E / 'native-v1' / (name + '.stdout')).read_text()
        rows = [list(map(int, match)) for match in re.findall(
            r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out', output)]
        totals[name] = {key: sum(row[i] for row in rows) for i, key in enumerate(
            ['passed', 'failed', 'ignored', 'measured', 'filtered'])}
    entry = {
        'updated_on': '2026-10-15',
        'status': 'Implemented with the recorded local and binding qualification; broader plan requirements remain open.',
        'properties': ['P6', 'P8', 'P9', 'P17', 'P18', 'P25', 'P35', 'P37'],
        'owners': ['CSGRS modeling Boolean composition', 'Hypermesh batch-to-mesh conversion'],
        'domain': 'Canonical empty geometry and the existing nonempty closed-PWN domain, under STRICT or APPROXIMATE_512. Noncanonical emptiness, malformed topology and undecided validation remain typed failures.',
        'implementation': 'Reduce one-empty operations through the kernel truth program, retain original operand indices in diagnostics, and convert empty batch rows to zero positions and zero triangles. Nonempty rows retain their shared arena.',
        'baseline': 'Eight new acceptance tests failed on the target revision; the initial candidate passed all eight plus 13 existing tests. The final suite adds hole/disconnected and post-warming mutation cases, for ten acceptance tests.',
        'native_test_totals': totals,
        'binding_consumers': ['C f64 ABI', 'C++ F64 wrapper', 'Python TriangleMeshF64 wrapper', 'Generated WASM JavaScript runtime and TypeScript declarations'],
        'checks': ['Final workspace formatting', 'Strict minimal-feature Clippy', 'Strict all-feature/all-target Clippy for csgrs, hypermesh and csgrs-ffi', 'Strict all-feature rustdoc for csgrs, hypermesh and csgrs-ffi'],
        'evidence': str(E.relative_to(ROOT) / 'closure-v1.json'),
        'remaining': [
            'Primary constructors and convenience operations still contain silent-empty fallbacks; typed failures through all adapters and bindings require further work.',
            'These receipts describe operation-local decisions. End-to-end construction provenance, including export/import, remains open.',
            'Ordinary strict-mode completion, including the original drilled-body case, remains open; no policies or fixtures were weakened here.',
            'Internal total-work, heap-byte and cancellation contracts are not established by these externally contained checks.',
            'The existing ignored README PNG regeneration test was not run. No new sanitizer or directed-fuzz campaign is claimed for this change.',
            'This is not the full repository pre-push matrix or a clean external package/release validation; no push was performed.',
            'The original 35 pending native changes remain byte-identical. Their earlier DRC runtime failures, including the closed scalar-integration trial, were neither resolved nor rerun here.',
        ],
        'plan_complete': False,
    }
    ledger_path = ROOT / 'docs/properties-status.json'
    raw = ledger_path.read_bytes().decode()
    ledger = json.loads(raw)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    outer = lambda data: {key: value for key, value in data.items() if key != 'current_work'}
    assert outer(ledger) == outer(baseline)
    ledger['current_work']['updated_on'] = '2026-10-15'
    ledger['current_work']['empty_result_composition'] = entry
    assert ledger['current_work']['plan_complete'] is False
    match = re.search(r'(?m)^([ \t]*)"current_work": *', raw)
    start = match.end()
    _, end = json.JSONDecoder().raw_decode(raw[start:])
    replacement = json.dumps(ledger['current_work'], indent=2)
    replacement = replacement.replace('\n', '\n' + match.group(1))
    updated = raw[:start] + replacement + raw[start + end:]
    assert json.loads(updated) == ledger and outer(json.loads(updated)) == outer(baseline)
    ledger_path.write_bytes(updated.encode())
    v.dump(E / 'closure-v1.json', {
        'qualification': entry, 'command_records': records, 'final_checks': checks,
        'implementation_and_tests_sha256': {path: v.sha(ROOT / path) for path in changed},
        'preserved_executables_sha256': binaries,
        'pending_native_sha256': native,
        'final_runtime_source_map': 'native-v1/source-hashes.json.gz',
        'ledger_after_sha256': v.sha(ledger_path),
        'only_current_work_changed_in_ledger': True,
        'full_plan_complete': False,
    })
    print(json.dumps({'qualification_closed': True, 'native_tests': totals, 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
