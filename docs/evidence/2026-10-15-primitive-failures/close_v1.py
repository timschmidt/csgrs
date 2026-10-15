"""Seal the constructor migration qualification without rewriting prior trials."""
import gzip
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('validation', E / 'validate_v1.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)
ROOT = v.ROOT


def main():
    closure = E / 'closure-v1.json'
    assert not closure.exists()
    records, binaries = {}, {}
    phases = [('baseline-v1', [101]), ('candidate-v1', [0, 101]),
              ('candidate-v2', [0, 0]), ('native-v1', [0] * 7 + [101]),
              ('bindings-v1', [0] * 11 + [1]), ('native-v2', [0] * 10),
              ('bindings-v2', [0])]
    for phase, expected in phases:
        record = json.loads((E / phase / 'result.json').read_text())
        assert [row['exit_code'] for row in record['commands']] == expected
        for row in record['commands']:
            assert row['sources_unchanged']
            for stream in ['stdout', 'stderr']:
                assert v.sha(E / phase / (row['name'] + '.' + stream)) == row[stream + '_sha256']
            if phase not in ['baseline-v1', 'candidate-v1', 'native-v1']:
                assert not re.search(r'^warning(?:\[|:)', (E / phase / (row['name'] + '.stderr')).read_text(), re.M)
            binaries.update(row['binaries_sha256'])
        records[phase] = record
    for phase in ['native-v2', 'bindings-v2']:
        with gzip.open(E / phase / 'source-hashes.json.gz', 'rt') as f:
            assert v.sources() == json.load(f)
    with gzip.open(E / 'bindings-v1/source-hashes.json.gz', 'rt') as f:
        c_binding_sources = json.load(f)
    final_sources = v.sources()
    changed_after_c_consumers = sorted(path for path in set(c_binding_sources) | set(final_sources) if c_binding_sources.get(path) != final_sources.get(path))
    assert changed_after_c_consumers == ['src/wasm/mesh_js.rs']
    # The C/C++/Python consumers ran with the same native source. The only later
    # change is in a module excluded from the C ABI's feature configuration.
    target = ROOT / 'target/primitive-failures/bindings-v1'
    artifacts = [target / 'release/libcsgrs_ffi.so', target / 'wasm32-unknown-unknown/wasm/csgrs.wasm']
    artifacts += [target / (name + '-' + language) for name in ['empty_composition', 'primitive_failures'] for language in ['c', 'cpp']]
    artifacts += [target / 'wasm-bindings-node' / name for name in ['csgrs.js', 'csgrs.d.ts', 'csgrs_bg.wasm']]
    corrected_wasm = ROOT / 'target/primitive-failures/bindings-v2'
    artifacts += [corrected_wasm / 'wasm32-unknown-unknown/wasm/csgrs.wasm']
    artifacts += [corrected_wasm / 'wasm-bindings-node' / name for name in ['csgrs.js', 'csgrs.d.ts', 'csgrs_bg.wasm']]
    binaries.update({str(path.relative_to(ROOT)): v.sha(path) for path in artifacts})
    for phase in ['native-v2', 'bindings-v2']:
        binaries.update(json.loads((E / phase / 'protocol.json').read_text())['preserved_prior_executables_sha256'])
    prior = json.loads((ROOT / 'docs/evidence/2026-10-14-empty-composition/closure-v1.json').read_text())
    binaries.update(prior['preserved_executables_sha256'])
    assert all(v.sha(ROOT / path) == digest for path, digest in binaries.items())
    native = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert all(v.sha(ROOT / path) == digest for path, digest in native.items())
    def counts(phase, name):
        text = (E / phase / (name + '.stdout')).read_text()
        rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', text)
        return dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'], map(sum, zip(*[list(map(int, row)) for row in rows]))))
    entry = {
        'updated_on': '2026-10-15',
        'status': 'Native primitive failure migration implemented and locally qualified in the recorded scope.',
        'properties': ['P4', 'P6', 'P8', 'P18', 'P20', 'P25', 'P30', 'P35', 'P37'],
        'owners': ['CSGRS primitive constructors and successful-result cache', 'shared modeling predicate diagnostics', 'scalar adapters and affected C/C++/Python/WASM error boundaries'],
        'domain': 'Strict cube, cuboid, sphere, cylinder, ellipsoid, frustum, frustum_between, octahedron, icosahedron, arrow and ring torus construction; nonnegative dimensions/radii, directed signed heights, documented resolution minima and representable buffer capacities.',
        'migration': 'Primary constructors now return Result<TriangleMesh, ValidationError>. Negative inputs, invalid resolutions, undecided predicates, unsupported representations, and capacity/reservation failures no longer masquerade as empty geometry. Proven degeneracies remain canonical empty after validation. Only successes are cached. Rust consumers must handle/propagate Result; C statuses are additive, C++/Python preserve status, and WASM constructor exceptions expose code.',
        'baseline': 'All four initial adapter acceptance tests failed on 22ee9939e plus the original in-scope native changes. Candidate v1 passed 66 focused tests but failed strict Clippy on missing const. Candidate v2 fixed that lint and passed 67 focused tests plus strict minimal all-target Clippy. Native v1 later passed the runtime/downstream checks but failed fuzz Clippy on an existing nested conditional; that condition was collapsed without weakening validation. Binding v1 passed all native consumers and reproduced plain-string nonfinite constructor errors in actual WASM. Native v2 and WASM binding v2 qualify the corrections; all failed phases and executables are retained.',
        'test_totals': {name: counts('native-v2', name) for name in ['all-feature-tests', 'ffi-tests', 'downstream-release-model', 'doctests']},
        'binding_consumers': ['Actual C f64 ABI', 'C++ F64 RAII wrapper', 'Python ctypes F64 wrapper', 'Generated WASM Node.js runtime and TypeScript declarations'],
        'evidence': str(closure.relative_to(ROOT)),
        'remaining': [
            'Other modeling conveniences and curve/sampled-field constructors still contain silent-empty or origin fallbacks; this patch does not complete visible-failure migration.',
            'Existing non-constructor adapters still flatten some native error categories. The Go wrapper retains only error text and was not qualified (no Go toolchain found). The separate native JS wrapper requires an external loader; generated WASM was the exercised JavaScript distribution.',
            'Construction uses strict decisions but does not attach end-to-end provenance or certify fidelity to analytic surfaces; export/import provenance remains open.',
            'Checked platform capacities and fallible primary-buffer reservations are not total-work, expression-growth, heap-byte or cancellation budgets. Secondary allocations and allocator/process termination remain outside the contract; no complete internal resource bound is claimed.',
            'The original drilled-body strict-mode frontier and all full-scale geometry remain unchanged. Ordinary strict-mode completion still requires work.',
            'One existing README PNG regeneration test remains ignored. No sanitizer, live fuzz campaign, performance comparison, full repository pre-push matrix, or clean external release installation is claimed for this migration. No push was performed.',
            'The original 35 pending native files remain byte-identical. Earlier DRC runtime failures and scalar-integration limitations remain open and were not rerun here.',
        ],
        'plan_complete': False,
    }
    ledger_path = ROOT / 'docs/properties-status.json'
    raw = ledger_path.read_text()
    ledger = json.loads(raw)
    original = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    outer = lambda data: {key: value for key, value in data.items() if key != 'current_work'}
    assert outer(ledger) == outer(original)
    ledger['current_work']['updated_on'] = '2026-10-15'
    ledger['current_work']['primitive_failure_visibility'] = entry
    assert ledger['current_work']['plan_complete'] is False
    match = re.search(r'(?m)^([ \t]*)"current_work": *', raw)
    start = match.end()
    _, end = json.JSONDecoder().raw_decode(raw[start:])
    replacement = json.dumps(ledger['current_work'], indent=2).replace('\n', '\n' + match.group(1))
    updated = raw[:start] + replacement + raw[start + end:]
    assert json.loads(updated) == ledger and outer(json.loads(updated)) == outer(original)
    ledger_path.write_text(updated)
    changed = set(subprocess.check_output(['git', 'diff', '--name-only', '22ee9939e', 'HEAD'], cwd=ROOT, text=True).splitlines())
    changed.update(subprocess.check_output(['git', 'diff', '--name-only', 'HEAD'], cwd=ROOT, text=True).splitlines())
    changed.difference_update(native)

    v.dump(closure, {
        'qualification': entry, 'command_records': records, 'closure_runner_sha256': v.sha(Path(__file__)),
        'implementation_and_tests_sha256': {path: v.sha(ROOT / path) for path in sorted(changed) if not path.startswith('docs/evidence/')},
        'preserved_executables_sha256': binaries, 'pending_native_sha256': native,
        'final_source_map': 'native-v2/source-hashes.json.gz',
        'source_delta_after_c_consumers': changed_after_c_consumers,
        'ledger_after_sha256': v.sha(ledger_path),
        'only_current_work_changed_in_ledger': True, 'full_plan_complete': False,
    })
    print(json.dumps({'qualification_closed': True, 'totals': entry['test_totals'], 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
