"""Close the bounds acceptance record after all named trials terminate."""
import gzip
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('validation', E / 'validate_v2.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)
ROOT = v.ROOT


def counts(text):
    matches = re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', text)
    result = dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'],
                      [sum(int(row[i]) for row in matches) for i in range(5)]))
    result['passed'] += len(re.findall(r'^--- PASS:', text, re.M))
    result['failed'] += len(re.findall(r'^--- FAIL:', text, re.M))
    return result


def main():
    output = E / 'closure-v1.json'
    assert not output.exists()
    old = ROOT / 'docs/evidence/2026-10-15-transform-failures/closure-v1.json'
    binaries = dict(json.loads(old.read_text())['preserved_executables_sha256'])
    pending = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert all(v.sha(ROOT / name) == digest for name, digest in pending.items())
    extra_inputs = json.loads((E / 'documentation-and-runner-inputs.json').read_text())
    assert all(v.sha(ROOT / name) == digest for name, digest in extra_inputs.items())
    current = v.sources()
    accepted = ['native-v2', 'bindings-v1', 'downstream-v1']
    totals = {}
    results = {}
    retained_logs = {}
    for phase in ['baseline-v1', 'candidate-v1', 'candidate-v2', 'native-v1', *accepted]:
        directory = E / phase
        result = json.loads((directory / 'result.json').read_text())
        results[phase] = result
        for row in result['commands']:
            assert row['sources_unchanged']
            binaries.update(row['binaries_sha256'])
            for stream in ['stdout', 'stderr']:
                path = directory / (row['name'] + '.' + stream)
                assert v.sha(path) == row[stream + '_sha256']
                retained_logs[str(path.relative_to(ROOT))] = v.sha(path)
            if phase in accepted:
                assert row['exit_code'] == 0
                assert not re.search(r'^warning(?:\[|:)', (directory / (row['name'] + '.stderr')).read_text(), re.M)
                totals[phase + '/' + row['name']] = counts((directory / (row['name'] + '.stdout')).read_text())
        if phase in accepted:
            assert result['passed']
            assert len(result['commands']) == len(json.loads((E / (phase + '.json')).read_text()))
            with gzip.open(directory / 'source-hashes.json.gz', 'rt') as f:
                assert current == json.load(f), phase
    assert counts((E / 'baseline-v1/bounds-regressions.stdout').read_text())['failed'] == 8
    assert totals['native-v2/all-feature-tests']['passed'] == 202
    assert totals['native-v2/minimal-tests']['passed'] == 38
    assert totals['native-v2/hypermesh-tests']['passed'] == 235
    assert totals['native-v2/ffi-tests']['passed'] == 10
    assert totals['downstream-v1/downstream-materialize-unit']['passed'] == 10
    assert totals['bindings-v1/go-run']['passed'] == 1
    assert totals['downstream-v1/downstream-release-models']['passed'] == 2
    assert totals['downstream-v1/downstream-materialize']['passed'] == 3
    assert 'thermal_spoke_bounds_preserve_empty_clearance' in (E / 'downstream-v1/downstream-materialize-unit.stdout').read_text()
    assert '--- PASS: TestBoundsAbsenceAndPlacement' in (E / 'bindings-v1/go-run.stdout').read_text()
    target = ROOT / 'target/bounds-failures/bindings-v1'
    artifacts = ['release/libcsgrs_ffi.so', 'wasm32-unknown-unknown/wasm/csgrs.wasm',
                 'wasm-bindings-node/csgrs.js', 'wasm-bindings-node/csgrs.d.ts', 'wasm-bindings-node/csgrs_bg.wasm', 'go-bounds.test']
    artifacts += [name + '-' + language for name in ['empty_composition', 'primitive_failures', 'transform_failures', 'bounds_failures'] for language in ['c', 'cpp']]
    binaries.update({str((target / name).relative_to(ROOT)): v.sha(target / name) for name in artifacts})
    assert all(v.sha(ROOT / name) == digest for name, digest in binaries.items())
    entry = {
        'updated_on': '2026-10-15',
        'status': 'Optional bounds and positioning/TPMS failure migration implemented and locally qualified in the recorded scope.',
        'properties': ['P4', 'P6', 'P8', 'P9', 'P18', 'P20', 'P25', 'P30', 'P35', 'P37'],
        'owners': ['Hypermesh indexed bounds', 'CSGRS optional bounds and positioning', 'native curve bounds bridge', 'sampled TPMS extraction', 'scalar and language adapters', 'HyperCircuit thermal-clearance bounds'],
        'domain': 'Valid indexed point sets and open or closed meshes, canonical empty meshes, and native curve regions. Absence differs from nonempty zero-size bounds. Bounds queries do not certify closed-solid topology. TPMS numerical certainty applies to sampled geometry, not the continuous field; operation-local receipts do not certify inherited construction provenance.',
        'migration': 'Modeling and scalar-adapter bounds return Result<Option<Aabb>> or optional context receipts. Center/float and all six TPMS constructors return Result. C NO_BOUNDS=10 is additive and leaves output untouched; C++ optional, Python None, Go nil pointer with nil error, WASM undefined or isEmpty/optional endpoints expose absence. Go StatusError retains native category. HyperCircuit uses shared native optional bounds under its existing policy and retains typed curve failures. Resolution below two is deliberately rejected instead of clamped; period must be positive and wall thickness nonnegative.',
        'semantic_evidence': [
            'Origin point bounds are Some while canonical empty has None; exact centered/floated cuboids retain coordinate extrema, signed volume 48, required topology and subsequent Boolean composition.',
            'Native indexed bounds reject malformed or subsequently mutated triangle indices before reusing position facts.',
            'Strict/approximate/strict bounds and positioning retain policy admissibility and certainty; undecided failures preserve native payloads through adapters and opaque-real C status.',
            'Empty curves retain absent bounds and completed empty Hilbert output after parameter validation.',
            'All three TPMS sheet and solid families consume empty bounds; zero-thickness solids remain empty only after validating parameters. Invalid/unknown periods and thicknesses and overflowing resolution remain typed failures.',
            'The shared sampled-field path preserves SurfaceNetsError and recovers on a subsequent constant field. Existing 10-resolution TPMS solid topology fixtures and other source fixture resolutions are unchanged.',
        ],
        'baseline_and_failed_trials': 'All eight baseline tests failed at 542871237 plus the original in-scope native changes. Candidate v1 failed compilation during migration. Candidate v2 passed 133 focused tests but failed strict Clippy on a WASM getter annotation. Native v1 passed its commands but exposed a minimal-feature unused-import warning and predates the final downstream/WASM call-site corrections; it is retained as historical evidence. Final accepted phases are recorded separately.',
        'test_totals': totals,
        'bindings': 'Actual f64 C, C++, Python, and compiled cgo consumers exercise absence, positioning, error/recovery and composition. Opaque-real FFI tests verify absent/undecided output records remain untouched and failures publish no geometry handle. Generated Node WASM and strict generated TypeScript checks pass. Go toolchain metadata and SHA-256 are retained in go-toolchain.json.',
        'resources': 'Native bounds add an O(triangles) index-validation traversal before position-fact reuse. Primary sample buffers use checked capacity/reservation; secondary allocations, native extraction and scalar expression work remain unbounded internally. No total-work, heap-byte or cancellation contract is claimed. Commands use an external 1800-second wall limit. No performance improvement is claimed.',
        'limitations': [
            'Eight existing ignored tests remain unexecuted: one CSGRS rendering case and seven Hypermesh external-fixture/manual scale or memory cases.',
            'This is not the complete workflow matrix, sanitizer admission, clean external release/MSRV validation, or a performance comparison.',
            'The separate native JavaScript loader is supplied by consumers and remains unqualified. Go qualification uses the local cgo wrapper and actual native library, not a published Go module.',
            'HyperCircuit thermal-spoke placement formulas are preserved. Their full geometric fidelity is not established by the bounds migration or the existing report-oriented integration fixtures; independent placement/set-equality coverage remains follow-up work.',
            'Legacy SDF conveniences, distribution, projection/slicing and other silent fallback migrations remain open, together with end-to-end provenance, internal resource limits, ordinary strict workflow completion and prior scalar/native admission gaps.',
        ],
        'evidence': str(output.relative_to(ROOT)),
        'implementation_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'plan_complete': False,
    }
    ledger_path = ROOT / 'docs/properties-status.json'
    raw = ledger_path.read_text()
    before = v.sha(ledger_path)
    ledger = json.loads(raw)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    outer = lambda data: {key: value for key, value in data.items() if key != 'current_work'}
    assert outer(ledger) == outer(baseline)
    ledger['current_work']['bounds_failure_visibility'] = entry
    ledger['current_work']['updated_on'] = '2026-10-15'
    ledger['current_work']['plan_complete'] = False
    match = re.search(r'(?m)^([ \t]*)"current_work": *', raw)
    start = match.end()
    _, end = json.JSONDecoder().raw_decode(raw[start:])
    replacement = json.dumps(ledger['current_work'], indent=2).replace('\n', '\n' + match.group(1))
    updated = raw[:start] + replacement + raw[start + end:]
    assert json.loads(updated) == ledger and outer(json.loads(updated)) == outer(baseline)
    ledger_path.write_text(updated)
    v.dump(output, {
        'qualification': entry, 'closure_runner_sha256': v.sha(Path(__file__)),
        'prior_qualification_sha256': v.sha(old), 'phase_results': results,
        'preserved_executables_sha256': binaries, 'retained_logs_sha256': retained_logs,
        'pending_native_sha256': pending,
        'documentation_and_runner_inputs_sha256': extra_inputs,
        'ledger_before_sha256': before, 'ledger_after_sha256': v.sha(ledger_path),
        'only_current_work_changed_in_ledger': True, 'full_plan_complete': False,
    })
    print(json.dumps({'qualification_recorded': True, 'executables_verified': len(binaries), 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
