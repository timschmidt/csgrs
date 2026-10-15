"""Close the transform acceptance record after all named trials terminate."""
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


def counts(text):
    matches = re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', text)
    return dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'],
                    [sum(int(row[i]) for row in matches) for i in range(5)]))


def main():
    output = E / 'closure-v1.json'
    assert not output.exists()
    old = ROOT / 'docs/evidence/2026-10-15-primitive-failures/qualification-correction-v1.json'
    binaries = dict(json.loads(old.read_text())['preserved_executables_sha256'])
    pending = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert all(v.sha(ROOT / name) == digest for name, digest in pending.items())
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
    assert counts((E / 'baseline-v1/transform-regressions.stdout').read_text())['failed'] == 6
    assert totals['native-v2/all-feature-tests']['passed'] == 191
    assert totals['native-v2/hypermesh-tests']['passed'] == 235
    assert totals['native-v2/ffi-tests']['passed'] == 9
    assert totals['downstream-v1/downstream-release-models']['passed'] == 2
    downstream = (E / 'downstream-v1/downstream-release-models.stdout').read_text()
    for test in ['external_obj_vrml_and_gltf_package_models_resolve_with_digest_and_scene_identity',
                 'failed_package_model_transform_is_an_explicit_omission_with_recovery']:
        assert test + ' ... ok' in downstream
    target = ROOT / 'target/transform-failures/bindings-v1'
    artifacts = ['release/libcsgrs_ffi.so', 'wasm32-unknown-unknown/wasm/csgrs.wasm',
                 'wasm-bindings-node/csgrs.js', 'wasm-bindings-node/csgrs.d.ts', 'wasm-bindings-node/csgrs_bg.wasm']
    artifacts += [name + '-' + language for name in ['empty_composition', 'primitive_failures', 'transform_failures'] for language in ['c', 'cpp']]
    binaries.update({str((target / name).relative_to(ROOT)): v.sha(target / name) for name in artifacts})
    assert all(v.sha(ROOT / name) == digest for name, digest in binaries.items())
    entry = {
        'updated_on': '2026-10-15',
        'status': 'Transform failure migration and projective finiteness repair implemented and locally qualified in the recorded scope.',
        'properties': ['P4', 'P6', 'P8', 'P18', 'P20', 'P25', 'P30', 'P35', 'P37'],
        'owners': ['Hypermesh indexed projective transformation', 'CSGRS modeling transforms', 'scalar and language adapters', 'HyperCircuit package-model materialization'],
        'domain': 'Indexed open or closed meshes, point sets, and canonical empty geometry. Modeling transforms require invertibility and nonzero projective scale of constant sign on each triangle. Distinct finite components may have opposite signs. Numerical certainty is operation-local; topology and inherited provenance are separate.',
        'migration': 'solid::transform/rotate/scale/mirror, SolidExt::transformed/translated, and curve::translated_solid now return Result. Existing try_ entry points retain typed native Mesh errors. C UNBOUNDED=9 is additive; C++/Python retain status and WASM GeometryError.code distinguishes unbounded/invalid/undecided failures. Failed HyperCircuit placement produces a typed omission without publishing a model or resolution evidence.',
        'semantic_evidence': ['Singular transforms and invalid planes cannot become successful empty geometry.', 'Invalid triangle indices fail even after a previous successful transform; shared kernel checks serve modeling and adapters.', 'An exact rational triangle with finite vertices and an edge crossing x=0 rejects the invertible map (x,y,z)->(1/x,y/x,z/x).', 'Separate positive/negative-scale components retain independently derived exact coordinates and orientation.', 'Exact reflected/scaled/translated/rotated cubes preserve positive signed volume 24, required topology, empty-union composition, and subtraction to canonical empty.', 'Strict/approximate/strict failure sequences retain their category, and subsequent valid operations recover.'],
        'baseline_and_failed_trials': 'All six baseline tests failed on 8850192f2 plus the original in-scope native changes. Candidate v1 failed compilation during call-site migration; candidate v2 passed 75 focused tests and strict minimal all-target Clippy. Native v1 exposed an unmigrated curve alias and an unused WASM import; native v2 qualifies their correction. All failed trials, sources, patches and observed binaries are retained.',
        'test_totals': totals,
        'bindings': 'Actual C f64, C++ and Python consumers execute empty composition, primitive failures and transform failures; actual generated Node WASM and strict TypeScript checks pass. Opaque-real FFI tests exercise undecided transform status without publishing a handle.',
        'resources': 'Index validation adds O(triangles) traversal; non-affine transforms retain O(vertices) projective signs and scan triangles for sign changes. No internal total-work, heap-byte or cancellation bound is claimed. Test commands use 1800-second external wall containment. No latency or allocation improvement is claimed.',
        'limitations': ['Eight existing ignored tests remain unexecuted: one CSGRS rendering test and seven Hypermesh external-fixture/manual scale or memory tests.', 'This is not the complete workflow matrix, sanitizer admission, clean external release/MSRV validation, or an optimization comparison.', 'Go and the separate native JS loader remain unqualified.', 'Bounds, positioning, distributions, projection/slicing and other legacy convenience failures remain open, as do end-to-end provenance, resource limits, strict workflow completion and prior scalar/native admission gaps.'],
        'evidence': str(output.relative_to(ROOT)),
        'implementation_commit': 'fc99c3fb6',
        'plan_complete': False,
    }
    ledger_path = ROOT / 'docs/properties-status.json'
    raw = ledger_path.read_text()
    before = v.sha(ledger_path)
    ledger = json.loads(raw)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    outer = lambda data: {key: value for key, value in data.items() if key != 'current_work'}
    assert outer(ledger) == outer(baseline)
    ledger['current_work']['transform_failure_visibility'] = entry
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
        'ledger_before_sha256': before, 'ledger_after_sha256': v.sha(ledger_path),
        'only_current_work_changed_in_ledger': True, 'full_plan_complete': False,
    })
    print(json.dumps({'qualification_recorded': True, 'executables_verified': len(binaries), 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
