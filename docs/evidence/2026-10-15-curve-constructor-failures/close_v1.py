"""Close constructor acceptance once, without changing older evidence or ledger keys."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import statistics
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
spec = importlib.util.spec_from_file_location('validation', E / 'validate_v1.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)
HASHES = {}


def sha(path):
    path = ROOT / path
    st = path.stat()
    key = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
    if key not in HASHES:
        with path.open('rb') as stream:
            HASHES[key] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return HASHES[key]


def merge(target, values):
    for key, value in values.items():
        assert key not in target or target[key] == value, key
        target[key] = value


def counts(path):
    rows = re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', path.read_text())
    return dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'],
                    [sum(int(row[i]) for row in rows) for i in range(5)]))


def main():
    output = E / 'closure-v1.json'
    assert not output.exists()
    prior_path = ROOT / 'docs/evidence/2026-10-15-curve-metaball-failures/closure-v1.json'
    prior = json.loads(prior_path.read_text())
    artifacts = dict(prior['preserved_executables_sha256'])
    pending = prior['pending_native_sha256']
    assert all(sha(name) == digest for name, digest in pending.items())
    results, totals = {}, {}
    for result_path in sorted(E.glob('*/result.json')):
        directory = result_path.parent
        phase = directory.name
        if phase.startswith('cost-'):
            continue
        result = json.loads(result_path.read_text())
        protocol = json.loads((directory / 'protocol.json').read_text())
        assert protocol['runner_sha256'] == sha(E / 'validate_v1.py')
        assert protocol['commands'] == json.loads((E / (phase + '.json')).read_text())
        assert result['passed'] or result['commands'][-1]['exit_code'] != 0
        results[phase], totals[phase] = result, {}
        for row in result['commands']:
            assert row['sources_unchanged']
            merge(artifacts, row['binaries_sha256'])
            for stream in ['stdout', 'stderr', 'resources']:
                assert sha(directory / (row['name'] + '.' + stream)) == row[stream + '_sha256']
            totals[phase][row['name']] = counts(directory / (row['name'] + '.stdout'))
        if phase in {'native-v1', 'bindings-v1'}:
            assert result['passed'] and len(result['commands']) == len(protocol['commands'])
            assert all(not re.search(r'^warning(?:\[|:)', (directory / (row['name'] + '.stderr')).read_text(), re.M)
                       for row in result['commands'])
    assert totals['baseline-v1']['regressions']['failed'] == 9
    assert totals['candidate-v1']['regressions']['passed'] == 14
    assert totals['native-v1']['all-feature-tests']['passed'] == 253
    assert len(results['native-v1']['commands']) == 19
    assert len(results['bindings-v1']['commands']) == 39
    final = v.sources()
    for phase in ['native-v1', 'bindings-v1']:
        assert final == json.loads(gzip.decompress((E / phase / 'source-hashes.json.gz').read_bytes()))
    native = ROOT / 'target/curve-constructor-failures/native-v1'
    bindings = ROOT / 'target/curve-constructor-failures/bindings-v1'
    outputs = [native / 'release/libcsgrs_ffi.so', bindings / 'go-bindings.test',
               bindings / 'wasm32-unknown-unknown/wasm/csgrs.wasm']
    outputs += [bindings / 'wasm-bindings-node' / name for name in ['csgrs.js', 'csgrs.d.ts', 'csgrs_bg.wasm', 'geometry_certainty.ts']]
    outputs += [bindings / (fixture + '-' + language)
                for fixture in ['empty_composition', 'primitive_failures', 'transform_failures', 'bounds_failures', 'exact_linear_modeling', 'curve_constructor_failures']
                for language in ['c', 'cpp']]
    merge(artifacts, {str(path.relative_to(ROOT)): sha(path) for path in outputs})
    for path in E.glob('*-preserve.json'):
        merge(artifacts, json.loads(path.read_text()))

    failed_cost_path = E / 'cost-v1'
    failed_cost = json.loads((failed_cost_path / 'result.json').read_text())
    assert not failed_cost['passed'] and len(failed_cost['commands']) == 1
    assert failed_cost['commands'][0]['exit_code'] == 1
    for row in failed_cost['commands']:
        assert row['sources_unchanged']
        for stream, digest in row['streams_sha256'].items():
            assert sha(failed_cost_path / (row['name'] + '.' + stream)) == digest
    merge(artifacts, json.loads((failed_cost_path / 'protocol.json').read_text())['inputs_sha256'])
    cost_path = E / 'cost-v2'
    cost = json.loads((cost_path / 'result.json').read_text())
    cost_protocol = json.loads((cost_path / 'protocol.json').read_text())
    assert cost['passed'] and len(cost['commands']) == 62
    merge(artifacts, cost_protocol['inputs_sha256'])
    merge(artifacts, {name: row['sha256'] for name, row in json.loads((cost_path / 'artifacts.json').read_text()).items()})
    samples = {}
    for row in cost['commands']:
        assert row['sources_unchanged'] and row['exit_code'] == 0
        for stream, digest in row['streams_sha256'].items():
            assert sha(cost_path / (row['name'] + '.' + stream)) == digest
        if 'sample' in row:
            sample = row['sample']
            assert sample['completed'] == sample['iterations'] == 1024
            key = '-'.join(row['name'].split('-')[1:])
            samples.setdefault(key, []).append(sample)
    summary = {key: {metric: statistics.median(row[metric] for row in rows)
                     for metric in ['nanos', 'allocations', 'requested_bytes', 'retained_byte_delta', 'peak_live_bytes_above_start']}
               for key, rows in samples.items()}
    assert all(sha(name) == digest for name, digest in artifacts.items())

    compiler = {}
    removed, linked = 0, 0
    for manifest in E.glob('removed-*.json.gz'):
        rows = json.loads(gzip.decompress(manifest.read_bytes()))
        removed += len(rows)
        for name, row in rows.items():
            assert not Path(name).exists()
            merge(compiler, {row['retained_original']: row['sha256']})
    for manifest in E.glob('linked-*.json.gz'):
        rows = json.loads(gzip.decompress(manifest.read_bytes()))
        linked += len(rows)
        for name, row in rows.items():
            merge(compiler, {name: row['sha256'], row['retained_original']: row['sha256']})
    for manifest in prior_path.parent.glob('removed-*.json.gz'):
        for row in json.loads(gzip.decompress(manifest.read_bytes())).values():
            merge(compiler, {row['retained_original']: row['sha256']})
    assert all(sha(name) == digest for name, digest in compiler.items())

    entry = {
        'updated_on': '2026-10-15',
        'decision': 'kept: restores typed failure and reusable empties. The focused warm-rectangle median increased from 1.277 to 1.549 microseconds; median allocation counts/byte requests match. The correctness migration is not a performance-only optimization.',
        'status': 'Foundational planar constructors and existing context compounds preserve typed failure and reusable canonical empties; locally qualified API migration.',
        'properties': ['P1', 'P3', 'P4', 'P6', 'P8', 'P9', 'P13', 'P17', 'P18', 'P20', 'P25', 'P28', 'P30', 'P35', 'P36', 'P37', 'P39'],
        'owners': ['CSGRS modeling parameter validation and composition', 'Hypercurve native contour/region validity', 'Scalar adapters, Gerber, Hypercircuit and language boundaries'],
        'domain': 'Nonnegative rectangle/square/right-triangle dimensions; sampled circle/ellipse/ngon require at least three samples including zero area; valid nonempty polygon rings; nonnegative rounded rectangle dimensions/radius with checked sampling capacity and existing clamp. Keyhole, Reuleaux, ring, crescent and circle-cut compounds retain the positive/range restrictions detailed in the evidence README.',
        'migration': 'Basic constructors return Result<CurveRegion2, ValidationError>; context variants and existing compounds preserve GeometryOutcome with ValidationError. Negative, malformed, undecided and capacity failures no longer publish empty success. Every relevant argument is checked before canonical zero-area success. Native causes survive scalar/binding/IO/downstream wrappers. Hypercircuit materialization/handoff diagnostic enums retain PartialEq but lose Eq. Language binding signatures are unchanged.',
        'semantic_evidence': [
            'Nine baseline failures reproduced; 14 expanded regressions establish typed categories, cause preservation, recovery, empty union/subtraction/extrusion composition and invalid companions to zero dimensions.',
            'Independent rectangle area 35/6, triangle area 35/12, sampled circle area 2 and ellipse area 4; exact triangulated area and height-three signed volume, with separate closed manifold checks.',
            'Strict/approximate/strict sequences distinguish selecting policy from consuming terminal evidence. Cache admits only successful strict proofs, with owned structural keys, cloned values and 64-entry thread-local FIFO; cold/warm, failure, eviction, reset, transformed-output and thread isolation tests pass.',
            'HyperCircuit rectangle materialization retains cause and consumed certainty; the public keepout handoff records typed construction omission and recovers after corrected geometry. Actual C/C++/Python/Go/WASM consumers exercise invalid constructors, canonical empties and recovery.',
            'Existing strict shape/gear catalog, README inputs, adversarial fixtures and sampling resolutions are preserved; no ignored test was added.',
        ],
        'test_totals': totals['native-v1'],
        'validation': 'All 19 native and 39 binding commands passed after the final implementation edit. Includes formatting, all-target/all-feature strict Clippy for four affected crates, CSGRS library/integration, affected Hypercircuit/HyperDRC tests, feature-minimal/curve configurations, fuzz-target Clippy, warning-free docs/doctests and real binding consumers. Failed quota and missing-import trials remain preserved.',
        'cost_probe': {'kind': cost_protocol['kind'], 'median_samples': summary, 'raw_evidence': str(cost_path.relative_to(ROOT)), 'library_sizes': json.loads((cost_path / 'library_sizes.json').read_text()), 'claim': 'Proportionate measurement on three small successful strict workloads, with exact-area assertions and materialized output. Instrumented allocation counts and binary sizes; no supported-corpus speedup or native heap bound claim.'},
        'resources': 'Checked primary capacities/reservations. Serial validation with two Cargo jobs, offline resolution, external 1800-second wall and 8-GiB per-process address-space containment; Node/WASM exempts the address-space limit. Primary scalar growth, secondary allocations, total work, retained heap bytes and cancellation remain internally unbounded or unmeasured. Closed compiler targets are immutable; seed copies have separate inodes.',
        'limitations': [
            'Other sampled shapes, gear/text constructors and legacy transforms still need typed-failure migration. Operation-local certainty is not end-to-end provenance; sampled polygons do not carry analytic-fidelity bounds.',
            'This is worktree evidence including the unchanged original 35 pending native files, not a clean-checkout or published-package qualification. Their separate admission and original scalar/native gaps remain open.',
            'Full CI/pre-push, release/MSRV/external-package validation, directed fuzz/sanitizer campaigns and regenerated image/export artifacts remain open. No push occurred.',
            'The focused cost probe uses a shared host and instrumented Rust allocation wrapper. It does not count every native allocation or qualify performance across the supported corpus.',
        ],
        'implementation_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'evidence': str(output.relative_to(ROOT)), 'plan_complete': False,
    }
    ledger_path = ROOT / 'docs/properties-status.json'
    raw = ledger_path.read_text()
    before = sha(ledger_path)
    ledger = json.loads(raw)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    outer = lambda value: {key: item for key, item in value.items() if key != 'current_work'}
    assert outer(ledger) == outer(baseline)
    ledger['current_work']['curve_constructor_failures'] = entry
    ledger['current_work']['updated_on'] = '2026-10-15'
    ledger['current_work']['plan_complete'] = False
    match = re.search(r'(?m)^([ \t]*)"current_work": *', raw)
    start = match.end()
    _, end = json.JSONDecoder().raw_decode(raw[start:])
    replacement = json.dumps(ledger['current_work'], indent=2).replace('\n', '\n' + match.group(1))
    updated = raw[:start] + replacement + raw[start + end:]
    assert json.loads(updated) == ledger and outer(json.loads(updated)) == outer(baseline)
    ledger_path.write_text(updated)
    files = {str(path.relative_to(ROOT)): sha(path) for path in sorted(E.rglob('*')) if path.is_file() and '__pycache__' not in path.parts}
    v.dump(output, {
        'qualification': entry, 'closure_runner_sha256': sha(Path(__file__)),
        'prior_qualification_sha256': sha(prior_path), 'phase_results': results,
        'phase_test_totals': totals, 'cost_results': {'cost-v1': failed_cost, 'cost-v2': cost},
        'preserved_executables_sha256': artifacts, 'evidence_files_sha256': files,
        'pending_native_sha256': pending, 'removed_duplicate_paths_verified': removed,
        'linked_duplicate_paths_verified': linked, 'compiler_paths_verified': len(compiler),
        'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger_path),
        'only_current_work_changed_in_ledger': True, 'full_plan_complete': False,
    })
    print(json.dumps({'qualification_recorded': True, 'artifacts_verified': len(artifacts),
                      'compiler_paths_verified': len(compiler), 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
