"""Close this fidelity correction without broadening any completion claim."""
import csv
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
    for name, digest in values.items():
        assert name not in target or target[name] == digest, name
        target[name] = digest


def counts(path):
    rows = re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', path.read_text())
    return dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'],
                    [sum(int(row[i]) for row in rows) for i in range(5)]))


def main():
    output = E / 'closure-v1.json'
    assert not output.exists()
    prior_path = ROOT / 'docs/evidence/2026-10-15-curve-constructor-failures/closure-v1.json'
    prior = json.loads(prior_path.read_text())
    artifacts = dict(prior['preserved_executables_sha256'])
    pending = prior['pending_native_sha256']
    assert len(pending) == 35 and all(sha(name) == digest for name, digest in pending.items())
    assert all(sha(name) == digest for name, digest in prior['evidence_files_sha256'].items())
    results, totals = {}, {}
    for result_path in sorted(E.glob('*/result.json')):
        directory = result_path.parent
        phase = directory.name
        if phase.startswith('cost-') or re.fullmatch(r'render-v\d+', phase):
            continue
        result = json.loads(result_path.read_text())
        protocol = json.loads((directory / 'protocol.json').read_text())
        assert protocol['runner_sha256'] == sha(E / 'validate_v1.py')
        assert protocol['commands'] == json.loads((E / (phase + '.json')).read_text())
        assert result['passed'] or result['commands'][-1]['exit_code'] != 0
        merge(artifacts, protocol['preserved_prior_executables_sha256'])
        results[phase], totals[phase] = result, {}
        for row in result['commands']:
            assert row['sources_unchanged']
            merge(artifacts, row['binaries_sha256'])
            for stream in ['stdout', 'stderr', 'resources']:
                assert sha(directory / (row['name'] + '.' + stream)) == row[stream + '_sha256']
            totals[phase][row['name']] = counts(directory / (row['name'] + '.stdout'))
    assert totals['baseline-v1']['regressions']['failed'] == 3
    assert totals['candidate-v2']['regressions']['failed'] == 1
    assert totals['candidate-v3']['regressions']['passed'] == 10
    assert results['native-v1']['passed'] and len(results['native-v1']['commands']) == 13
    assert totals['native-v1']['all-feature-tests']['passed'] == 263
    assert totals['native-v1']['all-feature-tests']['ignored'] == 1
    for row in results['native-v1']['commands']:
        assert not re.search(r'^warning(?:\[|:)', (E / 'native-v1' / (row['name'] + '.stderr')).read_text(), re.M)
    final = v.sources()
    assert final == json.loads(gzip.decompress((E / 'native-v1/source-hashes.json.gz').read_bytes()))
    native = ROOT / 'target/rounded-rectangle-fidelity/native-v1/release'
    merge(artifacts, {str(p.relative_to(ROOT)): sha(p) for p in [native / 'examples/readme_renders']})

    special = {}
    for phase in ['render-v1', 'render-v2', 'cost-v1']:
        directory = E / phase
        result = json.loads((directory / 'result.json').read_text())
        protocol = json.loads((directory / 'protocol.json').read_text())
        assert result['passed'] == (phase != 'render-v1')
        if phase == 'render-v1':
            assert len(result['commands']) == 1 and result['commands'][0]['exit_code'] == 1
        merge(artifacts, protocol['inputs_sha256'])
        for row in result['commands']:
            assert row['sources_unchanged']
            for stream, digest in row['streams_sha256'].items():
                assert sha(directory / (row['name'] + '.' + stream)) == digest
        if result['passed']:
            merge(artifacts, {name: row['sha256'] for name, row in json.loads((directory / 'artifacts.json').read_text()).items()})
        special[phase] = result
    image = json.loads((E / 'image-migration-v1.json').read_text())
    assert sha(image['path']) == sha(image['generated_path']) == sha(image['repeat']) == image['generated_sha256']
    bench_lines = (E / 'native-v1/benchmark-consumer.stdout').read_text().splitlines()
    header = next(i for i, line in enumerate(bench_lines) if line.startswith('engine,temperature,'))
    bench = list(csv.DictReader(bench_lines[header:]))
    assert len(bench) == 3 and all(row['case'] == 'rounded_rectangle' and row['iterations'] == '8' for row in bench)
    cost = special['cost-v1']
    assert len(cost['commands']) == 19
    samples = {}
    for row in cost['commands']:
        if 'sample' not in row:
            continue
        sample = row['sample']
        assert sample['completed'] == sample['iterations'] == 128
        assert sample['certainty'] == 'Certified'
        key = sample['shape'] + '-' + sample['cache']
        samples.setdefault(key, []).append(sample)
    assert len(samples) == 6 and all(len(rows) == 3 for rows in samples.values())
    medians = {key: {metric: statistics.median(row[metric] for row in rows)
                     for metric in ['nanos', 'allocations', 'requested_bytes', 'retained_byte_delta', 'peak_live_bytes_above_start']}
               for key, rows in samples.items()}
    assert all(sha(name) == digest for name, digest in artifacts.items())
    print(json.dumps({'protected_artifacts_verified': len(artifacts)}), flush=True)

    # Recheck the same closed-compiler preservation manifests as the prior closure.
    compiler = {}
    removed, linked = 0, 0
    for manifest in prior_path.parent.glob('removed-*.json.gz'):
        rows = json.loads(gzip.decompress(manifest.read_bytes()))
        removed += len(rows)
        for name, row in rows.items():
            assert not Path(name).exists()
            merge(compiler, {row['retained_original']: row['sha256']})
    for manifest in prior_path.parent.glob('linked-*.json.gz'):
        rows = json.loads(gzip.decompress(manifest.read_bytes()))
        linked += len(rows)
        for name, row in rows.items():
            merge(compiler, {name: row['sha256'], row['retained_original']: row['sha256']})
    grandprior = ROOT / 'docs/evidence/2026-10-15-curve-metaball-failures'
    for manifest in grandprior.glob('removed-*.json.gz'):
        for row in json.loads(gzip.decompress(manifest.read_bytes())).values():
            merge(compiler, {row['retained_original']: row['sha256']})
    assert all(sha(name) == digest for name, digest in compiler.items())
    assert len(compiler) == prior['compiler_paths_verified']
    print(json.dumps({'closed_compiler_paths_verified': len(compiler)}), flush=True)

    entry = {
        'updated_on': '2026-10-15',
        'decision': 'kept: corrects the represented sampled set. Output geometry, vertex counts and derived artifacts intentionally change. This is not an optimization or a matched comparison with the old incorrect result.',
        'status': 'Rounded rectangle retains nonzero straight sides and uses existing clamp facts to omit only coincident endpoints; locally qualified correction.',
        'properties': ['P1', 'P8', 'P9', 'P17', 'P18', 'P20', 'P25', 'P26', 'P27', 'P35', 'P36', 'P37', 'P39'],
        'owners': ['CSGRS sampled rounded-rectangle geometry', 'Hypercurve contour and topology validation', 'Gerber and README rendering boundaries'],
        'domain': 'Nonnegative width, height, radius. Radius clamps to half the smaller dimension. Validated zero dimensions give canonical empty; zero radius or zero corner-segment count gives the sharp rectangle. Positive n gives n equal-angle chords per quarter-circle and all nonzero tangent straight sides; at most 4*(n+1) points, checked before allocation. No analytic fidelity bound is claimed.',
        'migration': 'No API signature changes. Rounded rectangles and Gerber obrounds can gain up to four tangent vertices. One-chord 4x2 radius-1/2 area changes from incorrect 4 to 15/2; clamped 4x2/2x4 capsules from 4 to 6. Derived exports/previews change. Regenerate artifacts; docs/rounded_rectangle.png is regenerated from the original full fixture and unchanged renderer functions.',
        'semantic_evidence': [
            'Three baseline failures, four initial passing regressions and the unchanged 49-test strict shape catalog; ten expanded regressions pass. One preserved test-only failure used structural equality on reordered sums; final tests require strict mathematical equality.',
            'Independent exact rational, algebraic and symbolic areas; explicit one-chord vertices and full-resolution benchmark/README tangent endpoints. Both clamping directions, full-radius and sharp controls retained.',
            'Strict translate, subtract interior hole, restore by union, subtract to empty and height-three extrusion: exact area 13/2, signed volume 39/2 and separate closed-manifold check. OBJ round-trip preserves this dyadic fixture exactly. Gerber obround import/export preserves exact tangent boundary points and independent inside/outside classifications.',
            'Approximate clamp evidence remains qualified across strict/approximate/strict requests. Capacity overflow remains typed before empty shortcuts. Existing constructor/cache/context and adversarial regressions remain enabled.',
        ],
        'test_totals': totals['native-v1'],
        'validation': 'All 13 final native commands passed: root and fuzz formatting; all-target strict Clippy for all features, no defaults and curve-only; all-feature library/integration tests; minimal and curve library/affected integration tests; fuzz-target Clippy; warning-free docs and doctests; original rounded-rectangle benchmark consumer; full renderer build. Direct language bindings do not expose rounded_rectangle or Gerber import; no binding implementation/signature changed in this slice.',
        'render': {'evidence': str((E / 'render-v2').relative_to(ROOT)), 'migration': image, 'claim': 'Two byte-identical focused renders, original dimensions and 12 corner segments. All renderer functions and constants unchanged. Full image-corpus regression remains open. First harness build omitted the hypermesh extern; failed trial preserved.'},
        'cost_probe': {'kind': 'focused corrected-output measurement, not a speedup or supported-corpus comparison', 'median_samples': medians, 'operations_per_sample': 128, 'fresh_process_samples_per_case': 3, 'measured_completed_operations': 2304, 'library_sizes': json.loads((E / 'cost-v1/library_sizes.json').read_text()), 'evidence': str((E / 'cost-v1').relative_to(ROOT)), 'claim': 'All sampled operations completed Certified with materialized boundaries, exact tangent assertions, and independent area for one-chord cases. Allocation wrapper includes instrumentation; ring-cache warm and FIFO-miss states do not imply globally cold scalar caches.'},
        'resources': 'Primary point capacity and reservation checked. Serial offline trials, two Cargo jobs, external 1800-second wall and 8-GiB address-space containment per process. Scalar growth, total work, secondary allocations, retained heap-byte budgets and cancellation remain internally unbounded or unmeasured. No native heap bound or broad performance claim.',
        'limitations': [
            'Other constructors and legacy wrappers still need typed-failure migration. Operation-local certainty does not establish end-to-end provenance or fidelity to analytic source geometry.',
            'The 35 original pending native files are byte-identical and uncommitted, pending separate admission. This is worktree qualification, not clean-checkout or published-package validation.',
            'Full CI/pre-push, MSRV/release/external-package and full image-corpus validation, directed fuzz/sanitizer campaigns, and the rest of the properties plan remain open. No push occurred.',
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
    ledger['current_work']['rounded_rectangle_fidelity'] = entry
    ledger['current_work']['updated_on'] = '2026-10-15'
    ledger['current_work']['plan_complete'] = False
    match = re.search(r'(?m)^([ \t]*)"current_work": *', raw)
    start = match.end()
    _, end = json.JSONDecoder().raw_decode(raw[start:])
    replacement = json.dumps(ledger['current_work'], indent=2).replace('\n', '\n' + match.group(1))
    updated = raw[:start] + replacement + raw[start + end:]
    assert json.loads(updated) == ledger and outer(json.loads(updated)) == outer(baseline)
    assert updated[:start] == raw[:start] and updated[start + len(replacement):] == raw[start + end:]
    ledger_path.write_text(updated)
    files = {str(p.relative_to(ROOT)): sha(p) for p in sorted(E.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
    v.dump(output, {
        'qualification': entry, 'closure_runner_sha256': sha(Path(__file__)),
        'prior_qualification_sha256': sha(prior_path), 'phase_results': results,
        'phase_test_totals': totals, 'render_and_cost_results': special,
        'benchmark_consumer_rows': bench, 'preserved_executables_sha256': artifacts,
        'evidence_files_sha256': files, 'pending_native_sha256': pending,
        'removed_duplicate_paths_verified': removed, 'linked_duplicate_paths_verified': linked,
        'compiler_paths_verified': len(compiler), 'ledger_before_sha256': before,
        'ledger_after_sha256': sha(ledger_path), 'only_current_work_changed_in_ledger': True,
        'full_plan_complete': False,
    })
    print(json.dumps({'qualification_recorded': True, 'evidence_files_verified': len(files),
                      'artifacts_verified': len(artifacts), 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
