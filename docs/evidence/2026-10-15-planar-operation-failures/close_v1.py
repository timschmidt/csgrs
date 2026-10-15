"""Record qualified planar changes without relabeling failed downstream gates."""
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


def benchmark_rows(path):
    lines = path.read_text().splitlines()
    header = next(i for i, line in enumerate(lines) if line.startswith('engine,temperature,'))
    return list(csv.DictReader(lines[header:]))


def main():
    output = E / 'closure-v1.json'
    assert not output.exists()
    prior_path = ROOT / 'docs/evidence/2026-10-15-sampled-profile-failures/closure-v1.json'
    prior = json.loads(prior_path.read_text())
    pending = prior['pending_native_sha256']
    assert len(pending) == 35 and all(sha(name) == digest for name, digest in pending.items())
    assert all(sha(name) == digest for name, digest in prior['evidence_files_sha256'].items())
    artifacts = dict(prior['preserved_executables_sha256'])
    merge(artifacts, json.loads((E / 'baseline-artifacts-v1.json').read_text()))
    phases = ['baseline-v1', 'baseline-v2', 'baseline-v3', 'candidate-v1', 'native-v1',
              'downstream-v1', 'downstream-remainder-v2', 'downstream-baseline-v1',
              'native-v2', 'bindings-v1', 'wasm-v1', 'cost-build-v1']
    results, totals = {}, {}
    for phase in phases:
        directory = E / phase
        result = json.loads((directory / 'result.json').read_text())
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
    assert totals['baseline-v2']['regressions']['failed'] == 10
    assert totals['baseline-v3']['regressions']['failed'] == 14
    assert totals['candidate-v1']['regressions']['passed'] == 17
    assert totals['native-v1']['ffi-tests']['failed'] == 1
    assert totals['downstream-v1']['regressions']['passed'] == 20
    assert totals['downstream-v1']['hypercircuit-tests']['failed'] == 6
    assert totals['downstream-remainder-v2']['workflow']['failed'] == 1
    assert totals['downstream-baseline-v1']['downstream-failures']['failed'] == 7
    assert results['native-v2']['passed'] and len(results['native-v2']['commands']) == 22
    assert results['bindings-v1']['passed'] and len(results['bindings-v1']['commands']) == 43
    assert results['wasm-v1']['passed'] and len(results['wasm-v1']['commands']) == 2
    assert results['cost-build-v1']['passed'] and len(results['cost-build-v1']['commands']) == 1
    assert totals['native-v2']['all-feature-tests']['passed'] == 313
    assert totals['native-v2']['all-feature-tests']['ignored'] == 1
    assert totals['native-v2']['minimal-tests']['passed'] == 38
    assert totals['native-v2']['curve-tests']['passed'] == 128
    final = v.sources()
    for phase in ['downstream-v1', 'downstream-remainder-v2', 'native-v2', 'bindings-v1', 'wasm-v1', 'cost-build-v1']:
        assert final == json.loads(gzip.decompress((E / phase / 'source-hashes.json.gz').read_bytes()))
        for row in results[phase]['commands']:
            assert not re.search(r'^warning(?:\[|:)', (E / phase / (row['name'] + '.stderr')).read_text(), re.M)
    comparison = json.loads((E / 'downstream-comparison-v1/result.json').read_text())
    assert comparison['runner_sha256'] == sha(E / 'compare_downstream_v1.py')
    assert len(comparison['failures']) == 7
    merge(artifacts, comparison['inputs_sha256'])
    baseline_source = ROOT / 'target/planar-operation-failures/downstream-baseline-source-v1'
    source_hashes = json.loads(gzip.decompress((E / 'downstream-baseline-source-v1.json.gz').read_bytes()))
    assert len(source_hashes) == comparison['baseline_sources_unchanged'] == 2593
    merge(artifacts, {str(baseline_source / name): digest for name, digest in source_hashes.items()})
    for name, row in comparison['failures'].items():
        suffix = 'zip' if name.startswith('default_manufacturing') else 'json'
        for label in ['actual', 'expected']:
            assert sha(E / 'downstream-comparison-v1' / f'{name}-{label}.{suffix}') == row[label + '_sha256']
    native = ROOT / 'target/planar-operation-failures/native-v2/release'
    wasm = ROOT / 'target/planar-operation-failures/wasm-v1'
    bindings = ROOT / 'target/planar-operation-failures/bindings-v1'
    built = [native / 'examples/readme_renders', native / 'libcsgrs_ffi.so', bindings / 'go-bindings.test']
    built += list(bindings.glob('*-c')) + list(bindings.glob('*-cpp'))
    built += [wasm / name for name in ['wasm32-unknown-unknown/wasm/csgrs.wasm',
              'wasm-bindings-node/csgrs.js', 'wasm-bindings-node/csgrs.d.ts',
              'wasm-bindings-node/csgrs_bg.wasm', 'wasm-bindings-node/geometry_certainty.ts']]
    merge(artifacts, {str(path.relative_to(ROOT)): sha(path) for path in built})
    assert sha(wasm / 'wasm-bindings-node/geometry_certainty.ts') == sha('tests/wasm/geometry_certainty.ts')
    bench = {name: benchmark_rows(E / 'native-v2' / (name + '.stdout'))
             for name in ['benchmark-consumer', 'profile-benchmark-consumer']}
    assert len(bench['benchmark-consumer']) == 6 and len(bench['profile-benchmark-consumer']) == 3

    failed_cost = json.loads((E / 'cost-v1/result.json').read_text())
    failed_protocol = json.loads((E / 'cost-v1/protocol.json').read_text())
    assert not failed_cost['passed'] and len(failed_cost['commands']) == 2
    assert failed_cost['commands'][1]['exit_code'] == 1
    assert 'E0308' in (E / 'cost-v1/candidate-build.stderr').read_text()
    merge(artifacts, failed_protocol['inputs_sha256'])
    merge(artifacts, json.loads((E / 'cost-failure-artifacts-v1.json').read_text()))
    for row in failed_cost['commands']:
        assert row['sources_unchanged']
        for stream, digest in row['streams_sha256'].items():
            assert sha(E / 'cost-v1' / (row['name'] + '.' + stream)) == digest
    cost = json.loads((E / 'cost-v2/result.json').read_text())
    protocol = json.loads((E / 'cost-v2/protocol.json').read_text())
    assert cost['passed'] and len(cost['commands']) == 114
    merge(artifacts, protocol['inputs_sha256'])
    merge(artifacts, {name: row['sha256'] for name, row in json.loads((E / 'cost-v2/artifacts.json').read_text()).items()})
    samples, pairs = {}, {}
    for row in cost['commands']:
        assert row['sources_unchanged']
        for stream, digest in row['streams_sha256'].items():
            assert sha(E / 'cost-v2' / (row['name'] + '.' + stream)) == digest
        if 'sample' not in row:
            continue
        sample = row['sample']
        assert sample['completed'] == sample['iterations'] == 128
        assert sample['policy'] == 'strict' and sample['certainty'] == 'Certified'
        pair, label = row['name'].rsplit('-', 1)
        pairs.setdefault(pair, {})[label] = sample
        samples.setdefault(sample['shape'] + '-' + sample['cache'], {}).setdefault(label, []).append(sample)
    assert len(pairs) == 48 and len(samples) == 16
    assert all(pair['baseline']['checksum'] == pair['candidate']['checksum'] and
               pair['baseline']['output_bytes'] == pair['candidate']['output_bytes'] for pair in pairs.values())
    exact = json.loads((E / 'cost-v2/exact-rows.json').read_text())
    assert len(exact) == 8 and sum(row['rows'] for row in exact.values()) == 1024
    for case, row in exact.items():
        for label in ['baseline', 'candidate']:
            assert sha(E / 'cost-v2' / f'{case}-{label}-dump.stdout') == row['sha256']
    medians = {key: {label: {metric: statistics.median(row[metric] for row in rows)
                            for metric in ['nanos', 'allocations', 'requested_bytes', 'retained_byte_delta', 'peak_live_bytes_above_start']}
                      for label, rows in pair.items()} for key, pair in samples.items()}
    ratios = {key: pair['candidate']['nanos'] / pair['baseline']['nanos'] for key, pair in medians.items()}
    assert all(sha(name) == digest for name, digest in artifacts.items())
    print(json.dumps({'protected_artifacts_verified': len(artifacts)}), flush=True)

    compiler = {}
    removed = linked = 0
    for folder in ['2026-10-15-curve-constructor-failures', '2026-10-15-curve-metaball-failures',
                   '2026-10-15-sampled-profile-failures', '2026-10-15-planar-operation-failures']:
        directory = ROOT / 'docs/evidence' / folder
        for manifest in directory.glob('removed-*.json.gz'):
            rows = json.loads(gzip.decompress(manifest.read_bytes()))
            removed += len(rows)
            for name, row in rows.items():
                assert not Path(name).exists()
                merge(compiler, {row['retained_original']: row['sha256']})
        for manifest in directory.glob('linked-*.json.gz'):
            rows = json.loads(gzip.decompress(manifest.read_bytes()))
            linked += len(rows)
            for name, row in rows.items():
                merge(compiler, {name: row['sha256'], row['retained_original']: row['sha256']})
    assert len(compiler) >= prior['compiler_paths_verified']
    assert all(sha(name) == digest for name, digest in compiler.items())
    print(json.dumps({'compiler_paths_verified': len(compiler)}), flush=True)
    implementation = json.loads((E / 'implementation-paths.json').read_text())
    changed = subprocess.check_output(['git', 'diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD'], cwd=ROOT, text=True).splitlines()
    assert set(changed) == set(implementation)
    source_sizes = {name: {'baseline_bytes': len(subprocess.check_output(['git', 'show', '227792db1:' + name], cwd=ROOT)),
                           'candidate_bytes': (ROOT / name).stat().st_size}
                    for name in ['src/curve/native.rs', 'src/adapter/curve.rs', 'src/wasm/curve_js.rs',
                                 'crates/hypercircuit/src/materialize.rs', 'crates/hypercircuit/src/fabrication.rs']}
    entry = {
        'updated_on': '2026-10-15',
        'decision': 'kept: typed-failure and homogeneous-meaning repair; not a performance-only change',
        'status': 'Affected planar APIs and consumers locally qualified. Seven broader Hypercircuit fixture failures reproduce byte-for-byte on the starting worktree and remain failed.',
        'properties': ['P1', 'P4', 'P6', 'P8', 'P9', 'P17', 'P18', 'P19', 'P20', 'P25', 'P26', 'P27', 'P28', 'P29', 'P30', 'P35', 'P36', 'P37', 'P39'],
        'owners': ['CSGRS public planar operations and homogeneous matrix admission', 'Hypercurve affine topology and classification', 'Scalar/FFI/WASM diagnostic boundaries', 'Hypercircuit materialization certainty aggregation'],
        'domain': 'Nonsingular affine restriction to z=0 with constant nonzero homogeneous weight and image in the XY plane. Normalize six XY coefficients by the weight, including negative weights; the third column is irrelevant. Native Hypercurve owns determinant and topology. Perspective/out-of-plane maps are Unsupported, zero weight with nonzero coordinate image is Unbounded, and an all-zero homogeneous image is Invalid. Unproved constraints are Undecided or explicitly qualified under the selected approximate policy. Invalid arguments are checked on canonical empty inputs too.',
        'migration': 'curve transformed/translated/rotated/scaled now return Result<CurveRegion2, ValidationError>; try_ and context variants preserve the same native causes. contains_xy returns Result<Option<bool>, ValidationError>, with None reserved for boundary. Scalar adapters preserve all three input components and apply X/Y/Z rotations in order. Nonzero Z translation and out-of-plane rotations are rejected; plane-preserving half-turns perform reflections. Gerber, Hypercircuit, FFI and WASM retain typed causes. FabricationPackageError adds Construction and retains PartialEq without Eq.',
        'semantic_evidence': [
            'Unchanged baseline regressions reproduce fourteen defects (3 pass/14 fail). Twenty final regressions pass, with independent exact coordinate substitution, area 18, volume 36, a separate manifold check, holes, affine-cut commutation, empty-result composition and mutation/recovery.',
            'Matrix admission consumes the selected policy and preserves qualification through native identity shortcuts. Strict/approximate/strict does not promote approximate evidence. Definite membership distinguishes inside, outside and boundary from a native undecided blocker.',
            'Hypercircuit translates/scales/rotates through its selected materialization context and aggregates certainty. Qualified empty construction remains qualified through subsequent transforms. Failures cannot become empty fabrication images.',
            'Actual C, C++, Python, Go and generated Node/WASM consumers exercise errors, normalized coordinates, plane-preserving reflections, canonical empty operands and recovery. TypeScript checks the generated declarations.',
        ],
        'test_totals': totals,
        'validation': 'All 22 final native commands, 43 native-binding commands and two WASM commands pass. Native checks include all-feature/minimal/curve-only strict Clippy and tests, fuzz-target Clippy, docs/doctests, original benchmark cases, renderer build, FFI default/all-feature tests and Clippy, and Hypercircuit docs/doctests. Hypercircuit all-feature Clippy passes; its complete library/integration execution has 341 passes and seven fixture failures. All seven actual and expected byte pairs match the reconstructed starting revision with the same 35 pending native files. The failures are retained, not waived or relabeled as passing.',
        'preserved_failed_trials': ['baseline-v1 missing check-cfg in direct-rustc harness', 'baseline-v2/v3 reproduced defects', 'native-v1 all-zero homogeneous image misclassified as Unbounded; existing Invalid expectation retained', 'native-v1 profile benchmark filter selected zero cases and receives no validation credit', 'downstream-remainder-v1 setup encountered stale compiler fingerprints without executables', 'downstream-v1/remainder-v2 and reconstructed baseline retain all seven pre-existing fixture failures', 'cost-v1 candidate rustc E0308 exposed mismatched transitive library identity before any measurements; v1 unit-configuration check is insufficient. cost-build-v1 uses the exact baseline example command for cost-v2'],
        'cost_probe': {'kind': 'focused matched successful-output probe, not a supported-corpus speedup claim',
                       'evidence': str((E / 'cost-v2').relative_to(ROOT)), 'commands': 114,
                       'fresh_process_samples': 96, 'operations_per_sample': 128,
                       'timed_completed_operations': 12288, 'exact_variants_per_revision': 1024,
                       'median_samples': medians, 'candidate_to_baseline_latency_ratios': ratios,
                       'library_sizes': json.loads((E / 'cost-v2/library_sizes.json').read_text()),
                       'claim': 'Same completed strict workloads include construction, operation, independent exact coordinates/area, output materialization, serialization and drop. All exact rows and paired checksums agree. Every favorable and unfavorable sample is retained. Warm adapter translation median is 15.2% higher and warm affine-matrix median is 15.1% higher; other latencies are mixed. Adapter rotation adds 15 allocations and 792 requested bytes per operation; other measured allocation counts match. Shared unpinned host and allocator instrumentation limit timing claims. Warm and varied inputs are distinct; varied does not imply globally cold scalar caches. cost-v1 produced no timings after its dependency mismatch; cost-build-v1 uses the exact baseline example command for cost-v2.'},
        'source_sizes': source_sizes,
        'resources': 'No internal total-work, expression-growth, heap-byte or cancellation bound is claimed. Serial offline trials with two Cargo jobs use external 1800-second wall and 8-GiB per-process address-space containment; Node/WASM retains only wall containment because of virtual reservations. Allocation probes cover the Rust global allocator, not all native allocations or RSS. Identical non-executable compiler files in terminal trials are hard-linked with every path and byte retained; new trials copy to independent inodes.',
        'limitations': ['Seven baseline downstream fixtures remain failing and require native/release admission; no snapshots or original workload resolutions changed.', 'The 35 original pending native files remain byte-identical and uncommitted pending separate admission. This is worktree qualification, not a clean external-package release.', 'Gear/rack/airfoil/text and other convenience failures, end-to-end source provenance, full strict workload completion, internal resource bounds, coverage/fuzz/formal goals and the full workflow/MSRV/release/user-path matrix remain open. No push or CI changes.'],
        'implementation_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'evidence': str(output.relative_to(ROOT)), 'turn_outcome': 'progress', 'plan_complete': False,
    }
    ledger_path = ROOT / 'docs/properties-status.json'
    raw = ledger_path.read_text()
    before = sha(ledger_path)
    ledger = json.loads(raw)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    outer = lambda value: {key: item for key, item in value.items() if key != 'current_work'}
    assert outer(ledger) == outer(baseline)
    ledger['current_work']['planar_operation_failures'] = entry
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
    files = {str(path.relative_to(ROOT)): sha(path) for path in sorted(E.rglob('*'))
             if path.is_file() and '__pycache__' not in path.parts}
    v.dump(output, {'qualification': entry, 'closure_runner_sha256': sha(Path(__file__)),
                   'prior_qualification_sha256': sha(prior_path), 'phase_results': results,
                   'phase_test_totals': totals, 'cost_results': {'cost-v1': failed_cost, 'cost-v2': cost}, 'benchmark_consumer_rows': bench,
                   'exact_cost_rows': exact, 'downstream_baseline_comparison': comparison,
                   'preserved_executables_sha256': artifacts, 'evidence_files_sha256': files,
                   'pending_native_sha256': pending, 'compiler_paths_verified': len(compiler),
                   'removed_duplicate_paths_verified': removed, 'linked_duplicate_paths_verified': linked,
                   'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger_path),
                   'only_current_work_changed_in_ledger': True, 'full_plan_complete': False})
    print(json.dumps({'qualification_recorded': True, 'evidence_files_verified': len(files),
                      'artifacts_verified': len(artifacts), 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
