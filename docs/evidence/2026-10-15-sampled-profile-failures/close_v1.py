"""Close the sampled-profile slice while preserving failed trials and open contracts."""
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
    prior_path = ROOT / 'docs/evidence/2026-10-15-rounded-rectangle-fidelity/closure-v1.json'
    prior = json.loads(prior_path.read_text())
    artifacts = dict(prior['preserved_executables_sha256'])
    migrations = json.loads((E / 'source-input-migrations-v1.json').read_text())
    # A prior renderer recorded a live production source as an immutable input.
    # Archive its original bytes, retaining that proof without freezing the API caller.
    for name, row in migrations.items():
        assert artifacts.pop(name) == row['original_sha256']
        assert sha(row['archive']) == row['original_sha256']
        assert sha(name) == row['current_sha256']
        merge(artifacts, {row['archive']: row['original_sha256']})
    pending = prior['pending_native_sha256']
    assert len(pending) == 35 and all(sha(name) == digest for name, digest in pending.items())
    assert all(sha(name) == digest for name, digest in prior['evidence_files_sha256'].items())
    results, totals = {}, {}
    for phase in ['baseline-v1', 'candidate-v1', 'candidate-v2', 'native-v1', 'wasm-v1']:
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
    assert totals['baseline-v1']['regressions']['passed'] == 9
    assert totals['baseline-v1']['regressions']['failed'] == 15
    assert totals['candidate-v1']['regressions']['passed'] == 23
    assert totals['candidate-v1']['regressions']['failed'] == 1
    assert results['candidate-v2']['passed'] and totals['candidate-v2']['regressions']['passed'] == 30
    assert results['native-v1']['passed'] and len(results['native-v1']['commands']) == 13
    assert results['wasm-v1']['passed'] and len(results['wasm-v1']['commands']) == 2
    assert totals['native-v1']['all-feature-tests']['passed'] == 293
    assert totals['native-v1']['all-feature-tests']['ignored'] == 1
    assert totals['native-v1']['minimal-tests']['passed'] == 38
    assert totals['native-v1']['curve-tests']['passed'] == 108
    assert totals['native-v1']['doctests']['passed'] == 1
    final = v.sources()
    for phase in ['native-v1', 'wasm-v1']:
        assert final == json.loads(gzip.decompress((E / phase / 'source-hashes.json.gz').read_bytes()))
        for row in results[phase]['commands']:
            assert not re.search(r'^warning(?:\[|:)', (E / phase / (row['name'] + '.stderr')).read_text(), re.M)
    native = ROOT / 'target/sampled-profile-failures/native-v1/release'
    wasm = ROOT / 'target/sampled-profile-failures/wasm-v1'
    built = [native / 'examples/readme_renders'] + [wasm / name for name in [
        'wasm32-unknown-unknown/wasm/csgrs.wasm', 'wasm-bindings-node/csgrs.js',
        'wasm-bindings-node/csgrs.d.ts', 'wasm-bindings-node/csgrs_bg.wasm',
        'wasm-bindings-node/geometry_certainty.ts',
    ]]
    merge(artifacts, {str(path.relative_to(ROOT)): sha(path) for path in built})
    assert sha(wasm / 'wasm-bindings-node/geometry_certainty.ts') == sha('tests/wasm/geometry_certainty.ts')
    exact_readme = {}
    for path in sorted((E / 'baseline-v1/rows').glob('*.json')):
        digest = sha(path)
        for phase in ['candidate-v1', 'candidate-v2', 'native-v1']:
            assert sha(E / phase / 'rows' / path.name) == digest
        exact_readme[path.name] = {'rows': len(json.loads(path.read_text())), 'sha256': digest}
    assert len(exact_readme) == 9 and sum(row['rows'] for row in exact_readme.values()) == 729
    bench_lines = (E / 'native-v1/benchmark-consumer.stdout').read_text().splitlines()
    header = next(i for i, line in enumerate(bench_lines) if line.startswith('engine,temperature,'))
    bench = list(csv.DictReader(bench_lines[header:]))
    expected_iterations = dict(arrow=64, trapezoid=64, star=16, teardrop=8, egg=8,
                               squircle=4, pie_slice=8, supershape=4, heart=4)
    assert len(bench) == 27
    for case, iterations in expected_iterations.items():
        rows = [row for row in bench if row['case'] == case]
        assert len(rows) == 3 and all(int(row['iterations']) == iterations for row in rows)

    costs = {}
    for phase in ['cost-v1', 'cost-v2']:
        directory = E / phase
        result = json.loads((directory / 'result.json').read_text())
        protocol = json.loads((directory / 'protocol.json').read_text())
        assert result['passed'] == (phase == 'cost-v2')
        if phase == 'cost-v1':
            assert len(result['commands']) == 1 and result['commands'][0]['exit_code'] == 1
            assert 'E0505' in (directory / 'baseline-build.stderr').read_text()
        merge(artifacts, protocol['inputs_sha256'])
        for row in result['commands']:
            assert row['sources_unchanged']
            for stream, digest in row['streams_sha256'].items():
                assert sha(directory / (row['name'] + '.' + stream)) == digest
        if result['passed']:
            merge(artifacts, {name: row['sha256'] for name, row in json.loads((directory / 'artifacts.json').read_text()).items()})
        costs[phase] = result
    assert len(costs['cost-v2']['commands']) == 128
    exact_cost = json.loads((E / 'cost-v2/exact-rows.json').read_text())
    assert len(exact_cost) == 9 and sum(row['rows'] for row in exact_cost.values()) == 1152
    for shape, row in exact_cost.items():
        for label in ['baseline', 'candidate']:
            path = E / 'cost-v2' / f'{shape}-{label}-dump.stdout'
            assert sha(path) == row['sha256'] and len(json.loads(path.read_text())) == 128
    samples, pairs = {}, {}
    for row in costs['cost-v2']['commands']:
        if 'sample' not in row:
            continue
        sample = row['sample']
        assert sample['completed'] == sample['iterations'] == 128
        assert sample['certainty'] == 'Certified' and sample['policy'] == 'strict'
        pair, label = row['name'].rsplit('-', 1)
        pairs.setdefault(pair, {})[label] = sample
        key = sample['shape'] + '-' + sample['cache']
        samples.setdefault(key, {}).setdefault(label, []).append(sample)
    assert len(samples) == 18 and len(pairs) == 54
    assert all(len(rows) == 3 for pair in samples.values() for rows in pair.values())
    assert all(pair['baseline']['checksum'] == pair['candidate']['checksum'] and
               pair['baseline']['output_bytes'] == pair['candidate']['output_bytes'] for pair in pairs.values())
    medians = {key: {label: {metric: statistics.median(row[metric] for row in rows)
                            for metric in ['nanos', 'allocations', 'requested_bytes', 'retained_byte_delta', 'peak_live_bytes_above_start']}
                      for label, rows in pair.items()} for key, pair in samples.items()}
    ratios = {key: pair['candidate']['nanos'] / pair['baseline']['nanos'] for key, pair in medians.items()}
    assert all(sha(name) == digest for name, digest in artifacts.items())
    print(json.dumps({'protected_artifacts_verified': len(artifacts)}), flush=True)

    compiler = {}
    removed, linked = 0, 0
    constructor = ROOT / 'docs/evidence/2026-10-15-curve-constructor-failures'
    for manifest in constructor.glob('removed-*.json.gz'):
        rows = json.loads(gzip.decompress(manifest.read_bytes()))
        removed += len(rows)
        for name, row in rows.items():
            assert not Path(name).exists()
            merge(compiler, {row['retained_original']: row['sha256']})
    for manifest in constructor.glob('linked-*.json.gz'):
        rows = json.loads(gzip.decompress(manifest.read_bytes()))
        linked += len(rows)
        for name, row in rows.items():
            merge(compiler, {name: row['sha256'], row['retained_original']: row['sha256']})
    metaball = ROOT / 'docs/evidence/2026-10-15-curve-metaball-failures'
    for manifest in metaball.glob('removed-*.json.gz'):
        for row in json.loads(gzip.decompress(manifest.read_bytes())).values():
            merge(compiler, {row['retained_original']: row['sha256']})
    assert len(compiler) == prior['compiler_paths_verified']
    for manifest in E.glob('linked-*.json.gz'):
        rows = json.loads(gzip.decompress(manifest.read_bytes()))
        linked += len(rows)
        for name, row in rows.items():
            merge(compiler, {name: row['sha256'], row['retained_original']: row['sha256']})
    assert all(sha(name) == digest for name, digest in compiler.items())
    print(json.dumps({'closed_compiler_paths_verified': len(compiler)}), flush=True)

    implementation = json.loads((E / 'implementation-paths.json').read_text())
    assert set(subprocess.check_output(['git', 'diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD'], cwd=ROOT, text=True).splitlines()) == set(implementation)
    source_sizes = {}
    for name in ['src/curve/native.rs', 'src/solid.rs', 'src/wasm/curve_js.rs']:
        before = subprocess.check_output(['git', 'show', '2271ad6c5:' + name], cwd=ROOT)
        source_sizes[name] = {'baseline_bytes': len(before), 'candidate_bytes': (ROOT / name).stat().st_size}
    entry = {
        'updated_on': '2026-10-15',
        'decision': 'kept: typed-failure and certainty migration with an equal-width-arrow completeness correction; not a performance-only change or supported-corpus speedup claim',
        'status': 'Nine sampled-profile APIs and their teardrop-solid/WASM consumers locally qualified; old successful fixtures retained exactly.',
        'properties': ['P1', 'P4', 'P6', 'P8', 'P9', 'P17', 'P18', 'P19', 'P20', 'P25', 'P26', 'P27', 'P28', 'P29', 'P30', 'P35', 'P36', 'P37', 'P39'],
        'owners': ['CSGRS profile parameter validation and sampling', 'Hypercurve topology', 'CSGRS teardrop-solid certainty composition', 'WASM diagnostic boundary'],
        'domain': 'Arrow/trapezoid positive dimensions; star n>=3 and outer>inner>0; teardrop n>=2 and length>width/2>0; egg/squircle positive dimensions and n>=3; heart positive dimensions and n>=8; sector positive radius, n>=1, nonzero signed sweep with magnitude<=360; supershape positive axes, n>=3, nonzero n1 and defined scalar powers. Original sample formulas/resolutions retained. Full-turn sector preserves max(n,3) circle sampling; valid zero-area samples may regularize to canonical empty. No analytic-fidelity bound.',
        'migration': 'arrow, trapezoid, star, teardrop, egg, squircle, pie_slice, heart and supershape now return Result<CurveRegion2, ValidationError>. Their _with_context variants return GeometryOutcome. Adapt callers with typed Result handling. Equal-width positive arrows omit only proven duplicate shoulders and now retain shaft plus triangular head. teardrop_cylinder joins profile/extrusion certainty. WASM star/teardrop/egg/heart retain signatures and throw GeometryError with code instead of publishing failed geometry.',
        'semantic_evidence': [
            'Baseline 9 pass/15 fail: fourteen reproduced error/completeness defects and one incorrect zero-area-sector oracle. Candidate-v1 23 pass/1 fail. That oracle was corrected after establishing successful native regularization, without changing production behavior. Candidate-v2 has 30 passing regressions; all failed trials retained.',
            'Independent rational/algebraic areas for all nine families under strict and approximate policies remain Certified when only proofs are consumed. Empty union/self-subtraction and downstream extrusion compose. Equal-width arrow area is 5/2; one-chord semicircle is Certified empty with downstream union/extrusion checks.',
            'Arrow with an interior hole has exact area 9/4 and volume 9 at height 4; sampled teardrop also has volume 9. Independent signed-volume computation and a separate closed-manifold check pass. Dyadic OBJ round-trip preserves positions exactly; this is a format-specific claim.',
            'Approximate arrow-shoulder equality remains qualified even with an identical strictly proved ring cached. Strict/approximate/strict does not promote the qualified comparison. Capacity failures remain typed and recovery succeeds. This does not establish end-to-end source provenance.',
            'Nine original full README fixtures retain all 729 exact coordinate pairs across baseline, both candidates and final native phase. Original benchmark fixtures and iteration counts remain. Cost samples additionally preserve all 128 exact coordinate-row variants per family. These artifact checks supplement semantic oracles.',
        ],
        'test_totals': totals['native-v1'],
        'validation': 'All 13 native commands passed: root/fuzz formatting; all-target -D warnings Clippy for all features, minimal and curve-only; all-feature library/integration tests (293 pass, one existing full image-corpus test ignored); minimal 38 pass; curve-only 108 pass; fuzz-target Clippy; warning-free docs; one doctest; nine original benchmark consumers; full renderer build. 440 passing test executions across overlapping configurations. WASM-target Clippy and actual Node/TypeScript binding consumers passed, including 13 new rejection cases and four successful recovery/classification/extrusion fixtures. No C/C++/Python/Go surface exposes these constructors.',
        'cost_probe': {
            'kind': 'focused matched successful-output measurement, not a broad performance claim',
            'evidence': str((E / 'cost-v2').relative_to(ROOT)),
            'failed_trial': 'cost-v1 retained: rustc E0505 in the independent sector-area harness. cost-v2 borrows both factors; production source, inputs and timing boundary unchanged.',
            'commands': 128, 'fresh_process_samples': 108, 'operations_per_sample': 128,
            'timed_completed_operations': 13824, 'exact_variants_per_revision': 1152,
            'median_samples': medians, 'candidate_to_baseline_latency_ratios': ratios,
            'library_sizes': json.loads((E / 'cost-v2/library_sizes.json').read_text()),
            'binary_artifacts': json.loads((E / 'cost-v2/artifacts.json').read_text()),
            'claim': 'Same successful strict workload with independent exact-area proof, boundary materialization, exact-expression serialization, checksum and drop in timing. All outputs and paired checksums agree. Three samples per case on a shared unpinned host with allocator instrumentation; noisy latency is mixed. Warm egg median is about 15.4% higher, FIFO-miss squircle about 11.1% higher. Arrow adds one allocation and 672 requested bytes per operation; star removes one allocation/384 bytes, sector one/192, heart five/544. Other measured allocation counts match. Ring FIFO misses do not imply globally cold scalar caches.',
        },
        'source_sizes': source_sizes,
        'resources': 'Checked primary-buffer counts and fallible reservations; no internal bound on total work, scalar growth, secondary allocations, heap retention or cancellation. Serial offline Cargo trials with two jobs and external 1800-second wall/per-process 8-GiB address-space containment. Actual WASM/Node command has wall containment without address-space cap because of V8/WASM reservations. Allocation probe covers the Rust global allocator, not all native allocations or RSS.',
        'limitations': [
            'Gear, rack, airfoil, text and other legacy convenience paths still require failure migration. Operation-local certainty is separate from source provenance, topology and analytic fidelity.',
            'Original 35 pending native files remain byte-identical and uncommitted, awaiting separate admission. This is worktree qualification, not clean external-package or release qualification.',
            'Full workflow/pre-push matrix, MSRV/release/external-package checks, full image corpus, directed fuzz/sanitizers and remaining properties-plan requirements stay open. No push or CI edits.',
        ],
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
    ledger['current_work']['sampled_profile_failures'] = entry
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
    v.dump(output, {
        'qualification': entry, 'closure_runner_sha256': sha(Path(__file__)),
        'prior_qualification_sha256': sha(prior_path), 'phase_results': results,
        'phase_test_totals': totals, 'cost_results': costs, 'benchmark_consumer_rows': bench,
        'exact_readme_rows': exact_readme, 'exact_cost_rows': exact_cost,
        'source_input_migrations': migrations, 'preserved_executables_sha256': artifacts,
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
