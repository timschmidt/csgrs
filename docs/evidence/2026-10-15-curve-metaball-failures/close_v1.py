"""Close the planar migration evidence once, preserving all earlier trials."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
spec = importlib.util.spec_from_file_location('validation', E / 'validate_v7.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)


def counts(text):
    rows = re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', text)
    return dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'],
                    [sum(int(row[i]) for row in rows) for i in range(5)]))


def source_snapshot(phase):
    return json.loads(gzip.decompress((E / phase / 'source-hashes.json.gz').read_bytes()))


def main():
    output = E / 'closure-v1.json'
    assert not output.exists()
    old = ROOT / 'docs/evidence/2026-10-15-metaball-failures/closure-v1.json'
    prior = json.loads(old.read_text())
    binaries = dict(prior['preserved_executables_sha256'])
    pending = prior['pending_native_sha256']
    assert all(v.sha(ROOT / name) == digest for name, digest in pending.items())
    results, totals = {}, {}
    runner_hashes = {v.sha(path) for path in E.glob('validate_v*.py')}
    for result_path in sorted(E.glob('*/result.json')):
        directory = result_path.parent
        phase = directory.name
        result = json.loads(result_path.read_text())
        protocol = json.loads((directory / 'protocol.json').read_text())
        assert protocol['runner_sha256'] in runner_hashes
        assert protocol['commands'] == json.loads((E / (phase + '.json')).read_text())
        assert result['passed'] or result['commands'][-1]['exit_code'] != 0
        results[phase] = result
        totals[phase] = {}
        for row in result['commands']:
            assert row['sources_unchanged']
            binaries.update(row['binaries_sha256'])
            for stream in ['stdout', 'stderr', 'resources']:
                key = stream + '_sha256'
                if key in row:
                    assert v.sha(directory / (row['name'] + '.' + stream)) == row[key]
            totals[phase][row['name']] = counts((directory / (row['name'] + '.stdout')).read_text())
        if phase in {'native-v2', 'bindings-v2'}:
            assert result['passed'] and len(result['commands']) == len(protocol['commands'])
            for row in result['commands']:
                assert not re.search(r'^warning(?:\[|:)', (directory / (row['name'] + '.stderr')).read_text(), re.M)
    assert len(results['native-v2']['commands']) == 22
    assert len(results['bindings-v1']['commands']) == 37
    assert all(row['exit_code'] == 0 for row in results['bindings-v1']['commands'][:-1])
    assert results['bindings-v1']['commands'][-1]['name'] == 'wasm'
    assert results['bindings-v1']['commands'][-1]['exit_code'] == 1
    assert len(results['bindings-v2']['commands']) == 5
    assert totals['baseline-v1']['regressions']['failed'] == 9
    for name, passed in {
        'all-feature-tests': 238, 'hypercurve-tests': 2493, 'hyperlimit-tests': 365,
        'hypertri-tests': 188, 'hypermesh-tests': 236, 'hypersdf-tests': 115,
        'ffi-tests': 10, 'minimal-tests': 38, 'planar-tests': 16,
        'metaballs-only-tests': 15, 'native-default-grid': 6, 'doctests': 6,
    }.items():
        assert totals['native-v2'][name]['passed'] == passed
    native, bindings, final = map(source_snapshot, ['native-v2', 'bindings-v1', 'bindings-v2'])
    assert native.keys() == bindings.keys() == final.keys()
    after_native = [name for name in native if native[name] != bindings[name]]
    assert set(after_native) == {'src/wasm/curve_js.rs', 'src/wasm/mesh_js.rs', 'tests/wasm/geometry_certainty.cjs'}
    after_bindings = [name for name in bindings if bindings[name] != final[name]]
    assert after_bindings == ['tests/wasm/geometry_certainty.cjs']
    assert v.sources() == final
    missing_artifacts = []
    for phase in ['bindings-v1', 'bindings-v2']:
        for name in json.loads((E / (phase + '-artifacts.json')).read_text()):
            path = ROOT / name
            if path.exists():
                binaries[name] = v.sha(path)
            else:
                missing_artifacts.append(name)
    assert missing_artifacts == ['/tmp/csgrs-curve-metaball-failures/bindings-v1/wasm-bindings-node/geometry_certainty.ts']
    # The first script stopped at JS cleanup, before creating the TypeScript
    # consumer copy. The fresh binding phase created and checked its own copy.
    for path in E.glob('*-preserve.json'):
        for name, digest in json.loads(path.read_text()).items():
            assert v.sha(ROOT / name) == digest
            binaries[name] = digest
    assert all(v.sha(ROOT / name) == digest for name, digest in binaries.items())
    # Verify every retained original referenced by the duplicate-reclamation
    # manifests. No deleted compiler copy is the sole surviving instance.
    originals = {}
    removed_count = 0
    for manifest in E.glob('removed-*.json.gz'):
        rows = json.loads(gzip.decompress(manifest.read_bytes()))
        removed_count += len(rows)
        for name, row in rows.items():
            assert not Path(name).exists()
            original = str((ROOT / row['retained_original']).resolve())
            if original in originals:
                assert originals[original] == row['sha256']
            originals[original] = row['sha256']
    assert all(v.sha(Path(name)) == digest for name, digest in originals.items())
    entry = {
        'updated_on': '2026-10-15',
        'status': 'Planar exact-field sampling, native contour extraction and exact linear-profile composition implemented and locally qualified, with explicit API/geometry migrations.',
        'properties': ['P1', 'P3', 'P4', 'P6', 'P8', 'P9', 'P13', 'P17', 'P18', 'P19', 'P20', 'P25', 'P26', 'P27', 'P28', 'P29', 'P30', 'P35', 'P36', 'P37', 'P39'],
        'owners': ['Hypersdf exact fields and conservative bounds', 'Hypercurve marching-squares proposals, arrangement and retained line-image authority', 'Hyperlimit independent coordinate witnesses', 'Hypertri triangulation and conservative filter composition', 'CSGRS sampling/modeling and typed adapter/binding failures'],
        'domain': 'Positive-radius planar inverse-square sources, positive iso, nonnegative padding and grid dimensions at least two. Parameters are validated before empty success. Native sampled chords require exterior boundary samples and a supported arrangement. Exact certified line profiles retain material/hole ownership and native coordinates through triangulation and extrusion, including authored line-image Beziers, rational holes and large exact translations.',
        'migration': 'curve::metaballs returns Result and adds metaballs_with_context. Sampling shares native F=(iso-I)/(iso+I), with -1 at centers, replacing cutoff/sentinel/empty-on-error behavior. The original 2D core grid and center-sum diagonal rule remain; normalized interpolation intentionally changes vertices and serialization. Exact line-profile modeling replaces its former f64 projection, also changing formerly quantized artifacts. Numerical certainty concerns the sampled proposal or native profile, with topology and source fidelity separate. This is not a performance-only change.',
        'semantic_evidence': [
            'Nine planar baseline failures reproduced invalid/undecided arguments, hidden invalid empty cases, grid limits, incorrect sampled diamond area and a lost ring hole.',
            'Independent normalized diamond area 2*(20201/20200)^2; grid diamond/zero-vertex/ring/disconnected/diagonal cases; closed extrusion with exact signed volume; reusable empty and subtraction-to-empty compositions.',
            'Independent rational material/hole area 23/3 and height-two volume 46/3, including translation by 10^400, native coordinate membership and closed manifold validation.',
            'Original 24x24 README and 48x48 benchmark inputs remain full-size strict successes; the unchanged involute/cycloidal gear catalog passes after native predicate and filter fixes.',
            'Point equality can use one certified unequal coordinate despite uncertainty elsewhere; true equality still needs every coordinate. Lexicographic ordering is unchanged. Unknown AABB rejection filters defer to final triangle predicates without an approximate filter decision.',
            'Typed native adapter errors, strict/approximate/strict evidence isolation, C/C++/Python/Go integer width 2^53+1 and failure recovery, plus WASM large-translation/extrusion recovery and typed zero/non-finite input failures.',
        ],
        'test_totals': totals['native-v2'],
        'validation': 'All 22 native commands passed. The final WASM ingress edit passed host all-target/all-feature Clippy and WASM Clippy. Binding v1 passed 36/37 commands; every WASM semantic assertion passed but duplicate test ownership caused a cleanup failure before TypeScript. Binding v2 changed only that JS bookkeeping, reused identical generated module bytes, and passed all five commands including Node cleanup and TypeScript. Existing ignored cases total 41 across the principal native suites; alternate-feature counts overlap.',
        'resources': 'Serial commands, two Cargo jobs, offline resolution, external 1800-second wall and 8-GiB per-process address-space containment. Node/WASM and final TypeScript omit RLIMIT_AS for V8 virtual reservations while retaining wall containment. GNU time records command timing/maximum resident memory, including compilation. Primary buffers are checked; total work, scalar depth/coefficient growth, secondary allocations, heap-byte bounds and cancellation remain unbounded or unmeasured. No matched benchmark, allocation-efficiency or speedup claim.',
        'limitations': [
            'Other profile representations retain the existing finite sampled path; complete fidelity and end-to-end provenance remain open. Finite display receipts do not certify topology after quantization.',
            'The full workflow/pre-push matrix, sanitizers, directed campaigns, external package/release/MSRV validation, image/export regeneration and continuous-field fidelity bounds were not completed here. No push occurred.',
            'Validation includes the original 35 pending native files, unchanged and still awaiting separate admission. It is not clean-checkout or published-package evidence for this partial commit set.',
            'The original scalar/native integration gaps and the full properties plan remain open. The earlier 3D record\'s open 2D migration item is superseded only by this local qualification.',
            'The first seed-copy failure has only the preserved truncated-tool-output summary, not full stderr. The second failure has a complete captured preparation log. All later unfavorable trials and generated artifacts remain.',
        ],
        'implementation_commits': ['77053db06', '4af09da67'],
        'implementation_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'evidence': str(output.relative_to(ROOT)),
        'plan_complete': False,
    }
    ledger_path = ROOT / 'docs/properties-status.json'
    raw = ledger_path.read_text()
    before = v.sha(ledger_path)
    ledger = json.loads(raw)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    outer = lambda data: {key: value for key, value in data.items() if key != 'current_work'}
    assert outer(ledger) == outer(baseline)
    ledger['current_work']['curve_metaball_composition'] = entry
    ledger['current_work']['updated_on'] = '2026-10-15'
    ledger['current_work']['plan_complete'] = False
    match = re.search(r'(?m)^([ \t]*)"current_work": *', raw)
    start = match.end()
    _, end = json.JSONDecoder().raw_decode(raw[start:])
    replacement = json.dumps(ledger['current_work'], indent=2).replace('\n', '\n' + match.group(1))
    updated = raw[:start] + replacement + raw[start + end:]
    assert json.loads(updated) == ledger and outer(json.loads(updated)) == outer(baseline)
    ledger_path.write_text(updated)
    files = {str(path.relative_to(ROOT)): v.sha(path) for path in sorted(E.rglob('*')) if path.is_file() and '__pycache__' not in path.parts}
    hardware = {
        'cpu': next(line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')),
        'mem_total': Path('/proc/meminfo').read_text().splitlines()[0],
        'note': 'No dedicated-host, frequency-control or matched-performance claim.',
    }
    v.dump(output, {
        'qualification': entry, 'closure_runner_sha256': v.sha(Path(__file__)),
        'prior_qualification_sha256': v.sha(old), 'phase_results': results,
        'phase_test_totals': totals, 'preserved_executables_sha256': binaries,
        'evidence_files_sha256': files, 'pending_native_sha256': pending,
        'source_changes_after_native_validation': after_native,
        'source_changes_after_first_binding_validation': after_bindings,
        'uncreated_artifacts_from_failed_binding_phase': missing_artifacts,
        'duplicate_cache_entries_verified': removed_count,
        'retained_compiler_originals_verified': len(originals),
        'hardware': hardware, 'ledger_before_sha256': before,
        'ledger_after_sha256': v.sha(ledger_path),
        'only_current_work_changed_in_ledger': True, 'full_plan_complete': False,
    })
    print(json.dumps({'qualification_recorded': True, 'executables_and_artifacts_verified': len(binaries),
                      'retained_compiler_originals_verified': len(originals), 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
