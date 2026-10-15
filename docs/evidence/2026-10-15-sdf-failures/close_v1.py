"""Close the SDF migration evidence once, preserving all earlier trials."""
import gzip
import importlib.util
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
spec = importlib.util.spec_from_file_location('validation', E / 'validate_v2.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)


def counts(text):
    rows = re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', text)
    return dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'], [sum(int(row[i]) for row in rows) for i in range(5)]))


def main():
    output = E / 'closure-v1.json'
    assert not output.exists()
    old = ROOT / 'docs/evidence/2026-10-15-bounds-failures/closure-v1.json'
    prior = json.loads(old.read_text())
    binaries = dict(prior['preserved_executables_sha256'])
    pending = prior['pending_native_sha256']
    assert all(v.sha(ROOT / name) == digest for name, digest in pending.items())
    extra = json.loads((E / 'final-v3-documentation-and-runner-inputs.json').read_text())
    assert all(v.sha(ROOT / name) == digest for name, digest in extra.items())
    accepted = 'native-v3'
    phases = ['baseline-v2', 'candidate-v1', 'candidate-v2', 'candidate-v3', 'candidate-v4', 'native-v1', 'native-v2', accepted]
    results, totals, logs, archives = {}, {}, {}, {}
    for phase in phases:
        directory = E / phase
        for name in ['protocol.json', 'source-hashes.json.gz', 'tracked.patch', 'untracked-sources.tar.gz', 'result.json']:
            archives[str((directory / name).relative_to(ROOT))] = v.sha(directory / name)
        result = json.loads((directory / 'result.json').read_text())
        results[phase] = result
        protocol = json.loads((directory / 'protocol.json').read_text())
        assert protocol['runner_sha256'] == v.sha(E / 'validate_v2.py')
        assert protocol['commands'] == json.loads((E / (phase + '.json')).read_text())
        for row in result['commands']:
            assert row['sources_unchanged']
            binaries.update(row['binaries_sha256'])
            for stream in ['stdout', 'stderr']:
                path = directory / (row['name'] + '.' + stream)
                assert v.sha(path) == row[stream + '_sha256']
                logs[str(path.relative_to(ROOT))] = v.sha(path)
            if phase == accepted:
                assert row['exit_code'] == 0
                assert not re.search(r'^warning(?:\[|:)', (directory / (row['name'] + '.stderr')).read_text(), re.M)
                totals[row['name']] = counts((directory / (row['name'] + '.stdout')).read_text())
        if phase == accepted:
            assert result['passed'] and len(result['commands']) == len(protocol['commands'])
            with gzip.open(directory / 'source-hashes.json.gz', 'rt') as f:
                assert v.sources() == json.load(f)
    assert counts((E / 'baseline-v2/sdf-regressions.stdout').read_text())['failed'] == 8
    assert totals['all-feature-tests']['passed'] == 213
    assert totals['hypermesh-tests']['passed'] == 236
    assert totals['sdf-only-tests']['passed'] == 18
    assert totals['ffi-tests']['passed'] == 10
    assert totals['hypersdf-all-feature-tests']['passed'] == 110
    assert totals['hypersdf-default-tests']['passed'] == 109
    assert totals['doctests']['passed'] == 1
    assert all(v.sha(ROOT / name) == digest for name, digest in binaries.items())
    entry = {
        'updated_on': '2026-10-15',
        'status': 'Typed SDF scalar evaluation and modeling failures implemented and locally qualified; standalone-grid extraction supplied by Hypermesh to the modeling sampler.',
        'properties': ['P4', 'P6', 'P8', 'P9', 'P18', 'P20', 'P25', 'P35', 'P37'],
        'owners': ['Hypersdf strict scalar/domain/branch evaluation', 'CSGRS shared sampled-field modeling and diagnostics', 'Hypermesh Surface Nets face ownership'],
        'domain': 'Real callbacks and retained SdfExpr values over rectangular sample grids, preserving valid formulas and fixture resolutions. The output is the sampled Surface Nets level-surface proposal; numerical certainty does not establish closed topology or continuous-field fidelity. Strict construction branches and selected-policy extraction have separate roles.',
        'migration': 'solid::sdf, sdf_with_diagnostics, sdf_expr and sdf_expr_with_diagnostics now return Result and delegate to the shared context sampler. Expression failures retain sample index and SdfEvaluationError including native scalar source. Added sdf_expr_with_diagnostics_and_context. Resolution below two now fails instead of clamping; diagnostics fail when requested signs/extrema cannot be decided. The modeling sampler uses the new shared Hypermesh surface_nets_standalone entry point, including final-layer faces whenever all four incident cells are present. The existing native surface_nets entry point retains its documented chunk ownership convention. This explicitly migrates modeling output rather than claiming unchanged artifacts. Hypersdf classification and preview keep their existing report contracts; the legacy primitive scalar_value Option explicitly discards the new fallible method\'s errors.',
        'semantic_evidence': [
            'Eight reproduced SDF failures distinguish invalid domains, undecided scalar branches/samples, capacity overflow and invalid resolution from successful empty geometry.',
            'Arithmetic failure after two successful samples preserves grid index [2,0,0] and the native SqrtNegative cause through Error::source. Subsequent constant-field sampling, union from empty and subtraction to empty succeed.',
            'Independently derived scalar values cover all eight native primitives, expression composition, boundaries, invalid parameters and unresolved domains/branches.',
            'Exact rational plane positions, sample counts and extrema are checked independently of wrapper agreement. Native planes crossing first/last layers on all three axes have independently known unit oriented area and exact transverse coordinates.',
            'Strict/approximate/strict sequences retain local certainty: approximate sample evidence does not authorize approximate expression branches or enter strict preview sign buckets. Selecting an approximate policy for a known constant still yields Certified.',
            'Existing sampled spheres, extreme exact scalar magnitudes, closed-mesh validation, downstream Boolean use and all three TPMS solid fixtures retain their original resolutions and pass.',
        ],
        'trials': 'baseline-v1 preparation failed before tests; baseline-v2 failed all eight regressions at a8ca70322 plus original native changes. Candidate v1/v2 compile failures are retained. Candidate v3 exposed missing final-layer plane faces in the modeling API despite successful error tests. Native chunk ownership was subsequently identified as deliberate and retained; the modeling layer now selects standalone extraction. Candidate v4 passed focused runtime checks but strict Clippy rejected test loop style. Native v1 stopped on pre-existing nested fuzz formatting drift; formatting-only fixes were applied. Native v2 passed runtime and root lint checks but stopped on a pre-existing nested fuzz loop lint. Native v3 fixes that lint and qualifies the final source, including the explicit native chunk/standalone split.',
        'test_totals': totals,
        'resources': 'Checked primary sample, coordinate and diagnostic-preview reservations report capacity/allocation failures. Scalar expression work, native extraction and secondary preview allocations still lack internal total-work, heap-byte and cancellation limits. Each validation command uses external 1800-second wall containment. No performance improvement or bounded execution is claimed.',
        'limitations': [
            'Eight existing ignored cases remain: one root rendering case and seven Hypermesh external-fixture/manual scale or memory tests. Counts across feature configurations overlap.',
            'The lossy Hypersdf preview backend retains its chunk boundary convention and may omit final-layer faces or withhold unknown-sign samples accepted by approximate meshing. It remains PreviewOnly and never replaces the authoritative native mesh.',
            'Examples and benchmark consumers are compiled with strict all-target Clippy; no benchmark timing or generated rendering claim is made. Fuzz targets are compiled and linted, not fuzzed for a duration.',
            'SDF Rust APIs are not exposed by the current C/Go/Python/WASM bindings. FFI unit tests and all-target compilation pass, but live binding consumers were not rerun for this Rust/API and native-grid change.',
            'The complete workflow matrix, sanitizers, clean external package/release/MSRV validation and performance comparison remain open.',
            'Separate metaball fallbacks and approximation policy, other modeling failure migrations, end-to-end provenance, resource bounds, ordinary strict-workflow completion and prior scalar/native admission gaps remain open.',
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
    ledger['current_work']['sdf_failure_visibility'] = entry
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
        'preserved_executables_sha256': binaries, 'retained_logs_sha256': logs,
        'phase_archives_sha256': archives,
        'pending_native_sha256': pending, 'documentation_and_runner_inputs_sha256': extra,
        'ledger_before_sha256': before, 'ledger_after_sha256': v.sha(ledger_path),
        'only_current_work_changed_in_ledger': True, 'full_plan_complete': False,
    })
    print(json.dumps({'qualification_recorded': True, 'executables_verified': len(binaries), 'test_totals': totals, 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
