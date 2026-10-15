"""Close the metaball migration evidence once without changing earlier trials."""
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
    old = ROOT / 'docs/evidence/2026-10-15-sdf-failures/closure-v1.json'
    prior = json.loads(old.read_text())
    binaries = dict(prior['preserved_executables_sha256'])
    pending = prior['pending_native_sha256']
    assert all(v.sha(ROOT / name) == digest for name, digest in pending.items())
    extra = json.loads((E / 'documentation-and-runner-inputs-v1.json').read_text())
    assert all(v.sha(ROOT / name) == digest for name, digest in extra.items())
    accepted = 'native-v1'
    phases = ['baseline-v1', 'candidate-v1', 'candidate-v2', accepted]
    results, totals, logs, archives = {}, {}, {}, {}
    for phase in phases:
        directory = E / phase
        for name in ['protocol.json', 'source-hashes.json.gz', 'tracked.patch', 'untracked-sources.tar.gz', 'result.json']:
            archives[str((directory / name).relative_to(ROOT))] = v.sha(directory / name)
        result = json.loads((directory / 'result.json').read_text())
        results[phase] = result
        protocol = json.loads((directory / 'protocol.json').read_text())
        runner = 'validate_v1.py' if phase == 'baseline-v1' else 'validate_v2.py'
        assert protocol['runner_sha256'] == v.sha(E / runner)
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
            assert result['passed'] and len(result['commands']) == len(protocol['commands']) == 19
            with gzip.open(directory / 'source-hashes.json.gz', 'rt') as f:
                assert v.sources() == json.load(f)
    assert counts((E / 'baseline-v1/metaball-regressions.stdout').read_text())['failed'] == 10
    for name, expected in {
        'all-feature-tests': 225, 'hypermesh-tests': 236, 'metaballs-only-tests': 15,
        'minimal-tests': 38, 'sdf-only-tests': 18, 'ffi-tests': 10,
        'hypersdf-all-feature-tests': 115, 'hypersdf-default-tests': 114,
        'doctests': 1,
    }.items():
        assert totals[name]['passed'] == expected and totals[name]['failed'] == 0
    assert totals['all-feature-tests']['ignored'] == 1
    assert totals['hypermesh-tests']['ignored'] == 7
    assert all(v.sha(ROOT / name) == digest for name, digest in binaries.items())
    auxiliary = {
        str((E / name).relative_to(ROOT)): v.sha(E / name)
        for name in [
            'README.md', 'reclaim_unexecuted_caches_v1.py', 'cache-reclamation-v1.json',
            'candidate-v1/cache-cleanup.json', 'documentation-and-runner-inputs-v1.json',
            'validate_v1.py', 'validate_v2.py',
        ] + [phase + suffix for phase in phases for suffix in ['.json', '-seed.json']]
    }
    entry = {
        'updated_on': '2026-10-15',
        'status': 'Native exact inverse-square fields and fallible 3D metaball modeling implemented and locally qualified, with an explicit normalized-field output migration.',
        'properties': ['P1', 'P4', 'P6', 'P8', 'P9', 'P17', 'P18', 'P20', 'P25', 'P26', 'P27', 'P35', 'P37'],
        'owners': ['Hypersdf inverse-square evaluation and conservative field bounds', 'CSGRS modeling composition and shared sampled-field diagnostics', 'Hypermesh standalone Surface Nets extraction'],
        'domain': 'Retained Real centers and strictly positive radii, positive iso value, nonnegative padding and requested sample-grid dimensions of at least two. All parameters are validated before empty-input success. Field construction and conservative bounds use strict predicates; extraction uses the selected policy and retains its local certainty. Numerical certainty applies to the sampled proposal, with topology and continuous-field fidelity separate.',
        'migration': 'solid::metaballs and metaballs_with_diagnostics return Result. Added metaballs_with_context and metaballs_with_diagnostics_and_context. MetaBall is the native Hypersdf Metaball carrier; influence returns an exact Result without a distant cutoff or finite pole sentinel. MetaballField borrows validated sources and evaluates F=(iso-I)/(iso+I), I=sum(r^2/distance^2), continuously extended to -1 at centers. The exact zero set is preserved, but interpolation and serialized mesh coordinates intentionally change. Grid locations retain the conservative 101/100 margin and one-cell exterior border. Diagnostics now describe normalized F, negative inside. Hidden f32 extraction and the duplicated 3D sampler were removed in favor of the shared native sampler. This is a geometry/API migration, not a performance-only change.',
        'semantic_evidence': [
            'Ten baseline regressions reproduced invalid radius/padding, unresolved radius, empty shortcuts hiding invalid parameters, resolution clamping, capacity exhaustion, distant influence truncation, a finite pole sentinel and inexact sampled vertices on 5d49530ee plus the unchanged original native work.',
            'Independent exact influence and field values cover near/far points, r=2 at squared distance 9, poles, iso scaling and coincident sources. All radii are validated before any center shortcut; native scalar causes survive Error::source.',
            'A sampled unit source at requested resolution 3 has independently derived coordinate magnitudes 20201/60600 and volume (2*20201/60600)^3. Its closed topology, exact diagnostics and empty-union/subtraction-to-empty composition pass.',
            'Strict/approximate/strict queries preserve unresolved construction as typed Undecided. Selecting approximate meshing for exact decided samples still produces Certified. Successful construction after an error remains possible.',
            'The original two-source 20^3 benchmark fixture and all three sources of the original 8^3 README fixture complete under strict policy, with closed topology, positive exact signed volume and bounds strictly inside independently justified conservative bounds. The pre-existing two-source 6^3 diagnostic fixture also passes unchanged in geometry input and resolution.',
            'The metaballs-only feature compiles, passes its failure/diagnostic tests and strict all-target Clippy without enabling the public sdf feature. Shared SDF/TPMS, native Hypermesh, Hypersdf and FFI regression checks pass.',
        ],
        'trials': 'Baseline v1 failed all ten tests. Its preparatory protocol described (iso-I)/(1+I); the final v2 protocol records the scale-invariant denominator iso+I. The unit-iso oracle is unchanged. Candidate v1 stopped before compilation because dep:hypersdf suppressed the implicit feature used by sdf; the manifest now retains hypersdf. Candidate v2 passed focused tests, five native field tests and strict all-target Clippy. Native v1 adds both full-size published workloads and final documentation, and all 19 commands pass. Every failed trial and source snapshot is retained.',
        'test_totals': totals,
        'resources': 'The shared sampler checks grid capacities and primary reservations, exposing overflow/allocation failure instead of an empty result. Each validation command has external 1800-second wall containment. Scalar expression growth, coefficient growth, native extraction allocations, total work, heap bytes and cancellation lack internal bounds; no bounded-execution or performance improvement claim is made. Peak memory and allocation counters were not measured.',
        'limitations': [
            'The separate 2D curve::metaballs field and contour implementation still has its prior cutoff, sentinel and failure behavior. Its native owning-layer migration remains open.',
            'Eight existing cases remain ignored: one root rendering case and seven Hypermesh external-fixture/manual scale or memory tests. Feature-configuration test counts overlap.',
            'Examples, benchmark and fuzz consumers compile under strict all-target or fuzz Clippy. The two full-size construction fixtures run as tests, but generated images, export artifacts, benchmark timings and duration-based fuzzing were not qualified here.',
            'Metaball constructors are not exposed by the current C/Go/Python/WASM bindings. FFI unit tests and all-target compilation pass; live binding consumers were not rerun for this Rust-only API migration.',
            'The full workflow matrix, sanitizers, clean external package/release/MSRV validation, end-to-end provenance, continuous-field fidelity bounds and performance comparisons remain open.',
            'The original 35 pending native files remain unchanged and unadmitted. Earlier scalar/native integration gaps and the complete properties plan remain open.',
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
    ledger['current_work']['metaball_failure_visibility'] = entry
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
        'phase_archives_sha256': archives, 'auxiliary_evidence_sha256': auxiliary,
        'pending_native_sha256': pending, 'documentation_and_runner_inputs_sha256': extra,
        'ledger_before_sha256': before, 'ledger_after_sha256': v.sha(ledger_path),
        'only_current_work_changed_in_ledger': True, 'full_plan_complete': False,
    })
    print(json.dumps({'qualification_recorded': True, 'executables_verified': len(binaries), 'test_totals': totals, 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
