"""Preserve the accepted trial, prior evidence, pending work and ledger boundaries."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
MEMO = {}


def sha(path):
    path = Path(path)
    stat = path.stat()
    key = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
    if key not in MEMO:
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        MEMO[key] = digest.hexdigest()
    return MEMO[key]


def span(raw):
    start = re.search(r'"current_work"\s*:\s*', raw).end()
    value, end = json.JSONDecoder().raw_decode(raw[start:])
    return start, start + end, value


def main():
    assert not (E / 'closure-v1.json').exists()
    assessment = json.loads((E / 'assessment-v1.json').read_text())
    decision = json.loads((E / 'decision-v1.json').read_text())
    assert decision['status'] == 'kept'
    assert json.loads((E / 'build-identity-v1/result.json').read_text())['passed']
    baseline_sources = json.loads(gzip.decompress((E / 'baseline-v2/source-hashes.json.gz').read_bytes()))
    candidate_sources = json.loads(gzip.decompress((E / 'candidate-v1/source-hashes.json.gz').read_bytes()))
    assert baseline_sources.keys() == candidate_sources.keys()
    assert {name for name in baseline_sources if baseline_sources[name] != candidate_sources[name]} == {'src/solid.rs'}
    assert candidate_sources == json.loads(gzip.decompress((E / 'cargo-v1/source-hashes.json.gz').read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in candidate_sources.items())
    assert all(sha(ROOT / name) == digest for name, digest in assessment['implementation_sha256'].items())
    previous_path = ROOT / 'docs/evidence/2026-10-15-slice-projection-contract/closure-v1.json'
    previous = json.loads(previous_path.read_text())
    assert assessment['pending_native_sha256'] == previous['pending_native_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in previous['pending_native_sha256'].items())
    protected = dict(previous['preserved_artifacts_sha256'])
    for phase in ['baseline-v1', 'baseline-v2', 'candidate-v1', 'cargo-v1']:
        folder = E / phase
        result = json.loads((folder / 'result.json').read_text())
        protocol = json.loads((folder / 'protocol.json').read_text())
        assert sha(E / 'validate_v1.py') == protocol['runner_sha256']
        protected.update(protocol['preserved_prior_executables_sha256'])
        for row in result['commands']:
            assert row['sources_unchanged']
            for suffix in ['stdout', 'stderr', 'resources']:
                assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
            protected.update(row['binaries_sha256'])
        if phase != 'cargo-v1':
            for path in (ROOT / 'target/section-proof-reuse' / phase).iterdir():
                if path.is_file(): protected[str(path.relative_to(ROOT))] = sha(path)
    cost = json.loads((E / 'cost-v1/result.json').read_text())
    assert cost['passed'] and len(cost['commands']) == 88
    for row in cost['commands']:
        assert row['sources_unchanged'] and row['exit_code'] == 0
        for suffix, digest in row['streams_sha256'].items():
            assert sha(E / 'cost-v1' / (row['name'] + '.' + suffix)) == digest
    protocol = json.loads((E / 'cost-v1/protocol.json').read_text())
    protected.update(protocol['inputs_sha256'])
    for path, record in json.loads((E / 'cost-v1/artifacts.json').read_text()).items():
        assert sha(ROOT / path) == record['sha256']
        protected[path] = record['sha256']
    print('Verifying all preserved artifacts and the prior closed evidence.', flush=True)
    assert all(sha(ROOT / name) == digest for name, digest in previous['evidence_files_sha256'].items())
    assert all(sha(ROOT / name) == digest for name, digest in protected.items())
    links = 0
    for manifest in E.glob('linked-compiler-duplicates-v*.json.gz'):
        for name, row in json.loads(gzip.decompress(manifest.read_bytes())).items():
            origin = Path(row['retained_original'])
            assert sha(name) == sha(origin) == row['sha256']
            assert Path(name).stat().st_ino == origin.stat().st_ino
            links += 1
    ledger = ROOT / 'docs/properties-status.json'
    raw = ledger.read_text()
    before = sha(ledger)
    reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
    start, end, current = span(raw)
    rstart, rend, _ = span(reference)
    assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
    current['section_proof_reuse'] = {
        'status': 'kept: identical native section inputs reuse their winding proof',
        'baseline_commit': '3a2d3fd1b',
        'evidence': 'docs/evidence/2026-10-15-section-proof-reuse/assessment-v1.json',
        'decision': 'docs/evidence/2026-10-15-section-proof-reuse/decision-v1.json',
        'closure': 'docs/evidence/2026-10-15-section-proof-reuse/closure-v1.json',
        'validation': assessment['tests_passed_failed_ignored'],
        'preserved_contract': '25 section/projection regressions and 43 byte-identical snapshots preserve exact rows, exposed boundary evidence and certainty, including the original full-resolution sphere and frustum. No policy or fixture changes.',
        'cost': decision['cost'],
        'limitations': assessment['limitations'],
        'full_plan_complete': False,
    }
    current['updated_on'] = '2026-10-15'
    current['plan_complete'] = False
    replacement = json.dumps(current, indent=2).replace('\n', '\n  ')
    ledger.write_text(raw[:start] + replacement + raw[end:])
    updated = ledger.read_text()
    ustart, uend, _ = span(updated)
    assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
    first, second = json.loads(updated), json.loads(reference)
    first.pop('current_work'); second.pop('current_work')
    assert first == second
    evidence = {str(path.relative_to(ROOT)): sha(path) for path in sorted(E.rglob('*'))
                if path.is_file() and '__pycache__' not in path.parts}
    closure = {
        'assessment_sha256': sha(E / 'assessment-v1.json'),
        'decision_sha256': sha(E / 'decision-v1.json'),
        'previous_closure_sha256': sha(previous_path),
        'closure_runner_sha256': sha(Path(__file__)),
        'evidence_files_sha256': evidence,
        'preserved_artifacts_sha256': protected,
        'pending_native_sha256': previous['pending_native_sha256'],
        'linked_compiler_paths_verified': links,
        'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger),
        'only_current_work_changed_in_ledger': True,
        'implementation_sha256': assessment['implementation_sha256'],
        'performance_notes_sha256': sha(ROOT / 'PERFORMANCE.md'),
        'full_plan_complete': False,
    }
    (E / 'closure-v1.json').write_text(json.dumps(closure, indent=2) + '\n')
    print(json.dumps({'evidence_files_including_closure': len(evidence) + 1,
                      'protected_artifacts': len(protected), 'linked_compiler_paths': links,
                      'pending_native_files_unchanged': 35, 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
