"""Verify the fixed native profile without treating it as a matched benchmark."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'profile-v1'
PARENT = ROOT / 'docs/evidence/2026-10-13-native-tower-sign-retention'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    assert not (E / 'assessment-v1.json').exists()
    inputs = json.loads((E / 'inputs-v1.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    manifest = json.loads((OUT / 'manifest.json').read_text())
    observed = json.loads((OUT / 'observations.json').read_text())
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    source = Path(parent['source'])
    assert manifest['complete'] and manifest['closed_profile_runs'] == 1
    assert protocol['source'] == inputs['source'] == str(source)
    assert protocol['source_count'] == parent['combined_files'] == 10131
    assert sha(PARENT / 'preparation-v1/protocol.json') == inputs['parent_preparation_sha256'] == protocol['parent_preparation_sha256']
    assert sha(PARENT / 'preparation-v1/source-hashes.json.gz') == inputs['parent_source_hashes_sha256']
    assert sha(PARENT / 'qualification-assessment-v2.json') == inputs['parent_qualification_sha256'] == protocol['parent_validation_sha256']
    assert sha(ROOT / 'docs/evidence/2026-10-13-borrowed-same-radicand-fields/decision-v1.json') == inputs['closed_helper_series_decision_sha256']
    assert sha(E / 'inputs-v1.json') == protocol['inputs_sha256']
    assert sha(E / 'profile_v1.py') == protocol['runner_sha256']
    assert sha(E / 'analyze_v1.py') == observed['runner_sha256']
    for name, digest in protocol['frozen_inputs_sha256'].items():
        assert sha(E / name) == digest, name
    for name, digest in manifest['artifacts_sha256'].items():
        assert sha(OUT / name) == digest, name
    assert sha(OUT / 'manifest.json') == observed['profile_manifest_sha256']
    assert sha(OUT / 'stacks.stdout.gz') == observed['stacks_gzip_sha256']
    assert sha(Path(manifest['binary'])) == manifest['binary_sha256']
    commands = json.loads((OUT / 'commands.json').read_text())
    assert [r['name'] for r in commands] == ['build', 'record', 'self-cost', 'inclusive-cost', 'header', 'stacks']
    logs = {}
    for command in commands:
        assert command['sources_unchanged']
        assert command['exit_code'] == (manifest['record_exit_code'] if command['name'] == 'record' else 0)
        logs[command['name']] = {}
        for stream in ['stdout', 'stderr']:
            path = OUT / (command['name'] + '.' + stream + '.gz')
            raw = gzip.decompress(path.read_bytes())
            assert len(raw) == command[stream + '_bytes']
            assert hashlib.sha256(raw).hexdigest() == command[stream + '_sha256']
            assert sha(path) == command[stream + '_gzip_sha256']
            logs[command['name']][stream] = raw.decode()
    build_events = [json.loads(line) for line in logs['build']['stdout'].splitlines() if line.startswith('{')]
    warnings = [row for row in build_events if row.get('reason') == 'compiler-message' and row['message']['level'] == 'warning']
    assert not warnings
    record = logs['record']['stdout'] + logs['record']['stderr']
    assert 'skipping complex project fixture test' not in record
    assert record.count('hyperdrc: finished emit report') == manifest['complete_reports_observed'] == 2
    findings = [int(x) for x in re.findall(r'finished run \((\d+) active finding', record)]
    assert findings == [12, 197]
    code = manifest['record_exit_code']
    assert code in [0, 101]
    if code == 101:
        assert 'test result: FAILED. 0 passed; 1 failed;' in record
        worker = float(re.search(r'complex project KiCad smoke suite took ([0-9.]+)s of worker time', record)[1])
        assert worker >= 120 and worker == manifest['profiled_worker_cpu_seconds']
    else:
        assert 'test result: ok. 1 passed; 0 failed;' in record
        assert manifest['profiled_worker_cpu_seconds'] is None
        worker = None
    mapping = json.loads(gzip.decompress((OUT / 'source-hashes.json.gz').read_bytes()))
    assert mapping == json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(mapping)
    assert all(sha(source / name) == digest for name, digest in mapping.items())
    assert sources() == parent['main_source_sha256'] and len(sources()) == 1134
    pending = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert pending == parent['pending_native_sha256'] and len(pending) == 35
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    for entry in inputs['source_review']:
        assert sha(source / entry['path']) == entry['sha256']
    for entry in inputs['related_count_evidence']:
        assert sha(ROOT / entry['path']) == entry['sha256']
    assert observed['samples'] > 0 and observed['total_period'] > 0
    assert sum(r['period'] for r in observed['exclusive_recovered_stack_partition']) == observed['total_period']
    result = {
        'status': 'native-normal-form-profile-observed-await-review',
        'assessor_sha256': sha(Path(__file__)),
        'quality': {k: observed[k] for k in ['samples', 'total_period', 'lost_samples', 'empty_stack_period_percent', 'unknown_leaf_period_percent']},
        'profiled_worker_cpu_seconds': worker,
        'original_worker_cpu_limit_seconds': 120,
        'profiled_gate_passed': code == 0,
        'record_process_wall_seconds': commands[1]['seconds'],
        'finding_counts': findings,
        'groups': observed['groups'],
        'exclusive_recovered_stack_partition': observed['exclusive_recovered_stack_partition'],
        'intersections': observed['intersections'],
        'normal_form_nearest_consumers': observed['normal_form_nearest_consumers'],
        'coefficient_multiply_nearest_consumers': observed['coefficient_multiply_nearest_consumers'],
        'source_review': inputs['source_review'],
        'related_count_evidence': inputs['related_count_evidence'],
        'source_parent_main_and_binary_unchanged': True,
        'commands_verified': len(commands), 'compiler_warnings': len(warnings),
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'inputs_sha256': sha(E / 'inputs-v1.json'),
        'limits': observed['limits'] + [
            'The profile build enables frame pointers and line tables; its assertion result does not replace the parent release result.',
            'The original copper-neck check-body gate was not rerun. Whole-process wall time is not either assertion metric.',
            'Matching finding counts do not establish report-byte identity. No full report replay or CLI size measurement was performed.',
            'Previous closed samples remain closed and all rejected helper/storage candidates remain rejected.',
            'All 35 pending native paths and all 39 properties, 14 targets and crate future work remain in scope.'
        ],
        'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'assessment-v1.json').write_text(json.dumps(result, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    base = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in status.items() if k != 'current_work'} == {k: v for k, v in base.items() if k != 'current_work'}
    work = status['current_work']['native_normal_form_attribution']
    work.update(status=result['status'], evidence=str((E / 'assessment-v1.json').relative_to(ROOT)),
                result='The single fixed profile and its predeclared attribution are complete; source, binary, commands and lossless logs are verified. Interpretation remains pending.',
                quality=result['quality'], profiled_worker_cpu_seconds=worker,
                original_worker_cpu_limit_seconds=120, profiled_gate_passed=code == 0,
                next_step='Review the complete attribution alongside frozen source and earlier counts; select an avoidable operation only when source and observed coverage support it. No equivalent multiplication-helper trial is proposed.')
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ['status', 'quality', 'profiled_worker_cpu_seconds', 'profiled_gate_passed', 'finding_counts', 'commands_verified']}))


if __name__ == '__main__':
    main()
