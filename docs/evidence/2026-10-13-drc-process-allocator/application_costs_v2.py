"""Fixed fresh-process DRC timing/RSS/startup trial; execute only after gate success."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time

from common_v2 import E, PREP, ROOT, sha, verify

PRIOR = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse/report-replay-v1'
INPUTS = ROOT / 'target/endpoint-coordinate-reuse/report-replay-v1/inputs'
WORK = ROOT / 'target/drc-process-allocator-v2/application-costs-v2'
OUT = E / 'application-costs-v2'


def packed(path, raw):
    assert not path.exists()
    path.write_bytes(gzip.compress(raw, mtime=0))


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    qualification = json.loads((E / 'qualification-assessment-v3.json').read_text())
    assert qualification['candidate_original_gates_passed']
    verify(prep)
    scope = json.loads((E / 'scope-v1.json').read_text())
    design = scope['fixed_application_design_if_candidate_gates_pass']
    assert design['blocks'] == [['baseline', 'candidate'], ['candidate', 'baseline'],
                                ['baseline', 'candidate'], ['candidate', 'baseline'], ['baseline', 'candidate']]
    assert (design['complete_report_preflight_runs'], design['board_measurements'],
            design['startup_help_runs_per_measurement'], design['startup_measurements']) == (4, 20, 50, 10)
    old = json.loads((PRIOR / 'protocol.json').read_text())
    old_result = json.loads((PRIOR / 'result.json').read_text())
    assert old_result['complete'] and old_result['all_reports_byte_identical']
    assert sha(PRIOR / 'input-hashes.json.gz') == old['input_hashes_sha256']
    inputs = json.loads(gzip.decompress((PRIOR / 'input-hashes.json.gz').read_bytes()))
    assert len(inputs) == old['extracted_input_files'] == 108
    assert {str(p.relative_to(INPUTS)) for p in INPUTS.rglob('*') if p.is_file()} == set(inputs)
    assert all(sha(INPUTS / name) == digest for name, digest in inputs.items())
    fixtures = old['fixtures']
    assert [f['zip'] for f in fixtures] == design['fixtures']
    prior_reports = {}
    prior_artifacts = {name: sha(PRIOR / name) for name in ['protocol.json', 'result.json', 'input-hashes.json.gz']}
    for fixture in fixtures:
        assert sha(Path(fixture['board_path'])) == fixture['board_sha256']
        for identity in prep['variants'].values():
            assert sha(Path(identity['source']) / 'crates/hyperdrc/docs' / fixture['zip']) == fixture['archive_sha256']
        for old_variant in ['baseline', 'candidate']:
            filename = fixture['label'] + '-' + old_variant + '.json.gz'
            prior_artifacts[filename] = sha(PRIOR / filename)
            raw = gzip.decompress((PRIOR / filename).read_bytes())
            comparison = next(r for r in old_result['comparisons'] if r['fixture'] == fixture['label'])
            assert len(raw) == comparison[old_variant + '_bytes']
            assert hashlib.sha256(raw).hexdigest() == comparison[old_variant + '_sha256']
            if fixture['label'] in prior_reports:
                assert prior_reports[fixture['label']] == raw
            prior_reports[fixture['label']] = raw
    allocator_keys = sorted(k for k in os.environ if k.startswith(('MIMALLOC_', 'MI_'))
                            or k in ['LD_PRELOAD', 'MALLOC_CONF', 'GLIBC_TUNABLES', 'MALLOC_ARENA_MAX'])
    assert not allocator_keys, allocator_keys
    OUT.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    binaries = {}
    for variant in ['baseline', 'candidate']:
        original = qualification['binaries'][variant]['cli']
        assert sha(Path(original['path'])) == original['sha256']
        destination = WORK / 'bins' / variant / 'hyperdrc'
        destination.parent.mkdir(parents=True)
        shutil.copy2(original['path'], destination)
        assert sha(destination) == original['sha256']
        binaries[variant] = original | {'path': str(destination), 'qualified_path': original['path']}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'assessor_sha256': sha(E / 'assess_application_v2.py'),
        'preparation_sha256': sha(PREP / 'protocol.json'),
        'qualification_assessment_sha256': sha(E / 'qualification-assessment-v3.json'),
        'scope_sha256': sha(E / 'scope-v1.json'), 'design': design,
        'fixtures': fixtures, 'input_root': str(INPUTS), 'input_files': len(inputs),
        'prior_report_evidence': str(PRIOR), 'prior_artifacts_sha256': prior_artifacts,
        'binaries': binaries, 'cwd': str(ROOT), 'timeout_seconds': 1800,
        'inherited_allocator_settings': allocator_keys,
        'board_resource_format': 'wall_seconds\t%e\nuser_seconds\t%U\nsystem_seconds\t%S\nmax_rss_kib\t%M',
        'startup_processes_per_measurement': 50,
        'startup_metric': 'Wall time of a serial 50-process --help batch, including identical Python launch/wait overhead, divided by 50. Child user/system CPU deltas are divided by 50. This is not the internal Rust main duration.',
        'memory_metric': 'Maximum CLI process resident set in KiB, measured by GNU time. Not live/requested payload or allocation traffic. No startup RSS claim.',
        'comparison': 'Every complete JSON report compared without normalization. Preflight checks are excluded from the fixed 20-board timing sample; all 10 startup batches are retained.',
        'cache_state': 'Fresh processes with the same frozen input files. OS filesystem caches are not reset.',
        'original_gate_metrics_replaced': False, 'runtime_admitted': False, 'plan_complete': False}
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands, board_rows, startup_rows, reference, reference_help = [], [], [], {}, None

    def guards():
        verify(prep)
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'assess_application_v2.py') == protocol['assessor_sha256']
        assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
        assert sha(E / 'qualification-assessment-v3.json') == protocol['qualification_assessment_sha256']
        assert sha(E / 'scope-v1.json') == protocol['scope_sha256']
        assert all(sha(INPUTS / name) == digest for name, digest in inputs.items())
        assert all(sha(PRIOR / name) == digest for name, digest in prior_artifacts.items())
        for variant, binary in binaries.items():
            assert sha(Path(binary['path'])) == binary['sha256']
            for original in qualification['binaries'][variant].values():
                assert sha(Path(original['path'])) == original['sha256']

    def save():
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        (OUT / 'board-rows.json').write_text(json.dumps(board_rows, indent=2) + '\n')
        (OUT / 'startup-rows.json').write_text(json.dumps(startup_rows, indent=2) + '\n')

    def board(phase, block, fixture, variant):
        guards()
        name = f'{phase}-{block:02}-{fixture["label"]}-{variant}'
        resources = OUT / (name + '.resources.tsv')
        argv = ['timeout', '--signal=TERM', '--kill-after=10s', '1800',
                '/usr/bin/time', '--quiet', '-o', str(resources), '-f', protocol['board_resource_format'],
                binaries[variant]['path'], '--kicad-pcb', fixture['board_path'],
                '--check', 'layer-sanity', '--check', 'board-outline-sanity',
                '--check', 'min-copper-neck', '--check', 'drill-spacing',
                '--min-width', '0.0762', '--format', 'json']
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        process = subprocess.run(argv, cwd=ROOT, capture_output=True)
        elapsed = time.monotonic() - start
        record = {'name': name, 'phase': phase, 'block': block, 'fixture': fixture['label'],
                  'variant': variant, 'command': argv, 'exit_code': process.returncode, 'process_wall_seconds': elapsed}
        for stream in ['stdout', 'stderr']:
            path = OUT / (name + '.' + stream + '.gz')
            raw = getattr(process, stream)
            packed(path, raw)
            record[stream + '_path'] = path.name
            record[stream + '_sha256'] = sha(path)
            record[stream + '_bytes'] = len(raw)
        record.update(resources_path=resources.name, resources_sha256=sha(resources))
        commands.append(record)
        save()
        assert process.returncode == 1
        report = json.loads(process.stdout)
        assert report['violation_count'] == fixture['expected_findings'] == len(report['violations'])
        assert report['inputs']
        rows = dict(line.split('\t') for line in resources.read_text().splitlines())
        assert set(rows) == {'wall_seconds', 'user_seconds', 'system_seconds', 'max_rss_kib'}
        metrics = {k: (int(v) if k == 'max_rss_kib' else float(v)) for k, v in rows.items()}
        assert metrics['wall_seconds'] > 0 and metrics['max_rss_kib'] > 0
        metrics['cpu_seconds'] = metrics['user_seconds'] + metrics['system_seconds']
        metrics['process_wall_seconds'] = elapsed
        expected = reference.setdefault(fixture['label'], process.stdout)
        identical = expected == process.stdout and json.loads(expected) == report
        board_rows.append({'name': name, 'phase': phase, 'block': block, 'fixture': fixture['label'], 'variant': variant,
            'metrics': metrics, 'findings': fixture['expected_findings'], 'report_bytes': len(process.stdout),
            'report_sha256': hashlib.sha256(process.stdout).hexdigest(),
            'complete_bytes_and_values_equal_to_current_baseline': identical,
            'complete_bytes_equal_to_prior_endpoint_and_metadata_reports': process.stdout == prior_reports[fixture['label']]})
        save()
        guards()
        assert identical, 'Complete report mismatch; retain this failed preflight/sample and stop.'
        print(json.dumps({'finished': name, 'metrics': metrics, 'report_identical': identical}), flush=True)

    def startup(block, variant):
        nonlocal reference_help
        guards()
        name = f'startup-{block:02}-{variant}'
        argv = [binaries[variant]['path'], '--help']
        print(json.dumps({'start': name}), flush=True)
        processes = []
        usage_before = resource.getrusage(resource.RUSAGE_CHILDREN)
        start = time.monotonic()
        for _ in range(50):
            processes.append(subprocess.run(argv, cwd=ROOT, capture_output=True, timeout=20))
        elapsed = time.monotonic() - start
        usage_after = resource.getrusage(resource.RUSAGE_CHILDREN)
        records = [{'exit_code': p.returncode, 'stdout': p.stdout.decode(), 'stderr': p.stderr.decode()} for p in processes]
        path = OUT / (name + '.processes.json.gz')
        packed(path, (json.dumps(records, indent=2) + '\n').encode())
        record = {'name': name, 'phase': 'startup', 'block': block, 'variant': variant, 'command': argv,
                  'repetitions': 50, 'processes_path': path.name, 'processes_sha256': sha(path),
                  'batch_wall_seconds': elapsed, 'batch_user_seconds': usage_after.ru_utime - usage_before.ru_utime,
                  'batch_system_seconds': usage_after.ru_stime - usage_before.ru_stime}
        commands.append(record)
        save()
        assert all(r['exit_code'] == 0 and '--help' in r['stdout'] and not r['stderr'] for r in records)
        if reference_help is None:
            reference_help = records[0]['stdout']
        assert all(r['stdout'] == reference_help for r in records)
        metrics = {k: record['batch_' + k] / 50 for k in ['wall_seconds', 'user_seconds', 'system_seconds']}
        metrics['cpu_seconds'] = metrics['user_seconds'] + metrics['system_seconds']
        startup_rows.append({'name': name, 'phase': 'startup', 'block': block, 'variant': variant,
                             'repetitions': 50, 'metrics_seconds_per_process': metrics,
                             'all_complete_help_bytes_identical': True})
        save()
        guards()
        print(json.dumps({'finished': name, 'metrics_seconds_per_process': metrics}), flush=True)

    guards()
    for fixture in fixtures:
        for variant in fixture['order']:
            board('preflight', 0, fixture, variant)
    for block, order in enumerate(design['blocks'], 1):
        for fixture in fixtures:
            for variant in order:
                board('sample', block, fixture, variant)
        for variant in order:
            startup(block, variant)
    guards()
    assert len(commands) == 34 and len(board_rows) == 24 and len(startup_rows) == 10
    result = {'complete': True, 'commands': len(commands), 'preflight_board_runs': 4,
        'board_measurements': 20, 'startup_measurements': 10, 'startup_processes': 500,
        'complete_report_bytes_and_values_identical': all(r['complete_bytes_and_values_equal_to_current_baseline'] for r in board_rows),
        'complete_report_bytes_equal_to_prior_endpoint_and_metadata_reports': all(r['complete_bytes_equal_to_prior_endpoint_and_metadata_reports'] for r in board_rows),
        'inputs_sources_dependencies_binaries_and_prior_evidence_unchanged': True,
        'all_original_gates_preserved': True, 'no_sample_filtering_or_extension': True,
        'runtime_admitted': False, 'plan_complete': False}
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
