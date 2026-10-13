"""Verify every fixed application sample and summarize paired descriptive costs."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

from common_v2 import E, PREP, sha, verify


def compare(fixture, metric, baseline, candidate, unit):
    assert len(baseline) == len(candidate) == 5 and all(v > 0 for v in baseline)
    ratios = [c / b for b, c in zip(baseline, candidate)]
    return {'fixture': fixture, 'metric': metric, 'unit': unit,
            'baseline': baseline, 'candidate': candidate, 'paired_candidate_over_baseline': ratios,
            'baseline_median': statistics.median(baseline), 'candidate_median': statistics.median(candidate),
            'ratio_of_medians': statistics.median(candidate) / statistics.median(baseline),
            'median_paired_ratio': statistics.median(ratios), 'paired_ratio_range': [min(ratios), max(ratios)],
            'candidate_higher_blocks': sum(c > b for b, c in zip(baseline, candidate)),
            'candidate_lower_blocks': sum(c < b for b, c in zip(baseline, candidate)),
            'equal_blocks': sum(c == b for b, c in zip(baseline, candidate)),
            'paired_deltas': [c - b for b, c in zip(baseline, candidate)]}


def main():
    out = E / 'application-costs-v1'
    destination = E / 'application-assessment-v1.json'
    assert not destination.exists()
    protocol = json.loads((out / 'protocol.json').read_text())
    result = json.loads((out / 'result.json').read_text())
    commands = json.loads((out / 'commands.json').read_text())
    board_rows = json.loads((out / 'board-rows.json').read_text())
    startup_rows = json.loads((out / 'startup-rows.json').read_text())
    prep = json.loads((PREP / 'protocol.json').read_text())
    qualification = json.loads((E / 'qualification-assessment-v2.json').read_text())
    assert result['complete'] and qualification['candidate_original_gates_passed']
    assert sha(E / 'application_costs_v1.py') == protocol['runner_sha256']
    assert sha(Path(__file__)) == protocol['assessor_sha256']
    assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
    assert sha(E / 'qualification-assessment-v2.json') == protocol['qualification_assessment_sha256']
    assert sha(E / 'scope-v1.json') == protocol['scope_sha256']
    verify(prep)
    prior = Path(protocol['prior_report_evidence'])
    assert all(sha(prior / name) == digest for name, digest in protocol['prior_artifacts_sha256'].items())
    inputs = json.loads(gzip.decompress((prior / 'input-hashes.json.gz').read_bytes()))
    input_root = Path(protocol['input_root'])
    assert len(inputs) == protocol['input_files'] == 108
    assert all(sha(input_root / name) == digest for name, digest in inputs.items())
    assert {str(p.relative_to(input_root)) for p in input_root.rglob('*') if p.is_file()} == set(inputs)
    for variant, binary in protocol['binaries'].items():
        assert sha(Path(binary['path'])) == binary['sha256']
        assert Path(binary['path']).stat().st_size == binary['bytes']
        identity = qualification['binaries'][variant]['cli']
        assert identity['path'] == binary['qualified_path']
        assert identity['sha256'] == binary['sha256'] and identity['bytes'] == binary['bytes']
        for original in qualification['binaries'][variant].values():
            assert sha(Path(original['path'])) == original['sha256']
    fixtures = protocol['fixtures']
    assert fixtures == json.loads((prior / 'protocol.json').read_text())['fixtures']
    design = json.loads((E / 'scope-v1.json').read_text())['fixed_application_design_if_candidate_gates_pass']
    assert design == protocol['design']
    assert protocol['inherited_allocator_settings'] == []
    assert protocol['startup_processes_per_measurement'] == 50
    expected = [f'preflight-00-{f["label"]}-{v}' for f in fixtures for v in f['order']]
    for block, order in enumerate(design['blocks'], 1):
        expected += [f'sample-{block:02}-{f["label"]}-{v}' for f in fixtures for v in order]
        expected += [f'startup-{block:02}-{v}' for v in order]
    assert [c['name'] for c in commands] == expected
    assert len(commands) == result['commands'] == 34
    assert len(board_rows) == 24 and len(startup_rows) == 10
    raw_reports, help_texts = {}, []
    boards = {r['name']: r for r in board_rows}
    startups = {r['name']: r for r in startup_rows}
    for command in commands:
        name, variant = command['name'], command['variant']
        binary = protocol['binaries'][variant]['path']
        if command['phase'] == 'startup':
            assert command['command'] == [binary, '--help'] and command['repetitions'] == 50
            path = out / command['processes_path']
            assert sha(path) == command['processes_sha256']
            processes = json.loads(gzip.decompress(path.read_bytes()))
            assert len(processes) == 50
            assert all(p['exit_code'] == 0 and '--help' in p['stdout'] and not p['stderr'] for p in processes)
            help_texts.extend(p['stdout'] for p in processes)
            row = startups[name]
            metrics = {k: command['batch_' + k] / 50 for k in ['wall_seconds', 'user_seconds', 'system_seconds']}
            metrics['cpu_seconds'] = metrics['user_seconds'] + metrics['system_seconds']
            assert row['metrics_seconds_per_process'] == metrics
            assert row['repetitions'] == 50 and row['all_complete_help_bytes_identical']
        else:
            fixture = next(f for f in fixtures if f['label'] == command['fixture'])
            resources = out / command['resources_path']
            assert sha(resources) == command['resources_sha256']
            assert command['command'] == ['timeout', '--signal=TERM', '--kill-after=10s', '1800',
                '/usr/bin/time', '--quiet', '-o', str(resources), '-f', protocol['board_resource_format'],
                binary, '--kicad-pcb', fixture['board_path'], '--check', 'layer-sanity',
                '--check', 'board-outline-sanity', '--check', 'min-copper-neck', '--check', 'drill-spacing',
                '--min-width', '0.0762', '--format', 'json']
            assert command['exit_code'] == 1
            for stream in ['stdout', 'stderr']:
                path = out / command[stream + '_path']
                assert sha(path) == command[stream + '_sha256']
                raw = gzip.decompress(path.read_bytes())
                assert len(raw) == command[stream + '_bytes']
                if stream == 'stdout':
                    raw_reports[name] = raw
            report = json.loads(raw_reports[name])
            assert report['violation_count'] == len(report['violations']) == fixture['expected_findings']
            assert report['inputs']
            fields = dict(line.split('\t') for line in resources.read_text().splitlines())
            assert set(fields) == {'wall_seconds', 'user_seconds', 'system_seconds', 'max_rss_kib'}
            metrics = {k: int(v) if k == 'max_rss_kib' else float(v) for k, v in fields.items()}
            metrics['cpu_seconds'] = metrics['user_seconds'] + metrics['system_seconds']
            metrics['process_wall_seconds'] = command['process_wall_seconds']
            row = boards[name]
            assert row['metrics'] == metrics and row['findings'] == fixture['expected_findings']
            assert row['report_bytes'] == len(raw_reports[name])
            assert row['report_sha256'] == hashlib.sha256(raw_reports[name]).hexdigest()
        assert row['variant'] == variant and row['block'] == command['block'] and row['phase'] == command['phase']
    assert len(help_texts) == 500 and len(set(help_texts)) == 1
    report_comparisons = []
    for fixture in fixtures:
        label = fixture['label']
        reference = raw_reports[f'preflight-00-{label}-baseline']
        rows = [r for r in board_rows if r['fixture'] == label]
        assert len(rows) == 12
        old = gzip.decompress((prior / (label + '-candidate.json.gz')).read_bytes())
        for row in rows:
            raw = raw_reports[row['name']]
            assert raw == reference and json.loads(raw) == json.loads(reference)
            assert row['complete_bytes_and_values_equal_to_current_baseline']
            assert row['complete_bytes_equal_to_prior_endpoint_and_metadata_reports'] == (raw == old)
        report_comparisons.append({'fixture': label, 'complete_reports': len(rows), 'bytes': len(reference),
            'sha256': hashlib.sha256(reference).hexdigest(), 'findings': fixture['expected_findings'],
            'all_current_reports_byte_and_value_identical': True,
            'identical_to_prior_endpoint_and_metadata_reports': reference == old})
    comparisons = []
    for fixture in fixtures:
        for metric in ['process_wall_seconds', 'cpu_seconds', 'max_rss_kib']:
            values = {v: [next(r for r in board_rows if r['phase'] == 'sample' and r['block'] == b
                              and r['fixture'] == fixture['label'] and r['variant'] == v)['metrics'][metric]
                           for b in range(1, 6)] for v in ['baseline', 'candidate']}
            comparisons.append(compare(fixture['label'], metric, values['baseline'], values['candidate'],
                                       'KiB' if metric == 'max_rss_kib' else 'seconds'))
    for metric in ['wall_seconds', 'cpu_seconds']:
        values = {v: [next(r for r in startup_rows if r['block'] == b and r['variant'] == v)
                      ['metrics_seconds_per_process'][metric] for b in range(1, 6)] for v in ['baseline', 'candidate']}
        comparisons.append(compare('startup-help', metric, values['baseline'], values['candidate'], 'seconds per process'))
    sizes = {kind: {'baseline_bytes': qualification['binaries']['baseline'][kind]['bytes'],
                    'candidate_bytes': qualification['binaries']['candidate'][kind]['bytes'],
                    'candidate_minus_baseline_bytes': qualification['binaries']['candidate'][kind]['bytes'] - qualification['binaries']['baseline'][kind]['bytes']}
             for kind in ['cli', 'test']}
    assert result['complete_report_bytes_and_values_identical']
    assert result['complete_report_bytes_equal_to_prior_endpoint_and_metadata_reports'] == all(
        r['identical_to_prior_endpoint_and_metadata_reports'] for r in report_comparisons)
    assessment = {'assessor_sha256': sha(Path(__file__)), 'preflight_board_runs': 4, 'board_measurements': 20,
        'startup_measurements': 10, 'startup_processes': 500,
        'reports': report_comparisons, 'comparisons': comparisons, 'binary_sizes': sizes,
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'inputs_sources_dependencies_binaries_and_prior_evidence_unchanged': True,
        'limits': [
            'Five fixed blocks, no sample extension or filtering. Ratios are descriptive, not significance claims.',
            'Preflight runs check reports and are excluded from cost samples. Fresh processes share OS file caches.',
            'RSS is resident process memory, not allocated/live payload. Startup batches include identical launch/wait overhead.',
            'Whole CLI timing does not replace either original assertion metric.',
            'Inherited scalar/native costs, other platforms and the complete properties/target/future-work plan remain open.',
        ], 'runtime_admitted': False, 'plan_complete': False}
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'reports': report_comparisons, 'comparisons': comparisons, 'binary_sizes': sizes,
                      'runtime_admitted': False}))


if __name__ == '__main__':
    main()
