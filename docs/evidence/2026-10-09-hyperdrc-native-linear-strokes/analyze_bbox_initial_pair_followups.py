"""Preserve and compare the focused unconditioned and CPU-conditioned probes."""
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(EVIDENCE / 'bbox-initial-pair-candidate/sources.json')
native = read(Path(source['native_source_record']))
assert all(digest(ROOT / 'target/native-bbox-initial-pair/source' / name) == value for name, value in source['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
reports = {}
paths = []
for name in ['focused', 'conditioned']:
    folder = EVIDENCE / ('bbox-initial-pair-' + name + '-cost')
    manifest = read(folder / 'manifest.json')
    assert len(manifest['commands']) == 34 and all(command['exit_code'] == 0 for command in manifest['commands'])
    assert len(manifest['trials']) == 30 and len(manifest['summaries']) == 12
    assert digest(folder / 'probe.rs') == digest(EVIDENCE / ('bbox_initial_pair_' + name + '_probe.rs')) == manifest['probe_sha256']
    assert all(digest(Path(row['path'])) == row['sha256'] for row in manifest['libraries'].values())
    assert all(digest(folder / binary) == row['sha256'] for binary, row in manifest['binaries'].items())
    for case, _, _, count in manifest['cases']:
        trials = [trial for trial in manifest['trials'] if trial['case'] == case]
        assert len(trials) == 10
        witnesses = [row for row in trials[0]['rows'] if 'witness' in row]
        assert len(witnesses) == 3 and all(row['decided'] == count for row in witnesses)
        assert all([row for row in trial['rows'] if 'witness' in row] == witnesses for trial in trials)
    rows = []
    for summary in manifest['summaries']:
        for variant, fields in summary['variants'].items():
            samples = [next(row['nanos'] for row in trial['rows'] if row.get('name') == summary['name']) for trial in manifest['trials'] if trial['case'] == summary['case'] and trial['variant'] == variant]
            assert len(samples) == 5 and fields['nanos']['samples'] == samples
            assert fields['nanos']['median'] == statistics.median(samples)
        old = summary['variants']['baseline']['nanos']['median']
        new = summary['variants']['candidate']['nanos']['median']
        rows.append({'case': summary['case'], 'name': summary['name'], 'iterations': summary['iterations'], 'baseline_ns_per_box': old / summary['iterations'], 'candidate_ns_per_box': new / summary['iterations'], 'candidate_over_baseline': new / old})
    reports[name] = {'scope': manifest['scope'], 'all_exact_witnesses_preserved': True, 'rows': rows}
    paths.extend(path for path in sorted(folder.iterdir()) if path.is_file() and path.name not in manifest['binaries'])
    paths.extend(EVIDENCE / file for file in ['bbox_initial_pair_' + name + '_probe.rs', 'reproduce_bbox_initial_pair_' + name + '_cost.py'])
paths.append(Path(__file__))
record = {
    'decision': 'initial-pair optimization advances to isolated production validation: all nine conditioned follow-up query medians improve, while unconditioned short-process startup remains variable; no cold-start speedup or production admission is claimed',
    'initial_trial': 'diagnostics-38.json',
    'reports': reports,
    'source_unchanged': True, 'main_runtime_unchanged': True,
    'limitations': 'The original short-batch regression and all uncontrolled-startup samples remain in the record. Conditioning changes processor startup conditions and excludes that fixed activity from measured query spans; it is not a cold-start latency claim. No scalar cache or production source is changed. Full isolated production validation, report-byte replay and broader native runtime admission remain separate gates.',
    'artifacts': {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths},
}
(EVIDENCE / 'diagnostics-39.json').write_text(json.dumps(record, indent=2) + '\n')
for name, report in reports.items():
    print(name)
    for row in report['rows']:
        print(row['case'], row['name'], round(row['candidate_over_baseline'], 4))
