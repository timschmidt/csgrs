"""Review candidate-index correctness, resource deltas and complete report bytes."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
COST = EVIDENCE / 'contact-index-production-cost-corrected'
REPLAY = EVIDENCE / 'contact-index-production-replay'
SOURCE = ROOT / 'target/contact-index-outward-width/source'
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
cost = read(COST / 'manifest.json')
failed_harness = read(EVIDENCE / 'contact-index-production-cost/failure.json')
assert [command['exit_code'] for command in failed_harness['commands']] == [0, 0, 1]
for name, expected in failed_harness['artifacts'].items():
    assert digest(EVIDENCE / 'contact-index-production-cost' / name) == expected
replay = read(REPLAY / 'manifest.json')
validation = read(EVIDENCE / 'diagnostics-49.json')
sources = read(Path(cost['candidate_source_record']))
main = read(Path(sources['main_source_record']))
assert all(digest(SOURCE / name) == value for name, value in sources['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in main['source_hashes'].items())
assert len(cost['commands']) == 30 and all(command['exit_code'] == 0 for command in cost['commands'])
assert len(replay['commands']) == 20 and all(command['exit_code'] == 0 for command in replay['commands'])
assert len(cost['trials']) == 20 and len(cost['summaries']) == 72
for library in cost['libraries'].values():
    assert digest(Path(library['path'])) == library['sha256']
assert digest(COST / 'probe.rs') == cost['probe_sha256']
for variant, directory in [('baseline', ROOT / 'target/ray-filter-finite-guard/source'), ('candidate', SOURCE)]:
    probe = cost['probe_sources'][variant]
    assert digest(Path(probe['path'])) == probe['sha256']
    text = (directory / 'crates/hypercurve/src/curve_region_boolean.rs').read_text()
    begin = text.index('#[derive(Default)]\nstruct ContactPointIndex {')
    implementation = text[begin:text.index('\nfn replace_topology_vertex(', begin)]
    assert implementation in Path(probe['path']).read_text()
    assert hashlib.sha256(implementation.encode()).hexdigest() == probe['implementation_sha256']

witnesses = {}
for mode in ['time', 'memory']:
    for variant in ['baseline', 'candidate']:
        trials = [trial for trial in cost['trials'] if trial['mode'] == mode and trial['variant'] == variant]
        assert len(trials) == 5
        rows = [[row for row in trial['rows'] if 'witness' in row] for trial in trials]
        assert len(rows[0]) == 36 and all(row == rows[0] for row in rows)
        witnesses[mode, variant] = {row['witness']:row for row in rows[0]}
        if variant == 'candidate':
            assert all(row['correct'] == row['count'] for row in rows[0])
        if mode == 'memory':
            measures = [[{key:value for key,value in row.items() if key != 'nanos'} for row in trial['rows'] if 'name' in row] for trial in trials]
            assert all(rows == measures[0] for rows in measures)

deltas = []
unchanged = []
for name, baseline in witnesses['time', 'baseline'].items():
    candidate = witnesses['time', 'candidate'][name]
    if baseline == candidate:
        unchanged.append(name)
    else:
        assert baseline['kind'] == 'query'
        assert baseline['correct'] < candidate['correct'] == candidate['count']
        deltas.append({'name':name, 'baseline':baseline, 'candidate':candidate})
assert deltas
changed = {row['name'] for row in deltas}
assert {name for name in witnesses['memory','baseline'] if witnesses['memory','baseline'][name] != witnesses['memory','candidate'][name]} == changed

timing = []
memory = []
for row in cost['summaries']:
    name, variants = row['name'], row['variants']
    if row['mode'] == 'time':
        a, b = variants['baseline']['nanos'], variants['candidate']['nanos']
        timing.append({'name':name, 'witness_changed':name in changed, 'baseline_samples_ns':a['samples'], 'candidate_samples_ns':b['samples'], 'baseline_median_ns':a['median'], 'candidate_median_ns':b['median'], 'candidate_over_baseline':b['median']/a['median']})
    else:
        fields = {field:{'baseline':variants['baseline'][field]['median'], 'candidate':variants['candidate'][field]['median'], 'delta':variants['candidate'][field]['median']-variants['baseline'][field]['median']} for field in variants['baseline']}
        memory.append({'name':name,'witness_changed':name in changed,'metrics':fields})
unchanged_ratios = [row['candidate_over_baseline'] for row in timing if not row['witness_changed']]

assert replay['historical_row_deltas'] == 0
assert len(read(REPLAY / 'rows.json')) == 231
previous = read(EVIDENCE / 'ray-filter-production-replay/manifest.json')
for board, record in replay['reports'].items():
    assert record['sha256'] == previous['reports'][board]['sha256']
    for variant in ['baseline','candidate']:
        for repetition in [1,2]:
            raw = gzip.decompress((REPLAY / f'{variant}-reports-{repetition}-{board}.json.gz').read_bytes())
            assert len(raw) == record['bytes'] and hashlib.sha256(raw).hexdigest() == record['sha256']
for variant in replay['variants'].values():
    for filename, record in variant['libraries'].items():
        assert digest(Path(filename)) == record['sha256']
    for record in variant['binaries'].values():
        assert digest(Path(record['path'])) == record['sha256']

sizes = {}
for mode in ['time','memory']:
    a,b = [cost['binaries'][variant+'-'+mode] for variant in ['baseline','candidate']]
    for variant,record in [('baseline',a),('candidate',b)]:
        assert digest(COST / (variant+'-'+mode)) == record['sha256']
    sizes[mode] = {'baseline_bytes':a['bytes'], 'candidate_bytes':b['bytes'], 'delta_bytes':b['bytes']-a['bytes']}

record = {
    'decision':'not retained in this form: exactness and report preservation pass, but unconditionally rounding the exact MAX width to infinity causes an avoidable 2.86-3.62x scan regression in the saturation control',
    'counterexample_record':'diagnostics-47.json',
    'focused_record':'diagnostics-48.json',
    'production_validation_record':'diagnostics-49.json',
    'production_suites':validation['suites'],
    'original_production_gate_passed':validation['original_board_gate'],
    'historical_capability_rows_preserved':231,
    'complete_reports':replay['reports'],
    'matched_probe': {'scope':cost['scope'], 'processes':20, 'time_fixtures_per_row':1024, 'memory_fixtures_per_row':64, 'workloads':9, 'measures_per_process':36, 'unchanged_witness_rows':len(unchanged), 'corrected_witness_rows':len(deltas), 'allocation_samples_repeat_exactly':True},
    'correctness_witness_deltas':deltas,
    'unchanged_timing_ratios':{'minimum':min(unchanged_ratios),'median':statistics.median(unchanged_ratios),'maximum':max(unchanged_ratios)},
    'timing_rows':timing,
    'allocation_rows':memory,
    'binary_sizes':sizes,
    'maximum_rss_medians_kib':{mode:{variant:statistics.median(trial['maximum_rss_kib'] for trial in cost['trials'] if trial['mode']==mode and trial['variant']==variant) for variant in ['baseline','candidate']} for mode in ['time','memory']},
    'main_runtime_unchanged_when_measured':True,
    'failed_harness_preserved':'contact-index-production-cost/failure.json: first cost harness failed compilation because Real has no From<usize>; explicit u32 conversions correct the bounded fixture coordinates. No resource sample ran before the correction.',
    'resource_review': {
        'unchanged_allocation_rows':sum(not row['witness_changed'] and all(metric['delta']==0 for metric in row['metrics'].values()) for row in memory),
        'corrected_output_cost':'The twelve formerly empty candidate lists now allocate their required output: 32 retained bytes per singleton query or 512 retained bytes for 64 contacts. Five allocation samples repeat exactly.',
        'avoidable_saturation':'The independently verified source box [0, MAX] has exactly representable width MAX. Unconditional next_up makes it infinity and causes queries to scan 64 additional negative-key boxes. All three saturation query phases regress, with disjoint baseline/candidate sample ranges.',
        'other_timing_controls':'The full samples and unfavorable ordinary controls are retained; no uniform speedup is claimed.',
        'next_candidate':'Preserve the exact subtraction for equal endpoints or a zero endpoint; round every remaining width upward. This retains the completeness proof while avoiding unnecessary saturation and zero-width inflation.',
    },
    'next_step':'Freeze and validate the refined width calculation in a new production checkout. Repeat the correctness, report and resource gates before admitting any runtime change.',
    'limitations':['Direct index microbenchmarks use the exact extracted implementation and actual public production point producers, with the public STRICT bounds wrapper replacing private access. They do not establish full Boolean performance.', 'The signed-zero adapter failure is a private mathematical-order contract case; actual signed-zero producer reachability is unverified.', 'No complete public Boolean failure, broad native-stroke admission, full-workspace validation or properties.md completion is claimed.'],
    'artifacts':{str(path.relative_to(EVIDENCE)):digest(path) for path in [COST/'manifest.json',REPLAY/'manifest.json',EVIDENCE/'diagnostics-49.json',EVIDENCE/'diagnostics-48.json',EVIDENCE/'diagnostics-47.json',EVIDENCE/'contact-index-production-cost/failure.json']},
}
(EVIDENCE / 'diagnostics-50.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'corrected_witness_rows':len(deltas),'unchanged_witness_rows':len(unchanged),'unchanged_timing_ratios':record['unchanged_timing_ratios'],'binary_sizes':sizes},indent=2))
