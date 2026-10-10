"""Review the early cost gate and the remaining symmetric-width regression."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
COST = EVIDENCE / 'contact-index-exact-widths-cost'
SOURCE = ROOT / 'target/contact-index-exact-widths/source'
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
cost = read(COST / 'manifest.json')
sources = read(Path(cost['candidate_source_record']))
main = read(Path(sources['main_source_record']))
assert all(digest(SOURCE / name) == value for name, value in sources['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in main['source_hashes'].items())
assert len(cost['commands']) == 30 and all(command['exit_code'] == 0 for command in cost['commands'])
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

sizes = {}
for mode in ['time','memory']:
    a,b = [cost['binaries'][variant+'-'+mode] for variant in ['baseline','candidate']]
    for variant,record in [('baseline',a),('candidate',b)]:
        assert digest(COST / (variant+'-'+mode)) == record['sha256']
    sizes[mode] = {'baseline_bytes':a['bytes'], 'candidate_bytes':b['bytes'], 'delta_bytes':b['bytes']-a['bytes']}

symmetric = read(EVIDENCE / 'contact-index-symmetric-width/manifest.json')
assert [row['exit_code'] for row in symmetric['commands']] == [0,0,0,101]
assert symmetric['probes']['baseline']['result']['visited_indices'] == 1
assert symmetric['probes']['candidate']['result']['visited_indices'] == 65
assert symmetric['probes']['baseline']['result']['candidates'] == symmetric['probes']['candidate']['result']['candidates'] == [64]
for variant, record in symmetric['probes'].items():
    assert digest(EVIDENCE / 'contact-index-symmetric-width' / (variant+'.rs')) == record['probe_sha256']
    assert record['library'] == cost['libraries'][variant]
    assert record['implementation_sha256'] == cost['probe_sources'][variant]['implementation_sha256']
record = {
    'decision':'not retained: the special-case refinement fixes the measured zero-endpoint saturation, but an actual symmetric nonzero-endpoint producer still expands one necessary index visit to 65',
    'focused_record':'diagnostics-51.json',
    'previous_experiment':'diagnostics-50.json',
    'resource_gate_scope':cost['scope'],
    'matched_processes':20,
    'time_fixtures_per_row':1024,
    'memory_fixtures_per_row':64,
    'unchanged_witness_rows':len(unchanged),
    'corrected_witness_rows':len(deltas),
    'correctness_witness_deltas':deltas,
    'allocation_samples_repeat_exactly':True,
    'unchanged_allocation_rows':sum(not row['witness_changed'] and all(metric['delta']==0 for metric in row['metrics'].values()) for row in memory),
    'zero_endpoint_saturation_ratios':{row['name']:row['candidate_over_baseline'] for row in timing if row['name'].startswith('max_width_saturation_')},
    'symmetric_endpoint_counterexample':symmetric,
    'timing_rows':timing,
    'allocation_rows':memory,
    'binary_sizes':sizes,
    'main_runtime_unchanged':True,
    'full_suites_or_report_replay_run_for_this_candidate':False,
    'next_step':'Evaluate a general subtraction-residual upper bound instead of enumerating exact endpoint cases. Verify its width bounds with an independent exact rational oracle across subnormals, overflow boundaries, exponent gaps and cancellation before testing another production candidate.',
    'research': {
        'primary_source':'https://people.eecs.berkeley.edu/~jrs/papers/robustr.pdf',
        'relevant_algorithm':'Two-Diff paragraph following Theorem 7',
        'limitation':'The paper explicitly excludes overflow and underflow. Any use here must separately handle nonfinite intermediates and justify gradual-underflow behavior for addition/subtraction; no implementation or validation of that alternative is claimed yet.',
        'license_reference':'https://www.cs.cmu.edu/~quake/robust.html identifies the accompanying C implementation as public domain. No external code has been copied into production.',
    },
    'artifacts':{str(path.relative_to(EVIDENCE)):digest(path) for path in [COST/'manifest.json',EVIDENCE/'diagnostics-51.json',EVIDENCE/'contact-index-symmetric-width/manifest.json']},
}
(EVIDENCE/'diagnostics-52.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'corrected_witness_rows':len(deltas),'unchanged_witness_rows':len(unchanged),'zero_endpoint_ratios':record['zero_endpoint_saturation_ratios'],'symmetric_visits':{'baseline':1,'candidate':65},'binary_sizes':sizes},indent=2))
