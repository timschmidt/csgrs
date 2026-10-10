"""Review residual width resources before testing a cheaper equivalent key mapping."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
COST = EVIDENCE / 'contact-index-directed-width-cost'
SOURCE = ROOT / 'target/contact-index-directed-width/source'
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
cost = read(COST / 'manifest.json')
sources = read(Path(cost['candidate_source_record']))
main = read(Path(sources['main_source_record']))
assert all(digest(SOURCE / name) == value for name, value in sources['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in main['source_hashes'].items())
assert len(cost['commands']) == 30 and all(command['exit_code'] == 0 for command in cost['commands'])
assert len(cost['trials']) == 20 and len(cost['summaries']) == 80
retained = read(COST / 'retained-libraries.json')
for variant, library in cost['libraries'].items():
    copy = retained['libraries'][variant]
    assert copy['original_path'] == library['path'] and copy['sha256'] == library['sha256']
    assert digest(Path(copy['retained_path'])) == library['sha256']
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
        assert len(rows[0]) == 40 and all(row == rows[0] for row in rows)
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

record = {
    'decision':'advance to full production validation: directed widths restore all twelve failing query rows, avoid unnecessary range saturation, and integer keys remove the previously measured ordinary-query regressions',
    'focused_record':'diagnostics-55.json',
    'previous_experiment':'diagnostics-54.json',
    'resource_gate_scope':cost['scope'],
    'matched_processes':20,
    'time_fixtures_per_row':1024,
    'memory_fixtures_per_row':64,
    'unchanged_witness_rows':len(unchanged),
    'corrected_witness_rows':len(deltas),
    'correctness_witness_deltas':deltas,
    'allocation_samples_repeat_exactly':True,
    'unchanged_allocation_rows':sum(not row['witness_changed'] and all(metric['delta']==0 for metric in row['metrics'].values()) for row in memory),
    'saturation_ratios':{row['name']:row['candidate_over_baseline'] for row in timing if 'saturation' in row['name']},
    'ordinary_controls_to_review':[row for row in timing if row['name'] in ['exact_sparse_cloned','exact_crowded_first']],
    'timing_rows':timing,
    'allocation_rows':memory,
    'binary_sizes':sizes,
    'measured_libraries_retained':retained,
    'main_runtime_unchanged':True,
    'full_suites_or_report_replay_run_for_this_candidate':False,
    'resource_review': {
        'unchanged_controls':'All 28 unchanged witness rows preserve all four allocation metrics, with five identical memory samples. Their timing ratios range from 0.8635 to 1.1704, with median 0.9912; no uniform speedup is claimed.',
        'previous_ordinary_regressions':'The fresh matched ratios for exact_sparse_cloned and exact_crowded_first are 0.9131 and 0.9941. This resolves the two controls that delayed the previous candidate; it is not a separately matched causal comparison against that candidate.',
        'unfavorable_samples':'The MAX-width first-query median is 17.0% slower (385.4 to 451.1 ns/query), with broadly overlapping samples. Other unchanged medians regress by at most 5.9%, with overlapping sample ranges. All unfavorable samples are retained.',
        'restored_output_cost':'Corrected queries allocate their required candidate vectors: 32 retained bytes for a singleton or 512 retained bytes for 64 contacts. These outputs were incorrectly absent in the baseline.',
        'binary_cost':'Standalone probe ELF size increases by 392 bytes for timing and 424 bytes for allocation accounting.',
        'scope':'The exactness repair is the intended benefit. Full application validation and report preservation still gate admission.',
    },
    'next_step':'Review the full matched samples, then run complete production validation and report replay before admission.',
    'artifacts':{str(path.relative_to(EVIDENCE)):digest(path) for path in [COST/'manifest.json',COST/'retained-libraries.json',EVIDENCE/'diagnostics-55.json']},
}
(EVIDENCE/'diagnostics-56.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'corrected_witness_rows':len(deltas),'unchanged_witness_rows':len(unchanged),'saturation_ratios':record['saturation_ratios'],'ordinary_ratios':{row['name']:row['candidate_over_baseline'] for row in record['ordinary_controls_to_review']},'binary_sizes':sizes},indent=2))
