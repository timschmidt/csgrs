"""Collect final scoped evidence only after every validation, cost and oracle gate passes."""
from pathlib import Path
import gzip,hashlib,json,re,shutil
record=Path(__file__).resolve().parent
load=lambda p:json.loads(p.read_bytes())
info=lambda p:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
inputs={'validation':Path('/tmp/drc-pad-source-final-validation3'),'cost':Path('/tmp/drc-pad-source-final-cost3'),'source-audit':Path('/tmp/drc-pad-source-final-source-audit'),'report-replay':Path('/tmp/drc-pad-source-final-reports'),'drill-audit':Path('/tmp/drc-pad-source-drill-audit')}
validation=inputs['validation']; steps=load(validation/'status.json');assert len(steps)==8 and all(s['exit_code']==0 for s in steps)
sources=load(validation/'sources.json');assert sources['unchanged'];assert all(info(Path(p))['sha256']==h for p,h in sources['source_hashes'].items())
assert load(validation/'fixtures.json')['unchanged']
cost=load(inputs['cost']/'manifest.json');assert cost['source_unchanged'] and cost['fixture_exit_code']==0 and not cost['previous_row_deltas']
source=load(inputs['source-audit']/'manifest.json');assert source['source_unchanged']
assert all(b['result']['all_admitted'] for b in source['variants']['candidate']['complete_boards'])
reports=load(inputs['report-replay']/'manifest.json');assert reports['source_unchanged'] and all(t['exit_code']==0 for t in reports['trials'])
audit=load(inputs['report-replay']/'finding-audit.json')
drills=load(inputs['drill-audit']/'manifest.json');assert drills['source_unchanged'] and all(row['exit_code']==0 for row in drills['results'])
for label,path in inputs.items():
    destination=record/label;destination.mkdir(exist_ok=True)
    for p in path.rglob('*'):
        if not p.is_file() or p.suffix not in ['.log','.json','.jsonl','.gz','.diff'] or (label=='report-replay' and p.parent.name in ['trial-1','trial-2']):continue
        q=destination/p.relative_to(path);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy(p,q)
oracle=record/'oracle-inputs';oracle.mkdir(exist_ok=True)
for p in Path('/tmp/drc-pad-source-oracle-final-inputs').glob('*.json'):
    (oracle/(p.name+'.gz')).write_bytes(gzip.compress(p.read_bytes(),mtime=0))
for p in Path('/tmp/drc-pad-source-drill-oracle-inputs').glob('*.json'):
    (oracle/('drills-'+p.name+'.gz')).write_bytes(gzip.compress(p.read_bytes(),mtime=0))
tests={}
for label in ['drc-tests','consumer-drc','consumer-curves']:
    rows=[tuple(map(int,m)) for m in re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', (validation/(label+'.log')).read_text())]
    tests[label]=dict(zip(['passed','failed','ignored'],map(sum,zip(*rows))))
fixture=(inputs['cost']/'fixture.log').read_text()
manifest=dict(base_revision='b4d29cb484956a82d1c139b3c670e66bdc3c51cb',status='kept-scoped-progress',scope='Exact KiCad pad source arithmetic and placement, combined rounded/chamfered corners, outer-copper layer alias, and typed required marker/net-class boundaries. All 39 properties, 14 targets and crate future work remain in scope.',contract=[
'Exact source decimals precede center/size/delta/ratio/diameter arithmetic and native primitive construction. Present malformed numeric tokens retain source failure; absent optional fields retain their documented defaults. Nonzero source dimensions outside binary64 range stay native.',
'Pad coordinates are footprint-relative, pad angles are absolute in the board frame, and KiCad positive rotation turns toward negative Y. Selected chamfers preserve rounding on unselected corners. Oval decomposition and custom local-primitive limitations remain explicit.',
'F&B.Cu expands to the two outer copper layers even on a multilayer board; the complete CPArti board exposed this separate existing bug after the earlier passing suite.',
'Required copper/drill marker projections return typed uncertainty with STRICT/budget guards. Copper markers also reject retained source-construction failures. Native class windows retain unknown comparisons; class aggregation delays marker projection until a finding requires it. Missing finite broad-phase anchors retain candidates.',
'These formulas retain exact authored values and source conventions without reproducing KiCad integer-unit rounding. The source oracle checks positions, exact primitive areas and envelopes, not whole-application KiCad equivalence or every downstream DRC heuristic.'
],references='reference-sources.json',baseline=dict(record='baseline/tests.json',passed=1,failed=21),marker_failure=dict(record='marker-failure/manifest.json',new_failures=3),initial_candidate='initial-candidate/full/manifest.json',early_marker_regression=dict(record='early-marker-validation/status.json',cause='Eager class marker projection produced six uncertainties instead of the existing single length uncertainty. The implementation now delays projection; the original no-partial-length test is unchanged and passes.'),outer_alias_regression=dict(record='before-outer-alias/outer-alias-regression.log',passed=0,failed=1),first_oracle=dict(record='first-source-oracle/manifest.json',harness_correction='The initial oracle omitted HVP109A string-only net names; corrected in the oracle, without changing production net parsing.',production_correction='Two complete-board CPArti pads used F&B.Cu; expanded to four real-layer features. All original board bytes are preserved.'),validation=dict(record='validation/status.json',sources='validation/sources.json',source_files=len(sources['source_hashes']),all_gates_passed=True,tests=tests,new_regressions=dict(pad_source_integration=33,native_net_scope=2,outer_layer_alias=1,total=36)),snapshots=dict(record='snapshot-versions.json',boards=6,versions_per_board=160,before=19,after=20,all_other_candidate_bytes_identical=True),cost=dict(record='cost/manifest.json',rows=cost['rows'],all_231_prior_witnesses_unchanged=True,trials=5,fixture_seconds=float(re.search(r'fixture_smoke_total ms=([\d.]+)',fixture)[1])/1000,fixture_gate_seconds=120,fixture_peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',fixture)[1])),source_audit=dict(record='source-audit/manifest.json',loader_rows=19,trials_per_library=5,iterations_per_row=50,changed_correctness_witnesses=source['deltas'],complete_boards={b['label']:b['result'] for b in source['variants']['candidate']['complete_boards']},scope=source['scope']),reports=dict(record='report-replay/manifest.json',finding_audit='report-replay/finding-audit.json',boards={label:dict(violations=d['violations'],bytes=d['bytes'],sha256=d['sha256']) for label,d in reports['reports'].items()},complete_bytes_identical_in_two_fresh_processes=True,limitation='Correct source placement and layer mapping change findings. Every unmatched finding and metadata delta is retained. The independent source oracle does not independently validate all readiness heuristics.'),reused_evidence='unchanged-core-source-audit.json',remaining='remaining-source-audit.json',push='No push. The complete repository-wide pre-push matrix was not run.',artifact_scope='Correctness capability costs and process RSS/binary bytes; no matched speedup, allocation-count or library-size improvement claim.')
manifest['drill_source_audit'] = dict(record='drill-audit/manifest.json', results=[row['result'] for row in drills['results']], scope=drills['scope'])
manifest['reports']['review'] = 'report-replay/review.json'
manifest['artifact_audit'] = 'artifact-audit.json'
(record/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({k:manifest[k] for k in ['validation','cost','source_audit','reports']},indent=2))
