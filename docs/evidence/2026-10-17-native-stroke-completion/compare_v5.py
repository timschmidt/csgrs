"""Matched complete-operation measurements and retained representation deltas."""
import gzip,hashlib,json,resource,statistics,subprocess,time,zipfile
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];O=W/'comparison-v1'
OLD=ROOT/'target/properties/kicad-track-failures'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
def gz(p):
    with gzip.open(p,'rt') as f:return json.load(f)
def serialization_errors(value,path='$'):
    if isinstance(value,dict):
        return ([path] if 'serialization_error' in value else [])+[p for k,v in value.items() for p in serialization_errors(v,path+'.'+k)]
    if isinstance(value,list):return [p for i,v in enumerate(value) for p in serialization_errors(v,path+f'[{i}]')]
    return []
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()=='b137381247a03342ce5b6184710475fcc22c9d63'
assert read(W/'candidate-validation-v4/result.json')['complete']
assert read(W/'candidate-validation-v4/result.json')['unexpected_exits']==['curve-all','circuit-all','clippy']
assert read(W/'consumer-validation-v1/result.json')['unexpected_exits']==['circuit-all']
assert read(W/'snapshot-requalification-v1/result.json')['complete']
assert read(W/'candidate-preparation-v6/result.json')['production_code_and_test_bodies_unchanged_from_v5']
assert read(W/'ui-validation-v2/result.json')['unexpected_exits']==[]
assert read(W/'cost-preflight-v2/result.json')['complete']
assert read(OLD/'candidate-validation-v3/result.json')['unexpected_exits']==[]
inputs={'baseline':gz(W/'candidate-preparation-v1/baseline-source-hashes.json.gz'),
        'candidate':gz(W/'candidate-preparation-v6/source-hashes.json.gz')}
sources={'baseline':W/'baseline-source-v1','candidate':W/'candidate-source-v6'}
assert all(sha(sources[v]/p)==h for v in inputs for p,h in inputs[v].items())
assert all(sha(ROOT/p)==h for p,h in inputs['baseline'].items())
libraries={};cli={};readonly={}
for variant,validation in [('baseline',OLD/'candidate-validation-v3'),('candidate',W/'consumer-validation-v1')]:
    rows=[json.loads(s) for s in (validation/'native-cli.stdout').read_text().splitlines() if s.startswith('{')]
    libraries[variant]={n:next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']==n for p in row['filenames'] if p.endswith('.rlib')) for n in ['hypercurve','hyperlimit','serde_json']}
    cli[variant]=next(Path(row['executable']) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hyperdrc' and row.get('executable'))
    deps=libraries[variant]['hyperlimit'].parent
    readonly.update({str(p):sha(p) for p in deps.glob('*.rlib')});readonly.update({str(p):sha(p) for p in libraries[variant].values()});readonly[str(cli[variant])]=sha(cli[variant])
pins={str(p):sha(p) for p in [Path(__file__),W/'cost_probe_v2.rs',W/'cost_body_v2.rs',W/'cost-contract-v1.md',
    W/'candidate-validation-v4/result.json',W/'candidate-preparation-v6/source-hashes.json.gz',
    W/'ui-validation-v2/result.json',W/'consumer-validation-v1/result.json',W/'snapshot-requalification-v1/result.json',
    OLD/'candidate-validation-v3/result.json',OLD/'candidate-preparation-v3/source-hashes.json.gz']}
O.mkdir(exist_ok=False);(O/'inputs').mkdir()
fixtures=read(OLD/'baseline-preparation-v1/result.json')['fixtures'];source_boards={}
for fixture in fixtures:
    archive=Path(fixture['archive']);assert sha(archive)==fixture['sha256']
    label='cparti' if 'CPArti' in archive.name else 'hvp109a'
    with zipfile.ZipFile(archive) as z:payload=z.read(fixture['entry'])
    assert hashlib.sha256(payload).hexdigest()==fixture['entry_sha256'] and len(payload)==fixture['entry_bytes']
    path=O/'inputs'/(label+'.kicad_pcb');path.write_bytes(payload);source_boards[label]=path;pins[str(path)]=sha(path)
dump(O/'protocol.json',{'base_commit':'b137381247a03342ce5b6184710475fcc22c9d63','input_sha256':pins,
    'read_only_library_and_cli_sha256':readonly,'libraries':{v:{n:str(p) for n,p in libs.items()} for v,libs in libraries.items()},
    'rustc':subprocess.check_output(['rustc','-vV'],text=True),'hardware':subprocess.check_output(['lscpu'],text=True),
    'dependency_lock_sha256':sha(ROOT/'Cargo.lock'),'started_epoch':time.time(),'completed_targets_reopened':False,
    'final_validation_completed_before_measurement':True,
    'preserved_incomplete_attempt':'candidate-validation-v4 curve-all timeout and reproduced Hypercircuit compile failure',
    'complete_all_features_requalification':'consumer-validation-v1',
    'complete_circuit_snapshot_requalification':'snapshot-requalification-v1',
    'compiled_source_v5_differs_from_final_v6_only_in_seven_expected_artifacts':True,'pairs_per_case_cap':5,
    'timeout_seconds_per_command':1800,'address_space_bytes':8*1024**3,
    'phases':['fresh_process','repeat_after_model_drop'],
    'timing':'Exact input/path construction through stroke, analytic checks, native output materialization, reconstruction, independent grid classifications and snapshot Value construction. Excludes process startup, JSON encoding/file IO and model destruction.',
    'cache':'Fresh process per sample, then repeat after dropping model owners; scalar global caches may remain warm. Stronger evidence is allowed; every definite classification must match the independent oracle.',
    'artifact_contract':'Same mathematical stroke contract. Complete serialized scalar/native boundary observations are retained; declared subdivision/order/expression migrations are measured separately.'})
commands=[]
def limit():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=20s','1800s',*command],cwd=O,stdout=out,stderr=err,preexec_fn=limit,start_new_session=True)
    row={'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True);return r.returncode
probes={}
for variant in ['baseline','candidate']:
    libs=libraries[variant];deps=libs['hyperlimit'].parent
    flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(deps)]
    for name,path in libs.items():flags+=['--extern',name+'='+str(path)]
    probes[variant]=O/(variant+'-probe')
    assert run(variant+'-compile',[*flags,str(W/'cost_probe_v2.rs'),'-o',str(probes[variant])])==0
samples=[];first_snapshots={};snapshot_hashes={};repeat_differences=[]
for pair in range(5):
    for case in range(6):
        for cap in ['butt','round']:
            for variant in (['baseline','candidate'] if pair%2==0 else ['candidate','baseline']):
                name=f'pair-{pair:02d}-case-{case}-{cap}-{variant}';snapshot=O/(name+'.snapshot.json')
                assert run(name,[str(probes[variant]),str(case),cap,str(snapshot)])==0
                rows=read(O/(name+'.stdout'));states=read(snapshot);assert len(rows)==len(states)==2
                assert not serialization_errors(states),name
                snapshot_hashes[name]=sha(snapshot);key=(case,cap,variant)
                if key not in first_snapshots:first_snapshots[key]=states
                elif states!=first_snapshots[key]:repeat_differences.append({'pair':pair,'case':case,'cap':cap,'variant':variant,'snapshot':snapshot.name})
                for phase,row in enumerate(rows):
                    samples.append({'pair':pair,'case':case,'label':states[phase]['case'],'cap':cap,'variant':variant,'snapshot':snapshot.name}|row)
summary=[]
for case in range(6):
    for cap in ['butt','round']:
        for phase in ['fresh_process','repeat_after_model_drop']:
            pairs=[]
            for pair in range(5):
                select=lambda variant:next(r for r in samples if r['case']==case and r['cap']==cap and r['phase']==phase and r['pair']==pair and r['variant']==variant)
                before,after=select('baseline'),select('candidate')
                pairs.append({'pair':pair,'baseline':before,'candidate':after,
                    'elapsed_ratio':after['elapsed_ns']/before['elapsed_ns'],
                    **{k+'_delta':after[k]-before[k] for k in ['allocation_calls','allocated_bytes','retained_requested_bytes_delta','peak_extra_requested_bytes']}})
            summary.append({'case':case,'label':pairs[0]['baseline']['label'],'cap':cap,'phase':phase,
                'matched_completed_semantic_contract':all(p[v]['certified_completion'] for p in pairs for v in ['baseline','candidate']),
                'candidate_always_completes':all(p['candidate']['certified_completion'] for p in pairs),
                'baseline_always_completes':all(p['baseline']['certified_completion'] for p in pairs),
                'median_elapsed_ratio':statistics.median(p['elapsed_ratio'] for p in pairs),'all_pairs':pairs})
def differences(a,b,path='$'):
    if type(a)!=type(b):return [{'path':path,'baseline':a,'candidate':b}]
    if isinstance(a,dict):
        out=[]
        for k in sorted(a.keys()|b.keys()):
            if k not in a or k not in b:out.append({'path':path+'.'+k,'baseline':a.get(k),'candidate':b.get(k)})
            else:out+=differences(a[k],b[k],path+'.'+k)
        return out
    if isinstance(a,list):
        if len(a)!=len(b):return [{'path':path,'baseline':a,'candidate':b}]
        return [d for i,(x,y) in enumerate(zip(a,b)) for d in differences(x,y,path+f'[{i}]')]
    return [] if a==b else [{'path':path,'baseline':a,'candidate':b}]
artifact_deltas=[]
for case in range(6):
    for cap in ['butt','round']:
        before,after=(first_snapshots[(case,cap,v)] for v in ['baseline','candidate'])
        diffs=differences(before,after);name=f'case-{case}-{cap}-artifact-differences.json';dump(O/name,diffs)
        artifact_deltas.append({'case':case,'cap':cap,'difference_count':len(diffs),'path':name,'sha256':sha(O/name),
            'baseline_snapshot_bytes':(O/f'pair-00-case-{case}-{cap}-baseline.snapshot.json').stat().st_size,
            'candidate_snapshot_bytes':(O/f'pair-00-case-{case}-{cap}-candidate.snapshot.json').stat().st_size})
reports={}
for label,path in source_boards.items():
    for variant in ['baseline','candidate']:
        name=f'report-{label}-{variant}'
        command=[str(cli[variant]),'--kicad-pcb',str(path),'--check','layer-sanity','--check','board-outline-sanity','--check','min-copper-neck','--check','drill-spacing','--min-width','0.0762','--format','json','--allow-findings']
        assert run(name,command)==0
    reports[label]=differences(read(O/f'report-{label}-baseline.stdout'),read(O/f'report-{label}-candidate.stdout'))
assert all(sha(p)==h for p,h in pins.items()) and all(sha(p)==h for p,h in readonly.items())
assert all(sha(sources[v]/p)==h for v in inputs for p,h in inputs[v].items())
sizes={v:{'probe_bytes':probes[v].stat().st_size,'probe_sha256':sha(probes[v]),'cli_bytes':cli[v].stat().st_size,'cli_sha256':sha(cli[v])} for v in ['baseline','candidate']}
record={'complete':True,'commands':len(commands),'all_samples':samples,'paired_summary':summary,
    'exact_scalar_serialization_errors':0,
    'candidate_incomplete_cases':[{'case':r['case'],'cap':r['cap'],'phase':r['phase']} for r in summary if not r['candidate_always_completes']],
    'retained_snapshot_sha256':snapshot_hashes,'repeated_snapshot_differences':repeat_differences,
    'artifact_differences':artifact_deltas,'raw_full_board_report_differences':reports,'sizes':sizes,
    'source_and_read_only_artifacts_unchanged':True,'performance_improvement_claimed':False,'full_plan_complete':False}
dump(O/'result.json',record)
print(json.dumps({'complete':True,'commands':len(commands),'candidate_incomplete_cases':record['candidate_incomplete_cases'],
    'repeated_snapshot_differences':len(repeat_differences),'report_differences':{k:len(v) for k,v in reports.items()}}),flush=True)
