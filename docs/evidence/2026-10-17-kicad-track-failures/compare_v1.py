"""Compare identical completed loads and actual CLI reports; retain all costs."""
import gzip,hashlib,json,os,resource,statistics,subprocess,time,zipfile
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'comparison-v1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
qualifications={};libraries={};binaries={};sourcepins={};readonly={}
for variant in ['baseline','candidate']:
    q=W/(variant+'-validation-'+('v3' if variant=='candidate' else 'v1'));qualifications[variant]=read(q/'result.json')
    assert qualifications[variant]['complete'] and not qualifications[variant]['unexpected_exits']
    with gzip.open(W/(variant+'-preparation-'+('v3' if variant=='candidate' else 'v1')+'/source-hashes.json.gz'),'rt') as f:inputs=json.load(f)
    sourcepins.update({str(W/(variant+'-source-'+('v3' if variant=='candidate' else 'v1'))/p):h for p,h in inputs.items()})
    rows=[json.loads(line) for line in (q/'native-cli.stdout').read_text().splitlines() if line.startswith('{')]
    libraries[variant]={name:next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']==name for p in row['filenames'] if p.endswith('.rlib')) for name in ['hyperdrc','hypercurve','hyperlimit','serde_json']}
    dependency=next(Path(p).parent for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hyperlimit' for p in row['filenames'] if p.endswith('.rlib'))
    libraries[variant]['dependency']=dependency
    binaries[variant]=next(Path(row['executable']) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hyperdrc' and row.get('executable'))
    readonly.update({str(p):sha(p) for p in dependency.iterdir() if p.is_file() and p.suffix in ['.rlib','.rmeta']})
    readonly.update({str(p):sha(p) for name,p in libraries[variant].items() if name!='dependency'})
    readonly[str(binaries[variant])]=sha(binaries[variant])
assert all(sha(p)==h for p,h in sourcepins.items())
O.mkdir(exist_ok=False);(O/'inputs').mkdir();(O/'scratch').mkdir()
fixtures={}
synthetic='(kicad_pcb (net 1 signal) '+''.join(f'(segment (start {i*4} 0) (end {i*4+2} 0) (width 2) (layer F.Cu) (net 1))' for i in range(64))+')'
p=O/'inputs/ordinary64.kicad_pcb';p.write_text(synthetic);fixtures['ordinary64']=p
for fixture in read(W/'baseline-preparation-v1/result.json')['fixtures']:
    archive=Path(fixture['snapshot_archive']);assert sha(archive)==fixture['sha256']
    with zipfile.ZipFile(archive) as z:data=z.read(fixture['entry'])
    assert len(data)==fixture['entry_bytes'] and hashlib.sha256(data).hexdigest()==fixture['entry_sha256']
    name='cparti' if fixture['entry'].startswith('CPArti') else 'hvp109a'
    p=O/'inputs'/(name+'.kicad_pcb');p.write_bytes(data);fixtures[name]=p
identified=read(W/'source-disks-v1/result.json');assert identified['complete']
disk_sources={name:read(W/'source-disks-v1'/(name+'.json')) for name in fixtures}
assert all(sha(fixtures[name])==m['source_sha256'] for name,m in disk_sources.items())
pins={str(p):sha(p) for p in [Path(__file__),W/'load_cost_v1.rs',W/'disk_corpus_v1.rs',W/'cost-contract-v2.md',W/'prototype-v1/cases.json',*fixtures.values(),*(W/'source-disks-v1').iterdir()] if p.is_file()}
protocol={'baseline_commit':'ac14e99174318291a548749395552ab982a7d097','source_inputs_sha256':sourcepins,
    'read_only_library_and_cli_sha256':readonly,'inputs_sha256':pins,'fixtures':{n:{'path':str(p),'bytes':p.stat().st_size,'sha256':sha(p)} for n,p in fixtures.items()},
    'rustc':subprocess.check_output(['rustc','-vV'],text=True),'hardware':subprocess.check_output(['lscpu'],text=True),
    'pairs':5,'process_order':'alternate baseline/candidate by pair; one fresh process per fixture and variant',
    'phases':['fresh_process_load','repeat_after_projection_and_drop'],
    'timed':'read source, parse and materialize BoardModel; excludes process startup, output projection/serialization, and drop',
    'contract':str(W/'cost-contract-v2.md'),'elapsed_limit_seconds_per_command':1800,'address_space_limit_bytes':8*1024**3,
    'declared_geometry_migration':{n:len(v['disks']) for n,v in disk_sources.items()},
    'completed_cargo_targets_reopened':False,'certified_exact_stroke_fidelity_claimed':False}
dump(O/'protocol.json',protocol)
commands=[];env=os.environ.copy();env['TMPDIR']=str(O/'scratch')
def limits():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        result=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=20s','1800s',*map(str,command)],
            stdout=out,stderr=err,cwd=O,env=env,preexec_fn=limits,start_new_session=True)
    row={'name':name,'command':list(map(str,command)),'exit_code':result.returncode,'elapsed_seconds':time.monotonic()-start,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands)
    print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
    return result.returncode
sizes={}
for variant,libs in libraries.items():
    flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(libs['dependency'])]
    for name,p in libs.items():
        if name!='dependency':flags+=['--extern',name+'='+str(p)]
    assert run(variant+'-compile',[*flags,W/'load_cost_v1.rs','-o',O/(variant+'-load')])==0
    sizes[variant]={'cli_bytes':binaries[variant].stat().st_size,'cli_sha256':sha(binaries[variant]),
        'probe_bytes':(O/(variant+'-load')).stat().st_size,'probe_sha256':sha(O/(variant+'-load'))}
assert run('disk-corpus-compile',[*flags,W/'disk_corpus_v1.rs','-o',O/'disk-corpus'])==0
assert run('disk-corpus',[O/'disk-corpus',W/'source-disks-v1/hvp109a.json'])==0
disk_checks=read(O/'disk-corpus.stdout');assert len(disk_checks)==15
samples={};snapshots={};geometry_deltas={}
for pair in range(5):
    for fixture,input_path in fixtures.items():
        order=['baseline','candidate'] if pair%2==0 else ['candidate','baseline']
        for variant in order:
            name=f'load-{pair:02d}-{fixture}-{variant}';snapshot=O/(name+'.geometry.json')
            assert run(name,[O/(variant+'-load'),input_path,snapshot])==0
            samples[name]=read(O/(name+'.stdout'));snapshots[name]=sha(snapshot)
            assert len(samples[name])==2
        old=read(O/f'load-{pair:02d}-{fixture}-baseline.geometry.json')
        new=read(O/f'load-{pair:02d}-{fixture}-candidate.geometry.json')
        declared={d['positive_segment_index']:d for d in disk_sources[fixture]['disks']}
        segment_index=0;kept=[];added=[]
        for index,feature in enumerate(new['copper']):
            if feature['kind']=='Segment':
                if segment_index in declared:added.append({'board_feature_index':index,'source':declared[segment_index],'feature':feature})
                else:kept.append(feature)
                segment_index+=1
            else:kept.append(feature)
        assert segment_index==disk_sources[fixture]['positive_segments']
        assert len(added)==len(declared)
        assert new|{'copper':kept}==old,fixture
        if fixture in geometry_deltas:assert geometry_deltas[fixture]==added
        else:geometry_deltas[fixture]=added
for fixture in fixtures:
    for variant in ['baseline','candidate']:
        assert len({digest for name,digest in snapshots.items() if '-'+fixture+'-' in name and name.endswith(variant)})==1
def differences(a,b,path='$'):
    if type(a)!=type(b):return [{'path':path,'baseline':a,'candidate':b}]
    if isinstance(a,dict):
        if a.keys()!=b.keys():return [{'path':path,'baseline_keys':list(a),'candidate_keys':list(b)}]
        return [row for key in a for row in differences(a[key],b[key],path+'.'+key)]
    if isinstance(a,list):
        if len(a)!=len(b):return [{'path':path,'baseline_length':len(a),'candidate_length':len(b)}]
        return [row for i,(x,y) in enumerate(zip(a,b)) for row in differences(x,y,path+f'[{i}]')]
    return [] if a==b else [{'path':path,'baseline':a,'candidate':b}]
reports={};unexpected=[]
for fixture,input_path in fixtures.items():
    for variant in ['baseline','candidate']:
        name='report-'+fixture+'-'+variant
        assert run(name,[binaries[variant],'--kicad-pcb',input_path,'--check','copper-net-intent','--format','json','--allow-findings'])==0
    diffs=differences(read(O/('report-'+fixture+'-baseline.stdout')),read(O/('report-'+fixture+'-candidate.stdout')))
    assert diffs,fixture
    reports[fixture]=diffs
    unexpected += [{'fixture':fixture,**d} for d in diffs if not(d['path'].endswith('.check_version') and d.get('baseline')==20 and d.get('candidate')==21)]
for fixture in ['cparti','hvp109a']:
    for variant in ['baseline','candidate']:
        name='report-smoke-'+fixture+'-'+variant
        assert run(name,[binaries[variant],'--kicad-pcb',fixtures[fixture],
            '--check','layer-sanity','--check','board-outline-sanity','--check','min-copper-neck','--check','drill-spacing',
            '--min-width','0.0762','--format','json','--allow-findings'])==0
    diffs=differences(read(O/('report-smoke-'+fixture+'-baseline.stdout')),read(O/('report-smoke-'+fixture+'-candidate.stdout')))
    reports[fixture+'-smoke']=diffs
    unexpected += [{'fixture':fixture+'-smoke',**d} for d in diffs if not(d['path'].endswith('.check_version') and d.get('baseline')==20 and d.get('candidate')==21)]
invalid=[]
for case in read(W/'prototype-v1/cases.json'):
    if case['expected']!='invalid_input':continue
    p=O/'inputs'/('invalid-'+case['name']+'.kicad_pcb')
    p.write_text('(kicad_pcb (segment (layer F.Cu) '+case['fields']+'))')
    name='invalid-cli-'+case['name']
    assert run(name,[binaries['candidate'],'--kicad-pcb',p,'--check','copper-net-intent','--format','json','--allow-findings'])==1
    assert (O/(name+'.stdout')).stat().st_size==0
    assert 'KiCad segment 1:' in (O/(name+'.stderr')).read_text()
    invalid.append({'name':case['name'],'source_sha256':sha(p),'source':str(p),'exit_code':1,'empty_stdout':True,'diagnostic_survives':True})
summary=[]
for fixture in fixtures:
    for phase in protocol['phases']:
        pairs=[]
        for pair in range(5):
            rows={v:next(r for r in samples[f'load-{pair:02d}-{fixture}-{v}'] if r['phase']==phase) for v in ['baseline','candidate']}
            old,new=rows['baseline'],rows['candidate']
            pairs.append({'pair':pair,'elapsed_ratio':new['elapsed_ns']/old['elapsed_ns'],
                **{key+'_delta':new[key]-old[key] for key in ['allocation_calls','allocated_bytes','retained_requested_bytes_delta','peak_extra_requested_bytes']},
                **rows})
        summary.append({'fixture':fixture,'phase':phase,'matched_completed_contract':not disk_sources[fixture]['disks'],
            'changed_workload_added_disks':len(disk_sources[fixture]['disks']),
            'median_elapsed_ratio':statistics.median(p['elapsed_ratio'] for p in pairs),'all_pairs':pairs})
assert all(sha(p)==h for p,h in pins.items()) and all(sha(p)==h for p,h in readonly.items()) and all(sha(p)==h for p,h in sourcepins.items())
dump(O/'result.json',{'complete':True,'commands':len(commands),'paired_summary':summary,'all_samples':samples,
    'all_previously_materialized_features_identical':True,'geometry_snapshot_sha256':snapshots,
    'geometry_additions':geometry_deltas,'independent_full_board_disk_checks':disk_checks,
    'raw_report_differences':reports,'unexpected_report_differences':unexpected,'malformed_cli':invalid,
    'sizes':sizes,'source_and_read_only_artifacts_unchanged':True,'general_exact_source_stroke_fidelity_claimed':False,'performance_improvement_claimed':False})
print(json.dumps({'complete':True,'commands':len(commands),'unexpected_report_differences':len(unexpected),'malformed_cli':len(invalid),
    'median_elapsed_ratios':[{k:r[k] for k in ['fixture','phase','median_elapsed_ratio']} for r in summary]}),flush=True)
