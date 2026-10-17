"""Compare complete CLI reports with each original project archive extracted."""
import hashlib,json,os,resource,shutil,stat,subprocess,time,zipfile
from pathlib import Path,PurePosixPath
W=Path(__file__).resolve().parent;ROOT=W.parents[2];O=W/'project-reports-v1'
OLD=ROOT/'target/properties/kicad-track-failures'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert read(W/'comparison-v1/result.json')['complete']
assert not read(W/'comparison-v1/result.json')['candidate_incomplete_cases']
protocol=read(W/'comparison-v1/protocol.json');binaries={}
for variant in ['baseline','candidate']:
    validation=OLD/'candidate-validation-v3' if variant=='baseline' else W/'consumer-validation-v1'
    rows=[json.loads(s) for s in (validation/'native-cli.stdout').read_text().splitlines() if s.startswith('{')]
    binaries[variant]=next(Path(row['executable']) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hyperdrc' and row.get('executable'))
    assert sha(binaries[variant])==protocol['read_only_library_and_cli_sha256'][str(binaries[variant])]
O.mkdir(exist_ok=False);(O/'inputs').mkdir();inputs={};boards={};fixtures=read(OLD/'baseline-preparation-v1/result.json')['fixtures']
for fixture in fixtures:
    archive=Path(fixture['archive']);assert sha(archive)==fixture['sha256']
    label='cparti' if 'CPArti' in archive.name else 'hvp109a';base=O/'inputs'/label;base.mkdir()
    with zipfile.ZipFile(archive) as z:
        names=set()
        for entry in z.infolist():
            relative=PurePosixPath(entry.filename)
            assert not relative.is_absolute() and '..' not in relative.parts and '\\' not in entry.filename
            assert not stat.S_ISLNK(entry.external_attr>>16)
            assert entry.filename not in names;names.add(entry.filename)
            dest=base/relative
            if entry.is_dir():dest.mkdir(parents=True,exist_ok=True);continue
            dest.parent.mkdir(parents=True,exist_ok=True)
            with z.open(entry) as src,dest.open('xb') as out:shutil.copyfileobj(src,out)
            assert dest.stat().st_size==entry.file_size
            inputs[str(dest)]={'sha256':sha(dest),'bytes':entry.file_size,'archive':str(archive),'entry':entry.filename}
    boards[label]=base/fixture['entry'];assert sha(boards[label])==fixture['entry_sha256']
pins={str(p):sha(p) for p in [Path(__file__),W/'comparison-v1/result.json',W/'comparison-v1/protocol.json',*binaries.values()]}
dump(O/'protocol.json',{'input_sha256':pins,'project_files':inputs,'original_archives':fixtures,
    'source_scope':'All files from each original archive, including .kicad_pro and .kicad_dru sidecars. Both variants receive the same path.',
    'relationship':'Separate from the earlier standalone board-file comparison; neither result is substituted for the other.',
    'cache':'Fresh CLI process for each board and variant. The library full-board gate instead visits both projects sequentially in one test process.',
    'timeout_seconds_per_command':1800,'address_space_bytes':8*1024**3,'performance_comparison_claimed':False})
commands=[];reports={}
def limit():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        r=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=20s','1800s',*command],cwd=O,stdout=out,stderr=err,preexec_fn=limit,start_new_session=True)
    row={'name':name,'command':command,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,
         **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True);return r.returncode
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
for label,board in boards.items():
    for variant in ['baseline','candidate']:
        name=label+'-'+variant
        assert run(name,[str(binaries[variant]),'--kicad-pcb',str(board),'--check','layer-sanity','--check','board-outline-sanity',
            '--check','min-copper-neck','--check','drill-spacing','--min-width','0.0762','--format','json','--allow-findings'])==0
    before,after=[read(O/(label+'-'+variant+'.stdout')) for variant in ['baseline','candidate']]
    diff=differences(before,after);dump(O/(label+'-differences.json'),diff)
    reports[label]={'difference_count':len(diff),'difference_file':label+'-differences.json',
        'baseline_violation_count':before['violation_count'],'candidate_violation_count':after['violation_count'],
        'baseline_waived_count':before['waived_count'],'candidate_waived_count':after['waived_count']}
assert all(sha(p)==h for p,h in pins.items()) and all(sha(p)==r['sha256'] for p,r in inputs.items())
dump(O/'result.json',{'complete':True,'commands':len(commands),'reports':reports,'all_report_values_identical':all(r['difference_count']==0 for r in reports.values()),
    'all_project_inputs_and_binaries_unchanged':True,'project_files_retained':len(inputs),'performance_comparison_claimed':False})
print(json.dumps({'complete':True,'reports':reports}),flush=True)
