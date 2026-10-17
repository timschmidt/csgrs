"""Observe every public malformed-source case against the freshly built baseline."""
import gzip,hashlib,json,re,resource,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'baseline-probe-v1';P=W/'prototype-v1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
qualification=read(W/'baseline-validation-v1/result.json')
assert qualification['complete'] and not qualification['unexpected_exits']
with gzip.open(W/'baseline-preparation-v1/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(W/'baseline-source-v1'/p)==h for p,h in inputs.items())
rows=[json.loads(line) for line in (W/'baseline-validation-v1/native-cli.stdout').read_text().splitlines() if line.startswith('{')]
libraries={name:next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']==name for p in row['filenames'] if p.endswith('.rlib'))
    for name in ['hyperdrc','hypercurve','hyperlimit','serde_json','anyhow']}
deps=libraries['hyperlimit'].parent
readonly={str(p):sha(p) for p in deps.glob('*.rlib')}
readonly.update({str(p):sha(p) for p in libraries.values()})
pins={str(p):sha(p) for p in [Path(__file__),*P.iterdir(),W/'baseline-validation-v1/result.json',W/'baseline-preparation-v1/source-hashes.json.gz'] if p.is_file()}
O.mkdir(exist_ok=False)
dump(O/'protocol.json',{'base_commit':'ac14e99174318291a548749395552ab982a7d097','source_inputs':len(inputs),
    'libraries':{n:str(p) for n,p in libraries.items()},'read_only_libraries_sha256':readonly,'inputs_sha256':pins,
    'native_qualification_sha256':sha(W/'baseline-validation-v1/result.json'),
    'elapsed_limit_seconds_per_command':180,'address_space_limit_bytes':8*1024**3,
    'rustc':subprocess.check_output(['rustc','-vV'],text=True),'completed_targets_reopened':False,
    'purpose':'Distinguish every malformed-source observation and retain the unchanged full proposed fidelity regressions; no production source is changed.'})
commands=[]
def limits():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
    start=time.monotonic()
    with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
        result=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=10s','180s',*command],
            stdout=out,stderr=err,cwd=O,preexec_fn=limits,start_new_session=True)
    row={'name':name,'command':command,'exit_code':result.returncode,'elapsed_seconds':time.monotonic()-start,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row);dump(O/'commands.json',commands)
    print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
    return result.returncode
flags=['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(deps)]
for name,p in libraries.items():flags+=['--extern',name+'='+str(p)]
assert run('probe-compile',[*flags,str(P/'reproduce.rs'),'-o',str(O/'probe')])==0
assert run('probe',[str(O/'probe'),str(O/'fixtures')])==0
assert run('public-tests-compile',[*flags,'--test',str(P/'public_tests.rs'),'-o',str(O/'public-tests')])==0
public=run('public-tests',[str(O/'public-tests'),'--nocapture','--test-threads=1'])
assert public in [0,101]
assert run('original-regressions-compile',[*flags,'--test',str(P/'original_native_linear_strokes.rs'),'-o',str(O/'original-regressions')])==0
original=run('original-regressions',[str(O/'original-regressions'),'--nocapture','--test-threads=1'])
assert original in [0,101]
assert all(sha(p)==h for p,h in readonly.items()) and all(sha(p)==h for p,h in pins.items())
assert all(sha(W/'baseline-source-v1'/p)==h for p,h in inputs.items())
observations=read(O/'probe.stdout');invalid=[r for r in observations['observations'] if r['expected']=='invalid_input']
categories={'missing_feature':0,'successful_geometry':0,'retained_failure':0}
for r in invalid:
    v=r['observation']
    if not v['copper_count']:categories['missing_feature']+=1
    elif all(f['area']['status']=='error' for f in v['features']):categories['retained_failure']+=1
    else:categories['successful_geometry']+=1
counts=re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed;', (O/'original-regressions.stdout').read_text())
assert len(counts)==1
dump(O/'result.json',{'complete':True,'invalid_source_observations':len(invalid),'invalid_source_categories':categories,
    'original_complete_regression_file_unchanged':True,'original_regressions_exit_code':original,
    'public_tests_exit_code':public,
    'original_regressions':{'passed':int(counts[0][0]),'failed':int(counts[0][1])},
    'original_regression_scope':'Each test stops at its first failed assertion; no credit for unexecuted later assertions.',
    'budget_escape':observations['exhausted_scope_escaped_board'],'recovery':observations['recovery'],
    'read_only_libraries_unchanged':True,'baseline_sources_unchanged':True,'main_sources_modified':False})
print(json.dumps({'complete':True,'invalid_source_categories':categories,'original_regressions':counts[0]}),flush=True)
