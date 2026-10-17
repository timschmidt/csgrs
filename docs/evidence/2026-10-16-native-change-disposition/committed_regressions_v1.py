"""Replay the complete proposed public DRC regression file against qualified committed libraries."""
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import subprocess
import time

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
WORK=ROOT/'target/properties/native-change-disposition'
OUT=WORK/'committed-proposed-regressions-v1'
SOURCE=E/'pending-source-v1/crates/hyperdrc/tests/native_linear_strokes.rs'
BINARY=OUT/'native_linear_strokes'


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')

qualified=read(WORK/'committed-validation-v1/result.json')
assert qualified['complete']
cli=next(row for row in read(WORK/'committed-validation-v1/commands.json') if row['name']=='native-cli')
assert cli['exit_code']==0
assert sha(WORK/'committed-validation-v1/native-cli.stdout')==cli['stdout_sha256']
artifacts={}
for line in (WORK/'committed-validation-v1/native-cli.stdout').read_text().splitlines():
    row=json.loads(line)
    if row.get('reason')=='compiler-artifact' and 'lib' in row['target']['kind'] and row['target']['name'] in ['hyperdrc','hypercurve','hyperlimit']:
        libraries=[Path(path) for path in row['filenames'] if path.endswith('.rlib')]
        assert len(libraries)==1
        artifacts[row['target']['name']]={'path':str(libraries[0]),'sha256':sha(libraries[0]),'cargo_artifact':row}
assert set(artifacts)=={'hyperdrc','hypercurve','hyperlimit'}
OUT.mkdir(exist_ok=False)
protocol={'runner_sha256':sha(__file__),'source':str(SOURCE),'source_sha256':sha(SOURCE),
          'complete_unchanged_regression_file':True,'baseline_production_modified':False,
          'qualified_libraries':artifacts,'qualification_sha256':sha(WORK/'committed-validation-v1/result.json'),
          'rustc':subprocess.check_output(['rustc','-vV'],text=True),
          'elapsed_limit_seconds_per_command':1800,'address_space_limit_bytes':8*1024**3,
          'purpose':'Expose which proposed mathematical claims remain unmet by the committed alternative. Do not count expected legacy failures as successful geometry or weaken their assertions.',
          'completed_cargo_target_reopened':False,'performance_claimed':False}
dump(OUT/'protocol.json',protocol)
env=os.environ.copy()
for key in ['RUSTFLAGS','CARGO_ENCODED_RUSTFLAGS','RUSTC_WRAPPER','RUSTC_WORKSPACE_WRAPPER','RUSTDOCFLAGS']:env.pop(key,None)
env.update({'TMPDIR':str(OUT),'RUST_BACKTRACE':'0'})

def limits():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))

rows=[]
def run(name,command):
    before=time.monotonic()
    with (OUT/(name+'.stdout')).open('wb') as stdout,(OUT/(name+'.stderr')).open('wb') as stderr:
        p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/(name+'.resources')),
                          'timeout','--signal=TERM','--kill-after=20s','1800s',*command],
                         cwd=OUT,env=env,stdout=stdout,stderr=stderr,preexec_fn=limits,start_new_session=True)
    record={'name':name,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-before,
            **{suffix+'_sha256':sha(OUT/(name+'.'+suffix)) for suffix in ['stdout','stderr','resources']}}
    rows.append(record)
    dump(OUT/'commands.json',rows)
    print(json.dumps({key:record[key] for key in ['name','exit_code','elapsed_seconds']}),flush=True)
    return p.returncode

command=['rustc','--edition=2024','--test',str(SOURCE),'-C','opt-level=3','-C','debuginfo=0',
         '-L','dependency='+str(WORK/'committed-target-v1/release/deps'),'-o',str(BINARY)]
for name,row in artifacts.items():command+=['--extern',name+'='+row['path']]
compiled=run('compile',command)
if compiled==0:
    code=run('regressions',[str(BINARY),'--test-threads=1','--nocapture'])
else:code=None
assert sha(SOURCE)==protocol['source_sha256'] and sha(__file__)==protocol['runner_sha256']
assert all(sha(row['path'])==row['sha256'] for row in artifacts.values())
text=(OUT/'regressions.stdout').read_text() if code is not None else ''
counts=re.search(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',text)
record={'complete':compiled==0 and code in [0,101] and counts is not None,
        'compile_exit_code':compiled,'test_exit_code':code,
        'counts':dict(zip(['passed','failed','ignored'],map(int,counts.groups()))) if counts else None,
        'failed_tests':re.findall(r'^test (\S+) \.\.\. FAILED$',text,re.MULTILINE),
        'source_and_libraries_unchanged':True,'regressions_weakened':False,
        'binary_sha256':sha(BINARY) if BINARY.exists() else None,'full_plan_complete':False}
dump(OUT/'result.json',record)
print(json.dumps(record),flush=True)
raise SystemExit(0 if record['complete'] else 101)
