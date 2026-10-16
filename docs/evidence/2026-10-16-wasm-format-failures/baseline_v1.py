"""Reproduce public binding failures against the verified current generated WASM."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E = ROOT / 'docs/evidence/2026-10-16-wasm-format-failures'
PREVIOUS = E.parent / '2026-10-16-export-projection-contracts'
WORK = ROOT / 'target/properties/wasm-format-failures'
OUT = WORK / 'baseline-v1'
BASE = '3d335db43eac00d003bc2fb0ed1beb2a9c19d79b'
OLD = ROOT / 'target/properties/export-projection-contracts'
MODULE = OLD / 'wasm-target-v1/wasm-bindings-node/csgrs.js'
TYPESCRIPT = OLD / 'wasm-source-v1/tests/wasm/node_modules/typescript'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def read(path):
    return json.loads(Path(path).read_text())


assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip() == BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
parent = read(PREVIOUS / 'checkpoint-v1.json')
assert all(sha(ROOT / path) == digest for path,digest in parent['pending_native_sha256'].items())
assert sha(ROOT / 'docs/properties-status.json') == parent['ledger_after_sha256']
with gzip.open(PREVIOUS / 'final-input-hashes-v1.json.gz','rt') as stream:
    inputs = json.load(stream)
assert all(sha(ROOT / path) == digest for path,digest in inputs.items() if path != 'docs/properties-status.json')
inputs['docs/properties-status.json'] = parent['ledger_after_sha256']
wasm = read(PREVIOUS / 'wasm-validation-v1/result.json')
assert wasm['all_commands_passed']
artifacts = {path:record['sha256'] for path,record in wasm['artifacts'].items()}
assert read(TYPESCRIPT / 'package.json')['version'] == '5.7.3'
artifacts.update({str(path):sha(path) for path in TYPESCRIPT.rglob('*') if path.is_file()})
assert all(sha(path) == digest for path,digest in artifacts.items())
OUT.mkdir(exist_ok=False)
with gzip.open(OUT / 'source-hashes.json.gz','wt') as stream:
    json.dump(inputs,stream,sort_keys=True)
dump(OUT / 'baseline-artifact-hashes.json',artifacts)
ts = OUT / 'typescript'
ts.mkdir()
shutil.copyfile(MODULE.with_suffix('.d.ts'),ts / 'csgrs.d.ts')
shutil.copyfile(E / 'io_contracts_v1.ts',ts / 'io_contracts.ts')
probe_hashes = {str(path):sha(path) for path in [E/'io_contracts_v1.cjs',E/'io_contracts_v1.ts',ts/'csgrs.d.ts',ts/'io_contracts.ts',ROOT/'tests/wasm/export_projection.cjs',ROOT/'tests/wasm/geometry_certainty.cjs']}
protocol = {'runner_sha256':sha(__file__),'head':BASE,'source_hashes_sha256':sha(OUT/'source-hashes.json.gz'),
            'baseline_artifact_hashes_sha256':sha(OUT/'baseline-artifact-hashes.json'),'module':str(MODULE),
            'probes':probe_hashes,'typescript_version':'5.7.3',
            'node':subprocess.check_output(['node','--version'],text=True).strip(),
            'elapsed_limit_seconds_per_command':120,'address_space_limit_bytes_per_process':64*1024**3,
            'completed_targets_modified':False,'native_changes_admitted':False,'full_plan_complete':False}
dump(OUT/'protocol.json',protocol)
env=os.environ.copy()
env.pop('NODE_OPTIONS',None)
env.update({'TMPDIR':str(OUT),'RUST_BACKTRACE':'0'})
rows=[]


def limits():
    resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))


def run(name, command, expected):
    start=time.monotonic()
    with (OUT/(name+'.stdout')).open('wb') as stdout,(OUT/(name+'.stderr')).open('wb') as stderr:
        p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/(name+'.resources')),
                          'timeout','--signal=TERM','--kill-after=10s','120s',*command],
                         cwd=ROOT,env=env,stdout=stdout,stderr=stderr,preexec_fn=limits,start_new_session=True)
    row={'name':name,'command':command,'exit_code':p.returncode,'expected_exit_code':expected,
         'elapsed_seconds':time.monotonic()-start,
         **{s+'_sha256':sha(OUT/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    rows.append(row)
    dump(OUT/'commands.json',rows)
    print(json.dumps({k:row[k] for k in ['name','exit_code','expected_exit_code','elapsed_seconds']}),flush=True)
    assert p.returncode==expected,name


run('existing-certainty',['node',str(ROOT/'tests/wasm/geometry_certainty.cjs'),str(MODULE)],0)
run('existing-export',['node',str(ROOT/'tests/wasm/export_projection.cjs'),str(MODULE)],0)
run('new-io-contracts',['node',str(E/'io_contracts_v1.cjs'),str(MODULE)],1)
text=(OUT/'new-io-contracts.stdout').read_text()
assert '# tests 12' in text and '# pass 1' in text and '# fail 11' in text
run('new-typescript',['node',str(TYPESCRIPT/'lib/tsc.js'),'--noEmit','--strict','--target','ES2020',
                     '--lib','ES2020,DOM,ESNext.Disposable','--module','Node16','--moduleResolution','Node16',str(ts/'io_contracts.ts')],2)
text=(OUT/'new-typescript.stdout').read_text()
assert 'GeometryIoError' in text and 'fromSTL' in text
assert all(sha(ROOT/path)==digest for path,digest in inputs.items())
assert all(sha(path)==digest for path,digest in artifacts.items())
assert all(sha(path)==digest for path,digest in probe_hashes.items())
assert sha(__file__)==protocol['runner_sha256']
dump(OUT/'result.json',{'complete':True,'all_expected_exits':True,'public_tests':{'passed':1,'failed':11,'total':12},
                      'existing_consumers_passed':True,'typescript_missing_api_and_types_reproduced':True,
                      'sources_and_artifacts_unchanged':True,'completed_targets_modified':False})
