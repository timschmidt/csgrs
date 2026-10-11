"""Capped exact symbolic trig lattice probes; no coefficient expansion in the oracle."""
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = ROOT / 'docs/evidence/2026-10-10-scalar-evaluation-failure'
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
TARGET = ROOT / 'target/scalar-evaluation-candidate/target'
BUILD = EVIDENCE / sys.argv[1]
OUT = EVIDENCE / sys.argv[2]
WORK = ROOT / 'target' / sys.argv[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert read(BUILD/'result.json')['exit_code'] == 0
source_hashes = json.loads(gzip.decompress((BUILD/'source-hashes.json.gz').read_bytes()))
main_hashes = json.loads(gzip.decompress((EVIDENCE/'candidate-preparation/main-hashes.json.gz').read_bytes()))
assert not OUT.exists() and not WORK.exists()
OUT.mkdir(); WORK.mkdir()

def dependency(parent, name):
    fingerprints = TARGET/'release/.fingerprint'
    metadata = read(fingerprints/parent.stem.removeprefix('lib')/('lib-'+parent.stem.split('-')[0].removeprefix('lib')+'.json'))
    identity = next(row[3] for row in metadata['deps'] if row[1] == name)
    paths = []
    for path in (TARGET/'release/deps').glob('lib'+name+'-*.rlib'):
        receipt = fingerprints/path.stem.removeprefix('lib')/('lib-'+name)
        if receipt.is_file() and int.from_bytes(bytes.fromhex(receipt.read_text()),'little') == identity:
            paths.append(path)
    assert len(paths)==1, (name, paths)
    return paths[0]

matches = [p for p in (TARGET/'release/deps').glob('libhyperreal-*.rlib')
           if sha(p) == sha(TARGET/'release/libhyperreal.rlib')]
assert len(matches)==1
scalar = matches[0]
libraries = {'hyperreal': scalar, 'serde_json': dependency(scalar,'serde_json')}
library_hashes = {str(p): sha(p) for p in libraries.values()}
shutil.copyfile(__file__,OUT/'reproduce.py')
shutil.copyfile(EVIDENCE/'symbolic_trig_scale_probe.rs',OUT/'probe.rs')
frozen = {p:sha(p) for p in OUT.iterdir() if p.is_file()}

def verify():
    assert all(sha(SOURCE/name)==value for name,value in source_hashes.items())
    assert all(sha(ROOT/name)==value for name,value in main_hashes.items())
    assert all(sha(Path(name))==value for name,value in library_hashes.items())
    assert all(sha(p)==value for p,value in frozen.items())

def cap():
    resource.setrlimit(resource.RLIMIT_AS,(192*1024*1024,192*1024*1024))
    resource.setrlimit(resource.RLIMIT_CPU,(2,3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))

commands=[]
def run(name, command, capped=False):
    command = list(map(str,command))
    start=time.monotonic()
    timeout=False
    with (OUT/(name+'.stdout')).open('w') as stdout, (OUT/(name+'.stderr')).open('w') as stderr:
        try:
            result=subprocess.run(command,cwd=ROOT,stdout=stdout,stderr=stderr,
                                  timeout=4 if capped else 120,preexec_fn=cap if capped else None)
            code=result.returncode
        except subprocess.TimeoutExpired:
            code=None; timeout=True
    receipt={'name':name,'command':command,'exit_code':code,'wall_seconds':time.monotonic()-start,
             'wall_timeout':timeout,'timeout_seconds':4 if capped else 120,
             'address_space_cap_mib':192 if capped else None,'cpu_seconds':[2,3] if capped else None}
    commands.append(receipt)
    (OUT/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
    return receipt,(OUT/(name+'.stdout')).read_text()

verify()
assert run('format',['rustfmt','--edition','2024','--check',OUT/'probe.rs'])[0]['exit_code']==0
command=['rustc','--edition=2024','-O','-D','warnings',OUT/'probe.rs','-L','dependency='+str(TARGET/'release/deps')]
for name,path in libraries.items(): command+=['--extern',name+'='+str(path)]
assert run('build',command+['-o',WORK/'probe'])[0]['exit_code']==0
frozen[WORK/'probe']=sha(WORK/'probe')
verify()
print('Symbolic trig-scale probe built; no builds during capped replay.',flush=True)

rows=[]
inputs=[]
for shift in [0, 1, 4096, 1048576, 1048577, 2147483647]:
    inputs.append(('offset_pi',shift,2 if shift==0 else 0))
for shift in [4096,1048577,2147483647]:
    inputs.append(('inverse_offsets',shift,2))
    inputs.append(('shifted_plus_half',shift,1))
    inputs.append(('factored_plus_half',shift,1))
for shift in [4096,1048577,2147483646]:
    inputs.append(('rational_split',shift,0))
inputs.append(('half_via_extremes',0,1))
for shift in [4096,1048577]:
    inputs.append(('deep_inverse_offsets',shift,2))
for shape,shift,index in inputs:
    for family in ['sin','cos','tan','cot']:
        for mode in ['serialized','public']:
            name='case-'+str(len(rows))
            command,output=run(name,[WORK/'probe',family,mode,shape,shift,index],True)
            records=[json.loads(line) for line in output.splitlines()]
            samples=[row for row in records if row['stage']=='sample']
            rows.append({'name':name,'shape':shape,'shift':shift,'index':index,
                         'family':family,'mode':mode,'command':command,'records':records,
                         'complete':command['exit_code']==0 and len(samples)==4,
                         'valid_samples':sum(row['valid'] for row in samples),
                         'invalid_samples':sum(not row['valid'] for row in samples),
                         'numeric_samples':sum(row['numeric'] for row in samples)})
            (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
    verify()
    print(shape,shift,'completed',flush=True)
verify()
summary={'source_hashes_sha256':sha(BUILD/'source-hashes.json.gz'),'build_phase':BUILD.name,
         'libraries_sha256':library_hashes,'guards_unchanged':True,'cases':len(rows),
         'incomplete_cases':sum(not row['complete'] for row in rows),
         'invalid_samples':sum(row['invalid_samples'] for row in rows),
         'valid_samples':sum(row['valid_samples'] for row in rows),
         'numeric_samples':sum(row['numeric_samples'] for row in rows),
         'scope':'Exact half-pi lattice identities with symbolic binary offsets, cancellation, distributed rational coefficients and deep wrappers. Every returned one-unit interval must contain the exact integer oracle; poles require DivideByZero. Every process uses the unchanged 192 MiB, CPU 2/3-second, wall 4-second caps. This is scoped correctness/resource evidence, not exhaustive symbolic completeness or a benchmark.'}
summary['artifacts_sha256']={str(path.relative_to(OUT)):sha(path) for path in OUT.iterdir() if path.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:summary[k] for k in ['cases','incomplete_cases','invalid_samples','valid_samples','numeric_samples']}),flush=True)
