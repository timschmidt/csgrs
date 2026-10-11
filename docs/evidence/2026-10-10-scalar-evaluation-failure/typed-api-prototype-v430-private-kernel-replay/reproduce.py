"""Capped private-kernel convergence and exact-pole witnesses; numerical accuracy is assessed separately."""
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
shutil.copyfile(EVIDENCE/'private_kernel_probe.rs',OUT/'probe.rs')
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
print('Private-kernel probe built; no builds during capped replay.',flush=True)

rows=[]
cases=[]
for family in ['PrescaledAtan','PrescaledAsinh']:
    for n,d in [(0,1),(-1,2),(1,2),(-1,1),(1,1),(-2,1),(2,1),(-8,1),(8,1)]:
        cases.append((family,'rational',n,d,0,False))
for family, denominator in [('PrescaledTan',2),('TanDeferred',2),('PrescaledCot',1)]:
    for n in [-1,1]:
        for delta in [-1,0,1]:
            cases.append((family,'pi',n,denominator,delta,delta==0))
for delta in [-1,0,1]:
    cases.append(('PrescaledCot','rational',0,1,delta,delta==0))
for family,base,n,d,delta,pole in cases:
    for mode in ['serialized','public']:
        name='case-'+str(len(rows))
        command,output=run(name,[WORK/'probe',family,mode,base,n,d,delta,-32],True)
        receipts=[json.loads(line) for line in output.splitlines()]
        result=next((r for r in receipts if r['stage']=='result'),None)
        valid_shape=bool(command['exit_code']==0 and result and
                         ((result['error'] in ['DivideByZero','NotANumber'] and result['sample'] is None)
                          if pole else result['error'] is None and result['sample'] is not None))
        rows.append({'name':name,'family':family,'mode':mode,'base':base,'numerator':n,'denominator':d,
                     'perturbation':delta,'precision':-32,'exact_pole':pole,'command':command,
                     'receipts':receipts,'expected_result_shape':valid_shape,
                     'numerical_accuracy_assessed':False})
        (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
    print(family,base,n,d,delta,'completed',flush=True)
verify()
summary={'source_hashes_sha256':sha(BUILD/'source-hashes.json.gz'),'build_phase':BUILD.name,
         'libraries_sha256':library_hashes,'guards_unchanged':True,'cases':len(rows),
         'incomplete_cases':sum(row['command']['exit_code']!=0 for row in rows),
         'unexpected_result_shapes':sum(not row['expected_result_shape'] for row in rows),
         'by_family_mode':{family+':'+mode:{'cases':sum(row['family']==family and row['mode']==mode for row in rows),
             'incomplete_cases':sum(row['command']['exit_code']!=0 for row in rows if row['family']==family and row['mode']==mode),
             'unexpected_result_shapes':sum(not row['expected_result_shape'] for row in rows if row['family']==family and row['mode']==mode)}
             for family in ['PrescaledAtan','PrescaledAsinh','PrescaledTan','TanDeferred','PrescaledCot'] for mode in ['serialized','public']},
         'scope':'Termination and typed-domain result-shape witnesses only. The zero trigonometric denominators follow exact pi-multiple identities; perturbed and inverse-function inputs are mathematically in domain. Successful finite samples are not credited with verified numerical accuracy. Independent enclosures and constructor-valid controls remain required for repair validation.'}
summary['artifacts_sha256']={str(p.relative_to(OUT)):sha(p) for p in OUT.iterdir() if p.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:summary[k] for k in ['cases','incomplete_cases','unexpected_result_shapes','by_family_mode']}),flush=True)
