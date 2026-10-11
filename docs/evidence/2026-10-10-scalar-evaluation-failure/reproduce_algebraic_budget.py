"""Exercise shared evaluation budgets on the preserved non-rational trig corpus."""
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path
import resource
import re
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
UNIT = EVIDENCE / sys.argv[3]
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
assert read(UNIT/'result.json')['exit_code'] == 0
assert sha(UNIT/'source-hashes.json.gz') == sha(BUILD/'source-hashes.json.gz')
unit_binaries = re.findall(r"Running unittests src/lib\.rs \(([^)\n]*/hyperreal-[^)]+)\)", (UNIT/'stderr.log').read_text())
assert len(unit_binaries) == 1
unit_fingerprint = TARGET/'release/.fingerprint'/Path(unit_binaries[0]).name/'test-lib-hyperreal.json'
rug_identity = next(row[3] for row in read(unit_fingerprint)['deps'] if row[1] == 'rug')
rug_matches = []
for path in (TARGET/'release/deps').glob('librug-*.rlib'):
    receipt = TARGET/'release/.fingerprint'/path.stem.removeprefix('lib')/'lib-rug'
    if receipt.is_file() and int.from_bytes(bytes.fromhex(receipt.read_text()),'little') == rug_identity:
        rug_matches.append(path)
assert len(rug_matches) == 1, rug_matches
libraries = {'hyperreal': scalar, 'serde_json': dependency(scalar,'serde_json'), 'rug': rug_matches[0]}
(OUT/'library-selection.json').write_text(json.dumps({'unit_phase':UNIT.name, 'unit_fingerprint':str(unit_fingerprint), 'unit_fingerprint_sha256':sha(unit_fingerprint), 'rug_dependency_identity':rug_identity},indent=2)+'\n')
library_hashes = {str(p): sha(p) for p in libraries.values()}
shutil.copyfile(__file__,OUT/'reproduce.py')
shutil.copyfile(EVIDENCE/'algebraic_budget_probe.rs',OUT/'probe.rs')
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
print('Algebraic shared-budget probe built; no builds during capped replay.',flush=True)

rows=[]
mode=sys.argv[4]
assert mode in ['replay','cost']
if mode=='replay':
    fixtures=[(degree,count,0,inverse) for degree in [2,3,4] for count in [2,3,4] for inverse in [False,True]]
    fixtures += [(2,2,bits,inverse) for bits in [64,256,1024] for inverse in [False,True]]
    assert len(fixtures)==24
    for degree,count,bits,inverse in fixtures:
        for perturbation in [-1,0,1]:
            for units in [0,8,256,1_000_000]:
                name='case-'+str(len(rows))
                command,output=run(name,[WORK/'probe','case',degree,count,bits,str(inverse).lower(),perturbation,units],True)
                records=[json.loads(line) for line in output.splitlines()]
                complete=command['exit_code']==0 and len(records)==1 and records[0]['stage']=='case'
                row={'name':name,'command':command,'complete':complete,'records':records}
                rows.append(row)
                (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
                assert complete,(name,command,output)
                receipt=records[0]
                expected={-1:'Negative',0:'Zero',1:'Positive'}[perturbation]
                assert receipt['expected']==expected and receipt['answer'] in [None,expected]
                assert receipt['consumed']+receipt['remaining']==units
                assert receipt['zero_budget_proof_retained']
                exact=Fraction(perturbation,2**96)
                for when in ['before','after']:
                    unit=Fraction(2)**receipt[when+'_precision']
                    center=int(receipt[when+'_sample'])*unit
                    assert center-unit<=exact<=center+unit
        print('completed',degree,count,bits,inverse,flush=True)
    summary={'cases':len(rows),'numeric_observations':2*len(rows),
        'unknown_answers':sum(row['records'][0]['answer'] is None for row in rows),
        'certified_answers':sum(row['records'][0]['answer'] is not None for row in rows),
        'zero_budget_retained_proofs':len(rows),'incomplete_cases':0,'invalid_samples':0,
        'scope':'24 exact principal-root product identities, including reciprocal forms and 64/256/1024-bit rational factors, three exact perturbations and four allowances. Numerical warmup precedes the explicit shared allowance. Independent rational oracles check both approximations; every definite sign must agree, and an unlimited retry must certify and retain the result at zero budget. This is a scoped accounting/certificate replay, not a wall-time or memory proof.'}
else:
    import statistics
    cases=[('square_roots',2,2,0,False),('cube_roots',3,3,0,False),('fourth_roots',4,4,0,False),
           ('reciprocals',2,3,0,True),('wide_256',2,2,256,False),('wide_1024_inverse',2,2,1024,True)]
    metrics={}
    for case,degree,count,bits,inverse in cases:
        samples=[]
        for repetition in range(9):
            name=case+'-'+str(repetition)
            command,output=run(name,[WORK/'probe','cost',degree,count,bits,str(inverse).lower(),64],True)
            assert command['exit_code']==0,(name,command,output)
            receipt=json.loads(output)
            assert receipt['stage']=='cost' and receipt['iterations']==receipt['certified']==64
            samples.append(int(receipt['elapsed_ns']))
            rows.append({'name':name,'command':command,'receipt':receipt})
            (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
        metrics[case]={'iterations':64,'certified':64,'elapsed_ns':samples,'median_ns':statistics.median(samples)}
        print(case,'completed',flush=True)
    summary={'processes':len(rows),'cases':metrics,'scope':'Six default sign-query workloads, nine separate-process repetitions each, 64 independently reconstructed roots per process. Construction and 16384-bit numerical warmup occur outside the timed loop. Exact zero certificates are the checksum. Identical order/caps; no concurrent build, test or measurement workload. Scoped host timing observations, not full-board, allocation or binary-size admission evidence.'}
verify()
summary.update({'source_hashes_sha256':sha(BUILD/'source-hashes.json.gz'),'build_phase':BUILD.name,
                'libraries_sha256':library_hashes,'guards_unchanged':True,'mode':mode})
summary['artifacts_sha256']={str(path.relative_to(OUT)):sha(path) for path in OUT.iterdir() if path.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ['artifacts_sha256','libraries_sha256','scope','cases']}),flush=True)
