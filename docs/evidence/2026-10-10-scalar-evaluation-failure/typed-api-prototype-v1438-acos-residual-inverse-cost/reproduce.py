"""Replay inverse-trig budget queries with exact rational identity oracles."""
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

# Admit the full fixed rational inputs and dyadic oracle without unbounded decimal parsing.
sys.set_int_max_str_digits(20000)

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
shutil.copyfile(EVIDENCE/'inverse_trig_budget_probe.rs',OUT/'probe.rs')
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
print('Inverse-trig budget probe built; no builds during capped workloads.',flush=True)

rows=[]
mode=sys.argv[4]
assert mode in ['replay','cost']
if mode=='replay':
    for family in ['atan','acos','terms','pi']:
        for bits in [0,16,128,1024]:
            for warm in [False,True]:
                for perturbation in [-1,0,1]:
                    for units in [0,8,256,1_000_000]:
                        name='case-'+str(len(rows))
                        command,output=run(name,[WORK/'probe','case',family,bits,perturbation,units,str(warm).lower()],True)
                        records=[json.loads(line) for line in output.splitlines()]
                        complete=command['exit_code']==0 and len(records)==1 and records[0]['stage']=='case'
                        row={'name':name,'command':command,'complete':complete,'records':records}
                        rows.append(row)
                        (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
                        assert complete,(name,command,output)
                        r=records[0]
                        assert r['family']==family and r['coefficient_bits']==bits and r['warm']==warm
                        expected={-1:'Negative',0:'Zero',1:'Positive'}[perturbation]
                        assert r['expected']==expected and r['answer'] in [None,expected]
                        assert r['consumed']+r['remaining']==units and r['zero_budget_proof_retained']
                        exact=Fraction(perturbation,2**96)
                        for when in ['before','precise','after']:
                            if r[when+'_sample'] is None:
                                assert when=='before' and not warm
                                continue
                            unit=Fraction(2)**r[when+'_precision']
                            center=int(r[when+'_sample'])*unit
                            assert center-unit<=exact<=center+unit
            print('completed',family,bits,flush=True)
    assert len(rows)==384
    summary={'cases':len(rows),'numeric_observations':sum(2+int(r['records'][0]['warm']) for r in rows),
        'unknown_answers':sum(r['records'][0]['answer'] is None for r in rows),
        'certified_answers':sum(r['records'][0]['answer'] is not None for r in rows),
        'zero_budget_retained_proofs':len(rows),'incomplete_cases':0,'invalid_samples':0,
        'scope':'Four exact atan/asin-acos/commuted-atan/pi identities at four coefficient widths, cold and numerically warmed requests, three exact perturbations and four allowances. Each definitive sign agrees with the exact identity; unlimited retry certifies it and a later zero-budget query retains the proof. Python Fraction and Rug independently check each numerical observation against the exact rational perturbation. A finite public/serialized corpus, not an allocation/time proof.'}
else:
    import statistics
    cases=[('atan_small','atan',0),('atan_wide','atan',1024),('acos_wide','acos',1024),
           ('terms_small','terms',0),('terms_wide','terms',1024),('pi_wide','pi',1024)]
    metrics={}
    for name,family,bits in cases:
        samples=[]
        for repeat in range(9):
            case=name+'-'+str(repeat)
            command,output=run(case,[WORK/'probe','cost',family,bits,64],True)
            assert command['exit_code']==0,(case,command,output)
            r=json.loads(output)
            assert r['stage']=='cost' and r['iterations']==r['certified']==64
            samples.append(int(r['elapsed_ns']))
            rows.append({'name':case,'command':command,'receipt':r})
            (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
        metrics[name]={'iterations':64,'certified':64,'elapsed_ns':samples,'median_ns':statistics.median(samples)}
        print(name,'completed',flush=True)
    summary={'processes':len(rows),'cases':metrics,'scope':'Six default sign-query workloads; nine separate-process repetitions with 64 independently reconstructed roots per process. Construction and 512-bit numeric warmup precede the timed loop; every sign must certify exact zero. Same inputs/order/caps; no concurrent builds, tests or measurements. Local timing observations, not allocation, binary-size or whole-application admission evidence.'}
verify()
summary.update({'source_hashes_sha256':sha(BUILD/'source-hashes.json.gz'),'build_phase':BUILD.name,
                'libraries_sha256':library_hashes,'guards_unchanged':True,'mode':mode})
summary['artifacts_sha256']={str(p.relative_to(OUT)):sha(p) for p in OUT.iterdir() if p.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ['artifacts_sha256','libraries_sha256','scope','cases']}),flush=True)
