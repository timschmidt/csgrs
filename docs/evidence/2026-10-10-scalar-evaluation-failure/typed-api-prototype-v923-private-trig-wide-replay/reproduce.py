"""Capped large serialized trig inputs with independent directed MPFR enclosures."""
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
shutil.copyfile(EVIDENCE/'private_trig_wide_probe.rs',OUT/'probe.rs')
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
print('Wide private-trig probe built; no builds during capped replay.',flush=True)

rows=[]
oracles={}
families=['SincSmall','CoscSmall','PrescaledSin','PrescaledCos','PrescaledSinRational','PrescaledCosRational']
inputs=[(sign*n,d) for n,d in [(63,2),(65,2),(64,1),(257,2),(512,1),(1024,1),(65536,1),(1048576,1)] for sign in [-1,1]]
for family in families:
    for numerator,denominator in inputs:
        for mode in ['serialized','public']:
            for precision in [-4,-64]:
                name='case-'+str(len(rows))
                command,output=run(name,[WORK/'probe',family,mode,numerator,denominator,precision],True)
                records=[json.loads(line) for line in output.splitlines()]
                oracle=next((row for row in records if row['stage']=='oracle'),None)
                assert oracle, (name, command)
                lower,upper=Fraction(oracle['lower']),Fraction(oracle['upper'])
                assert -1<=lower<=upper<=1 and upper-lower<Fraction(1,2**768)
                key=family+':'+str(Fraction(numerator,denominator))
                if key in oracles: assert oracles[key]==oracle
                else: oracles[key]=oracle
                samples=[row for row in records if row['stage']=='sample']
                for sample in samples:
                    if sample['sample'] is not None:
                        center=int(sample['sample'])*Fraction(2)**sample['precision']
                        unit=Fraction(2)**sample['precision']
                        assert sample['enclosed']==(center-unit<=lower<=upper<=center+unit)
                rows.append({'name':name,'family':family,'mode':mode,'input':str(Fraction(numerator,denominator)),
                    'precision':precision,'oracle_key':key,'command':command,'samples':samples,
                    'complete':command['exit_code']==0 and len(samples)==4,
                    'invalid_numeric_samples':sum(row['numeric'] and not row['enclosed'] for row in samples),
                    'typed_errors':sum(row['error'] is not None for row in samples)})
                (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
                (OUT/'oracles.json').write_text(json.dumps(oracles,indent=2)+'\n')
        print(family,numerator,denominator,'completed',flush=True)
    verify()
verify()
summary={'source_hashes_sha256':sha(BUILD/'source-hashes.json.gz'),'build_phase':BUILD.name,
    'libraries_sha256':library_hashes,'guards_unchanged':True,'cases':len(rows),
    'numeric_samples':sum(len(row['samples']) for row in rows),
    'invalid_numeric_samples':sum(row['invalid_numeric_samples'] for row in rows),
    'incomplete_cases':sum(not row['complete'] for row in rows),
    'typed_errors':sum(row['typed_errors'] for row in rows),
    'by_family_mode':{family+':'+mode:{'cases':sum(row['family']==family and row['mode']==mode for row in rows),
        'invalid_numeric_samples':sum(row['invalid_numeric_samples'] for row in rows if row['family']==family and row['mode']==mode),
        'incomplete_cases':sum(not row['complete'] for row in rows if row['family']==family and row['mode']==mode),
        'typed_errors':sum(row['typed_errors'] for row in rows if row['family']==family and row['mode']==mode)}
        for family in families for mode in ['serialized','public']},
    'scope':'Six serialized trig node families and equivalent public controls, sixteen signed exact dyadic inputs from 31.5 through 1048576, two initial precisions and four-query schedules. Directed 1024-bit MPFR sin/cos enclosures are converted to exact rationals; sinc/cosc bounds use exact rational division/subtraction. Each process is capped at 192 MiB, CPU 2/3 seconds and wall 4 seconds. This is scoped correctness/resource evidence, not an exhaustive range proof or a matched benchmark.'}
summary['artifacts_sha256']={str(path.relative_to(OUT)):sha(path) for path in OUT.iterdir() if path.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({key:summary[key] for key in ['cases','numeric_samples','invalid_numeric_samples','incomplete_cases','typed_errors','by_family_mode']}),flush=True)
