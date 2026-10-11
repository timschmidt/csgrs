"""Capped serialized private-trig probes with independent exact-rational enclosures."""
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
shutil.copyfile(EVIDENCE/'private_trig_probe.rs',OUT/'probe.rs')
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
print('Private-trig probe built; no builds during capped replay.',flush=True)

# Exact rational alternating-series bounds. Once the omitted term decreases
# and its denominator factors exceed |x|, every subsequent term decreases.
# The finite prefix plus/minus that first omitted term encloses the exact sum.
def enclosure(family,x):
    first={'PrescaledSin':1,'PrescaledCos':0,'SincSmall':1,'CoscSmall':2}[family]
    term=Fraction(1,2) if first==2 else (x if family=='PrescaledSin' else Fraction(1))
    total=term; n=first
    while True:
        next_term=-term*x*x/((n+1)*(n+2))
        if (n+1)*(n+2)>x*x and abs(next_term)<Fraction(1,2**768):
            return min(total,total+next_term),max(total,total+next_term)
        total+=next_term; term=next_term; n+=2

rows=[]
oracles={}
for family in ['SincSmall','CoscSmall','PrescaledSin','PrescaledCos']:
    for x in [Fraction(0)]+[sign*value for value in [Fraction(1,2),Fraction(3,4),Fraction(1),Fraction(3,2),Fraction(7,4),Fraction(2047,1024),Fraction(2),Fraction(2049,1024),Fraction(4),Fraction(8),Fraction(16),Fraction(32)] for sign in [-1,1]]:
        lo,hi=enclosure(family,x)
        assert hi-lo < Fraction(1,2**768)
        key=family+':'+str(x)
        oracles[key]={'lower':str(lo),'upper':str(hi)}
        (OUT/'oracles.json').write_text(json.dumps(oracles,indent=2)+'\n')
        for mode in ['serialized','public']:
            for precision in [-4,-64,-256]:
                name='case-'+str(len(rows))
                receipt,output=run(name,[WORK/'probe',family,mode,x.numerator,x.denominator,precision],True)
                samples=[json.loads(line) for line in output.splitlines()]
                for sample in samples:
                    n=sample['sample']; p=sample['precision']
                    scale=Fraction(2)**(-p)
                    sample['enclosure_contains_sample_contract'] = bool(n is not None and int(n)-1<=lo*scale<=hi*scale<=int(n)+1)
                rows.append({'name':name,'family':family,'mode':mode,'input':str(x),'precision':precision,
                             'oracle_key':key,'command':receipt,'samples':samples,
                             'complete':receipt['exit_code']==0 and len(samples)==4,
                             'invalid_numeric_samples':sum(s['sample'] is not None and not s['enclosure_contains_sample_contract'] for s in samples),
                             'typed_errors':sum(s['error'] is not None for s in samples)})
                (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
    verify();print(family+' completed',flush=True)
verify()
summary={'source_hashes_sha256':sha(BUILD/'source-hashes.json.gz'),'build_phase':BUILD.name,
         'libraries_sha256':library_hashes,'guards_unchanged':True,'cases':len(rows),
         'numeric_samples':sum(len(r['samples']) for r in rows),
         'invalid_numeric_samples':sum(r['invalid_numeric_samples'] for r in rows),
         'incomplete_cases':sum(not r['complete'] for r in rows),'typed_errors':sum(r['typed_errors'] for r in rows),
         'by_mode':{mode:{'cases':sum(r['mode']==mode for r in rows),
                         'invalid_numeric_samples':sum(r['invalid_numeric_samples'] for r in rows if r['mode']==mode),
                         'incomplete_cases':sum(not r['complete'] for r in rows if r['mode']==mode),
                         'typed_errors':sum(r['typed_errors'] for r in rows if r['mode']==mode)} for mode in ['serialized','public']},
         'scope':'Four serialized private trig node families, exact-rational input matrix, ordinary constructor controls and repeated precision orders. Passing samples are checked against independent exact-rational alternating-series enclosures. This does not establish all private kernel ranges, exact poles, concurrency or aggregate work bounds.'}
summary['artifacts_sha256']={str(p.relative_to(OUT)):sha(p) for p in OUT.iterdir() if p.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:summary[k] for k in ['cases','numeric_samples','invalid_numeric_samples','incomplete_cases','typed_errors','by_mode']}),flush=True)
