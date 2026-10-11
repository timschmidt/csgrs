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
shutil.copyfile(EVIDENCE/'evaluation_budget_probe.rs',OUT/'probe.rs')
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
print('Shared evaluation-budget probe built; no builds during capped replay.',flush=True)

rows=[]
oracles={}
fixtures=[]
for shift in [4096,1048577,2147483647]:
    for residual in ['sqrt2','ln2','tiny_sqrt2','large_sqrt2']:
        for shape in ['offset_pi','offset_third','shifted_plus_half']:
            fixtures.append((shape,shift,residual))
for residual in ['exp1','atan_half','sqrt2_minus_one','negative_sqrt2','exp100']:
    fixtures.append(('offset_third',2147483647,residual))
for residual in ['sqrt2','large_sqrt2']:
    fixtures += [('factored_third',2147483647,residual),('split_third',2147483646,residual)]
for shape in ['sixth_via_extremes','deep_inverse_offsets','ordinary_third']:
    fixtures.append((shape,1048577 if shape=='deep_inverse_offsets' else 0,'sqrt2'))
assert len(fixtures)==48 and len(set(fixtures))==48

def pi_residue(shape, shift):
    # Python's integer modular exponentiation is independent of the candidate's
    # extraction and BigUint implementation. Cancelled offsets use identities
    # over the integer exponents, never materialized 2**huge-exponent integers.
    if shape in ['offset_pi','split_third']: return Fraction(pow(2,shift,2))
    if shape in ['offset_third','factored_third']: return Fraction(pow(2,shift,6),3)
    if shape=='negative_third': return -Fraction(pow(2,shift,6),3)
    if shape=='shifted_plus_half': return Fraction(pow(2,shift,2))+Fraction(1,2)
    if shape=='sixth_via_extremes': return Fraction(1,6)
    if shape in ['deep_inverse_offsets','ordinary_third']: return Fraction(1,3)
    raise AssertionError(shape)

for shape,shift,residual in fixtures:
    q=pi_residue(shape,shift)
    for family in ['sin','cos','tan','cot']:
        for mode in ['serialized','public']:
            name='case-'+str(len(rows))
            command,output=run(name,[WORK/'probe',family,mode,shape,shift,residual,q.numerator,q.denominator],True)
            records=[json.loads(line) for line in output.splitlines()]
            oracle=next((record for record in records if record['stage']=='oracle'),None)
            assert oracle, (name,command)
            assert Fraction(oracle['pi_coefficient'])==q and oracle['residual']==residual
            lower,upper=Fraction(oracle['lower']),Fraction(oracle['upper'])
            assert lower<=upper and upper-lower<Fraction(1,2**768)
            # Every selected quotient is separated from a pole. This independent
            # oracle also bounds sample size for the Rust >4096-bit rejection.
            assert -2048<lower<=upper<2048
            key=family+':'+str(q)+':'+str(residual)
            if key in oracles: assert oracles[key]==oracle
            else: oracles[key]=oracle
            samples=[record for record in records if record['stage']=='sample']
            for sample in samples:
                if sample['sample'] is not None:
                    center=int(sample['sample'])*Fraction(2)**sample['precision']
                    unit=Fraction(2)**sample['precision']
                    assert sample['enclosed']==(center-unit<=lower<=upper<=center+unit)
            empty = [record for record in records if record['stage']=='empty_budget']
            warm = [record for record in records if record['stage']=='warm_budget']
            assert len(empty)==len(warm)==1 and len(samples)==4, (name,command)
            empty,warm=empty[0],warm[0]
            assert empty['consumed']==warm['empty_consumed']==0
            assert empty['error'] in [None,'Exhausted']
            assert warm['consumed']+warm['remaining']==1_000_000
            assert warm['enclosed']
            center=int(warm['sample'])*Fraction(1,256)
            assert center-Fraction(1,256)<=lower<=upper<=center+Fraction(1,256)
            if warm['before_sign'] is not None: assert warm['after_sign']==warm['before_sign']
            if warm['after_sign'] is not None:
                assert warm['after_sign']==('Positive' if lower>0 else 'Negative')
            rows.append({'empty_budget':empty,'warm_budget':warm,'name':name,'family':family,'mode':mode,'shape':shape,'shift':shift,
                'residual':str(residual),'pi_coefficient':str(q),'oracle_key':key,'command':command,
                'samples':samples,'complete':command['exit_code']==0 and len(samples)==4,
                'invalid_numeric_samples':sum(sample['numeric'] and not sample['enclosed'] for sample in samples),
                'typed_errors':sum(sample['error'] is not None for sample in samples),
                'last_stage':records[-1]['stage']})
            (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
            (OUT/'oracles.json').write_text(json.dumps(oracles,indent=2)+'\n')
    print(shape,shift,str(residual),'completed',flush=True)
verify()
summary={'source_hashes_sha256':sha(BUILD/'source-hashes.json.gz'),'build_phase':BUILD.name,
    'libraries_sha256':library_hashes,'guards_unchanged':True,'cases':len(rows),
    'observations':sum(len(row['samples']) for row in rows),
    'numeric_samples':sum(sample['numeric'] for row in rows for sample in row['samples']),
    'invalid_numeric_samples':sum(row['invalid_numeric_samples'] for row in rows),
    'incomplete_cases':sum(not row['complete'] for row in rows),
    'typed_errors':sum(row['typed_errors'] for row in rows),
    'by_family_mode':{family+':'+mode:{'cases':sum(row['family']==family and row['mode']==mode for row in rows),
        'invalid_numeric_samples':sum(row['invalid_numeric_samples'] for row in rows if row['family']==family and row['mode']==mode),
        'incomplete_cases':sum(not row['complete'] for row in rows if row['family']==family and row['mode']==mode),
        'typed_errors':sum(row['typed_errors'] for row in rows if row['family']==family and row['mode']==mode)}
        for family in ['sin','cos','tan','cot'] for mode in ['serialized','public']},
    'warm_zero_budget_checks':sum(row['warm_budget']['enclosed'] for row in rows),
    'cold_zero_budget_exhaustions':sum(row['empty_budget']['error']=='Exhausted' for row in rows),
    'scope':'New budgeted APIs on 48 symbolic angle fixtures with nine finite non-rational residuals, four trig families and serialized/public controls. Includes sqrt(2), ln(2), exp(1), atan(1/2), sqrt(2)-1, signed/scaled sqrt(2), and exp(100); lattice/non-lattice periodic coefficients, nearby poles, factored/distributed arithmetic, cancelled extreme offsets and 64 offset wrappers. Four-query schedule shares one allowance of one million work units. A prior zero-unit request is typed or reuses an already available constructor cache; a later zero-unit request must reuse the completed cache and retain every established sign. All returned samples are independently enclosed. Directed 1024-bit MPFR pi and residual enclosures feed exact derivative-one sin/cos intervals and exact quotient bounds. Each process is capped at 192 MiB, CPU 2/3 seconds and wall 4 seconds. Scoped evidence, not an exhaustive proof or matched benchmark.'}
summary['artifacts_sha256']={str(path.relative_to(OUT)):sha(path) for path in OUT.iterdir() if path.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({key:summary[key] for key in ['cases','observations','numeric_samples','invalid_numeric_samples','incomplete_cases','typed_errors']}),flush=True)
