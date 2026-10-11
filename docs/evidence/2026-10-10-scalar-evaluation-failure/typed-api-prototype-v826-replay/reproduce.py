"""Replay the original scalar controls against a frozen fallible candidate."""
from fractions import Fraction
import gzip, hashlib, json, math, resource, shutil, subprocess, sys, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
EVIDENCE=Path(__file__).resolve().parent
BUILD=EVIDENCE/sys.argv[1]
OUT=EVIDENCE/sys.argv[2]; OUT.mkdir()
SOURCE=ROOT/'target/scalar-evaluation-candidate/source'
TARGET=ROOT/'target/scalar-evaluation-candidate/target'
WORK=ROOT/('target/'+sys.argv[2]); WORK.mkdir()
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
info=lambda p:{'bytes':p.stat().st_size,'sha256':sha(p)}
assert read(BUILD/'result.json')['exit_code']==0
source_hashes=json.loads(gzip.decompress((BUILD/'source-hashes.json.gz').read_bytes()))
main_hashes=json.loads(gzip.decompress((EVIDENCE/'candidate-preparation/main-hashes.json.gz').read_bytes()))
fingerprints=TARGET/'release/.fingerprint'

def root_library(name):
    exact=TARGET/('release/lib'+name+'.rlib')
    matches=[p for p in (TARGET/'release/deps').glob('lib'+name+'-*.rlib') if sha(p)==sha(exact)]
    assert len(matches)==1,(name,matches)
    return matches[0]

def dependency(parent,name):
    metadata=read(fingerprints/parent.stem.removeprefix('lib')/('lib-'+parent.stem.split('-')[0].removeprefix('lib')+'.json'))
    identity=next(row[3] for row in metadata['deps'] if row[1]==name)
    paths=[]
    for p in (TARGET/'release/deps').glob('lib'+name+'-*.rlib'):
        receipt=fingerprints/p.stem.removeprefix('lib')/('lib-'+name)
        if receipt.is_file() and int.from_bytes(bytes.fromhex(receipt.read_text()),'little')==identity: paths.append(p)
    assert len(paths)==1,(name,paths)
    return paths[0]

curve=root_library('hypercurve'); scalar=dependency(curve,'hyperreal')
assert sha(scalar)==sha(root_library('hyperreal'))
libraries={'hypercurve':curve,'hyperreal':scalar,'serde_json':dependency(scalar,'serde_json')}
library_info={str(p):info(p) for p in libraries.values()}
shutil.copyfile(__file__,OUT/'reproduce.py')
for name in ['candidate_probe.rs','cache_candidate_probe.rs']:shutil.copyfile(EVIDENCE/name,OUT/name)
frozen={p:info(p) for p in OUT.iterdir() if p.is_file()}
commands=[]
def verify():
    assert all(sha(SOURCE/name)==expected for name,expected in source_hashes.items())
    assert all(sha(ROOT/name)==expected for name,expected in main_hashes.items())
    assert all(info(Path(p))==expected for p,expected in library_info.items())
    assert all(info(p)==expected for p,expected in frozen.items())
def cap():
    resource.setrlimit(resource.RLIMIT_AS,(192*1024*1024,192*1024*1024))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command,capped=False):
    start=time.monotonic()
    with (OUT/(name+'.stdout')).open('w') as stdout,(OUT/(name+'.stderr')).open('w') as stderr:
        result=subprocess.run(['timeout','--kill-after=2s','3' if capped else '120',*map(str,command)],cwd=ROOT,stdout=stdout,stderr=stderr,preexec_fn=cap if capped else None)
    commands.append({'name':name,'command':list(map(str,command)),'exit_code':result.returncode,'wall_seconds':time.monotonic()-start,'timeout_seconds':3 if capped else 120,'address_space_cap_mib':192 if capped else None})
    (OUT/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
    return result.returncode,(OUT/(name+'.stdout')).read_text()
verify()
for file,binary in [('candidate_probe.rs','probe'),('cache_candidate_probe.rs','cache-probe')]:
    assert run('format-'+binary,['rustfmt','--edition','2024','--check',OUT/file])[0]==0
    command=['rustc','--edition=2024','-O','-D','warnings',OUT/file,'-L','dependency='+str(TARGET/'release/deps')]
    for name,path in libraries.items():command+=['--extern',name+'='+str(path)]
    assert run('build-'+binary,command+['-o',WORK/binary])[0]==0
    frozen[WORK/binary]=info(WORK/binary)
verify()
print('Both probes built; no builds during replay.',flush=True)
def atan(inverse):
    n=100
    lo=sum((Fraction((-1)**i,(2*i+1)*inverse**(2*i+1)) for i in range(n)),Fraction())
    return lo,lo+Fraction(1,(2*n+1)*inverse**(2*n+1))


a,b=atan(5),atan(239)
pi=(16*a[0]-4*b[1],16*a[1]-4*b[0])
e=sum((Fraction(1,math.factorial(i)) for i in range(101)),Fraction())
e=(e,e+Fraction(1,100*math.factorial(100)))
root=math.isqrt(2*10**200)
ln=2*sum((Fraction(1,(2*i+1)*3**(2*i+1)) for i in range(100)),Fraction())
sin=sum((Fraction((-1)**i,math.factorial(2*i+1)) for i in range(100)),Fraction())
cos=sum((Fraction((-1)**i,math.factorial(2*i)) for i in range(100)),Fraction())
bounds={'one':(Fraction(1),Fraction(1)),'rational':(Fraction(7,8),Fraction(7,8)),'pi':pi,'e':e,'sqrt_two':(Fraction(root,10**100),Fraction(root+1,10**100)),'ln_two':(ln,ln+Fraction(2,201*3**201)/(1-Fraction(1,9))),'sin_one':(sin,sin+Fraction(1,math.factorial(201))),'cos_one':(cos,cos+Fraction(1,math.factorial(200))),'pi_plus_e':(pi[0]+e[0],pi[1]+e[1]),'pi_times_e':(pi[0]*e[0],pi[1]*e[1])}

rows=[]
for family in bounds:
    for preparation in ['control','cold','warm','finer']:
        for operation in ['approx','explicit_signal','near_integer','curve_scope']:
            for precision in ([0] if operation=='near_integer' else [0,-64,-256]):
                name='case-'+str(len(rows));code,raw=run(name,[WORK/'probe',family,preparation,operation,precision],True)
                receipts=[json.loads(line) for line in raw.splitlines()]
                row={'name':name,'family':family,'preparation':preparation,'operation':operation,'precision':precision,'exit_code':code,'receipts':receipts}
                lo,hi=bounds[family]
                if operation=='near_integer':
                    allowed={lo.numerator//lo.denominator,-(-lo.numerator//lo.denominator),hi.numerator//hi.denominator,-(-hi.numerator//hi.denominator)}
                    valid=lambda n:n in allowed
                else:
                    scale=Fraction(2**(-precision));lo,hi=lo*scale,hi*scale
                    valid=lambda n:lo-1<=n<=hi+1
                row['first_contract_valid']=bool(len(receipts)==2 and (receipts[0]['error']=='Exhausted' and receipts[0]['result'] is None if preparation!='control' else receipts[0]['error'] is None and valid(int(receipts[0]['result']))))
                row['retry_valid']=bool(len(receipts)==2 and receipts[1]['error'] is None and receipts[1]['result'] is not None and valid(int(receipts[1]['result'])))
                rows.append(row)
                (OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
    verify();print(family,'completed',flush=True)
extremes=[]
for family in ['one','pi','pi_plus_e']:
    for precision in [-(2**31),2**31-1]:
        name='extreme-'+str(len(extremes));code,raw=run(name,[WORK/'probe',family,'cold','approx',precision],True)
        receipts=[json.loads(line) for line in raw.splitlines()]
        valid=bool(code==0 and len(receipts)==2 and receipts[0]['error']=='Exhausted' and (receipts[1]['error']=='OutOfRange' if precision<0 else receipts[1]['error'] is None and receipts[1]['result']=='0'))
        extremes.append({'name':name,'family':family,'precision':precision,'exit_code':code,'receipts':receipts,'valid':valid})
    verify()
cache=[]
def rational(text):
    pieces=text.split()
    return Fraction(pieces[0]) if len(pieces)==1 else Fraction(pieces[0])+Fraction(pieces[1])
for trial in range(3):
    for mode in ['control','cold-poison','warm-poison']:
        code,raw=run('cache-'+str(trial)+'-'+mode,[WORK/'cache-probe',mode],True)
        row={'trial':trial,'exit_code':code,'receipt':json.loads(raw) if code==0 else None}
        receipt=row['receipt']
        row['valid']=bool(receipt and not receipt['interval_excludes_pi'] and rational(receipt['interval_lower'])<=pi[0] and rational(receipt['interval_upper'])>=pi[1] and receipt['checked_scientific']=='Ok("3.141593e0")' and (receipt['extreme_result'] is None if mode=='control' else receipt['extreme_result']=='Err(OutOfRange)'))
        if receipt:
            row['valid']=row['valid'] and all(pi[0]*2**(-x['precision'])-1<=int(x['approximation'])<=pi[1]*2**(-x['precision'])+1 for x in receipt['ordinary'])
        cache.append(row)
verify()
summary={'build_record':str(BUILD/'command.json'),'build_record_sha256':sha(BUILD/'command.json'),'source_hashes_sha256':sha(BUILD/'source-hashes.json.gz'),'libraries':library_info,'guards_unchanged':True,'ordinary_cases':len(rows),'invalid_first_contracts':sum(not row['first_contract_valid'] for row in rows),'invalid_retries':sum(not row['retry_valid'] for row in rows),'typed_interruption_errors':sum(row['receipts'][0]['error']=='Exhausted' for row in rows if row['receipts']),'extremes':extremes,'cache':cache,'commands':commands,'scope':'Original scalar failure/control and curve-scope workloads. The curve scope can contain a typed scalar Err; its existing counters still do not bound scalar work. This is not admission of the prototype or a total resource guarantee.'}
summary['artifacts']={str(p.relative_to(OUT)):info(p) for p in OUT.iterdir() if p.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print({key:summary[key] for key in ['ordinary_cases','invalid_first_contracts','invalid_retries','typed_interruption_errors']},'extremes',sum(x['valid'] for x in extremes),'cache',sum(x['valid'] for x in cache),flush=True)
