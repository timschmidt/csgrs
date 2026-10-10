"""Retain public interrupted-evaluation receipts before changing scalar work semantics."""
from fractions import Fraction
import gzip
import hashlib
import json
import math
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PHASE = sys.argv[1] if len(sys.argv) > 1 else 'baseline-v3'
OUT = EVIDENCE / PHASE
OUT.mkdir()
WORK = ROOT / ('target/scalar-evaluation-failure-' + PHASE)
WORK.mkdir()
record_path = EVIDENCE.parent / '2026-10-10-artifact-numeric-keys/candidate-v2/sources.json'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': sha(p)}
record = read(record_path)
source, target = Path(record['source_directory']), Path(record['target_directory'])
guards = []
for kind, base in [('source', source), ('main', ROOT)]:
    p = Path(record[kind + '_hashes_file'])
    assert sha(p) == record[kind + '_hashes_file_sha256']
    guards.append((base, json.loads(gzip.decompress(p.read_bytes()))))
library = target / 'release/libhyperdrc.rlib'
matched = [p for p in (target / 'release/deps').glob('libhyperdrc-*.rlib') if sha(p) == sha(library)]
assert len(matched) == 1
fingerprints = target / 'release/.fingerprint'
metadata = read(fingerprints / matched[0].stem.removeprefix('lib') / 'lib-hyperdrc.json')
libraries = {}
for name in ['hyperreal', 'hypercurve', 'serde_json']:
    identifier = next(row[3] for row in metadata['deps'] if row[1] == name)
    paths = [p for p in (target / 'release/deps').glob('lib' + name + '-*.rlib') if int.from_bytes(bytes.fromhex((fingerprints / p.stem.removeprefix('lib') / ('lib-' + name)).read_text()), 'little') == identifier]
    assert len(paths) == 1
    libraries[name] = paths[0]
library_info = {str(p): info(p) for p in libraries.values()}
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'probe.rs', OUT / 'probe.rs')
for name in ['crates/hyperreal/src/computable/node/approximation_queries.rs', 'crates/hyperreal/src/computable/node/representation.rs', 'crates/hyperreal/src/computable/node/bounds.rs', 'crates/hyperreal/src/computable/approximation/dispatch.rs', 'crates/hypersolve/src/work_budget.rs', 'crates/hypercurve/src/policy.rs']:
    destination = OUT / 'inputs' / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source / name, destination)
frozen = {p: info(p) for p in OUT.rglob('*') if p.is_file()}
commands = []


def verify():
    assert all(sha(base / name) == expected for base, values in guards for name, expected in values.items())
    assert all(info(Path(p)) == expected for p, expected in library_info.items())
    assert all(info(p) == expected for p, expected in frozen.items())


def cap():
    resource.setrlimit(resource.RLIMIT_AS, (192 * 1024 * 1024, 192 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, command, capped=False):
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=2s', '3' if capped else '120', *map(str, command)], cwd=ROOT, stdout=stdout, stderr=stderr, preexec_fn=cap if capped else None, check=False)
    commands.append({'name':name,'command':list(map(str,command)),'exit_code':result.returncode,'wall_seconds':time.monotonic()-start,'timeout_seconds':3 if capped else 120,'address_space_cap_mib':192 if capped else None})
    (OUT / 'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
    return result.returncode, (OUT / (name + '.stdout')).read_text()


verify()
assert run('format', ['rustfmt', '--edition', '2024', '--check', OUT / 'probe.rs'])[0] == 0
command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', OUT / 'probe.rs', '-L', 'dependency=' + str(target / 'release/deps')]
for name, path in libraries.items(): command += ['--extern', name + '=' + str(path)]
assert run('build', command + ['-o', WORK / 'probe'])[0] == 0
frozen[WORK / 'probe'] = info(WORK / 'probe')
verify()
print('Baseline probe built; no builds during replay.',flush=True)


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
                name='case-'+str(len(rows))
                code,raw=run(name,[WORK/'probe',family,preparation,operation,precision],capped=True)
                receipts=[json.loads(line) for line in raw.splitlines()]
                row={'name':name,'family':family,'preparation':preparation,'operation':operation,'precision':precision,'exit_code':code,'receipts':receipts}
                lo,hi=bounds[family]
                if operation=='near_integer':
                    allowed=sorted({lo.numerator//lo.denominator,-(-lo.numerator//lo.denominator),hi.numerator//hi.denominator,-(-hi.numerator//hi.denominator)})
                    valid=lambda n:n in allowed
                else:
                    scale=Fraction(2**(-precision))
                    lo,hi=lo*scale,hi*scale
                    valid=lambda n:lo-1<=n<=hi+1
                row['returned_valid_approximation']=bool(receipts and receipts[0]['result'] is not None and valid(int(receipts[0]['result'])))
                row['retry_valid']=bool(len(receipts)==2 and receipts[1]['result'] is not None and valid(int(receipts[1]['result'])))
                if preparation=='control':assert row['returned_valid_approximation'] and row['retry_valid'],row
                rows.append(row)
    verify()
    print(family,'completed',flush=True)
for family in ['one','pi','pi_plus_e']:
    for precision in [-(2**31),2**31-1]:
        name='extreme-'+str(len(rows))
        code,raw=run(name,[WORK/'probe',family,'cold','approx',precision],capped=True)
        rows.append({'name':name,'family':family,'preparation':'cold','operation':'approx','precision':precision,'exit_code':code,'receipts':[json.loads(line) for line in raw.splitlines()]})
verify()
(OUT/'receipts.json').write_text(json.dumps(rows,indent=2)+'\n')
summary={'source_record':str(record_path),'source_record_sha256':sha(record_path),'libraries':library_info,'binary':info(WORK/'probe'),'guards_unchanged':True,'commands':commands,'ordinary_cases':sum('retry_valid' in row for row in rows),'invalid_returned_approximations':sum(not row['returned_valid_approximation'] for row in rows if 'returned_valid_approximation' in row),'invalid_retries':sum(not row['retry_valid'] for row in rows if 'retry_valid' in row),'nonzero_exit_cases':[row['name'] for row in rows if row['exit_code']!=0],'scope':'Interrupted scalar evaluation and its propagation through existing curve budget scopes. Existing budgets explicitly exclude scalar work, so this records a required boundary extension rather than claiming their documented attempt counters are incorrect.'}
summary['artifacts']={str(p.relative_to(OUT)):info(p) for p in sorted(OUT.rglob('*')) if p.is_file()}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('Scalar failure baseline complete.',flush=True)
