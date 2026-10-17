"""Exercise native board composition against completed candidate v1 read-only."""
import gzip, hashlib, json, os, re, resource, subprocess, time
from pathlib import Path
W = Path(__file__).resolve().parent
O = W / 'aggregation-reproduction-v1'
S = W / 'candidate-source-v1'
TEST = W / 'probe-source-v2/kicad_native_aggregation.rs'
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def read(p): return json.loads(Path(p).read_text())
def dump(p, v): Path(p).write_text(json.dumps(v, indent=2) + '\n')
assert read(W / 'candidate-validation-v1/result.json')['unexpected_exits'] == ['source-failures']
with gzip.open(W / 'candidate-preparation-v1/source-hashes.json.gz', 'rt') as f: inputs = json.load(f)
assert all(sha(S / p) == h for p, h in inputs.items())
build = W / 'candidate-validation-v1/native-build.stdout'
row = next(r for r in read(W / 'candidate-validation-v1/commands.json') if r['name'] == 'native-build')
assert sha(build) == row['stdout_sha256'] and row['exit_code'] == 0
artifacts = [json.loads(l) for l in build.read_text().splitlines() if l.startswith('{')]
libraries = {}
for name in ['hyperdrc', 'hypercurve', 'hyperlimit', 'hyperreal', 'anyhow']:
    matches = [Path(p) for a in artifacts if a.get('reason') == 'compiler-artifact' and a['target']['name'] == name
        for p in a['filenames'] if p.endswith('.rlib')]
    assert len(matches) == 1
    libraries[name] = matches[0]
deps = libraries['hyperdrc'].parent
readonly = {str(p): sha(p) for p in deps.iterdir() if p.suffix in ['.rlib', '.so']}
pins = {str(p): sha(p) for p in [Path(__file__), TEST, W / 'contract-v1.md', W / 'aggregation-contract-v1.md', build,
    W / 'candidate-preparation-v1/source-hashes.json.gz', W / 'candidate-validation-v1/result.json']}
O.mkdir(exist_ok=False)
dump(O / 'protocol.json', {'input_sha256': pins, 'source_inputs': inputs, 'libraries': {n: str(p) for n,p in libraries.items()},
    'read_only_library_sha256': readonly, 'target_reopened': False, 'timeout_seconds': 1800, 'address_space_bytes': 8*1024**3})
env = os.environ | {'TMPDIR': str(O)}
commands = []
def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8*1024**3, 8*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
def run(name, command):
    begin = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        r = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')),
            'timeout', '--kill-after=20s', '1800s', *command], cwd=O, env=env, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    row = {'name': name, 'command': command, 'exit_code': r.returncode, 'elapsed_seconds': time.monotonic()-begin,
        **{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
    commands.append(row); dump(O/'commands.json',commands)
    print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
    return r.returncode
binary = O / 'kicad_native_aggregation'
flags = ['rustc', '--edition=2024', '--test', '-D', 'warnings', '-C', 'opt-level=3', '-L', 'dependency='+str(deps)]
for n, p in libraries.items(): flags += ['--extern', n+'='+str(p)]
rc = run('compile', [*flags, str(TEST), '-o', str(binary)])
counts = []
if not rc:
    rc = run('aggregation', [str(binary), '--nocapture', '--test-threads=1'])
    counts = re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;', (O/'aggregation.stdout').read_text())
assert all(sha(p)==h for p,h in pins.items())
assert all(sha(p)==h for p,h in readonly.items())
assert all(sha(S/p)==h for p,h in inputs.items())
dump(O/'result.json', {'complete':True, 'counts': counts, 'exit_code':rc, 'read_only_inputs_unchanged':True,
    'source_modified':False, 'scope':'No assertion after a failure receives credit.'})
print(json.dumps({'complete':True,'counts':counts,'exit_code':rc}),flush=True)
