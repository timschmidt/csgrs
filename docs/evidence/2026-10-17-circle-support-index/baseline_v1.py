"""Reproduce cache semantics and preliminary scaling before indexing."""
import gzip
import hashlib
import json
import os
import re
import resource
import shutil
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
S = W / 'baseline-source-v1'
O = W / 'baseline-validation-v1'
T = Path('/tmp/csgrs-circle-support-baseline-v1')
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def read(p): return json.loads(Path(p).read_text())
def dump(p, v): Path(p).write_text(json.dumps(v, indent=2) + '\n')
prep = read(W / 'baseline-preparation-v1/result.json'); assert prep['complete']
manifest = W / 'baseline-preparation-v1/source-hashes.json.gz'
assert sha(manifest) == prep['source_hashes_sha256']
with gzip.open(manifest, 'rt') as f: inputs = json.load(f)
assert all(sha(S / p) == h for p, h in inputs.items())
O.mkdir(exist_ok=False); T.mkdir(exist_ok=False)
scratch = O / 'scratch'; scratch.mkdir()
env = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    env.pop(key, None)
env.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(T), 'CARGO_BUILD_JOBS': '2',
            'CARGO_NET_OFFLINE': 'true', 'CARGO_INCREMENTAL': '0', 'TMPDIR': str(scratch),
            'CARGO_TERM_COLOR': 'never', 'RUST_BACKTRACE': '0'})
pins = {str(p): sha(p) for p in [Path(__file__), manifest, W / 'baseline-preparation-v1/result.json', W / 'contract-v1.md']}
dump(O / 'protocol.json', {'input_sha256': pins, 'source': str(S), 'target': str(T), 'target_initially_empty': not list(T.iterdir()),
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True), 'cargo': subprocess.check_output(['cargo', '-V'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True), 'environment': {k: env[k] for k in ['CARGO_HOME','CARGO_TARGET_DIR','CARGO_BUILD_JOBS','CARGO_NET_OFFLINE','CARGO_INCREMENTAL','TMPDIR']},
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'cost_scope': 'Diagnostic preflight only. Preconstructed input pairs are outside cold/warm timing. Each phase includes lookup, exact independent witness assertions and complete ordered scalar-string row materialization; printing and destruction are outside phase timing. Process RSS includes all work. No allocation or final performance admission claim.'})
rows = []
def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8*1024**3, 8*1024**3)); resource.setrlimit(resource.RLIMIT_CORE, (0,0))
def run(name, command, extra=None):
    print('START ' + name, flush=True)
    start = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        r = subprocess.run(['/usr/bin/time','-v','-o',str(O / (name + '.resources')), 'timeout','--signal=TERM','--kill-after=20s','1800s',*command],
            cwd=S, env=env | (extra or {}), stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    assert all(sha(S / p) == h for p, h in inputs.items()) and all(sha(p) == h for p, h in pins.items())
    row = {'name':name,'command':command,'environment_overrides':extra or {},'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,
           **{suffix+'_sha256':sha(O / (name+'.'+suffix)) for suffix in ['stdout','stderr','resources']}}
    rows.append(row); dump(O / 'commands.json',rows)
    print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}), flush=True)
    return r.returncode
rc = run('build', ['cargo','test','--offline','--locked','-p','hypercurve','--release','--no-default-features','--lib','--no-run','--message-format=json'])
binary = None
if not rc:
    artifacts = [json.loads(line) for line in (O / 'build.stdout').read_text().splitlines() if line.startswith('{')]
    binaries = {a['executable'] for a in artifacts if a.get('reason')=='compiler-artifact' and a['target']['name']=='hypercurve' and a.get('executable')}
    assert len(binaries)==1
    binary=Path(binaries.pop()); retained=O / 'hypercurve-tests';shutil.copy2(binary,retained);assert sha(binary)==sha(retained)
    dump(O / 'binary.json', {'built':str(binary),'retained':str(retained),'sha256':sha(binary),'bytes':binary.stat().st_size})
    run('contracts',[str(retained),'curve_intersection::circle_support_cache_tests::','--nocapture','--test-threads=1'])
    run('existing-cache',[str(retained),'curve_intersection::native_dispatch_tests::batch_cache_reuses_one_circle_support_relation_across_spans','--exact','--nocapture','--test-threads=1'])
    for count in [1,16,128,512,2048]:
        run('cost-'+str(count),[str(retained),'curve_intersection::circle_support_cost_probe::support_lookup_cost','--exact','--ignored','--nocapture','--test-threads=1'],{'CIRCLE_CACHE_COST_N':str(count)})
dump(O / 'result.json', {'complete':True,'command_failures':[r['name'] for r in rows if r['exit_code']],
    'sources_unchanged':True,'cargo_target_closed':True,'native_runtime_admitted':False,'full_plan_complete':False,
    'cost_is_preflight_only':True,'main_sources_modified':False})
print(json.dumps({'complete':True,'failures':[r['name'] for r in rows if r['exit_code']]}), flush=True)
