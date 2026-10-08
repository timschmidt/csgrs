from pathlib import Path
import hashlib, json, os, re, subprocess, time

root = Path('/tmp/hypercurve-pruning-qualification')
audit = Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
prefix = 'finite-parallel-pruning-isolated'
manifest = json.loads((audit / (prefix + '-sources.json')).read_text())
def verify():
    for row in manifest['files']:
        assert hashlib.sha256((root / row['file']).read_bytes()).hexdigest() == row['sha256'], row['file']
verify()
bin_dir = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
env = dict(os.environ, CARGO_BUILD_JOBS='2', RUSTC=str(bin_dir/'rustc'), RUSTDOC=str(bin_dir/'rustdoc'),
           CCACHE_DIR=str(root/'ccache'), CCACHE_TEMPDIR=str(root/'ccache-tmp'))
(root/'ccache-tmp').mkdir(exist_ok=True)
rows = []
for name, args in [
    ('check', ['check', '--all-targets', '--no-default-features', '--locked', '--offline']),
    ('fuzz-check', ['check', '--manifest-path', 'fuzz/Cargo.toml', '--all-targets', '--locked', '--offline']),
    ('test-build', ['test', '--release', '--all-features', '--locked', '--offline', '--lib', '--tests', '--no-run', '--message-format=json']),
]:
    stem = prefix + '-hyperbrep-' + name
    command = [str(bin_dir/'cargo'), *args]
    start = time.monotonic()
    with (audit/(stem+'.log')).open('w') as err, (audit/(stem+'.jsonl')).open('w') as out:
        result = subprocess.run(command, cwd=root/'hyperbrep', env=env, stdout=out if name == 'test-build' else err, stderr=err, timeout=900)
    row = dict(name=name, command=command, returncode=result.returncode, elapsed_seconds=time.monotonic()-start, log=stem+'.log')
    rows.append(row)
    (audit/(stem+'.exit')).write_text(str(result.returncode)+'\n')
    print(name, result.returncode, round(row['elapsed_seconds'],2), flush=True)
    verify()
    assert result.returncode == 0, (audit/(stem+'.log')).read_text()[-5000:]
jobs = []
for line in (audit/(prefix+'-hyperbrep-test-build.jsonl')).read_text().splitlines():
    item = json.loads(line)
    if item.get('reason') == 'compiler-artifact' and item.get('executable') and item['profile']['test']:
        jobs.append(item)
assert len(jobs) == 1, len(jobs)
tests = []
for job in jobs:
    executable = Path(job['executable'])
    command = [str(executable), '--test-threads=4', '--color', 'never']
    log = audit/(prefix+'-hyperbrep-tests.log')
    start = time.monotonic()
    with log.open('w') as out:
        result = subprocess.run(command,cwd=root/'hyperbrep',stdout=out,stderr=subprocess.STDOUT,timeout=300)
    text = log.read_text()
    statuses = re.findall(r'^test ([^\n]+?) \.\.\. (ok|FAILED|ignored[^\n]*)$',text,re.M)
    row = dict(command=command, binary_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(), returncode=result.returncode, elapsed_seconds=time.monotonic()-start, log=log.name,
               passed=[n for n,s in statuses if s=='ok'], failed=[n for n,s in statuses if s=='FAILED'], ignored=[n for n,s in statuses if s.startswith('ignored')])
    tests.append(row)
    print('tests',result.returncode,'passed',len(row['passed']),'failed',len(row['failed']),flush=True)
    assert result.returncode == 0 and len(row['passed']) == 232
verify()
report = dict(status='passed', source_manifest=prefix+'-sources.json', verified_source_files=len(manifest['files']), repositories=manifest['repositories'], checks=rows, tests=tests,
              replaces='Original HyperBREP validation used a concurrently edited Hyperreal dependency; those earlier results remain historical evidence only.')
(audit/(prefix+'-qualification.json')).write_text(json.dumps(report,indent=2)+'\n')
print('isolated downstream qualification complete',flush=True)
