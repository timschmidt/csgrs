from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
prefix = f'root-identity-20260925-{version}'
plan = json.loads((A / f'{prefix}-plan.json').read_text())
manifest = json.loads((A / plan['source_manifest']).read_text())
assert not (A / f'{prefix}-terminal.json').exists()
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain / 'cargo')
repo = root / 'hypersolve'
report = dict(checks=[], cases=[], all_processes_reaped=False, plan=f'{prefix}-plan.json')

def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name

def save():
    verify()
    report['all_sources_unchanged'] = True
    (A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2) + '\n')

def run(label, command, limit):
    verify()
    log = A / f'{prefix}-{label}.log'
    assert not log.exists()
    started = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command, cwd=repo, env=env, stdout=out,
                                  stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    verify()
    row = dict(label=label, command=command, returncode=code,
               elapsed_seconds=time.monotonic()-started, log=log.name)
    print(label, code, log.read_text()[-2000:] if code else '', flush=True)
    return row

def require(row, group):
    report[group].append(row)
    if row['returncode'] != 0:
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)

require(run('fmt', [str(toolchain/'rustfmt'), '--edition', '2024', '--check', *plan['files']], 60), 'checks')
for feature in ['--all-features', '--no-default-features']:
    require(run('clippy-' + str(len(report['checks'])),
                [cargo, 'clippy', '--all-targets', feature, '--locked', '--offline', '--', '-D', 'warnings'],
                1200), 'checks')
os.utime(repo/'src/lib.rs', None)
command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A/f'{prefix}-build.jsonl').open('w') as out, (A/f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
if code:
    report['build_returncode'] = code
    report['all_processes_reaped'] = True
    save()
    raise SystemExit(code)
rows = [json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact'
                and row['target']['name'] == 'hypersolve' and row.get('executable'))
assert not artifact['fresh']
binary = A/f'{prefix}-libtest'
shutil.copy2(artifact['executable'], binary)
report['binary'] = str(binary)
report['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines()
         if line.endswith(': test')]
report['library_case_count'] = len(names)
for suffix in ['algebraic_root_comparison_reuses_nested_singleton_evidence',
               'algebraic_root_comparison_keeps_overlapping_conjugates_distinct',
               'algebraic_root_comparison_respects_owned_interval_endpoints']:
    matching = [name for name in names if name.rsplit('::', 1)[-1] == suffix]
    assert len(matching) == 1, suffix
    require(run(suffix, [str(binary), '--exact', matching[0], '--test-threads=1', '--color', 'never'], 75), 'cases')
require(run('full-library', [str(binary), '--test-threads=2', '--color', 'never'], 600), 'cases')
output = (A/report['cases'][-1]['log']).read_text()
match = re.search(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;.*? (\d+) filtered out;', output)
assert match, output[-1000:]
passed, failed, ignored, filtered = map(int, match.groups())
assert not failed and not filtered and passed+ignored == len(names)
report.update(passed=passed, ignored=ignored, all_processes_reaped=True)
assert hashlib.sha256(binary.read_bytes()).hexdigest() == report['binary_sha256']
save()
print('Hypersolve terminal:', passed, 'passed,', ignored, 'ignored; sources and executable unchanged.', flush=True)
