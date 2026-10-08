from pathlib import Path
import hashlib, json, os, shutil, subprocess

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypersolve-low-degree-witness-parent-2026-09-23')
candidate = Path('/tmp/hypercurve-chord-normal-witness-v2-2026-09-23')
prefix = 'low-degree-witness-20260923-parent1'
env = dict(os.environ, **json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
bindings = json.loads((audit/'low-degree-witness-parent-20260923-sources.json').read_text())
candidate_bindings = json.loads((audit/'chord-normal-witness-20260923-focused2-sources.json').read_text())

def verify():
    for directory, sources in [(root, bindings), (candidate, candidate_bindings)]:
        for name, sha in sources.items():
            assert hashlib.sha256((directory/name).read_bytes()).hexdigest() == sha, name

verify()
command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (audit/f'{prefix}-build.jsonl').open('w') as out, (audit/f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=root/'hypersolve', env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0
artifacts = []
for line in (audit/f'{prefix}-build.jsonl').read_text().splitlines():
    row = json.loads(line)
    if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == 'hypersolve':
        artifacts.append(row)
assert len(artifacts) == 1
binary = audit/f'{prefix}-libtest'
shutil.copy2(artifacts[0]['executable'], binary)
rows = []
for index, suffix in enumerate(['selected_low_degree_witness_does_not_depend_on_interval_precision', 'selected_low_degree_witness_accepts_exact_nonrational_coefficients']):
    name = 'algebraic_tensor_image::tests::'+suffix
    log = audit/f'{prefix}-case-{index}.log'
    with log.open('w') as out:
        code = subprocess.run([str(binary), '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], cwd=root/'hypersolve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
    verify()
    output = log.read_text()
    assert code == 101 and 'running 1 test' in output and '0 passed; 1 failed;' in output, output
    rows.append(dict(name=name, returncode=code, log=log.name))
    print('Parent fails new regression:', name, output[-1200:], flush=True)
checks = []
for feature in ['--all-features', '--no-default-features']:
    command = [cargo, 'check', '--all-targets', feature, '--locked', '--offline']
    log = audit/f'low-degree-witness-20260923-check-{len(checks)}.log'
    with log.open('w') as out:
        code = subprocess.run(command, cwd=candidate/'hypersolve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=1200).returncode
    verify()
    checks.append(dict(command=command, returncode=code, log=log.name))
    assert code == 0 and 'warning:' not in log.read_text()
    print('Candidate all-target check passed without warnings:', feature, flush=True)
report = dict(parent_cases=rows, parent_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), candidate_checks=checks, all_sources_unchanged=True, all_processes_reaped=True)
(audit/'low-degree-witness-20260923-checks-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Terminal: parent regressions and both candidate checks reaped.', flush=True)
