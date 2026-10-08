from pathlib import Path
import hashlib, json, os, shutil, subprocess

audit = Path(__file__).resolve().parent
prefix = 'witness-ownership-20260923'
env = dict(os.environ, **json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
reports = {}
for kind in ['candidate', 'parent']:
    root = Path('/tmp/hypersolve-witness-ownership-'+kind+'-2026-09-23')
    bindings = json.loads((audit/f'{prefix}-{kind}-sources.json').read_text())
    def verify():
        for name, sha in bindings.items():
            assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
    verify()
    stem = f'{prefix}-{kind}'
    command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
    with (audit/f'{stem}-build.jsonl').open('w') as out, (audit/f'{stem}-build.log').open('w') as err:
        code = subprocess.run(command, cwd=root/'hypersolve', env=env, stdout=out, stderr=err, timeout=1200).returncode
    verify()
    assert code == 0
    artifacts = []
    for line in (audit/f'{stem}-build.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == 'hypersolve':
            artifacts.append(row)
    assert len(artifacts) == 1
    binary = audit/f'{stem}-libtest'
    shutil.copy2(artifacts[0]['executable'], binary)
    command = [str(binary), '--test-threads=2', '--color', 'never'] if kind == 'candidate' else [str(binary), '--exact', 'algebraic_tensor_image::tests::selected_quadratic_witness_preserves_repeated_and_endpoint_roots', '--nocapture', '--test-threads=1', '--color', 'never']
    log = audit/f'{stem}-tests.log'
    with log.open('w') as out:
        code = subprocess.run(command, cwd=root/'hypersolve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=600).returncode
    verify()
    output = log.read_text()
    if kind == 'candidate':
        assert code == 0 and '506 passed; 0 failed;' in output, output[-3000:]
    else:
        assert code == 101 and '0 passed; 1 failed;' in output, output[-3000:]
    reports[kind] = dict(returncode=code, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), log=log.name)
    print(kind, output[-1300:], flush=True)
    if kind == 'candidate':
        checks = []
        for feature in ['--all-features', '--no-default-features']:
            command = [cargo, 'check', '--all-targets', feature, '--locked', '--offline']
            check_log = audit/f'{stem}-check-{len(checks)}.log'
            with check_log.open('w') as out:
                code = subprocess.run(command, cwd=root/'hypersolve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=1200).returncode
            verify()
            assert code == 0 and 'warning:' not in check_log.read_text()
            checks.append(dict(command=command, returncode=code, log=check_log.name))
            print('Passed without warnings:', feature, flush=True)
        reports[kind]['checks'] = checks
reports.update(all_processes_reaped=True, all_sources_unchanged=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(reports, indent=2)+'\n')
print('Terminal: witness ownership candidate and parent fully reaped.', flush=True)
