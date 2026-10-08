from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
version, phase = sys.argv[1:]
assert phase in {'qualification', 'staged', 'post-commit'}
prefix = f'polynomial-determinant-20260925-{version}'
plan = json.loads((A/f'{prefix}-plan.json').read_text())
terminal = json.loads((A/f'{prefix}-terminal.json').read_text())
manifest = json.loads((A/plan['source_manifest']).read_text())
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
assert terminal['all_processes_reaped'] and terminal['all_sources_unchanged']
assert len(terminal['checks']) == 3 and len(terminal['cases']) == 4
assert all(row['returncode'] == 0 for row in terminal['checks'] + terminal['cases'])
assert terminal['passed'] + terminal['ignored'] == terminal['library_case_count']
assert terminal['passed'] > 0
assert 'warning:' not in (A/f'{prefix}-build.log').read_text()
for name, sha in manifest.items():
    assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
assert hashlib.sha256(Path(terminal['binary']).read_bytes()).hexdigest() == terminal['binary_sha256']

def git(repo, *args):
    return subprocess.check_output(['git', *args], cwd=repo)

repositories = []
for repo in sorted(W.iterdir()):
    if not (repo/'.git').exists():
        continue
    head = git(repo, 'rev-parse', 'HEAD').decode().strip()
    unstaged = git(repo, 'diff', '--name-only').decode().splitlines()
    staged = git(repo, 'diff', '--cached', '--name-only').decode().splitlines()
    assert not git(repo, 'ls-files', '--others', '--exclude-standard'), repo.name
    subprocess.run(['git', 'diff', '--check'], cwd=repo, check=True)
    if repo.name == 'hypersolve':
        assert unstaged == (plan['files'] if phase == 'qualification' else [])
        assert staged == (plan['files'] if phase == 'staged' else [])
        if phase == 'post-commit':
            assert git(repo, 'rev-parse', 'HEAD^').decode().strip() == plan['parent']
        else:
            assert head == plan['parent']
        if phase != 'qualification':
            tree = ':' if phase == 'staged' else 'HEAD:'
            for name in plan['files']:
                assert hashlib.sha256(git(repo, 'show', tree+name)).hexdigest() == manifest['hypersolve/'+name]
    elif repo.name == 'hypercurve':
        assert unstaged == ['src/bezier_offset.rs', 'src/bezier_parameter.rs']
        assert not staged
    else:
        assert not unstaged and not staged, repo.name
    repositories.append(dict(name=repo.name, head=head))
if phase != 'qualification':
    qualified = json.loads((A/f'{prefix}-qualification.json').read_text())
    before = {row['name']: row['head'] for row in qualified['repositories']}
    for row in repositories:
        if row['name'] != 'hypersolve':
            assert row['head'] == before[row['name']]
report = dict(phase=phase, parent=plan['parent'], repositories=repositories,
              files={name: manifest['hypersolve/'+name] for name in plan['files']},
              terminal=f'{prefix}-terminal.json', source_manifest=plan['source_manifest'],
              all_sources_unchanged=True, all_owned_processes_reaped=True,
              passed=terminal['passed'], ignored=terminal['ignored'],
              hypercurve_candidate_uncommitted=True, full_goal_complete=False)
output = A/f'{prefix}-{phase}.json'
assert not output.exists()
output.write_text(json.dumps(report, indent=2)+'\n')
print(phase, terminal['passed'], 'passed,', terminal['ignored'], 'ignored;', len(repositories), 'repositories audited.')
