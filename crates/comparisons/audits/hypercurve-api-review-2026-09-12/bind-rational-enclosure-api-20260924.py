from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
version, phase = sys.argv[1:]
assert phase in ['qualified', 'staged', 'post-commit']
prefix = f'rational-enclosure-api-20260924-{version}'
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
plan = json.loads((A/f'{prefix}-commit-plan.json').read_text())
manifest = json.loads((A/f'{prefix}-sources.json').read_text())
terminal = json.loads((A/f'{prefix}-terminal.json').read_text())
assert terminal['all_processes_reaped'] and terminal['all_sources_unchanged']
assert len(terminal['checks']) == 17
assert len(terminal['cases']) == 8
assert all(row['returncode'] == 0 for row in terminal['checks'] + terminal['cases'])
for binary in terminal['binaries'].values():
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
for name, sha in manifest.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
def git(repo, *args):
    return subprocess.check_output(['git', *args], cwd=W/repo)
repositories = []
for path in sorted(W.iterdir()):
    if not (path/'.git').exists(): continue
    repo = path.name
    head = git(repo, 'rev-parse', 'HEAD').decode().strip()
    unstaged = git(repo, 'diff', '--name-only').decode().splitlines()
    staged = git(repo, 'diff', '--cached', '--name-only').decode().splitlines()
    assert not git(repo, 'ls-files', '--others', '--exclude-standard'), repo
    git(repo, 'diff', '--check')
    git(repo, 'diff', '--cached', '--check')
    if repo not in plan:
        assert not staged and not unstaged, repo
    else:
        expected = plan[repo]['staged_files']
        if phase != 'post-commit':
            assert head == plan[repo]['parent'], repo
        if phase == 'qualified':
            assert not staged, repo
            assert unstaged == sorted(expected), (repo, unstaged)
            if repo != 'hypercurve':
                for name, sha in expected.items():
                    assert hashlib.sha256((W/repo/name).read_bytes()).hexdigest() == sha, (repo, name)
        else:
            assert staged == (sorted(expected) if phase == 'staged' else []), (repo, staged)
            assert unstaged == (['src/bezier_offset.rs', 'src/bezier_parameter.rs'] if repo == 'hypercurve' else []), (repo, unstaged)
            for name, sha in expected.items():
                blob = (':' if phase == 'staged' else 'HEAD:') + name
                assert hashlib.sha256(git(repo, 'show', blob)).hexdigest() == sha, (repo, name)
            if phase == 'post-commit':
                assert head != plan[repo]['parent'], repo
                assert git(repo, 'rev-parse', 'HEAD^').decode().strip() == plan[repo]['parent'], repo
                assert git(repo, 'diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD').decode().splitlines() == sorted(expected), repo
    repositories.append(dict(repo=repo, head=head, staged=staged, unstaged=unstaged))
assert len(repositories) == 30
report = dict(phase=phase, repositories=repositories, manifest=f'{prefix}-sources.json',
              all_source_and_executable_hashes_match=True, all_owned_processes_reaped=True,
              scalar_and_caller_qualification=f'{prefix}-terminal.json',
              curve_migration_is_mechanical=True, pending_curve_contact_changes_uncommitted=True,
              full_goal_complete=False)
target = A/f'{prefix}-{phase}.json'
assert not target.exists()
target.write_text(json.dumps(report, indent=2)+'\n')
print(phase, 'verified:', len(plan), 'migrated repositories;', len(manifest), 'bound source/configuration inputs')
