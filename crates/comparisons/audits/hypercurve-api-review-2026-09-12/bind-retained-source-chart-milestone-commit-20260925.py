from pathlib import Path
import hashlib, json, subprocess, sys
A = Path(__file__).resolve().parent
W = A.parent
version, phase = sys.argv[1:]
assert phase in ['staged', 'post-commit']
prefix = f'local-chord-complete-replay-20260924-{version}'
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
qualified = json.loads((A / f'{prefix}-milestone-qualification.json').read_text())
assert qualified['all_owned_processes_reaped'] and qualified['all_2044_inputs_match']
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
for name, sha in manifest.items():
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
full = json.loads((A / f'{prefix}-full-terminal.json').read_text())
for artifact in full['binaries'].values():
    assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
def git(repo, *args):
    return subprocess.check_output(['git', *args], cwd=repo)
repo = W / 'hypercurve'
head = git(repo, 'rev-parse', 'HEAD').decode().strip()
if phase == 'staged':
    assert head == qualified['parent']
    assert git(repo, 'diff', '--cached', '--name-only').decode().splitlines() == list(qualified['files'])
    assert not git(repo, 'diff', '--name-only')
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=repo, check=True)
    revision = ':'
else:
    assert git(repo, 'rev-parse', 'HEAD^').decode().strip() == qualified['parent']
    revision = 'HEAD:'
for name, sha in qualified['files'].items():
    assert hashlib.sha256(git(repo, 'show', revision + name)).hexdigest() == sha, name
repositories = []
for repo in sorted(W.iterdir()):
    if not (repo / '.git').exists():
        continue
    status = git(repo, 'status', '--porcelain=v1').decode()
    if phase == 'post-commit' or repo.name != 'hypercurve':
        assert not status, (repo.name, status)
    assert not git(repo, 'ls-files', '--others', '--exclude-standard')
    repositories.append(dict(name=repo.name, head=git(repo, 'rev-parse', 'HEAD').decode().strip(), clean=not status))
assert len(repositories) == 30
report = dict(head=head, parent=qualified['parent'], files=qualified['files'],
              all_2044_inputs_match=True, all_owned_processes_reaped=True,
              committed_bytes_match_qualification=phase == 'post-commit',
              repositories=repositories, unresolved_completion_cases=qualified['unresolved_completion_cases'], full_goal_complete=False, pushed=False)
(A / f'{prefix}-milestone-{phase}.json').write_text(json.dumps(report, indent=2) + '\n')
print(phase, 'bytes match qualification; all 2044 source bindings and executable hashes verified.')
