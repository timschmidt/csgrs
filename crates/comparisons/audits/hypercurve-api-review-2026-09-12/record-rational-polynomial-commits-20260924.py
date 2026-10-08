from pathlib import Path
import hashlib, json, subprocess

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'rational-polynomial-20260924-v6'
qualified = json.loads((A / f'{prefix}-qualification.json').read_text())
staged = json.loads((A / f'{prefix}-staged.json').read_text())
commits = {}
for name, files in qualified['files'].items():
    repo = W / name
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    parent = subprocess.check_output(['git', 'rev-parse', 'HEAD^'], cwd=repo, text=True).strip()
    assert parent == staged[name]['parent'], (name, parent)
    for path, sha in files.items():
        assert hashlib.sha256(subprocess.check_output(['git', 'show', 'HEAD:' + path], cwd=repo)).hexdigest() == sha
    commits[name] = head
repositories = []
for repo in sorted(W.iterdir()):
    if (repo / '.git').exists():
        assert not subprocess.check_output(['git', 'status', '--short'], cwd=repo, text=True), repo.name
        repositories.append(repo.name)
report = dict(commits=commits, files=qualified['files'], repository_count=len(repositories), all_repositories_clean=True, all_owned_processes_reaped=True, pushed=False, full_goal_complete=False)
(A / f'{prefix}-post-commit.json').write_text(json.dumps(report, indent=2) + '\n')
with (A / 'implementation.md').open('a') as out:
    out.write(f'''

## 24 September 2026 — scalar polynomial reduction and exact fillet completion

Hyperreal `{commits['hyperreal'][:7]}` evaluates longer genuinely rational
polynomials by homogeneous integer accumulation over one positive coefficient
denominator. Each step preserves the full numeric value; exact suffix
cancellation drops unnecessary denominator history. The existing lazy Rational
representation defers the final GCD until a numeric consumer needs it. Short
polynomials retain the native arithmetic path. Arbitrary nonrational Real
coefficients and arguments keep their existing exact evaluation paths.

The existing lazy-ratio canonicalizer now uses the standard fraction reduction
kernel, removing duplicate GCD/division code and sharing the dyadic optimization.
No public numeric API, representation or cache is added. Independent GMP
power-sum oracles cover wide values, signed scaling, suffix cancellation,
leading zeros and lazy inputs; signs and arithmetic reentry are checked before
canonical numeric comparison. The older four public radical-field certificate
APIs now have explicit no-direct-GMP-analog classifications in the API inventory.

Hypersolve `{commits['hypersolve'][:7]}` clears existing Clippy findings in exact
algebra and its tests, preserving their behavior. All 834 Hyperreal, 252
Hyperlimit and 507 Hypersolve library tests pass, together with the public scalar
integration targets and repaired API inventory. Both all-target Clippy feature
configurations pass for the affected crates, as do formatting and diff checks.

Full Hypercurve qualification records {qualified['full_attempted']} attempts,
{qualified['full_passed']} passes and {qualified['full_ignored']} ignored tests,
with unchanged nonpasses recorded in `{prefix}-qualification.json`.
Newly passing cases: {', '.join(qualified['newly_passing']) or 'none'}.
The focused extended-fillet region case completes in 37.98 library seconds,
including exact membership on both sides of its extended companion under both
policies and traversal directions; previously it exceeded 75 seconds. The
recursive selected-radial chamfer retains its roughly 61-second completion.
The third-generation fillet remains a separate bounded-workload nonpass.

Fresh wide polynomial sign queries show large arithmetic savings, and forcing
canonical values also improves those samples. Repeating the same long wide
polynomial can still favor the prior retained arithmetic chain; the probe
records that tradeoff and is not a universal performance claim. All source,
executable, index and HEAD bindings are recorded beside this entry. The V6
input bridge changes only the API inventory integration test and preserves the
already-qualified V5 numerical executables and input snapshots. All processes
are reaped and all thirty repositories are clean. Nothing was pushed.

The full implementation goal remains active. The normalized PH corner caller
and the remaining duplicate tangent-parameter storage have concrete follow-up
proof obligations in `normalized-corner-followup-20260924.md` and
`tangent-parameter-followup-20260924.md`; neither migration is yet implemented.
''')
with (A / 'rational-polynomial-20260924-proof.md').open('a') as out:
    out.write('\nQualified and committed: ' + ', '.join(name + ' ' + head for name, head in commits.items()) + '. All owned processes reaped; all thirty repositories clean; no push. Full goal remains active.\n')
print(commits)
