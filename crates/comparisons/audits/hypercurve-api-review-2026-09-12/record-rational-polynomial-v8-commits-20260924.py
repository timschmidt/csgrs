from pathlib import Path
import hashlib, json, subprocess

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'rational-polynomial-20260924-v8'
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
    out.write(f"""

## 24 September 2026 — exact rational reentry and shared polynomial accumulation

Hyperreal `{commits['hyperreal'][:7]}` fixes a demonstrated lazy-rational
invariant violation in the shared scalar layer. Adding one to a retained 4/6
payload previously published 10/6 as if reduced: numeric equality with 5/3
held, but canonical payload queries and hashes disagreed. Addition, subtraction,
unit helpers, optimized means and power-of-two division now enforce the
reduced-input preconditions of their arithmetic theorems. Multiplication and
division also reuse cached canonical inputs before specialized arithmetic.
The duplicate lazy normalization implementation is replaced by the standard
fraction reducer, preserving its dyadic shift optimization. No public numeric
interface or compatibility shim is added.

Long genuinely rational polynomials use homogeneous integer accumulation over
one positive coefficient denominator. Every Horner step preserves numeric value;
exactly cancelled suffixes discard unnecessary denominator history. The existing
lazy Rational representation defers final normalization until needed. Degree-four
and shorter polynomials retain the existing scalar arithmetic path. Arbitrary
nonrational Real coefficients and arguments keep their exact evaluation paths.
There is no new representation or cache.

Independent GMP power sums cover wide coefficients and arguments, signed scales,
cancellation, leading zeros and lazy inputs. An independent BigRational oracle
checks canonical payloads, integer/unit facts and equal-value hashes through
arithmetic and aggregate reentry. The public polynomial counterexample now
passes all three checks. An old dyadic dispatch assertion was updated to require
the newly recovered shift-only path, retaining its exact-value oracle.

All 835 Hyperreal, 19 Hyperlattice, 252 Hyperlimit and 507 Hypersolve library tests
pass. All 41 public scalar integration targets pass (15 Hyperreal, 17 Hyperlattice,
9 Hypersolve), including the API inventory. Both all-target Clippy feature
configurations pass for all five stack crates, including Hypercurve. Hyperreal's
pre-existing fixed-chunk test lints and four missing no-direct-GMP-analog exact
certificate classifications are repaired. Hypersolve `{commits['hypersolve'][:7]}`
clears its existing Clippy findings without changing mathematical behavior.

Full Hypercurve qualification covers {qualified['full_attempted']} cases:
{qualified['full_passed']} pass, {qualified['full_ignored']} are ignored, and
{len(qualified['unchanged_nonpasses'])} remain unchanged nonpasses. All 121 public
integration cases pass. Seven focused receipts from the exact same frozen
library executable are reused; the remaining cases run with two workers.
All deadlines are unchanged. Earlier concurrent V6 timeouts for the recursive
chamfer and chord/selected-circle fillet pass in isolation at 61.23 and 147.20
library seconds, respectively, matching their earlier isolated timings.
No concurrent throughput improvement is claimed.

Newly passing cases: {', '.join(qualified['newly_passing']) or 'none'}.
The extended companion fillet completes in 38.09 library seconds, and the
selected-parallel companion fillet in 0.35 seconds. Both had exceeded the
75-second limit previously. The third-generation fillet remains a separate
bounded-workload nonpass. Fresh wide polynomial sign queries improve in the
recorded samples; forcing canonical values gives mixed to substantial gains.
Repeating identical long wide inputs can still favor the older retained scalar
arithmetic chain. These are bounded comparisons, not universal speedup claims.

Source, executable, index and HEAD bindings are recorded in `{prefix}-qualification.json`
and the adjacent staged/post-commit records. All 2,044 input bindings match;
all owned processes are reaped; all thirty repositories are clean. Nothing was
pushed. The full implementation goal remains active. Normalized corner callers,
duplicate retained tangent parameters and shared selected-root authority retain
their proof obligations in the adjacent follow-up documents.
""")
with (A / 'rational-polynomial-20260924-proof.md').open('a') as out:
    out.write('\nQualified and committed: ' + ', '.join(name + ' ' + head for name, head in commits.items()) + '. All owned processes reaped; all thirty repositories clean; no push. Full goal remains active.\n')
print(commits)
