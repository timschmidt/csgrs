from pathlib import Path
import hashlib, json, re, subprocess

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-rational-polynomial-v6-20260924')
prefix = 'rational-polynomial-20260924-v6'
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
for name, sha in manifest.items():
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
scalar = json.loads((A / 'rational-polynomial-20260924-v5-terminal.json').read_text())
dependent = json.loads((A / 'rational-polynomial-20260924-v5-dependents-terminal.json').read_text())
public = json.loads((A / 'rational-polynomial-20260924-v5-public-scalars-terminal.json').read_text())
full = json.loads((A / f'{prefix}-full-terminal.json').read_text())
demand = json.loads((A / 'rational-polynomial-20260924-v5-demand-terminal.json').read_text())
inventory = json.loads((A / f'{prefix}-inventory-terminal.json').read_text())
old = json.loads((A / 'rational-polynomial-20260924-v5-sources.json').read_text())
assert [name for name in manifest if manifest[name] != old[name]] == ['hyperreal/tests/gmp_api_coverage.rs']
for report in [scalar, dependent, public, full, demand, inventory]:
    assert report['all_processes_reaped'] and report['all_sources_unchanged']
assert all(row['returncode'] == 0 for row in scalar['checks'] + scalar['tests'])
for crate in dependent['crates']:
    assert all(row['returncode'] == 0 for row in crate['checks'])
    if crate['crate'] != 'hypercurve':
        assert all(row['returncode'] == 0 for row in crate['cases'])
assert inventory['returncode'] == 0 and all(row['returncode'] == 0 for row in inventory['checks'])
for crate in public['crates']:
    assert all(row['returncode'] == 0 for row in crate['targets'] if (crate['crate'], row['name']) != ('hyperreal', 'gmp_api_coverage')), crate
baseline = json.loads((A / 'homogeneous-composition-20260924-v3-qualification-cases.json').read_text())
baseline += json.loads((A / 'shared-source-bounds-20260924-v2-broad-cases.json').read_text())
previous = {(row['target'], row['name']): row for row in baseline}
rows = json.loads((A / f'{prefix}-full-cases.json').read_text())
assert len(rows) == len({(row['target'], row['name']) for row in rows})
nonpasses = [row for row in rows if not row['passed'] and not row['ignored']]
for row in nonpasses:
    old = previous.get((row['target'], row['name']))
    assert old and not old['passed'] and old['returncode'] == row['returncode'], row
assert sum(row['passed'] for row in rows if row['target'] != 'hypercurve') == 121
newly_passing = [row['name'] for row in rows if row['passed'] and (old := previous.get((row['target'], row['name']))) and not old['passed'] and not old['ignored']]
changed = {
    'hyperreal': [
        'src/computable/node/quadratic_tower.rs',
        'src/rational/arithmetic/aggregate_products.rs',
        'src/rational/arithmetic/construction.rs',
        'src/real/arithmetic/linear_algebra.rs',
        'src/real/arithmetic/tests.rs',
        'tests/gmp_api_coverage.rs',
    ],
    'hypersolve': [
        'src/algebraic_fiber.rs', 'src/algebraic_tensor_image.rs',
        'src/integer_interpolation.rs', 'tests/tensor_remainder_scale.rs',
    ],
}
repositories = []
for repo in sorted(W.iterdir()):
    if not (repo / '.git').exists():
        continue
    paths = subprocess.check_output(['git', 'diff', '--name-only'], cwd=repo, text=True).splitlines()
    assert set(paths) == set(changed.get(repo.name, [])), (repo.name, paths)
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=repo, text=True)
    assert not subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard'], cwd=repo, text=True)
    subprocess.run(['git', 'diff', '--check'], cwd=repo, check=True)
    repositories.append(dict(name=repo.name, head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()))
files = {repo: {path: manifest[repo + '/' + path] for path in paths} for repo, paths in changed.items()}
report = dict(files=files, repositories=repositories, manifest=f'{prefix}-sources.json', all_2044_inputs_verified=True, all_owned_processes_reaped=True, full_attempted=len(rows), full_passed=sum(row['passed'] for row in rows), full_ignored=sum(row['ignored'] for row in rows), unchanged_nonpasses=nonpasses, newly_passing=newly_passing, scalar_tests=scalar['tests'], dependent_crates=dependent['crates'], scalar_public_targets=public['crates'], repaired_inventory=inventory, input_bridge='rational-polynomial-20260924-v6-input-bridge.json')
(A / f'{prefix}-qualification.json').write_text(json.dumps(report, indent=2) + '\n')
print({key: report[key] for key in ['full_attempted', 'full_passed', 'full_ignored', 'newly_passing']})
print('Qualified files:', files)
