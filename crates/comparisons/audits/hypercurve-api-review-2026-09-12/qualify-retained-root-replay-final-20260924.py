from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'retained-coordinate-identity-20260924-{version}'
root = Path(f'/tmp/hypercurve-retained-coordinate-identity-{version}-20260924')
prior_prefix = 'retained-coordinate-identity-20260924-v4'
prior_root = Path('/tmp/hypercurve-retained-coordinate-identity-v4-20260924')
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
prior_manifest = json.loads((A / f'{prior_prefix}-sources.json').read_text())
focused = json.loads((A / f'{prefix}-terminal.json').read_text())
full = json.loads((A / f'{prior_prefix}-full-terminal.json').read_text())
cases = json.loads((A / f'{prior_prefix}-full-cases.json').read_text())
selection = json.loads((A / f'{prior_prefix}-full-selection.json').read_text())
assert len(manifest) == len(prior_manifest) == 2044
assert [name for name in manifest if manifest[name] != prior_manifest[name]] == ['hypercurve/src/bezier_region.rs']
for name, sha in manifest.items():
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((prior_root / name).read_bytes()).hexdigest() == prior_manifest[name], name

def without_corrected_test(source):
    start = source.index('    #[test]\n    fn one_fragment_materialized_loop_extends_algebraic_chamfer_cuts_once()')
    end = source.index('    #[test]\n    fn one_fragment_selected_loop_extends_chamfer_cuts_on_its_analytic_carrier()', start)
    return source[:start] + source[end:]

assert without_corrected_test((root / 'hypercurve/src/bezier_region.rs').read_text()) == without_corrected_test((prior_root / 'hypercurve/src/bezier_region.rs').read_text())
for result in [focused, full]:
    assert result['all_sources_unchanged'] and result['all_processes_reaped']
    assert len(result['checks']) == 3
    assert all(row['returncode'] == 0 for row in result['checks'])
assert len(focused['cases']) == 10
assert all(row['returncode'] == 0 for row in focused['cases'])
binary = A / f'{prefix}-libtest'
assert hashlib.sha256(binary.read_bytes()).hexdigest() == focused['binary_sha256']
for artifact in full['binaries'].values():
    assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
assert len(cases) == full['attempted'] == len(selection['jobs']) == 1467
assert {(row['target'], row['name']) for row in cases} == {tuple(job) for job in selection['jobs']}
names = {line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines() if line.endswith(': test')}
assert names == {row['name'] for row in cases if row['target'] == 'hypercurve'}
assert len(names) == 1242
combined = {(row['target'], row['name']): dict(row, source_manifest=f'{prior_prefix}-sources.json', reused_from=f'{prior_prefix}-full-terminal.json') for row in cases}
for case in focused['cases']:
    matches = [name for name in names if name.rsplit('::', 1)[-1] == case['label']]
    assert len(matches) == 1
    output = (A / case['log']).read_text()
    assert 'running 1 test' in output and '1 passed;' in output
    combined['hypercurve', matches[0]] = dict(target='hypercurve', name=matches[0], returncode=0, passed=True, ignored=False, elapsed_seconds=case['elapsed_seconds'], limit_seconds=75, log=case['log'], source_manifest=f'{prefix}-sources.json', reused_from=f'{prefix}-terminal.json')
cases = list(combined.values())
public = [row for row in cases if row['target'] != 'hypercurve']
assert len(public) == 225 and all(row['passed'] for row in public)
baseline_rows = json.loads((A / 'chord-point-containment-20260924-v3-full-cases.json').read_text())
baseline_rows += json.loads((A / 'represented-circle-incidence-20260924-v2-full-cases.json').read_text())
baseline = {(row['target'], row['name']): row for row in baseline_rows}
resolved = {
    'one_fragment_materialized_loop_chamfers_to_one_middle_interval',
    'one_fragment_selected_loop_chamfers_from_one_interval',
    'one_fragment_materialized_loop_extends_algebraic_chamfer_cuts_once',
    'one_fragment_selected_loop_extends_chamfer_cuts_on_its_analytic_carrier',
}
for row in cases:
    prior = baseline[row['target'], row['name']]
    if row['name'].rsplit('::', 1)[-1] in resolved or prior['passed']:
        assert row['passed'], row
    elif prior['ignored']:
        assert row['ignored'] or row['passed'], row
    elif not row['passed']:
        assert row['returncode'] == prior['returncode'], row
        assert row['limit_seconds'] == prior['limit_seconds'], row
files = ['src/bezier_offset.rs', 'src/bezier_region.rs']
parent = 'c46890e2694b50d8ed5dc0baddd43023e757dc3e'
repositories = []
for repo in sorted(W.iterdir()):
    if not (repo / '.git').exists():
        continue
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    paths = subprocess.check_output(['git', 'diff', '--name-only'], cwd=repo, text=True).splitlines()
    assert paths == (files if repo.name == 'hypercurve' else []), (repo.name, paths)
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=repo, text=True)
    assert not subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard'], cwd=repo, text=True)
    subprocess.run(['git', 'diff', '--check'], cwd=repo, check=True)
    if repo.name == 'hypercurve':
        assert head == parent
    repositories.append(dict(name=repo.name, head=head))
report = dict(parent=parent, files={name: manifest['hypercurve/' + name] for name in files}, repositories=repositories, manifest=f'{prefix}-sources.json', all_2044_inputs_match=True, all_owned_processes_reaped=True, production_identical_to_full_run=True, only_followup_change='Correct one test to pair traversal endpoints with oriented source parameters; retain exact replay assertions.', focused=f'{prefix}-terminal.json', full=f'{prior_prefix}-full-terminal.json', attempted=len(cases), passed=sum(row['passed'] for row in cases), ignored=sum(row['ignored'] for row in cases), unchanged_nonpasses=[row for row in cases if not row['passed'] and not row['ignored']], newly_passing=[row['name'] for row in cases if row['passed'] and not baseline[row['target'], row['name']]['passed']], public_integration_passes=len(public), full_goal_complete=False)
(A / f'{prefix}-combined-cases.json').write_text(json.dumps(cases, indent=2) + '\n')
(A / f'{prefix}-qualification.json').write_text(json.dumps(report, indent=2) + '\n')
print('Qualified:', report['attempted'], 'combined cases,', report['passed'], 'passed,', report['ignored'], 'ignored,', len(report['unchanged_nonpasses']), 'unchanged nonpasses')
