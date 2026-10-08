from pathlib import Path
import hashlib, json, shutil, subprocess

workspace = Path('/home/tim/Documents/GitHub/workspace')
root = Path('/tmp/hypercurve-fixed-distance-domains-qualification')
audit = workspace / 'hypercurve-api-review-2026-09-12'
prefix = 'finite-fixed-distance-domains'
parent = 'fedd740a91a89456b7e655db32f94dd8f702136c'
destination = audit / (prefix + '-qualification.json')
assert not destination.exists(), 'Do not overwrite a successful qualification.'
read = lambda suffix: json.loads((audit / (prefix + '-' + suffix + '.json')).read_text())
hashfile = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
working, isolated = read('source-before-tests'), read('isolated-sources')
for directory, rows in [(workspace, working), (root, isolated)]:
    for row in rows:
        assert hashfile(directory / row['file']) == row['sha256'], row['file']
toolchain = json.loads((audit / 'finite-parallel-pruning-toolchain.json').read_text())
for name, sha in toolchain['binary_sha256'].items():
    assert hashfile(Path(toolchain['direct_toolchain_directory']) / name) == sha, name
builds, focused, full = read('builds'), read('focused'), read('full-results')
assert len(builds) == 4 and len(focused) == 18 and len(full) == 49
assert all(row['returncode'] == 0 for row in builds + focused)
for row in focused:
    assert hashfile(row['binary']) == row['sha256']
    assert '1 passed; 0 failed;' in (audit / row['log']).read_text()
for row in full:
    assert not row['timed_out'] and not row['new_failures']
    assert row['summaries'] and row['returncode'] in [0, 101]
    assert row['returncode'] == 0 or row['known_failures']
    assert hashfile(row['binary']) == row['sha256'], row['binary']
known_failed = sorted(name for row in full for name in row['known_failures'])
assert known_failed == sorted(name for names in read('baseline-failures').values() for name in names)
items = [json.loads(line) for line in (audit / (prefix + '-test-build.jsonl')).read_text().splitlines()]
library = Path(next(f for x in items if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
library_sha = hashfile(library)
matrices = []
for path in sorted(audit.glob(prefix + '-*.json')):
    row = json.loads(path.read_text())
    if isinstance(row, dict) and 'normal_library_sha256' in row and 'source_sha256' in row and 'commit' not in row and 'queries' not in row:
        assert row['returncode'] == 0 and row['normal_library_sha256'] == library_sha, path.name
        assert hashfile(audit / row['source']) == row['source_sha256'], row['source']
        assert hashfile(audit / path.stem) == row['executable_sha256'], path.name
        matrices.append(path.name)
assert len(matrices) == 37, len(matrices)
baseline_path = audit / 'finite-fixed-distance-baseline.json'
baseline = json.loads(baseline_path.read_text())
assert baseline['commit'] == parent
assert sum(row['counts']['failures'] for row in baseline['queries']) == 16
assert all(row['returncode'] != 124 for row in baseline['queries'])
assert hashfile(audit / baseline['source']) == baseline['source_sha256']
assert hashfile(audit / baseline['executable']) == baseline['executable_sha256']
assert hashfile(baseline['normal_library']) == baseline['normal_library_sha256']
public = read('public')
assert public['normal_library_sha256'] == library_sha
assert hashfile(audit / public['source']) == public['source_sha256']
assert hashfile(audit / public['executable']) == public['executable_sha256']
assert len(public['queries']) == 8
assert all(row['returncode'] == 0 and row['counts']['failures'] == 0 for row in public['queries'])
assert sum(row['counts']['successes'] for row in public['queries']) == 32
assert sum(row['counts']['point_replays'] for row in public['queries']) == 176
changed = subprocess.check_output(['git', 'diff', '--name-only'], cwd=workspace / 'hypercurve', text=True).splitlines()
assert set(changed) == {'src/bezier_offset.rs', 'src/curve.rs', 'src/curve_region_boolean.rs', 'tests/hypercurve_path_closure.rs'}
command = [str(Path(toolchain['direct_toolchain_directory']) / 'rustfmt'), '--edition', '2024', '--config', 'skip_children=true', '--check', *changed]
with (audit / (prefix + '-format-check.log')).open('w') as out:
    result = subprocess.run(command, cwd=workspace / 'hypercurve', stdout=out, stderr=subprocess.STDOUT)
assert result.returncode == 0
subprocess.run(['git', 'diff', '--check'], cwd=workspace / 'hypercurve', check=True)
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=workspace / 'hypercurve', text=True).strip() == parent
assert sum(len(row['passed']) for row in full) == 2303
archive = audit / (prefix + '-libraries')
archive.mkdir()
shutil.copy2(library, archive / library.name)
libtest = next(Path(row['binary']) for row in full if row['repo'] == 'hypercurve' and row['target'] == 'lib')
shutil.copy2(libtest, archive / libtest.name)
report = dict(
    status='passed within the recorded broad qualification scope', goal_status='active',
    previous_commit=parent, commit=None, targets=len(full),
    passed=sum(len(row['passed']) for row in full),
    passed_by_repo={name: sum(len(row['passed']) for row in full if row['repo'] == name) for name in ['hypercurve', 'hyperbrep']},
    known_failed=known_failed, new_failures=[], ignored=sum(len(row['ignored']) for row in full),
    expensive_previously_unqualified_exclusions=8, timeouts_in_broad_run=0,
    builds=builds, focused_checks=len(focused), public_matrices_and_probes=matrices,
    parent_public_baseline=baseline_path.name,
    public_chart_cases=32, public_chart_point_replays=176,
    public_integration_cases=32, additional_public_chart_executions=8,
    attempts=[prefix + '-initial-attempt1'],
    computational_scope='Generic finite ranges replace unit substitution and endpoint promotion. Local center proofs replace native source admission. One polynomial-speed selector serves retained parameter kinds; ordinary algebraic centers project the unsquared polynomial relation when both speeds are polynomial, avoiding repeated radical replay. The first attempt timed out on this expanded regression; final qualification retains the regression. No general throughput or memory improvement claim.',
    source_files_verified=dict(working=len(working), isolated=len(isolated)),
    dependency_commits='finite-parallel-pruning-isolated-sources.json', isolated_dependency_sources=True,
    workspace_clean_claim=False, other_session_owns='Hyperreal verified/ and verification/ changes; left untouched',
    normal_library=str(library), normal_library_sha256=library_sha, archived_library=str(archive / library.name),
    libtest_sha256=hashfile(libtest), toolchain='finite-parallel-pruning-toolchain.json',
    evidence_audit=prefix + '-worklog.md',
    remaining='Selected-point finite incidence, one-sided source-cusp frames, normalized public-region construction, independent inverse replay, known failures/exclusions and broader algebraic/computational consolidation.',
)
destination.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({key: report[key] for key in ['status', 'passed', 'passed_by_repo', 'known_failed', 'source_files_verified', 'normal_library_sha256', 'libtest_sha256']}, indent=2))
