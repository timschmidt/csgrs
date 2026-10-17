"""Admit only the independently tested temporary-path repair."""
import gzip
import hashlib
import json
import re
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

prep = read(W / 'temp-preparation-v1/result.json')
assert prep['complete'] and not prep['cache_index_included']
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == prep['base_commit']
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
with gzip.open(W / 'temp-preparation-v1/main-input-hashes.json.gz', 'rt') as f:
    main = json.load(f)
assert all(sha(ROOT / p) == h for p, h in main.items())
reproduction = read(W / 'temp-reproduction-v1/result.json')
assert reproduction['expected_file_failures_reproduced'] == 5 and reproduction['selected_TMPDIR_writable_but_unused']
assert read(W / 'temp-validation-v3/result.json')['command_failures'] == ['compile-tests']
validation = read(W / 'temp-replay-v4/result.json')
assert validation['complete'] and validation['command_failures'] == [] and validation['selected_temporary_directory_cleaned']
commands = read(W / 'temp-validation-v3/commands.json')
assert all(next(r for r in commands if r['name'] == n)['exit_code'] == 0 for n in ['format', 'clippy'])
counts = {}
for name in ['conversion-tests', 'parquet-test', 'sqlite-test']:
    text = (W / 'temp-replay-v4' / (name + '.stdout')).read_text()
    count = re.findall(r'test result: ok\. (\d+) passed; 0 failed;', text)
    assert len(count) == 1
    counts[name] = int(count[0])
assert counts == {'conversion-tests': 18, 'parquet-test': 1, 'sqlite-test': 1}
reports = read(W / 'project-reports-v1/result.json')
assert reports['complete'] and reports['all_report_bytes_identical']
for folder in ['project-reports-v1', 'temp-reproduction-v1', 'temp-validation-v3', 'temp-replay-v4']:
    for row in read(W / folder / 'commands.json'):
        for suffix in ['stdout', 'stderr']:
            assert sha(W / folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        if 'resources_sha256' in row:
            assert sha(W / folder / (row['name'] + '.resources')) == row['resources_sha256']
with gzip.open(W / 'temp-preparation-v1/source-hashes.json.gz', 'rt') as f:
    final = json.load(f)
S = W / 'temp-candidate-source-v1'
assert all(sha(S / p) == h for p, h in final.items())
for p in prep['changes']:
    assert (ROOT / p).read_text().split('#[cfg(test)]', 1)[0] == (S / p).read_text().split('#[cfg(test)]', 1)[0]
decision = {
    'status': 'kept: file-output tests honor selected temporary storage; cache report bytes preserved but cache remains unadmitted',
    'progress_classification': 'progress', 'main_parent_commit': prep['base_commit'],
    'owning_layer': 'HyperDRC test harness', 'properties': ['P20', 'P27', 'P35', 'P37'],
    'implementation_sha256': prep['changes'], 'qualified_source': str(S),
    'production_prefixes_unchanged': True, 'assertions_and_fixture_contents_preserved': True,
    'input_directory_collision_removed': 'Successful conversion and directory-creation-error tests now use distinct input directory names in one test process.',
    'reproduced_failures': 5, 'unchanged_control_passed': 1, 'passing_tests': counts,
    'strict_clippy': next(r['command'] for r in commands if r['name'] == 'clippy'),
    'cleanup_verified': True, 'cache_index_included': False, 'cache_index_admitted': False,
    'native_report_comparison': reports['reports'],
    'prepared_harness_failures_retained': [
        'v1 saw both CSGRS rlib and shared library in the artifact list before any build',
        'v2 saw zip 2.4.2 and zip 7.2.0 before any build; v3 resolves direct dependencies from Cargo.lock',
        'v3 direct build omitted CARGO_PKG_AUTHORS; strict Cargo Clippy passed; v4 supplies Cargo package metadata and all tests pass on unchanged sources',
    ],
    'limits': [
        'No production geometry, predicate, report-format or API change.',
        'Converter test stubs still use /bin/true; this repair does not establish all-operating-system support.',
        'E38 native failures remain recorded failures; the complete indexed candidate still needs qualification with the independently validated test fix.',
        'Report equality is scoped to the complete CPArti and HVP109A projects at the same fixed paths; it is not an analytic source-fidelity proof or timing claim.',
        'Full curve/modeling/circuit/binding qualification, matched performance and published-package validation remain open.',
        'The /tmp quota is still exhausted; repository-local lint validation retained a 768 MiB free-space reserve and did not hit it.',
        'Inherited audit remains incomplete with 27,093 missing paths; all original property and future-work obligations remain active.',
    ],
    'next': [
        'Requalify the indexed candidate with the admitted temporary-directory repair, using fresh build targets or verified read-only dependency reuse.',
        'Complete remaining Hypercurve, CSGRS, Hypercircuit and actual binding checks and matched cost evidence before deciding cache retention.',
        'Preserve the rejected exact-track importer until its own unchanged workload and migration contracts pass.',
    ],
    'full_archived_integration_admitted': False, 'native_runtime_admitted': False, 'full_plan_complete': False,
}
dump(W / 'decision-v1.json', decision)
(W / 'progress-v1.md').write_text("""# Temporary test storage and report recovery

The file-output tests now honor the platform temporary-directory selection.
Conversion success and directory-error cases also use distinct input directory
names, so concurrent tests do not remove one another's fixture.

The current test bodies reproduced five failures while a writable TMPDIR was
selected; they wrote to hard-coded /tmp paths instead. The repair changes only
test paths, their cleanup and two unused test imports. Production prefixes,
assertions, fixture contents, converter arguments and report readback remain
unchanged. The converter stub still uses /bin/true; broader operating-system
support is not claimed.

All 18 conversion tests pass with four test threads, and the Parquet and SQLite
readback tests pass. The selected directory is clean afterward. Strict Clippy
passes for all HyperDRC features and targets, and formatting passes. Source-matched
main dependencies were reused read-only; the cache index was not included in this
test-only repair. Two dependency-selection attempts and a missing Cargo metadata
environment are preserved separately from the successful final test build.

The previously successful E38 native build already contained a complete CLI.
Using that pinned executable, complete CPArti and HVP109A reports were compared
with the verified baseline at the same original project paths. JSON values,
ordering, serialized bytes, sizes and checksums are identical: 12 and 201 findings,
respectively. This closes that scoped report check, not cache admission, source
fidelity, a performance claim or the full user-path contract.

The cache proposal remains unadmitted. Its earlier quota failures remain failures;
full affected-crate and binding validation and matched cost evidence are still
required. The per-user /tmp quota remains full. A small fresh lint target fit in
repository storage with a monitored 768 MiB reserve; no completed target was
reopened. The original 35-file disposition and rejected exact-track integration
remain intact, and all broader plan obligations stay active. The inherited artifact
audit still has 27,093 missing paths.
""")
print(json.dumps({'test_only_change_kept': True, 'passing_tests': counts, 'reports_byte_identical': True, 'cache_admitted': False}), flush=True)
