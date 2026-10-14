"""Assess the completed main-source qualification and admit only its test repair."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'mpfr-test-cleanup-main-v1'
DESTINATION = E / 'mpfr-test-cleanup-main-admission-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def mapping(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not DESTINATION.exists() and not (OUT / 'failure.json').exists()
    result = read(OUT / 'result.json')
    protocol = read(OUT / 'protocol.json')
    scope = read(E / 'mpfr-test-cleanup-main-scope-v1.json')
    assert result['complete'] and result['commands_all_zero'] == 5
    assert not result['unadmitted_scalar_or_mesh_runtime_prototypes_included']
    assert all(sha(OUT / name) == digest for name, digest in result['artifacts_sha256'].items())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    commands = read(OUT / 'commands.json')
    assert [row['name'] for row in commands] == [
        'format', 'default-clippy', 'all-features-clippy', 'default-tests', 'all-features-tests']
    assert [row['exit_code'] for row in commands] == [0] * 5
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    sources = mapping(OUT / 'source-hashes.json.gz')
    roots = mapping(OUT / 'root-source-hashes.json.gz')
    source = Path(protocol['source'])
    baseline = mapping(E / 'mesh-identity-preparation-v1/source-hashes.json.gz')
    delta = scope['expected_delta_sha256']
    assert len(delta) == 13 and len(sources) == 10130 and len(roots) == 10129
    assert sources == baseline | delta | scope['candidate_ledger_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    assert all(sha(source / name) == digest for name, digest in sources.items())
    prior = E / 'normal-image-test-cleanup-v1'
    prior_protocol = read(prior / 'protocol.json')
    prior_source = Path(prior_protocol['source'])
    parent = Path(prior_protocol['parent_source'])
    for base, table in [
        (prior_source, mapping(prior / 'source-hashes.json.gz')),
        (parent, mapping(E / 'sphere-normal-image-v1/source-hashes.json.gz')),
    ]:
        assert all(sha(base / name) == digest for name, digest in table.items())
    assert all(sha(prior_source / name) == digest for name, digest in delta.items())
    binaries = read(OUT / 'binaries.json')
    for row in [*protocol['preserved_libraries_and_binaries'], *binaries.values()]:
        assert sha(row['path']) == row['sha256']
    counts = {}
    for name in ['default-tests', 'all-features-tests']:
        stdout = (OUT / (name + '.stdout')).read_text()
        stderr = (OUT / (name + '.stderr')).read_text()
        rows = [tuple(map(int, row)) for row in re.findall(
            r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', stdout, re.MULTILINE)]
        assert rows and all(row[1] == row[3] == row[4] == 0 for row in rows)
        counts[name] = dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered_out'], map(sum, zip(*rows))))
        counts[name]['completed_test_summaries'] = len(rows)
        assert 'ERROR: AddressSanitizer:' not in stderr and 'LeakSanitizer' not in stderr
        for path in re.findall(r'Running .* \(([^()\n]+)\)', stderr):
            row = binaries[name + '/' + Path(path).name]
            assert row['path'] == path and sha(path) == row['sha256']
    assert counts == result['test_counts']
    pending = read(ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json')['pending_native_sha256']
    assert len(pending) == 35 and not set(pending).intersection(delta)
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    old_ledger = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in ledger.items() if k != 'current_work'} == {
        k: v for k, v in old_ledger.items() if k != 'current_work'}
    # The complete relevant main-runtime test suites passed before this copy.
    # Existing target files were frozen as root preimages; only the helper is new.
    for name in delta:
        if name in roots:
            assert sha(ROOT / name) == baseline[name]
        else:
            assert not (ROOT / name).exists()
    DESTINATION.mkdir()
    (DESTINATION / 'source-hashes-before.json.gz').write_bytes((OUT / 'root-source-hashes.json.gz').read_bytes())
    for name in delta:
        path = ROOT / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((source / name).read_bytes())
    prefix = str(E.relative_to(ROOT)) + '/'
    entry = ledger['current_work']['sampled_sphere_hull']
    entry.update({
        'status': 'test-oracle-cleanup-admitted-runtime-prototypes-still-isolated',
        'sphere_normal_image_test_cleanup_admitted': True,
        'mpfr_test_cleanup_main_qualification_scope': prefix + 'mpfr-test-cleanup-main-scope-v1.json',
        'mpfr_test_cleanup_main_qualification_result': prefix + 'mpfr-test-cleanup-main-v1/result.json',
        'mpfr_test_cleanup_main_qualification_complete': True,
        'mpfr_test_cleanup_main_qualification_counts': counts,
        'mpfr_test_cleanup_main_admission': prefix + 'mpfr-test-cleanup-main-admission-v1/admission.json',
        'mpfr_test_cleanup_main_source_mapping': prefix + 'mpfr-test-cleanup-main-admission-v1/source-hashes-after.json.gz',
        'sphere_normal_image_runtime_admitted': False,
        'sphere_stable_selection_runtime_admitted': False,
        'sphere_certificate_runtime_admitted': False, 'plan_complete': False,
    })
    dump(ledger_path, ledger)
    after = roots | delta | {'docs/properties-status.json': sha(ledger_path)}
    assert len(after) == 10130
    assert all(sha(ROOT / name) == digest for name, digest in after.items())
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    (DESTINATION / 'source-hashes-after.json.gz').write_bytes(gzip.compress(
        (json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
    admission = {
        'complete': True, 'main_qualification_trial_closed': True,
        'main_qualification_driver_terminal_exit_code': 0,
        'commands_all_zero': 5, 'test_counts': counts,
        'test_cleanup_admitted': True, 'admitted_files_sha256': delta,
        'admitted_scope': 'Private MPFR test-thread cache cleanup at 31 oracle entry points, its shared test-only helper, and a nonleaking dispatch-label test fixture.',
        'production_arithmetic_geometry_and_public_api_changed': False,
        'all_other_mapped_root_files_unchanged_except_current_work_ledger': True,
        'pending_native_files_preserved': 35,
        'source_mapping_before_sha256': sha(DESTINATION / 'source-hashes-before.json.gz'),
        'source_mapping_after_sha256': sha(DESTINATION / 'source-hashes-after.json.gz'),
        'qualification_result_sha256': sha(OUT / 'result.json'),
        'prototype_qualification_closure_sha256': sha(E / 'normal-image-test-cleanup-closure-v1.json'),
        'admission_script_sha256': sha(__file__),
        'driver_stdout_sha256': sha(E / 'mpfr-test-cleanup-main-driver-v1.stdout'),
        'driver_stderr_sha256': sha(E / 'mpfr-test-cleanup-main-driver-v1.stderr'),
        'limits': scope['limits'], 'unadmitted_runtime_prototypes_admitted': False,
        'plan_complete': False,
    }
    dump(DESTINATION / 'admission.json', admission)
    print(json.dumps({'test_cleanup_admitted': True, 'admitted_files': len(delta),
                      'test_counts': counts, 'admission_sha256': sha(DESTINATION / 'admission.json')}))


if __name__ == '__main__':
    main()
