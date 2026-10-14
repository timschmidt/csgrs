"""Close the standalone MPFR experiment without extending either sanitizer run."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'normal-image-mpfr-cache-v1'
DESTINATION = E / 'normal-image-mpfr-cache-closure-v1.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def main():
    assert not DESTINATION.exists() and not (OUT / 'failure.json').exists()
    result = read(OUT / 'result.json')
    protocol = read(OUT / 'protocol.json')
    assert result['complete'] and result['commands_matching_expected_exit'] == 4
    assert all(sha(OUT / name) == digest for name, digest in result['artifacts_sha256'].items())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    rows = [read(OUT / (name + '.json')) for name in ['format', 'build', 'retain', 'cleanup']]
    assert [row['exit_code'] for row in rows] == [0, 0, 66, 0]
    for row in rows:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    stderr = (OUT / 'retain.stderr').read_text()
    assert 'ERROR: LeakSanitizer: detected memory leaks' in stderr
    assert 'ERROR: AddressSanitizer:' not in stderr and 'fatal error' not in stderr
    match = re.search(r'SUMMARY: AddressSanitizer: (\d+) byte\(s\) leaked in (\d+) allocation\(s\)', stderr)
    assert match and tuple(map(int, match.groups())) == (7744, 68)
    assert result['retain_leak_bytes'] == 7744 and result['retain_leak_allocations'] == 68
    blocks = [block for block in re.split(r'\n\n+', stderr) if re.search(r'leak of \d+ byte', block)]
    assert blocks and all('__gmp_default_' in block for block in blocks)
    assert (OUT / 'cleanup.stderr').read_bytes() == b''
    assert (OUT / 'retain.stdout').read_bytes() == (OUT / 'cleanup.stdout').read_bytes()
    assert sha(OUT / 'retain.stdout') == result['witness_sha256']
    roots = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    prior = E / 'sphere-normal-image-v1'
    source = Path(read(prior / 'protocol.json')['source'])
    mapping = json.loads(gzip.decompress((prior / 'source-hashes.json.gz').read_bytes()))
    assert len(roots) == 10129 and len(mapping) == 10136
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    assert all(sha(source / name) == digest for name, digest in mapping.items())
    for row in [read(OUT / 'binary.json'), *protocol['libraries'], *protocol['preserved_executed_binaries']]:
        assert sha(row['path']) == row['sha256']
    closure = {
        'complete': True, 'trial_closed': True, 'driver_phase_terminal_exit_codes': [0, 0],
        'commands_matching_expected_exit': 4, 'native_processes': 2,
        'retain_leak_bytes': 7744, 'retain_leak_allocations': 68,
        'cleanup_exit_code': 0, 'cleanup_stderr_empty': True,
        'numeric_witnesses_byte_identical': True, 'repository_crates_linked': False,
        'all_frozen_inputs_sources_libraries_and_executed_binaries_rechecked': True,
        'result_sha256': sha(OUT / 'result.json'), 'closure_reader_sha256': sha(__file__),
        'driver_logs_sha256': {name: sha(E / name) for name in [
            'normal-image-mpfr-cache-prepare-v1.stdout', 'normal-image-mpfr-cache-prepare-v1.stderr',
            'normal-image-mpfr-cache-compare-v1.stdout', 'normal-image-mpfr-cache-compare-v1.stderr']},
        'interpretation': 'Explicit worker-local MPFR cache cleanup removes the standalone GMP leak report while preserving numeric output. This establishes the mechanism for this control, not yet the cause of every allocation in the failed full-suite report.',
        'limits': result['limits'], 'runtime_admitted': False, 'plan_complete': False,
    }
    DESTINATION.write_text(json.dumps(closure, indent=2) + '\n')
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    base = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in ledger.items() if k != 'current_work'} == {
        k: v for k, v in base.items() if k != 'current_work'}
    entry = ledger['current_work']['sampled_sphere_hull']
    prefix = str(E.relative_to(ROOT)) + '/'
    entry.update({
        'status': 'normal-form-image-filter-mpfr-thread-cache-control-complete',
        'sphere_normal_image_mpfr_cache_scope': prefix + 'normal-image-mpfr-cache-scope-v1.json',
        'sphere_normal_image_mpfr_cache_protocol': prefix + 'probe_normal_image_mpfr_cache_v1.py',
        'sphere_normal_image_mpfr_cache_result': prefix + 'normal-image-mpfr-cache-v1/result.json',
        'sphere_normal_image_mpfr_cache_closure': prefix + DESTINATION.name,
        'sphere_normal_image_mpfr_cache_complete': True,
        'sphere_normal_image_mpfr_thread_cache_mechanism_demonstrated': True,
        'sphere_normal_image_full_suite_leak_cause_established': False,
        'sphere_normal_image_runtime_admitted': False, 'plan_complete': False,
    })
    ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
    print(json.dumps({'complete': True, 'trial_closed': True, 'closure_sha256': sha(DESTINATION)}))


if __name__ == '__main__':
    main()
