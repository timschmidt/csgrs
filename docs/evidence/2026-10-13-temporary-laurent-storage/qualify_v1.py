"""Sequential qualification of four matched temporary-storage sources.

All variants execute the same checks in separate fresh targets. Default and
all-feature suites, focused controls and doctests overlap; counts are not added
as unique coverage. No native admission or resource claim follows from this run.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PREP = E / 'preparation-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def verify(preparation):
    assert sha(E / 'prepare_v1.py') == preparation['runner_sha256']
    assert sha(E / 'qualify_v1.py') == preparation['qualification_runner_sha256']
    assert sha(E / 'temporary_storage_tests.rs') == preparation['test_input_sha256']
    assert sha(PREP / 'parent-source-hashes.json.gz') == preparation['parent_hashes_sha256']
    original = json.loads(gzip.decompress((PREP / 'parent-source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(preparation['parent_source']) / p) == h for p, h in original.items())
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / p) == h for p, h in preparation['pending_native_sha256'].items())
    for variant, row in preparation['variants'].items():
        hashes = PREP / (variant + '-source-hashes.json.gz')
        assert sha(hashes) == row['source_hashes_sha256']
        current = json.loads(gzip.decompress(hashes.read_bytes()))
        assert current == original | row['source_changes_sha256']
        assert all(sha(Path(row['source']) / p) == h for p, h in current.items())


def main():
    preparation = json.loads((PREP / 'protocol.json').read_text())
    variants = preparation['variants']
    assert all(not Path(row['target']).exists() for row in variants.values())
    out = E / 'qualification-v1'
    out.mkdir(exist_ok=False)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'RUSTDOCFLAGS': '-D warnings'}
    scalar = ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features']
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check'], None),
        ('storage-regressions', scalar + ['--lib', 'temporary_storage_tests', '--', '--test-threads=1', '--nocapture'], [8]),
        ('cache-frontiers', scalar + ['--lib', 'persistent_laurent_cache_tests', '--', '--test-threads=1'], [6]),
        ('sign-regressions', scalar + ['--lib', 'quadratic_tower_sign_retention_tests', '--', '--test-threads=1'], [11]),
        ('all-feature-library', scalar + ['--lib', '--', '--test-threads=2'], [882]),
        ('default-library', ['cargo', 'test', '--locked', '-p', 'hyperreal', '--lib', '--', '--test-threads=2'], [795]),
        ('doctests', scalar + ['--doc'], [28]),
        ('all-feature-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings'], None),
        ('default-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperreal', '--all-targets', '--', '-D', 'warnings'], None),
        ('docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps'], None),
    ]
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'preparation_sha256': sha(PREP / 'protocol.json'), 'commands_per_variant': commands,
                'variant_order': list(variants), 'environment': env,
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'command_timeout_seconds': 1800, 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    phases, tests, layouts, feature_sets = [], [], {}, {}
    for variant, row in variants.items():
        for label, command, expected in commands:
            verify(preparation)
            assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
            name = variant + '-' + label
            extra = {'CARGO_TARGET_DIR': row['target']}
            wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
            print(json.dumps({'start': name}), flush=True)
            started = time.monotonic()
            with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
                code = subprocess.run(wrapped, cwd=row['source'], env=os.environ | env | extra, stdout=stdout, stderr=stderr).returncode
            phase = {'name': name, 'variant': variant, 'phase': label, 'command': wrapped, 'cwd': row['source'],
                     'extra_environment': extra, 'exit_code': code, 'seconds': time.monotonic() - started,
                     'stdout_sha256': sha(out / (name + '.stdout')), 'stderr_sha256': sha(out / (name + '.stderr'))}
            phases.append(phase)
            (out / 'commands.json').write_text(json.dumps(phases, indent=2) + '\n')
            print(json.dumps({'name': name, 'exit_code': code, 'seconds': phase['seconds']}), flush=True)
            verify(preparation)
            assert code == 0, name
            raw = (out / (name + '.stdout')).read_text()
            if expected is not None:
                groups = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
                assert groups == [(str(n), '0', '0') for n in expected], (name, groups)
                tests.append({'variant': variant, 'phase': label, 'passed': sum(expected), 'failed': 0, 'ignored': 0})
                (out / 'tests.json').write_text(json.dumps(tests, indent=2) + '\n')
            if label == 'storage-regressions':
                values = re.findall(r'TEMPORARY_LAURENT_LAYOUT\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)', raw)
                assert len(values) == 1
                layouts[variant] = dict(zip(['Rational', 'RadicalCoefficient', 'PiLaurentPolynomial', 'ClosedRadicalCoefficient', 'ClosedPiLaurentPolynomial', 'Computable', 'Node'], map(int, values[0])))
                (out / 'layouts.json').write_text(json.dumps(layouts, indent=2) + '\n')
        # Record resolved dependency features from the actual fresh builds.
        fingerprints = sorted(Path(row['target']).glob('*/.fingerprint/smallvec-*/lib-smallvec.json'))
        assert fingerprints
        entries = [{'path': str(p), 'sha256': sha(p), 'features': json.loads(json.loads(p.read_text())['features'])} for p in fingerprints]
        feature_sets[variant] = entries
        assert all(e['features'] == [] for e in entries), entries
        (out / 'smallvec-features.json').write_text(json.dumps(feature_sets, indent=2) + '\n')
    persistent = ['ClosedRadicalCoefficient', 'ClosedPiLaurentPolynomial', 'Computable', 'Node']
    assert all(all(row[k] == layouts['baseline'][k] for k in persistent) for row in layouts.values())
    verify(preparation)
    result = {'complete': True, 'phases': phases, 'tests': tests, 'layouts': layouts,
              'smallvec_features': feature_sets, 'source_parent_and_main_unchanged': True,
              'focused_and_configuration_totals_overlap': True,
              'runtime_admitted': False, 'plan_complete': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'variants': list(variants), 'layouts': layouts}), flush=True)


if __name__ == '__main__':
    main()
