"""Sequential scalar qualification of the frozen scaling variants.

The unchanged runtime baseline first establishes that all four new regressions
test existing contracts. Each candidate then runs those tests, the six retained
cache frontiers, the full scalar suite, doctests, Clippy and warning-denied docs.
Focused test totals overlap the full suite and are not additive coverage.
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


def main():
    out = E / 'validation-v1'
    out.mkdir(exist_ok=False)
    preparation = json.loads((PREP / 'protocol.json').read_text())
    original = json.loads(gzip.decompress((PREP / 'parent-source-hashes.json.gz').read_bytes()))
    variants = preparation['variants']
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'RUSTDOCFLAGS': '-D warnings'}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'preparation_sha256': sha(PREP / 'protocol.json'),
                'environment': env, 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'variants': {key: {'source': row['source'], 'target': row['target']} for key, row in variants.items()},
                'command_timeout_seconds': 1800, 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results, tests = [], []

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
        assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in original.items())
        for row in variants.values():
            assert all(sha(Path(row['source']) / name) == digest
                       for name, digest in (original | row['source_changes_sha256']).items())

    def run(variant, label, command, passed=None):
        verify()
        name = variant + '-' + label
        directory = Path(variants[variant]['source'])
        extra = {'CARGO_TARGET_DIR': variants[variant]['target']}
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        command = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=directory, env=os.environ | env | extra,
                                  stdout=stdout, stderr=stderr).returncode
        result = {'name': name, 'variant': variant, 'phase': label, 'command': command,
                  'cwd': str(directory), 'extra_environment': extra, 'exit_code': code,
                  'seconds': time.monotonic() - started, 'stdout_sha256': sha(out / (name + '.stdout')),
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        results.append(result)
        (out / 'commands.json').write_text(json.dumps(results, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        verify()
        assert code == 0, name
        if passed is not None:
            raw = (out / (name + '.stdout')).read_text()
            counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
            assert counts == [(str(passed), '0', '0')], (name, counts)
            tests.append({'variant': variant, 'phase': label, 'passed': passed, 'failed': 0, 'ignored': 0})
            (out / 'tests.json').write_text(json.dumps(tests, indent=2) + '\n')

    scalar = ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features']
    for variant in ['baseline', 'right', 'both']:
        run(variant, 'format', ['cargo', 'fmt', '--all', '--', '--check'])
        run(variant, 'scaling-regressions', scalar + ['--lib', 'coefficient_scale_tests', '--', '--test-threads=1'], passed=4)
        if variant == 'baseline':
            continue
        run(variant, 'cache-frontiers', scalar + ['--lib', 'persistent_laurent_cache_tests', '--', '--test-threads=1'], passed=6)
        run(variant, 'strict-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings'])
        run(variant, 'scalar-library', scalar + ['--lib', '--', '--test-threads=2'], passed=867)
        run(variant, 'scalar-doctests', scalar + ['--doc'], passed=28)
        run(variant, 'scalar-docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps'])
    verify()
    result = {'complete': True, 'variants': ['baseline', 'right', 'both'], 'tests': tests,
              'focused_totals_overlap_full_suite': True, 'source_parent_and_main_unchanged': True,
              'phases': results, 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'candidate_full_scalar_results_each': 867,
                      'candidate_doctests_each': 28, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
