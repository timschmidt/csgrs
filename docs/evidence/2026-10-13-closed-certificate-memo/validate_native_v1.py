"""Qualify the frozen closed-certificate memo candidate with native consumers, sequentially.

The candidate gets a new native Cargo target. Full all-feature solver, curve and DRC
tests include integration tests and doctests. The two original expensive DRC
limits are skipped only in the combined command and executed individually from
the bound test executable. Their original fixtures and assertions are intact.
Gate failures are recorded, never turned into skips or larger limits.
"""
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
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from qualify_v1 import verify as verify_sources


def main():
    out = E / 'native-validation-v1'
    out.mkdir(exist_ok=False)
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    decision = json.loads((E / 'scalar-decision-v1.json').read_text())
    assert decision['decision'] == 'advance-closed-certificate-memo-to-native-investigation-with-scalar-regressions-open'
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'RUSTDOCFLAGS': '-D warnings'}
    variants = {name: {'source': row['source'], 'target': str(Path(row['source']).parent / 'native-target')}
                for name, row in preparation['variants'].items() if name in ['candidate']}
    assert list(variants) == ['candidate']
    assert not Path(variants['candidate']['target']).exists()
    gates = [
        {'label': 'full-board', 'test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
         'limit_seconds': 120, 'metric': 'worker CPU'},
        {'label': 'copper-neck', 'test': 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
         'limit_seconds': 60, 'metric': 'check body wall time'},
    ]
    skips = [arg for gate in gates for arg in ['--skip', gate['test']]]
    commands = [
        ('strict-stack-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '-p', 'hypersolve', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings']),
        ('solver-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypersolve', '--all-features', '--', '--test-threads=2']),
        ('endpoint-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', 'endpoint_proof_tests', '--', '--test-threads=1']),
        ('curve-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--', '--test-threads=2']),
        ('drc-tests-except-separate-gates', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--', '--test-threads=2', *skips]),
        ('stack-docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '-p', 'hypersolve', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--no-deps']),
        ('build-board-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json']),
    ]
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'assessor_sha256': sha(E / 'assess_native_v1.py'),
                'preparation_sha256': sha(E / 'preparation-v1/protocol.json'),
                'scalar_decision_sha256': sha(E / 'scalar-decision-v1.json'),
                'cost_assessment_sha256': sha(E / 'cost-assessment-v1.json'),
                'scalar_correctness_sha256': sha(E / 'correctness-v1.json'),
                'variants': variants, 'variant_order': ['candidate'],
                'commands': commands, 'gates': gates, 'environment': env,
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'command_timeout_seconds': 1800,
                'source_format_already_qualified': 'qualification-v1/result.json and correctness-v1.json; source remains frozen',
                'gate_duration_note': 'Whole subprocess durations include setup and are not either internal assertion metric. A passing copper-neck assertion does not print its check-body time.',
                'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results, tests, binaries = [], [], {}

    def verify():
        verify_sources(preparation)
        assert sha(E / 'correctness-v1.json') == protocol['scalar_correctness_sha256']
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'assess_native_v1.py') == protocol['assessor_sha256']
        assert sha(E / 'cost-assessment-v1.json') == protocol['cost_assessment_sha256']
        assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
        assert sha(E / 'scalar-decision-v1.json') == protocol['scalar_decision_sha256']
        assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())

    def run(variant, label, command, allow_failure=False):
        verify()
        name = variant + '-' + label
        directory = Path(variants[variant]['source'])
        extra = {'CARGO_TARGET_DIR': variants[variant]['target']}
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(wrapped, cwd=directory, env=os.environ | env | extra,
                                  stdout=stdout, stderr=stderr).returncode
        result = {'name': name, 'variant': variant, 'phase': label, 'command': wrapped,
                  'cwd': str(directory), 'extra_environment': extra, 'exit_code': code,
                  'seconds': time.monotonic() - started, 'stdout_sha256': sha(out / (name + '.stdout')),
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        results.append(result)
        (out / 'commands.json').write_text(json.dumps(results, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        verify()
        assert code == 0 or (allow_failure and code == 101), name
        raw = (out / (name + '.stdout')).read_text()
        if label in ['solver-tests', 'endpoint-regressions', 'curve-tests', 'drc-tests-except-separate-gates']:
            counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
            assert counts and all(int(failed) == 0 for _, failed, _ in counts), name
            tests.append({'variant': variant, 'phase': label,
                          'passed': sum(int(passed) for passed, _, _ in counts), 'failed': 0,
                          'ignored': sum(int(ignored) for _, _, ignored in counts),
                          'result_groups': len(counts)})
            expected = {'solver-tests': (974, 0, 11), 'endpoint-regressions': (5, 0, 1),
                        'curve-tests': (2494, 33, 51), 'drc-tests-except-separate-gates': (1644, 0, 10)}[label]
            assert tuple(tests[-1][k] for k in ['passed', 'ignored', 'result_groups']) == expected, tests[-1]
            (out / 'tests.json').write_text(json.dumps(tests, indent=2) + '\n')
        return raw

    for variant in protocol['variant_order']:
        for label, command in commands:
            log = run(variant, label, command)
        events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
        artifacts = [event for event in events if event.get('reason') == 'compiler-artifact'
                     and event.get('executable') and event['target']['name'] == 'hyperdrc']
        assert len(artifacts) == 1
        artifact = artifacts[0]
        assert Path(artifact['manifest_path']) == Path(variants[variant]['source']) / 'crates/hyperdrc/Cargo.toml'
        assert artifact['target']['kind'] == ['lib'] and artifact['profile']['test']
        binary = Path(artifact['executable'])
        assert binary.is_relative_to(Path(variants[variant]['target']))
        binaries[variant] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                             'manifest_path': artifact['manifest_path'], 'features': artifact['features'],
                             'package_id': artifact['package_id'],
                             'target': artifact['target'], 'profile': artifact['profile']}
        (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
        for gate in gates:
            log = run(variant, gate['label'], ['/usr/bin/time', '-v', str(binary), gate['test'],
                      '--exact', '--test-threads=1', '--nocapture'], allow_failure=True)
            both = log + (out / (variant + '-' + gate['label'] + '.stderr')).read_text()
            assert 'skipping complex project fixture test' not in both
            assert '1 passed' in log or '1 failed' in log
            if gate['label'] == 'full-board':
                assert both.count('hyperdrc: finished emit report') == 2
    verify()
    result = {'complete': True, 'all_passed': all(row['exit_code'] == 0 for row in results),
              'tests': tests, 'phases': results, 'source_parent_main_and_binaries_unchanged': True,
              'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'all_passed': result['all_passed'], 'tests': tests,
                      'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
