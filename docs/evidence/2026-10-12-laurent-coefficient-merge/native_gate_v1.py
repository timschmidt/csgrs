"""Early native consumer and original full-board gates for owned coefficient merging.

The source-frozen candidate keeps its own Cargo target directory. These gates
are diagnostic qualification, not full native correctness or matched application
performance evidence. Both original full-board fixtures and limits are intact.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/laurent-coefficient-merge'
SOURCE = WORK / 'source'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'native-gate-v1'
    out.mkdir(exist_ok=False)
    preparation = json.loads((E / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    delta = json.loads((E / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    current = original | delta
    cost = json.loads((E / 'cost-v1/result.json').read_text())
    assert cost['complete'] and cost['source_unchanged'] and cost['mathematical_witnesses_identical']
    binaries = json.loads((E / 'cost-v1/binaries.json').read_text())
    assert binaries['libraries']['baseline']['sha256'] != binaries['libraries']['candidate']['sha256']
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
           'CARGO_TARGET_DIR': str(WORK / 'target')}
    commands = [
        ('strict-consumer-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings']),
        ('curve-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--test', 'native_linear_stroke', '--test', 'contour_area_normal_form', '--test', 'ray_filter_extreme_coordinates', '--', '--test-threads=1']),
        ('drc-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--test', 'native_linear_strokes', '--', '--test-threads=1']),
        ('build-board-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json']),
    ]
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'preparation_sha256': sha(E / 'preparation.json'),
                'candidate_changes_sha256': delta, 'cost_evidence_sha256': sha(E / 'cost-v1/result.json'),
                'scalar_assessment_sha256': sha(E / 'scalar-assessment-v1.json'),
                'environment': env, 'commands': commands,
                'gates': [
                    {'test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', 'limit_seconds': 120, 'metric': 'worker CPU'},
                    {'test': 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers', 'limit_seconds': 60, 'metric': 'check wall time'},
                ], 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert all(sha(SOURCE / p) == expected for p, expected in current.items())

    def run(name, command, allow_failure=False):
        verify()
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | env,
                                  stdout=stdout, stderr=stderr).returncode
        verify()
        result = {'name': name, 'command': command, 'exit_code': code, 'seconds': time.monotonic() - started,
                  'sources_unchanged': True, 'stdout_sha256': sha(out / (name + '.stdout')),
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        results.append(result)
        (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': False}, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        if code and not allow_failure:
            raise SystemExit(code)
        return (out / (name + '.stdout')).read_text()

    for name, command in commands:
        log = run(name, command)
    events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
    artifacts = [e for e in events if e.get('reason') == 'compiler-artifact' and e.get('executable') and e['target']['name'] == 'hyperdrc']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(WORK / 'target')
    identity = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                'manifest_path': artifact['manifest_path'], 'package_id': artifact['package_id']}
    (out / 'binary.json').write_text(json.dumps(identity, indent=2) + '\n')
    for label, gate in zip(['full-board', 'copper-neck'], protocol['gates']):
        assert sha(binary) == identity['sha256']
        log = run(label, ['/usr/bin/time', '-v', str(binary), gate['test'], '--exact', '--test-threads=1', '--nocapture'], allow_failure=True)
        both_streams = log + (out / (label + '.stderr')).read_text()
        assert 'skipping complex project fixture test' not in both_streams
        assert '1 passed' in log or '1 failed' in log
        if label == 'full-board':
            assert both_streams.count('hyperdrc: finished emit report') == 2
    (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': True,
                                               'all_passed': all(r['exit_code'] == 0 for r in results),
                                               'runtime_admitted': False}, indent=2) + '\n')


if __name__ == '__main__':
    main()
