"""Observe existing coordinate proofs during unchanged endpoint queries.

The previous goal turn made progress through five commits. This diagnostic
forks the frozen metadata-first candidate, preserves all original math queries,
and observes only their returned facts. Timers/counters perturb execution and
retain no geometry or scalar owners. Counters are collected from exiting threads.
"""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-12-region-metadata-first'
BASE = ROOT / 'target/region-metadata-first/source'
WORK = ROOT / 'target/endpoint-coordinate-facts/diagnostic-v1'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'diagnostic-v1'
    out.mkdir(exist_ok=False)
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    baseline = original | json.loads((PARENT / 'preparation.json').read_text())['source_changes_sha256']
    guard = json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    assert sources() == guard
    assert all(sha(BASE / name) == digest for name, digest in baseline.items())
    WORK.mkdir(parents=True, exist_ok=False)
    with (out / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE), str(SOURCE)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)

    def replace(name, before, after):
        path = SOURCE / name
        text = path.read_text()
        assert text.count(before) == 1, name
        path.write_text(text.replace(before, after))

    lib = 'crates/hypercurve/src/lib.rs'
    point = 'crates/hypercurve/src/point.rs'
    segment = 'crates/hypercurve/src/segment.rs'
    helper = 'crates/hypercurve/src/endpoint_fact_diagnostic.rs'
    replace(lib, 'mod events;', 'mod events;\nmod endpoint_fact_diagnostic;')
    replace(point,
            '        Arc::ptr_eq(&self.0, &other.0)\n'
            '            || ((self.x() - other.x()).zero_status() == ZeroKnowledge::Zero\n'
            '                && (self.y() - other.y()).zero_status() == ZeroKnowledge::Zero)',
            '        use crate::endpoint_fact_diagnostic as diagnostic;\n'
            '        diagnostic::point_result(\n'
            '            Arc::ptr_eq(&self.0, &other.0)\n'
            '                || (diagnostic::zero(0, (self.x() - other.x()).zero_status()) == ZeroKnowledge::Zero\n'
            '                    && diagnostic::zero(1, (self.y() - other.y()).zero_status()) == ZeroKnowledge::Zero)\n'
            '        )')
    replace(segment,
            '        if point == self.start() || point == self.end() {\n'
            '            return Some(true);\n'
            '        }\n'
            '        let at_start = points_equal(point, self.start(), policy);\n'
            '        if at_start == Some(true) {\n'
            '            return Some(true);\n'
            '        }\n'
            '        match points_equal(point, self.end(), policy) {\n'
            '            Some(true) => Some(true),\n'
            '            Some(false) if at_start == Some(false) => Some(false),\n'
            '            Some(false) | None => None,\n'
            '        }',
            '        use crate::endpoint_fact_diagnostic as diagnostic;\n'
            '        diagnostic::observe(policy, || {\n'
            '            if diagnostic::run(0, || point == self.start())\n'
            '                || diagnostic::run(1, || point == self.end()) {\n'
            '                return Some(true);\n'
            '            }\n'
            '            let at_start = diagnostic::policy_query(2, || points_equal(point, self.start(), policy));\n'
            '            if at_start == Some(true) {\n'
            '                return Some(true);\n'
            '            }\n'
            '            match diagnostic::policy_query(3, || points_equal(point, self.end(), policy)) {\n'
            '                Some(true) => Some(true),\n'
            '                Some(false) if at_start == Some(false) => Some(false),\n'
            '                Some(false) | None => None,\n'
            '            }\n'
            '        })')
    shutil.copyfile(E / 'endpoint_fact_diagnostic.rs', SOURCE / helper)
    with (out / 'prepare-format.log').open('w') as log:
        subprocess.run(['rustfmt', '--edition', '2024', str(SOURCE / lib)], stdout=log,
                       stderr=subprocess.STDOUT, check=True)
    changed = [lib, point, segment, helper]
    current = {name: sha(SOURCE / name) for name in sorted(set(baseline) | {helper})}
    assert {name for name, digest in current.items() if baseline.get(name) != digest} == set(changed)
    with tarfile.open(out / 'instrumentation.tar.gz', 'w:gz') as archive:
        for name in changed:
            archive.add(SOURCE / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (BASE / name).read_text().splitlines(True) if name in baseline else [],
        (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
    (out / 'instrumentation.patch').write_text(patch)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
    test = ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib']
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check']),
        ('diagnostic-regressions', test + ['endpoint_fact_diagnostic::tests', '--', '--test-threads=1']),
        ('point-equality-regressions', test + ['point::equality_tests', '--', '--test-threads=1']),
        ('arc-membership-regression', test + ['segment::tests::sweep_membership_uses_a_decisive_half_plane', '--', '--exact', '--test-threads=1']),
        ('strict-curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--all-targets', '--', '-D', 'warnings']),
        ('build-board-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json']),
    ]
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'parent_preparation_sha256': sha(PARENT / 'preparation.json'),
        'profile_assessment_sha256': sha(ROOT / 'docs/evidence/2026-10-12-region-metadata-profile/assessment-v1.json'),
        'source_changes_sha256': {name: current[name] for name in changed},
        'archive_sha256': sha(out / 'instrumentation.tar.gz'), 'patch_sha256': sha(out / 'instrumentation.patch'),
        'commands': commands, 'environment': env,
        'policy_bits': ['edge preview active', 'approximate terminal permitted', 'bounded exact pass active'],
        'proof_codes': ['unvisited', 'equal', 'proved unequal', 'inconclusive false'],
        'result_codes': ['Unknown', 'false', 'true', 'unvisited'],
        'row_contract': 'One row per endpoint per completed contains_endpoint call; key is policy/endpoint/initial proof/repeated proof/policy result/overall result. Both original initial comparisons and policy-based retries keep their order and short circuits. Only already returned coordinate zero facts are observed. Initial timers surround the initial Point2 comparisons; retry timers surround the complete existing points_equal call.',
        'policy_limit': 'Permission flags do not establish actual approximation consumption. No receipt is inferred from them.',
        'unchanged_worker_limit_seconds': 120, 'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    phases = []

    def verify():
        assert sources() == guard
        assert all(sha(BASE / name) == digest for name, digest in baseline.items())
        assert all(sha(SOURCE / name) == digest for name, digest in current.items())
        assert sha(Path(__file__)) == protocol['runner_sha256']

    def run(name, command, allow_failure=False):
        verify()
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | env, stdout=stdout, stderr=stderr).returncode
        verify()
        row = {'name': name, 'command': command, 'exit_code': code, 'seconds': time.monotonic() - started,
               'sources_unchanged': True, 'stdout_sha256': sha(out / (name + '.stdout')),
               'stderr_sha256': sha(out / (name + '.stderr'))}
        phases.append(row)
        (out / 'result.json').write_text(json.dumps({'phases': phases, 'complete': False}, indent=2) + '\n')
        print(json.dumps(row), flush=True)
        if code and not allow_failure:
            raise SystemExit(code)
        return (out / (name + '.stdout')).read_text()

    for name, command in commands:
        log = run(name, command)
    events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
    artifacts = [event for event in events if event.get('reason') == 'compiler-artifact'
                 and event.get('executable') and event['target']['name'] == 'hyperdrc']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(TARGET)
    (out / 'binary.json').write_text(json.dumps({'path': str(binary), 'sha256': sha(binary),
                                               'bytes': binary.stat().st_size, 'features': artifact['features'],
                                               'manifest_path': artifact['manifest_path']}, indent=2) + '\n')
    log = run('full-board', ['/usr/bin/time', '-v', str(binary),
                            'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
                            '--exact', '--test-threads=1', '--nocapture'], allow_failure=True)
    stderr = (out / 'full-board.stderr').read_text()
    assert 'skipping complex project fixture test' not in log + stderr
    assert stderr.count('hyperdrc: finished emit report') == 2
    assert '1 passed' in log or '1 failed' in log
    rows = []
    for line in stderr.splitlines():
        if line.startswith('ENDPOINT_FACT_DIAGNOSTIC '):
            key, calls, initial, retry = map(int, line.split()[1:])
            overall = key % 3; key //= 3
            policy_result = key % 4; key //= 4
            repeated = key % 4; key //= 4
            first = key % 4; key //= 4
            endpoint = key % 2; policy = key // 2
            rows.append({'policy': policy, 'endpoint': endpoint, 'initial_proof': first,
                         'repeated_proof': repeated, 'policy_result': policy_result, 'overall_result': overall,
                         'calls': calls, 'initial_nanos': initial, 'retry_nanos': retry})
    assert rows and sum(row['calls'] for row in rows) % 2 == 0
    (out / 'counters.json').write_text(json.dumps(rows, indent=2) + '\n')
    (out / 'result.json').write_text(json.dumps({'phases': phases, 'complete': True,
                                               'rows': len(rows), 'observed_endpoint_calls': sum(row['calls'] for row in rows) // 2,
                                               'timing_is_perturbed': True, 'runtime_admitted': False}, indent=2) + '\n')


if __name__ == '__main__':
    main()
