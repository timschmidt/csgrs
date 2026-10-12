"""Diagnose retained certificate projections on the complete native boards.

Make a separate instrumented source and target. Preserve the original runtime
candidate, fallback, ownership, query limits and fixtures. Counter/timer costs
and changed code generation mean this is not matched speedup evidence.
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
WORK = ROOT / 'target/persistent-laurent-cache'
SOURCE = WORK / 'diagnostic-v2/source'
TARGET = WORK / 'diagnostic-v2/target'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'diagnostic-v2'
    out.mkdir(exist_ok=False)
    preparation = json.loads((E / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((E / 'initial-source-hashes.json.gz').read_bytes()))
    delta = json.loads((E / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    candidate = original | delta
    baseline = WORK / 'candidate/source'
    assert all(sha(baseline / p) == expected for p, expected in candidate.items())
    assert sources() == preparation['main_source_sha256']
    SOURCE.parent.mkdir(exist_ok=False)
    with (out / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(baseline), str(SOURCE)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    prefix = 'crates/hyperreal/src/computable/node/'
    algebra, structural, helper = [prefix + n for n in ['algebra.rs', 'structural_analysis.rs', 'persistent_laurent_diagnostic.rs']]
    path = SOURCE / algebra
    text = path.read_text()
    start = text.index('    fn project_pi_laurent_polynomial<T>(')
    end = text.index('    fn pi_laurent_polynomial(&self,', start)
    body = text[start:end]
    body = body.replace('        budget: usize,', '        budget: usize,\n        diagnostic_kind: usize,', 1)
    body = body.replace('        let mut memo = None;', '        let diagnostic_started = std::time::Instant::now();\n        let mut memo = None;', 1)
    body = body.replace('            return Some(value);', '''            persistent_laurent_diagnostic::record(
                diagnostic_kind, usize::from(reused), diagnostic_started.elapsed().as_nanos(), 0,
                MAX_PI_LAURENT_NORMAL_FORM_NODES - remaining_work,
            );
            return Some(value);''', 1)
    before = '''        if reused {
            self.pi_laurent_polynomial(budget).and_then(project)
        } else {
            None
        }'''
    after = '''        let diagnostic_first = diagnostic_started.elapsed().as_nanos();
        let diagnostic_nodes = MAX_PI_LAURENT_NORMAL_FORM_NODES - remaining_work;
        if reused {
            let diagnostic_retry = std::time::Instant::now();
            let result = self.pi_laurent_polynomial(budget).and_then(project);
            persistent_laurent_diagnostic::record(
                diagnostic_kind, if result.is_some() { 3 } else { 4 }, diagnostic_first,
                diagnostic_retry.elapsed().as_nanos(), diagnostic_nodes,
            );
            result
        } else {
            persistent_laurent_diagnostic::record(diagnostic_kind, 2, diagnostic_first, 0, diagnostic_nodes);
            None
        }'''
    assert body.count(before) == 1
    body = body.replace(before, after)
    text = 'mod persistent_laurent_diagnostic;\n\n' + text[:start] + body + text[end:]
    for before, after in [
        ('self.project_pi_laurent_polynomial(budget, |polynomial|', 'self.project_pi_laurent_polynomial(budget, 0, |polynomial|'),
        ('self.project_pi_laurent_polynomial(64, |polynomial|', 'self.project_pi_laurent_polynomial(64, 3, |polynomial|'),
        ('self.project_pi_laurent_polynomial(48, |polynomial|', 'self.project_pi_laurent_polynomial(48, 2, |polynomial|'),
    ]:
        assert text.count(before) == 1
        text = text.replace(before, after)
    path.write_text(text)
    path = SOURCE / structural
    text = path.read_text()
    before = 'self.project_pi_laurent_polynomial(64, |polynomial|'
    assert text.count(before) == 1
    path.write_text(text.replace(before, 'self.project_pi_laurent_polynomial(64, 1, |polynomial|'))
    shutil.copyfile(E / 'persistent_laurent_diagnostic_v2.rs', SOURCE / helper)
    with (out / 'prepare-format.log').open('w') as log:
        subprocess.run(['rustfmt', '--edition', '2024', str(SOURCE / algebra), str(SOURCE / structural)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    changed = [algebra, structural, helper]
    current = {name: sha(SOURCE / name) for name in sorted(set(candidate) | {helper})}
    assert {p for p, value in current.items() if candidate.get(p) != value} == set(changed)
    with tarfile.open(out / 'instrumentation.tar.gz', 'w:gz') as archive:
        for name in changed:
            archive.add(SOURCE / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (baseline / name).read_text().splitlines(True) if name in candidate else [],
        (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
    (out / 'instrumentation.patch').write_text(patch)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'preparation_sha256': sha(E / 'preparation.json'),
                'candidate_delta_sha256': sha(E / 'validation-v1/protocol.json'),
                'only_instrumented_sources': {name: current[name] for name in changed},
                'instrumentation_archive_sha256': sha(out / 'instrumentation.tar.gz'),
                'environment': env,
                'outcome_order': ['cold_success', 'retained_success', 'cold_decline', 'retry_success', 'retry_decline'],
                'unchanged_full_board_limit_seconds': 120, 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert all(sha(SOURCE / p) == expected for p, expected in current.items())
        assert all(sha(baseline / p) == expected for p, expected in candidate.items())

    def run(name, command, allow_failure=False):
        verify()
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | env, stdout=stdout, stderr=stderr).returncode
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

    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('cache-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', 'persistent_laurent_cache_tests', '--', '--test-threads=1'])
    log = run('build-board-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json'])
    events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
    artifacts = [event for event in events if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(TARGET)
    identity = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    (out / 'binary.json').write_text(json.dumps(identity, indent=2) + '\n')
    stdout = run('full-board', ['/usr/bin/time', '-v', str(binary), 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'], allow_failure=True)
    stderr = (out / 'full-board.stderr').read_text()
    combined = stdout + stderr
    assert 'skipping complex project fixture test' not in combined
    assert combined.count('hyperdrc: finished emit report') == 2
    assert '1 passed' in stdout or '1 failed' in stdout
    rows = [json.loads(line.split('LAURENT_CACHE_DIAGNOSTIC ', 1)[1]) for line in combined.splitlines() if line.startswith('LAURENT_CACHE_DIAGNOSTIC ')]
    assert rows
    (out / 'counters.json').write_text(json.dumps(rows, indent=2) + '\n')
    (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': True,
                                               'counter_rows': len(rows), 'timing_is_perturbed': True,
                                               'runtime_admitted': False}, indent=2) + '\n')


if __name__ == '__main__':
    main()
