"""Observe shallow certificate eligibility on the full original board fixtures.

The previous goal turn made progress through six committed checkpoints. This
diagnostic forks the frozen integer-leaf candidate into separate source and
build directories. It observes the root and immediate children before the
unchanged quadratic query, then counts the actual outcome and query duration.
Counters contain only integers. Temporary radicand clones/read guards are gone
before recognition. Exact radicand comparisons can touch Rational's existing
lazy canonicalization cache, so observation perturbs timing/cache warmth.
Only counters from exiting threads are printed; this is not a whole-system CPU
profile, matched performance evidence, or runtime admission.
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
PARENT = ROOT / 'docs/evidence/2026-10-12-small-integer-certificates'
BASELINE = ROOT / 'target/small-integer-certificates/source'
WORK = ROOT / 'target/quadratic-certificate-eligibility/diagnostic-v2'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'diagnostic-v2'
    out.mkdir(exist_ok=False)
    preparation = json.loads((PARENT / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    delta = json.loads((PARENT / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    candidate = original | delta
    assert all(sha(BASELINE / p) == expected for p, expected in candidate.items())
    assert sources() == preparation['main_source_sha256']
    WORK.mkdir(parents=True, exist_ok=False)
    with (out / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASELINE), str(SOURCE)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    prefix = 'crates/hyperreal/src/computable/node/'
    structural = prefix + 'structural_analysis.rs'
    helper = prefix + 'quadratic_certificate_diagnostic.rs'
    path = SOURCE / structural
    text = path.read_text()
    start = text.index('    pub(crate) fn exact_quadratic_surd_parts(')
    end = text.index('    #[cfg(test)]', start)
    body = text[start:end]
    before = '        let value = self.project_pi_laurent_polynomial(64, |polynomial| {'
    assert body.count(before) == 1
    body = body.replace(before, '''        let diagnostic_observation = quadratic_certificate_diagnostic::classify(self);
        let diagnostic_started = std::time::Instant::now();
        let value = self.project_pi_laurent_polynomial(64, |polynomial| {''')
    before = '        })?;'
    assert body.count(before) == 1
    body = body.replace(before, '''        });
        quadratic_certificate_diagnostic::record(
            diagnostic_observation, value.is_some(), diagnostic_started.elapsed().as_nanos(),
        );
        let value = value?;''')
    path.write_text('mod quadratic_certificate_diagnostic;\n\n' + text[:start] + body + text[end:])
    shutil.copyfile(E / 'quadratic_certificate_diagnostic_v2.rs', SOURCE / helper)
    with (out / 'prepare-format.log').open('w') as log:
        subprocess.run(['rustfmt', '--edition', '2024', str(SOURCE / structural), str(SOURCE / helper)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    changed = [structural, helper]
    current = {name: sha(SOURCE / name) for name in sorted(set(candidate) | {helper})}
    assert {p for p, value in current.items() if candidate.get(p) != value} == set(changed)
    with tarfile.open(out / 'instrumentation.tar.gz', 'w:gz') as archive:
        for name in changed:
            archive.add(SOURCE / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (BASELINE / name).read_text().splitlines(True) if name in candidate else [],
        (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
    (out / 'instrumentation.patch').write_text(patch)
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check']),
        ('diagnostic-format', ['rustfmt', '--edition', '2024', '--check', helper]),
        ('diagnostic-regression', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', 'quadratic_certificate_diagnostic::tests', '--', '--test-threads=1']),
        ('cache-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', 'persistent_laurent_cache_tests', '--', '--test-threads=1']),
        ('strict-scalar-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings']),
        ('build-board-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json']),
    ]
    protocol = {'scope': __doc__, 'previous_goal_turn': 'progress',
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'runner_sha256': sha(Path(__file__)),
                'preparation_sha256': sha(PARENT / 'preparation.json'),
                'runtime_delta_sha256': sha(PARENT / 'validation-v1/protocol.json'),
                'prior_report_assessment_sha256': sha(PARENT / 'report-assessment-v1.json'),
                'only_instrumented_sources': {name: current[name] for name in changed},
                'instrumentation_archive_sha256': sha(out / 'instrumentation.tar.gz'),
                'instrumentation_patch_sha256': sha(out / 'instrumentation.patch'),
                'environment': env, 'commands': commands,
                'operations': ['other', 'one', 'integer', 'ratio', 'constant', 'add', 'multiply', 'negate', 'offset', 'inverse', 'square', 'sqrt', 'linear_combination3'],
                'root_kinds': ['unretained', 'rational', 'quadratic', 'multiradical', 'laurent'],
                'child_kinds': ['unavailable', 'intrinsic_rational', 'retained_rational', 'retained_quadratic', 'retained_multiradical', 'retained_laurent', 'absent'],
                'lanes': ['decline', 'retained_root', 'rational_add', 'affine_add', 'same_field_add', 'negate', 'offset_at_most_64', 'rational_scale', 'intrinsic_only'],
                'mask_contract': 'Bit 0/1 denotes a retained immediate left/right operand. For LinearCombination3, bits 0..2 denote the Computable coefficients; its three Rational values have no Computable certificate cells. A retained proof may be wider than quadratic.',
                'duration_contract': 'Elapsed wall nanoseconds around the complete original projection call after observation, including its retries and memo destruction. Instrumentation and cache observations perturb execution. This boundary differs from the earlier per-pass diagnostic timers.',
                'unchanged_full_board_limit_seconds': 120, 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert all(sha(SOURCE / p) == expected for p, expected in current.items())
        assert all(sha(BASELINE / p) == expected for p, expected in candidate.items())

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

    for name, command in commands:
        log = run(name, command)
    events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
    artifacts = [event for event in events if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(TARGET)
    identity = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                'manifest_path': artifact['manifest_path'], 'features': artifact['features']}
    (out / 'binary.json').write_text(json.dumps(identity, indent=2) + '\n')
    stdout = run('full-board', ['/usr/bin/time', '-v', str(binary), 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'], allow_failure=True)
    combined = stdout + (out / 'full-board.stderr').read_text()
    assert 'skipping complex project fixture test' not in combined
    assert combined.count('hyperdrc: finished emit report') == 2
    assert '1 passed' in stdout or '1 failed' in stdout
    rows, masks = [], []
    for line in combined.splitlines():
        if line.startswith('QUADRATIC_SHAPE_DIAGNOSTIC '):
            values = [int(value) for value in line.split()[1:]]
            assert len(values) == 8
            operation, root, left, right, lane, success, calls, nanos = values
            rows.append({'operation': protocol['operations'][operation],
                         'root': protocol['root_kinds'][root],
                         'left': protocol['child_kinds'][left], 'right': protocol['child_kinds'][right],
                         'lane': protocol['lanes'][lane], 'success': bool(success),
                         'calls': calls, 'projection_nanos': nanos})
        elif line.startswith('QUADRATIC_CHILD_MASK_DIAGNOSTIC '):
            operation, mask, success, calls = [int(value) for value in line.split()[1:]]
            masks.append({'operation': protocol['operations'][operation], 'retained_mask': mask,
                          'success': bool(success), 'calls': calls})
    assert rows and masks
    assert sum(row['calls'] for row in rows) == sum(row['calls'] for row in masks)
    (out / 'counters.json').write_text(json.dumps({'shapes': rows, 'child_masks': masks}, indent=2) + '\n')
    (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': True,
                                               'shape_rows': len(rows), 'mask_rows': len(masks),
                                               'observed_calls': sum(row['calls'] for row in rows),
                                               'timing_is_perturbed': True, 'runtime_admitted': False}, indent=2) + '\n')


if __name__ == '__main__':
    main()
