#!/usr/bin/env python3
"""Retain constructor allocation failures and unresolved new test assumptions."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'inverse-constructor-baseline-v1216.json'
    assert not out.exists()
    previous = read(E / 'typed-api-prototype-v1195-state.json')
    for name, digest in previous['artifacts_sha256'].items():
        assert sha(E / name) == digest, name
    old_record = E / previous['source_record']
    before = json.loads(gzip.decompress(old_record.read_bytes()))
    record = E / 'typed-api-prototype-v1206-inverse-constructor-baseline-build/source-hashes.json.gz'
    source = json.loads(gzip.decompress(record.read_bytes()))
    assert sha(record) == 'd009c11a2c33b07ee7ad25c767973d7f951406f4f87b7b9d689bbcce7771a3e6'
    for name, digest in source.items():
        assert sha(SOURCE / name) == digest, name
    changed = {n: {'before': before.get(n), 'after': v} for n, v in source.items() if before.get(n) != v}
    prefix = 'crates/hyperreal/src/computable/'
    node = prefix + 'node.rs'
    probe = prefix + 'node/inverse_constructor_probe_tests.rs'
    tests = prefix + 'node/inverse_constructor_preflight_tests.rs'
    assert set(changed) == {node, probe, tests}
    original_node = (SOURCE / node).read_text()
    for test in [probe, tests]:
        original_node = original_node.replace(f'include!("{test.removeprefix(prefix)}");\n', '')
    assert original_node == (old_record.parent / 'inputs' / node).read_text()
    assert all(source[n] == v for n, v in before.items() if 'test' in n)
    first_tests = (E / 'typed-api-prototype-v1198-inverse-constructor-baseline-build/inputs' / tests).read_text()
    assert (SOURCE / tests).read_text().startswith(first_tests.rstrip()[:-1])
    guard = json.loads(gzip.decompress((E / 'candidate-preparation/main-hashes.json.gz').read_bytes()))
    for name, digest in guard.items():
        assert sha(ROOT / name) == digest, name
    expected = read(E.parent / '2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines()
    assert [s for s in status if not s[3:].startswith(str(E.relative_to(ROOT)))] == expected
    caps = read(E / 'inverse-constructor-baseline-caps-v1216.json')
    assert caps['source_record_sha256'] == sha(record)
    observations, codes = {}, []
    for phase, row in caps['tests'].items():
        result = read(E / phase / 'result.json')
        assert result == row['result'] and result['source_unchanged'] and result['main_unchanged']
        assert sha(E / phase / 'source-hashes.json.gz') == sha(record)
        cap = row['cap']
        assert cap['address_space_bytes'] == 192 * 1024 * 1024
        assert cap['cpu_seconds_soft_hard'] == [2, 3] and cap['wall_seconds_limit'] == 4
        assert not cap['wall_timeout'] and cap['exit_code'] == result['exit_code']
        assert cap['helper_sha256'] == sha(E / 'run_limited_unit_test.py')
        codes.append(result['exit_code'])
        observations[row['test']] = (E / phase / 'stderr.log').read_text().splitlines()
    assert codes == [101, 101, 0, 0, 0, 0, 0, 101, 101, 101]
    survey_phase = E / 'typed-api-prototype-v1197-inverse-constructor-probe-cap'
    survey = [s for s in (survey_phase / 'stderr.log').read_text().splitlines() if s.startswith('inverse constructor survey ')]
    assert len(survey) == 160
    survey_log = str((survey_phase / 'stderr.log').relative_to(E))
    assert read(survey_phase / 'result.json')['exit_code'] == 0
    costs = {}
    for family, p in read(E / 'rational-preflight-paired-cost-v1161/summary.json')['provenance'].items():
        binary = Path(p['binaries']['candidate'])
        assert sha(binary) == p['binary_sha256_at_paired_run']['candidate']
        assert binary.stat().st_size == p['executable_bytes']['candidate']
        costs[family] = {'phase': p['phases']['candidate'], 'binary': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    out.write_text(json.dumps({
        'status': 'Frozen constructor baseline; three allocation failures, five passing guards, two unresolved newly introduced log-offset proof expectations; no runtime repair or admission',
        'previous_checkpoint': 'typed-api-prototype-v1195-state.json',
        'verified': {'artifacts': len(previous['artifacts_sha256']), 'prior_sources': len(before), 'current_sources': len(source), 'main_guarded': len(guard), 'original_worktree_entries': len(expected)},
        'source_record': str(record.relative_to(E)), 'source_record_sha256': sha(record),
        'test_only_changes': changed, 'previous_checkpoint_tests_unchanged': True,
        'survey_log': survey_log, 'survey_log_sha256': sha(E / survey_log), 'survey_observations': len(survey),
        'baseline_caps': 'inverse-constructor-baseline-caps-v1216.json', 'exit_codes_in_test_order': codes,
        'observations': observations,
        'preparation': {
            'initial_build_and_survey': ['typed-api-prototype-v1196-inverse-constructor-probe-build', survey_phase.name],
            'seven_test_baseline': 'inverse-constructor-baseline-caps-v1146.json',
            'collector_filename_correction': 'The executed first collector retained a stale v1146 aggregate suffix. Its seven actual phases are v1199 through v1205. Both helper and output are preserved unchanged. The expanded v2 collector writes the correct v1216 suffix.',
            'expanded_baseline': 'The first seven test bodies are unchanged. Three appended tests add public ln(2) and sqrt(2) offsets. The two appended retry/cutpoint guards fail on their first ln(2) generous-work expectation; they are unresolved expectations, not established allocation regressions.'},
        'findings': [
            'The public million-bit atan(n+pi)-atan(m+pi) cases request 251176 Rust allocation bytes at allowances 32 through 128 in the survey, despite returning Unknown. Eight-bit controls are much smaller.',
            'The internal raw AtanDeferred(Int) shape requests up to 876688 bytes at allowance 24; at allowances 32 through 48 it returns the correct sign after 25 charged units while copying wide leaves. Ordinary public atan(rational) uses a different, already preflighted shape.',
            'The public sqrt(2) offset independently reproduces wide copies: million-bit cases request 251800 bytes at 48 units and 251920 bytes at 64. The corresponding ln(2) low-work observations remain width-independent at the measured allowances.',
            'Public rational-atan preflight, six pi-offset cutpoint schedules, six wide pi-offset retry schedules, exact cancellation and infallible exact-leaf construction all pass.',
            'No production source changed. Investigate the new log-offset proof expectations before correcting tests or retaining a runtime repair.'
        ],
        'allocation_scope': 'Thread-local synchronous Rust allocation requests; fixture construction excluded. These are neither native allocation nor RSS/peak-live bounds. Public query wrappers are included and compared at the same allowance.',
        'retained_cost_baselines': costs,
        'limits': ['All prior artifacts and failures remain immutable.', 'No uniform performance claim or runtime admission.', 'All 39 properties, 14 targets and crate future work remain in scope.']
    }, indent=2) + '\n')
    print(json.dumps({'record': out.name, 'verified_artifacts': len(previous['artifacts_sha256']), 'codes': codes}))


if __name__ == '__main__':
    main()
