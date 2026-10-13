"""Verify complete report equality without changing the cost rejection."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'report-replay-v1'
    result = json.loads((out / 'result.json').read_text())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert result['complete'] and result['all_reports_byte_identical']
    assert result['all_report_values_identical'] and result['inputs_and_sources_unchanged']
    assert protocol['extracted_input_files'] == 108
    assert protocol['normalization'] is None
    assert sha(E / 'replay_reports_v1.py') == protocol['runner_sha256']
    for command in json.loads((out / 'commands.json').read_text()):
        name = command['name']
        if name.startswith('build-'):
            raw = (out / (name + '.stdout')).read_bytes()
            assert command['exit_code'] == 0
        else:
            raw = gzip.decompress((out / (name + '.json.gz')).read_bytes())
            assert command['exit_code'] == 1
        assert hashlib.sha256(raw).hexdigest() == command['stdout_sha256']
        assert len(raw) == command['stdout_bytes']
        assert sha(out / (name + '.stderr')) == command['stderr_sha256']
    for comparison in result['comparisons']:
        label = comparison['fixture']
        baseline = gzip.decompress((out / (label + '-baseline.json.gz')).read_bytes())
        candidate = gzip.decompress((out / (label + '-candidate.json.gz')).read_bytes())
        assert baseline == candidate and json.loads(baseline) == json.loads(candidate)
        assert hashlib.sha256(baseline).hexdigest() == comparison['baseline_sha256']
    assessment = {
        'decision': 'complete-report-equality-verified-cost-rejection-unchanged',
        'scope': 'The frozen integer-leaf parent versus the rejected coefficient-merge version. Both include the same pending native geometry and persistent cache. This does not compare against uncached main or the older geometry implementation.',
        'reports': result['comparisons'],
        'comparison': 'Complete unnormalized JSON bytes and parsed values are identical for both full original boards. Four fresh CLI processes use the same absolute inputs and four check selections. All 108 extracted input files remain unchanged.',
        'cli_exit_contract': 'Exit 1 denotes active findings in a complete report; both the JSON and its finding counts were validated separately.',
        'cost_qualification': 'The version remains rejected after 165.086329211 worker CPU seconds / 120 and 63.677895054 copper-neck check wall seconds / 60, plus unresolved scalar construction timing. Report equality supplies no performance admission.',
        'limits': 'One replay per variant/board is not a matched application benchmark. Replay paths differ from the preceding experiment, so hashes from the two experiments are not directly comparable. No fixture, field, diagnostic, geometry, identifier, checksum or report byte is filtered from this comparison.',
        'next_step': 'Run the prepared separate frame-pointer profile of the frozen integer-leaf parent on the complete original smoke test. Attribute remaining work to callers and preserve original limits; do not inherit the rejected merge change. Continue the full properties and future-work scope.',
        'prepared_profile_runner': 'docs/evidence/2026-10-12-integer-leaf-profile/profile_v1.py',
        'profile_executed_at_this_checkpoint': False,
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'report-assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'reports': result['comparisons']}))


if __name__ == '__main__':
    main()
