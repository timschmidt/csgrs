"""Record the original native failures and reject runtime admission of this version."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'native-gate-v1'
    result = json.loads((out / 'result.json').read_text())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert result['complete'] and not result['all_passed']
    assert sha(E / 'native_gate_v1.py') == protocol['runner_sha256']
    phases = {phase['name']: phase for phase in result['phases']}
    for name, phase in phases.items():
        assert phase['exit_code'] == (101 if name in ['full-board', 'copper-neck'] else 0)
        assert phase['sources_unchanged']
        for stream in ['stdout', 'stderr']:
            assert sha(out / (name + '.' + stream)) == phase[stream + '_sha256']
    board = (out / 'full-board.stderr').read_text()
    neck = (out / 'copper-neck.stderr').read_text()
    worker = float(re.search(r'took ([0-9.]+)s of worker time', board)[1])
    check = float(re.search(r'copper neck regression fixture took ([0-9.]+)s', neck)[1])
    assert worker > 120 and check > 60
    counts = [int(value) for value in re.findall(r'finished run \((\d+) active finding', board)]
    assert counts == [12, 197] and board.count('finished emit report') == 2
    assessment = {
        'scope': __doc__,
        'decision': 'rejected-for-runtime-admission-unresolved-scalar-cost-and-both-native-gates',
        'focused_tests': {name: re.findall(r'test result: .*', (out / (name + '.stdout')).read_text())
                          for name in ['curve-regressions', 'drc-regressions']},
        'strict_consumer_clippy': 'Release all-feature all-target Clippy with -D warnings passes for hypercurve and hyperdrc.',
        'full_board': {
            'exit_code': 101, 'worker_cpu_seconds': worker, 'original_limit_seconds': 120,
            'finding_counts': counts,
            'check_progress': [line for line in board.splitlines() if 'finished check ' in line],
            'complete_process_seconds': phases['full-board']['seconds'],
            'process_user_seconds': float(re.search(r'User time \(seconds\): ([0-9.]+)', board)[1]),
            'process_system_seconds': float(re.search(r'System time \(seconds\): ([0-9.]+)', board)[1]),
            'process_maximum_rss_kib': int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', board)[1]),
        },
        'copper_neck': {
            'exit_code': 101, 'check_wall_seconds': check, 'original_check_wall_limit_seconds': 60,
            'check_assertion_passed': False,
            'complete_process_seconds': phases['copper-neck']['seconds'],
            'qualification': 'The test reports its timed check duration on failure. Complete process duration includes full-board parsing and layer preparation and is a different metric.',
        },
        'comparison_limits': 'The integer-leaf parent previously failed the full-board gate at 154.353265288 worker seconds and passed its separate copper-neck assertion. These standalone runs are not a matched application comparison and do not establish a causal slowdown. This version nevertheless fails both required original assertions and retains the unresolved signed-word construction slowdown.',
        'review': 'The borrowed and owned Rational Add implementations both call add_ref. The sole coefficient-add caller mutates an owned temporary; every polynomial insertion caller propagates failure and discards the temporary before fallback. Structural equality uses immutable DAG data and identity, not Arc strong-count decisions. This is a local source review, not a whole-crate proof.',
        'next_step': 'Finish complete unnormalized report comparison against the frozen integer-leaf parent. Keep this version rejected, preserve all adverse scalar samples and native failures, and start further performance investigation from the parent rather than silently inheriting this merge change.',
        'remaining': [
            'The parent persistent-cache/integer-leaf/native-geometry combination still needs original full-board qualification and broader/default/feature consumer coverage.',
            'Public two-pass recognition-budget wording and broader cache-frontier evidence remain open before any cache admission.',
            'The separate typed scalar prototype and the complete 39-property, 14-target and crate-future-work scope remain active.',
        ],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'main_runtime_unchanged': True, 'plan_complete': False,
    }
    (E / 'native-assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'worker_cpu_seconds': worker,
                      'copper_neck_check_wall_seconds': check, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
