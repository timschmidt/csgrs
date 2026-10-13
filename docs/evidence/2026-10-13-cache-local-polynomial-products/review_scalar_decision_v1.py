"""Correct one timing-variation sentence without altering samples or the decision."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'scalar-decision-v2.json'
    assert not destination.exists()
    original = json.loads((E / 'scalar-decision-v1.json').read_text())
    assessment = json.loads((E / 'cost-assessment-v1.json').read_text())
    assert sha(E / 'cost-assessment-v1.json') == original['cost_assessment_sha256']
    assert sha(E / 'decide_scalar_v1.py') == original['decision_runner_sha256']
    assert all(sha(E / p) == h for p, h in original['native_runners_sha256'].items())
    observed = {}
    for suite in ['rational', 'quadratic', 'integer']:
        rows = [r for r in assessment['timing_rows'] if r['suite'] == suite]
        observed[suite] = {'phase_count': len(rows),
            'first_block_candidate_faster': sum(r['block_ratios'][0] < 1 for r in rows),
            'first_block_candidate_slower': sum(r['block_ratios'][0] > 1 for r in rows),
            'first_block_ratios_below_0_7': sum(r['block_ratios'][0] < .7 for r in rows)}
    assert observed['rational'] == {'phase_count': 24, 'first_block_candidate_faster': 21, 'first_block_candidate_slower': 3, 'first_block_ratios_below_0_7': 20}
    old = 'All first-block rational and many quadratic ratios reflect slower baseline samples.'
    new = 'The first block has lower candidate timings in 21 of 24 rational phases and 18 of 32 quadratic phases; ratios below 0.7 occur in 20 rational and 12 quadratic phases. The other first-block phases remain visible.'
    reviewed = json.loads(json.dumps(original))
    assert reviewed['unresolved_costs'][3].count(old) == 1
    reviewed['unresolved_costs'][3] = reviewed['unresolved_costs'][3].replace(old, new)
    reviewed['native_runners_sha256'] = {name: sha(E / name) for name in ['validate_native_v2.py', 'assess_native_v2.py']}
    reviewed['decision_review'] = {
        'status': 'one-overbroad-sentence-corrected-samples-and-decision-unchanged',
        'review_runner_sha256': sha(Path(__file__)), 'original_decision_sha256': sha(E / 'scalar-decision-v1.json'),
        'first_block_observations': observed,
        'native_v1_runners_executed': False,
        'new_native_runner_version_reason': 'Keep the original referenced inputs intact and bind native execution to this corrected interpretation.',
    }
    recovered = json.loads(json.dumps(reviewed))
    del recovered['decision_review']
    recovered['native_runners_sha256'] = original['native_runners_sha256']
    recovered['unresolved_costs'][3] = recovered['unresolved_costs'][3].replace(new, old)
    assert recovered == original
    assert not (E / 'native-validation-v1').exists() and not (E / 'native-validation-v2').exists()
    destination.write_text(json.dumps(reviewed, indent=2) + '\n')
    print(json.dumps({'sentence_corrected': True, 'all_measurements_and_decision_unchanged': True, 'first_block_observations': observed}))


if __name__ == '__main__':
    main()
