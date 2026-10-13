"""Clarify full-plan scope without reopening already verified ledger targets."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    source, destination = E / 'decision-v1.json', E / 'decision-v2.json'
    assert not destination.exists()
    original = json.loads(source.read_text())
    reviewed = json.loads(source.read_text())
    before = 'The original native changes and all 39 properties, 14 targets and crate future work remain open as previously recorded.'
    after = 'The full plan remains incomplete. All 39 properties, 14 targets and crate future work remain in scope; individual ledger statuses are unchanged, including already verified targets.'
    assert sum(before in row for row in reviewed['limits']) == 1
    reviewed['limits'] = [row.replace(before, after) for row in reviewed['limits']]
    assert {k: v for k, v in reviewed.items() if k != 'limits'} == {
        k: v for k, v in original.items() if k != 'limits'
    }
    reviewed['scope_wording_review'] = {
        'source_sha256': sha(source), 'review_runner_sha256': sha(Path(__file__)),
        'change': 'Clarify that the entire plan remains in scope and incomplete; do not imply that all individual target statuses are open.',
        'measurements_summary_and_decision_unchanged': True,
        'individual_ledger_statuses_unchanged': True,
    }
    destination.write_text(json.dumps(reviewed, indent=2) + '\n')
    print(json.dumps({'status': reviewed['status'], 'scope_wording_corrected': True,
                      'measurements_summary_and_decision_unchanged': True}))


if __name__ == '__main__':
    main()
