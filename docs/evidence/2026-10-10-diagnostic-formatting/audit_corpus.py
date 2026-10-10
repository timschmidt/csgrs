"""Classify every observed legacy rounding and diagnostic consumer delta."""
from collections import Counter
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import sys

out = Path(sys.argv[1]).resolve() / 'corpus'
rows = json.loads((out / 'decimal-legacy-differences.json').read_text())
counts, examples = Counter(), {}
for row in rows:
    old, new = row['legacy'], row['checked']
    value = Fraction(row['input']['token'])
    if ('e' in old) != ('e' in new):
        kind = 'exact notation window differs from legacy leading-bit estimate'
    elif Fraction(old) == Fraction(new):
        kind = 'same value, different spelling'
    elif value == (Fraction(old) + Fraction(new)) / 2:
        kind = 'nearest-even halfway choice differs from legacy approximation'
    else:
        kind = 'non-halfway rounding difference'
    counts[kind] += 1
    row['classification'] = kind
    examples.setdefault(kind, {'value': str(value), 'places': row['input']['places'], 'old': old, 'new': new})
    exponent = 0
    if 'e' in old:
        magnitude = abs(value)
        exponent = len(str(magnitude.numerator)) - len(str(magnitude.denominator))
        power = Fraction(10 ** exponent) if exponent >= 0 else Fraction(1, 10 ** -exponent)
        if magnitude < power:
            exponent -= 1
    exponent -= row['input']['places']
    quantum = Fraction(10 ** exponent) if exponent >= 0 else Fraction(1, 10 ** -exponent)
    assert abs(Fraction(old) - Fraction(new)) == quantum
    assert abs(value - Fraction(new)) <= quantum / 2
    if kind == 'non-halfway rounding difference':
        assert abs(value - Fraction(old)) > quantum / 2
result = {'counts': dict(counts), 'examples': examples, 'differences': rows, 'all_changed_outputs_differ_by_one_last_place_unit': True, 'all_checked_results_are_within_half_a_last_place_unit': True, 'non_halfway_legacy_results_exceed_half_a_last_place_unit': True, 'notation_changes_in_this_rational_corpus': counts['exact notation window differs from legacy leading-bit estimate'], 'audit_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(out / 'legacy-difference-audit.json').write_text(json.dumps(result, indent=2) + '\n')
consumers = json.loads((out / 'consumer-rows.json').read_text())
differences = []
for row in consumers:
    if row['stdout'] != row['baseline_stdout'] or row['stderr'] != row['baseline_stderr']:
        assert row['family'] in ['unresolved_zero', 'aborted_pi']
        differences.append({k: v for k, v in row.items() if k != 'wall_seconds'})
(out / 'consumer-difference-audit.json').write_text(json.dumps({'cases': len(consumers), 'unchanged_outputs': len(consumers) - len(differences), 'changed_outputs': len(differences), 'all_changes_are_unknown_or_abort_markers': True, 'differences': differences}, indent=2) + '\n')
print('Audited', len(rows), 'rational output differences and', len(differences), 'consumer output differences.')
