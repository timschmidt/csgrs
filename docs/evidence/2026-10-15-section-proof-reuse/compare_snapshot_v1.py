"""Require identical exact rows and public evidence for a performance-only trial."""
import hashlib
import json
from pathlib import Path
import sys

e = Path(__file__).resolve().parent
phase = sys.argv[1]
left = e / 'baseline-v2/snapshot.stdout'
right = e / phase / 'snapshot.stdout'


def cases(path):
    rows = {}
    for line in path.read_text().splitlines():
        if line.startswith('case '):
            key = line
            rows[key] = []
        else:
            rows[key].append(line)
    return rows


baseline, candidate = cases(left), cases(right)
result = {
    'inputs': {str(p): {'sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
                      'bytes': p.stat().st_size} for p in [left, right]},
    'case_names_match': list(baseline) == list(candidate),
    'cases': {key: {'identical': value == candidate.get(key),
                    'baseline_lines': len(value),
                    'candidate_lines': len(candidate.get(key, []))}
              for key, value in baseline.items()},
    'passed': left.read_bytes() == right.read_bytes(),
    'scope': 'Exact ordered boundary/profile endpoints, roles, area expressions, curve families, native-carrier availability, scalar parameter domains, arrangement-source flags and operation certainty. Shared-kernel agreement is not an independent set oracle; the separately executed 25 regressions and snapshot exact-area assertions establish those semantics. No analytic-fidelity or inherited mesh-provenance claim.'
}
(e / phase / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({'passed': result['passed'], 'cases': len(baseline),
                  'differing': [key for key, value in result['cases'].items() if not value['identical']]}))
raise SystemExit(0 if result['passed'] else 1)
