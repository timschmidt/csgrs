"""Compare every retained full-resolution source-coordinate artifact."""
import hashlib
import json
from pathlib import Path
import sys

baseline, candidate, output = map(Path, sys.argv[1:])
assert not output.exists()
names = sorted(path.name for path in baseline.glob('*.json'))
assert len(names) == 10
assert names == sorted(path.name for path in candidate.glob('*.json'))
rows = []
for name in names:
    before, after = (baseline / name).read_bytes(), (candidate / name).read_bytes()
    rows.append({
        'fixture': name, 'baseline': str(baseline / name),
        'candidate': str(candidate / name), 'identical': before == after,
        'baseline_sha256': hashlib.sha256(before).hexdigest(),
        'candidate_sha256': hashlib.sha256(after).hexdigest(),
        'baseline_bytes': len(before), 'candidate_bytes': len(after),
        'vertices': len(json.loads(before)),
    })
result = {'passed': all(row['identical'] for row in rows), 'rows': rows,
          'scope': 'Exact serialized coordinate expressions for all ten original README and profile-benchmark inputs. This checks preservation, not independent analytic fidelity.'}
output.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({'passed': result['passed'], 'fixtures': len(rows),
                  'mismatches': [row['fixture'] for row in rows if not row['identical']]}))
assert result['passed']
