"""Run costs and retained replay/cap gates sequentially after broad validation."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
validation = read(E / 'domain-bound-candidate-validation-v1702.json')
expected = read(E / 'domain-bound-candidate-caps-v1693.json')['source_record_sha256']
assert validation['source_record_sha256'] == expected
assert len(validation['phases']) == 9
assert all(r['result']['exit_code'] == 0 and r['result']['main_unchanged']
           and r['result']['source_unchanged'] for r in validation['phases'].values())
out = E / 'domain-bound-continuation-v1753'
assert not out.exists()
out.mkdir()
(out / 'runner.py').write_bytes(Path(__file__).read_bytes())
steps = [
    ('standalone-costs', 'run_domain_bound_costs.py', None),
    ('paired-costs', 'replay_domain_bound_cost.py', 'standalone-costs'),
    ('budget-replays', 'run_domain_bound_replays.py', None),
    ('compare-budget-replays', 'compare_domain_bound_budget_replays.py', 'budget-replays'),
    ('remaining-caps-and-replays', 'validate_domain_bound_remaining.py', None),
]
rows = []
def save(complete):
    (out / 'results.json').write_text(json.dumps({'source_record_sha256':expected,
        'complete':complete,'steps':rows,'limits':'A failed cost comparison remains failed; unrelated replay and cap gates continue. No failed phase is rerun or overwritten.'},indent=2)+'\n')
for label, helper, dependency in steps:
    if dependency and next(r for r in rows if r['label']==dependency)['exit_code'] != 0:
        rows.append({'label':label,'helper':helper,'exit_code':None,'skipped_because':dependency+' failed'})
        save(False)
        print(label,'not run: prerequisite failed',flush=True)
        continue
    command = [sys.executable, str(E / helper)]
    started = time.monotonic()
    print(label,'started',flush=True)
    with (out / (label+'.stdout')).open('wb') as stdout, (out / (label+'.stderr')).open('wb') as stderr:
        result = subprocess.run(command,cwd=ROOT,stdout=stdout,stderr=stderr)
    rows.append({'label':label,'helper':helper,'helper_sha256':sha(E/helper),'command':command,
                 'exit_code':result.returncode,'seconds':time.monotonic()-started})
    save(False)
    print(label,'exit',result.returncode,flush=True)
save(True)
assert all(r['exit_code']==0 for r in rows),rows
