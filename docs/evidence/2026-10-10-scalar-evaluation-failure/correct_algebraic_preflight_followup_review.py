#!/usr/bin/env python3
"""Correct the domain-cache wording without rewriting the retained review."""
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
SOURCE = E.parents[2]/'target/scalar-evaluation-candidate/source'
sha = lambda data: hashlib.sha256(data).hexdigest()
prior = E/'algebraic-preflight-supplemental-review-v1133.json'
data = json.loads(prior.read_text())
finding = next(r for r in data['findings'] if r['kind'] == 'separate domain-certificate payload review')
old = finding['finding']
assert 'Construction-time finite-real hints' in old
finding['finding'] = (
    'Several rational domain cases compare canonical numerator/denominator components after a node debit. '
    'A previously published finite-real certificate can bypass these branches. Node construction retains '
    'inverse-trig/linear-demand hints, not a finite-real certificate; the inspected finite-real stores occur '
    'in successful domain validation. Remaining lazy-reduction and wide-scan exposure needs branch-specific measurement.'
)
prefix = 'crates/hyperreal/src/computable/node/'
for name,ranges in {'representation.rs':[(1,100),(380,450)],'domain_validation.rs':[(1,180)]}.items():
    raw = (SOURCE/(prefix+name)).read_bytes()
    lines = raw.splitlines(keepends=True)
    data['reviewed_source_slices'][prefix+name] = {
        'source_sha256':sha(raw),'total_lines':len(lines),
        'reviewed_slices':[{'first_line':a,'last_line':b,'sha256':sha(b''.join(lines[a-1:b]))} for a,b in ranges],
        'scope':'Only these additional inclusive ranges were read while correcting the domain-cache wording.'
    }
command = ['rg','-n','--glob','*.rs','store_finite_real\\(','crates/hyperreal/src']
result = subprocess.run(command,cwd=SOURCE,text=True,capture_output=True)
assert result.returncode == 0 and not result.stderr
data['text_searches'].append({'command':command,'exit_code':0,'stdout':result.stdout,
    'scope':'Store-site inventory; only the separately listed source ranges were read.'})
data['review_correction'] = {
    'previous_record':prior.name,'previous_record_sha256':sha(prior.read_bytes()),
    'previous_wording':old,
    'reason':'Node::new and AtomicFacts::new retain inverse-trig/linear-demand hints but do not set the finite-real flag. The original review conflated construction hints with cached domain certificates.',
    'runtime_tests_fixtures_and_measurements_unchanged':True
}
out = E/'algebraic-preflight-supplemental-review-v1133-corrected.json'
assert not out.exists()
out.write_text(json.dumps(data,indent=2)+'\n')
print('Corrected domain-cache wording; preserved the original review.')
