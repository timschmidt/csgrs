"""Record immutable baseline failures and the scoped sign-domain extension contract."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
from parse_test_results_v3 import parse_phase

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
SOURCE=ROOT/'target/scalar-evaluation-candidate/source'
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
hashes=lambda p:json.loads(gzip.decompress(p.read_bytes()))
record=E/'typed-api-prototype-v1592-domain-sign-other-baseline-tests/source-hashes.json.gz'
source=hashes(record)
old=hashes(E/'typed-api-prototype-v1573-restored-build/source-hashes.json.gz')
changed=sorted(k for k in old.keys() | source.keys() if old.get(k)!=source.get(k))
assert changed==['crates/hyperreal/src/computable/node.rs','crates/hyperreal/src/computable/node/domain_sign_tests.rs','crates/hyperreal/src/computable/node/log_range_probe_tests.rs']
assert (SOURCE/changed[0]).read_text()==(E/'typed-api-prototype-v1573-restored-build/inputs'/changed[0]).read_text()+'include!("node/log_range_probe_tests.rs");\ninclude!("node/domain_sign_tests.rs");\n'
assert all(sha(SOURCE/k)==v for k,v in source.items())
failures=[];rows=[]
for file in ['domain-sign-baseline-caps-v1589.json','domain-sign-public-baseline-caps-v1591.json']:
 for phase,r in read(E/file)['tests'].items():
  expected=101 if any(s in r['test'] for s in ['canonical_leaves','retained_child','public_queries']) else 0
  assert r['result']['exit_code']==r['cap']['exit_code']==expected
  assert not r['cap']['wall_timeout']
  if expected: failures.append(phase)
  rows.extend(s for s in (E/phase/'stderr.log').read_text().splitlines() if s.startswith('domain sign '))
assert len(rows)==76 and len(failures)==3
public=[r for r in rows if r.startswith('domain sign public ')]
assert len(public)==12 and sum('answer=Ok(' in r for r in public)==4
units=parse_phase(E/'typed-api-prototype-v1592-domain-sign-other-baseline-tests')
assert units['top_level_passed']==1061 and units['top_level_failed']==0 and units['child_process_passed']==1
result=read(E/'typed-api-prototype-v1592-domain-sign-other-baseline-tests/result.json')
assert result['exit_code']==0 and result['source_unchanged'] and result['main_unchanged']
libraries=read(E/'atan-restoration-validation-v1576.json')['restored_libraries_sha256']
assert all(sha(Path(k))==v for k,v in libraries.items())
guard=hashes(E/'candidate-preparation/main-hashes.json.gz')
assert all(sha(ROOT/k)==v for k,v in guard.items())
original=read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
status=subprocess.check_output(['git','status','--short','--untracked-files=all'],cwd=ROOT,text=True).splitlines()
assert [r for r in status if not r[3:].startswith(str(E.relative_to(ROOT))+'/')]==original
out=E/'domain-sign-baseline-v1593.json';assert not out.exists()
out.write_text(json.dumps({
 'status':'Scoped completeness baseline before production edits; no runtime admission',
 'source_record':str(record.relative_to(E)),'source_record_sha256':sha(record),'source_changes':changed,
 'production_matches_retained_v1404':True,'release_libraries_sha256':libraries,
 'earlier_survey':'log-range-survey-v1583.json','earlier_survey_commit':'335170220',
 'compile_failure':'typed-api-prototype-v1584-domain-sign-baseline-build',
 'compile_correction':'The new test used nonexistent Computable::subtract; corrected to add(-3) before executing tests. The original failed build and source are retained.',
 'failed_regression_phases':failures,'observations':rows,'public_queries':12,'public_completed':4,
 'other_scalar_tests':units,'main_guarded_files':len(guard),'original_worktree_entries_preserved':len(original),
 'contract':{
  'owner':'hyperreal finite-real domain proofs',
  'extension':'Use an existing root sign certificate or the literal sign of a canonical rational, integer, one or positive shared constant before bounded rational recognition.',
  'existence':'The independent child finite-real proof still precedes sign use. A sign alone does not prove the child is defined.',
  'scope':'Keep lazy/unreduced rational preflight, rational recognition, exact-sign traversal and numeric domain refinement as existing fallbacks. No new graph traversal, coefficient copying or speculative budget is introduced.',
  'outputs':'Former exhausted work may complete. Every definite fact, numerical error bound and typed invalid-domain result must agree; previously available proofs remain reusable.',
  'limits':'Work units remain implementation-defined cooperative charges. No byte/time bound, new representation, public API or serialized format is introduced.',
  'priority':'Completeness before performance or allocation savings; preserve all higher-priority exactness and prior completeness.'
 },
 'validation_plan':[
  'Run all five frozen domain regressions and five logarithm survey/retry tests under independent caps.',
  'Run all existing scalar tests and strict Clippy; do not remove lazy-rational preflight tests or full-size fixtures.',
  'Compare public budget/replay outcomes as monotone strengthening, with unchanged definite values/error contracts, and retain any changed allocation/allowance observations.',
  'Measure ordinary retained cost controls with unchanged probe sources and exact output evidence before checkpoint retention.',
  'Full stack, full-board, concurrent schedules and implementation-connected formal proofs remain required for runtime admission.'
 ]
},indent=2)+'\n')
print(json.dumps({'record':out.name,'failures':len(failures),'observations':len(rows),'other_tests':1061}))
