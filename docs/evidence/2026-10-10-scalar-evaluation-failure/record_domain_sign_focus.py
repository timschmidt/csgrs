"""Freeze the corrected sign-domain candidate before broader retention gates finish."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
from parse_test_results_v3 import parse_phase
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
S=ROOT/'target/scalar-evaluation-candidate/source'
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
build=E/'typed-api-prototype-v1624-domain-sign-release-build'
record=build/'source-hashes.json.gz';source=json.loads(gzip.decompress(record.read_bytes()))
assert all(sha(S/k)==v for k,v in source.items())
baseline=E/'typed-api-prototype-v1592-domain-sign-other-baseline-tests'
old=json.loads(gzip.decompress((baseline/'source-hashes.json.gz').read_bytes()))
changed=sorted(k for k in old.keys()|source.keys() if old.get(k)!=source.get(k))
assert changed==['crates/hyperreal/src/computable/node/domain_certificates.rs','crates/hyperreal/src/computable/node/domain_sign_tests.rs','crates/hyperreal/src/computable/node/domain_validation.rs']
patch=''
for k in changed:
 if k.endswith('domain_sign_tests.rs'):continue
 patch+=''.join(difflib.unified_diff((baseline/'inputs'/k).read_text().splitlines(keepends=True),
  (build/'inputs'/k).read_text().splitlines(keepends=True),fromfile='a/'+k,tofile='b/'+k))
p=E/'domain-sign-retaining.patch';assert not p.exists();p.write_text(patch)
caps=read(E/'domain-sign-retaining-caps-v1621.json')
assert caps['source_record_sha256']==sha(record) and len(caps['tests'])==11
assert all(r['result']['exit_code']==r['cap']['exit_code']==0 and not r['cap']['wall_timeout'] for r in caps['tests'].values())
phases={}
for version in range(1622,1627):
 matches=list(E.glob(f'typed-api-prototype-v{version}-*'));assert len(matches)==1
 phase=matches[0];r=read(phase/'result.json')
 assert r['exit_code']==0 and r['source_unchanged'] and r['main_unchanged']
 assert sha(phase/'source-hashes.json.gz')==sha(record)
 phases[phase.name]=r
units=parse_phase(E/'typed-api-prototype-v1622-domain-sign-scalar-tests')
assert units['top_level_passed']==1065 and units['top_level_failed']==0 and units['child_process_passed']==1
pub=(E/'typed-api-prototype-v1615-domain-sign-cap/stderr.log').read_text().splitlines()
pub=[l for l in pub if l.startswith('domain sign public ')];assert len(pub)==12 and all('answer=Ok(0)' in l for l in pub)
proof=(E/'typed-api-prototype-v1616-domain-sign-cap/stderr.log').read_text().splitlines()
proof=[l for l in proof if l.startswith('domain sign publication ')];assert len(proof)==12 and all('sign=Some(Positive)' in l and 'consumed=2,' in l for l in proof)
out=E/'domain-sign-focused-implementation-v1626.json';assert not out.exists()
out.write_text(json.dumps({
 'status':'Corrected isolated implementation passes focused scalar/lint/format gates; broader validation and costs pending; no runtime admission',
 'baseline_record':'domain-sign-baseline-v1593.json','baseline_commit':'11776868d',
 'rejected_variant_and_correction':'domain-sign-publication-correction-v1621.json',
 'source_record':str(record.relative_to(E)),'source_record_sha256':sha(record),
 'changed_since_baseline':changed,'production_patch':p.name,'production_patch_sha256':sha(p),
 'focused_caps':'domain-sign-retaining-caps-v1621.json','focused_test_functions':11,
 'scalar_tests':units,'phases':phases,'public_coarse_queries':pub,'retained_child_proofs':proof,
 'public_completion_before_after':[4,12],
 'new_behavior':'Finite-real domain checks consult retained signs and canonical leaf signs before copying coefficients or traversing rational expressions. Literal proofs are published with the existing monotone atomic fact mechanism.',
 'preserved_behavior':['Independent child existence still precedes sign use.','Lazy/unreduced rational preflight and all existing recognition/refinement fallbacks remain.','All 1054 earlier scalar tests and full-size fixtures remain unchanged.','No API, serialization, work-unit definition, numerical kernel or main runtime file changed.'],
 'limits':['Public query examples use public serialization; retained-child diagnostics inspect internal domain proofs and then use public zero-work queries.','The sign-domain private query itself still is not a public API.','Million-bit public ln1p sampling still requests 875240 Rust bytes; no aggregate byte/time guarantee follows from the cheaper domain proof.','The survey warmup comment is known to overstate shared-log warming; log-range-survey-v1583.json records the exact limitation.','Prior conditional proof inputs are unchanged, but the new runtime domain implementation is outside their scope.','The broad consumer/default/debug checks, cost controls and replay comparisons were not complete at this record. The full 39 properties, 14 targets and crate future work remain open.']
},indent=2)+'\n')
print(json.dumps({'record':out.name,'scalar_tests':1065,'public_completions':[4,12]}))
