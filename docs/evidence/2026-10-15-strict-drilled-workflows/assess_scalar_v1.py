"""Assess only the scalar identity, keeping unresolved modeling work explicit."""
from pathlib import Path
import gzip, hashlib, json, re, statistics
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert not (E/'scalar-assessment-v1.json').exists()
qualification=json.loads((E/'qualification-v2/result.json').read_text())
assert qualification['passed']
current=json.loads(gzip.decompress((E/'qualification-v2/source-hashes.json.gz').read_bytes()))
assert all(sha(ROOT/name)==digest for name,digest in current.items())
tests={}
for row in qualification['commands']:
    assert row['exit_code']==0 and row['sources_unchanged']
    matches=re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored',(E/'qualification-v2'/(row['name']+'.stdout')).read_text())
    if matches:
        totals=[sum(int(m[i]) for m in matches) for i in range(3)]
        assert totals[1]==0
        tests[row['name']]=dict(zip(['passed','failed','ignored'],totals))
        tests[row['name']]['test_executables_or_doc_groups']=len(matches)
cost=json.loads((E/'cost-v2/result.json').read_text())
assert cost['passed'] and len(cost['commands'])==11
samples={}
for row in cost['commands']:
    assert row['exit_code']==0 and row['sources_unchanged']
    if not re.fullmatch(r'\d-(baseline|candidate)',row['name']):continue
    revision=row['name'].split('-')[1]
    rows=[json.loads(line) for line in (E/'cost-v2'/(row['name']+'.stdout')).read_text().splitlines()]
    assert len(rows)==24
    for sample in rows:
        assert sample['iterations']==128
        assert sample['certified']+sample['unknown']==128
        if revision=='candidate' or sample['case']!='cancel':assert sample['certified']==128
        key=(sample['case'],sample['method'],sample['prepared'])
        samples.setdefault(key,{}).setdefault(revision,[]).append(sample)
comparisons=[]
for (case,method,prepared), revisions in sorted(samples.items()):
    values={}
    for revision,rows in revisions.items():
        assert len(rows)==3
        values[revision]={'raw_samples':rows}
        for metric in ['nanoseconds','allocations','requested_bytes']:
            values[revision][metric+'_per_operation_median']=statistics.median(r[metric]/r['iterations'] for r in rows)
        for metric in ['peak_live_bytes','retained_bytes']:
            values[revision][metric+'_median']=statistics.median(r[metric] for r in rows)
        values[revision]['certified_total']=sum(r['certified'] for r in rows)
        values[revision]['unknown_total']=sum(r['unknown'] for r in rows)
    both_completed=all(r['certified']==128 for rows in revisions.values() for r in rows)
    before=values['baseline']['nanoseconds_per_operation_median'];after=values['candidate']['nanoseconds_per_operation_median']
    comparisons.append({'case':case,'method':['diff_of_products','signed_product_sum','active_signed_product_sum'][method],'prepared':prepared,'both_contracts_completed':both_completed,'latency_change_percent_for_completed_contract':100*(after/before-1) if both_completed else None,'measurements':values})
artifacts={}
for p in Path('/tmp/csgrs-strict-drilled-workflows/cost-v2').iterdir():
    if p.is_file():artifacts[str(p)]={'sha256':sha(p),'bytes':p.stat().st_size}
contract=json.loads((E/'cost-contract-v2.json').read_text())
assert all(sha(ROOT/'crates/hyperreal/src'/name)==digest for name,digest in contract['candidate_source_sha256'].items())
root=Path('/tmp/csgrs-strict-drilled-workflows/cost-v1-source')
assert {p.relative_to(root/'candidate/src').as_posix() for p in (root/'candidate/src').rglob('*.rs') if sha(p)!=sha(root/'baseline/src'/p.relative_to(root/'candidate/src'))}=={'real/arithmetic/facts.rs','real/arithmetic/linear_algebra.rs'}
assert artifacts['/tmp/csgrs-strict-drilled-workflows/cost-v2/libhyperreal_baseline.rlib']['sha256']!=artifacts['/tmp/csgrs-strict-drilled-workflows/cost-v2/libhyperreal_candidate.rlib']['sha256']
pending=json.loads((ROOT/'docs/evidence/2026-10-15-section-proof-reuse/closure-v1.json').read_text())['pending_native_sha256']
assert len(pending)==35 and all(sha(ROOT/name)==digest for name,digest in pending.items())
record={
 'status':'qualified scalar identity; retention decision is recorded separately; strict drilled-body completion remains open',
 'baseline_commit':'f1092edb1',
 'implementation_commit':'775af8774',
 'implementation_sha256':{name:sha(ROOT/name) for name in ['crates/hyperreal/src/real/arithmetic/facts.rs','crates/hyperreal/src/real/arithmetic/linear_algebra.rs','crates/hyperreal/tests/determinant_product_cancellation.rs','crates/hypermesh/tests/symbolic_plane_normals.rs']},
 'properties':['P2','P3','P4','P5','P7','P8','P9','P24','P25','P35','P36','P37','P39'],
 'proof':'For valid Real operands, structural equality proves matching factors. Commutativity and opposite signs prove a*b-b*a=0 in either order. Unknown equality never cancels. Exact-rational reduction remains first. A shared helper serves specialized and generic product sums; no tolerance, refinement, cache, or native workaround is added.',
 'domain':'Exactly two opposite-sign two-factor products with structural factor matches. Other signs, arities and factors retain the original arithmetic path. Near-distinct 1e-12 controls remain strictly nonzero.',
 'tests_by_configuration':tests,
 'new_regressions':{'scalar':8,'native_plane':2},
 'qualified_commands':qualification['commands'],
 'paired_scalar_cost':comparisons,
 'cost_artifacts':artifacts,
 'cost_scope':contract,
 'pending_native_sha256':pending,
 'workflow_frontier':{'strict_certified_segments':[8,24],'strict_undecided_segments':[16,32],'qualified_approximate_segments':[16,32],'original_16_segment_fixture_unchanged':True,'exact_volume_comparison_16_segments':'unresolved','next_blocker':'The second neighboring-triangle coplanarity test retains the affine identity h(v-o)u-h(u-o)v-[h(v-o)o-h(u-o)o]=0 as an opaque expression. Diagnostic-v3 preserves the current trace. No trigonometric identity is required for that cancellation.'},
 'representation_migration':contract['artifact_migration'],
 'preserved_failed_trials':['diagnostic-v1: mixed Hypermesh dependency types','scalar-baseline-v1: incorrect positive precision exponents in two new controls','scalar-baseline-v2: two genuine reversed-product regressions','qualification-v1: OS disk quota in downstream dependency compilation','narrow-format-v1-failure.json: pre-write formatting equivalence check','cost-preparation-v1-failure.json: ambiguous num dependency selection','cost-v1: missing direct-rustc expected cfg(test) declaration; failed before measurement'],
 'limitations':['Current ordinary strict drill is still undecided; successful scalar completion does not establish completed mesh geometry.','Existing 35 native curve/DRC edits remain unadmitted and byte-identical; original full-board cost failures and circuit fixture differences were not rerun here.','Structural comparisons use the existing DAG walk. Four factor comparisons bound the number of equality calls, not their input-dependent work or heap bytes. No new total-work, cancellation or retained-memory contract is claimed.','Eight GiB address-space and 1800-second wall limits are external containment, not kernel budgets.','No full CI/pre-push, released-package, MSRV, binding, sanitizer, formal-proof or whole-stack admission is claimed. No push performed.','Scalar cost samples are instrumented on a shared unpinned host. Baseline Unknown latency is not a successful-operation comparison; no downstream speedup or full mesh artifact equality claim.'],
 'strict_drilled_body_complete':False,
 'full_plan_complete':False,
}
(E/'scalar-assessment-v1.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'status':record['status'],'tests_by_configuration':tests,'cost_comparisons':len(comparisons),'pending_native_unchanged':35},indent=2))
