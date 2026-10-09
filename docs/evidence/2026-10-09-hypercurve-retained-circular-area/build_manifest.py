"""Assemble this scoped checkpoint after all frozen validation and replay gates."""
import hashlib,json,re
from pathlib import Path

record=Path(__file__).resolve().parent
load=lambda p:json.loads(p.read_bytes())
info=lambda p:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
pattern=re.compile(r'^test result: (ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',re.M)
def tests(path):
    rows=pattern.findall(path.read_text());assert rows,path
    return {key:sum(int(r[i]) for r in rows) for key,i in [('passed',1),('failed',2),('ignored',3)]}
validation=load(record/'validation/status.json')
assert len(validation)==14
assert all(s['exit_code']==0 or (s['name']=='demo-tests' and s['exit_code']==101) for s in validation)
demo_baseline=load(record/'demo-baseline/manifest.json')
assert demo_baseline['status']=='matched-existing-failures'
assert info(record/'validation/demo-tests.log')==demo_baseline['candidate_log']
source=load(record/'validation/sources.json');assert source['unchanged']
assert all(info(Path(p))['sha256']==h for p,h in source['source_hashes'].items())
cost=load(record/'cost/manifest.json');assert cost['source_unchanged'] and cost['fixture_exit_code']==0
assert load(record/'cost/all-prior-row-deltas.json')==[]
fixture_audit=load(record/'cost/fixture-report-audit.json')
assert fixture_audit['ordered_check_counts_unchanged'] and fixture_audit['normalized_reports_unchanged']
reports=load(record/'report-replay/manifest.json');assert reports['source_unchanged']
assert load(record/'report-replay/initial-comparison.json')['status']=='passed'
pair=load(record/'original-pair/manifest.json');assert pair['status']=='passed'
capability=load(record/'capability-cost/manifest.json')
fixture=(record/'cost/fixture.log').read_text()
seconds=float(re.search(r'fixture_smoke_total ms=([\d.]+)',fixture)[1])/1000
assert seconds<=120
manifest=dict(
    base_revision=load(record/'baseline-reuse.json')['base_revision'],status='progress',
    scope='Optional exact scalar reconstruction of selected quadratic roots and shared native line/arc lowering for retained fragments. All 39 properties, 14 targets, and crate future work remain in scope.',
    contract=[
        'Quadratic candidates share the existing exact field-division/square-root construction. STRICT interval selection and polynomial replay certify an optional scalar witness; the selected polynomial, isolator, source chart and root authority remain retained.',
        'General root and coordinate projection keeps its existing compact field behavior. Only native line/arc endpoints request the new optional quadratic bridge; rational-search exhaustion cannot suppress a successful explicit witness. The bridge never populates the general scalar-witness cache; completed native contour/area views own their existing caches. Projection requests cannot change subsequent selected-field chord queries.',
        'Exact circle sources and represented selected endpoints produce native arcs without display samples. Adjacent algebraic chords use the same isolated projection of their retained coordinate-image polynomials. Reversal swaps endpoints and circle direction. The source certificate is restricted to its unit chart; exterior traversal and unsupported families retain blockers.',
        'Signed boundary area and native contour conversion share this same lowering. An unavailable line fit no longer prevents an independent certified circle fit for zero/mixed homogeneous weights.',
        'Similarity transforms preserve original retained algebraic carriers even when their native contour view is available. Cold and warmed native views retain selected chord fields and CircularArc round-offset carriers under both policies.',
        'The original complete CPArti board and copper indices 1723/1724 remain intact. Its overlap now has an exact area and native contour view, while independent bounds and the optional duplicate-rejection cap remain valid.',
    ],
    focused='focused/manifest.json',
    regressions=dict(new_tests=8,root_tests=4,circular_tests=4,coverage=['both selected branches','negative polynomial scaling','roots outside the unit interval','rational-isolation negative-cache behavior','repeated and nested radical roots','cold/repeated/cloned projections without general witness-cache mutation','zero-work-budget direct scalar reconstruction','original full-scale pad overlap','closed-form circular cap','degree-five elevation','reversal','semicircle and major-arc sweep','typed exterior/general-curve rejection']),
    baseline=dict(record='baseline.json',tests=tests(record/'baseline.log'),endpoint_diagnosis='endpoint-diagnosis.log'),
    rejected_global_projection='rejected-global-projection/status.json',
    withdrawn_shared_witness_cache='withdrawn-shared-witness-cache/status.json',
    direct_source_evaluation_failure='direct-source-evaluation-failure/status.json',
    fragment_diagnosis='fragment-diagnosis.log',
    image_projection_variant='image-projection-variant/status.json',
    native_similarity_failure='native-similarity-failure/status.json',
    expanded_existing_regression='algebraic_chords_and_round_centers_survive_exact_similarities: both policies with cold and warmed native contour views; all prior family and classification assertions retained.',
    initial_expanded_failure=dict(record='expanded-initial.log',tests=tests(record/'expanded-initial.log'),cause='The shared native source helper returned its line recognizer blocker before trying the exact circle recognizer for zero or mixed homogeneous weights.'),
    validation=dict(record='validation/status.json',sources='validation/sources.json',source_files=len(source['source_hashes']),all_gates_passed=False,baseline_verified_exception='demo-baseline/manifest.json',tests={name:tests(record/'validation'/f'{name}.log') for name in ['curve-tests','demo-tests','consumer-drc','consumer-curves','drc-tests']}),
    snapshots=dict(record='snapshot-versions.json',before=18,after=19),
    cost=dict(record='cost/manifest.json',rows=cost['rows'],all_231_prior_witnesses_unchanged=True,trials=5,fixture_seconds=seconds,fixture_gate_seconds=120,fixture_peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',fixture)[1]),scope=cost['scope']),
    capability_cost=dict(record='capability-cost/manifest.json',rows=4,trials=5,medians_ns=capability['medians_ns'],binary=capability['binary'],peak_rss_kib=[min(o['peak_rss_kib'] for o in capability['observations']),max(o['peak_rss_kib'] for o in capability['observations'])],scope=capability['scope']),
    original_pair='original-pair/manifest.json',reports=dict(record='report-replay/manifest.json',results=reports['reports'],scope=reports['scope']),
    remaining=[
        'The demo still has two restricted-fragment editability failures, independently reproduced with identical diagnostics on the committed baseline. They remain failed pre-push checks.',
        'General retained rational/algebraic integration, higher moments, arbitrary exact scalar identities and unrestricted exterior traversal remain open.',
        'These gates do not establish total time/memory/report-allocation bounds, all ignored compositions, universal report identity, or complete semantic/representational/evidentiary/computational closure.',
        'The remaining native DRC parser/source conversions, legacy copper-index consumers, shape/drill/distribution logic, scalar thresholds, report-chord widths and density proxies remain open.',
        'All 39 properties, 14 targets and crate future work remain in scope. No push or CI configuration change; the full repository pre-push matrix has not been run.',
    ],
)
manifest['artifacts']={str(p.relative_to(record)):info(p) for p in sorted(record.rglob('*')) if p.is_file() and p not in [record/'manifest.json',record/'artifact-audit.json']}
(record/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({k:manifest[k] for k in ['status','validation','cost','capability_cost']},indent=2))
