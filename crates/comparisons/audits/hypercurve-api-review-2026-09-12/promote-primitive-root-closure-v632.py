from pathlib import Path
import hashlib,json

A=Path(__file__).resolve().parent;W=A.parent
assert json.loads((A/'primitive-root-evidence-minimal-20260928-v631-reaped.json').read_text())['outer_exit_code']==0
guard=json.loads((A/'parameter-construction-20260928-v605-sources.json').read_text())
for name,sha in guard.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
source=(A/'retained-structural-pair-candidate-v625.rs').read_text()
body='''let result = match result {
    Classification::Decided(intersections) if !retained_contacts.is_empty() => {
        merge_parallel_pair_intersection_sets(
            intersections,
            BezierParallelPairIntersectionSet2::complete(
                retained_contacts.into(),
                Arc::from([]),
            ),
            policy,
        )?
    }
    result => result,
};'''
changed=0
for indent in ['                ','        ']:
 old='\n'.join(indent+line for line in body.splitlines())
 changed+=source.count(old)
 source=source.replace(old,indent+'let result = extend_parallel_pair_contacts(result, retained_contacts, policy)?;')
assert changed==3,changed
marker='fn merge_parallel_pair_intersection_sets('
helper='''/// Combines retained component events with replayed isolated contacts through
/// the same exact deduplication and completeness authority on every domain.
fn extend_parallel_pair_contacts(
    result: Classification<BezierParallelPairIntersectionSet2>,
    retained_contacts: Vec<BezierParallelPairIntersectionContact2>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
    match result {
        Classification::Decided(intersections) if !retained_contacts.is_empty() => {
            merge_parallel_pair_intersection_sets(
                intersections,
                BezierParallelPairIntersectionSet2::complete(
                    retained_contacts.into(),
                    Arc::from([]),
                ),
                policy,
            )
        }
        result => Ok(result),
    }
}

'''
assert source.count(marker)==1
source=source.replace(marker,helper+marker)
assert 'PAIR_SATURATION' not in source and 'PAIR_STRUCTURAL' not in source
root=(A/'primitive-root-evidence-candidate-v630.rs').read_text()
root=root.replace('''                assert_eq!(reduced,expected);''','''                assert_eq!(reduced.len(),expected.len());
                for (actual,expected) in reduced.iter().zip(&expected) {
                    assert_eq!(actual.exact_rational_ref(),expected.exact_rational_ref());
                }''')
root=root.replace('''        assert_eq!(reduced,[1,1,-6].map(Real::from));''','''        assert_eq!(reduced.len(),3);
        for (actual,expected) in reduced.iter().zip([1,1,-6].map(HyperRational::new)) {
            assert_eq!(actual.exact_rational_ref(),Some(&expected));
        }''')
paths={'hypercurve/src/bezier_offset.rs':source,'hypersolve/src/root_isolation.rs':root}
for name,data in paths.items():
 (W/name).write_text(data)
(A/'primitive-root-closure-promoted-v632.json').write_text(json.dumps({name:hashlib.sha256(data.encode()).hexdigest()for name,data in paths.items()},indent=2)+'\n')
j=A/'current-stationary-fillet-20260927.md';j.write_text('''## V631 passes; minimal closure fix promoted V632

V631 exactouter70909reaped0. Removing the proposed finite-root envelope change preserves both public cusp tests (offset0.164s, construction0.016s); V625 envelope machinery and test are NOT promoted. Applied only retained structural correspondence/domain regression in existingHCbezier_offset and primitive rational square-free root evidence/two regressions inHSroot_isolation. Consolidated three repeated retained-contact merge blocks into one private authority. No Hyperreal implementation change is needed. Production now23pendingfiles (21HC,2HS), including earlier parameter API/image-selection changes. Formatting/normal qualification still required. V626 unit overlap-only fallback audit and V600 incident cusp probe remain unrun. NextunusedartifactV633.

'''+j.read_text())
print('Promoted minimal closure repair; rational-envelope proposal discarded')
