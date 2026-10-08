from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent;C=A/'boundary-provenance-admission-candidate-v783';assert not C.exists()
base={};results={}
def change(name,edit):
 p=W/name;old=p.read_text();new=edit(old);assert new!=old,name
 base[name]=hashlib.sha256(p.read_bytes()).hexdigest();results[name]=new

def geometry(s):
 s=s.replace('/// Arrangement provenance for one retained boundary fragment.','''/// Provenance recorded while constructing a retained boundary fragment.
///
/// Curve operations supply these records; completed boundaries expose them
/// for inspection without accepting caller-authored arrangement indices.''',1)
 a=s.index('impl CurveRegionFragmentSource2 {');b=s.index('    /// Returns the retained arrangement-graph fragment index.',a)
 prefix=s[a:b];assert prefix.count('pub const fn new(')==1
 s=s[:a]+prefix.replace('pub const fn new(','pub(crate) const fn new(')+s[b:]
 a=s.index('    /// Constructs a retained boundary loop with one source record per fragment.');b=s.index('    fn try_new_from_certified_arrangement_chain(',a);s=s[:a]+s[b:]
 old='''            CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(
                fragments,
                sources,
                &CurveContext::STRICT,
            )'''
 new='''            let boundary = CurveRegionBoundaryLoop2::new(fragments, &CurveContext::STRICT).unwrap();
            CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                boundary.fragments,
                sources,
                &CurveContext::STRICT,
            )'''
 assert s.count(old)==1;s=s.replace(old,new)
 old='''            CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(
                fragments,
                (0..4)'''
 new='''            let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
            CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                boundary.fragments,
                (0..4)'''
 assert s.count(old)==1;s=s.replace(old,new)
 a=s.index('    #[test]\n    fn retained_region_constructor_rejects_reused_arrangement_sources_across_loops()')
 s=s[:a]+'''    #[test]
    fn certified_boundary_constructors_validate_arrangement_sources() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let fragments = [(p(0, 0), p(1, 0)), (p(1, 0), p(0, 0))]
                .into_iter()
                .map(|(start, end)| BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        start.clone(), start.lerp(&end, q(1,2)), end,
                    )),
                })
                .collect();
            let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap();
            for sources in [
                Vec::new(),
                vec![CurveRegionFragmentSource2::new(0,0,0)],
                vec![CurveRegionFragmentSource2::new(0,0,0), CurveRegionFragmentSource2::new(0,1,0)],
            ] {
                assert!(matches!(
                    CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(boundary.fragments.clone(), sources.clone(), &policy),
                    Err(CurveError::Topology(_))
                ));
                assert!(matches!(
                    CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(boundary.fragments.clone(), Some(sources), &policy),
                    Err(CurveError::Topology(_))
                ));
            }
            let sources = vec![CurveRegionFragmentSource2::new(7,3,0), CurveRegionFragmentSource2::new(8,3,1)];
            for result in [
                CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(boundary.fragments.clone(), sources.clone(), &policy),
                CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(boundary.fragments.clone(), Some(sources.clone()), &policy),
            ] {
                let result = result.unwrap();
                assert_eq!(result.len(), 2);
                assert_eq!(result.arrangement_sources(), Some(sources.as_slice()));
            }
            assert!(matches!(CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(Vec::new(), Vec::new(), &policy), Err(CurveError::Topology(_))));
            assert!(matches!(CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(Vec::new(), Some(Vec::new()), &policy), Err(CurveError::Topology(_))));
        }
    }

'''+s[a:]
 assert 'try_new_with_arrangement_sources'not in s
 return s
change('hypercurve/src/bezier_region.rs',geometry)
def integration(s):
 s=s.replace('CurveRegionFragmentSource2, ','')
 old='''    assert_topology_error(CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(
        Vec::new(),
        Vec::new(),
        &policy(),
    ));
''';assert s.count(old)==1;s=s.replace(old,'')
 a=s.index('#[test]\nfn retained_boundary_loop_constructor_rejects_duplicate_arrangement_sources()');b=s.index('#[test]',a+8)
 s=s[:a]+s[b:]
 assert 'try_new_with_arrangement_sources'not in s and 'CurveRegionFragmentSource2'not in s
 return s
change('hypercurve/tests/hypercurve_bezier_region.rs',integration)
def docs(s):
 old='''`CurveRegion2` stores filled topology as oriented native Bézier boundary
fragments and is the main input to mixed-family region operations.'''
 assert s.count(old)==1
 return s.replace(old,'''`CurveRegion2` stores filled topology as oriented exact boundary curves and
is the main input to mixed-family region operations. Curve operations emit
boundary provenance for inspection; authored input supplies geometry and fill
semantics.''')
change('hypercurve/README.md',docs)
for name,source in results.items():
 p=C/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(source)
(A/'boundary-provenance-admission-base-v783.json').write_text(json.dumps(base,indent=2)+'\n')
print('Prepared',len(base),'candidate files')
