from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent;C=A/'native-boundary-admission-candidate-v777';assert not C.exists()
base={}
def change(name,edit):
 p=W/name;s=p.read_text();t=edit(s);assert s!=t,name
 base[name]=hashlib.sha256(p.read_bytes()).hexdigest();q=C/name;q.parent.mkdir(parents=True,exist_ok=True);q.write_text(t)
def region(s):
 a=s.index('    /// Nests unordered native boundary contours and promotes their decided roles.')
 b=s.index('    /// Classifies native contours through the shared raw-loop nesting authority.',a)
 return s[:a]+'''    /// Constructs the exact regularized fill of native boundary contours.
    ///
    /// The explicit fill rule applies to the sum of signed winding across all
    /// contours, independent of their individual fill rules. `EvenOdd` gives
    /// nesting parity; `NonZero` adds equally oriented contours and cancels
    /// opposite traversals. Crossings, overlaps, and touching boundaries use
    /// the same arrangement as general boundary paths.
    pub fn try_from_native_boundary_contours(
        contours: &[Contour2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        let paths = contours
            .iter()
            .map(curve_path_from_native_contour)
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Self::try_from_boundary_paths(&paths, fill_rule, policy)
    }

'''+s[b:]
change('hypercurve/src/bezier_region.rs',region)
def native(s):
 s=s.replace('CurveRegion2::try_from_native_boundary_contours(Vec::new(), &policy)','CurveRegion2::try_from_native_boundary_contours(&[], FillRule::EvenOdd, &policy)')
 s=s.replace('''        let Classification::Decided(region) = outcome.into_value() else {
            panic!("empty boundary input has decided topology");
        };''','''        let region = outcome.into_value();''')
 s=s.replace('fn boundary_contour_nesting_assigns_disjoint_nested_roles()', 'fn boundary_contour_fill_assigns_disjoint_nested_roles()')
 s=s.replace('''    let classified = CurveRegion2::try_from_native_boundary_contours(
        vec![rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7)],
        &policy(),''','''    let region = CurveRegion2::try_from_native_boundary_contours(
        &[rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7)],
        FillRule::EvenOdd,
        &policy(),''')
 s=s.replace('''    let Classification::Decided(region) = classified else {
        panic!("nested contours should be decided: {classified:?}");
    };
''','')
 a=s.index('#[test]\nfn boundary_contour_nesting_rejects_crossing_or_touching_loops()');b=s.index('#[test]',a+8)
 s=s[:a]+'''#[test]
fn boundary_contour_fill_regularizes_crossings_and_touches() {
    use crate::{BooleanOp, OffsetCornerStyle2};
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill in [FillRule::EvenOdd, FillRule::NonZero] {
            for (contours, probes) in [
                (
                    vec![rectangle(0, 0, 4, 4), rectangle(2, -1, 6, 3)],
                    vec![
                        (p(1, 1), RegionPointLocation::Inside),
                        (p(3, 1), if fill == FillRule::EvenOdd { RegionPointLocation::Outside } else { RegionPointLocation::Inside }),
                        (p(5, 1), RegionPointLocation::Inside),
                        (p(7, 1), RegionPointLocation::Outside),
                    ],
                ),
                (
                    vec![rectangle(0, 0, 4, 4), rectangle(4, 0, 8, 4)],
                    vec![(p(4, 2), RegionPointLocation::Inside), (p(8, 2), RegionPointLocation::Boundary)],
                ),
                (
                    vec![rectangle(0, 0, 4, 4), rectangle(4, 4, 8, 8)],
                    vec![(p(4, 4), RegionPointLocation::Boundary), (p(6, 6), RegionPointLocation::Inside), (p(6, 2), RegionPointLocation::Outside)],
                ),
            ] {
                let outcome = CurveRegion2::try_from_native_boundary_contours(&contours, fill, &policy).unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                let region = outcome.into_value();
                let Classification::Decided(native) = region.native_contours_fast_path(&policy).unwrap().into_value() else {
                    panic!("line boundaries retain a native view");
                };
                let boundaries = native.material_contours().iter().chain(native.hole_contours()).cloned().collect::<Vec<_>>();
                let restored = CurveRegion2::try_from_native_boundary_contours(&boundaries, fill, &policy).unwrap().into_value();
                for (point, expected) in probes {
                    for result in [&region, &restored] {
                        assert_eq!(result.classify_point(&point.clone().into(), &policy).unwrap().into_value(), Classification::Decided(expected));
                    }
                }
                assert!(region.boolean_region(&restored, BooleanOp::Xor, &policy).unwrap().into_value().is_empty());
            }
            let joined = CurveRegion2::try_from_native_boundary_contours(&[rectangle(0,0,4,4), rectangle(4,0,8,4)], fill, &policy).unwrap().into_value();
            let grown = joined.offset(Real::one(), &OffsetCornerStyle2::Round, &policy).unwrap().into_value();
            for (point, expected) in [(p(4,0), RegionPointLocation::Inside), (p(4,-1), RegionPointLocation::Boundary), (p(4,-2), RegionPointLocation::Outside)] {
                assert_eq!(grown.classify_point(&point.into(), &policy).unwrap().into_value(), Classification::Decided(expected));
            }
        }
    }
}

#[test]
fn native_boundary_global_fill_retains_winding_and_recursive_islands() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for contour_fill in [FillRule::EvenOdd, FillRule::NonZero] {
            let outer = rectangle(0, 0, 10, 10);
            let doubled = Contour2::try_new_with_fill_rule(outer.segments().iter().chain(outer.segments()).cloned().collect(), contour_fill).unwrap();
            for fill in [FillRule::EvenOdd, FillRule::NonZero] {
                let contours = [doubled.clone(), reversed_rectangle(2,2,8,8)];
                let region = CurveRegion2::try_from_native_boundary_contours(&contours, fill, &policy).unwrap().into_value();
                for (point, expected) in [(p(1,1), if fill == FillRule::EvenOdd {RegionPointLocation::Outside} else {RegionPointLocation::Inside}), (p(5,5), RegionPointLocation::Inside), (p(11,5), RegionPointLocation::Outside)] {
                    assert_eq!(region.classify_point(&point.into(), &policy).unwrap().into_value(), Classification::Decided(expected));
                }
                let opposite = Contour2::try_new(outer.segments().iter().rev().map(Segment2::reversed).collect()).unwrap();
                assert!(CurveRegion2::try_from_native_boundary_contours(&[outer.clone(), opposite], fill, &policy).unwrap().into_value().is_empty());
            }
        }
        let mut nested = (0..5).map(|i| rectangle(i*2,i*2,20-i*2,20-i*2)).collect::<Vec<_>>();
        for reverse_order in [false,true] {
            if reverse_order {nested.reverse();}
            let region = CurveRegion2::try_from_native_boundary_contours(&nested, FillRule::EvenOdd, &policy).unwrap().into_value();
            assert_eq!(region.loop_role_counts(&policy).unwrap().into_value(), Classification::Decided((3,2)));
            for i in 0..5 {
                let expected = if i % 2 == 0 {RegionPointLocation::Inside} else {RegionPointLocation::Outside};
                assert_eq!(region.classify_point(&p(2*i+1,10).into(), &policy).unwrap().into_value(), Classification::Decided(expected));
            }
        }
    }
}

#[test]
fn native_boundary_circle_fills_reenter_exact_boolean_operations() {
    use crate::BooleanOp;
    let circle = |x| Contour2::try_new(vec![Segment2::Arc(arc_bulge(x-2,0,x+2,0,1)), Segment2::Arc(arc_bulge(x+2,0,x-2,0,1))]).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill in [FillRule::EvenOdd, FillRule::NonZero] {
            for separation in [2,4] {
                let region = CurveRegion2::try_from_native_boundary_contours(&[circle(0),circle(separation)], fill, &policy).unwrap().into_value();
                let expected = if separation==4 {RegionPointLocation::Boundary} else if fill==FillRule::NonZero {RegionPointLocation::Inside} else {RegionPointLocation::Outside};
                assert_eq!(region.classify_point(&p(separation/2,0).into(), &policy).unwrap().into_value(), Classification::Decided(expected));
                let restored = region.boolean_region(&region, BooleanOp::Intersection, &policy).unwrap().into_value();
                assert_eq!(restored.classify_point(&p(separation/2,0).into(), &policy).unwrap().into_value(), Classification::Decided(expected));
                assert!(restored.boolean_region(&region, BooleanOp::Xor, &policy).unwrap().into_value().is_empty());
            }
        }
    }
}

'''+s[b:]
 return s
change('hypercurve/src/native_region_tests.rs',native)
def bench(s):
 a=s.index('        let Classification::Decided(region) = CurveRegion2::try_from_native_boundary_contours(')
 b=s.index('        total_roles +=',a)
 return s[:a]+'''        let region = CurveRegion2::try_from_native_boundary_contours(
            &[material.clone(), hole.clone(), island.clone()],
            FillRule::EvenOdd,
            &policy,
        )
        .expect("native boundary construction must evaluate")
        .into_value();
'''+s[b:]
change('hypercurve/benches/editing.rs',bench)
def promotion(s):
 old='''    let nested = decided(
        CurveRegion2::try_from_native_boundary_contours(boundaries, &policy)
            .unwrap()
            .into_value(),
    );'''
 assert old in s
 return s.replace(old,'''    let nested = CurveRegion2::try_from_native_boundary_contours(&boundaries, FillRule::EvenOdd, &policy)
        .unwrap()
        .into_value();''')
change('hypercurve/tests/hypercurve_curve_region_promotion.rs',promotion)
def docs(s):
 old='''  winding multiplicity is resolved during construction.'''
 assert s.count(old)==1
 return s.replace(old,old+'''
  Boundary-path and native-boundary constructors take one explicit global fill
  rule and return a regularized region directly. Native contours may cross,
  overlap, or touch; their individual contour fill rules do not override the
  chosen compound fill. Explicit material/hole constructors retain per-loop
  fill semantics.''')
change('hypercurve/README.md',docs)
def gerber(s):
 s=s.replace('FiniteRegionProfile2,','FiniteRegionProfile2, FillRule,')
 a=s.index('        let region = match decisions.consume_curve(\n            CurveRegion2::try_from_native_boundary_contours(');b=s.index('        Ok(region)',a)
 return s[:a]+'''        let region = decisions.consume_curve(
            CurveRegion2::try_from_native_boundary_contours(
                &contours,
                FillRule::EvenOdd,
                decisions.curve_policy(),
            )
            .map_err(|error| IoError::Geometry {
                format: "Gerber",
                detail: error.to_string(),
            })?,
        );
'''+s[b:]
change('csgrs/src/io/gerber.rs',gerber)
(A/'native-boundary-admission-base-v777.json').write_text(json.dumps(base,indent=2)+'\n')
print('Prepared',len(base),'candidate sources')
