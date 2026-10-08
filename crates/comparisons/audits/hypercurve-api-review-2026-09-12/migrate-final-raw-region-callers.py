from pathlib import Path
import re
w=Path('/home/tim/Documents/GitHub/workspace/hypercurve')
p=w/'tests/hypercurve_bezier_region.rs'
s=p.read_text()
original=s

def body(text,name):
    start=text.index('fn '+name+'(')
    end=start+re.search(r'(?m)^}',text[start:]).end()
    return text[start:end]

def replace_fn(name,new):
    global s
    old=body(s,name)
    s=s.replace(old,new,1)

def remove_test(name):
    global s
    old='#[test]\n'+body(s,name)+'\n\n'
    assert old in s
    s=s.replace(old,'',1)

for name in ['retained_region_constructor_rejects_reused_arrangement_sources_across_loops','native_boundary_loops_convert_into_unified_region_validation','retained_region_constructor_rejects_duplicate_boundary_loops']:
    remove_test(name)
for name in ['retained_region','retained_line_loop_with_sources']:
    s=s.replace(body(s,name)+'\n\n','',1)
s=s.replace('CurveCertainty, CurveContext, CurveError, CurveOutcome, CurvePoint2, CurveRegion2,','Curve2, CurveCertainty, CurveContext, CurveError, CurveOutcome, CurvePath2, CurvePoint2, CurveRegion2,')
replace_fn('retained_algebraic_endpoint_line_fragment','''fn algebraic_endpoint_line(start: Point2, end: Point2) -> Curve2 {
    let far = Point2::new(
        end.x() + (end.x() - start.x()),
        end.y() + (end.y() - start.y()),
    );
    Curve2::from(QuadraticBezier2::new(start, end, far))
        .subcurve(
            Real::zero().into(),
            BezierParameter2::algebraic(algebraic_midpoint_parameter()).into(),
            &policy(),
        )
        .unwrap()
        .into_value()
}''')
replace_fn('retained_line_loop','''fn quadratic_polygon_path(vertices: &[Point2]) -> CurvePath2 {
    CurvePath2::try_new(
        (0..vertices.len())
            .map(|i| {
                let start = &vertices[i];
                let end = &vertices[(i + 1) % vertices.len()];
                QuadraticBezier2::new(start.clone(), start.lerp(end, q(1, 2)), end.clone()).into()
            })
            .collect(),
    )
    .unwrap()
}''')
s=s.replace('retained_line_loop(', 'quadratic_polygon_path(')
s=s.replace('retained_algebraic_endpoint_line_fragment(', 'algebraic_endpoint_line(')
s=s.replace('''    let retained = retained_region(vec![outer, same_orientation_inner]);
    assert!(retained.boundary_loops()[0].arrangement_sources().is_none());''','''    let retained = CurveRegion2::try_from_boundary_paths(&[outer, same_orientation_inner], &policy())
        .unwrap().into_value();''',1)
replace_fn('retained_algebraic_line_images_reject_crossing_loops_under_both_policies','''fn retained_algebraic_line_images_normalize_crossing_loops_under_both_policies() {
    let exact_path = |points: &[(Point2, Point2)]| {
        CurvePath2::try_new_with_policy(
            points.iter().cloned().map(|(start, end)| algebraic_endpoint_line(start, end)).collect(),
            &policy(),
        ).unwrap().into_value()
    };
    let paths = [
        exact_path(&[(p(0, 0), p(4, 0)), (p(4, 0), p(4, 4)), (p(4, 4), p(0, 4)), (p(0, 4), p(0, 0))]),
        exact_path(&[(p(2, -1), p(6, -1)), (p(6, -1), p(6, 3)), (p(6, 3), p(2, 3)), (p(2, 3), p(2, -1))]),
    ];
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let outcome = CurveRegion2::try_from_boundary_paths(&paths, &policy).unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
        let retained = outcome.into_value();
        assert_eq!(decided(retained.loop_roles(&policy).unwrap()), vec![CurveRegionLoopRole::Material; 2]);
        assert_eq!(decided(retained.filled_side_is_left(&policy).unwrap()), &[true; 2]);
        assert_eq!(decided(retained.filled_area(&policy).unwrap()), Some(r(20)));
        for (point, expected) in [
            (p(1, 1), RegionPointLocation::Inside),
            (p(5, 1), RegionPointLocation::Inside),
            (p(3, 1), RegionPointLocation::Outside),
            (p(7, 1), RegionPointLocation::Outside),
            (p(0, 2), RegionPointLocation::Boundary),
            (p(2, 0), RegionPointLocation::Boundary),
            (p(4, 3), RegionPointLocation::Boundary),
        ] {
            assert_eq!(decided(retained.classify_point(&point, &policy).unwrap()), expected);
        }
        assert_eq!(retained.regularized_region(&policy).unwrap().into_value(), retained);
    }
}''')
t=body(s,'retained_exact_algebraic_endpoint_line_images_assign_roles')
t=t.replace('retained_loop(vec![','CurvePath2::try_new_with_policy(vec![')
t=t.replace('''    ]);
''','''    ], &policy()).unwrap().into_value();
''')
t=t.replace('let retained = retained_region(vec![outer, same_orientation_inner]);','let retained = CurveRegion2::try_from_boundary_paths(&[outer, same_orientation_inner], &policy()).unwrap().into_value();')
replace_fn('retained_exact_algebraic_endpoint_line_images_assign_roles',t)
t=body(s,'retained_nonlinear_algebraic_carriers_classify_without_materialization')
start=t.index('    let upper = ')
end=t.index('    let clone = region.clone();')
t=t[:start]+'''    let upper = Curve2::from(QuadraticBezier2::new(p(-1, 0), p(0, 2), p(1, 0)));
    let parameter = BezierParameter2::algebraic(algebraic_sqrt_half_parameter()).into();
    let split = upper.split_at(parameter, &policy).unwrap();
    assert_eq!(split.certainty, CurveCertainty::Certified);
    let (first, second) = split.into_value();
    assert!(first.end().coordinates().is_none());
    assert!(second.start().coordinates().is_none());
    assert!(decided(first.end().coincides_with(&second.start(), &policy)));
    let lower = Curve2::from(QuadraticBezier2::new(p(1, 0), p(0, -2), p(-1, 0)));
    let path = CurvePath2::try_new_with_policy(vec![first, second, lower], &policy)
        .unwrap().into_value();
    let region = CurveRegion2::try_from_boundary_paths(&[path], &policy).unwrap().into_value();
'''+t[end:]
replace_fn('retained_nonlinear_algebraic_carriers_classify_without_materialization',t)
t=body(s,'retained_certified_nonlinear_line_image_uses_authoritative_roles')
start=t.index('    let nonlinear_edge')
end=t.index('    assert_eq!',start)
t=t[:start]+'''    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(p(0, 0), p(1, 0), p(4, 0)).into(),
        QuadraticBezier2::new(p(4, 0), p(4, 2), p(4, 4)).into(),
        QuadraticBezier2::new(p(4, 4), p(2, 4), p(0, 4)).into(),
        QuadraticBezier2::new(p(0, 4), p(0, 2), p(0, 0)).into(),
    ]).unwrap();
    let retained = CurveRegion2::try_from_boundary_paths(&[path], &policy()).unwrap().into_value();

'''+t[end:]
replace_fn('retained_certified_nonlinear_line_image_uses_authoritative_roles',t)
replace_fn('retained_quadratic_lens_loop','''fn quadratic_lens_path(left_x: i32, right_x: i32, height: i32) -> CurvePath2 {
    CurvePath2::try_new(vec![
        QuadraticBezier2::new(p(left_x, 0), p((left_x + right_x) / 2, height), p(right_x, 0)).into(),
        QuadraticBezier2::new(p(right_x, 0), p((left_x + right_x) / 2, -height), p(left_x, 0)).into(),
    ]).unwrap()
}''')
t=body(s,'retained_curved_nesting_role_evidence_assigns_same_orientation_nonlinear_hole')
t=t.replace('retained_quadratic_lens_loop(0, 8, 4, true)','quadratic_lens_path(0, 8, 4)')
t=t.replace('retained_quadratic_lens_loop(2, 6, 1, true)','quadratic_lens_path(2, 6, 1)')
t=t.replace('let retained = retained_region(vec![material, same_orientation_inner]);','let retained = CurveRegion2::try_from_boundary_paths(&[material, same_orientation_inner], &policy()).unwrap().into_value();')
t=t.replace('curved nesting evidence records absence of graph provenance per loop','curved nesting evidence retains normalized graph provenance per loop')
t=t.replace('''    assert!(nesting_sources[0].is_none());
    assert!(nesting_sources[1].is_none());''','''    for (sources, boundary) in nesting_sources.iter().zip(retained.boundary_loops()) {
        assert_eq!(sources.as_deref(), boundary.arrangement_sources());
    }''')
t=t.replace('nesting.signed_areas()[0], q(-64, 3)','nesting.signed_areas()[0], q(64, 3)')
replace_fn('retained_curved_nesting_role_evidence_assigns_same_orientation_nonlinear_hole',t)
t=body(s,'material_components_keep_recursive_hole_ownership_and_recompose_exactly')
t=t.replace('''        // During the raw-constructor migration, decomposition still has to
        // normalize any authored loops it receives before assigning ownership.
        let authored = CurveRegion2::try_new_with_loop_topology(''','''        // Admission removes the inner filled seam before component extraction.
        let authored = CurveRegion2::try_from_boundary_paths_with_loop_topology(''')
t=t.replace('''            vec![
                quadratic_polygon_path''','''            &[
                quadratic_polygon_path''')
t=t.replace('vec![CurveRegionLoopRole::Material; 2]','&[CurveRegionLoopRole::Material; 2]')
t=t.replace('vec![hypercurve::FillRule::NonZero; 2]','&[hypercurve::FillRule::NonZero; 2]')
t=t.replace('vec![hypercurve::CurveBoundaryInteriorSide2::Left; 2]','&[hypercurve::CurveBoundaryInteriorSide2::Left; 2]')
t=t.replace('''            &[hypercurve::CurveBoundaryInteriorSide2::Left; 2],
        )
        .unwrap();''','''            &[hypercurve::CurveBoundaryInteriorSide2::Left; 2],
            &policy,
        ).unwrap().into_value();
        assert_eq!(authored.len(), 1);
        assert_eq!(decided(authored.classify_point(&p(2, 4), &policy).unwrap()), RegionPointLocation::Inside);''')
replace_fn('material_components_keep_recursive_hole_ownership_and_recompose_exactly',t)
assert 'CurveRegion2::new(' not in s
assert 'CurveRegion2::try_new_with_loop_topology(' not in s
assert 'retained_region(' not in s
p.write_text(s)

p=w/'src/bezier_region.rs'
s=p.read_text()
s=s.replace('''    /// Constructs an exact curved region from already materialized boundary loops.
    pub fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> CurveResult<Self> {''','''    /// Validates raw boundary collections for internal normalization.
    pub(crate) fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> CurveResult<Self> {''')
s=s.replace('''    /// Constructs retained exact loops with explicit role, fill, and interior-side topology.
    ///
    /// This is the authoritative constructor for procedural carriers whose
    /// Green integral is not represented as a native [`Real`], including
    /// analytic Bezier parallels. The interior side is authored topology
    /// evidence; it is never inferred from a finite projection.
    pub fn try_new_with_loop_topology(''','''    /// Attaches authored role, fill, and interior-side hints before normalization.
    ///
    /// These hints retain procedural carrier semantics without requiring a
    /// represented Green integral. They do not certify a regularized boundary;
    /// public path admission normalizes the resulting internal region.
    pub(crate) fn try_new_with_loop_topology(''')
unit='''    #[test]
    fn retained_region_constructor_rejects_reused_arrangement_sources_across_loops() {
        let boundary = |vertices: &[Point2], sources| {
            let fragments = (0..vertices.len()).map(|i| {
                let start = &vertices[i];
                let end = &vertices[(i + 1) % vertices.len()];
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        start.clone(), start.lerp(end, q(1, 2)), end.clone(),
                    )),
                }
            }).collect();
            CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(
                fragments, sources, &CurveContext::STRICT,
            ).unwrap()
        };
        let outer = boundary(&[p(0, 0), p(6, 0), p(6, 6), p(0, 6)], vec![
            CurveRegionFragmentSource2::new(0, 0, 0),
            CurveRegionFragmentSource2::new(1, 0, 1),
            CurveRegionFragmentSource2::new(2, 0, 2),
            CurveRegionFragmentSource2::new(3, 0, 3),
        ]);
        let inner = boundary(&[p(2, 2), p(4, 2), p(4, 4), p(2, 4)], vec![
            CurveRegionFragmentSource2::new(0, 1, 0),
            CurveRegionFragmentSource2::new(4, 1, 1),
            CurveRegionFragmentSource2::new(5, 1, 2),
            CurveRegionFragmentSource2::new(6, 1, 3),
        ]);
        assert!(matches!(CurveRegion2::new(vec![outer, inner]), Err(CurveError::Topology(_))));
    }

    #[test]
    fn native_boundary_loops_convert_into_unified_region_validation() {
        let boundary = BezierBoundaryLoop2::new(vec![
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0))),
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(1, -1), p(0, 0))),
        ], &CurveContext::STRICT).unwrap();
        let boundary: CurveRegionBoundaryLoop2 = boundary.into();
        assert!(matches!(CurveRegion2::new(vec![boundary.clone(), boundary]), Err(CurveError::Topology(_))));
    }

    #[test]
    fn retained_region_constructor_rejects_duplicate_boundary_loops() {
        let chord = |start: Point2, end: Point2| {
            let Classification::Decided(chord) = crate::BezierAlgebraicChord2::try_new(
                start.into(), end.into(), &CurveContext::STRICT,
            ).unwrap() else { panic!("an exact chord is represented"); };
            BezierSplitFragment2::AlgebraicChord(chord)
        };
        let boundary = CurveRegionBoundaryLoop2::new(vec![
            chord(p(0, 0), p(1, 0)), chord(p(1, 0), p(0, 0)),
        ], &CurveContext::STRICT).unwrap();
        assert!(matches!(CurveRegion2::new(vec![boundary.clone(), boundary]), Err(CurveError::Topology(_))));
    }

'''
needle='    #[test]\n    fn material_component_reentry_shares_single_region_evidence()'
assert needle in s
s=s.replace(needle,unit+needle,1)
assert 'pub fn try_new_with_loop_topology(' not in s
assert 'pub fn new(boundary_loops:' not in s
p.write_text(s)
print('Migrated the final five public raw constructor calls, moved three internal validation tests, and made both raw factories crate-private.')
