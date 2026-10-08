from pathlib import Path
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_offset.rs')
s=p.read_text()
def one(old,new,count=1):
    global s
    assert s.count(old)==count,(old[:100],s.count(old),count)
    s=s.replace(old,new)
def section(start,end,new=''):
    global s
    a=s.index(start); b=s.index(end,a+len(start)); s=s[:a]+new+s[b:]
one('''    /// Same oriented source tangent after a caller-certified positive affine
    /// reparameterization of an edited boundary envelope. The center remains
    /// on its original correlated support; this optional one-word authority
    /// lets later joins replay the canonical boundary tangent without
    /// adjoining or reconstructing either selected parameter field.
    tangent_authority: Option<Arc<BezierSelectedParallelTangentAuthorityData2>>,
''','')
section('#[derive(Debug, PartialEq)]\nstruct BezierSelectedParallelTangentAuthorityData2 {', '#[derive(Debug, PartialEq)]\nstruct BezierSelectedChordNormalFrameData2 {')
section('                    tangent_authority: match frame.tangent_authority.as_ref() {','                    policy: frame.policy,')
one('                        tangent_authority: None,\n','')
section('/// Maps one edited-carrier parameter into its selected-circle source chart.', 'impl BezierAlgebraicCuspSemicircle2 {')
section('    /// Retains the parameter-zero tangent after an edit has moved the anchor','    /// Builds a selected circle around arbitrary retained center evidence,')
section('    /// Reuses the complete same-source selected-normal kernel after a retained','    fn finite_parallel_intersections_from_rational_component(')
section('    /// Re-expresses the target axis through the certified positive affine','    fn mapped_data(')
a=s.index('        let shares_tangent_parameter = if other.source() == frame.center_support.source() {')
b=s.index('        if !shares_tangent_parameter {',a)
s=s[:a]+'''        let shares_tangent_parameter = other.source() == frame.center_support.source()
            && frame.center_parameter.same_value(
                &contact.parallel_parameter.clone().into(), policy,
            )? == Classification::Decided(true);
'''+s[b:]
one('''    /// endpoint carried through the same source tangent (or its certified
    /// positive affine reparameterization).''','''    /// endpoint retained in the same source chart.''')
section('        if !same_source\n            && let Some(frame) = self.data.frame.parallel_normal()', '        let normalize_source_reversal =')
section('        let (tangent_support, tangent_parameter) = frame.tangent_authority.as_ref().map_or_else(', '        let tangent_parameter =', '')
one('''match promote_curve_region_bezier_parameter(&tangent_parameter, policy)?''','''match promote_curve_region_bezier_parameter(&frame.center_parameter, policy)?''')
one('''            tangent_support
                .source_tangent_pair_cross_dot_linear_combination_sign(''','''            frame.center_support
                .source_tangent_pair_cross_dot_linear_combination_sign(''')
# Keep chart composition coverage on the parameter itself, without rebuilding its support.
section('            // Q(u)=P(2u) uses the same positive source tangent.', '            let retained_frame_parameter = local_frame.selected_frame_parameter().unwrap();', '''            // Optional scalar charts compose back to the original selected
            // root. The circle and every retained support stay in that source
            // chart, so tangent reattachment needs no separate map.
            let Classification::Decided(mapped) = roots[0]
                .affine_image_unbounded(&half, &Real::zero(), &policy).unwrap()
            else { panic!("the local scalar chart must construct") };
            let Classification::Decided(restored) = mapped
                .affine_image_unbounded(&Real::from(2_i8), &Real::zero(), &policy).unwrap()
            else { panic!("the inverse scalar chart must construct") };
            assert!(Arc::ptr_eq(
                &selected.data, &restored.as_recursive_projective().unwrap().data,
            ));
''')
section('    #[test]\n    fn selected_affine_tangent_source_retains_incident_endpoint_and_contacts_locally() {','    #[test]\n    fn selected_fiber_interval_supports_exact_and_nonexact_exterior_roots() {','''    #[test]
    fn selected_source_range_retains_incident_endpoint_and_contacts_locally() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        // P(u)=(u,0). The unit circle centered at u=1/4+2s meets
        // [2s,2s+2] once, at u=5/4+2s, including exterior source ranges.
        let support = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(half.clone(), Real::zero()),
            Point2::from_values(1, 0),
        ).parallel_left(Real::zero()).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for shift in [-3, 0, 2] {
                let start = Real::from(2 * shift);
                let center_parameter = CurveParameter2::from(&quarter + &start);
                let expected = &start + &quarter + Real::one();
                let end = &start + Real::from(2_i8);
                let Classification::Decided(Some(circle)) =
                    BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                        support.clone(), center_parameter, Real::one(), false, &policy,
                    ).unwrap()
                else { panic!("the selected-normal circle must construct") };
                let endpoint = CurveParameter2::from_selected_fiber(
                    degree_nine_selected_fiber_parameter_for_test(half.clone(), 32_768, &policy),
                );
                let Classification::Decided(endpoint) = endpoint
                    .affine_image_unbounded(&Real::from(2_i8), &start, &policy).unwrap()
                else { panic!("the finite endpoint must remain in its local fiber") };
                assert!(matches!(
                    endpoint.as_selected_fiber().unwrap().promoted_bezier_parameter(&policy).unwrap(),
                    Classification::Uncertain(_),
                ));
                assert_eq!(endpoint.cmp_by_refinement(&expected.clone().into(), &policy).unwrap(),
                    Classification::Decided(std::cmp::Ordering::Less));
                for lower in [CurveParameter2::from(start), endpoint] {
                    for reversed in [false, true] {
                        let range = if reversed {
                            CurveParameterRange2::new_validated(end.clone().into(), lower.clone())
                        } else {
                            CurveParameterRange2::new_validated(lower.clone(), end.clone().into())
                        };
                        let mut retained = Vec::new();
                        for half_circle in [circle.clone(), circle.complementary_half()] {
                            let result = half_circle.parallel_intersections(
                                &support, &range, None, &policy,
                            ).unwrap();
                            let Classification::Decided(
                                BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber {
                                    contacts, overlaps,
                                },
                            ) = result else {
                                panic!("the finite source range must retain local contacts")
                            };
                            assert!(overlaps.is_empty());
                            retained.extend(contacts);
                        }
                        assert_eq!(retained.len(), 1);
                        let parameter = retained[0].other_parameter();
                        assert!(parameter.represented_value() == Some(&expected));
                        assert_eq!(parameter.cmp_bezier_parameter(
                            &BezierParameter2::Exact(expected.clone()), &policy,
                        ).unwrap(), Classification::Decided(std::cmp::Ordering::Equal));
                    }
                }
            }
        }
    }

''')
assert 'BezierSelectedParallelTangentAuthorityData2' not in s
assert 'frame.tangent_authority' not in s
assert 'affine_tangent_source_region_parameter' not in s
assert 'affine_target_parameterization(' not in s
if s.count('bivariate_affine_second_parameter')==1:
    a=s.index('fn bivariate_affine_second_parameter(')
    # This helper has no documentation block and is followed by another fn.
    b=s.index('\nfn ',a+3)
    s=s[:a]+s[b+1:]
p.write_text(s)
print('Removed obsolete affine tangent authority and transport; tests now retain source ranges')
