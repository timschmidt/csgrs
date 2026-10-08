from pathlib import Path
W=Path(__file__).resolve().parent.parent
p=W/'hypercurve/src/bezier_offset.rs'
s=p.read_text()
a=s.index('    fn selected_source_range_retains_incident_endpoint_and_contacts_locally()')
b=s.index('    #[test]',a)
t=s[a:b]
old='''                            let Classification::Decided(
                                BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber {
                                    contacts, overlaps,
                                },
                            ) = result else {
                                panic!("the finite source range must retain local contacts")
                            };
                            assert!(overlaps.is_empty());
                            retained.extend(contacts);
'''
new='''                            match result {
                                Classification::Decided(
                                    BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber {
                                        contacts, overlaps,
                                    },
                                ) => {
                                    assert!(overlaps.is_empty());
                                    retained.extend(contacts.into_iter().map(|contact|
                                        CurveParameter2::from_selected_fiber(contact.other_parameter().clone())
                                    ));
                                }
                                Classification::Decided(
                                    BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
                                        contacts, overlaps,
                                    },
                                ) => {
                                    assert!(overlaps.is_empty());
                                    retained.extend(contacts.into_iter().map(|contact|
                                        CurveParameter2::from(contact.parallel_parameter)
                                    ));
                                }
                                Classification::Decided(
                                    BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(contacts),
                                ) => retained.extend(contacts.into_iter().map(|contact|
                                    contact.other_parameter().clone()
                                )),
                                Classification::Uncertain(reason) => panic!("finite source contact uncertainty: {reason:?}"),
                                Classification::Decided(_) => panic!("a straight source and circle must have isolated contacts"),
                            }
'''
assert old in t
t=t.replace(old,new)
old='''                        let parameter = retained[0].other_parameter();
                        assert!(parameter.represented_value() == Some(&expected));
                        assert_eq!(
                            parameter
                                .cmp_bezier_parameter(
                                    &BezierParameter2::Exact(expected.clone()),
                                    &policy,
                                )
                                .unwrap(),
'''
new='''                        assert_eq!(
                            retained[0]
                                .cmp_by_refinement(&CurveParameter2::from(expected.clone()), &policy)
                                .unwrap(),
'''
assert old in t
t=t.replace(old,new)
s=s[:a]+t+s[b:]
p.write_text(s)
p=W/'hypercurve/src/bezier_region.rs'
s=p.read_text()
a=s.index('    fn one_fragment_materialized_loop_extends_algebraic_chamfer_cuts_once()')
b=s.index('    #[test]',a)
t=s[a:b]
old='''                        let source = Curve2::from(
                            selected
                                .rational_curve()
                                .expect("the exact cubic envelope")
                                .clone(),
                        );'''
new='''                        let source = selected
                            .rational_curve()
                            .expect("the retained cubic source")
                            .parallel_left(Real::zero()).unwrap();'''
assert old in t
t=t.replace(old,new)
old='''                            let replay = source.point_at(parameter, &policy).unwrap();
                            let equality = replay.value.coincides_with(point, &policy);
                            assert_eq!(replay.certainty, CurveCertainty::Certified);'''
new='''                            // Evaluate the support independently of its stored endpoints.
                            // The exact source parameter may lie outside [0, 1].
                            let replay = CurvePoint2::from(
                                crate::BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                                    source.clone(), parameter, Real::zero(), &policy,
                                ).expect("a cubic source accepts each retained scalar")
                            );
                            let equality = replay.coincides_with(point, &policy);'''
assert old in t
t=t.replace(old,new).replace('one common envelope must retain its two algebraic cuts','one source range must retain its two algebraic cuts')
s=s[:a]+t+s[b:]
for name in ['one_fragment_selected_loop_extends_chamfer_cuts_on_its_analytic_carrier','one_fragment_retained_ph_loop_extends_fillet_on_one_analytic_carrier']:
    a=s.index('    fn '+name+'(')
    b=s.find('    #[test]',a)
    if name.startswith('one_fragment_retained_ph'):
        b=s.index('    fn parallel_pair_fillet_region(',a)
    t=s[a:b]
    key='''let mut found_extension = false;'''
    assert t.count(key)==1
    t=t.replace(key,'''let original_support = CurveSupport2::from_fragment(
                        &region.boundary_loops()[0].fragments()[0],
                    );
                    let unit = CurveParameterRange2::unit();
                    let mut found_extension = false;''')
    old='''matches!(fragment, BezierSplitFragment2::AnalyticParallel(_))'''
    assert t.count(old)==1
    t=t.replace(old,'''CurveSupport2::from_fragment(fragment) == original_support
                                        && matches!(
                                            crate::bezier_split::CurveParameterDomain2::new(&unit, None)
                                                .contains_finite_range(&fragment.curve_region_parameter_range(), &policy)
                                                .unwrap(),
                                            Classification::Decided(false),
                                        )''')
    t=t.replace('the extended analytic range was lost','the exterior interval must retain its original support').replace('the extended analytic interval was lost','the exterior interval must retain its original support')
    s=s[:a]+t+s[b:]
a=s.index('    fn one_fragment_selected_projective_extensions_keep_the_local_fiber()')
b=s.index('    #[test]',a)
t=s[a:b]
key='''                assert_eq!(replacement_fragment.is_reversed(), reversed);'''
assert t.count(key)==1
t=t.replace(key,key+'''
                assert!(replacement_fragment.start_point().shares_storage(&next_cut.point));
                assert!(replacement_fragment.end_point().shares_storage(&previous_cut.point));''')
key='''                    assert_eq!(rebuilt.is_reversed(), reversed);'''
assert t.count(key)==1
t=t.replace(key,key+'''
                    let BezierSplitFragment2::SelectedFiber(original) = &fragment else { unreachable!() };
                    if previous {
                        assert!(rebuilt.start_point().shares_storage(original.start_point()));
                        assert!(rebuilt.end_point().shares_storage(&one_cut.point));
                    } else {
                        assert!(rebuilt.start_point().shares_storage(&one_cut.point));
                        assert!(rebuilt.end_point().shares_storage(original.end_point()));
                    }''')
s=s[:a]+t+s[b:]
s=s.replace('native selected cuts must not require a finite envelope','native selected cuts must keep their source chart')
p.write_text(s)
print('Updated geometric contact and exterior source oracles; added endpoint storage identity checks.')
