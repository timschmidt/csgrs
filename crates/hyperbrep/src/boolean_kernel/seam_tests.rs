use super::*;
use crate::Vector3;

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

#[test]
fn collinear_trace_union_keeps_tiny_gaps_and_distinct_curved_images() {
    let mut traces = Vec::new();
    for (a, b) in [(0, 2), (4, 2), (1, 3), (7, 6)] {
        push_unique_planar_trace(&mut traces, Curve3::line(p(a, 0, 0), p(b, 0, 0)).unwrap())
            .unwrap();
    }
    assert_eq!(traces.len(), 2);
    let delta: Real = "1e-12".parse().unwrap();
    push_unique_planar_trace(
        &mut traces,
        Curve3::line(
            Point3::new(Real::from(4) + delta, 0.into(), 0.into()),
            p(5, 0, 0),
        )
        .unwrap(),
    )
    .unwrap();
    assert_eq!(traces.len(), 3);
    let mut arcs = Vec::new();
    for y in [Vector3::y(), -Vector3::y()] {
        push_unique_planar_trace(&mut arcs, circle(Vector3::x(), y, 0.into(), Real::pi())).unwrap();
    }
    assert_eq!(arcs.len(), 2);
}

#[test]
fn partial_circle_dedup_uses_complete_anchored_map() {
    let plane = Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap();
    let retain = |c| SurfaceIntersectionCurve::on_plane(c, &plane).unwrap();
    let quarter = (Real::pi() / Real::from(2)).unwrap();
    let first = circle(Vector3::x(), Vector3::y(), 0.into(), quarter.clone());
    let equivalent = circle(Vector3::y(), Vector3::x(), 0.into(), quarter.clone());
    let result = circle_traces::coalesce_planar_images(
        &plane,
        vec![retain(first.clone()), retain(equivalent)],
    )
    .unwrap();
    assert_eq!(result.len(), 1);
    let complement = circle(Vector3::y(), -Vector3::x(), 0.into(), Real::tau() - quarter);
    let result =
        circle_traces::coalesce_planar_images(&plane, vec![retain(first), retain(complement)])
            .unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(
        exact_order(
            &(result[0].curve().domain().represented_end().unwrap()
                - result[0].curve().domain().represented_start().unwrap()),
            &Real::tau()
        )
        .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn rational_torus_latitude_retains_contact_through_all_incident_faces() {
    let (source, solid) = crate::construction::torus(10.into(), 5.into()).unwrap();
    let (cutter, cutter_solid) =
        crate::construction::cuboid(p(-40, -40, 3), p(40, 40, 40)).unwrap();
    let ArenaBooleanResult::Solid { model, solid, .. } = sets::boolean(
        &source,
        &[solid],
        &cutter,
        &[cutter_solid],
        BooleanOperation::Intersection,
    )
    .unwrap() else {
        panic!("one toroidal band")
    };
    // Integrating 4*pi*R*sqrt(r^2-z^2) from 3 to 5 gives this volume.
    let a = (Real::from(3) / Real::from(5)).unwrap().asin().unwrap();
    let expected = Real::from(20)
        * Real::pi()
        * (Real::from(25) * ((Real::pi() / Real::from(2)).unwrap() - a) - Real::from(12));
    assert_eq!(
        exact_order(&model.solid_volume(solid).unwrap(), &expected).unwrap(),
        Ordering::Equal
    );
    for (point, expected) in [
        (p(10, 0, 4), crate::SolidPointLocation::Inside),
        (p(10, 0, 2), crate::SolidPointLocation::Outside),
        (p(6, 0, 3), crate::SolidPointLocation::Boundary),
        (p(14, 0, 3), crate::SolidPointLocation::Boundary),
    ] {
        assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
    }
    let replay = |model: &ArenaModel, label: &str| {
        let json = model.to_json().unwrap();
        let loaded =
            crate::persistence::ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
                .unwrap()
                .validate()
                .unwrap_or_else(|error| panic!("{label}: {error:?}"));
        assert_eq!(loaded.to_json().unwrap(), json);
        assert_eq!(
            exact_order(&loaded.solid_volume(solid).unwrap(), &expected).unwrap(),
            Ordering::Equal
        );
        loaded
    };
    let mut loaded = replay(&model, "before subdivision");
    for iteration in 0..4 {
        let (edge_id, edge) = loaded.edges().next().unwrap();
        let parameter = ((edge.domain().start().try_real().unwrap()
            + edge.domain().end().try_real().unwrap())
            / Real::from(2))
        .unwrap();
        let (split, _) = loaded.split_edge(edge_id, parameter.into()).unwrap();
        loaded = replay(&split, &format!("after subdivision {}", iteration + 1));
    }
}
fn circle(x: Vector3, y: Vector3, start: Real, end: Real) -> Curve3 {
    Curve3::circle_arc(p(0, 0, 0), x, y, 2.into(), start, end).unwrap()
}

#[test]
fn circular_clip_preimages_preserve_chart_scope_and_both_seam_ends() {
    let plane = Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap();
    let curve = circle(Vector3::x(), Vector3::y(), Real::zero(), Real::tau());
    let retained = SurfaceIntersectionCurve::on_plane(curve.clone(), &plane).unwrap();
    let carrier = retained.first_pcurve().materialize().unwrap();
    let start = Real::zero();
    let end = Real::one();
    let middle = (Real::one() / Real::from(2)).unwrap();
    let contact = |source: Curve3, parameter: Real| {
        SurfaceIntersectionBoundaryContact::new(
            source,
            parameter,
            None,
            hypercurve::Point2::new((-2).into(), 0.into()),
            p(-2, 0, 0),
        )
    };
    let hint = contact(curve.clone(), Real::pi());
    let actual = retained_spatial_parameter(&carrier, &middle, &[hint], &curve).unwrap();
    assert_eq!(exact_order(&actual, &Real::pi()).unwrap(), Ordering::Equal);
    // These source curves pass through the same point at pi/2. Neither that
    // shared point nor a shared circle support proves equal parameterization.
    for source in [
        circle(Vector3::y(), -Vector3::x(), Real::zero(), Real::tau()),
        Curve3::circle_arc(
            p(-1, 0, 0),
            Vector3::y(),
            -Vector3::x(),
            Real::one(),
            Real::zero(),
            Real::tau(),
        )
        .unwrap(),
    ] {
        let parameter = (Real::pi() / Real::from(2)).unwrap();
        assert!(
            points_exactly_equal(&source.evaluate_real(&parameter).unwrap(), &p(-2, 0, 0)).unwrap()
        );
        let actual =
            retained_spatial_parameter(&carrier, &middle, &[contact(source, parameter)], &curve)
                .unwrap();
        assert_eq!(exact_order(&actual, &Real::pi()).unwrap(), Ordering::Equal);
    }
    let shift = Real::from(100) * Real::tau();
    let shifted = circle(
        Vector3::x(),
        Vector3::y(),
        shift.clone(),
        &shift + Real::tau(),
    );
    let actual = retained_spatial_parameter(
        &carrier,
        &middle,
        &[contact(shifted, shift + Real::pi())],
        &curve,
    )
    .unwrap();
    assert_eq!(exact_order(&actual, &Real::pi()).unwrap(), Ordering::Equal);
    // A seam contact has two exact lifts; the selected pcurve endpoint owns
    // which parameter identity is used.
    let seam = SurfaceIntersectionBoundaryContact::new(
        curve.clone(),
        Real::tau(),
        None,
        hypercurve::Point2::new(2.into(), 0.into()),
        p(2, 0, 0),
    );
    for (parameter, expected) in [(&start, Real::zero()), (&end, Real::tau())] {
        let actual =
            retained_spatial_parameter(&carrier, parameter, std::slice::from_ref(&seam), &curve)
                .unwrap();
        assert_eq!(exact_order(&actual, &expected).unwrap(), Ordering::Equal);
    }
}

#[test]
fn complete_circular_cone_cap_trim_replays_native_parameters() {
    let (source, solid) = crate::construction::cone_frustum(5.into(), 2.into(), 6.into()).unwrap();
    let (cutter, cutter_solid) =
        crate::construction::cuboid(p(0, -20, -20), p(20, 20, 20)).unwrap();
    let ArenaBooleanResult::Solid { model: half, .. } = sets::boolean(
        &source,
        &[solid],
        &cutter,
        &[cutter_solid],
        BooleanOperation::Intersection,
    )
    .unwrap() else {
        panic!("one half")
    };
    let ArenaBooleanResult::Solid { model: other, .. } = sets::boolean(
        &source,
        &[solid],
        &cutter,
        &[cutter_solid],
        BooleanOperation::Difference,
    )
    .unwrap() else {
        panic!("one complementary half")
    };
    let mut count = 0;
    for (first, _) in half.faces() {
        for (second, _) in other.faces() {
            if face_surface(&half, first).kind() != crate::SurfaceKind::Plane
                || face_surface(&other, second).kind() != crate::SurfaceKind::Cone
            {
                continue;
            }
            let SurfaceSurfaceIntersection::Curve(trace) = face_surface(&half, first)
                .intersect_surface(face_surface(&other, second))
                .unwrap()
            else {
                continue;
            };
            for (model, face, pcurve, name) in [
                (&half, first, trace.first_pcurve(), "plane"),
                (&other, second, trace.second_pcurve(), "cone"),
            ] {
                let region = planar_face_region(model, face).unwrap();
                for carrier in pcurve.clipping_carriers().unwrap().unwrap() {
                    let clipped = carrier.curve().trim_inside_region_with_parameters(&region);
                    assert!(clipped.is_ok(), "{name} {face:?}: {clipped:?}");
                    count += 1;
                }
            }
        }
    }
    assert!(count > 0);
}

#[test]
fn complete_circle_matching_distinguishes_phase_support_and_traversal() {
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let first = circle(Vector3::x(), Vector3::y(), Real::zero(), half_pi.clone());
    let rotated = circle(Vector3::y(), -Vector3::x(), -half_pi, Real::zero());
    assert!(exact_edge_curve_equal(&rotated, &first, false).unwrap());
    assert!(exact_edge_curve_equal(&rotated.reversed().unwrap(), &first, true).unwrap());
    assert!(!exact_edge_curve_equal(&rotated.reversed().unwrap(), &first, false).unwrap());
    let upper = circle(Vector3::x(), Vector3::y(), Real::zero(), Real::pi());
    let lower = circle(Vector3::x(), -Vector3::y(), Real::zero(), Real::pi());
    assert!(points_exactly_equal(&upper.start().unwrap(), &lower.start().unwrap()).unwrap());
    assert!(points_exactly_equal(&upper.end().unwrap(), &lower.end().unwrap()).unwrap());
    assert!(!exact_edge_curve_equal(&upper, &lower, false).unwrap());
    let wrong_support = Curve3::circle_arc(
        p(1, 0, 0),
        Vector3::x(),
        Vector3::y(),
        2.into(),
        Real::zero(),
        Real::pi(),
    )
    .unwrap();
    assert!(!exact_edge_curve_equal(&upper, &wrong_support, false).unwrap());
    let full = circle(Vector3::x(), Vector3::y(), Real::zero(), Real::tau());
    assert!(points_exactly_equal(&full.start().unwrap(), &full.end().unwrap()).unwrap());
    assert!(exact_edge_curve_equal(&full, &full, false).unwrap());
    assert!(!exact_edge_curve_equal(&full, &full, true).unwrap());
    assert!(exact_edge_curve_equal(&full.reversed().unwrap(), &full, true).unwrap());
}

#[test]
fn complete_planar_circle_trace_union_handles_reflected_and_redundant_charts() {
    let plane = Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap();
    let quarter = (Real::pi() / Real::from(2)).unwrap();
    let mut traces = Vec::new();
    for (x, y) in [(Vector3::x(), Vector3::y()), (Vector3::y(), Vector3::x())] {
        for index in 0..4 {
            traces.push(
                SurfaceIntersectionCurve::on_plane(
                    circle(
                        x.clone(),
                        y.clone(),
                        &quarter * Real::from(index),
                        &quarter * Real::from(index + 1),
                    ),
                    &plane,
                )
                .unwrap(),
            );
        }
    }
    traces.push(
        SurfaceIntersectionCurve::on_plane(
            circle(Vector3::x(), Vector3::y(), Real::zero(), Real::tau()),
            &plane,
        )
        .unwrap(),
    );
    let result = coalesce_circle_traces(&plane, traces).unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(
        compare_reals(
            &(result[0].curve().domain().represented_end().unwrap()
                - result[0].curve().domain().represented_start().unwrap()),
            &Real::tau(),
            crate::STRICT_PREDICATES,
        )
        .value(),
        Some(Ordering::Equal)
    );
}

#[test]
fn mixed_planar_trace_union_joins_complete_semicircles_without_filling_gaps() {
    let plane = Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap();
    let retain = |curve| SurfaceIntersectionCurve::on_plane(curve, &plane).unwrap();
    let quarter = (Real::pi() / Real::from(2)).unwrap();
    let first = circle(Vector3::x(), Vector3::y(), Real::zero(), quarter.clone());
    let second = circle(Vector3::x(), Vector3::y(), quarter.clone(), Real::pi());
    let line = Curve3::line(p(-20, 0, 0), p(20, 0, 0)).unwrap();
    let result = coalesce_circle_traces(
        &plane,
        vec![
            retain(first.clone()),
            retain(line.clone()),
            retain(second),
            retain(first.clone()),
        ],
    )
    .unwrap();
    assert_eq!(result.len(), 2);
    let arc = result
        .iter()
        .find(|trace| trace.curve().kind() == crate::Curve3Kind::CircleArc)
        .unwrap();
    assert!(
        exact_edge_curve_equal(
            arc.curve(),
            &circle(Vector3::x(), Vector3::y(), Real::zero(), Real::pi()),
            false,
        )
        .unwrap()
    );
    let last = circle(
        Vector3::x(),
        Vector3::y(),
        &quarter * Real::from(3),
        Real::tau(),
    );
    let wrapped = coalesce_circle_traces(
        &plane,
        vec![retain(first.clone()), retain(line.clone()), retain(last)],
    )
    .unwrap();
    assert_eq!(wrapped.len(), 2);
    let arc = wrapped
        .iter()
        .find(|trace| trace.curve().kind() == crate::Curve3Kind::CircleArc)
        .unwrap();
    assert!(
        exact_edge_curve_equal(
            arc.curve(),
            &circle(
                Vector3::x(),
                Vector3::y(),
                &quarter * Real::from(3),
                &quarter * Real::from(5)
            ),
            false,
        )
        .unwrap()
    );
    let delta = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    let gap = circle(Vector3::x(), Vector3::y(), &quarter + delta, Real::pi());
    let result =
        coalesce_circle_traces(&plane, vec![retain(first), retain(line), retain(gap)]).unwrap();
    assert_eq!(result.len(), 3);
}

#[test]
fn native_circle_trace_union_requires_complete_coverage_in_actual_charts() {
    let plane = Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap();
    let retain = |curve| SurfaceIntersectionCurve::on_plane(curve, &plane).unwrap();
    let upper = circle(Vector3::x(), Vector3::y(), Real::zero(), Real::pi());
    let lower = circle(Vector3::x(), -Vector3::y(), Real::zero(), Real::pi());
    let result =
        coalesce_circle_traces(&plane, vec![retain(upper.clone()), retain(lower)]).unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(
        compare_reals(
            &(result[0].curve().domain().represented_end().unwrap()
                - result[0].curve().domain().represented_start().unwrap()),
            &Real::tau(),
            crate::STRICT_PREDICATES
        )
        .value(),
        Some(Ordering::Equal)
    );
    let quarter = (Real::pi() / Real::from(2)).unwrap();
    let lower_quarter = circle(Vector3::x(), -Vector3::y(), Real::zero(), quarter);
    let result =
        coalesce_circle_traces(&plane, vec![retain(upper.clone()), retain(lower_quarter)]).unwrap();
    assert_eq!(result.len(), 2);
    // Adjacent numerical domains describe the SAME upper semicircle here.
    // Discarding the retained angle offset would invent the missing lower half.
    let Curve3ExactData::EllipseArc(mut duplicate) = upper.exact_data() else {
        unreachable!()
    };
    duplicate.domain_start = Real::pi();
    duplicate.domain_end = Real::tau();
    let duplicate = Curve3::from_exact_data(Curve3ExactData::EllipseArc(duplicate)).unwrap();
    let result =
        coalesce_circle_traces(&plane, vec![retain(upper.clone()), retain(duplicate)]).unwrap();
    // Complete-map deduplication now retains one copy of the same upper half.
    // In particular it must not invent the missing lower semicircle.
    assert_eq!(result.len(), 1);
    assert!(exact_edge_curve_equal(result[0].curve(), &upper, false).unwrap());
    assert_eq!(
        exact_order(
            &(result[0].curve().domain().represented_end().unwrap()
                - result[0].curve().domain().represented_start().unwrap()),
            &Real::pi()
        )
        .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn periodic_iso_lifts_retain_images_across_distant_signed_charts() {
    let (model, _) = crate::construction::torus(5.into(), 2.into()).unwrap();
    let face = FaceId::from_index(0).unwrap();
    let plane = Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap();
    let SurfaceSurfaceIntersection::Curves(traces) = face_surface(&model, face)
        .intersect_surface(&plane)
        .unwrap()
    else {
        panic!("two axial torus circles");
    };
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let trace = traces[0].subcurve(&Real::zero(), &half_pi).unwrap();
    let baseline =
        periodic_face_trace_lifts(&model, face, &trace, SurfaceIntersectionOperand::First).unwrap();
    assert!(!baseline.is_empty());
    for turns in [-1_000_000, -1, 0, 1, 1_000_000] {
        let shifted = trace
            .translated_iso_pcurve(
                SurfaceIntersectionOperand::First,
                &(Real::from(turns) * Real::tau()),
                &Real::zero(),
            )
            .unwrap();
        let lifts =
            periodic_face_trace_lifts(&model, face, &shifted, SurfaceIntersectionOperand::First)
                .unwrap();
        assert_eq!(lifts.len(), baseline.len());
        for (actual, expected) in lifts.iter().zip(&baseline) {
            assert!(actual.curve().shares_parameterization(expected.curve()));
            for parameter in [
                actual.curve().domain().represented_start().unwrap(),
                actual.curve().domain().represented_end().unwrap(),
            ] {
                let a = actual
                    .first_pcurve()
                    .point_at(&Real::clone(parameter).into())
                    .map(crate::geometry::represented_test_surface_parameter)
                    .unwrap();
                let b = expected
                    .first_pcurve()
                    .point_at(&Real::clone(parameter).into())
                    .map(crate::geometry::represented_test_surface_parameter)
                    .unwrap();
                assert_eq!(
                    compare_reals(&a.x, &b.x, crate::STRICT_PREDICATES).value(),
                    Some(Ordering::Equal)
                );
                assert_eq!(
                    compare_reals(&a.y, &b.y, crate::STRICT_PREDICATES).value(),
                    Some(Ordering::Equal)
                );
            }
        }
    }
}

#[test]
fn plane_bound_separation_keeps_touching_and_undecided_queries_unrejected() {
    let (model, _) = crate::construction::cuboid(p(0, 0, 0), p(8, 8, 8)).unwrap();
    let face = FaceId::from_index(0).unwrap();
    for (x, separated) in [(9, true), (8, false), (7, false)] {
        let plane = Surface::plane(p(x, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        assert_eq!(
            plane_separates_finite_face(&plane, &model, face).unwrap(),
            separated
        );
    }
    let enclosure = Real::pi().certified_rational_interval(-2048).unwrap();
    let midpoint = ((Real::new(enclosure[0].clone()) + Real::new(enclosure[1].clone()))
        / Real::from(2))
    .unwrap();
    let x = Real::from(8) + Real::pi() - midpoint;
    let plane = Surface::plane(
        Point3::new(x.clone(), 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap();
    let bounds = certified_face_bounds(&model, face).unwrap().unwrap();
    assert!(matches!(
        hyperlimit::classify_plane_aabb3(
            &hyperlimit::Plane3::new(p(1, 0, 0), -x),
            &bounds.mins,
            &bounds.maxs,
            crate::STRICT_PREDICATES
        ),
        PredicateOutcome::Unknown { .. }
    ));
    assert!(!plane_separates_finite_face(&plane, &model, face).unwrap());
}
