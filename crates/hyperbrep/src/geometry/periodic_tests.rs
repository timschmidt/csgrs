use super::*;

#[test]
fn conic_plane_contact_keeps_exact_endpoint_lifts() {
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let a = q(3, 4).atan().unwrap();
    let tiny = q(1, 1_000_000) * q(1, 1_000_000);
    for elliptical in [false, true] {
        for shift in [Real::zero(), Real::from(100) * Real::tau()] {
            let start = &a + &shift;
            let end = Real::pi() * q(1, 2) + &shift;
            let center = Point3::new(10.into(), 0.into(), 0.into());
            let curve = if elliptical {
                Curve3::ellipse_arc(
                    center,
                    Vector3::x(),
                    Vector3::z(),
                    10.into(),
                    5.into(),
                    start,
                    end,
                )
            } else {
                Curve3::circle_arc(center, Vector3::x(), Vector3::z(), 5.into(), start, end)
            }
            .unwrap();
            for reversed in [false, true] {
                let curve = if reversed {
                    curve.reversed().unwrap()
                } else {
                    curve.clone()
                };
                for height in [
                    Real::from(3),
                    Real::from(3) - &tiny,
                    Real::from(3) + &tiny,
                    Real::from(5),
                ] {
                    let plane = Surface::plane(
                        Point3::new(0.into(), 0.into(), height.clone()),
                        Vector3::x(),
                        Vector3::y(),
                    )
                    .unwrap();
                    let relation = plane.intersect_curve(&curve).unwrap_or_else(|e| panic!("ellipse={elliptical} shift={shift:?} reversed={reversed} height={height:?}: {e:?}"));
                    if height == Real::from(3) - &tiny {
                        assert!(matches!(relation, CurveSurfaceIntersection::None));
                        continue;
                    }
                    let CurveSurfaceIntersection::Points(points) = relation else {
                        panic!("one contact")
                    };
                    assert_eq!(points.len(), 1);
                    assert_eq!(
                        compare_reals(
                            &points[0].point.try_coordinates().unwrap().z,
                            &height,
                            crate::STRICT_PREDICATES
                        )
                        .value(),
                        Some(Ordering::Equal)
                    );
                    if height == Real::from(3) {
                        let endpoint = if reversed {
                            curve.domain().end()
                        } else {
                            curve.domain().start()
                        };
                        assert_eq!(
                            compare_reals(
                                points[0].parameter.try_real().unwrap(),
                                endpoint,
                                crate::STRICT_PREDICATES
                            )
                            .value(),
                            Some(Ordering::Equal)
                        );
                        assert_eq!(
                            compare_reals(
                                &points[0].point.try_coordinates().unwrap().x,
                                &Real::from(if elliptical { 18 } else { 14 }),
                                crate::STRICT_PREDICATES
                            )
                            .value(),
                            Some(Ordering::Equal)
                        );
                        assert_eq!(points[0].kind, CurveSurfaceContactKind::Simple);
                    } else if height == Real::from(5) {
                        assert_eq!(points[0].kind, CurveSurfaceContactKind::Repeated);
                    } else {
                        assert_eq!(
                            compare_reals(
                                points[0].parameter.try_real().unwrap(),
                                curve.domain().start(),
                                crate::STRICT_PREDICATES
                            )
                            .value(),
                            Some(Ordering::Greater)
                        );
                        assert_eq!(
                            compare_reals(
                                points[0].parameter.try_real().unwrap(),
                                curve.domain().end(),
                                crate::STRICT_PREDICATES
                            )
                            .value(),
                            Some(Ordering::Less)
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn a_full_circle_contact_retains_both_seam_parameters() {
    let angle = (Real::from(3) / Real::from(4)).unwrap().atan().unwrap();
    let curve = Curve3::circle_arc(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Real::from(5),
        angle.clone(),
        angle + Real::tau(),
    )
    .unwrap();
    let plane = Surface::plane(
        Point3::new(0.into(), 3.into(), 0.into()),
        Vector3::x(),
        Vector3::z(),
    )
    .unwrap();
    for curve in [curve.clone(), curve.reversed().unwrap()] {
        let CurveSurfaceIntersection::Points(points) = plane.intersect_curve(&curve).unwrap()
        else {
            panic!("circle contacts")
        };
        assert_eq!(points.len(), 3);
        assert_eq!(
            compare_reals(
                points[0].parameter.try_real().unwrap(),
                curve.domain().start(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                points[2].parameter.try_real().unwrap(),
                curve.domain().end(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(Ordering::Equal)
        );
        for (point, x) in points.iter().zip([4, -4, 4]) {
            assert_eq!(
                compare_reals(
                    &point.point.try_coordinates().unwrap().x,
                    &Real::from(x),
                    crate::STRICT_PREDICATES
                )
                .value(),
                Some(Ordering::Equal)
            );
            assert_eq!(
                compare_reals(
                    &point.point.try_coordinates().unwrap().y,
                    &Real::from(3),
                    crate::STRICT_PREDICATES
                )
                .value(),
                Some(Ordering::Equal)
            );
            assert_eq!(point.kind, CurveSurfaceContactKind::Simple);
        }
    }
}
