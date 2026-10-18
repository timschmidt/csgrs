//! Complete affine correspondence for axial plane/cylinder intersection lines.

use super::*;

impl SurfaceIntersectionCurve {
    /// Retains a native line segment on a plane and a circular cylinder.
    ///
    /// The plane incidence, axial direction and constant radial distance prove
    /// the whole segment image. The cylinder's angle is constant and its axial
    /// coordinate is affine in the original spatial parameter. Periodic face
    /// lifts are applied later, against the actual authored trim chart.
    pub(crate) fn on_plane_cylinder_line(
        curve: Curve3,
        first: &Surface,
        second: &Surface,
    ) -> GeometryResult<Option<Self>> {
        if curve.kind() != Curve3Kind::Line {
            return Ok(None);
        }
        let (plane, cylinder, cylinder_first) = match (&first.data.geometry, &second.data.geometry)
        {
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::Cylinder(cylinder)) => {
                (plane, cylinder, false)
            }
            (SurfaceGeometry::Cylinder(cylinder), SurfaceGeometry::Plane(plane)) => {
                (plane, cylinder, true)
            }
            _ => return Ok(None),
        };
        let start = curve.start()?;
        let end = curve.end()?;
        let direction = &end - &start;
        let normal = plane.u.cross(&plane.v);
        let zero = |value: Real| -> GeometryResult<bool> {
            Ok(decided_order(compare_reals(
                &value,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == Ordering::Equal)
        };
        if !zero(normal.dot(&(&start - &plane.origin)))?
            || !zero(normal.dot(&direction))?
            || !zero(direction.cross(&cylinder.frame.z).norm_squared())?
        {
            return Ok(None);
        }
        let offset = &start - &cylinder.origin;
        let axial_start = offset.dot(&cylinder.frame.z);
        let radial = offset - cylinder.frame.z.clone() * &axial_start;
        if !zero(radial.norm_squared() - &cylinder.radius * &cylinder.radius)? {
            return Ok(None);
        }
        let angle = certified_atan2(radial.dot(&cylinder.frame.y), radial.dot(&cylinder.frame.x))?;
        let axial_delta = direction.dot(&cylinder.frame.z);
        let parameter_span =
            curve.domain().represented_end()? - curve.domain().represented_start()?;
        let scale = (axial_delta / parameter_span).map_err(GeometryError::Numeric)?;
        let offset = axial_start - &scale * curve.domain().represented_start()?;
        let cylinder_pcurve = SurfaceIntersectionPcurve::tensor_iso(
            curve.domain().clone(),
            angle,
            TensorAxis::U,
            scale,
            offset,
        );
        let plane_pcurve = SurfaceIntersectionPcurve::plane_projection(curve.clone(), plane);
        Ok(Some(if cylinder_first {
            Self::new(curve, cylinder_pcurve, plane_pcurve)
        } else {
            Self::new(curve, plane_pcurve, cylinder_pcurve)
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(x.into(), y.into(), z.into())
    }

    fn cylinder() -> Surface {
        Surface::cylinder(
            p(0, 0, 0),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            5.into(),
        )
        .unwrap()
    }

    fn plane(x: Real) -> Surface {
        Surface::plane(
            Point3::new(x, 2.into(), 7.into()),
            Vector3::z(),
            Vector3::y(),
        )
        .unwrap()
    }

    fn check(trace: &SurfaceIntersectionCurve, first: &Surface, second: &Surface) {
        let domain = trace.curve().domain();
        for numerator in 0..=4 {
            let parameter = domain.represented_start().unwrap()
                + (domain.represented_end().unwrap() - domain.represented_start().unwrap())
                    * (Real::from(numerator) / Real::from(4)).unwrap();
            let point = trace.curve().evaluate_real(&parameter).unwrap();
            for (surface, pcurve) in [
                (first, trace.first_pcurve()),
                (second, trace.second_pcurve()),
            ] {
                let uv = pcurve.point_at(&parameter).unwrap();
                assert_eq!(
                    point3_equal(
                        &point,
                        &surface.point_at(&uv).unwrap(),
                        crate::STRICT_PREDICATES
                    )
                    .value(),
                    Some(true),
                );
            }
        }
    }

    #[test]
    fn axial_line_correspondence_retains_direction_domain_and_operand_order() {
        let cylinder = cylinder();
        for (x, y) in [(0, 5), (3, 4), (3, -4), (5, 0)] {
            let plane = plane(x.into());
            let curve = Curve3::line(p(x, y, -7), p(x, y, 13)).unwrap();
            let lower = (Real::one() / Real::from(5)).unwrap();
            let upper = (Real::from(4) / Real::from(5)).unwrap();
            for curve in [
                curve.clone(),
                curve.reversed().unwrap(),
                curve.subcurve(&lower, &upper).unwrap(),
            ] {
                for (first, second) in [(&plane, &cylinder), (&cylinder, &plane)] {
                    let trace = SurfaceIntersectionCurve::on_plane_cylinder_line(
                        curve.clone(),
                        first,
                        second,
                    )
                    .unwrap()
                    .unwrap();
                    assert_eq!(
                        trace.curve().domain().represented_start().unwrap(),
                        curve.domain().represented_start().unwrap()
                    );
                    assert_eq!(
                        trace.curve().domain().represented_end().unwrap(),
                        curve.domain().represented_end().unwrap()
                    );
                    check(&trace, first, second);
                    check(&trace.reversed().unwrap(), first, second);
                    let middle = ((curve.domain().represented_start().unwrap()
                        + curve.domain().represented_end().unwrap())
                        / Real::from(2))
                    .unwrap();
                    check(
                        &trace
                            .subcurve(curve.domain().represented_start().unwrap(), &middle)
                            .unwrap(),
                        first,
                        second,
                    );
                }
            }
        }
    }

    #[test]
    fn axial_line_proof_rejects_endpoint_fits_and_exact_near_misses() {
        let cylinder = cylinder();
        let plane = plane(3.into());
        let delta: Real = "1e-12".parse().unwrap();
        let wrong_plane = Surface::plane(
            Point3::new(Real::from(3) + &delta, 0.into(), 0.into()),
            Vector3::y(),
            Vector3::z(),
        )
        .unwrap();
        let valid = Curve3::line(p(3, 4, 0), p(3, 4, 6)).unwrap();
        assert!(
            SurfaceIntersectionCurve::on_plane_cylinder_line(
                valid.clone(),
                &wrong_plane,
                &cylinder
            )
            .unwrap()
            .is_none()
        );
        // Both endpoints lie on the cylinder, but the chord interior does not.
        let chord = Curve3::line(p(3, -4, 0), p(3, 4, 6)).unwrap();
        let radial_miss = Curve3::line(
            Point3::new(3.into(), Real::from(4) + &delta, 0.into()),
            Point3::new(3.into(), Real::from(4) + delta, 6.into()),
        )
        .unwrap();
        let arc = Curve3::circle_arc(
            p(0, 0, 0),
            Vector3::x(),
            Vector3::y(),
            5.into(),
            Real::zero(),
            Real::pi(),
        )
        .unwrap();
        for curve in [chord, radial_miss, arc] {
            assert!(
                SurfaceIntersectionCurve::on_plane_cylinder_line(curve, &plane, &cylinder)
                    .unwrap()
                    .is_none()
            );
        }
        assert!(
            SurfaceIntersectionCurve::on_plane_cylinder_line(valid, &plane, &plane)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn axial_trim_preserves_both_endpoint_contacts_and_image_certificates() {
        use crate::boolean::{FacePairRelation, FacePairTrim};
        let source = crate::builder::cylinder(5.into(), 6.into()).unwrap();
        let tool = crate::builder::cuboid(p(0, -20, 2), p(20, 20, 4)).unwrap();
        let first = source.bodies().next().unwrap();
        let second = tool.bodies().next().unwrap();
        for (first, second) in [(first, second), (second, first)] {
            let graph = crate::boolean::intersection_graph(first, second).unwrap();
            let mut checked = 0;
            for pair in graph.intersections() {
                if !matches!(
                    pair.relation(),
                    FacePairRelation::Exact(SurfaceSurfaceIntersection::Lines(_))
                ) {
                    continue;
                }
                let FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim() else {
                    continue;
                };
                for trace in fragments {
                    checked += 1;
                    for end in [false, true] {
                        let point = if end {
                            trace.curve().end()
                        } else {
                            trace.curve().start()
                        }
                        .unwrap();
                        let mut count = 0;
                        for operand in [
                            SurfaceIntersectionOperand::First,
                            SurfaceIntersectionOperand::Second,
                        ] {
                            assert!(trace.has_image_certificate(operand));
                            let contacts = if end {
                                trace.end_boundary_contacts(operand)
                            } else {
                                trace.start_boundary_contacts(operand)
                            };
                            count += contacts.len();
                            for contact in contacts {
                                assert_eq!(
                                    point3_equal(
                                        &point,
                                        contact.spatial_point(),
                                        crate::STRICT_PREDICATES
                                    )
                                    .value(),
                                    Some(true)
                                );
                                assert_eq!(
                                    point3_equal(
                                        &point,
                                        &contact
                                            .curve()
                                            .evaluate_real(contact.parameter())
                                            .unwrap(),
                                        crate::STRICT_PREDICATES
                                    )
                                    .value(),
                                    Some(true)
                                );
                            }
                        }
                        assert!(
                            count > 0,
                            "each finite endpoint retains its boundary witness"
                        );
                    }
                }
            }
            assert!(checked >= 2);
        }
    }
}
