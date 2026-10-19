//! Certified chart transitions on coincident periodic native carriers.
use super::*;

impl SurfaceIntersectionCurve {
    /// The source pcurve is an already validated complete edge image. Cylinder
    /// frames prove a global angular reversal/phase and axial reversal/offset;
    /// target trim lifting subsequently chooses the authored whole-period lift.
    pub(crate) fn from_coincident_iso_boundary(
        curve: Curve3,
        pcurve: &Pcurve,
        edge_scale: &Real,
        edge_offset: &Real,
        source: &Surface,
        target: &Surface,
        target_operand: SurfaceIntersectionOperand,
    ) -> GeometryResult<Option<Self>> {
        let Some(mut trace) =
            Self::from_certified_iso_boundary(curve, pcurve, edge_scale, edge_offset)?
        else {
            return Ok(None);
        };
        if source.same_periodic_analytic_parameterization(target)? {
            return Ok(Some(trace));
        }
        let Some((sign, phase, height)) = source.coincident_cylinder_chart(target)? else {
            return Ok(None);
        };
        let pcurve = match target_operand {
            SurfaceIntersectionOperand::First => &mut trace.first_pcurve,
            SurfaceIntersectionOperand::Second => &mut trace.second_pcurve,
        };
        let offset = match &mut pcurve.mapping {
            SurfaceIntersectionPcurveMapping::TensorIsoU { constant } => {
                *constant = &sign * &*constant + phase;
                &height
            }
            SurfaceIntersectionPcurveMapping::TensorIsoV { constant } => {
                *constant = &sign * &*constant + height;
                &phase
            }
            _ => return Ok(None),
        };
        pcurve.source_scale = &sign * &pcurve.source_scale;
        pcurve.source_offset = &sign * &pcurve.source_offset + offset;
        Ok(Some(trace))
    }
}

impl Surface {
    /// A global chart equivalence, established from complete cylinder frames.
    fn coincident_cylinder_chart(
        &self,
        target: &Self,
    ) -> GeometryResult<Option<(Real, Real, Real)>> {
        let (SurfaceGeometry::Cylinder(source), SurfaceGeometry::Cylinder(target)) =
            (&self.data.geometry, &target.data.geometry)
        else {
            return Ok(None);
        };
        let equal = |a: &Real, b: &Real| -> GeometryResult<bool> {
            Ok(decided_order(compare_reals(a, b, crate::STRICT_PREDICATES))? == Ordering::Equal)
        };
        let displacement = &source.origin - &target.origin;
        if !equal(&source.radius, &target.radius)?
            || !equal(
                &source.frame.z.cross(&target.frame.z).norm_squared(),
                &Real::zero(),
            )?
            || !equal(
                &displacement.cross(&target.frame.z).norm_squared(),
                &Real::zero(),
            )?
        {
            return Ok(None);
        }
        let sign = source.frame.z.dot(&target.frame.z);
        let phase = certified_atan2(
            source.frame.x.dot(&target.frame.y),
            source.frame.x.dot(&target.frame.x),
        )?;
        let height = displacement.dot(&target.frame.z);
        Ok(Some((sign, phase, height)))
    }

    pub(crate) fn coincident_cylinder_parameter(
        &self,
        target: &Self,
        parameter: &Point2,
    ) -> GeometryResult<Option<Point2>> {
        Ok(self
            .coincident_cylinder_chart(target)?
            .map(|(sign, phase, height)| {
                Point2::new(&sign * &parameter.x + phase, sign * &parameter.y + height)
            }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cylinder_chart_preserves_complete_images_and_authored_periods() {
        let source = Surface::cylinder(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            5.into(),
        )
        .unwrap();
        for reversed in [false, true] {
            let target = Surface::cylinder(
                Point3::new(0.into(), 0.into(), 13.into()),
                Vector3::y(),
                if reversed {
                    Vector3::x()
                } else {
                    -Vector3::x()
                },
                if reversed {
                    -Vector3::z()
                } else {
                    Vector3::z()
                },
                5.into(),
            )
            .unwrap();
            for turns in [-100, 0, 100] {
                let angle = Real::from(turns) * Real::tau();
                for axial in [false, true] {
                    let start = Point2::new(angle.clone(), (-2).into());
                    let end = if axial {
                        Point2::new(angle.clone(), 8.into())
                    } else {
                        Point2::new(&angle + Real::pi(), (-2).into())
                    };
                    let curve = if axial {
                        Curve3::line(
                            source.evaluate_real(&start).unwrap(),
                            source.evaluate_real(&end).unwrap(),
                        )
                        .unwrap()
                    } else {
                        Curve3::circle_arc(
                            Point3::new(0.into(), 0.into(), (-2).into()),
                            Vector3::x(),
                            Vector3::y(),
                            5.into(),
                            start.x.clone(),
                            end.x.clone(),
                        )
                        .unwrap()
                    };
                    let pcurve = Pcurve::new(Curve2::from(
                        hypercurve::LineSeg2::try_new(
                            hypercurve::Point2::new(start.x, start.y),
                            hypercurve::Point2::new(end.x, end.y),
                        )
                        .unwrap(),
                    ));
                    for operand in [
                        SurfaceIntersectionOperand::First,
                        SurfaceIntersectionOperand::Second,
                    ] {
                        let trace = SurfaceIntersectionCurve::from_coincident_iso_boundary(
                            curve.clone(),
                            &pcurve,
                            &(curve.domain().represented_end().unwrap()
                                - curve.domain().represented_start().unwrap()),
                            curve.domain().represented_start().unwrap(),
                            &source,
                            &target,
                            operand,
                        )
                        .unwrap()
                        .unwrap();
                        for n in 0..=8 {
                            let t = curve.domain().represented_start().unwrap()
                                + (curve.domain().represented_end().unwrap()
                                    - curve.domain().represented_start().unwrap())
                                    * (Real::from(n) / Real::from(8)).unwrap();
                            let actual = target
                                .evaluate_real(
                                    &trace
                                        .pcurve(operand)
                                        .point_at(&Real::clone(&t).into())
                                        .map(crate::geometry::represented_test_surface_parameter)
                                        .unwrap(),
                                )
                                .unwrap();
                            assert_eq!(
                                point3_equal(
                                    &actual,
                                    &curve.evaluate_real(&t).unwrap(),
                                    crate::STRICT_PREDICATES
                                )
                                .value(),
                                Some(true)
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn cylinder_chart_rejects_near_distinct_carriers() {
        let delta: Real = "1e-12".parse().unwrap();
        let make = |x, radius| {
            Surface::cylinder(
                Point3::new(x, 0.into(), 0.into()),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                radius,
            )
            .unwrap()
        };
        let source = make(Real::zero(), 5.into());
        for target in [
            make(delta.clone(), 5.into()),
            make(Real::zero(), Real::from(5) + delta),
        ] {
            assert!(
                source
                    .coincident_cylinder_parameter(&target, &Point2::new(0.into(), 2.into()))
                    .unwrap()
                    .is_none()
            );
        }
    }
}
