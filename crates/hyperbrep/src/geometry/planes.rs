//! Shared exact plane relation and intersection construction.

use super::*;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PlaneRelation {
    Coincident,
    Parallel,
    Transverse,
}

/// Temporary algebra shared by relation queries and complete intersections.
struct PlanePair {
    first_normal: Vector3,
    second_normal: Vector3,
    direction: Vector3,
    direction_squared: Real,
}

impl PlanePair {
    fn new(first: &PlaneSurface, second: &PlaneSurface) -> Self {
        let first_normal = first.u.cross(&first.v);
        let second_normal = second.u.cross(&second.v);
        let direction = first_normal.cross(&second_normal);
        let direction_squared = direction.norm_squared();
        Self {
            first_normal,
            second_normal,
            direction,
            direction_squared,
        }
    }

    fn relation(
        &self,
        first: &PlaneSurface,
        second: &PlaneSurface,
    ) -> GeometryResult<PlaneRelation> {
        if decided_order(compare_reals(
            &self.direction_squared,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != Ordering::Equal
        {
            return Ok(PlaneRelation::Transverse);
        }
        let separation = self.first_normal.dot(&(&second.origin - &first.origin));
        if decided_order(compare_reals(
            &separation,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == Ordering::Equal
        {
            Ok(PlaneRelation::Coincident)
        } else {
            Ok(PlaneRelation::Parallel)
        }
    }
}

impl Surface {
    /// Constructs an exact pcurve after proving the entire spatial curve lies
    /// on this plane. `None` means the curve is not wholly contained; unsupported
    /// projection families and undecided predicates remain typed failures.
    /// The returned map retains the spatial curve's parameter domain.
    pub fn planar_pcurve(
        &self,
        curve: &Curve3,
    ) -> GeometryResult<Option<MaterializedSurfacePcurve>> {
        crate::context::strict(|| {
            let SurfaceGeometry::Plane(plane) = &self.data.geometry else {
                return Err(GeometryError::UnsupportedIntersection);
            };
            if !matches!(
                self.intersect_curve(curve)?,
                CurveSurfaceIntersection::Contained
            ) {
                return Ok(None);
            }
            SurfaceIntersectionPcurve::plane_projection(curve.clone(), plane)
                .materialize()
                .map(Some)
        })
    }

    /// Certifies native plane coincidence without constructing an unused line.
    /// Unsupported carriers and unresolved predicates remain typed failures.
    pub(crate) fn planes_coincident(&self, other: &Self) -> GeometryResult<bool> {
        crate::context::strict(|| match (&self.data.geometry, &other.data.geometry) {
            (SurfaceGeometry::Plane(first), SurfaceGeometry::Plane(second)) => {
                Ok(PlanePair::new(first, second).relation(first, second)?
                    == PlaneRelation::Coincident)
            }
            _ => Err(GeometryError::UnsupportedIntersection),
        })
    }
}

pub(super) fn intersect_planes(
    first: &PlaneSurface,
    second: &PlaneSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let pair = PlanePair::new(first, second);
    match pair.relation(first, second)? {
        PlaneRelation::Coincident => return Ok(SurfaceSurfaceIntersection::Coincident),
        PlaneRelation::Parallel => return Ok(SurfaceSurfaceIntersection::None),
        PlaneRelation::Transverse => {}
    }
    let PlanePair {
        first_normal,
        second_normal,
        direction,
        direction_squared,
    } = pair;
    let first_constant = first_normal.dot(&Vector3::from(first.origin.clone()));
    let second_constant = second_normal.dot(&Vector3::from(second.origin.clone()));
    let numerator = (second_normal.clone() * first_constant
        - first_normal.clone() * second_constant)
        .cross(&direction);
    let point = Point3::from(Vector3::from_xyz(
        (numerator.0[0].clone() / &direction_squared).map_err(GeometryError::Numeric)?,
        (numerator.0[1].clone() / &direction_squared).map_err(GeometryError::Numeric)?,
        (numerator.0[2].clone() / direction_squared).map_err(GeometryError::Numeric)?,
    ));
    Ok(SurfaceSurfaceIntersection::Line(Box::new(
        SurfaceIntersectionLine { point, direction },
    )))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperreal::Rational;

    #[test]
    fn plane_relations_preserve_exact_charts_offsets_and_intersection_lines() {
        let gap = Real::new(Rational::fraction(1, 1_000_000_000_000_u64).unwrap());
        for height in [Real::from(3), Real::pi()] {
            let base = Surface::plane(
                Point3::new(Real::zero(), Real::zero(), height.clone()),
                Vector3::x(),
                Vector3::y(),
            )
            .unwrap();
            let coincident = Surface::plane(
                Point3::new(Real::pi(), Real::from(-7), height.clone()),
                Vector3::x() * Real::from(2),
                Vector3::x() - Vector3::y() * Real::from(3),
            )
            .unwrap();
            let separate = Surface::plane(
                Point3::new(Real::zero(), Real::zero(), &height + &gap),
                Vector3::x(),
                -Vector3::y(),
            )
            .unwrap();
            let transverse = Surface::plane(
                Point3::new(Real::from(2), Real::zero(), Real::zero()),
                Vector3::y() * Real::from(2),
                Vector3::y() + Vector3::z() * Real::from(3),
            )
            .unwrap();
            for (other, expected) in [
                (&base, PlaneRelation::Coincident),
                (&coincident, PlaneRelation::Coincident),
                (&separate, PlaneRelation::Parallel),
                (&transverse, PlaneRelation::Transverse),
            ] {
                for (first, second, direction_sign) in [(&base, other, 1), (other, &base, -1)] {
                    for _ in 0..2 {
                        assert_eq!(
                            first.planes_coincident(second).unwrap(),
                            expected == PlaneRelation::Coincident
                        );
                        match (expected, first.intersect_surface(second).unwrap()) {
                            (PlaneRelation::Coincident, SurfaceSurfaceIntersection::Coincident)
                            | (PlaneRelation::Parallel, SurfaceSurfaceIntersection::None) => {}
                            (PlaneRelation::Transverse, SurfaceSurfaceIntersection::Line(line)) => {
                                // Independently specified equations: z = height, x = 2.
                                let expected_point =
                                    Point3::new(Real::from(2), Real::zero(), height.clone());
                                let expected_direction =
                                    Vector3::y() * Real::from(6 * direction_sign);
                                assert_eq!(
                                    point3_equal(
                                        &line.point,
                                        &expected_point,
                                        crate::STRICT_PREDICATES
                                    )
                                    .value(),
                                    Some(true)
                                );
                                assert_eq!(
                                    point3_equal(
                                        &Point3::from(line.direction),
                                        &Point3::from(expected_direction),
                                        crate::STRICT_PREDICATES,
                                    )
                                    .value(),
                                    Some(true)
                                );
                            }
                            (expected, actual) => panic!("expected {expected:?}, got {actual:?}"),
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn plane_relation_rejects_nonplane_carriers() {
        let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let cylinder = Surface::cylinder(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Real::one(),
        )
        .unwrap();
        for (first, second) in [
            (&plane, &cylinder),
            (&cylinder, &plane),
            (&cylinder, &cylinder),
        ] {
            assert_eq!(
                first.planes_coincident(second).unwrap_err(),
                GeometryError::UnsupportedIntersection
            );
        }
    }
}
