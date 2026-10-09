//! Native circles constructed from authored centers and radii.

use hypercurve::{CircularArc2, Contour2, CurveRegion2, Point2, Segment2};
use hyperlimit::Sign;

use crate::{PREDICATE_POLICY, PcbGeometryUncertainty, Scalar};

/// Construct a circle without passing its source geometry through report chords.
/// A zero radius denotes the empty regularized region. Negative, unresolved,
/// and denied inputs remain failures; callers decide how to report them.
pub(crate) fn circle_region(
    center: &[Scalar; 2],
    radius: &Scalar,
) -> Result<CurveRegion2, PcbGeometryUncertainty> {
    let failure = |detail: String| PcbGeometryUncertainty {
        operation: "circle-construction".to_owned(),
        source: None,
        detail,
    };
    if let Some(exhaustion) = hypersolve::work_budget::exhaustion() {
        return Err(failure(format!("work budget exhausted: {exhaustion:?}")));
    }
    let result = hypercurve::evaluate_under(PREDICATE_POLICY, || {
        match crate::scalar::sign(radius) {
            Some(Sign::Positive) => {}
            Some(Sign::Zero) => return Ok(CurveRegion2::empty()),
            Some(Sign::Negative) => return Err(failure("negative circle radius".to_owned())),
            None => return Err(failure("circle radius sign is unresolved".to_owned())),
        }
        let center_point = Point2::new(center[0].clone(), center[1].clone());
        let right = Point2::new(&center[0] + radius, center[1].clone());
        let left = Point2::new(&center[0] - radius, center[1].clone());
        let first =
            CircularArc2::try_from_center(right.clone(), left.clone(), center_point.clone(), false)
                .map_err(|error| failure(error.to_string()))?;
        let second = CircularArc2::try_from_center(left, right, center_point, false)
            .map_err(|error| failure(error.to_string()))?;
        let contour = Contour2::try_new(vec![Segment2::Arc(first), Segment2::Arc(second)])
            .map_err(|error| failure(error.to_string()))?;
        CurveRegion2::try_from_native_material_contours(vec![contour])
            .map_err(|error| failure(error.to_string()))
    });
    if let Some(exhaustion) = hypersolve::work_budget::exhaustion() {
        return Err(failure(format!("work budget exhausted: {exhaustion:?}")));
    }
    result
        .certified()
        .ok_or_else(|| failure("circle construction lacks a STRICT receipt".to_owned()))?
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Coord, LineString, Polygon, circle_polygon, polygons_to_profile};
    use crate::scalar::scalar;

    #[test]
    fn report_resolution_does_not_change_circle_geometry() {
        let center = [Scalar::from(2), Scalar::from(-3)];
        let mut native_count = None;
        for segments in [16, 17, 48, 128] {
            let polygon = circle_polygon([2.0, -3.0], 4.0, segments);
            assert_eq!(polygon.exterior().0.len(), segments + 1);
            assert!(polygon.unsigned_area() < 16.0 * std::f64::consts::PI);
            assert_eq!(
                crate::scalar::compare(
                    polygon.exact_area().unwrap(),
                    &(Scalar::from(16) * Scalar::pi())
                ),
                Some(std::cmp::Ordering::Equal)
            );
            assert_eq!(
                polygon.exact_bounds(),
                Some(&[(-2).into(), (-7).into(), 6.into(), 1.into()])
            );
            let native = polygon
                .try_exact_region()
                .unwrap()
                .native_contours_fast_path()
                .unwrap()
                .unwrap();
            assert_eq!(native.material_contours().len(), 1);
            let arcs = native.material_contours()[0].segments();
            assert!(!arcs.is_empty());
            if let Some(count) = native_count {
                assert_eq!(arcs.len(), count);
            } else {
                native_count = Some(arcs.len());
            }
            assert!(
                arcs.iter()
                    .all(|segment| matches!(segment, Segment2::Arc(_)))
            );
            let point = Point2::new(&center[0] + Scalar::from(5), center[1].clone());
            let distances = arcs
                .iter()
                .map(|segment| segment.point_distance_squared(&point).unwrap())
                .collect::<Vec<_>>();
            assert!(distances.iter().all(|distance| {
                crate::scalar::compare(distance, &Scalar::one()).is_some_and(|order| !order.is_lt())
            }));
            assert!(distances.iter().any(|distance| {
                crate::scalar::compare(distance, &Scalar::one()) == Some(std::cmp::Ordering::Equal)
            }));
        }
    }

    #[test]
    fn profile_bounds_come_from_native_circle_not_its_report_diamond() {
        let region = circle_region(&[Scalar::zero(), Scalar::zero()], &Scalar::one()).unwrap();
        let polygon = Polygon::from_exact_region(
            region,
            LineString(vec![
                Coord { x: 0.7, y: 0.7 },
                Coord { x: -0.7, y: 0.7 },
                Coord { x: -0.7, y: -0.7 },
                Coord { x: 0.7, y: -0.7 },
                Coord { x: 0.7, y: 0.7 },
            ]),
            vec![],
        );
        let profile = polygons_to_profile(vec![polygon], None);
        assert_eq!(
            profile.exact_bounds(),
            Some(&[(-1).into(), (-1).into(), 1.into(), 1.into()])
        );
        assert_eq!(
            profile.classify_xy(scalar("0.999"), scalar("0.03")),
            Ok(hypercurve::RegionPointLocation::Inside)
        );
    }

    #[test]
    fn native_circle_preserves_decimal_center_and_radius_beyond_binary64_resolution() {
        let center = [
            scalar("9007199254740992.1"),
            scalar("-0.00000000000000000000001"),
        ];
        let radius = scalar("0.10000000000000000000002");
        let region = circle_region(&center, &radius).unwrap();
        let bounds = region.bounds().unwrap().unwrap();
        assert_eq!(bounds.min_x(), &(&center[0] - &radius));
        assert_eq!(bounds.max_x(), &(&center[0] + &radius));
        assert_eq!(bounds.min_y(), &(&center[1] - &radius));
        assert_eq!(bounds.max_y(), &(&center[1] + &radius));
    }

    #[test]
    fn circle_construction_reports_unknown_or_denied_radius_and_allows_retry() {
        let center = [Scalar::zero(), Scalar::zero()];
        assert!(circle_region(&center, &Scalar::from(-1)).is_err());
        assert!(circle_region(&center, &Scalar::zero()).unwrap().is_empty());
        let angle = Scalar::e();
        let sin = angle.clone().sin();
        let cos = angle.cos();
        let unknown = &sin * &sin + &cos * &cos - Scalar::one();
        assert!(circle_region(&center, &unknown).is_err());
        hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
            .unwrap()
            .evaluate(|| {
                assert!(circle_region(&center, &unknown).is_err());
            });
        let result = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                    [-36, 1368, -11034, 31728, -38280, 16620]
                        .into_iter()
                        .map(Scalar::from)
                        .collect(),
                )
                .unwrap();
                assert!(polynomial.isolate_unit_interval_roots().is_err());
                assert!(circle_region(&center, &Scalar::one()).is_err());
            },
        );
        assert!(result.result.is_err());
        assert!(!circle_region(&center, &Scalar::one()).unwrap().is_empty());
    }
}
