//! Point-spread helpers for readiness checks.
//!
//! The helpers in this module keep exact point-set diameter calculations out of
//! individual checks. They use a monotone-chain convex hull followed by a
//! rotating-calipers diameter pass.

use crate::Scalar;

/// Exact maximum Euclidean distance among a set of 2D points.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PointSpread {
    /// Maximum point-to-point distance.
    pub(super) distance: Scalar,
    /// Compatibility report endpoints that realize the maximum distance, when
    /// at least two unique points are present.
    pub(super) endpoints: Option<[[f64; 2]; 2]>,
    /// Number of points on the monotone-chain hull used for the exact pass.
    pub(super) hull_points: usize,
    /// Number of antipodal caliper states inspected after hull reduction.
    pub(super) caliper_steps: usize,
}

#[derive(Clone, Debug)]
struct ExactInputPoint {
    exact: [Scalar; 2],
    report: [f64; 2],
}

/// Compute exact maximum point spread with hull reduction.
///
/// Thermal-via distribution and similar checks care about the diameter of a
/// point set, not every interior point pair. Convex-hull reduction is exact for
/// Euclidean diameter because the farthest pair lies on the convex hull; the
/// rotating-calipers pass then visits antipodal hull vertices instead of all
/// source pairs. Each input carries its exact model coordinate and a primitive
/// projection used only for compatibility report geometry.
pub(super) fn maximum_point_spread(
    points: impl IntoIterator<Item = ([Scalar; 2], [f64; 2])>,
) -> PointSpread {
    let mut points = points
        .into_iter()
        .map(|(exact, report)| ExactInputPoint { exact, report })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left.exact[0]
            .partial_cmp(&right.exact[0])
            .expect("exact point x coordinates must be comparable")
            .then_with(|| {
                left.exact[1]
                    .partial_cmp(&right.exact[1])
                    .expect("exact point y coordinates must be comparable")
            })
    });
    points.dedup_by(|left, right| left.exact == right.exact);

    let hull = convex_hull(points);
    let hull_points = hull.len();
    let (distance, endpoints, caliper_steps) = hull_diameter(&hull);

    PointSpread {
        distance,
        endpoints,
        hull_points,
        caliper_steps,
    }
}

fn convex_hull(points: Vec<ExactInputPoint>) -> Vec<ExactInputPoint> {
    if points.len() <= 1 {
        return points;
    }

    let mut lower = Vec::new();
    for point in &points {
        while lower.len() >= 2
            && orientation_is_nonpositive(&lower[lower.len() - 2], &lower[lower.len() - 1], point)
        {
            lower.pop();
        }
        lower.push(point.clone());
    }

    let mut upper = Vec::new();
    for point in points.iter().rev() {
        while upper.len() >= 2
            && orientation_is_nonpositive(&upper[upper.len() - 2], &upper[upper.len() - 1], point)
        {
            upper.pop();
        }
        upper.push(point.clone());
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn hull_diameter(hull: &[ExactInputPoint]) -> (Scalar, Option<[[f64; 2]; 2]>, usize) {
    match hull.len() {
        0 | 1 => (Scalar::zero(), None, 0),
        2 => (
            distance(&hull[0], &hull[1]),
            Some([hull[0].report, hull[1].report]),
            1,
        ),
        count => {
            let mut maximum_squared = Scalar::zero();
            let mut endpoints = None;
            let mut caliper_steps = 0_usize;
            let mut antipodal = 1_usize;

            for index in 0..count {
                let next = (index + 1) % count;
                while triangle_area2(&hull[index], &hull[next], &hull[(antipodal + 1) % count])
                    > triangle_area2(&hull[index], &hull[next], &hull[antipodal])
                {
                    antipodal = (antipodal + 1) % count;
                    caliper_steps += 1;
                }

                let index_distance = squared_distance(&hull[index], &hull[antipodal]);
                if index_distance > maximum_squared {
                    maximum_squared = index_distance;
                    endpoints = Some([hull[index].report, hull[antipodal].report]);
                }
                let next_distance = squared_distance(&hull[next], &hull[antipodal]);
                if next_distance > maximum_squared {
                    maximum_squared = next_distance;
                    endpoints = Some([hull[next].report, hull[antipodal].report]);
                }
                caliper_steps += 1;
            }

            (
                maximum_squared.sqrt().unwrap_or_else(|_| Scalar::zero()),
                endpoints,
                caliper_steps,
            )
        }
    }
}

fn triangle_area2(
    left: &ExactInputPoint,
    right: &ExactInputPoint,
    point: &ExactInputPoint,
) -> Scalar {
    cross(left, right, point).abs()
}

fn cross(origin: &ExactInputPoint, left: &ExactInputPoint, right: &ExactInputPoint) -> Scalar {
    (&left.exact[0] - &origin.exact[0]) * (&right.exact[1] - &origin.exact[1])
        - (&left.exact[1] - &origin.exact[1]) * (&right.exact[0] - &origin.exact[0])
}

fn orientation_is_nonpositive(
    origin: &ExactInputPoint,
    left: &ExactInputPoint,
    right: &ExactInputPoint,
) -> bool {
    cross(origin, left, right) <= Scalar::zero()
}

fn distance(left: &ExactInputPoint, right: &ExactInputPoint) -> Scalar {
    squared_distance(left, right)
        .sqrt()
        .unwrap_or_else(|_| Scalar::zero())
}

fn squared_distance(left: &ExactInputPoint, right: &ExactInputPoint) -> Scalar {
    let dx = &left.exact[0] - &right.exact[0];
    let dy = &left.exact[1] - &right.exact[1];
    &dx * &dx + &dy * &dy
}

#[cfg(test)]
mod tests {
    use super::maximum_point_spread;
    use crate::{Scalar, scalar::scalar};

    fn point(report: [f64; 2]) -> ([Scalar; 2], [f64; 2]) {
        (
            [
                Scalar::try_from(report[0]).expect("test x must lift"),
                Scalar::try_from(report[1]).expect("test y must lift"),
            ],
            report,
        )
    }

    fn points<const N: usize>(reports: [[f64; 2]; N]) -> [([Scalar; 2], [f64; 2]); N] {
        reports.map(point)
    }

    #[test]
    fn maximum_point_spread_handles_empty_single_and_duplicate_points() {
        assert_eq!(maximum_point_spread([]).distance, scalar("0"));
        assert_eq!(
            maximum_point_spread(points([[1.0, 2.0]])).distance,
            scalar("0")
        );
        assert_eq!(
            maximum_point_spread(points([[1.0, 2.0], [1.0, 2.0], [1.0, 2.0]])).hull_points,
            1
        );
    }

    #[test]
    fn maximum_point_spread_ignores_interior_points() {
        let spread = maximum_point_spread(points([
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 3.0],
            [0.0, 3.0],
            [2.0, 1.5],
        ]));

        assert_eq!(spread.distance, scalar("5"));
        let endpoints = spread
            .endpoints
            .expect("rectangle spread should report a farthest endpoint pair");
        let endpoint_spread = maximum_point_spread(points(endpoints));
        assert_eq!(endpoint_spread.distance, spread.distance);
        assert_eq!(spread.hull_points, 4);
    }

    #[test]
    fn maximum_point_spread_handles_collinear_points() {
        let spread = maximum_point_spread(points([[0.0, 0.0], [1.0, 0.0], [3.0, 0.0], [2.0, 0.0]]));

        assert_eq!(spread.distance, scalar("3"));
        assert_eq!(spread.hull_points, 2);
    }
}
