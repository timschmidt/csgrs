//! Point-spread helpers for readiness checks.
//!
//! The helpers in this module keep exact point-set diameter calculations out of
//! individual checks. They use Andrew's monotone-chain convex hull followed by a
//! rotating-calipers diameter pass, following Andrew, "Another Efficient
//! Algorithm for Convex Hulls in Two Dimensions" (1979), and Toussaint,
//! "Solving Geometric Problems with the Rotating Calipers" (1983).

/// Exact maximum Euclidean distance among a set of 2D points.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PointSpread {
    /// Maximum point-to-point distance.
    pub(super) distance: f64,
    /// Endpoints that realize the maximum distance, when at least two unique
    /// points are present.
    pub(super) endpoints: Option<[[f64; 2]; 2]>,
    /// Number of points on the monotone-chain hull used for the exact pass.
    pub(super) hull_points: usize,
    /// Number of antipodal caliper states inspected after hull reduction.
    pub(super) caliper_steps: usize,
}

/// Compute exact maximum point spread with hull reduction.
///
/// Thermal-via distribution and similar checks care about the diameter of a
/// point set, not every interior point pair. Convex-hull reduction is exact for
/// Euclidean diameter because the farthest pair lies on the convex hull; the
/// rotating-calipers pass then visits antipodal hull vertices instead of all
/// source pairs.
pub(super) fn maximum_point_spread(points: impl IntoIterator<Item = [f64; 2]>) -> PointSpread {
    let mut points = points
        .into_iter()
        .filter(|point| point[0].is_finite() && point[1].is_finite())
        .collect::<Vec<_>>();
    points.sort_by(|left, right| {
        left[0]
            .total_cmp(&right[0])
            .then(left[1].total_cmp(&right[1]))
    });
    points.dedup_by(|left, right| left[0] == right[0] && left[1] == right[1]);

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

fn convex_hull(points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    if points.len() <= 1 {
        return points;
    }

    let mut lower = Vec::new();
    for point in &points {
        while lower.len() >= 2
            && cross(lower[lower.len() - 2], lower[lower.len() - 1], *point) <= 0.0
        {
            lower.pop();
        }
        lower.push(*point);
    }

    let mut upper = Vec::new();
    for point in points.iter().rev() {
        while upper.len() >= 2
            && cross(upper[upper.len() - 2], upper[upper.len() - 1], *point) <= 0.0
        {
            upper.pop();
        }
        upper.push(*point);
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn hull_diameter(hull: &[[f64; 2]]) -> (f64, Option<[[f64; 2]; 2]>, usize) {
    match hull.len() {
        0 | 1 => (0.0, None, 0),
        2 => (distance(hull[0], hull[1]), Some([hull[0], hull[1]]), 1),
        count => {
            let mut maximum_squared: f64 = 0.0;
            let mut endpoints = None;
            let mut caliper_steps = 0_usize;
            let mut antipodal = 1_usize;

            for index in 0..count {
                let next = (index + 1) % count;
                while triangle_area2(hull[index], hull[next], hull[(antipodal + 1) % count])
                    > triangle_area2(hull[index], hull[next], hull[antipodal])
                {
                    antipodal = (antipodal + 1) % count;
                    caliper_steps += 1;
                }

                let index_distance = squared_distance(hull[index], hull[antipodal]);
                if index_distance > maximum_squared {
                    maximum_squared = index_distance;
                    endpoints = Some([hull[index], hull[antipodal]]);
                }
                let next_distance = squared_distance(hull[next], hull[antipodal]);
                if next_distance > maximum_squared {
                    maximum_squared = next_distance;
                    endpoints = Some([hull[next], hull[antipodal]]);
                }
                caliper_steps += 1;
            }

            (maximum_squared.sqrt(), endpoints, caliper_steps)
        }
    }
}

fn triangle_area2(left: [f64; 2], right: [f64; 2], point: [f64; 2]) -> f64 {
    cross(left, right, point).abs()
}

fn cross(origin: [f64; 2], left: [f64; 2], right: [f64; 2]) -> f64 {
    (left[0] - origin[0]) * (right[1] - origin[1]) - (left[1] - origin[1]) * (right[0] - origin[0])
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    squared_distance(left, right).sqrt()
}

fn squared_distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    dx * dx + dy * dy
}

#[cfg(test)]
mod tests {
    use super::maximum_point_spread;

    #[test]
    fn maximum_point_spread_handles_empty_single_and_duplicate_points() {
        assert_eq!(maximum_point_spread([]).distance, 0.0);
        assert_eq!(maximum_point_spread([[1.0, 2.0]]).distance, 0.0);
        assert_eq!(
            maximum_point_spread([[1.0, 2.0], [1.0, 2.0], [1.0, 2.0]]).hull_points,
            1
        );
    }

    #[test]
    fn maximum_point_spread_ignores_interior_points() {
        let spread =
            maximum_point_spread([[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0], [2.0, 1.5]]);

        assert!((spread.distance - 5.0).abs() < 1.0e-12);
        let endpoints = spread
            .endpoints
            .expect("rectangle spread should report a farthest endpoint pair");
        let endpoint_distance = ((endpoints[0][0] - endpoints[1][0]).powi(2)
            + (endpoints[0][1] - endpoints[1][1]).powi(2))
        .sqrt();
        assert!((endpoint_distance - spread.distance).abs() < 1.0e-12);
        assert_eq!(spread.hull_points, 4);
    }

    #[test]
    fn maximum_point_spread_handles_collinear_points() {
        let spread = maximum_point_spread([[0.0, 0.0], [1.0, 0.0], [3.0, 0.0], [2.0, 0.0]]);

        assert!((spread.distance - 3.0).abs() < 1.0e-12);
        assert_eq!(spread.hull_points, 2);
    }
}
