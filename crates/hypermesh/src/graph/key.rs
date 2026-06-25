use hyperlimit::Point3;

/// Exact graph-local key for points whose coordinates are all retained rationals.
///
/// This is not a serialized certificate. It is an in-memory bucket key that
/// narrows exact equality checks before falling back to predicates for unkeyed
/// or potentially symbolic coordinates.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub(crate) struct ExactPoint3Key {
    x: String,
    y: String,
    z: String,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub(crate) struct ExactUndirectedPoint3EdgeKey {
    endpoints: [ExactPoint3Key; 2],
}

pub(crate) fn exact_point3_key(point: &Point3) -> Option<ExactPoint3Key> {
    Some(ExactPoint3Key {
        x: point.x.exact_rational_ref()?.to_string(),
        y: point.y.exact_rational_ref()?.to_string(),
        z: point.z.exact_rational_ref()?.to_string(),
    })
}

pub(crate) fn exact_undirected_point3_edge_key(
    points: &[Point3; 2],
) -> Option<ExactUndirectedPoint3EdgeKey> {
    let left = exact_point3_key(&points[0])?;
    let right = exact_point3_key(&points[1])?;
    let endpoints = if left <= right {
        [left, right]
    } else {
        [right, left]
    };
    Some(ExactUndirectedPoint3EdgeKey { endpoints })
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlimit::Point3;
    use hyperreal::Real;

    fn q(numerator: i64, denominator: i64) -> Real {
        (Real::from(numerator) / &Real::from(denominator)).expect("nonzero denominator")
    }

    fn p(x: [i64; 2], y: [i64; 2], z: [i64; 2]) -> Point3 {
        Point3::new(q(x[0], x[1]), q(y[0], y[1]), q(z[0], z[1]))
    }

    #[test]
    fn undirected_edge_key_canonicalizes_exact_rational_endpoints() {
        let left = p([1, 2], [2, 3], [3, 4]);
        let right = p([-5, 6], [7, 8], [-9, 10]);

        assert_eq!(
            exact_undirected_point3_edge_key(&[left.clone(), right.clone()]),
            exact_undirected_point3_edge_key(&[right, left])
        );
    }
}
