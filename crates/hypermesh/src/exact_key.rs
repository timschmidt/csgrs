use hyperlimit::Point3;

/// Exact run-local key for points whose coordinates are all retained rationals.
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

pub(crate) fn exact_point3_key(point: &Point3) -> Option<ExactPoint3Key> {
    Some(ExactPoint3Key {
        x: point.x.exact_rational_ref()?.to_string(),
        y: point.y.exact_rational_ref()?.to_string(),
        z: point.z.exact_rational_ref()?.to_string(),
    })
}
