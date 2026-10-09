//! Native circular probe alternatives, without an unnecessary maximum order.

use crate::ipc356::Ipc356Point;
use crate::{PcbGeometryUncertainty, Scalar};

pub(super) fn radius(diameter: &Scalar) -> Result<Scalar, PcbGeometryUncertainty> {
    if crate::scalar::try_compare(diameter, &Scalar::zero())?.is_lt() {
        return Err(crate::operations::failure(
            "testpoint-probe-radius",
            None,
            "negative testpoint diameter",
        ));
    }
    Ok(crate::scalar::half(diameter))
}

/// The effective diameter is max(declared, minimum). Membership in that probe
/// is the union of membership at each radius, so no maximum-order certificate
/// is necessary. Required source diameters are validated before use.
pub(super) fn radii(
    point: &Ipc356Point,
    minimum: &Scalar,
) -> Result<Vec<Scalar>, PcbGeometryUncertainty> {
    let mut radii = vec![radius(minimum)?];
    if let Some(diameter) = &point.diameter {
        if diameter != minimum {
            radii.push(radius(diameter)?);
        }
    }
    Ok(radii)
}

/// Optional scheduling arithmetic. An unknown enclosure retains candidates.
/// Required radius validity is checked by radii() after metadata eligibility.
pub(super) fn candidate_radii(
    point: &Ipc356Point,
    minimum: &Scalar,
    addition: &Scalar,
) -> Vec<Scalar> {
    std::iter::once(minimum)
        .chain(point.diameter.iter())
        .map(|diameter| crate::scalar::half(diameter) + addition)
        .collect()
}
