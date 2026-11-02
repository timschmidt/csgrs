//! Complete sections of a native ring torus by non-coaxial analytic carriers.
//!
//! A ring torus is the tensor product of two rational circles: its major
//! circle and the meridian circle of its tube. Using the same four exact arcs
//! as the cylinder atlas for both circles gives sixteen rational biquadratic
//! patches whose controls and weights are rational in the torus frame. The torus is bounded, so no height bound is needed; each
//! patch is sectioned completely by the implicit/tensor route.

use super::curve_implicit::ImplicitCarrier;
use super::cylinder_atlas::{COARSE_TANGENTS, section_patches, unit_arcs};
use super::*;

/// Intersects `torus` with `other` through the exact torus atlas.
pub(super) fn intersect(
    torus: &TorusSurface,
    other: ImplicitCarrier<'_>,
    torus_operand: SurfaceIntersectionOperand,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    section_patches(atlas(torus)?, other, torus_operand)
}

/// `center + (R + r m.x)(a.x X + a.y Y) + r m.y Z` with weights
/// `w_major * w_minor`: the revolution of the rational meridian circle `m`
/// by the rational major circle `a`.
fn atlas(torus: &TorusSurface) -> GeometryResult<Vec<Surface>> {
    let arcs = unit_arcs(COARSE_TANGENTS)?;
    let frame = &torus.frame;
    let mut patches = Vec::with_capacity(arcs.len() * arcs.len());
    for major in &arcs {
        let major_controls = [&major.start, &major.middle, &major.end];
        let major_weights = [Real::one(), major.weight.clone(), Real::one()];
        for minor in &arcs {
            let minor_controls = [&minor.start, &minor.middle, &minor.end];
            let minor_weights = [Real::one(), minor.weight.clone(), Real::one()];
            let mut controls = Vec::with_capacity(3);
            let mut weights = Vec::with_capacity(3);
            for (a, wa) in major_controls.iter().zip(&major_weights) {
                let mut row = Vec::with_capacity(3);
                let mut row_weights = Vec::with_capacity(3);
                for (m, wm) in minor_controls.iter().zip(&minor_weights) {
                    let radial = &torus.major_radius + &torus.minor_radius * &m.0;
                    let axial = &torus.minor_radius * &m.1;
                    row.push(
                        torus.center.clone()
                            + frame.x.clone() * (&radial * &a.0)
                            + frame.y.clone() * (&radial * &a.1)
                            + frame.z.clone() * axial,
                    );
                    row_weights.push(wa * wm);
                }
                controls.push(row);
                weights.push(row_weights);
            }
            patches.push(Surface::rational_bezier(controls, weights)?);
        }
    }
    Ok(patches)
}

/// Whether an existing coaxial special route owns the pair: a sphere centred
/// on the torus axis, or a cone or torus with parallel axis on the same line.
pub(super) fn special_route_applies(
    torus: &TorusSurface,
    other: ImplicitCarrier<'_>,
) -> GeometryResult<bool> {
    let zero = |value: &Real| -> GeometryResult<bool> {
        Ok(decided_order(compare_reals(
            value,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == Ordering::Equal)
    };
    let axis = &torus.frame.z;
    let on_axis = |point: &Point3| -> GeometryResult<bool> {
        let offset = point - &torus.center;
        let radial = &offset - &(axis.clone() * axis.dot(&offset));
        zero(&radial.norm_squared())
    };
    let parallel = |direction: &Vector3| zero(&axis.cross(direction).norm_squared());
    Ok(match other {
        ImplicitCarrier::Sphere(sphere) => on_axis(&sphere.center)?,
        // Non-coaxial torus pairs pull back to bidegree (8, 8) per patch; a
        // measured overlapping pair took minutes, so they keep the existing
        // route and its typed refusal until that cost is addressed.
        ImplicitCarrier::Torus(_) => true,
        ImplicitCarrier::Cone(cone) => parallel(&cone.frame.z)? && on_axis(&cone.apex)?,
        // Cylinders are charted by the cylinder atlas.
        ImplicitCarrier::Cylinder(_) => true,
    })
}
