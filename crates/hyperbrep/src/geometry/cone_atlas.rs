//! Complete sections of a native cone by a sphere centred off its axis.
//!
//! A sphere meets the single-nappe cone only at axial distances below an
//! exact rational bound `H`. The cone portion `0 <= h <= H` is covered by
//! rational patches that are quadratic along exact unit-circle arcs and
//! linear in `h`: `apex + h (axis + tan(alpha) (a.x X + a.y Y))`. Their apex
//! row collapses to one point, so the patches are injective away from it; when
//! the apex is not on the sphere the pulled-back equation is nonzero on that
//! row and no section reaches it. Each patch is sectioned completely.

use super::curve_implicit::ImplicitCarrier;
use super::cylinder_atlas::{FINE_TANGENTS, section_patches, unit_arcs};
use super::*;

/// Intersects `cone` with an off-axis `sphere` through the exact cone atlas.
pub(super) fn intersect(
    cone: &ConeSurface,
    sphere: &SphereSurface,
    cone_operand: SurfaceIntersectionOperand,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let order = |a: &Real, b: &Real| decided_order(compare_reals(a, b, crate::STRICT_PREDICATES));
    let offset = &cone.apex - &sphere.center;
    // An apex on the sphere would put section points on the collapsed row.
    if order(&offset.norm_squared(), &(&sphere.radius * &sphere.radius))? == Ordering::Equal {
        return Err(GeometryError::UnsupportedIntersection);
    }
    // Every common point has h = axis·(p - apex) <= axis·(c - apex) + r.
    let reach = cone.frame.z.dot(&(&sphere.center - &cone.apex)) + &sphere.radius;
    if order(&reach, &Real::zero())? != Ordering::Greater {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    let mut height = Real::one();
    while order(&height, &reach)? != Ordering::Greater {
        height *= Real::from(2);
    }
    let tangent = (cone.semi_angle.clone().sin() / cone.semi_angle.clone().cos())
        .map_err(GeometryError::Numeric)?;
    let frame = &cone.frame;
    let rim = |(x, y): &(Real, Real)| {
        cone.apex.clone()
            + (frame.z.clone()
                + frame.x.clone() * (&tangent * x)
                + frame.y.clone() * (&tangent * y))
                * &height
    };
    let mut patches = Vec::new();
    for arc in unit_arcs(FINE_TANGENTS)? {
        patches.push(Surface::rational_bezier(
            vec![
                vec![cone.apex.clone(), rim(&arc.start)],
                vec![cone.apex.clone(), rim(&arc.middle)],
                vec![cone.apex.clone(), rim(&arc.end)],
            ],
            vec![
                vec![Real::one(), Real::one()],
                vec![arc.weight.clone(), arc.weight.clone()],
                vec![Real::one(), Real::one()],
            ],
        )?);
    }
    section_patches(patches, ImplicitCarrier::Sphere(sphere), cone_operand)
}

/// Whether the existing coaxial route owns the pair: a sphere centred on the
/// cone axis.
pub(super) fn special_route_applies(
    cone: &ConeSurface,
    sphere: &SphereSurface,
) -> GeometryResult<bool> {
    let offset = &sphere.center - &cone.apex;
    let radial = &offset - &(cone.frame.z.clone() * cone.frame.z.dot(&offset));
    Ok(decided_order(compare_reals(
        &radial.norm_squared(),
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? == Ordering::Equal)
}
