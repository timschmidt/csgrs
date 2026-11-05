//! Complete sections of a native cylinder by a bounded or transverse carrier.
//!
//! A cylinder meets a sphere, a ring torus, or a cylinder with a nonparallel
//! axis only inside an exactly bounded height range. That finite portion of
//! the cylinder is covered by an exact atlas of ten rational quadratic tensor
//! patches whose arc endpoints have half-angle tangents `0, ±5/12, ±3/4,
//! ±4/3, ±12/5, ∞`: every endpoint, middle control and weight is rational in
//! the cylinder's own frame. Each patch is sectioned by the complete
//! implicit/tensor route, and the exact pieces are published together.
//!
//! The atlas is an internal exact chart, not either operand's authored chart.
//! Pieces therefore carry no operand pcurves, and adjacent pieces meet at
//! atlas seams that need not be physical vertices.

use super::curve_implicit::ImplicitCarrier;
use super::*;

/// Half-angle tangents `n/d` of the upper atlas points; `1 + t^2` is a square.
/// Upper half-angle tangents of the fine cylinder atlas (ten arcs).
pub(super) const FINE_TANGENTS: &[(i64, i64)] = &[(0, 1), (5, 12), (3, 4), (4, 3), (12, 5)];

/// Upper half-angle tangents of the coarse atlas (four arcs of about 106 and
/// 74 degrees), used where each patch section is costly.
pub(super) const COARSE_TANGENTS: &[(i64, i64)] = &[(0, 1), (4, 3)];

/// Intersects `cylinder` with `other`, whose intersection with the cylinder is
/// bounded. `cylinder_operand` names the cylinder's position in the query.
pub(super) fn intersect(
    cylinder: &CylinderSurface,
    other: ImplicitCarrier<'_>,
    cylinder_operand: SurfaceIntersectionOperand,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let height = height_bound(cylinder, other)?;
    section_patches(atlas(cylinder, &height)?, other, cylinder_operand)
}

/// Sections every exact atlas patch of one operand by `other` and publishes
/// the complete set of exact pieces and deduplicated isolated points.
///
/// The atlas is an internal exact chart, so no operand pcurve is claimed and
/// pieces may meet at seams that are not physical vertices.
pub(super) fn section_patches(
    patches: Vec<Surface>,
    other: ImplicitCarrier<'_>,
    atlas_operand: SurfaceIntersectionOperand,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    section_patches_with_adjacency(patches, other, atlas_operand, None)
}

/// How the closed images of two distinct atlas patches meet.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum PatchContact {
    /// Disjoint closed images.
    Disjoint,
    /// A shared boundary curve (or unknown): boundary points may coincide.
    Edge,
    /// A shared boundary curve on which UV coordinate `0` (u) or `1` (v) is
    /// a boundary value in both patches.
    EdgeAlong(usize),
    /// Only a shared corner: just UV corner points may coincide.
    Corner,
}

/// [`section_patches`] with an optional exact contact relation on distinct
/// patch indices, used to skip spatial comparisons of results that cannot
/// coincide.
pub(super) fn section_patches_with_adjacency(
    patches: Vec<Surface>,
    other: ImplicitCarrier<'_>,
    atlas_operand: SurfaceIntersectionOperand,
    adjacent: Option<&dyn Fn(usize, usize) -> PatchContact>,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    // Atlas patches are injective charts whose images meet only along shared
    // patch boundaries. Results of one patch are already distinct (an
    // isolated zero lies on no branch), and results of different patches can
    // coincide only when both lie on their patches' UV boundaries, so exact
    // spatial comparisons are spent only on such pairs.
    // Per UV coordinate, whether it may equal a boundary value 0 or 1 (an
    // undecided comparison counts as possibly equal).
    let boundary_coordinates = |point: &GeometricPoint3| -> [bool; 2] {
        let Some((_, uv)) = point.surface_source() else {
            return [true, true];
        };
        [(Real::one(), Real::zero()), (Real::zero(), Real::one())].map(|(a, b)| {
            let coordinate = uv.affine_form(a, b, Real::zero());
            [Real::zero(), Real::one()].iter().any(|value| {
                !matches!(
                    coordinate.compare_to_real(value),
                    Ok(Ordering::Less | Ordering::Greater)
                )
            })
        })
    };
    let may_coincide = |first: (&GeometricPoint3, usize),
                        second: (&GeometricPoint3, usize)|
     -> GeometryResult<bool> {
        if first.1 == second.1 {
            return Ok(false);
        }
        let contact = adjacent.map_or(PatchContact::Edge, |adjacent| adjacent(first.1, second.1));
        let (a, b) = (
            boundary_coordinates(first.0),
            boundary_coordinates(second.0),
        );
        Ok(match contact {
            PatchContact::Disjoint => false,
            PatchContact::Edge => (a[0] || a[1]) && (b[0] || b[1]),
            PatchContact::EdgeAlong(index) => a[index] && b[index],
            PatchContact::Corner => a[0] && a[1] && b[0] && b[1],
        })
    };
    let mut points = Vec::<(GeometricPoint3, usize)>::new();
    let mut curves = Vec::new();
    let mut curve_patches = Vec::<usize>::new();
    let other_bounds = carrier_bounds(other);
    for (patch_index, patch) in patches.into_iter().enumerate() {
        // A positive-weight patch lies in its control hull; an exactly
        // disjoint box cannot meet a bounded carrier.
        if let (Some(other_bounds), SurfaceBounds::Bounded(patch_bounds)) =
            (&other_bounds, patch.bounds()?)
            && boxes_disjoint(&patch_bounds, other_bounds)?
        {
            continue;
        }
        // Each patch is an injective chart: an arc of less than one turn
        // (injective in angle) times an injective height or meridian chart.
        let section = implicit_tensor::intersect_with(
            other,
            &patch,
            atlas_operand,
            implicit_tensor::Injectivity::ByConstruction,
        )?;
        let (patch_points, patch_curves) = match section {
            SurfaceSurfaceIntersection::None => (Vec::new(), Vec::new()),
            SurfaceSurfaceIntersection::Point(point) => (vec![*point], Vec::new()),
            SurfaceSurfaceIntersection::Points(points) => (points, Vec::new()),
            SurfaceSurfaceIntersection::Curve(curve) => (Vec::new(), vec![*curve]),
            SurfaceSurfaceIntersection::Curves(curves) => (Vec::new(), curves),
            SurfaceSurfaceIntersection::Components(components) => (
                components.points().to_vec(),
                components.surface_curves().to_vec(),
            ),
            // The admitted carriers cannot contain an atlas patch of the operand.
            _ => return Err(GeometryError::UnsupportedIntersection),
        };
        'point: for point in patch_points {
            // A point on an atlas seam is reported by both adjacent patches.
            for (existing, existing_patch) in &points {
                if may_coincide((existing, *existing_patch), (&point, patch_index))?
                    && existing.coincides_with(&point)?
                {
                    continue 'point;
                }
            }
            points.push((point, patch_index));
        }
        for section in patch_curves {
            let curve = section.curve().clone();
            let domain = curve.domain().clone();
            let mut piece = SurfaceIntersectionCurve::new(
                curve,
                SurfaceIntersectionPcurve::unsupported(domain.clone()),
                SurfaceIntersectionPcurve::unsupported(domain),
            );
            piece.image_certificates = [false, false];
            piece.polynomial_contact_exponent = section.polynomial_contact_exponent;
            curves.push(piece);
            curve_patches.push(patch_index);
        }
    }
    // Isolated contacts lying on a published piece are part of that piece.
    let mut isolated = Vec::with_capacity(points.len());
    'isolated: for (point, point_patch) in points {
        for (piece, piece_patch) in curves.iter().zip(&curve_patches) {
            let curve = piece.curve();
            for parameter in [curve.domain().start(), curve.domain().end()] {
                let end = curve.point_at(parameter)?;
                if may_coincide((&end, *piece_patch), (&point, point_patch))?
                    && end.coincides_with(&point)?
                {
                    continue 'isolated;
                }
            }
        }
        isolated.push(point);
    }
    Ok(match (isolated.len(), curves.len()) {
        (0, 0) => SurfaceSurfaceIntersection::None,
        (1, 0) => SurfaceSurfaceIntersection::Point(Box::new(isolated.pop().expect("one point"))),
        (_, 0) => SurfaceSurfaceIntersection::Points(isolated),
        (0, 1) => SurfaceSurfaceIntersection::Curve(Box::new(curves.pop().expect("one curve"))),
        (0, _) => SurfaceSurfaceIntersection::Curves(curves),
        _ => SurfaceSurfaceIntersection::Components(Box::new(SurfaceIntersectionComponents::new(
            isolated,
            Vec::new(),
            curves,
        ))),
    })
}

/// A rational `H` with every intersection point strictly inside the cylinder
/// heights `(-H, H)`, found by exact doubling against a certified bound.
fn height_bound(cylinder: &CylinderSurface, other: ImplicitCarrier<'_>) -> GeometryResult<Real> {
    let axis = &cylinder.frame.z;
    let absolute = |value: Real| -> GeometryResult<Real> {
        Ok(
            if decided_order(compare_reals(
                &value,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == Ordering::Less
            {
                -value
            } else {
                value
            },
        )
    };
    // Bound `B` and positive scale `s` with |h| * s <= B for every common point.
    let (bound, scale_squared) = match other {
        ImplicitCarrier::Sphere(sphere) => (
            absolute(axis.dot(&(&sphere.center - &cylinder.origin)))? + &sphere.radius,
            Real::one(),
        ),
        ImplicitCarrier::Torus(torus) => (
            absolute(axis.dot(&(&torus.center - &cylinder.origin)))?
                + &torus.major_radius
                + &torus.minor_radius,
            Real::one(),
        ),
        ImplicitCarrier::Cylinder(second) => {
            // Distance to the second axis is at least |h| |a1 x a2| minus the
            // origin offset and first radius; it equals the second radius.
            let offset = &second.origin - &cylinder.origin;
            let mut bound = &cylinder.radius + &second.radius;
            for i in 0..3 {
                bound += absolute(offset[i].clone())?;
            }
            (bound, axis.cross(&second.frame.z).norm_squared())
        }
        ImplicitCarrier::Cone(_) => return Err(GeometryError::UnsupportedIntersection),
    };
    if decided_order(compare_reals(
        &scale_squared,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != Ordering::Greater
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let bound_squared = &bound * &bound;
    let mut height = Real::one();
    for _ in 0..128 {
        if decided_order(compare_reals(
            &(&height * &height * &scale_squared),
            &bound_squared,
            crate::STRICT_PREDICATES,
        ))? == Ordering::Greater
        {
            return Ok(height);
        }
        height *= Real::from(2);
    }
    Err(GeometryError::UnsupportedIntersection)
}

/// One exact rational quadratic arc of the unit circle: end points, the
/// tangent intersection and the middle weight cos(half angle), all rational.
pub(super) struct UnitArc {
    pub(super) start: (Real, Real),
    pub(super) middle: (Real, Real),
    pub(super) end: (Real, Real),
    pub(super) weight: Real,
}

/// Atlas arcs of the unit circle, counterclockwise from (1, 0), whose seams
/// have the given upper half-angle tangents (each with `1 + t^2` a square),
/// their mirror images, and infinity.
pub(super) fn unit_arcs(tangents: &[(i64, i64)]) -> GeometryResult<Vec<UnitArc>> {
    let mut circle = Vec::with_capacity(2 * tangents.len());
    for &(numerator, denominator) in tangents {
        let t =
            (Real::from(numerator) / Real::from(denominator)).map_err(GeometryError::Numeric)?;
        let scale = Real::one() + &t * &t;
        circle.push((
            ((Real::one() - &t * &t) / scale.clone()).map_err(GeometryError::Numeric)?,
            ((Real::from(2) * t) / scale).map_err(GeometryError::Numeric)?,
        ));
    }
    circle.push((-Real::one(), Real::zero()));
    let lower = circle[1..circle.len() - 1]
        .iter()
        .rev()
        .map(|(x, y)| (x.clone(), -y.clone()))
        .collect::<Vec<_>>();
    circle.extend(lower);
    let mut arcs = Vec::with_capacity(circle.len());
    for index in 0..circle.len() {
        let (px, py) = circle[index].clone();
        let (qx, qy) = circle[(index + 1) % circle.len()].clone();
        // The tangents at unit points p and q meet at (p + q)/(1 + p·q); the
        // middle weight is cos(half angle) = sqrt((1 + p·q)/2), rational here.
        let sum = Real::one() + &px * &qx + &py * &qy;
        let middle = (
            ((&px + &qx) / sum.clone()).map_err(GeometryError::Numeric)?,
            ((&py + &qy) / sum.clone()).map_err(GeometryError::Numeric)?,
        );
        let weight = ((sum / Real::from(2)).map_err(GeometryError::Numeric)?)
            .sqrt()
            .map_err(GeometryError::Numeric)?;
        arcs.push(UnitArc {
            start: (px, py),
            middle,
            end: (qx, qy),
            weight,
        });
    }
    Ok(arcs)
}

/// Exact rational quadratic patches covering the cylinder for |h| <= H.
fn atlas(cylinder: &CylinderSurface, height: &Real) -> GeometryResult<Vec<Surface>> {
    let frame = &cylinder.frame;
    let lift = |x: &Real, y: &Real, h: &Real| {
        cylinder.origin.clone()
            + frame.x.clone() * (&cylinder.radius * x)
            + frame.y.clone() * (&cylinder.radius * y)
            + frame.z.clone() * h
    };
    let arcs = unit_arcs(FINE_TANGENTS)?;
    let mut patches = Vec::with_capacity(arcs.len());
    for arc in &arcs {
        let low = -height.clone();
        let row = |(x, y): &(Real, Real)| vec![lift(x, y, &low), lift(x, y, height)];
        patches.push(Surface::rational_bezier(
            vec![row(&arc.start), row(&arc.middle), row(&arc.end)],
            vec![
                vec![Real::one(), Real::one()],
                vec![arc.weight.clone(), arc.weight.clone()],
                vec![Real::one(), Real::one()],
            ],
        )?);
    }
    Ok(patches)
}

/// Whether a supported special route owns the pair: parallel cylinder axes, or
/// a sphere or torus centred on the cylinder axis (torus axis parallel too).
pub(super) fn special_route_applies(
    cylinder: &CylinderSurface,
    other: ImplicitCarrier<'_>,
) -> GeometryResult<bool> {
    let zero = |value: &Real| -> GeometryResult<bool> {
        Ok(decided_order(compare_reals(
            value,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == Ordering::Equal)
    };
    let axis = &cylinder.frame.z;
    let off_axis = |center: &Point3| -> GeometryResult<bool> {
        let offset = center - &cylinder.origin;
        let radial = &offset - &(axis.clone() * axis.dot(&offset));
        Ok(!zero(&radial.norm_squared())?)
    };
    Ok(match other {
        ImplicitCarrier::Cylinder(second) => zero(&axis.cross(&second.frame.z).norm_squared())?,
        ImplicitCarrier::Sphere(sphere) => !off_axis(&sphere.center)?,
        ImplicitCarrier::Torus(torus) => {
            zero(&axis.cross(&torus.frame.z).norm_squared())? && !off_axis(&torus.center)?
        }
        ImplicitCarrier::Cone(_) => true,
    })
}

/// An exact box containing a bounded carrier, or `None` when unbounded.
fn carrier_bounds(carrier: ImplicitCarrier<'_>) -> Option<Aabb> {
    let cube = |center: &Point3, half: Real| {
        Aabb::new(
            Point3::new(&center.x - &half, &center.y - &half, &center.z - &half),
            Point3::new(&center.x + &half, &center.y + &half, &center.z + &half),
        )
    };
    match carrier {
        ImplicitCarrier::Sphere(sphere) => Some(cube(&sphere.center, sphere.radius.clone())),
        ImplicitCarrier::Torus(torus) => Some(cube(
            &torus.center,
            &torus.major_radius + &torus.minor_radius,
        )),
        ImplicitCarrier::Cylinder(_) | ImplicitCarrier::Cone(_) => None,
    }
}

/// Whether two exact boxes are strictly separated along some axis.
fn boxes_disjoint(first: &Aabb, second: &Aabb) -> GeometryResult<bool> {
    let less = |a: &Real, b: &Real| -> GeometryResult<bool> {
        Ok(decided_order(compare_reals(a, b, crate::STRICT_PREDICATES))? == Ordering::Less)
    };
    for (a, b) in [
        (
            (&first.mins.x, &first.maxs.x),
            (&second.mins.x, &second.maxs.x),
        ),
        (
            (&first.mins.y, &first.maxs.y),
            (&second.mins.y, &second.maxs.y),
        ),
        (
            (&first.mins.z, &first.maxs.z),
            (&second.mins.z, &second.maxs.z),
        ),
    ] {
        if less(a.1, b.0)? || less(b.1, a.0)? {
            return Ok(true);
        }
    }
    Ok(false)
}
