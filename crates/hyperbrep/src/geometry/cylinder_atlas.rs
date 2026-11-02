//! Complete sections of a native cylinder by a bounded or transverse carrier.
//!
//! A cylinder meets a sphere, a ring torus, or a cylinder with a nonparallel
//! axis only inside an exactly bounded height range. That finite portion of
//! the cylinder is covered by an exact atlas of rational quadratic tensor
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
const UPPER_TANGENTS: [(i64, i64); 5] = [(0, 1), (5, 12), (3, 4), (4, 3), (12, 5)];

/// Intersects `cylinder` with `other`, whose intersection with the cylinder is
/// bounded. `cylinder_operand` names the cylinder's position in the query.
pub(super) fn intersect(
    cylinder: &CylinderSurface,
    other: ImplicitCarrier<'_>,
    cylinder_operand: SurfaceIntersectionOperand,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let height = height_bound(cylinder, other)?;
    let mut points = Vec::<GeometricPoint3>::new();
    let mut curves = Vec::new();
    for patch in atlas(cylinder, &height)? {
        let section = implicit_tensor::intersect(other, &patch, cylinder_operand)?;
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
            // A bounded or transverse carrier cannot contain a cylinder patch.
            _ => return Err(GeometryError::UnsupportedIntersection),
        };
        'point: for point in patch_points {
            // A point on an atlas seam is reported by both adjacent patches.
            for existing in &points {
                if existing.coincides_with(&point)? {
                    continue 'point;
                }
            }
            points.push(point);
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
        }
    }
    // Isolated contacts lying on a published piece are part of that piece.
    let mut isolated = Vec::with_capacity(points.len());
    'isolated: for point in points {
        for piece in &curves {
            let curve = piece.curve();
            for parameter in [curve.domain().start(), curve.domain().end()] {
                if curve.point_at(parameter)?.coincides_with(&point)? {
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

/// Exact rational quadratic patches covering the cylinder for |h| <= H.
fn atlas(cylinder: &CylinderSurface, height: &Real) -> GeometryResult<Vec<Surface>> {
    let mut circle = Vec::with_capacity(2 * UPPER_TANGENTS.len());
    for (numerator, denominator) in UPPER_TANGENTS {
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
    let frame = &cylinder.frame;
    let lift = |x: &Real, y: &Real, h: &Real| {
        cylinder.origin.clone()
            + frame.x.clone() * (&cylinder.radius * x)
            + frame.y.clone() * (&cylinder.radius * y)
            + frame.z.clone() * h
    };
    let mut patches = Vec::with_capacity(circle.len());
    for index in 0..circle.len() {
        let (px, py) = &circle[index];
        let (qx, qy) = &circle[(index + 1) % circle.len()];
        // The tangents at unit points p and q meet at (p + q)/(1 + p·q); the
        // middle weight is cos(half angle) = sqrt((1 + p·q)/2), rational here.
        let sum = Real::one() + px * qx + py * qy;
        let mx = ((px + qx) / sum.clone()).map_err(GeometryError::Numeric)?;
        let my = ((py + qy) / sum.clone()).map_err(GeometryError::Numeric)?;
        let weight = ((sum / Real::from(2)).map_err(GeometryError::Numeric)?)
            .sqrt()
            .map_err(GeometryError::Numeric)?;
        let low = -height.clone();
        let row = |x: &Real, y: &Real| vec![lift(x, y, &low), lift(x, y, height)];
        patches.push(Surface::rational_bezier(
            vec![row(px, py), row(&mx, &my), row(qx, qy)],
            vec![
                vec![Real::one(), Real::one()],
                vec![weight.clone(), weight],
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
