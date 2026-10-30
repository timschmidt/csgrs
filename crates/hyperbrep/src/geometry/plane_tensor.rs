//! Complete polynomial pullbacks and checked spatial images of plane sections.

use super::*;
use hypercurve::{PolynomialCurveSet2, RationalMap2};
use hypersolve::bivariate_arithmetic::{bivariate_add, bivariate_scale, bivariate_subtract};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};

pub(super) fn intersect(
    plane: &PlaneSurface,
    surface: &RationalBezierSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    // A positive-weight tensor lies in its control-point convex hull. Strict
    // one-sided homogeneous plane values prove an empty section without
    // constructing a projection polynomial. Zeros mixed with one sign are
    // deliberately left to complete decomposition (boundary contacts matter).
    let normal = plane.u.cross(&plane.v);
    let mut signs = [false; 3];
    'controls: for row in surface.homogeneous_controls() {
        for point in row {
            let displacement = Vector3::new([
                &point.x - &plane.origin.x * &point.w,
                &point.y - &plane.origin.y * &point.w,
                &point.z - &plane.origin.z * &point.w,
            ]);
            let Ok(order) = decided_order(compare_reals(
                &normal.dot(&displacement),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            )) else {
                signs = [true; 3];
                break 'controls;
            };
            let index = match order {
                Ordering::Less => 0,
                Ordering::Equal => 1,
                Ordering::Greater => 2,
            };
            signs[index] = true;
            if signs[0] && signs[2] {
                break 'controls;
            }
        }
    }
    if signs == [true, false, false] || signs == [false, false, true] {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    if signs == [false, true, false] {
        return Ok(SurfaceSurfaceIntersection::ContainedSurface(
            SurfaceIntersectionOperand::Second,
        ));
    }
    let (equation, map) = pullback(plane, surface)?;
    let mut zero = true;
    for coefficient in equation.coefficients.iter().flatten() {
        if decided_order(compare_reals(
            coefficient,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != Ordering::Equal
        {
            zero = false;
            break;
        }
    }
    if zero {
        return Ok(SurfaceSurfaceIntersection::ContainedSurface(
            SurfaceIntersectionOperand::Second,
        ));
    }
    let domain = BivariateCurveDomain {
        first: [Real::zero(), Real::one()],
        second: [Real::zero(), Real::one()],
    };
    let topology =
        decompose_bivariate_curve(&equation, &domain, BivariateTopologyConfig::default())
            .map_err(GeometryError::PolynomialTopology)?;
    let set =
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())?;
    if set.is_empty() {
        return Ok(SurfaceSurfaceIntersection::None);
    }

    let rectangle = hypercurve::Aabb2::from_points(&[
        CurvePoint2::new(Real::zero(), Real::zero()),
        CurvePoint2::new(Real::one(), Real::one()),
    ]);
    let rectangle = match rectangle {
        Classification::Decided(value) => value,
        Classification::Uncertain(_) => return Err(GeometryError::UnsupportedIntersection),
    };
    // UV topology alone does not rule out distinct parameter points with the
    // same spatial image. This whole-rectangle certificate also excludes rank
    // loss. Unproved image topology cannot publish a partial section as complete.
    map.certify_injective_on(&rectangle)?;
    let mut curves = Vec::with_capacity(set.curves().len());
    for uv in set.curves() {
        let projected = uv.rational_image(&map)?;
        let curve = Curve3::from_planar_image(PlanarCurve3 {
            source: projected.clone(),
            origin: plane.origin.clone(),
            u: plane.u.clone(),
            v: plane.v.clone(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
        })?;
        let mut section =
            SurfaceIntersectionCurve::from_certified_exact_pcurves(curve, projected, uv.clone())?;
        section.polynomial_contact_exponent = Some(
            uv.polynomial_branch()
                .expect("topology supplies a selected graph")
                .defining_exponent(),
        );
        curves.push(section);
    }
    let plane = Surface::plane(plane.origin.clone(), plane.u.clone(), plane.v.clone())?;
    let mut points = set
        .isolated_points()
        .iter()
        .map(|uv| plane.point_at(&uv.rational_image(&map)?))
        .collect::<GeometryResult<Vec<_>>>()?;
    Ok(match (points.len(), curves.len()) {
        (0, 0) => SurfaceSurfaceIntersection::None,
        (1, 0) => SurfaceSurfaceIntersection::Point(Box::new(points.pop().expect("one point"))),
        (_, 0) => SurfaceSurfaceIntersection::Points(points),
        (0, 1) => SurfaceSurfaceIntersection::Curve(Box::new(curves.pop().expect("one curve"))),
        (0, _) => SurfaceSurfaceIntersection::Curves(curves),
        _ => SurfaceSurfaceIntersection::Components(Box::new(SurfaceIntersectionComponents::new(
            points,
            Vec::new(),
            curves,
        ))),
    })
}

fn pullback(
    plane: &PlaneSurface,
    surface: &RationalBezierSurface,
) -> GeometryResult<(BivariatePolynomial, RationalMap2)> {
    let [x, y, z, weight] = surface_point::tensor_coefficients(surface);
    let coordinates = [x, y, z];
    let normal = plane.u.cross(&plane.v);
    let displaced: [BivariatePolynomial; 3] = std::array::from_fn(|i| {
        bivariate_subtract(
            &coordinates[i],
            &bivariate_scale(
                weight.clone(),
                [&plane.origin.x, &plane.origin.y, &plane.origin.z][i],
            ),
        )
    });
    let linear = |direction: &Vector3| {
        (0..3).fold(BivariatePolynomial::new(Vec::new()), |sum, i| {
            bivariate_add(&sum, &bivariate_scale(displaced[i].clone(), &direction[i]))
        })
    };
    let equation = linear(&normal);
    // Reuse the plane-frame owner to obtain the exact dual basis. On the
    // pullback zero set, lifting these projected coordinates equals the
    // original homogeneous surface image, not an approximation of it.
    let origin = Point3::origin();
    let basis = [Vector3::x(), Vector3::y(), Vector3::z()]
        .map(|axis| project_point_to_plane_frame(&origin, &plane.u, &plane.v, &Point3::from(axis)));
    let [a, b, c] = basis;
    let [a, b, c] = [a?, b?, c?];
    let plane_x = linear(&Vector3::new([a.x, b.x, c.x]));
    let plane_y = linear(&Vector3::new([a.y, b.y, c.y]));
    let map = RationalMap2::new([plane_x, plane_y], weight);
    Ok((equation, map))
}

impl Surface {
    /// Checks the entire retained polynomial branch, not a finite set of samples.
    pub(crate) fn certifies_polynomial_section_image(
        &self,
        curve: &Curve3,
        pcurve: &Curve2,
        scale: &Real,
        offset: &Real,
    ) -> GeometryResult<bool> {
        let SurfaceGeometry::RationalBezier(surface) = &self.data.geometry else {
            return Ok(false);
        };
        let CurveGeometry3::PlanarImage(image) = &curve.data.geometry else {
            return Ok(false);
        };
        let Some(branch) = pcurve.polynomial_branch() else {
            return Ok(false);
        };
        if branch.image_transform().is_some() || !branch.rational_maps().is_empty() {
            return Ok(false);
        }
        let plane = PlaneSurface {
            origin: image.origin.clone(),
            u: image.u.clone(),
            v: image.v.clone(),
        };
        let (equation, map) = pullback(&plane, surface)?;
        if hypersolve::divide_bivariate_polynomial_exact(
            &equation,
            branch.topology().geometric_equation(),
        )
        .is_none()
        {
            return Ok(false);
        }
        let source = pcurve.rational_image(&map)?;
        let inverse = (Real::one() / scale).map_err(GeometryError::Numeric)?;
        let expected = PlanarCurve3 {
            source,
            origin: image.origin.clone(),
            u: image.u.clone(),
            v: image.v.clone(),
            source_scale: inverse.clone(),
            source_offset: -offset * inverse,
        };
        image.parameterization_equals(&expected)
    }
}

impl SurfaceIntersectionCurve {
    /// Defining-factor exponent of a checked plane/tensor pullback branch.
    ///
    /// Where both sides of a regular open arc belong to the parameter domain,
    /// even exponents keep the signed plane value on the same side and odd
    /// exponents change side. An exponent above one does not
    /// certify transversality. This is not a local order at a singular endpoint.
    /// `None` means this evidence was not established for the construction.
    /// Restrictions, reversal and operand exchange preserve it. Native pcurves
    /// retain the source polynomial and reconstruct its factor evidence on replay.
    pub const fn polynomial_contact_exponent(&self) -> Option<usize> {
        self.polynomial_contact_exponent
    }
}
