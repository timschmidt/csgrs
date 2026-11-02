//! Complete sections of rational Bézier tensors by implicit analytic carriers.
//!
//! The homogenized sphere, cylinder, cone or ring-torus equation pulled back
//! through a positive-weight tensor is one bivariate polynomial on the unit
//! square. Hypersolve decomposes its complete real zero set there; every
//! branch keeps its selected UV source and lifts through the original tensor
//! as a retained [`Curve3::tensor_image`]. No branch is fitted, sampled or
//! converted to a fixed-degree rational curve.

use super::curve_implicit::ImplicitCarrier;
use super::*;
use hypercurve::{PolynomialCurveSet2, RationalMap2};
use hypersolve::bivariate_arithmetic::{
    bivariate_add, bivariate_multiply, bivariate_scale, bivariate_subtract,
};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};

/// Intersects an implicit carrier with a rational Bézier tensor.
///
/// The tensor is `tensor_operand` of the original query. Its pcurves are the
/// retained UV branches; the analytic carrier's angular pcurves are not
/// algebraic in general and are published as unavailable rather than fitted.
pub(super) fn intersect(
    carrier: ImplicitCarrier<'_>,
    tensor: &Surface,
    tensor_operand: SurfaceIntersectionOperand,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    intersect_with(carrier, tensor, tensor_operand, Injectivity::Certify)
}

/// How a tensor's injectivity on the unit square is established.
#[derive(Clone, Copy)]
pub(super) enum Injectivity {
    /// Require an injective coordinate-plane projection certificate.
    Certify,
    /// The caller constructed the tensor as an injective exact chart, such as
    /// an atlas patch of a cylinder or ring torus with arcs under one turn.
    ByConstruction,
}

pub(super) fn intersect_with(
    carrier: ImplicitCarrier<'_>,
    tensor: &Surface,
    tensor_operand: SurfaceIntersectionOperand,
    injectivity: Injectivity,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let SurfaceGeometry::RationalBezier(patch) = &tensor.data.geometry else {
        return Err(GeometryError::UnsupportedIntersection);
    };
    if let ImplicitCarrier::Cone(cone) = carrier {
        // Positive weights keep the patch in its control hull. A hull on the
        // closed upper side of the apex plane meets only the native nappe of
        // the double-cone equation, and a hull strictly below it misses the
        // nappe entirely. Other patches need a nappe selection.
        let mut signs = [false; 3];
        for row in patch.homogeneous_controls() {
            for point in row {
                let axial = &cone.frame.z[0] * (&point.x - &cone.apex.x * &point.w)
                    + &cone.frame.z[1] * (&point.y - &cone.apex.y * &point.w)
                    + &cone.frame.z[2] * (&point.z - &cone.apex.z * &point.w);
                signs[match decided_order(compare_reals(
                    &axial,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? {
                    Ordering::Less => 0,
                    Ordering::Equal => 1,
                    Ordering::Greater => 2,
                }] = true;
            }
        }
        if signs == [true, false, false] {
            return Ok(SurfaceSurfaceIntersection::None);
        }
        if signs[0] {
            return Err(GeometryError::UnsupportedIntersection);
        }
    }
    let [x, y, z, weight] = surface_point::tensor_coefficients(patch);
    let equation = pullback(carrier, [&x, &y, &z], &weight);
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
        return Ok(SurfaceSurfaceIntersection::ContainedSurface(tensor_operand));
    }
    let set = unit_square_zeros(&equation)?;
    if set.curves.is_empty() && set.points.is_empty() {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    // Distinct UV components are distinct spatial components only when the
    // tensor is injective on the square. An injective coordinate-plane
    // projection is a sufficient whole-rectangle certificate; without one,
    // no incomplete or overlapping trace is published as complete.
    if matches!(injectivity, Injectivity::Certify) {
        certify_injective_tensor([&x, &y, &z], &weight)?;
    }
    let mut curves = Vec::with_capacity(set.curves.len());
    for uv in &set.curves {
        // The spatial chart equals the source chart, matching the retained
        // pcurve's identity correspondence.
        let curve = Curve3::from_tensor_image(TensorCurve3 {
            surface: tensor.clone(),
            source: uv.clone(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
        })?;
        let retained = SurfaceIntersectionPcurve::retained_curve(uv.clone())?;
        let unavailable = SurfaceIntersectionPcurve::unsupported(curve.domain().clone());
        let mut section = match tensor_operand {
            SurfaceIntersectionOperand::First => {
                SurfaceIntersectionCurve::new(curve, retained, unavailable)
            }
            SurfaceIntersectionOperand::Second => {
                SurfaceIntersectionCurve::new(curve, unavailable, retained)
            }
        };
        section.image_certificates = match tensor_operand {
            SurfaceIntersectionOperand::First => [true, false],
            SurfaceIntersectionOperand::Second => [false, true],
        };
        // A reflected branch keeps its original selected graph and exponent.
        section.polynomial_contact_exponent = uv
            .polynomial_branch()
            .map(|branch| branch.defining_exponent());
        curves.push(section);
    }
    let mut points = set
        .points
        .iter()
        .map(|uv| tensor.point_at(uv))
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

/// `W^k F(X/W)` for the homogeneous tensor coordinates.
pub(super) fn pullback(
    carrier: ImplicitCarrier<'_>,
    coordinates: [&BivariatePolynomial; 3],
    weight: &BivariatePolynomial,
) -> BivariatePolynomial {
    let origin = carrier.origin();
    let axis = carrier.axis();
    let origin = [&origin.x, &origin.y, &origin.z];
    let displacement: [BivariatePolynomial; 3] = std::array::from_fn(|i| {
        bivariate_subtract(coordinates[i], &bivariate_scale(weight.clone(), origin[i]))
    });
    let square = |p: &BivariatePolynomial| bivariate_multiply(p, p);
    let squared_norm = bivariate_add(
        &bivariate_add(&square(&displacement[0]), &square(&displacement[1])),
        &square(&displacement[2]),
    );
    let axial = (0..3).fold(BivariatePolynomial::new(Vec::new()), |sum, i| {
        bivariate_add(&sum, &bivariate_scale(displacement[i].clone(), &axis[i]))
    });
    let weight_squared = square(weight);
    let radial_squared = || bivariate_subtract(&squared_norm, &square(&axial));
    match carrier {
        ImplicitCarrier::Sphere(surface) => bivariate_subtract(
            &squared_norm,
            &bivariate_scale(weight_squared, &(&surface.radius * &surface.radius)),
        ),
        ImplicitCarrier::Cylinder(surface) => bivariate_subtract(
            &radial_squared(),
            &bivariate_scale(weight_squared, &(&surface.radius * &surface.radius)),
        ),
        ImplicitCarrier::Cone(surface) => {
            let sin = surface.semi_angle.clone().sin();
            let cos = surface.semi_angle.clone().cos();
            bivariate_subtract(
                &bivariate_scale(radial_squared(), &(&cos * &cos)),
                &bivariate_scale(square(&axial), &(&sin * &sin)),
            )
        }
        ImplicitCarrier::Torus(surface) => {
            let major_squared = &surface.major_radius * &surface.major_radius;
            let minor_squared = &surface.minor_radius * &surface.minor_radius;
            let shifted = bivariate_add(
                &squared_norm,
                &bivariate_scale(weight_squared.clone(), &(&major_squared - minor_squared)),
            );
            bivariate_subtract(
                &square(&shifted),
                &bivariate_scale(
                    bivariate_multiply(&weight_squared, &radial_squared()),
                    &(Real::from(4) * major_squared),
                ),
            )
        }
    }
}

fn certify_injective_tensor(
    coordinates: [&BivariatePolynomial; 3],
    weight: &BivariatePolynomial,
) -> GeometryResult<()> {
    let rectangle = match hypercurve::Aabb2::from_points(&[
        CurvePoint2::new(Real::zero(), Real::zero()),
        CurvePoint2::new(Real::one(), Real::one()),
    ]) {
        Classification::Decided(value) => value,
        Classification::Uncertain(_) => return Err(GeometryError::UnsupportedIntersection),
    };
    for (first, second) in [(0, 1), (0, 2), (1, 2)] {
        let map = RationalMap2::new(
            [coordinates[first].clone(), coordinates[second].clone()],
            weight.clone(),
        );
        if map.certify_injective_on(&rectangle).is_ok() {
            return Ok(());
        }
    }
    Err(GeometryError::UnsupportedIntersection)
}

/// The complete real zero set of a bivariate polynomial on the unit square:
/// selected branches and isolated points in the original UV chart.
pub(super) struct UnitSquareZeros {
    pub(super) curves: Vec<Curve2>,
    pub(super) points: Vec<hypercurve::CurvePoint2>,
}

/// Decomposes `equation` on the unit square. The topology projects over one
/// coordinate and reports lines on which it is constant as vertical
/// components; such a set is decomposed again with exchanged coordinates and
/// mapped back through the exact reflection `(u, v) -> (v, u)`, which retains
/// every selected branch, fiber and source authority. Other blockers, and a
/// set with vertical components in both directions, remain typed.
pub(super) fn unit_square_zeros(equation: &BivariatePolynomial) -> GeometryResult<UnitSquareZeros> {
    use hypersolve::bivariate_arithmetic::bivariate_swap_parameters;
    use hypersolve::bivariate_topology::BivariateTopologyError;
    let domain = BivariateCurveDomain {
        first: [Real::zero(), Real::one()],
        second: [Real::zero(), Real::one()],
    };
    let decompose = |equation: &BivariatePolynomial| -> GeometryResult<PolynomialCurveSet2> {
        let topology =
            decompose_bivariate_curve(equation, &domain, BivariateTopologyConfig::default())
                .map_err(GeometryError::PolynomialTopology)?;
        Ok(PolynomialCurveSet2::from_topology(
            topology,
            AlgebraicFiberRootIsolationConfig::default(),
        )?)
    };
    match decompose(equation) {
        Ok(set) => Ok(UnitSquareZeros {
            curves: set.curves().to_vec(),
            points: set.isolated_points().to_vec(),
        }),
        Err(GeometryError::PolynomialTopology(BivariateTopologyError::VerticalComponent)) => {
            let set = decompose(&bivariate_swap_parameters(equation))?;
            let swap = RationalMap2::new(
                [
                    BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
                    BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]),
                ],
                BivariatePolynomial::new(vec![vec![Real::one()]]),
            );
            Ok(UnitSquareZeros {
                curves: set
                    .curves()
                    .iter()
                    .map(|curve| curve.rational_image(&swap))
                    .collect::<Result<_, _>>()?,
                points: set
                    .isolated_points()
                    .iter()
                    .map(|point| point.rational_image(&swap))
                    .collect::<Result<_, _>>()?,
            })
        }
        Err(error) => Err(error),
    }
}
