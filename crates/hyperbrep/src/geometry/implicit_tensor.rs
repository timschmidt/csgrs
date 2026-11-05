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
    tensor_image_sections(tensor, &set, tensor_operand)
}

/// Publishes a complete unit-square zero set of an injective tensor as
/// tensor-image sections. The tensor's pcurves are the retained UV branches;
/// the other operand's pcurves are published as unavailable, not fitted.
pub(super) fn tensor_image_sections(
    tensor: &Surface,
    set: &UnitSquareZeros,
    tensor_operand: SurfaceIntersectionOperand,
) -> GeometryResult<SurfaceSurfaceIntersection> {
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

pub(super) fn certify_injective_tensor(
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
/// components. Such a set is first split exactly into its pure-`u` factor,
/// pure-`v` factor and primitive remainder: pure-`u` components become exact
/// chords `u = r`, the other parts decompose natively, branches of different
/// parts are split at their exact crossings, and isolated points lying on
/// another part are dropped. When that fails (for example a pure-`u` root
/// without a scalar view), the set is decomposed with exchanged coordinates
/// and mapped back through the exact reflection `(u, v) -> (v, u)`, which
/// retains every selected branch, fiber and source authority. Other
/// blockers remain typed.
pub(super) fn unit_square_zeros(equation: &BivariatePolynomial) -> GeometryResult<UnitSquareZeros> {
    use hypersolve::bivariate_topology::BivariateTopologyError;
    let vertical = |error: &GeometryError| {
        matches!(
            error,
            GeometryError::PolynomialTopology(BivariateTopologyError::VerticalComponent)
        )
    };
    match native_zeros(equation) {
        Err(error) if vertical(&error) => match axis_factor_zeros(equation) {
            Ok(zeros) => Ok(zeros),
            Err(axis_error) => match swapped_zeros(equation) {
                Err(error) if vertical(&error) => Err(axis_error),
                other => other,
            },
        },
        other => other,
    }
}

fn decompose_unit_square(equation: &BivariatePolynomial) -> GeometryResult<PolynomialCurveSet2> {
    let domain = BivariateCurveDomain {
        first: [Real::zero(), Real::one()],
        second: [Real::zero(), Real::one()],
    };
    let topology = decompose_bivariate_curve(equation, &domain, BivariateTopologyConfig::default())
        .map_err(GeometryError::PolynomialTopology)?;
    Ok(PolynomialCurveSet2::from_topology(
        topology,
        AlgebraicFiberRootIsolationConfig::default(),
    )?)
}

fn native_zeros(equation: &BivariatePolynomial) -> GeometryResult<UnitSquareZeros> {
    let set = decompose_unit_square(equation)?;
    Ok(UnitSquareZeros {
        curves: set.curves().to_vec(),
        points: set.isolated_points().to_vec(),
    })
}

fn swapped_zeros(equation: &BivariatePolynomial) -> GeometryResult<UnitSquareZeros> {
    use hypersolve::bivariate_arithmetic::bivariate_swap_parameters;
    let set = decompose_unit_square(&bivariate_swap_parameters(equation))?;
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

/// The zero set of `f(u) g(v) R(u, v)` as the union of the three parts' zero
/// sets, with branches split at their mutual crossings. The factors come from
/// Hypersolve's exact axis-content extraction, so the parts share no curve
/// component and meet only at finitely many points.
fn axis_factor_zeros(equation: &BivariatePolynomial) -> GeometryResult<UnitSquareZeros> {
    use super::curve_implicit;
    use hypersolve::bivariate_topology::BivariateTopologyError;
    use hypersolve::curve_resultant::{
        BivariatePolynomialAxisFactorStatus, extract_bivariate_polynomial_system_axis_factors,
    };
    let report = extract_bivariate_polynomial_system_axis_factors(equation, equation);
    if report.status != BivariatePolynomialAxisFactorStatus::Reduced {
        return Err(GeometryError::PolynomialTopology(
            BivariateTopologyError::VerticalComponent,
        ));
    }
    let [remainder, _] = report
        .reduced_equations
        .expect("a reduced axis-factor report carries its primitive equations");
    // The pure-u factor's components are the vertical chords u = r over its
    // roots in [0, 1]. Hypercurve's native line authority intersects chords
    // with branches of either remaining part in their own chart; a root
    // without a scalar view has no exact chord yet and stays typed.
    let mut chords = Vec::new();
    let mut parts = Vec::new();
    if report.first_parameter_factor.len() > 1 {
        for (root, _) in curve_implicit::roots(power_to_bernstein(&report.first_parameter_factor))?
        {
            let Some(value) = root.try_real() else {
                return Err(GeometryError::UnsupportedIntersection);
            };
            chords.push(Curve2::from(hypercurve::LineSeg2::try_new(
                hypercurve::Point2::new(value.clone(), Real::zero()),
                hypercurve::Point2::new(value.clone(), Real::one()),
            )?));
        }
        parts.push(BivariatePolynomial::new(
            report
                .first_parameter_factor
                .iter()
                .map(|coefficient| vec![coefficient.clone()])
                .collect(),
        ));
    }
    if report.second_parameter_factor.len() > 1 {
        parts.push(BivariatePolynomial::new(vec![
            report.second_parameter_factor.clone(),
        ]));
    }
    let remainder_is_constant = remainder.coefficients.iter().enumerate().all(|(i, row)| {
        row.iter()
            .enumerate()
            .all(|(j, c)| i + j == 0 || c.definitely_zero())
    });
    if !remainder_is_constant {
        parts.push(remainder);
    }
    let mut zeros = Vec::with_capacity(parts.len());
    for (index, part) in parts.iter().enumerate() {
        zeros.push(if index == 0 && report.first_parameter_factor.len() > 1 {
            UnitSquareZeros {
                curves: std::mem::take(&mut chords),
                points: Vec::new(),
            }
        } else {
            native_zeros(part)?
        });
    }
    // Exact crossing parameters of each branch with every other part.
    let mut cuts: Vec<Vec<Vec<Parameter>>> = zeros
        .iter()
        .map(|part| vec![Vec::new(); part.curves.len()])
        .collect();
    for first in 0..zeros.len() {
        for second in first + 1..zeros.len() {
            for (a, curve_a) in zeros[first].curves.iter().enumerate() {
                for (b, curve_b) in zeros[second].curves.iter().enumerate() {
                    let crossing = curve_a.intersect_curve(curve_b)?;
                    if !crossing.blockers().is_empty()
                        || !crossing.overlaps().is_empty()
                        || !crossing.parameter_components().is_empty()
                    {
                        return Err(GeometryError::UnsupportedIntersection);
                    }
                    for contact in crossing.contacts() {
                        cuts[first][a].push(Parameter::from_curve(&contact.first().parameter()?)?);
                        cuts[second][b]
                            .push(Parameter::from_curve(&contact.second().parameter()?)?);
                    }
                }
            }
        }
    }
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let mut result = UnitSquareZeros {
        curves: Vec::new(),
        points: Vec::new(),
    };
    for (index, part) in zeros.into_iter().enumerate() {
        for (curve, mut parameters) in part.curves.into_iter().zip(cuts[index].drain(..)) {
            let start = Parameter::from_curve(curve.parameter_domain().start())?;
            let end = Parameter::from_curve(curve.parameter_domain().end())?;
            // Strictly interior cuts, ordered and without repeats.
            let mut interior: Vec<Parameter> = Vec::new();
            for parameter in parameters.drain(..) {
                if parameter.compare(&start)? != Ordering::Greater
                    || parameter.compare(&end)? != Ordering::Less
                {
                    continue;
                }
                let mut position = interior.len();
                let mut duplicate = false;
                for (k, existing) in interior.iter().enumerate() {
                    match existing.compare(&parameter)? {
                        Ordering::Equal => {
                            duplicate = true;
                            break;
                        }
                        Ordering::Greater => {
                            position = k;
                            break;
                        }
                        Ordering::Less => {}
                    }
                }
                if !duplicate {
                    interior.insert(position, parameter);
                }
            }
            if interior.is_empty() {
                result.curves.push(curve);
                continue;
            }
            let mut bounds = vec![start];
            bounds.extend(interior);
            bounds.push(end);
            for window in bounds.windows(2) {
                result.curves.push(
                    curve.subcurve(window[0].curve_parameter(), window[1].curve_parameter())?,
                );
            }
        }
        'point: for point in part.points {
            for (other, polynomial) in parts.iter().enumerate() {
                if other != index
                    && point
                        .rational_form(polynomial.clone(), one.clone())?
                        .compare_to_real(&Real::zero())?
                        == Ordering::Equal
                {
                    continue 'point;
                }
            }
            result.points.push(point);
        }
    }
    Ok(result)
}

/// Bernstein coefficients on `[0, 1]` of a power-basis polynomial:
/// `b_k = sum_{i <= k} C(k, i) / C(n, i) a_i`.
fn power_to_bernstein(power: &[Real]) -> Vec<Real> {
    let degree = power.len() - 1;
    let binomial = |n: usize, k: usize| -> Real {
        let mut value = Real::one();
        for i in 0..k {
            value = (value * Real::from((n - i) as i64) / Real::from((i + 1) as i64))
                .expect("nonzero factorial denominators");
        }
        value
    };
    (0..=degree)
        .map(|k| {
            (0..=k).fold(Real::zero(), |total, i| {
                total
                    + (binomial(k, i) / binomial(degree, i))
                        .expect("binomial coefficients are positive")
                        * &power[i]
            })
        })
        .collect()
}

#[cfg(test)]
mod axis_factor_tests {
    use super::*;
    use hypersolve::bivariate_arithmetic::bivariate_multiply;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn poly(rows: Vec<Vec<Real>>) -> BivariatePolynomial {
        BivariatePolynomial::new(rows)
    }

    /// (2u - 1)(2v - 1) has line components in both directions: the two lines
    /// are split at their crossing (1/2, 1/2) into four branches.
    #[test]
    fn crossing_axis_lines_split_at_their_crossing() {
        let lines = poly(vec![
            vec![1.into(), (-2).into()],
            vec![(-2).into(), 4.into()],
        ]);
        let zeros = unit_square_zeros(&lines).unwrap();
        assert_eq!(zeros.curves.len(), 4);
        assert!(zeros.points.is_empty());
        let mut ending_at_centre = 0;
        for curve in &zeros.curves {
            let (start, end) = curve.parameter_endpoints();
            for parameter in [start, end] {
                let point = curve.point_at(parameter).unwrap();
                let at_half = |a: i64, b: i64| {
                    point
                        .affine_form(a.into(), b.into(), Real::zero())
                        .compare_to_real(&q(1, 2))
                        .unwrap()
                        == Ordering::Equal
                };
                if at_half(1, 0) && at_half(0, 1) {
                    ending_at_centre += 1;
                }
            }
        }
        assert_eq!(ending_at_centre, 4);
    }

    /// Adding the circle (u - 1/2)^2 + (v - 1/2)^2 = 1/16: each line is cut
    /// at the other line and twice by the circle (four pieces each), and the
    /// circle's two branches (split at u = 1/4, 3/4) are cut once each by
    /// u = 1/2, so the set has twelve branches.
    #[test]
    fn remainder_branches_split_at_axis_lines() {
        let lines = poly(vec![
            vec![1.into(), (-2).into()],
            vec![(-2).into(), 4.into()],
        ]);
        let circle = poly(vec![
            vec![q(7, 16), (-1).into(), 1.into()],
            vec![(-1).into()],
            vec![1.into()],
        ]);
        let zeros = unit_square_zeros(&bivariate_multiply(&lines, &circle)).unwrap();
        assert_eq!(zeros.curves.len(), 12);
        assert!(zeros.points.is_empty());
    }

    /// (8u^3 - 2)(2v - 1): the vertical component u = cbrt(1/4) has no
    /// scalar view, so no exact chord exists yet and the set stays typed.
    #[test]
    fn axis_roots_without_scalar_views_stay_typed() {
        let cubic = poly(vec![
            vec![(-2).into()],
            vec![0.into()],
            vec![0.into()],
            vec![8.into()],
        ]);
        let horizontal = poly(vec![vec![(-1).into(), 2.into()]]);
        assert!(matches!(
            unit_square_zeros(&bivariate_multiply(&cubic, &horizontal)),
            Err(GeometryError::UnsupportedIntersection)
        ));
    }
}
