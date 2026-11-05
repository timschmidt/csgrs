//! Complete contacts between finite lines and rational Bézier tensors.
//!
//! A line `A + t d` is the intersection of two planes through it. Their
//! pullbacks `g1 = n1·(X - A W)` and `g2 = n2·(X - A W)` through a
//! positive-weight tensor vanish together exactly at the UV preimages of the
//! line's contacts. Hypersolve decomposes `g1 = 0` on the unit square and
//! Hypercurve intersects each branch with every branch of `g2 = 0`. The line
//! parameter of a contact is the source-bound rational form
//! `d·(X - A W) / (|d|^2 W)` at the selected UV point, so no coordinate root is
//! materialized independently.

use super::curve_plane::assemble;
use super::*;
use hypercurve::{CurveIntersectionPairBlockerKind2, CurvePointLocations2};
use hypersolve::bivariate_arithmetic::{
    bivariate_add, bivariate_multiply, bivariate_parameter_derivative, bivariate_scale,
    bivariate_subtract,
};
use hypersolve::{BivariatePolynomial, CurveResultantParameter};

/// Complete exact contacts of a finite line with a rational Bézier tensor.
pub(super) fn intersect_line_tensor(
    curve: &Curve3,
    start: &Point3,
    end: &Point3,
    patch: &RationalBezierSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let direction = end - start;
    let [x, y, z, weight] = surface_point::tensor_coefficients(patch);
    let origin = [&start.x, &start.y, &start.z];
    let coordinates = [&x, &y, &z];
    let displacement: [BivariatePolynomial; 3] = std::array::from_fn(|i| {
        bivariate_subtract(coordinates[i], &bivariate_scale(weight.clone(), origin[i]))
    });
    let linear = |vector: &Vector3| {
        (0..3).fold(BivariatePolynomial::new(Vec::new()), |sum, i| {
            bivariate_add(&sum, &bivariate_scale(displacement[i].clone(), &vector[i]))
        })
    };
    // Any two independent planes through the line define the same contact
    // set. Oblique auxiliary directions avoid pullbacks that are lines of
    // constant v, which the current polynomial topology reports as vertical
    // components; only such topology blockers move on to the next pair.
    let auxiliaries = [
        Vector3::new([1.into(), 2.into(), 3.into()]),
        Vector3::new([3.into(), 1.into(), 2.into()]),
        Vector3::new([2.into(), 3.into(), 1.into()]),
        Vector3::x(),
        Vector3::y(),
    ];
    let mut decomposed = None;
    let mut last_blocker = None;
    for auxiliary in &auxiliaries {
        let first = direction.cross(auxiliary);
        if is_zero(&first.norm_squared())? {
            continue;
        }
        let second = direction.cross(&first);
        let g1 = linear(&first);
        let g2 = linear(&second);
        if is_zero_polynomial(&g1)? || is_zero_polynomial(&g2)? {
            // The tensor lies in a plane through the line; the contact set can
            // contain whole ranges, which this route does not yet assemble.
            return Err(GeometryError::UnsupportedIntersection);
        }
        match (decompose(&g1), decompose(&g2)) {
            (Ok(first_set), Ok(second_set)) => {
                decomposed = Some((g1, g2, first_set, second_set));
                break;
            }
            (Err(error @ GeometryError::PolynomialTopology(_)), _)
            | (_, Err(error @ GeometryError::PolynomialTopology(_))) => {
                last_blocker = Some(error);
            }
            (Err(error), _) | (_, Err(error)) => return Err(error),
        }
    }
    let Some((g1, g2, first_set, second_set)) = decomposed else {
        return Err(last_blocker.unwrap_or(GeometryError::UnsupportedIntersection));
    };
    let along = linear(&direction);
    let denominator = bivariate_scale(weight, &direction.norm_squared());
    if first_set.curves.is_empty() && first_set.points.is_empty()
        || second_set.curves.is_empty() && second_set.points.is_empty()
    {
        return Ok(CurveSurfaceIntersection::None);
    }
    // At a contact X = W P with P on the line, so n·(X_u - A W_u) = W n·S_u
    // and det d(g1, g2)/d(u, v) = W^2 (n1 x n2)·(S_u x S_v). With n1 x n2
    // parallel to d, it vanishes exactly when the line is tangent; an
    // isolated zero of either pullback also has a vanishing Jacobian.
    let derivative = |p: &BivariatePolynomial, axis| bivariate_parameter_derivative(p, axis);
    let jacobian = bivariate_subtract(
        &bivariate_multiply(
            &derivative(&g1, CurveResultantParameter::First),
            &derivative(&g2, CurveResultantParameter::Second),
        ),
        &bivariate_multiply(
            &derivative(&g1, CurveResultantParameter::Second),
            &derivative(&g2, CurveResultantParameter::First),
        ),
    );
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let domain = curve.domain();
    let mut points = Vec::new();
    let mut push = |uv: &hypercurve::CurvePoint2| -> GeometryResult<()> {
        let tangent = uv
            .rational_form(jacobian.clone(), one.clone())?
            .compare_to_real(&Real::zero())?
            == Ordering::Equal;
        let form = uv.rational_form(along.clone(), denominator.clone())?;
        let parameter = Parameter::from_curve(&form.parameter()?)?;
        if domain.contains(&parameter)? {
            points.push(CurveSurfacePoint {
                point: curve.point_at(&parameter)?,
                parameter,
                kind: if tangent {
                    CurveSurfaceContactKind::Repeated
                } else {
                    CurveSurfaceContactKind::Simple
                },
            });
        }
        Ok(())
    };
    for branch in &first_set.curves {
        for other in &second_set.curves {
            let intersection = branch.intersect_curve(other)?;
            if let Some(blocker) = intersection.blockers().first() {
                return Err(match blocker.kind() {
                    CurveIntersectionPairBlockerKind2::Uncertain(reason) => {
                        GeometryError::PlanarClassificationUnresolved(*reason)
                    }
                    CurveIntersectionPairBlockerKind2::IncompleteReplay
                    | CurveIntersectionPairBlockerKind2::SharedComponent => {
                        GeometryError::UnsupportedIntersection
                    }
                });
            }
            if !intersection.overlaps().is_empty()
                || !intersection.parameter_components().is_empty()
            {
                // A shared UV component maps into the line: a coincident range.
                return Err(GeometryError::UnsupportedIntersection);
            }
            for contact in intersection.contacts() {
                push(contact.point())?;
            }
        }
        for isolated in &second_set.points {
            // Only existence is consumed here: every nonempty preimage locus
            // proves that this selected UV point belongs to the first set.
            match branch.point_locations(isolated)? {
                CurvePointLocations2::Locations(locations) if locations.is_empty() => {}
                CurvePointLocations2::Locations(_)
                | CurvePointLocations2::EntireCurve
                | CurvePointLocations2::Components(_) => push(isolated)?,
            }
        }
    }
    for isolated in &first_set.points {
        if isolated
            .rational_form(g2.clone(), one.clone())?
            .compare_to_real(&Real::zero())?
            == Ordering::Equal
        {
            push(isolated)?;
        }
    }
    assemble(curve, points, Vec::new())
}

fn decompose(equation: &BivariatePolynomial) -> GeometryResult<implicit_tensor::UnitSquareZeros> {
    implicit_tensor::unit_square_zeros(equation)
}

fn is_zero(value: &Real) -> GeometryResult<bool> {
    Ok(decided_order(compare_reals(
        value,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? == Ordering::Equal)
}

fn is_zero_polynomial(polynomial: &BivariatePolynomial) -> GeometryResult<bool> {
    for coefficient in polynomial.coefficients.iter().flatten() {
        if !is_zero(coefficient)? {
            return Ok(false);
        }
    }
    Ok(true)
}
