//! Complete contacts between retained tensor-image curves and further surfaces.
//!
//! A tensor-image curve is `S(c(t))` for a UV source `c` on a positive-weight
//! tensor `S`. A plane or implicit analytic carrier `F` meets it exactly where
//! `c` meets the zero set of the pulled-back polynomial `G = W^k F(S/W)` on the
//! unit square. Hypersolve decomposes that zero set completely; Hypercurve
//! intersects `c` with every resulting branch and locates isolated points on
//! `c`. A generated nonrational edge therefore remains intersectable with a
//! third surface without fitting, sampling or materializing coordinates.

use super::curve_implicit::ImplicitCarrier;
use super::curve_plane::{assemble, classify_derivative};
use super::*;
use hypercurve::{CurveIntersectionPairBlockerKind2, CurveParameterSet2, CurvePointLocations2};
use hypersolve::BivariatePolynomial;

/// Complete exact contacts of a tensor-image curve with a plane, sphere,
/// cylinder, cone or ring torus. Returns `None` for other surface families.
pub(super) fn intersect_tensor_image(
    curve: &Curve3,
    image: &TensorCurve3,
    surface: &Surface,
) -> Option<GeometryResult<CurveSurfaceIntersection>> {
    let SurfaceGeometry::RationalBezier(patch) = &image.surface.data.geometry else {
        unreachable!("tensor images retain rational Bézier tensors");
    };
    let cone = match &surface.data.geometry {
        SurfaceGeometry::Cone(cone) => Some(cone),
        _ => None,
    };
    let equation = match &surface.data.geometry {
        SurfaceGeometry::Plane(plane) => match plane_tensor::pullback(plane, patch) {
            Ok((equation, _)) => equation,
            Err(error) => return Some(Err(error)),
        },
        geometry => {
            let carrier = ImplicitCarrier::from_surface(geometry)?;
            let [x, y, z, weight] = surface_point::tensor_coefficients(patch);
            implicit_tensor::pullback(carrier, [&x, &y, &z], &weight)
        }
    };
    Some(intersect(curve, image, &equation, cone))
}

fn intersect(
    curve: &Curve3,
    image: &TensorCurve3,
    equation: &BivariatePolynomial,
    cone: Option<&ConeSurface>,
) -> GeometryResult<CurveSurfaceIntersection> {
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
    let domain = curve.domain();
    if zero {
        // The whole tensor lies on the carrier's equation. Only a cone needs a
        // nappe decision; certify both endpoints and the whole image instead
        // of guessing a subrange.
        if let Some(cone) = cone {
            let SurfaceGeometry::RationalBezier(patch) = &image.surface.data.geometry else {
                unreachable!("tensor images retain rational Bézier tensors");
            };
            for row in patch.homogeneous_controls() {
                for point in row {
                    let axial = &cone.frame.z[0] * (&point.x - &cone.apex.x * &point.w)
                        + &cone.frame.z[1] * (&point.y - &cone.apex.y * &point.w)
                        + &cone.frame.z[2] * (&point.z - &cone.apex.z * &point.w);
                    if decided_order(compare_reals(
                        &axial,
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? == Ordering::Less
                    {
                        return Err(GeometryError::UnsupportedIntersection);
                    }
                }
            }
        }
        return Ok(CurveSurfaceIntersection::Contained);
    }
    let set = implicit_tensor::unit_square_zeros(equation)?;
    let inverse = (Real::one() / &image.source_scale).map_err(GeometryError::Numeric)?;
    let offset = -&image.source_offset * &inverse;
    let spatial = |source: &hypercurve::CurveParameter2| -> GeometryResult<Parameter> {
        Parameter::from_curve(source)?.affine_image(&inverse, &offset)
    };
    let source = &image.source;
    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    let push = |parameter: Parameter,
                kind: CurveSurfaceContactKind,
                points: &mut Vec<CurveSurfacePoint>|
     -> GeometryResult<()> {
        if !domain.contains(&parameter)? {
            return Ok(());
        }
        let point = curve.point_at(&parameter)?;
        if let Some(cone) = cone
            && point.compare_linear_form(
                &cone.frame.z,
                &cone.frame.z.dot(&Vector3::from(cone.apex.clone())),
            )? == Ordering::Less
        {
            // A root of the double-cone equation on the opposite nappe.
            return Ok(());
        }
        points.push(CurveSurfacePoint {
            point,
            parameter,
            kind,
        });
        Ok(())
    };
    for branch in &set.curves {
        let intersection = source.intersect_curve(branch)?;
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
        for contact in intersection.contacts() {
            let parameter = spatial(&contact.first().parameter()?)?;
            // The zero set's gradient is normal to each regular branch, so a
            // UV tangency is a zero derivative of G along the source. Without
            // a certified sign or transversality, no tangency is claimed.
            let kind = match contact.tangent_cross_sign() {
                Some(sign) => classify_derivative(sign),
                None if contact.is_certified_transverse() => CurveSurfaceContactKind::Simple,
                None => classify_by_gradients(source, equation, contact.point())?,
            };
            push(parameter, kind, &mut points)?;
        }
        let mut append_range = |index, range: &hypercurve::CurveParameterRange2| {
            let first = spatial(&source.parameter_from_span(index, range.start())?)?;
            let second = spatial(&source.parameter_from_span(index, range.end())?)?;
            let (start, end) = if first.compare(&second)? == Ordering::Greater {
                (second, first)
            } else {
                (first, second)
            };
            match start.compare(&end)? {
                Ordering::Less => overlaps.push(ParameterRange::new(start, end)?),
                Ordering::Equal => push(start, CurveSurfaceContactKind::Boundary, &mut points)?,
                Ordering::Greater => {}
            }
            Ok::<(), GeometryError>(())
        };
        for overlap in intersection.overlaps() {
            if !overlap.includes_start() || !overlap.includes_end() {
                return Err(GeometryError::UnsupportedIntersection);
            }
            append_range(overlap.first_span_index(), overlap.first_range())?;
        }
        for component in intersection.parameter_components() {
            match component.first_parameters() {
                CurveParameterSet2::Range(range) => {
                    append_range(component.first_span_index(), range)?
                }
                CurveParameterSet2::Single(_) => {
                    return Err(GeometryError::UnsupportedIntersection);
                }
            }
        }
    }
    for isolated in &set.points {
        // G keeps one sign around an isolated real zero, so its derivative
        // along any curve through that point vanishes.
        match source.point_locations(isolated)? {
            CurvePointLocations2::Components(_) => {
                return Err(GeometryError::UnsupportedIntersection);
            }
            CurvePointLocations2::EntireCurve => {
                unreachable!("a noncollapsed source is not one point")
            }
            CurvePointLocations2::Locations(locations) => {
                for location in locations {
                    let parameter = spatial(&location.parameter()?)?;
                    push(parameter, CurveSurfaceContactKind::Repeated, &mut points)?;
                }
            }
        }
    }
    assemble(curve, points, overlaps)
}

/// Exact contact kind at a point `P` where Hypercurve supplies no crossing
/// certificate, such as a topology event at a branch endpoint.
///
/// Near a regular point of the source's defining equation `f = 0`, the source
/// is a smooth arc with tangent `(-f_v, f_u)`, so `dG/dt` vanishes exactly when
/// `f_u G_v - f_v G_u = 0`. At a singular point of `f` the arc's tangent is not
/// determined by `f`, and no classification is claimed.
fn classify_by_gradients(
    source: &Curve2,
    equation: &BivariatePolynomial,
    point: &hypercurve::CurvePoint2,
) -> GeometryResult<CurveSurfaceContactKind> {
    use hypersolve::CurveResultantParameter::{First, Second};
    use hypersolve::bivariate_arithmetic::{
        bivariate_multiply, bivariate_parameter_derivative, bivariate_subtract,
    };
    let source = source
        .source_restriction()
        .map_or(source, |(original, _, _)| original);
    let Some(branch) = source.polynomial_branch() else {
        return Err(GeometryError::UnsupportedIntersection);
    };
    let defining = branch.topology().geometric_equation();
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let vanishes = |polynomial: &BivariatePolynomial| -> GeometryResult<bool> {
        Ok(point
            .rational_form(polynomial.clone(), one.clone())?
            .compare_to_real(&Real::zero())?
            == Ordering::Equal)
    };
    let fu = bivariate_parameter_derivative(defining, First);
    let fv = bivariate_parameter_derivative(defining, Second);
    if vanishes(&fu)? && vanishes(&fv)? {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let jacobian = bivariate_subtract(
        &bivariate_multiply(&fu, &bivariate_parameter_derivative(equation, Second)),
        &bivariate_multiply(&fv, &bivariate_parameter_derivative(equation, First)),
    );
    Ok(if vanishes(&jacobian)? {
        CurveSurfaceContactKind::Repeated
    } else {
        CurveSurfaceContactKind::Simple
    })
}
