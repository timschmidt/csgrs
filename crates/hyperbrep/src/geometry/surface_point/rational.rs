//! Rational surface coefficients evaluated in the original joint UV chart.

use super::super::*;
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::{
    bivariate_add, bivariate_multiply, bivariate_outer_product, bivariate_scale,
};
use hypersolve::curve_substitution::{
    bernstein_scalar_power_basis, bspline_span_basis_power_basis,
};

/// Prepared coefficient maps share one checked UV source at the point owner.
/// The immutable surface remains the replay definition; these derived maps
/// carry no independently serialized proof or selected coordinate roots.
#[derive(Debug)]
pub(in crate::geometry) struct RationalSurfacePoint {
    coordinates: [BivariatePolynomial; 3],
    weight: BivariatePolynomial,
}

impl RationalSurfacePoint {
    pub(in crate::geometry) fn new(
        surface: &Surface,
        uv: &hypercurve::CurvePoint2,
    ) -> GeometryResult<Self> {
        let coefficients = match &surface.data.geometry {
            SurfaceGeometry::RationalBezier(surface) => {
                let map = |component| {
                    tensor_bernstein(
                        &surface
                            .homogeneous_controls()
                            .iter()
                            .map(|row| {
                                row.iter()
                                    .map(|p| homogeneous_component(p, component))
                                    .collect()
                            })
                            .collect::<Vec<_>>(),
                    )
                };
                [map(0), map(1), map(2), map(3)]
            }
            SurfaceGeometry::Nurbs(surface) => {
                let u_span = selected_span(
                    uv,
                    0,
                    &surface.u_knots,
                    surface.u_degree,
                    surface.control_points[0].len(),
                )?;
                let v_span = selected_span(
                    uv,
                    1,
                    &surface.v_knots,
                    surface.v_degree,
                    surface.control_points.len(),
                )?;
                let u_basis = span_basis(&surface.u_knots, surface.u_degree, u_span)?;
                let v_basis = span_basis(&surface.v_knots, surface.v_degree, v_span)?;
                let mut coefficients =
                    std::array::from_fn(|_| BivariatePolynomial::new(Vec::new()));
                for (j, v) in v_basis.iter().enumerate() {
                    for (i, u) in u_basis.iter().enumerate() {
                        let basis = bivariate_outer_product(u, v);
                        let point = &surface.homogeneous_controls()[v_span - surface.v_degree + j]
                            [u_span - surface.u_degree + i];
                        for (component, coefficient) in coefficients.iter_mut().enumerate() {
                            *coefficient = bivariate_add(
                                coefficient,
                                &bivariate_scale(
                                    basis.clone(),
                                    &homogeneous_component(point, component),
                                ),
                            );
                        }
                    }
                }
                coefficients
            }
            SurfaceGeometry::Extrusion(surface) => {
                let profile = profile_coefficients(&surface.profile, uv)?;
                let mut coefficients = profile.map(|p| bivariate_outer_product(&p, &[Real::one()]));
                for i in 0..3 {
                    let direction = &surface.direction[i];
                    let displacement = bivariate_multiply(
                        &coefficients[3],
                        &BivariatePolynomial::new(vec![vec![Real::zero(), direction.clone()]]),
                    );
                    coefficients[i] = bivariate_add(&coefficients[i], &displacement);
                }
                coefficients
            }
            _ => return Err(GeometryError::UnsupportedSelectedEvaluation),
        };
        let [x, y, z, weight] = coefficients;
        // Check the original weight at this joint UV before retaining a finite image.
        uv.rational_form(BivariatePolynomial::new(Vec::new()), weight.clone())?;
        Ok(Self {
            coordinates: [x, y, z],
            weight,
        })
    }

    pub(in crate::geometry) fn coordinate(
        &self,
        uv: &hypercurve::CurvePoint2,
        axis: usize,
    ) -> GeometryResult<hypercurve::CurvePointRationalForm2> {
        Ok(uv.rational_form(self.coordinates[axis].clone(), self.weight.clone())?)
    }

    pub(in crate::geometry) fn linear_form(
        &self,
        uv: &hypercurve::CurvePoint2,
        direction: &Vector3,
    ) -> GeometryResult<hypercurve::CurvePointRationalForm2> {
        let numerator = self.coordinates.iter().enumerate().fold(
            BivariatePolynomial::new(Vec::new()),
            |sum, (i, polynomial)| {
                bivariate_add(&sum, &bivariate_scale(polynomial.clone(), &direction[i]))
            },
        );
        Ok(uv.rational_form(numerator, self.weight.clone())?)
    }
}

fn homogeneous_component(point: &HomogeneousPoint3, component: usize) -> Real {
    match component {
        0 => point.x.clone(),
        1 => point.y.clone(),
        2 => point.z.clone(),
        _ => point.w.clone(),
    }
}

fn tensor_bernstein(grid: &[Vec<Real>]) -> BivariatePolynomial {
    let rows = grid
        .iter()
        .map(|row| bernstein_scalar_power_basis(row))
        .collect::<Vec<_>>();
    BivariatePolynomial::new(
        (0..rows[0].len())
            .map(|i| {
                bernstein_scalar_power_basis(
                    &rows.iter().map(|row| row[i].clone()).collect::<Vec<_>>(),
                )
            })
            .collect(),
    )
}

fn selected_span(
    uv: &hypercurve::CurvePoint2,
    axis: usize,
    knots: &[Real],
    degree: usize,
    count: usize,
) -> GeometryResult<usize> {
    let coordinate = uv.affine_form(
        Real::from(u8::from(axis == 0)),
        Real::from(u8::from(axis == 1)),
        Real::zero(),
    );
    for span in degree..count {
        if decided_order(compare_reals(
            &knots[span],
            &knots[span + 1],
            crate::STRICT_PREDICATES,
        ))? != Ordering::Less
        {
            continue;
        }
        if coordinate.compare_to_real(&knots[span])? != Ordering::Less {
            let upper = coordinate.compare_to_real(&knots[span + 1])?;
            if upper == Ordering::Less || (span == count - 1 && upper == Ordering::Equal) {
                return Ok(span);
            }
        }
    }
    Err(GeometryError::SurfaceParameterOutsideDomain)
}

fn span_basis(knots: &[Real], degree: usize, span: usize) -> GeometryResult<Vec<Vec<Real>>> {
    bspline_span_basis_power_basis(knots, degree, span, -512)
        .map_err(GeometryError::SplineBasisSubstitution)
}

fn profile_coefficients(
    curve: &Curve3,
    uv: &hypercurve::CurvePoint2,
) -> GeometryResult<[Vec<Real>; 4]> {
    match &curve.data.geometry {
        CurveGeometry3::Line(line) => Ok([
            vec![line.start.x.clone(), &line.end.x - &line.start.x],
            vec![line.start.y.clone(), &line.end.y - &line.start.y],
            vec![line.start.z.clone(), &line.end.z - &line.start.z],
            vec![Real::one()],
        ]),
        CurveGeometry3::RationalBezier(curve) => Ok(std::array::from_fn(|component| {
            bernstein_scalar_power_basis(
                &curve
                    .homogeneous_controls()
                    .iter()
                    .map(|point| homogeneous_component(point, component))
                    .collect::<Vec<_>>(),
            )
        })),
        CurveGeometry3::Nurbs(curve) => {
            let span = selected_span(
                uv,
                0,
                &curve.knots,
                curve.degree,
                curve.control_points.len(),
            )?;
            let basis = span_basis(&curve.knots, curve.degree, span)?;
            Ok(std::array::from_fn(|component| {
                basis
                    .iter()
                    .enumerate()
                    .fold(Vec::new(), |sum, (i, polynomial)| {
                        use hypersolve::bivariate_arithmetic::{polynomial_add, polynomial_scale};
                        polynomial_add(
                            &sum,
                            &polynomial_scale(
                                polynomial,
                                &homogeneous_component(
                                    &curve.homogeneous_controls()[span - curve.degree + i],
                                    component,
                                ),
                            ),
                        )
                    })
            }))
        }
        _ => Err(GeometryError::UnsupportedSelectedEvaluation),
    }
}
