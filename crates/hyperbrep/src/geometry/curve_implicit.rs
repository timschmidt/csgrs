//! Complete contacts between rational spatial curves and implicit analytic carriers.
//!
//! Spheres, circular cylinders, right circular cones and ring tori are zero
//! sets of exact polynomial equations. Composing the homogenized equation with
//! a positive-weight rational Bézier span gives one univariate numerator whose
//! Bernstein coefficients are exact. Its complete root set on the span, found by
//! Hypercurve's existing root authority, is the complete contact set: positive
//! weights keep the denominator nonzero, so no root is introduced or lost.

use super::curve_plane::{assemble, classify_derivative, is_zero};
use super::*;
use hypercurve::{Axis2, CurvePointLocations2};

/// Native analytic carriers with a complete exact implicit equation.
#[derive(Clone, Copy)]
pub(super) enum ImplicitCarrier<'a> {
    Sphere(&'a SphereSurface),
    Cylinder(&'a CylinderSurface),
    /// The equation describes both nappes; only `axis·(p-apex) >= 0` belongs
    /// to the native single-nappe cone.
    Cone(&'a ConeSurface),
    Torus(&'a TorusSurface),
}

impl<'a> ImplicitCarrier<'a> {
    pub(super) fn from_surface(surface: &'a SurfaceGeometry) -> Option<Self> {
        match surface {
            SurfaceGeometry::Sphere(surface) => Some(Self::Sphere(surface)),
            SurfaceGeometry::Cylinder(surface) => Some(Self::Cylinder(surface)),
            SurfaceGeometry::Cone(surface) => Some(Self::Cone(surface)),
            SurfaceGeometry::Torus(surface) => Some(Self::Torus(surface)),
            SurfaceGeometry::Plane(_)
            | SurfaceGeometry::Extrusion(_)
            | SurfaceGeometry::Revolution(_)
            | SurfaceGeometry::RationalBezier(_)
            | SurfaceGeometry::Nurbs(_) => None,
        }
    }

    fn origin(self) -> &'a Point3 {
        match self {
            Self::Sphere(surface) => &surface.center,
            Self::Cylinder(surface) => &surface.origin,
            Self::Cone(surface) => &surface.apex,
            Self::Torus(surface) => &surface.center,
        }
    }

    fn axis(self) -> &'a Vector3 {
        match self {
            Self::Sphere(surface) => &surface.frame.z,
            Self::Cylinder(surface) => &surface.frame.z,
            Self::Cone(surface) => &surface.frame.z,
            Self::Torus(surface) => &surface.frame.z,
        }
    }

    /// Bernstein coefficients of `W^k F(X/W)` for the homogeneous span.
    ///
    /// With displacement `D = X - W o`, `|D|^2`, axial `a = axis·D` and
    /// `W^2` all have degree `2n`; each equation below is homogeneous, so no
    /// degree elevation is needed and every coefficient is an exact product sum.
    fn numerator(
        self,
        displacement: &[Vec<Real>; 3],
        weight: &[Real],
    ) -> GeometryResult<Vec<Real>> {
        let squared_norm = sum(&[
            bernstein_product(&displacement[0], &displacement[0])?,
            bernstein_product(&displacement[1], &displacement[1])?,
            bernstein_product(&displacement[2], &displacement[2])?,
        ]);
        let weight_squared = bernstein_product(weight, weight)?;
        let axial = axial_coefficients(self.axis(), displacement);
        let radial_squared = || -> GeometryResult<Vec<Real>> {
            Ok(difference(
                &squared_norm,
                &bernstein_product(&axial, &axial)?,
            ))
        };
        Ok(match self {
            Self::Sphere(surface) => difference(
                &squared_norm,
                &scaled(&weight_squared, &(&surface.radius * &surface.radius)),
            ),
            Self::Cylinder(surface) => difference(
                &radial_squared()?,
                &scaled(&weight_squared, &(&surface.radius * &surface.radius)),
            ),
            Self::Cone(surface) => {
                // cos^2 |radial|^2 - sin^2 axial^2, matching the native line route.
                let sin = surface.semi_angle.clone().sin();
                let cos = surface.semi_angle.clone().cos();
                difference(
                    &scaled(&radial_squared()?, &(&cos * &cos)),
                    &scaled(&bernstein_product(&axial, &axial)?, &(&sin * &sin)),
                )
            }
            Self::Torus(surface) => {
                // (|D|^2 + (R^2-r^2)W^2)^2 - 4R^2 W^2 |radial|^2.
                let major_squared = &surface.major_radius * &surface.major_radius;
                let minor_squared = &surface.minor_radius * &surface.minor_radius;
                let shifted = sum(&[
                    squared_norm.clone(),
                    scaled(&weight_squared, &(&major_squared - minor_squared)),
                ]);
                difference(
                    &bernstein_product(&shifted, &shifted)?,
                    &scaled(
                        &bernstein_product(&weight_squared, &radial_squared()?)?,
                        &(Real::from(4) * major_squared),
                    ),
                )
            }
        })
    }
}

/// Intersects one positive-weight rational Bézier span on `[0, 1]`.
///
/// `parameter` maps each local root to the consumed curve's parameter.
/// Returned overlaps are local closed ranges in the same mapped chart.
fn intersect_span(
    curve: &Curve3,
    controls: &[HomogeneousPoint3],
    carrier: ImplicitCarrier<'_>,
    parameter: &dyn Fn(Parameter) -> GeometryResult<Parameter>,
    points: &mut Vec<CurveSurfacePoint>,
) -> GeometryResult<SpanContact> {
    let origin = carrier.origin();
    let weight = controls
        .iter()
        .map(|point| point.w.clone())
        .collect::<Vec<_>>();
    let displacement = [
        controls
            .iter()
            .map(|point| &point.x - &origin.x * &point.w)
            .collect::<Vec<_>>(),
        controls
            .iter()
            .map(|point| &point.y - &origin.y * &point.w)
            .collect::<Vec<_>>(),
        controls
            .iter()
            .map(|point| &point.z - &origin.z * &point.w)
            .collect::<Vec<_>>(),
    ];
    let numerator = carrier.numerator(&displacement, &weight)?;
    let mut identically_zero = true;
    for coefficient in &numerator {
        if !is_zero(coefficient)? {
            identically_zero = false;
            break;
        }
    }
    if identically_zero {
        return contained_span(curve, carrier, &displacement, parameter, points);
    }
    for (local, derivative_sign) in roots(numerator)? {
        let parameter = parameter(local)?;
        if !curve.domain().contains(&parameter)? {
            continue;
        }
        let point = curve.point_at(&parameter)?;
        if let ImplicitCarrier::Cone(cone) = carrier
            && point.compare_linear_form(
                &cone.frame.z,
                &cone.frame.z.dot(&Vector3::from(cone.apex.clone())),
            )? == Ordering::Less
        {
            // A root of the double-cone equation on the opposite nappe.
            continue;
        }
        points.push(CurveSurfacePoint {
            point,
            parameter,
            // At a root F=0, so d(W^k F)/dt = W^k dF/dt with W > 0.
            kind: classify_derivative(derivative_sign),
        });
    }
    Ok(SpanContact::Isolated)
}

enum SpanContact {
    Isolated,
    Contained,
}

/// The span lies on the complete implicit zero set.
fn contained_span(
    curve: &Curve3,
    carrier: ImplicitCarrier<'_>,
    displacement: &[Vec<Real>; 3],
    parameter: &dyn Fn(Parameter) -> GeometryResult<Parameter>,
    points: &mut Vec<CurveSurfacePoint>,
) -> GeometryResult<SpanContact> {
    let ImplicitCarrier::Cone(cone) = carrier else {
        return Ok(SpanContact::Contained);
    };
    // With W > 0, the axial sign is the sign of its Bernstein numerator.
    // Nonnegative coefficients certify the whole span on the native nappe.
    let axial = axial_coefficients(&cone.frame.z, displacement);
    let mut signs = [false; 3];
    for coefficient in &axial {
        signs[match decided_order(compare_reals(
            coefficient,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? {
            Ordering::Less => 0,
            Ordering::Equal => 1,
            Ordering::Greater => 2,
        }] = true;
    }
    if !signs[0] {
        return Ok(SpanContact::Contained);
    }
    if signs[2] {
        // The span crosses between nappes. Selecting the retained subranges
        // needs a sign decision between selected roots; it is not guessed.
        return Err(GeometryError::UnsupportedIntersection);
    }
    // Nonpositive axial values: the span meets the native nappe only at the
    // apex, where the axial numerator vanishes. These are clipping contacts of
    // a coincident support, not simple or repeated equation roots.
    for (local, _) in roots(axial)? {
        let parameter = parameter(local)?;
        if curve.domain().contains(&parameter)? {
            points.push(CurveSurfacePoint {
                point: curve.point_at(&parameter)?,
                parameter,
                kind: CurveSurfaceContactKind::Boundary,
            });
        }
    }
    Ok(SpanContact::Isolated)
}

/// Complete roots on `[0, 1]` of a nonzero Bernstein polynomial and the sign
/// of its first derivative at each root.
fn roots(coefficients: Vec<Real>) -> GeometryResult<Vec<(Parameter, hyperreal::RealSign)>> {
    let count = coefficients.len();
    // Hypercurve owns complete root isolation for arbitrary-degree Bézier
    // coordinates. The second coordinate is identically zero, so locating the
    // origin locates exactly the roots of the first coordinate.
    let projected = Curve2::from(RationalBezier2::try_new(
        coefficients
            .into_iter()
            .map(|value| CurvePoint2::new(value, Real::zero()))
            .collect(),
        vec![Real::one(); count],
    )?);
    let mut roots = Vec::new();
    match projected.point_locations(&CurvePoint2::new(Real::zero(), Real::zero()).into())? {
        CurvePointLocations2::EntireCurve => {
            unreachable!("a nonzero Bernstein coefficient excludes an identically zero polynomial")
        }
        CurvePointLocations2::Locations(locations) => {
            for location in locations {
                let selected = location.parameter()?;
                let derivative = projected.derivatives_at(&selected, 1)?;
                roots.push((
                    Parameter::from_curve(&selected)?,
                    derivative[0].coordinate_sign(Axis2::X)?,
                ));
            }
        }
    }
    Ok(roots)
}

fn axial_coefficients(axis: &Vector3, displacement: &[Vec<Real>; 3]) -> Vec<Real> {
    (0..displacement[0].len())
        .map(|i| {
            &axis[0] * &displacement[0][i]
                + &axis[1] * &displacement[1][i]
                + &axis[2] * &displacement[2][i]
        })
        .collect()
}

fn sum(terms: &[Vec<Real>]) -> Vec<Real> {
    (0..terms[0].len())
        .map(|i| {
            terms
                .iter()
                .fold(Real::zero(), |total, term| total + &term[i])
        })
        .collect()
}

fn difference(first: &[Real], second: &[Real]) -> Vec<Real> {
    debug_assert_eq!(first.len(), second.len());
    first.iter().zip(second).map(|(a, b)| a - b).collect()
}

fn scaled(values: &[Real], factor: &Real) -> Vec<Real> {
    values.iter().map(|value| value * factor).collect()
}

/// Complete exact contacts of a line, rational Bézier or NURBS curve with an
/// implicit analytic carrier. Returns `None` for other curve families.
pub(super) fn intersect_rational_curve_implicit(
    curve: &Curve3,
    carrier: ImplicitCarrier<'_>,
) -> Option<GeometryResult<CurveSurfaceIntersection>> {
    match &curve.data.geometry {
        CurveGeometry3::Line(line) => Some(intersect_single_span(
            curve,
            &[
                HomogeneousPoint3::from_affine(&line.start, &Real::one()),
                HomogeneousPoint3::from_affine(&line.end, &Real::one()),
            ],
            carrier,
        )),
        CurveGeometry3::RationalBezier(bezier) => Some(intersect_single_span(
            curve,
            bezier.homogeneous_controls(),
            carrier,
        )),
        CurveGeometry3::Nurbs(nurbs) => Some(intersect_nurbs(curve, nurbs, carrier)),
        CurveGeometry3::AffineImage(_)
        | CurveGeometry3::Rotation(_)
        | CurveGeometry3::Translation(_)
        | CurveGeometry3::PlanarImage(_)
        | CurveGeometry3::CircleArc(_)
        | CurveGeometry3::EllipseArc(_) => None,
    }
}

fn intersect_single_span(
    curve: &Curve3,
    controls: &[HomogeneousPoint3],
    carrier: ImplicitCarrier<'_>,
) -> GeometryResult<CurveSurfaceIntersection> {
    // Lines and native rational Bézier curves use the unit source chart.
    let mut points = Vec::new();
    let overlaps = match intersect_span(curve, controls, carrier, &Ok, &mut points)? {
        SpanContact::Isolated => Vec::new(),
        SpanContact::Contained => vec![ParameterRange::unit()],
    };
    assemble(curve, points, overlaps)
}

fn intersect_nurbs(
    curve: &Curve3,
    nurbs: &NurbsCurve3,
    carrier: ImplicitCarrier<'_>,
) -> GeometryResult<CurveSurfaceIntersection> {
    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    for (segment, domain) in decompose_nurbs_into_bezier_segments(nurbs)? {
        let CurveGeometry3::RationalBezier(bezier) = &segment.data.geometry else {
            unreachable!("NURBS decomposition produces rational Bézier segments");
        };
        let span = domain.end() - domain.start();
        let transport = |local: Parameter| local.affine_image(&span, domain.start());
        // Points are evaluated on the original NURBS so selected contacts keep
        // the authored source rather than a decomposition segment.
        match intersect_span(
            curve,
            bezier.homogeneous_controls(),
            carrier,
            &transport,
            &mut points,
        )? {
            SpanContact::Isolated => {}
            SpanContact::Contained => overlaps.push(domain.into()),
        }
    }
    assemble(curve, points, overlaps)
}
