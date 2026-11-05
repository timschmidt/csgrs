//! Complete contacts between rational spatial curves and full revolutions.
//!
//! A point lies on the full revolution of a profile about an axis exactly when
//! its squared axis distance and axial height equal those of some profile
//! point. The quotient map `p -> (rho^2, z)` sends a positive-weight rational
//! span of degree `n` to a positive-weight rational planar span of degree `2n`
//! in the same parameter: with `D = X - W o`, its homogeneous coordinates are
//! `(|D|^2 - (a·D)^2, (a·D) W, W^2)`. Hypercurve's complete planar incidence
//! between the curve image and the profile image therefore returns every
//! contact and coincident range of the spatial curve, in its own chart.

use super::curve_extrusion::classify_one_sided;
use super::curve_plane::{assemble, classify_derivative};
use super::*;
use hypercurve::{CurveIntersectionPairBlockerKind2, CurveParameterSet2, CurvePointLocations2};

/// One positive-weight rational Bézier span and its map into the curve chart.
struct Span {
    controls: Vec<HomogeneousPoint3>,
    start: Real,
    length: Real,
}

impl Span {
    fn unit(controls: Vec<HomogeneousPoint3>) -> Self {
        Self {
            controls,
            start: Real::zero(),
            length: Real::one(),
        }
    }

    fn global(&self, local: Parameter) -> GeometryResult<Parameter> {
        local.affine_image(&self.length, &self.start)
    }
}

/// Native rational spans of a line, rational Bézier or NURBS curve, or `None`
/// for families without an exact rational span decomposition here.
fn spans(curve: &Curve3) -> GeometryResult<Option<Vec<Span>>> {
    Ok(Some(match &curve.data.geometry {
        CurveGeometry3::Line(line) => vec![Span::unit(vec![
            HomogeneousPoint3::from_affine(&line.start, &Real::one()),
            HomogeneousPoint3::from_affine(&line.end, &Real::one()),
        ])],
        CurveGeometry3::RationalBezier(bezier) => {
            vec![Span::unit(bezier.homogeneous_controls().to_vec())]
        }
        CurveGeometry3::Nurbs(nurbs) => {
            let mut spans = Vec::new();
            for (segment, domain) in decompose_nurbs_into_bezier_segments(nurbs)? {
                let CurveGeometry3::RationalBezier(bezier) = &segment.data.geometry else {
                    unreachable!("NURBS decomposition produces rational Bézier segments");
                };
                spans.push(Span {
                    controls: bezier.homogeneous_controls().to_vec(),
                    start: domain.start().clone(),
                    length: domain.end() - domain.start(),
                });
            }
            spans
        }
        CurveGeometry3::AffineImage(_)
        | CurveGeometry3::Rotation(_)
        | CurveGeometry3::Translation(_)
        | CurveGeometry3::TensorImage(_)
        | CurveGeometry3::PlanarImage(_)
        | CurveGeometry3::CircleArc(_)
        | CurveGeometry3::EllipseArc(_) => return Ok(None),
    }))
}

/// Exact quotient image of one span.
enum Image {
    /// The span lies on one latitude circle (or one axis point).
    Point(CurvePoint2),
    Curve(Curve2),
}

fn image(controls: &[HomogeneousPoint3], surface: &RevolutionSurface) -> GeometryResult<Image> {
    let origin = &surface.axis_origin;
    let axis = &surface.axis;
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
    let axial = (0..controls.len())
        .map(|i| {
            &axis[0] * &displacement[0][i]
                + &axis[1] * &displacement[1][i]
                + &axis[2] * &displacement[2][i]
        })
        .collect::<Vec<_>>();
    let mut radial_squared = bernstein_product(&displacement[0], &displacement[0])?;
    for coordinate in &displacement[1..] {
        for (total, term) in radial_squared
            .iter_mut()
            .zip(bernstein_product(coordinate, coordinate)?)
        {
            *total += term;
        }
    }
    // The axis is a unit vector, so |D|^2 - (a·D)^2 is the squared distance.
    for (total, term) in radial_squared
        .iter_mut()
        .zip(bernstein_product(&axial, &axial)?)
    {
        *total -= term;
    }
    let height = bernstein_product(&axial, &weight)?;
    let weight = bernstein_product(&weight, &weight)?;
    // Positive native weights give positive Bernstein coefficients of W^2.
    let mut points = Vec::with_capacity(weight.len());
    for ((x, y), w) in radial_squared.iter().zip(&height).zip(&weight) {
        points.push(CurvePoint2::new(
            (x / w).map_err(GeometryError::Numeric)?,
            (y / w).map_err(GeometryError::Numeric)?,
        ));
    }
    let first = points[0].clone();
    let mut collapsed = true;
    for point in &points[1..] {
        if decided_order(compare_reals(
            point.x(),
            first.x(),
            crate::STRICT_PREDICATES,
        ))? != Ordering::Equal
            || decided_order(compare_reals(
                point.y(),
                first.y(),
                crate::STRICT_PREDICATES,
            ))? != Ordering::Equal
        {
            collapsed = false;
            break;
        }
    }
    Ok(if collapsed {
        Image::Point(first)
    } else {
        Image::Curve(Curve2::from(RationalBezier2::try_new(points, weight)?))
    })
}

/// Complete exact contacts of a line, rational Bézier or NURBS curve with the
/// full revolution of a line, rational Bézier or NURBS profile. Returns `None`
/// for other curve or profile families.
pub(super) fn intersect_rational_curve_revolution(
    curve: &Curve3,
    surface: &RevolutionSurface,
) -> Option<GeometryResult<CurveSurfaceIntersection>> {
    let (curve_spans, profile_spans) = match (spans(curve), spans(&surface.profile)) {
        (Ok(Some(first)), Ok(Some(second))) => (first, second),
        (Ok(None), _) | (_, Ok(None)) => return None,
        (Err(error), _) | (_, Err(error)) => return Some(Err(error)),
    };
    Some(intersect_spans(
        curve,
        surface,
        &curve_spans,
        &profile_spans,
    ))
}

fn intersect_spans(
    curve: &Curve3,
    surface: &RevolutionSurface,
    curve_spans: &[Span],
    profile_spans: &[Span],
) -> GeometryResult<CurveSurfaceIntersection> {
    let mut profiles = Vec::with_capacity(profile_spans.len());
    for span in profile_spans {
        match image(&span.controls, surface)? {
            Image::Curve(image) => profiles.push(image),
            // A latitude-circle profile span sweeps no surface area. The
            // degenerate carrier is not admitted by this route.
            Image::Point(_) => return Err(GeometryError::UnsupportedIntersection),
        }
    }
    let domain = curve.domain();
    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    for span in curve_spans {
        let local_unit = || -> GeometryResult<ParameterRange> {
            ParameterRange::new(
                span.global(Parameter::from_real(Real::zero()))?,
                span.global(Parameter::from_real(Real::one()))?,
            )
        };
        let image = match image(&span.controls, surface)? {
            Image::Point(point) => {
                // Every point of the span lies on one latitude circle.
                let mut on_surface = false;
                for profile in &profiles {
                    match profile.point_locations(&point.clone().into())? {
                        CurvePointLocations2::Locations(locations) if locations.is_empty() => {}
                        CurvePointLocations2::Locations(_)
                        | CurvePointLocations2::Components(_) => on_surface = true,
                        CurvePointLocations2::EntireCurve => {
                            unreachable!("a noncollapsed profile image is not one point")
                        }
                    }
                }
                if on_surface {
                    overlaps.push(local_unit()?);
                }
                continue;
            }
            Image::Curve(image) => image,
        };
        for profile in &profiles {
            let intersection = image.intersect_curve(profile)?;
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
                let local = contact.first().parameter()?;
                let parameter = span.global(Parameter::from_curve(&local)?)?;
                if !domain.contains(&parameter)? {
                    continue;
                }
                // Both images are single rational spans; joints of authored
                // splines appear in both adjacent spans and are merged below,
                // keeping a repeated classification from either side.
                let kind = match contact.tangent_cross_sign() {
                    Some(sign) => classify_derivative(sign),
                    None if contact.is_certified_transverse() => CurveSurfaceContactKind::Simple,
                    None => {
                        classify_one_sided(&image, &local, profile, &contact.second().parameter()?)?
                    }
                };
                points.push(CurveSurfacePoint {
                    point: curve.point_at(&parameter)?,
                    parameter,
                    kind,
                });
            }
            // Hypercurve may promote a span internally; its ranges are local
            // to the promoted span and are mapped back through the image.
            let mut append_range =
                |index, range: &hypercurve::CurveParameterRange2| -> GeometryResult<()> {
                    let first = span.global(Parameter::from_curve(
                        &image.parameter_from_span(index, range.start())?,
                    )?)?;
                    let second = span.global(Parameter::from_curve(
                        &image.parameter_from_span(index, range.end())?,
                    )?)?;
                    let (start, end) = if first.compare(&second)? == Ordering::Greater {
                        (second, first)
                    } else {
                        (first, second)
                    };
                    match start.compare(&end)? {
                        Ordering::Less => overlaps.push(ParameterRange::new(start, end)?),
                        Ordering::Equal => points.push(CurveSurfacePoint {
                            point: curve.point_at(&start)?,
                            parameter: start,
                            kind: CurveSurfaceContactKind::Boundary,
                        }),
                        Ordering::Greater => {}
                    }
                    Ok(())
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
    }
    assemble(curve, points, overlaps)
}
