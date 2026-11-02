//! Complete contacts between rational spatial curves and linear extrusions.
//!
//! An extrusion `S(u, v) = profile(u) + v d` is unbounded in `v`, so a point
//! lies on it exactly when its image under any linear map with kernel `span(d)`
//! lies on the projected profile. Projecting a line, rational Bézier or NURBS
//! curve by such a map preserves its family, weights, knots and parameters.
//! Hypercurve's complete planar curve/curve incidence then returns every
//! contact and coincident range in the spatial curve's own chart.

use super::curve_plane::{assemble, classify_derivative};
use super::*;
use hypercurve::{
    CurveIntersectionPairBlockerKind2, CurveParameterSet2, CurveParameterSide2,
    CurvePointLocations2, CurveVector2,
};
use hyperreal::RealSign;

/// Exact linear map `p -> (p·e1, p·e2)` whose kernel is the extrusion direction.
struct Projection {
    first: Vector3,
    second: Vector3,
}

impl Projection {
    fn along(direction: &Vector3) -> GeometryResult<Self> {
        // e1 = d x a is nonzero for a coordinate axis a not parallel to d;
        // e2 = d x e1 is orthogonal to both. Neither needs normalization.
        let mut first = direction.cross(&Vector3::x());
        if is_zero_vector(&first)? {
            first = direction.cross(&Vector3::y());
        }
        let second = direction.cross(&first);
        Ok(Self { first, second })
    }

    fn point(&self, point: &Point3) -> CurvePoint2 {
        let vector = Vector3::from(point.clone());
        CurvePoint2::new(self.first.dot(&vector), self.second.dot(&vector))
    }

    /// Projects a native rational spatial family without changing its chart.
    /// `Ok(None)` marks a family without a supported exact planar image.
    fn curve(&self, curve: &Curve3) -> GeometryResult<Option<ProjectedCurve>> {
        let (points, weights, knots) = match &curve.data.geometry {
            CurveGeometry3::Line(line) => (
                vec![self.point(&line.start), self.point(&line.end)],
                None,
                None,
            ),
            CurveGeometry3::RationalBezier(bezier) => (
                bezier
                    .control_points
                    .iter()
                    .map(|p| self.point(p))
                    .collect(),
                Some(bezier.weights.clone()),
                None,
            ),
            CurveGeometry3::Nurbs(nurbs) => (
                nurbs.control_points.iter().map(|p| self.point(p)).collect(),
                Some(nurbs.weights.clone()),
                Some((nurbs.degree, nurbs.knots.clone())),
            ),
            CurveGeometry3::AffineImage(_)
            | CurveGeometry3::Rotation(_)
            | CurveGeometry3::Translation(_)
            | CurveGeometry3::PlanarImage(_)
            | CurveGeometry3::CircleArc(_)
            | CurveGeometry3::EllipseArc(_) => return Ok(None),
        };
        let first = points[0].clone();
        let mut collapsed = true;
        for point in &points[1..] {
            if !same_point(point, &first)? {
                collapsed = false;
                break;
            }
        }
        if collapsed {
            return Ok(Some(ProjectedCurve::Point(first)));
        }
        let projected = match (weights, knots) {
            (None, _) => Curve2::from(LineSeg2::try_new(points[0].clone(), points[1].clone())?),
            (Some(weights), None) => Curve2::from(RationalBezier2::try_new(points, weights)?),
            (Some(weights), Some((degree, knots))) => {
                Curve2::try_nurbs(degree, points, weights, knots)?
            }
        };
        Ok(Some(ProjectedCurve::Curve(projected)))
    }
}

enum ProjectedCurve {
    /// The whole spatial curve lies on one line parallel to the direction.
    Point(CurvePoint2),
    Curve(Curve2),
}

fn is_zero_vector(vector: &Vector3) -> GeometryResult<bool> {
    Ok(decided_order(compare_reals(
        &vector.norm_squared(),
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? == Ordering::Equal)
}

fn same_point(first: &CurvePoint2, second: &CurvePoint2) -> GeometryResult<bool> {
    let x = decided_order(compare_reals(
        first.x(),
        second.x(),
        crate::STRICT_PREDICATES,
    ))?;
    let y = decided_order(compare_reals(
        first.y(),
        second.y(),
        crate::STRICT_PREDICATES,
    ))?;
    Ok(x == Ordering::Equal && y == Ordering::Equal)
}

/// Complete exact contacts of a line, rational Bézier or NURBS curve with a
/// linear extrusion of a line, rational Bézier or NURBS profile. Returns
/// `None` for other curve or profile families.
pub(super) fn intersect_rational_curve_extrusion(
    curve: &Curve3,
    surface: &ExtrusionSurface,
) -> Option<GeometryResult<CurveSurfaceIntersection>> {
    let projection = match Projection::along(&surface.direction) {
        Ok(projection) => projection,
        Err(error) => return Some(Err(error)),
    };
    let (projected, profile) = match (projection.curve(curve), projection.curve(&surface.profile)) {
        (Ok(Some(projected)), Ok(Some(profile))) => (projected, profile),
        (Ok(None), _) | (_, Ok(None)) => return None,
        (Err(error), _) | (_, Err(error)) => return Some(Err(error)),
    };
    Some(intersect_projected(curve, projected, profile))
}

fn intersect_projected(
    curve: &Curve3,
    projected: ProjectedCurve,
    profile: ProjectedCurve,
) -> GeometryResult<CurveSurfaceIntersection> {
    let ProjectedCurve::Curve(profile) = profile else {
        // A profile parallel to the direction sweeps no surface area; the
        // degenerate carrier is not admitted by this route.
        return Err(GeometryError::UnsupportedIntersection);
    };
    let projected = match projected {
        ProjectedCurve::Point(point) => {
            // Every point of the curve has the same projection.
            return Ok(match profile.point_locations(&point.into())? {
                CurvePointLocations2::EntireCurve => {
                    unreachable!("a noncollapsed projected profile is not one point")
                }
                CurvePointLocations2::Locations(locations) if locations.is_empty() => {
                    CurveSurfaceIntersection::None
                }
                CurvePointLocations2::Locations(_) => CurveSurfaceIntersection::Contained,
            });
        }
        ProjectedCurve::Curve(projected) => projected,
    };
    let intersection = projected.intersect_curve(&profile)?;
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
    let domain = curve.domain();
    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    for contact in intersection.contacts() {
        let source = contact.first().parameter()?;
        let parameter = Parameter::from_curve(&source)?;
        if !domain.contains(&parameter)? {
            continue;
        }
        // A deduplicated spline contact may name only one incident span, so
        // spline joints always inspect both sides of each authored tangent.
        let single_span = !is_spline(&projected) && !is_spline(&profile);
        let kind = match contact.tangent_cross_sign() {
            Some(sign) if single_span => classify_derivative(sign),
            None if single_span && contact.is_certified_transverse() => {
                CurveSurfaceContactKind::Simple
            }
            _ => classify_one_sided(
                &projected,
                &source,
                &profile,
                &contact.second().parameter()?,
            )?,
        };
        points.push(CurveSurfacePoint {
            point: curve.point_at(&parameter)?,
            parameter,
            kind,
        });
    }
    let mut append_range = |index,
                            range: &hypercurve::CurveParameterRange2|
     -> GeometryResult<()> {
        let first = Parameter::from_curve(&projected.parameter_from_span(index, range.start())?)?;
        let second = Parameter::from_curve(&projected.parameter_from_span(index, range.end())?)?;
        let (mut start, mut end) = if first.compare(&second)? == Ordering::Greater {
            (second, first)
        } else {
            (first, second)
        };
        if start.compare(domain.start())? == Ordering::Less {
            start = domain.start().clone();
        }
        if end.compare(domain.end())? == Ordering::Greater {
            end = domain.end().clone();
        }
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
            CurveParameterSet2::Range(range) => append_range(component.first_span_index(), range)?,
            // A curve range collapsing onto one profile parameter is still a
            // range on the spatial curve; a single curve parameter whose
            // projection traces a profile range cannot occur for a curve.
            CurveParameterSet2::Single(_) => return Err(GeometryError::UnsupportedIntersection),
        }
    }
    assemble(curve, points, overlaps)
}

fn is_spline(curve: &Curve2) -> bool {
    matches!(
        curve.family(),
        CurveFamily2::Nurbs | CurveFamily2::PolynomialBSpline
    )
}

/// Classifies a contact from one-sided tangents of both projected curves.
///
/// The extrusion's normal is the lifted normal of the projected profile, so
/// the contact equation's first derivative vanishes exactly when the projected
/// tangents are parallel (including a zero projected curve tangent, where the
/// spatial tangent is the extrusion direction). Both sides of authored spline
/// joints are inspected; an undecidable cross sign is not a tangency proof.
fn classify_one_sided(
    projected: &Curve2,
    curve_parameter: &hypercurve::CurveParameter2,
    profile: &Curve2,
    profile_parameter: &hypercurve::CurveParameter2,
) -> GeometryResult<CurveSurfaceContactKind> {
    let mut kind = CurveSurfaceContactKind::Simple;
    for curve_side in [CurveParameterSide2::Left, CurveParameterSide2::Right] {
        let tangent = projected.derivatives_at_side(curve_parameter, 1, curve_side)?;
        for profile_side in [CurveParameterSide2::Left, CurveParameterSide2::Right] {
            let direction = profile.derivatives_at_side(profile_parameter, 1, profile_side)?;
            if cross_sign(&tangent[0], &direction[0])? == RealSign::Zero {
                kind = CurveSurfaceContactKind::Repeated;
            }
        }
    }
    Ok(kind)
}

/// Exact sign of `a.x b.y - a.y b.x` when either vector has represented
/// coordinates; jointly selected pairs are left as a typed limitation.
fn cross_sign(first: &CurveVector2, second: &CurveVector2) -> GeometryResult<RealSign> {
    let sign = |order: Ordering| match order {
        Ordering::Less => RealSign::Negative,
        Ordering::Equal => RealSign::Zero,
        Ordering::Greater => RealSign::Positive,
    };
    if let Some((x, y)) = second.represented_coordinates() {
        // a.x * y + a.y * (-x)
        return Ok(sign(first.compare_linear_form(y, &-x, &Real::zero())?));
    }
    if let Some((x, y)) = first.represented_coordinates() {
        // -(b.x * y + b.y * (-x))
        return Ok(sign(
            second.compare_linear_form(y, &-x, &Real::zero())?.reverse(),
        ));
    }
    Err(GeometryError::UnsupportedIntersection)
}
