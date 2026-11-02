//! Complete incidence between finite lines and lines, rational Bézier curves
//! or NURBS curves in space.
//!
//! For a line `A + t d` and a rational curve `C(s)`, the affine map
//! `p -> (n1·(p - A), n2·(p - A))` with `n1, n2` spanning the normal plane of
//! `d` sends `C` to a planar curve of the same family, weights, knots and
//! parameter; the curve meets the line exactly where that image meets the
//! origin. Hypercurve's complete point location supplies every such `s`, and
//! the line parameter `t = d·(C(s) - A)/|d|^2` is read from a second affine
//! image at the same selected `s`, so no coordinate root is formed separately.

use super::*;
use hypercurve::CurvePointLocations2;

/// One isolated exact incidence between two spatial curves.
#[derive(Clone, Debug)]
pub struct CurveCurvePoint {
    /// Parameter on the first curve.
    pub first: Parameter,
    /// Parameter on the second curve.
    pub second: Parameter,
    /// Exact common point, retaining selected source evidence when needed.
    pub point: GeometricPoint3,
}

/// Complete exact incidence between two finite spatial curves.
#[derive(Clone, Debug)]
pub enum CurveCurveIntersection {
    /// The curves do not meet.
    None,
    /// The curves meet at isolated exact points, ordered by first parameter.
    Points(Vec<CurveCurvePoint>),
    /// The curves share one coincident range, given in both charts.
    Overlap {
        /// Range on the first curve.
        first: ParameterRange,
        /// Corresponding range on the second curve.
        second: ParameterRange,
    },
    /// Several coincident ranges, or isolated points together with ranges.
    /// Points inside a range are omitted from the point list.
    Components {
        /// Isolated incidences, ordered by first parameter.
        points: Vec<CurveCurvePoint>,
        /// Coincident ranges, each given in both charts.
        overlaps: Vec<(ParameterRange, ParameterRange)>,
    },
}

impl Curve3 {
    /// Intersects this finite curve with another finite curve.
    ///
    /// Line pairs, and lines against positive-weight rational Bézier or NURBS
    /// curves (in either order), return complete exact results with selected
    /// parameters on both curves. Coplanar pairs of lines, rational Bézier or
    /// NURBS curves and planar images are intersected completely in their
    /// common plane, and a line crossing a planar image's plane is located on
    /// it. Lines against circular or elliptic arcs solve one exact quadratic
    /// (coplanar) or locate the single plane crossing, with arc parameters from
    /// the arc's exact point location. Circular arcs meet on their radical line
    /// (coplanar) or on the line of their planes (otherwise), with both arc
    /// parameters exact; arcs of one circle return their exact coincident
    /// ranges and touching points in both charts. Noncoplanar rational
    /// Bézier/NURBS pairs
    /// are intersected through an
    /// exact planar projection whose candidates are confirmed by exact spatial
    /// coincidence. A rational curve lying along a line, spatial overlaps of
    /// noncoplanar curves and other curve families fail with
    /// [`GeometryError::UnsupportedIntersection`].
    pub fn intersect_curve(&self, other: &Self) -> GeometryResult<CurveCurveIntersection> {
        crate::context::strict(|| {
            match (self.line_endpoints(), other.line_endpoints()) {
                (Some(first), Some(second)) => return intersect_lines(self, first, other, second),
                (Some(line), None) if rational_net(other).is_some() => {
                    return intersect_line_curve(self, line, other);
                }
                (None, Some(line)) if rational_net(self).is_some() => {
                    return Ok(swap(intersect_line_curve(other, line, self)?));
                }
                (Some(line), None) => {
                    if let CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) =
                        &other.data.geometry
                    {
                        return intersect_line_conic(self, line, other, arc);
                    }
                }
                (None, Some(line)) => {
                    if let CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) =
                        &self.data.geometry
                    {
                        return Ok(swap(intersect_line_conic(other, line, self, arc)?));
                    }
                }
                _ => {}
            }
            if let (CurveGeometry3::CircleArc(first), CurveGeometry3::CircleArc(second)) =
                (&self.data.geometry, &other.data.geometry)
            {
                return intersect_circles(self, first, other, second);
            }
            if let Some(result) = intersect_coplanar(self, other)? {
                return Ok(result);
            }
            match (&self.data.geometry, &other.data.geometry) {
                (CurveGeometry3::Line(_), CurveGeometry3::PlanarImage(image)) => {
                    intersect_line_planar_image(self, other, image)
                }
                (CurveGeometry3::PlanarImage(image), CurveGeometry3::Line(_)) => {
                    Ok(swap(intersect_line_planar_image(other, self, image)?))
                }
                (CurveGeometry3::PlanarImage(image), _) if rational_net(other).is_some() => {
                    intersect_planar_image_rational(self, image, other)
                }
                (_, CurveGeometry3::PlanarImage(image)) if rational_net(self).is_some() => {
                    Ok(swap(intersect_planar_image_rational(other, image, self)?))
                }
                _ => match (rational_net(self), rational_net(other)) {
                    (Some(first), Some(second)) => {
                        intersect_rational_pair(self, &first, other, &second)
                    }
                    _ => Err(GeometryError::UnsupportedIntersection),
                },
            }
        })
    }
}

fn swap(result: CurveCurveIntersection) -> CurveCurveIntersection {
    match result {
        CurveCurveIntersection::None => CurveCurveIntersection::None,
        CurveCurveIntersection::Points(points) => {
            let mut points = points
                .into_iter()
                .map(|point| CurveCurvePoint {
                    first: point.second,
                    second: point.first,
                    point: point.point,
                })
                .collect::<Vec<_>>();
            sort_by_first(&mut points);
            CurveCurveIntersection::Points(points)
        }
        CurveCurveIntersection::Overlap { first, second } => CurveCurveIntersection::Overlap {
            first: second,
            second: first,
        },
        CurveCurveIntersection::Components { points, overlaps } => {
            let mut points = points
                .into_iter()
                .map(|point| CurveCurvePoint {
                    first: point.second,
                    second: point.first,
                    point: point.point,
                })
                .collect::<Vec<_>>();
            sort_by_first(&mut points);
            CurveCurveIntersection::Components {
                points,
                overlaps: overlaps.into_iter().map(|(a, b)| (b, a)).collect(),
            }
        }
    }
}

fn sort_by_first(points: &mut [CurveCurvePoint]) {
    // Insertion sort with exact comparisons; parameters are decidable.
    for index in 1..points.len() {
        let mut position = index;
        while position > 0
            && points[position - 1]
                .first
                .compare(&points[position].first)
                .is_ok_and(|order| order == Ordering::Greater)
        {
            points.swap(position - 1, position);
            position -= 1;
        }
    }
}

fn order(first: &Real, second: &Real) -> GeometryResult<Ordering> {
    decided_order(compare_reals(first, second, crate::STRICT_PREDICATES))
}

fn in_unit(value: &Real) -> GeometryResult<bool> {
    Ok(order(value, &Real::zero())? != Ordering::Less
        && order(value, &Real::one())? != Ordering::Greater)
}

fn intersect_lines(
    first: &Curve3,
    (a, a_end): (&Point3, &Point3),
    second: &Curve3,
    (b, b_end): (&Point3, &Point3),
) -> GeometryResult<CurveCurveIntersection> {
    let d = a_end - a;
    let e = b_end - b;
    let offset = b - a;
    let normal = d.cross(&e);
    let normal_squared = normal.norm_squared();
    if order(&normal_squared, &Real::zero())? != Ordering::Equal {
        if order(&offset.dot(&normal), &Real::zero())? != Ordering::Equal {
            return Ok(CurveCurveIntersection::None);
        }
        let t =
            (offset.cross(&e).dot(&normal) / &normal_squared).map_err(GeometryError::Numeric)?;
        let s =
            (offset.cross(&d).dot(&normal) / &normal_squared).map_err(GeometryError::Numeric)?;
        if !in_unit(&t)? || !in_unit(&s)? {
            return Ok(CurveCurveIntersection::None);
        }
        return Ok(CurveCurveIntersection::Points(vec![CurveCurvePoint {
            point: first.evaluate_real(&t)?.into(),
            first: t.into(),
            second: s.into(),
        }]));
    }
    // Parallel supports meet only when collinear.
    if order(&offset.cross(&d).norm_squared(), &Real::zero())? != Ordering::Equal {
        return Ok(CurveCurveIntersection::None);
    }
    let d_squared = d.norm_squared();
    let start = (offset.dot(&d) / &d_squared).map_err(GeometryError::Numeric)?;
    let end = ((b_end - a).dot(&d) / &d_squared).map_err(GeometryError::Numeric)?;
    let (low, high) = if order(&start, &end)? == Ordering::Less {
        (start.clone(), end.clone())
    } else {
        (end.clone(), start.clone())
    };
    let low = if order(&low, &Real::zero())? == Ordering::Less {
        Real::zero()
    } else {
        low
    };
    let high = if order(&high, &Real::one())? == Ordering::Greater {
        Real::one()
    } else {
        high
    };
    // Second-line parameter of a first-line parameter t: (t - start)/(end - start).
    let span = &end - &start;
    let second_of = |t: &Real| ((t - &start) / &span).map_err(GeometryError::Numeric);
    match order(&low, &high)? {
        Ordering::Greater => Ok(CurveCurveIntersection::None),
        Ordering::Equal => Ok(CurveCurveIntersection::Points(vec![CurveCurvePoint {
            point: first.evaluate_real(&low)?.into(),
            second: second_of(&low)?.into(),
            first: low.into(),
        }])),
        Ordering::Less => {
            let (s_low, s_high) = (second_of(&low)?, second_of(&high)?);
            let (s_low, s_high) = if order(&s_low, &s_high)? == Ordering::Less {
                (s_low, s_high)
            } else {
                (s_high, s_low)
            };
            let _ = second;
            Ok(CurveCurveIntersection::Overlap {
                first: ParameterRange::new(low.into(), high.into())?,
                second: ParameterRange::new(s_low.into(), s_high.into())?,
            })
        }
    }
}

/// Native rational controls, weights and optional knots of a curve family.
struct RationalNet {
    controls: Vec<Point3>,
    weights: Vec<Real>,
    knots: Option<(usize, Vec<Real>)>,
}

fn rational_net(curve: &Curve3) -> Option<RationalNet> {
    match &curve.data.geometry {
        CurveGeometry3::RationalBezier(bezier) => Some(RationalNet {
            controls: bezier.control_points.clone(),
            weights: bezier.weights.clone(),
            knots: None,
        }),
        CurveGeometry3::Nurbs(nurbs) => Some(RationalNet {
            controls: nurbs.control_points.clone(),
            weights: nurbs.weights.clone(),
            knots: Some((nurbs.degree, nurbs.knots.clone())),
        }),
        _ => None,
    }
}

impl RationalNet {
    /// The same family, weights and knots with projected planar controls.
    fn image(&self, points: Vec<CurvePoint2>) -> GeometryResult<Curve2> {
        Ok(match &self.knots {
            None => Curve2::from(RationalBezier2::try_new(points, self.weights.clone())?),
            Some((degree, knots)) => {
                Curve2::try_nurbs(*degree, points, self.weights.clone(), knots.clone())?
            }
        })
    }
}

fn intersect_line_curve(
    line: &Curve3,
    (a, a_end): (&Point3, &Point3),
    curve: &Curve3,
) -> GeometryResult<CurveCurveIntersection> {
    let Some(net) = rational_net(curve) else {
        return Err(GeometryError::UnsupportedIntersection);
    };
    let d = a_end - a;
    let mut first = d.cross(&Vector3::x());
    if order(&first.norm_squared(), &Real::zero())? == Ordering::Equal {
        first = d.cross(&Vector3::y());
    }
    let second = d.cross(&first);
    let relative = |p: &Point3| p - a;
    // A curve whose whole control net lies on the line lies along it.
    let mut on_line = true;
    for control in &net.controls {
        let r = relative(control);
        if order(&first.dot(&r), &Real::zero())? != Ordering::Equal
            || order(&second.dot(&r), &Real::zero())? != Ordering::Equal
        {
            on_line = false;
            break;
        }
    }
    if on_line {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let normal_image = net.image(
        net.controls
            .iter()
            .map(|p| {
                let r = relative(p);
                CurvePoint2::new(first.dot(&r), second.dot(&r))
            })
            .collect(),
    )?;
    let d_squared = d.norm_squared();
    let along = net
        .controls
        .iter()
        .map(|p| (d.dot(&relative(p)) / &d_squared).map_err(GeometryError::Numeric))
        .collect::<GeometryResult<Vec<_>>>()?;
    // A constant line coordinate needs no planar image.
    let constant = {
        let mut constant = true;
        for value in &along[1..] {
            if order(value, &along[0])? != Ordering::Equal {
                constant = false;
                break;
            }
        }
        constant
    };
    let along_image = if constant {
        None
    } else {
        Some(
            net.image(
                along
                    .iter()
                    .map(|value| CurvePoint2::new(value.clone(), Real::zero()))
                    .collect(),
            )?,
        )
    };
    let locations = match normal_image
        .point_locations(&CurvePoint2::new(Real::zero(), Real::zero()).into())?
    {
        CurvePointLocations2::EntireCurve => return Err(GeometryError::UnsupportedIntersection),
        CurvePointLocations2::Locations(locations) => locations,
    };
    let line_domain = line.domain();
    let curve_domain = curve.domain();
    let mut points = Vec::new();
    for location in locations {
        let selected = location.parameter()?;
        let s = Parameter::from_curve(&selected)?;
        if !curve_domain.contains(&s)? {
            continue;
        }
        let t = match &along_image {
            None => Parameter::from_real(along[0].clone()),
            Some(image) => Parameter::from_curve(
                &image
                    .point_at(&selected)?
                    .affine_form(Real::one(), Real::zero(), Real::zero())
                    .parameter()?,
            )?,
        };
        if !line_domain.contains(&t)? {
            continue;
        }
        points.push(CurveCurvePoint {
            point: curve.point_at(&s)?,
            first: t,
            second: s,
        });
    }
    sort_by_first(&mut points);
    Ok(if points.is_empty() {
        CurveCurveIntersection::None
    } else {
        CurveCurveIntersection::Points(points)
    })
}

/// A curve's exact image in a plane frame and its affine map back to the
/// spatial chart: `spatial = scale * planar + offset`.
struct PlanarView {
    curve: Curve2,
    scale: Real,
    offset: Real,
}

/// The plane frame `(origin, u, v)` of a planar image, or of three
/// noncollinear controls of a line or rational net.
fn plane_frame(curve: &Curve3) -> GeometryResult<Option<(Point3, Vector3, Vector3)>> {
    if let CurveGeometry3::PlanarImage(image) = &curve.data.geometry {
        return Ok(Some((
            image.origin.clone(),
            image.u.clone(),
            image.v.clone(),
        )));
    }
    let controls = match (curve.line_endpoints(), rational_net(curve)) {
        (Some((a, b)), _) => vec![a.clone(), b.clone()],
        (None, Some(net)) => net.controls,
        (None, None) => return Ok(None),
    };
    let origin = controls[0].clone();
    for (i, a) in controls.iter().enumerate().skip(1) {
        let u = a - &origin;
        for b in &controls[i + 1..] {
            let v = b - &origin;
            if order(&u.cross(&v).norm_squared(), &Real::zero())? != Ordering::Equal {
                return Ok(Some((origin, u, v)));
            }
        }
    }
    Ok(None)
}

/// The curve's exact image in the frame's plane, or `None` when it does not
/// lie in that plane or has no admitted planar view.
fn planar_view(
    curve: &Curve3,
    (origin, u, v): &(Point3, Vector3, Vector3),
) -> GeometryResult<Option<PlanarView>> {
    let normal = u.cross(v);
    let on_plane = |p: &Point3| -> GeometryResult<bool> {
        Ok(order(&normal.dot(&(p - origin)), &Real::zero())? == Ordering::Equal)
    };
    let project = |p: &Point3| -> GeometryResult<CurvePoint2> {
        let projected = project_point_to_plane_frame(origin, u, v, p)?;
        Ok(CurvePoint2::new(projected.x, projected.y))
    };
    if let Some((a, b)) = curve.line_endpoints() {
        if !on_plane(a)? || !on_plane(b)? {
            return Ok(None);
        }
        return Ok(Some(PlanarView {
            curve: Curve2::from(LineSeg2::try_new(project(a)?, project(b)?)?),
            scale: Real::one(),
            offset: Real::zero(),
        }));
    }
    if let Some(net) = rational_net(curve) {
        let mut points = Vec::with_capacity(net.controls.len());
        for control in &net.controls {
            if !on_plane(control)? {
                return Ok(None);
            }
            points.push(project(control)?);
        }
        return Ok(Some(PlanarView {
            curve: net.image(points)?,
            scale: Real::one(),
            offset: Real::zero(),
        }));
    }
    if let CurveGeometry3::PlanarImage(image) = &curve.data.geometry {
        // The image's own plane must be the frame's plane.
        if order(
            &normal.cross(&image.u.cross(&image.v)).norm_squared(),
            &Real::zero(),
        )? != Ordering::Equal
            || !on_plane(&image.origin)?
        {
            return Ok(None);
        }
        let materialized = match image.materialize_plane_projection(origin, u, v) {
            Ok(materialized) => materialized,
            // Frames related by a non-similarity have no exact source image.
            Err(GeometryError::PlanarCurve(_) | GeometryError::UnsupportedIntersection) => {
                return Ok(None);
            }
            Err(error) => return Err(error),
        };
        let Some((scale, offset)) = materialized
            .correspondence
            .affine_coefficients()
            .map(|(scale, offset)| (scale.clone(), offset.clone()))
        else {
            return Ok(None);
        };
        return Ok(Some(PlanarView {
            curve: materialized.curve,
            scale,
            offset,
        }));
    }
    Ok(None)
}

fn intersect_coplanar(
    first: &Curve3,
    second: &Curve3,
) -> GeometryResult<Option<CurveCurveIntersection>> {
    // Prefer a planar image's frame so its source needs no change of frame.
    let frame = match (&first.data.geometry, &second.data.geometry) {
        (_, CurveGeometry3::PlanarImage(_)) => plane_frame(second)?,
        _ => match plane_frame(first)? {
            Some(frame) => Some(frame),
            None => plane_frame(second)?,
        },
    };
    let Some(frame) = frame else {
        return Ok(None);
    };
    let (Some(a), Some(b)) = (planar_view(first, &frame)?, planar_view(second, &frame)?) else {
        return Ok(None);
    };
    let intersection = a.curve.intersect_curve(&b.curve)?;
    if let Some(blocker) = intersection.blockers().first() {
        return Err(match blocker.kind() {
            hypercurve::CurveIntersectionPairBlockerKind2::Uncertain(reason) => {
                GeometryError::PlanarClassificationUnresolved(*reason)
            }
            _ => GeometryError::UnsupportedIntersection,
        });
    }
    let spatial = |view: &PlanarView, parameter: &hypercurve::CurveParameter2| {
        Parameter::from_curve(parameter)?.affine_image(&view.scale, &view.offset)
    };
    let range = |view: &PlanarView, index, local: &hypercurve::CurveParameterRange2| {
        let start = spatial(view, &view.curve.parameter_from_span(index, local.start())?)?;
        let end = spatial(view, &view.curve.parameter_from_span(index, local.end())?)?;
        if start.compare(&end)? == Ordering::Greater {
            ParameterRange::new(end, start)
        } else {
            ParameterRange::new(start, end)
        }
    };
    let mut overlaps = Vec::new();
    for overlap in intersection.overlaps() {
        if !overlap.includes_start() || !overlap.includes_end() {
            return Err(GeometryError::UnsupportedIntersection);
        }
        overlaps.push((
            range(&a, overlap.first_span_index(), overlap.first_range())?,
            range(&b, overlap.second_span_index(), overlap.second_range())?,
        ));
    }
    if !intersection.parameter_components().is_empty() {
        // Collapsed-span components need a point/range pairing not yet exposed.
        return Err(GeometryError::UnsupportedIntersection);
    }
    let mut points = Vec::new();
    'contact: for contact in intersection.contacts() {
        let t = spatial(&a, &contact.first().parameter()?)?;
        let s = spatial(&b, &contact.second().parameter()?)?;
        if !first.domain().contains(&t)? || !second.domain().contains(&s)? {
            continue;
        }
        for (range, _) in &overlaps {
            if range.contains(&t)? {
                continue 'contact;
            }
        }
        points.push(CurveCurvePoint {
            point: first.point_at(&t)?,
            first: t,
            second: s,
        });
    }
    sort_by_first(&mut points);
    Ok(Some(match (points.is_empty(), overlaps.len()) {
        (true, 0) => CurveCurveIntersection::None,
        (false, 0) => CurveCurveIntersection::Points(points),
        (true, 1) => {
            let (first, second) = overlaps.pop().expect("one overlap");
            CurveCurveIntersection::Overlap { first, second }
        }
        _ => CurveCurveIntersection::Components { points, overlaps },
    }))
}

/// A line not lying in a planar image's plane meets it at most at the line's
/// single plane crossing, which is located exactly on the planar source.
fn intersect_line_planar_image(
    line: &Curve3,
    curve: &Curve3,
    image: &PlanarCurve3,
) -> GeometryResult<CurveCurveIntersection> {
    let plane = Surface::plane(image.origin.clone(), image.u.clone(), image.v.clone())?;
    let crossing = match plane.intersect_curve(line)? {
        CurveSurfaceIntersection::None => return Ok(CurveCurveIntersection::None),
        CurveSurfaceIntersection::Points(mut points) if points.len() == 1 => points.remove(0),
        // A coplanar line was handled by the planar route.
        _ => return Err(GeometryError::UnsupportedIntersection),
    };
    let Some(point) = crossing.point.try_coordinates() else {
        return Err(GeometryError::UnsupportedIntersection);
    };
    let parameters = match curve.parameters_of(point)? {
        CurveParameterLocation::None => return Ok(CurveCurveIntersection::None),
        CurveParameterLocation::Parameters(parameters) => parameters,
        CurveParameterLocation::EntireDomain => {
            return Err(GeometryError::UnsupportedIntersection);
        }
    };
    Ok(CurveCurveIntersection::Points(
        parameters
            .into_iter()
            .map(|parameter| CurveCurvePoint {
                first: crossing.parameter.clone(),
                second: parameter,
                point: crossing.point.clone(),
            })
            .collect(),
    ))
}

/// Every spatial incidence of two curves projects to an incidence of their
/// images under a linear map with a one-dimensional kernel, which keeps each
/// rational family, weight, knot and parameter. The complete planar result
/// supplies candidate parameter pairs; exact spatial coincidence keeps only
/// true incidences. A projection along which the images overlap or collapse
/// is replaced by another direction; persistent overlaps stay typed.
fn intersect_rational_pair(
    first: &Curve3,
    a: &RationalNet,
    second: &Curve3,
    b: &RationalNet,
) -> GeometryResult<CurveCurveIntersection> {
    let directions = [
        Vector3::z(),
        Vector3::x(),
        Vector3::y(),
        Vector3::new([1.into(), 2.into(), 3.into()]),
    ];
    'direction: for direction in &directions {
        let mut e1 = direction.cross(&Vector3::x());
        if order(&e1.norm_squared(), &Real::zero())? == Ordering::Equal {
            e1 = direction.cross(&Vector3::y());
        }
        let e2 = direction.cross(&e1);
        let project = |net: &RationalNet| -> GeometryResult<Option<Curve2>> {
            let points = net
                .controls
                .iter()
                .map(|p| {
                    let p = Vector3::from(p.clone());
                    CurvePoint2::new(e1.dot(&p), e2.dot(&p))
                })
                .collect::<Vec<_>>();
            let mut collapsed = true;
            for point in &points[1..] {
                if order(point.x(), points[0].x())? != Ordering::Equal
                    || order(point.y(), points[0].y())? != Ordering::Equal
                {
                    collapsed = false;
                    break;
                }
            }
            Ok(if collapsed {
                None
            } else {
                Some(net.image(points)?)
            })
        };
        let (Some(first_image), Some(second_image)) = (project(a)?, project(b)?) else {
            continue;
        };
        let intersection = first_image.intersect_curve(&second_image)?;
        if let Some(blocker) = intersection.blockers().first() {
            match blocker.kind() {
                hypercurve::CurveIntersectionPairBlockerKind2::SharedComponent => {
                    continue 'direction;
                }
                hypercurve::CurveIntersectionPairBlockerKind2::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(*reason));
                }
                hypercurve::CurveIntersectionPairBlockerKind2::IncompleteReplay => {
                    return Err(GeometryError::UnsupportedIntersection);
                }
            }
        }
        if !intersection.overlaps().is_empty() || !intersection.parameter_components().is_empty() {
            continue;
        }
        let mut points = Vec::new();
        for contact in intersection.contacts() {
            let t = Parameter::from_curve(&contact.first().parameter()?)?;
            let s = Parameter::from_curve(&contact.second().parameter()?)?;
            if !first.domain().contains(&t)? || !second.domain().contains(&s)? {
                continue;
            }
            let point = first.point_at(&t)?;
            if point.coincides_with(&second.point_at(&s)?)? {
                points.push(CurveCurvePoint {
                    point,
                    first: t,
                    second: s,
                });
            }
        }
        sort_by_first(&mut points);
        return Ok(if points.is_empty() {
            CurveCurveIntersection::None
        } else {
            CurveCurveIntersection::Points(points)
        });
    }
    Err(GeometryError::UnsupportedIntersection)
}

/// A rational curve not lying in a planar image's plane meets the image only
/// at its complete plane contacts. Each contact's plane-frame coordinates are
/// the rational curve's exact projection evaluated at the same selected
/// parameter, which is located on the planar source without forming
/// independent coordinates.
fn intersect_planar_image_rational(
    curve: &Curve3,
    image: &PlanarCurve3,
    other: &Curve3,
) -> GeometryResult<CurveCurveIntersection> {
    let net = rational_net(other).expect("caller checked a rational net");
    let plane = Surface::plane(image.origin.clone(), image.u.clone(), image.v.clone())?;
    let contacts = match plane.intersect_curve(other)? {
        CurveSurfaceIntersection::None => return Ok(CurveCurveIntersection::None),
        CurveSurfaceIntersection::Points(points) => points,
        // A coplanar curve was handled by the planar route.
        _ => return Err(GeometryError::UnsupportedIntersection),
    };
    let materialized = image.materialize_plane_projection(&image.origin, &image.u, &image.v)?;
    let (scale, offset) = materialized
        .correspondence
        .affine_coefficients()
        .map(|(scale, offset)| (scale.clone(), offset.clone()))
        .ok_or(GeometryError::UnsupportedIntersection)?;
    let projected = net.image(
        net.controls
            .iter()
            .map(|p| {
                let point = project_point_to_plane_frame(&image.origin, &image.u, &image.v, p)?;
                Ok(CurvePoint2::new(point.x, point.y))
            })
            .collect::<GeometryResult<Vec<_>>>()?,
    )?;
    let mut points = Vec::new();
    for contact in contacts {
        let uv = projected.point_at(&contact.parameter.curve_parameter())?;
        let locations = match materialized.curve.point_locations(&uv)? {
            CurvePointLocations2::EntireCurve => {
                return Err(GeometryError::UnsupportedIntersection);
            }
            CurvePointLocations2::Locations(locations) => locations,
        };
        for location in locations {
            let parameter =
                Parameter::from_curve(&location.parameter()?)?.affine_image(&scale, &offset)?;
            if curve.domain().contains(&parameter)? {
                points.push(CurveCurvePoint {
                    first: parameter,
                    second: contact.parameter.clone(),
                    point: contact.point.clone(),
                });
            }
        }
    }
    sort_by_first(&mut points);
    Ok(if points.is_empty() {
        CurveCurveIntersection::None
    } else {
        CurveCurveIntersection::Points(points)
    })
}

/// A line against a circular or elliptic arc `C + rx cos(θ) X + ry sin(θ) Y`
/// with orthonormal `X, Y`. A coplanar line satisfies the exact quadratic
/// `ry^2 (α + t a)^2 + rx^2 (β + t b)^2 = rx^2 ry^2`; a line crossing the plane
/// meets it at most at its single crossing. Arc parameters, including seam
/// visits, come from the arc's exact point location.
fn intersect_line_conic(
    line: &Curve3,
    (start, end): (&Point3, &Point3),
    curve: &Curve3,
    arc: &EllipseArc3,
) -> GeometryResult<CurveCurveIntersection> {
    let direction = end - start;
    let normal = arc.x.cross(&arc.y);
    let relative = start - &arc.center;
    let zero = |value: &Real| -> GeometryResult<bool> {
        Ok(order(value, &Real::zero())? == Ordering::Equal)
    };
    let along_normal = normal.dot(&direction);
    let offset = normal.dot(&relative);
    let parameters: Vec<Real> = if zero(&along_normal)? {
        if !zero(&offset)? {
            return Ok(CurveCurveIntersection::None);
        }
        let (alpha, beta) = (relative.dot(&arc.x), relative.dot(&arc.y));
        let (da, db) = (direction.dot(&arc.x), direction.dot(&arc.y));
        let rx2 = &arc.x_radius * &arc.x_radius;
        let ry2 = &arc.y_radius * &arc.y_radius;
        let a = &ry2 * &da * &da + &rx2 * &db * &db;
        let b = Real::from(2) * (&ry2 * &alpha * &da + &rx2 * &beta * &db);
        let c = &ry2 * &alpha * &alpha + &rx2 * &beta * &beta - &rx2 * &ry2;
        match quadratic_roots(a, b, c)? {
            QuadraticRoots::None => Vec::new(),
            QuadraticRoots::Isolated(roots) => roots.into_iter().map(|(t, _)| t).collect(),
            // A nonzero in-plane direction keeps the leading coefficient positive.
            QuadraticRoots::All => return Err(GeometryError::UnsupportedIntersection),
        }
    } else {
        vec![((-offset) / along_normal).map_err(GeometryError::Numeric)?]
    };
    let mut points = Vec::new();
    for t in parameters {
        if !in_unit(&t)? {
            continue;
        }
        let point = line.evaluate_real(&t)?;
        let visits = match curve.parameters_of(&point)? {
            CurveParameterLocation::None => continue,
            CurveParameterLocation::Parameters(parameters) => parameters,
            CurveParameterLocation::EntireDomain => {
                return Err(GeometryError::UnsupportedIntersection);
            }
        };
        for visit in visits {
            points.push(CurveCurvePoint {
                first: t.clone().into(),
                second: visit,
                point: point.clone().into(),
            });
        }
    }
    sort_by_first(&mut points);
    Ok(if points.is_empty() {
        CurveCurveIntersection::None
    } else {
        CurveCurveIntersection::Points(points)
    })
}

/// Two circular arcs. Common points lie on a line: the radical line of
/// coplanar circles, or the intersection line of their planes. That line is
/// intersected exactly with the first circle, candidates are kept only on the
/// second circle, and both arc parameters come from exact point location.
fn intersect_circles(
    first_curve: &Curve3,
    first: &EllipseArc3,
    second_curve: &Curve3,
    second: &EllipseArc3,
) -> GeometryResult<CurveCurveIntersection> {
    let zero = |value: &Real| -> GeometryResult<bool> {
        Ok(order(value, &Real::zero())? == Ordering::Equal)
    };
    let n1 = first.x.cross(&first.y);
    let n2 = second.x.cross(&second.y);
    let (r1, r2) = (&first.x_radius, &second.x_radius);
    let between = &second.center - &first.center;
    let direction = n1.cross(&n2);
    let (origin, along) = if zero(&direction.norm_squared())? {
        // Parallel planes: distinct planes miss; coplanar circles meet on the
        // radical line 2 (c2 - c1)·(p - c1) = |c2 - c1|^2 + r1^2 - r2^2.
        if !zero(&n1.dot(&between))? {
            return Ok(CurveCurveIntersection::None);
        }
        let distance = between.norm_squared();
        if zero(&distance)? {
            return if order(r1, r2)? == Ordering::Equal {
                intersect_coincident_arcs(first_curve, second_curve)
            } else {
                Ok(CurveCurveIntersection::None)
            };
        }
        let lambda = ((&distance + r1 * r1 - r2 * r2) / (Real::from(2) * &distance))
            .map_err(GeometryError::Numeric)?;
        (
            first.center.clone() + between.clone() * lambda,
            n1.cross(&between),
        )
    } else {
        // The point of both planes nearest the origin, from n_i·p = d_i.
        let d1 = n1.dot(&Vector3::from(first.center.clone()));
        let d2 = n2.dot(&Vector3::from(second.center.clone()));
        let (n11, n22, n12) = (n1.norm_squared(), n2.norm_squared(), n1.dot(&n2));
        let det = &n11 * &n22 - &n12 * &n12;
        let a = ((&d1 * &n22 - &d2 * &n12) / &det).map_err(GeometryError::Numeric)?;
        let b = ((&d2 * &n11 - &d1 * &n12) / &det).map_err(GeometryError::Numeric)?;
        (
            Point3::origin() + n1.clone() * a + n2.clone() * b,
            direction,
        )
    };
    // |origin + τ along - c1|^2 = r1^2.
    let relative = &origin - &first.center;
    let roots = match quadratic_roots(
        along.norm_squared(),
        Real::from(2) * relative.dot(&along),
        relative.norm_squared() - r1 * r1,
    )? {
        QuadraticRoots::None => Vec::new(),
        QuadraticRoots::Isolated(roots) => roots,
        QuadraticRoots::All => return Err(GeometryError::UnsupportedIntersection),
    };
    let mut points = Vec::new();
    for (tau, _) in roots {
        let point = origin.clone() + along.clone() * &tau;
        if order(&(&point - &second.center).norm_squared(), &(r2 * r2))? != Ordering::Equal {
            continue;
        }
        let visits = |curve: &Curve3| -> GeometryResult<Vec<Parameter>> {
            Ok(match curve.parameters_of(&point)? {
                CurveParameterLocation::None => Vec::new(),
                CurveParameterLocation::Parameters(parameters) => parameters,
                CurveParameterLocation::EntireDomain => {
                    return Err(GeometryError::UnsupportedIntersection);
                }
            })
        };
        let second_visits = visits(second_curve)?;
        for t in visits(first_curve)? {
            for s in &second_visits {
                points.push(CurveCurvePoint {
                    first: t.clone(),
                    second: s.clone(),
                    point: point.clone().into(),
                });
            }
        }
    }
    sort_by_first(&mut points);
    Ok(if points.is_empty() {
        CurveCurveIntersection::None
    } else {
        CurveCurveIntersection::Points(points)
    })
}

/// Two arcs of one circle. On the first arc, the overlap can only change at
/// its own endpoints or at the second arc's endpoints; each interval between
/// consecutive boundary parameters is tested at its exact midpoint. Ranges
/// map to the second chart by endpoint visits whose angular lengths agree
/// (equal radii preserve arc length); uncovered boundary parameters lying on
/// both arcs are isolated touching points.
fn intersect_coincident_arcs(
    first: &Curve3,
    second: &Curve3,
) -> GeometryResult<CurveCurveIntersection> {
    let visits = |curve: &Curve3, point: &Point3| -> GeometryResult<Vec<Parameter>> {
        Ok(match curve.parameters_of(point)? {
            CurveParameterLocation::None => Vec::new(),
            CurveParameterLocation::Parameters(parameters) => parameters,
            CurveParameterLocation::EntireDomain => {
                return Err(GeometryError::UnsupportedIntersection);
            }
        })
    };
    let real = |parameter: &Parameter| {
        parameter
            .try_real()
            .cloned()
            .ok_or(GeometryError::UnsupportedIntersection)
    };
    let (start, end) = (real(first.domain().start())?, real(first.domain().end())?);
    let mut boundaries = vec![start.clone(), end.clone()];
    for endpoint in [second.start()?, second.end()?] {
        for visit in visits(first, &endpoint)? {
            boundaries.push(real(&visit)?);
        }
    }
    // Sort and deduplicate exactly.
    let mut sorted: Vec<Real> = Vec::new();
    for value in boundaries {
        let mut index = sorted.len();
        let mut duplicate = false;
        while index > 0 {
            match order(&value, &sorted[index - 1])? {
                Ordering::Less => index -= 1,
                Ordering::Equal => {
                    duplicate = true;
                    break;
                }
                Ordering::Greater => break,
            }
        }
        if !duplicate {
            sorted.insert(index, value);
        }
    }
    let mut overlaps = Vec::new();
    let mut covered = vec![false; sorted.len()];
    for index in 0..sorted.len() - 1 {
        let (a0, a1) = (&sorted[index], &sorted[index + 1]);
        let middle = ((a0 + a1) / Real::from(2)).map_err(GeometryError::Numeric)?;
        if visits(second, &first.evaluate_real(&middle)?)?.is_empty() {
            continue;
        }
        let length = a1 - a0;
        let starts = visits(second, &first.evaluate_real(a0)?)?;
        let ends = visits(second, &first.evaluate_real(a1)?)?;
        let mut mapped = None;
        'pair: for b0 in &starts {
            for b1 in &ends {
                let (b0r, b1r) = (real(b0)?, real(b1)?);
                let span = &b1r - &b0r;
                if order(&span, &length)? == Ordering::Equal
                    || order(&-span, &length)? == Ordering::Equal
                {
                    mapped = Some(if order(&b0r, &b1r)? == Ordering::Less {
                        (b0r, b1r)
                    } else {
                        (b1r, b0r)
                    });
                    break 'pair;
                }
            }
        }
        let Some((b_low, b_high)) = mapped else {
            return Err(GeometryError::UnsupportedIntersection);
        };
        covered[index] = true;
        covered[index + 1] = true;
        overlaps.push((
            ParameterRange::new(a0.clone().into(), a1.clone().into())?,
            ParameterRange::new(b_low.into(), b_high.into())?,
        ));
    }
    // Merge ranges that meet at a shared boundary on the first arc.
    let mut merged: Vec<(ParameterRange, ParameterRange)> = Vec::new();
    for (a, b) in overlaps {
        if let Some((last_a, last_b)) = merged.last_mut()
            && last_a.end().compare(a.start())? == Ordering::Equal
            && (last_b.end().compare(b.start())? == Ordering::Equal
                || last_b.start().compare(b.end())? == Ordering::Equal)
        {
            let low = if last_b.start().compare(b.start())? == Ordering::Less {
                last_b.start().clone()
            } else {
                b.start().clone()
            };
            let high = if last_b.end().compare(b.end())? == Ordering::Greater {
                last_b.end().clone()
            } else {
                b.end().clone()
            };
            *last_a = ParameterRange::new(last_a.start().clone(), a.end().clone())?;
            *last_b = ParameterRange::new(low, high)?;
            continue;
        }
        merged.push((a, b));
    }
    let mut points = Vec::new();
    for (index, value) in sorted.iter().enumerate() {
        if covered[index] {
            continue;
        }
        let point = first.evaluate_real(value)?;
        for visit in visits(second, &point)? {
            points.push(CurveCurvePoint {
                first: value.clone().into(),
                second: visit,
                point: point.clone().into(),
            });
        }
    }
    Ok(match (points.is_empty(), merged.len()) {
        (true, 0) => CurveCurveIntersection::None,
        (false, 0) => CurveCurveIntersection::Points(points),
        (true, 1) => {
            let (first, second) = merged.pop().expect("one overlap");
            CurveCurveIntersection::Overlap { first, second }
        }
        _ => CurveCurveIntersection::Components {
            points,
            overlaps: merged,
        },
    })
}
