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
}

impl Curve3 {
    /// Intersects this finite curve with another finite curve.
    ///
    /// Line pairs, and lines against positive-weight rational Bézier or NURBS
    /// curves (in either order), return complete exact results with selected
    /// parameters on both curves. A rational curve lying along a line, and
    /// other curve families, fail with
    /// [`GeometryError::UnsupportedIntersection`].
    pub fn intersect_curve(&self, other: &Self) -> GeometryResult<CurveCurveIntersection> {
        crate::context::strict(|| match (self.line_endpoints(), other.line_endpoints()) {
            (Some(first), Some(second)) => intersect_lines(self, first, other, second),
            (Some(line), None) => intersect_line_curve(self, line, other),
            (None, Some(line)) => Ok(swap(intersect_line_curve(other, line, self)?)),
            (None, None) => Err(GeometryError::UnsupportedIntersection),
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
