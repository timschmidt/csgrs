//! Exact inverse queries on spatial curves, including selected roots.

use super::{
    Curve3, CurveParameterLocation, EllipseArc3, Line3, NurbsCurve3, RationalBezier3,
    certified_atan2, control_coordinate, decided_order, decompose_nurbs_into_bezier_segments,
    ellipse_arc_parameters_at_angle, points_equal, surface_projection_varies,
};
use crate::{GeometryError, GeometryResult, Parameter, Point3, Real};
use hypercurve::{Curve2, CurvePointLocations2, Point2 as CurvePoint2, RationalBezier2};
use hyperlimit::{PredicateOutcome, compare_reals, point3_equal};
use std::cmp::Ordering;

pub(super) fn locate_line_parameter(
    curve: &Curve3,
    line: &Line3,
    point: &Point3,
) -> GeometryResult<CurveParameterLocation> {
    let direction = &line.end - &line.start;
    let offset = point - &line.start;
    let mut parameter = None;
    let mut parameter_axis = None;
    for axis in 0..3 {
        match decided_order(compare_reals(
            &direction.0[axis],
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? {
            Ordering::Equal => {}
            Ordering::Less | Ordering::Greater => {
                parameter =
                    Some((&offset.0[axis] / &direction.0[axis]).map_err(GeometryError::Numeric)?);
                parameter_axis = Some(axis);
                break;
            }
        }
    }
    let parameter = parameter.ok_or(GeometryError::DegenerateLine)?;
    if !curve.domain().contains(&parameter)? {
        return Ok(CurveParameterLocation::None);
    }
    let parameter_axis = parameter_axis.expect("a line parameter has a source axis");
    for axis in 0..3 {
        if axis == parameter_axis {
            continue;
        }
        let left = &offset.0[axis] * &direction.0[parameter_axis];
        let right = &offset.0[parameter_axis] * &direction.0[axis];
        if decided_order(compare_reals(&left, &right, crate::STRICT_PREDICATES))? != Ordering::Equal
        {
            return Ok(CurveParameterLocation::None);
        }
    }
    Ok(CurveParameterLocation::Parameters(vec![parameter.into()]))
}

pub(super) fn locate_rational_bezier_parameters(
    curve: &Curve3,
    rational: &RationalBezier3,
    point: &Point3,
) -> GeometryResult<CurveParameterLocation> {
    let mut projection = None;
    for axes in [(0, 1), (0, 2), (1, 2)] {
        if surface_projection_varies(&rational.control_points, axes)? {
            projection = Some(axes);
            break;
        }
    }
    let Some((first_axis, second_axis)) = projection else {
        return if points_equal(&rational.control_points[0], point)? {
            Ok(CurveParameterLocation::EntireDomain)
        } else {
            Ok(CurveParameterLocation::None)
        };
    };
    let projected = project_bezier(rational, (first_axis, second_axis))?;
    let query = CurvePoint2::new(
        control_coordinate(point, first_axis).clone(),
        control_coordinate(point, second_axis).clone(),
    );
    match projected.point_locations(&query.into())? {
        CurvePointLocations2::EntireCurve => Err(GeometryError::UnsupportedParameterLocation),
        CurvePointLocations2::Locations(locations) => {
            let mut parameters = Vec::new();
            let mut remaining_projection = None;
            let remaining_axis = 3 - first_axis - second_axis;
            for location in locations {
                // One unit-chart span: local and authored parameters agree.
                let parameter = location.local_parameter();
                let incident = if let Some(value) = parameter.scalar() {
                    // Preserve the represented-value path without constructing
                    // another projected carrier or a selected point image.
                    points_equal(&curve.evaluate_real(value)?, point)?
                } else if let Some(constant) = &rational.constant_coordinates[remaining_axis] {
                    decided_order(compare_reals(
                        constant,
                        control_coordinate(point, remaining_axis),
                        crate::STRICT_PREDICATES,
                    ))? == Ordering::Equal
                } else {
                    // Replay the missing coordinate at the SAME selected root.
                    // Independent projection roots need not describe one 3D point.
                    if remaining_projection.is_none() {
                        remaining_projection =
                            Some(project_bezier(rational, (first_axis, remaining_axis))?);
                    }
                    let query = CurvePoint2::new(
                        control_coordinate(point, first_axis).clone(),
                        control_coordinate(point, remaining_axis).clone(),
                    );
                    remaining_projection
                        .as_ref()
                        .expect("projection initialized above")
                        .point_at(parameter)?
                        .coincides_with(&query.into())?
                };
                if incident {
                    let parameter = parameter
                        .as_bezier_parameter()
                        .ok_or(GeometryError::UnsupportedParameterLocation)?;
                    parameters.push(Parameter::from_bezier(parameter.clone()));
                }
            }
            if parameters.is_empty() {
                Ok(CurveParameterLocation::None)
            } else {
                Ok(CurveParameterLocation::Parameters(parameters))
            }
        }
    }
}

fn project_bezier(rational: &RationalBezier3, axes: (usize, usize)) -> GeometryResult<Curve2> {
    let controls = rational
        .control_points
        .iter()
        .map(|control| {
            CurvePoint2::new(
                control_coordinate(control, axes.0).clone(),
                control_coordinate(control, axes.1).clone(),
            )
        })
        .collect();
    Ok(Curve2::from(RationalBezier2::try_new(
        controls,
        rational.weights.clone(),
    )?))
}

pub(super) fn locate_nurbs_parameters(
    nurbs: &NurbsCurve3,
    point: &Point3,
) -> GeometryResult<CurveParameterLocation> {
    let mut all_controls_equal = true;
    for control in &nurbs.control_points[1..] {
        if !points_equal(control, &nurbs.control_points[0])? {
            all_controls_equal = false;
            break;
        }
    }
    if all_controls_equal {
        return if points_equal(&nurbs.control_points[0], point)? {
            Ok(CurveParameterLocation::EntireDomain)
        } else {
            Ok(CurveParameterLocation::None)
        };
    }
    let segments = decompose_nurbs_into_bezier_segments(nurbs)?;
    let mut parameters: Vec<Parameter> = Vec::new();
    for (segment, domain) in segments {
        match segment.parameters_of(point)? {
            CurveParameterLocation::None => {}
            CurveParameterLocation::Parameters(local_parameters) => {
                let span = domain.end() - domain.start();
                for local in local_parameters {
                    let global = local.affine_image(&span, domain.start())?;
                    let mut duplicate = false;
                    for existing in &parameters {
                        if existing.compare(&global)? == Ordering::Equal {
                            duplicate = true;
                            break;
                        }
                    }
                    if !duplicate {
                        parameters.push(global);
                    }
                }
            }
            CurveParameterLocation::EntireDomain => {
                return Err(GeometryError::UnsupportedParameterLocation);
            }
        }
    }
    if parameters.is_empty() {
        Ok(CurveParameterLocation::None)
    } else {
        Ok(CurveParameterLocation::Parameters(parameters))
    }
}

pub(super) fn locate_ellipse_arc_parameters(
    curve: &Curve3,
    arc: &EllipseArc3,
    point: &Point3,
) -> GeometryResult<CurveParameterLocation> {
    let mut endpoints = Vec::new();
    if point3_equal(&curve.start()?, point, crate::STRICT_PREDICATES).value() == Some(true) {
        endpoints.push(curve.domain().start().clone());
    }
    if point3_equal(&curve.end()?, point, crate::STRICT_PREDICATES).value() == Some(true)
        && (endpoints.is_empty()
            || decided_order(compare_reals(
                &endpoints[0],
                curve.domain().end(),
                crate::STRICT_PREDICATES,
            ))? != Ordering::Equal)
    {
        endpoints.push(curve.domain().end().clone());
    }
    if !endpoints.is_empty() {
        return Ok(CurveParameterLocation::Parameters(
            endpoints.into_iter().map(Parameter::from_real).collect(),
        ));
    }

    let relative = point - &arc.center;
    let x_coordinate = arc.x.dot(&relative);
    let y_coordinate = arc.y.dot(&relative);
    let represented = arc.x.clone() * &x_coordinate + arc.y.clone() * &y_coordinate;
    match compare_reals(
        &(&relative - &represented).norm_squared(),
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => {}
        PredicateOutcome::Decided { .. } => return Ok(CurveParameterLocation::None),
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(GeometryError::PredicateUnresolved { needed, stage });
        }
    }
    let normalized_x = (x_coordinate / &arc.x_radius).map_err(GeometryError::Numeric)?;
    let normalized_y = (y_coordinate / &arc.y_radius).map_err(GeometryError::Numeric)?;
    let normalized_radius_squared = &normalized_x * &normalized_x + &normalized_y * &normalized_y;
    match compare_reals(
        &normalized_radius_squared,
        &Real::one(),
        crate::STRICT_PREDICATES,
    ) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => {}
        PredicateOutcome::Decided { .. } => return Ok(CurveParameterLocation::None),
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(GeometryError::PredicateUnresolved { needed, stage });
        }
    }
    let angle = match certified_atan2(normalized_y.clone(), normalized_x.clone()) {
        Ok(angle) => angle,
        Err(GeometryError::PredicateUnresolved { .. }) => {
            atan2_replay_candidate(&normalized_y, &normalized_x)?
        }
        Err(error) => return Err(error),
    };
    let parameters = ellipse_arc_parameters_at_angle(curve, arc, angle)?;
    if parameters.is_empty() {
        return Ok(CurveParameterLocation::None);
    }
    Ok(CurveParameterLocation::Parameters(
        parameters.into_iter().map(Parameter::from_real).collect(),
    ))
}

fn atan2_replay_candidate(y: &Real, x: &Real) -> GeometryResult<Real> {
    let (Some(y_approx), Some(x_approx)) = (y.to_f64_lossy(), x.to_f64_lossy()) else {
        return Err(GeometryError::PredicateUnresolved {
            needed: hyperlimit::RefinementNeed::RealRefinement,
            stage: hyperlimit::Escalation::Undecided,
        });
    };
    if !y_approx.is_finite() || !x_approx.is_finite() {
        return Err(GeometryError::PredicateUnresolved {
            needed: hyperlimit::RefinementNeed::RealRefinement,
            stage: hyperlimit::Escalation::Undecided,
        });
    }
    if x_approx == 0.0 {
        return if y_approx > 0.0 {
            (Real::pi() / Real::from(2)).map_err(GeometryError::Numeric)
        } else if y_approx < 0.0 {
            (-Real::pi() / Real::from(2)).map_err(GeometryError::Numeric)
        } else {
            Err(GeometryError::PredicateUnresolved {
                needed: hyperlimit::RefinementNeed::RealRefinement,
                stage: hyperlimit::Escalation::Undecided,
            })
        };
    }
    let principal = (y / x)
        .map_err(GeometryError::Numeric)?
        .atan()
        .map_err(GeometryError::Numeric)?;
    Ok(if x_approx > 0.0 {
        principal
    } else if y_approx >= 0.0 {
        principal + Real::pi()
    } else {
        principal - Real::pi()
    })
}
