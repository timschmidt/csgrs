//! Exact arithmetic and geometric support for topology certificates.

use super::*;
use std::cmp::Ordering;

pub(super) fn certified_homothetic_loft_scale(
    lower: &[CurvePoint2],
    upper: &[CurvePoint2],
) -> Result<Option<Real>, GeometryError> {
    if lower.len() != upper.len() || lower.len() < 3 {
        return Ok(None);
    }
    let lower_dx = lower[1].x() - lower[0].x();
    let lower_dy = lower[1].y() - lower[0].y();
    let upper_dx = upper[1].x() - upper[0].x();
    let upper_dy = upper[1].y() - upper[0].y();
    let scale = if decided_model_order(compare_reals(
        &lower_dx,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Equal
    {
        (upper_dx / &lower_dx).map_err(GeometryError::Numeric)?
    } else {
        (upper_dy / &lower_dy).map_err(GeometryError::Numeric)?
    };
    if decided_model_order(compare_reals(
        &scale,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
    {
        return Ok(None);
    }
    for index in 0..lower.len() {
        for (actual, expected) in [
            (
                upper[index].x() - upper[0].x(),
                &scale * (lower[index].x() - lower[0].x()),
            ),
            (
                upper[index].y() - upper[0].y(),
                &scale * (lower[index].y() - lower[0].y()),
            ),
        ] {
            if decided_model_order(compare_reals(&actual, &expected, crate::STRICT_PREDICATES))?
                != std::cmp::Ordering::Equal
            {
                return Ok(None);
            }
        }
    }
    Ok(Some(scale))
}

pub(super) fn certified_convex_loft_interpolation(
    lower: &[CurvePoint2],
    upper: &[CurvePoint2],
) -> Result<bool, GeometryError> {
    if lower.len() != upper.len() || lower.len() < 3 {
        return Ok(false);
    }
    let edge = |points: &[CurvePoint2], index: usize| {
        let next = (index + 1) % points.len();
        (
            points[next].x() - points[index].x(),
            points[next].y() - points[index].y(),
        )
    };
    let cross =
        |first: &(Real, Real), second: &(Real, Real)| &first.0 * &second.1 - &first.1 * &second.0;
    for index in 0..lower.len() {
        let next = (index + 1) % lower.len();
        let lower_edge = edge(lower, index);
        let lower_next = edge(lower, next);
        let upper_edge = edge(upper, index);
        let upper_next = edge(upper, next);
        let lower_turn = cross(&lower_edge, &lower_next);
        let upper_turn = cross(&upper_edge, &upper_next);
        let mixed_turn = cross(&lower_edge, &upper_next) + cross(&upper_edge, &lower_next);
        if decided_model_order(compare_reals(
            &lower_turn,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(
                &upper_turn,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(
                &mixed_turn,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Less
        {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn loft_parameter_area_integral(
    lower: &[CurvePoint2],
    upper: &[CurvePoint2],
) -> Result<Real, GeometryError> {
    let cross =
        |first: &CurvePoint2, second: &CurvePoint2| first.x() * second.y() - first.y() * second.x();
    let mut lower_twice_area = Real::zero();
    let mut upper_twice_area = Real::zero();
    let mut mixed = Real::zero();
    for index in 0..lower.len() {
        let next = (index + 1) % lower.len();
        lower_twice_area += cross(&lower[index], &lower[next]);
        upper_twice_area += cross(&upper[index], &upper[next]);
        mixed += cross(&lower[index], &upper[next]) + cross(&upper[index], &lower[next]);
    }
    ((Real::from(2) * lower_twice_area + mixed + Real::from(2) * upper_twice_area) / Real::from(12))
        .map_err(GeometryError::Numeric)
}

pub(super) fn classify_convex_loft_section(
    lower: &[CurvePoint2],
    upper: &[CurvePoint2],
    parameter: &Real,
    point: &CurvePoint2,
) -> Result<ContourPointLocation, GeometryError> {
    let one_minus = Real::one() - parameter;
    let interpolate = |index: usize| {
        CurvePoint2::new(
            &one_minus * lower[index].x() + parameter * upper[index].x(),
            &one_minus * lower[index].y() + parameter * upper[index].y(),
        )
    };
    let mut boundary = false;
    for index in 0..lower.len() {
        let start = interpolate(index);
        let end = interpolate((index + 1) % lower.len());
        let side = (end.x() - start.x()) * (point.y() - start.y())
            - (end.y() - start.y()) * (point.x() - start.x());
        match decided_model_order(compare_reals(
            &side,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? {
            std::cmp::Ordering::Less => return Ok(ContourPointLocation::Outside),
            std::cmp::Ordering::Equal => boundary = true,
            std::cmp::Ordering::Greater => {}
        }
    }
    Ok(if boundary {
        ContourPointLocation::Boundary
    } else {
        ContourPointLocation::Inside
    })
}

pub(super) fn classification_is_inside(
    classification: Classification<ContourPointLocation>,
) -> Result<bool, BuildError> {
    match classification {
        Classification::Decided(ContourPointLocation::Inside) => Ok(true),
        Classification::Decided(_) => Ok(false),
        Classification::Uncertain(reason) => Err(BuildError::Geometry(
            GeometryError::PlanarClassificationUnresolved(reason),
        )),
    }
}

pub(super) fn project_point_to_plane_frame(
    point: &Point3,
    origin: &Point3,
    u: &Vector3,
    v: &Vector3,
) -> Result<CurvePoint2, BuildError> {
    let displacement = point - origin;
    let uu = u.dot(u);
    let uv = u.dot(v);
    let vv = v.dot(v);
    let du = displacement.dot(u);
    let dv = displacement.dot(v);
    let determinant = &uu * &vv - &uv * &uv;
    Ok(CurvePoint2::new(
        ((&du * &vv - &dv * &uv) / &determinant).map_err(GeometryError::Numeric)?,
        ((&dv * &uu - &du * &uv) / determinant).map_err(GeometryError::Numeric)?,
    ))
}

pub(super) fn certified_affine_sweep_progress(
    path: &Curve3,
    normal: &Vector3,
) -> Result<bool, BuildError> {
    let Curve3ExactData::RationalBezier {
        control_points,
        weights,
    } = path.exact_data()
    else {
        return Ok(false);
    };
    let Some(degree) = control_points.len().checked_sub(1) else {
        return Ok(false);
    };
    let Some(elevated_degree) = degree.checked_add(1) else {
        return Ok(false);
    };
    let scalar = |point: &Point3| normal.dot(&Vector3::from(point.clone()));
    let start = scalar(&control_points[0]);
    let end = scalar(&control_points[degree]);
    if decided_model_order(compare_reals(
        &(&end - &start),
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
    {
        return Ok(false);
    }
    let numerators = control_points
        .iter()
        .zip(&weights)
        .map(|(control, weight)| weight * scalar(control))
        .collect::<Vec<_>>();
    for index in 0..=elevated_degree {
        let lower_count = Real::from(
            u128::try_from(elevated_degree - index).expect("usize is representable as u128"),
        );
        let upper_count =
            Real::from(u128::try_from(index).expect("usize is representable as u128"));
        let elevated_numerator = (if index <= degree {
            &lower_count * &numerators[index]
        } else {
            Real::zero()
        }) + if index > 0 {
            &upper_count * &numerators[index - 1]
        } else {
            Real::zero()
        };
        let affine_times_weight = (if index <= degree {
            &lower_count * &start * &weights[index]
        } else {
            Real::zero()
        }) + if index > 0 {
            &upper_count * &end * &weights[index - 1]
        } else {
            Real::zero()
        };
        if !real_values_equal(&elevated_numerator, &affine_times_weight)? {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn translated_rational_bezier_data_equal(
    actual_points: &[Point3],
    actual_weights: &[Real],
    expected_points: &[Point3],
    expected_weights: &[Real],
    offset: &Vector3,
) -> Result<bool, BuildError> {
    if actual_points.len() != expected_points.len()
        || actual_weights.len() != expected_weights.len()
        || actual_points.is_empty()
    {
        return Ok(false);
    }
    let weight_scale =
        (&actual_weights[0] / &expected_weights[0]).map_err(GeometryError::Numeric)?;
    for ((actual_point, actual_weight), (expected_point, expected_weight)) in actual_points
        .iter()
        .zip(actual_weights)
        .zip(expected_points.iter().zip(expected_weights))
    {
        if !points_equal(actual_point, &(expected_point.clone() + offset))?
            || !real_values_equal(actual_weight, &(expected_weight * &weight_scale))?
        {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn rational_bezier_weights_proportional(
    actual: &[Real],
    expected: &[Real],
) -> Result<bool, BuildError> {
    if actual.len() != expected.len() || actual.is_empty() {
        return Ok(false);
    }
    let scale = (&actual[0] / &expected[0]).map_err(GeometryError::Numeric)?;
    for (actual, expected) in actual.iter().zip(expected) {
        if !real_values_equal(actual, &(expected * &scale))? {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn translated_curve(curve: &Curve3, offset: &Vector3) -> Result<Curve3, BuildError> {
    match curve.exact_data() {
        Curve3ExactData::Line(data) => Ok(Curve3::line(data.start + offset, data.end + offset)?),
        Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } => Ok(Curve3::rational_bezier(
            control_points
                .into_iter()
                .map(|point| point + offset)
                .collect(),
            weights,
        )?),
        Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => Ok(Curve3::nurbs(
            degree,
            control_points
                .into_iter()
                .map(|point| point + offset)
                .collect(),
            weights,
            knots,
        )?),
        Curve3ExactData::EllipseArc(mut data) => {
            data.center = data.center + offset;
            Ok(Curve3::from_exact_data(Curve3ExactData::EllipseArc(data))?)
        }
    }
}

pub(super) fn extrusion_constant_area_scale(
    profile: &Curve3,
    direction: &Vector3,
) -> Result<Option<Real>, BuildError> {
    let constant = match profile.exact_data() {
        Curve3ExactData::Line(_) => true,
        Curve3ExactData::EllipseArc(data) if data.circle => {
            let normal = data.x.cross(&data.y);
            real_values_equal(&direction.cross(&normal).norm_squared(), &Real::zero())?
        }
        _ => false,
    };
    if !constant {
        return Ok(None);
    }
    Ok(Some(
        profile
            .derivative_at(profile.domain().start(), 1)?
            .vector()
            .cross(direction)
            .norm_squared()
            .sqrt()
            .map_err(GeometryError::Numeric)?,
    ))
}

pub(super) fn certified_monotone_line_curve_image(curve: &Curve3) -> Result<bool, BuildError> {
    let control_points = match curve.exact_data() {
        Curve3ExactData::RationalBezier { control_points, .. }
        | Curve3ExactData::Nurbs { control_points, .. } => control_points,
        _ => return Ok(false),
    };
    let start = curve.evaluate_real(curve.domain().start())?;
    let end = curve.evaluate_real(curve.domain().end())?;
    if !points_equal(
        control_points
            .first()
            .expect("validated spline has control points"),
        &start,
    )? || !points_equal(
        control_points
            .last()
            .expect("validated spline has control points"),
        &end,
    )? {
        return Ok(false);
    }
    let direction = &end - &start;
    let length_squared = direction.norm_squared();
    if decided_model_order(compare_reals(
        &length_squared,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
    {
        return Ok(false);
    }
    let mut previous_projection = Real::zero();
    for control in control_points {
        let relative = control - &start;
        if decided_model_order(compare_reals(
            &relative.cross(&direction).norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(false);
        }
        let projection = relative.dot(&direction);
        if decided_model_order(compare_reals(
            &projection,
            &previous_projection,
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &projection,
                &length_squared,
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Greater
        {
            return Ok(false);
        }
        previous_projection = projection;
    }
    Ok(true)
}

/// Proves that `|C'(u) × direction|` integrates to
/// `|(C(end) - C(start)) × direction|`.
///
/// Coplanar controls keep every cross product on one fixed normal. Positive
/// rational Bézier and NURBS bases are normalized totally-positive bases, so
/// nondecreasing transverse control projections certify a nonnegative
/// derivative over the complete native domain without sampling.
pub(super) fn certified_monotone_planar_extrusion_image(
    curve: &Curve3,
    direction: &Vector3,
) -> Result<bool, BuildError> {
    let control_points = match curve.exact_data() {
        Curve3ExactData::RationalBezier { control_points, .. }
        | Curve3ExactData::Nurbs { control_points, .. } => control_points,
        _ => return Ok(false),
    };
    let start = curve.evaluate_real(curve.domain().start())?;
    let end = curve.evaluate_real(curve.domain().end())?;
    if !points_equal(
        control_points
            .first()
            .expect("validated spline has control points"),
        &start,
    )? || !points_equal(
        control_points
            .last()
            .expect("validated spline has control points"),
        &end,
    )? {
        return Ok(false);
    }
    let chord = &end - &start;
    let normal = chord.cross(direction);
    if decided_model_order(compare_reals(
        &normal.norm_squared(),
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
    {
        return Ok(false);
    }
    let transverse = direction.cross(&normal);
    let end_projection = chord.dot(&transverse);
    if decided_model_order(compare_reals(
        &end_projection,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
    {
        return Ok(false);
    }
    let mut previous_projection = Real::zero();
    for control in control_points {
        let relative = control - &start;
        if !real_values_equal(&relative.dot(&normal), &Real::zero())? {
            return Ok(false);
        }
        let projection = relative.dot(&transverse);
        if decided_model_order(compare_reals(
            &projection,
            &previous_projection,
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &projection,
                &end_projection,
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Greater
        {
            return Ok(false);
        }
        previous_projection = projection;
    }
    Ok(true)
}

pub(super) struct AffineTensorImage {
    pub(super) spatial_area: Real,
    pub(super) parameter_area: Real,
    pub(super) constant_weights: bool,
    pub(super) separable_weights: bool,
}

pub(super) fn affine_tensor_face_area(
    data: &SurfaceExactData,
    parameter_double_area: &Real,
) -> Result<Option<Real>, BuildError> {
    let Some(image) = affine_tensor_image(data)? else {
        return Ok(None);
    };
    if image.constant_weights {
        let double_area = (parameter_double_area * image.spatial_area / image.parameter_area)
            .map_err(GeometryError::Numeric)?;
        return Ok(Some(
            (double_area / Real::from(2)).map_err(GeometryError::Numeric)?,
        ));
    }
    if !image.separable_weights
        || !real_values_equal(
            parameter_double_area,
            &(Real::from(2) * &image.parameter_area),
        )?
    {
        return Ok(None);
    }
    Ok(Some(image.spatial_area))
}

pub(super) fn affine_tensor_image(
    data: &SurfaceExactData,
) -> Result<Option<AffineTensorImage>, BuildError> {
    match data {
        SurfaceExactData::RationalBezier {
            control_points,
            weights,
        } => {
            let u_count = control_points[0].len();
            let v_count = control_points.len();
            let u_denominator =
                Real::from(u128::try_from(u_count - 1).expect("usize is representable as u128"));
            let v_denominator =
                Real::from(u128::try_from(v_count - 1).expect("usize is representable as u128"));
            let u_coordinates = (0..u_count)
                .map(|index| {
                    (Real::from(u128::try_from(index).expect("usize is representable as u128"))
                        / &u_denominator)
                        .map_err(|problem| GeometryError::Numeric(problem).into())
                })
                .collect::<Result<Vec<_>, BuildError>>()?;
            let v_coordinates = (0..v_count)
                .map(|index| {
                    (Real::from(u128::try_from(index).expect("usize is representable as u128"))
                        / &v_denominator)
                        .map_err(|problem| GeometryError::Numeric(problem).into())
                })
                .collect::<Result<Vec<_>, BuildError>>()?;
            affine_control_net_image(
                control_points,
                &u_coordinates,
                &v_coordinates,
                &Real::one(),
                &Real::one(),
                tensor_weights_are_constant(weights)?,
                tensor_weights_are_separable(weights)?,
            )
        }
        SurfaceExactData::Nurbs {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
        } => {
            let u_count = control_points[0].len();
            let v_count = control_points.len();
            let u_start = &u_knots[*u_degree];
            let u_end = &u_knots[u_count];
            let v_start = &v_knots[*v_degree];
            let v_end = &v_knots[v_count];
            let u_span = u_end - u_start;
            let v_span = v_end - v_start;
            let u_coordinates =
                normalized_greville_coordinates(u_count, *u_degree, u_knots, u_start, &u_span)?;
            let v_coordinates =
                normalized_greville_coordinates(v_count, *v_degree, v_knots, v_start, &v_span)?;
            affine_control_net_image(
                control_points,
                &u_coordinates,
                &v_coordinates,
                &u_span,
                &v_span,
                tensor_weights_are_constant(weights)?,
                tensor_weights_are_separable(weights)?,
            )
        }
        _ => Ok(None),
    }
}

pub(super) fn tensor_weights_are_constant(weights: &[Vec<Real>]) -> Result<bool, BuildError> {
    let Some(first) = weights.first().and_then(|row| row.first()) else {
        return Ok(false);
    };
    for weight in weights.iter().flatten().skip(1) {
        if !real_values_equal(weight, first)? {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn tensor_weights_are_separable(weights: &[Vec<Real>]) -> Result<bool, BuildError> {
    let Some(first_row) = weights.first() else {
        return Ok(false);
    };
    let Some(first) = first_row.first() else {
        return Ok(false);
    };
    if weights.iter().any(|row| row.len() != first_row.len()) {
        return Ok(false);
    }
    // These cross-products prove that the positive weight matrix has rank one
    // without choosing a potentially expression-expanding quotient.
    for row in weights.iter().skip(1) {
        for (weight, first_row_weight) in row.iter().zip(first_row) {
            if !real_values_equal(&(weight * first), &(&row[0] * first_row_weight))? {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

pub(super) fn normalized_greville_coordinates(
    count: usize,
    degree: usize,
    knots: &[Real],
    domain_start: &Real,
    domain_span: &Real,
) -> Result<Vec<Real>, BuildError> {
    let degree_scalar = Real::from(u128::try_from(degree).expect("usize is representable as u128"));
    (0..count)
        .map(|index| {
            let sum = knots[index + 1..=index + degree]
                .iter()
                .fold(Real::zero(), |sum, knot| sum + knot);
            let greville = (sum / &degree_scalar).map_err(GeometryError::Numeric)?;
            ((greville - domain_start) / domain_span)
                .map_err(|problem| GeometryError::Numeric(problem).into())
        })
        .collect()
}

pub(super) fn affine_control_net_image(
    control_points: &[Vec<Point3>],
    u_coordinates: &[Real],
    v_coordinates: &[Real],
    u_parameter_span: &Real,
    v_parameter_span: &Real,
    constant_weights: bool,
    separable_weights: bool,
) -> Result<Option<AffineTensorImage>, BuildError> {
    let origin = &control_points[0][0];
    let u = &control_points[0][control_points[0].len() - 1] - origin;
    let v = &control_points[control_points.len() - 1][0] - origin;
    let cross_squared = u.cross(&v).norm_squared();
    if decided_model_order(compare_reals(
        &cross_squared,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
    {
        return Ok(None);
    }
    for (row, v_coordinate) in control_points.iter().zip(v_coordinates) {
        for (point, u_coordinate) in row.iter().zip(u_coordinates) {
            let expected = origin.clone() + u.clone() * u_coordinate + v.clone() * v_coordinate;
            if !points_equal(point, &expected)? {
                return Ok(None);
            }
        }
    }
    let spatial_area = cross_squared.sqrt().map_err(GeometryError::Numeric)?;
    Ok(Some(AffineTensorImage {
        spatial_area,
        parameter_area: u_parameter_span * v_parameter_span,
        constant_weights,
        separable_weights,
    }))
}

pub(super) fn divide_vector_by_real(
    vector: Vector3,
    denominator: &Real,
) -> Result<Vector3, BuildError> {
    Ok(Vector3::from_xyz(
        (vector.0[0].clone() / denominator).map_err(GeometryError::Numeric)?,
        (vector.0[1].clone() / denominator).map_err(GeometryError::Numeric)?,
        (vector.0[2].clone() / denominator).map_err(GeometryError::Numeric)?,
    ))
}

pub(super) fn certified_sweep_frame_area_integral(
    u_controls: &[Vector3],
    v_controls: &[Vector3],
    weights: &[Real],
    normal: &Vector3,
) -> Result<Option<Real>, BuildError> {
    if u_controls.len() != v_controls.len()
        || u_controls.len() != weights.len()
        || u_controls.is_empty()
    {
        return Ok(None);
    }
    for axis in u_controls.iter().chain(v_controls) {
        if !real_values_equal(&axis.dot(normal), &Real::zero())? {
            return Ok(None);
        }
    }
    let degree = u_controls.len() - 1;
    let Some(product_degree) = degree.checked_mul(2) else {
        return Ok(None);
    };
    let normal_squared = normal.norm_squared();
    let mut constant_area = true;
    let mut determinant_numerators = Vec::with_capacity(product_degree + 1);
    for product_index in 0..=product_degree {
        let first_min = product_index.saturating_sub(degree);
        let first_max = degree.min(product_index);
        let mut cross_coefficient = Vector3::zero();
        let mut weight_coefficient = Real::zero();
        for first in first_min..=first_max {
            let second = product_index - first;
            let coefficient = model_bernstein_product_coefficient(degree, first, second)?;
            let weighted_u = u_controls[first].clone() * &weights[first];
            let weighted_v = v_controls[second].clone() * &weights[second];
            cross_coefficient = cross_coefficient + weighted_u.cross(&weighted_v) * &coefficient;
            weight_coefficient += &coefficient * &weights[first] * &weights[second];
        }
        let expected = normal.clone() * weight_coefficient;
        constant_area &= vectors_equal(&cross_coefficient, &expected)?;
        let determinant_numerator =
            (cross_coefficient.dot(normal) / &normal_squared).map_err(GeometryError::Numeric)?;
        if !vectors_equal(
            &cross_coefficient,
            &(normal.clone() * &determinant_numerator),
        )? {
            return Ok(None);
        }
        determinant_numerators.push(determinant_numerator);
    }
    if constant_area {
        return Ok(Some(Real::one()));
    }
    for weight in &weights[1..] {
        if !real_values_equal(weight, &weights[0])? {
            return Ok(None);
        }
    }
    let weight_squared = &weights[0] * &weights[0];
    let mut integral = Real::zero();
    for numerator in determinant_numerators {
        let coefficient = (numerator / &weight_squared).map_err(GeometryError::Numeric)?;
        if decided_model_order(compare_reals(
            &coefficient,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        integral += coefficient;
    }
    Ok(Some(
        (integral
            / Real::from(
                u128::try_from(product_degree + 1).expect("usize is representable as u128"),
            ))
        .map_err(GeometryError::Numeric)?,
    ))
}

pub(super) fn model_bernstein_product_coefficient(
    degree: usize,
    first: usize,
    second: usize,
) -> Result<Real, BuildError> {
    let product_degree = degree
        .checked_mul(2)
        .ok_or(GeometryError::Numeric(hyperreal::Problem::OutOfRange))?;
    let numerator = model_binomial_real(degree, first)? * model_binomial_real(degree, second)?;
    (numerator / model_binomial_real(product_degree, first + second)?)
        .map_err(|problem| GeometryError::Numeric(problem).into())
}

pub(super) fn model_binomial_real(n: usize, k: usize) -> Result<Real, BuildError> {
    let k = k.min(n - k);
    let mut result = Real::one();
    for index in 1..=k {
        result = (result
            * Real::from(u128::try_from(n + 1 - index).expect("usize is representable as u128"))
            / Real::from(u128::try_from(index).expect("usize is representable as u128")))
        .map_err(GeometryError::Numeric)?;
    }
    Ok(result)
}

pub(super) fn vector_control_curve(
    controls: Vec<Vector3>,
    weights: &[Real],
) -> Result<Curve3, GeometryError> {
    Curve3::rational_bezier(
        controls.into_iter().map(Point3::from).collect(),
        weights.to_vec(),
    )
}

pub(super) fn constant_vector_curve(
    vector: &Vector3,
    weights: &[Real],
) -> Result<Curve3, GeometryError> {
    vector_control_curve(vec![vector.clone(); weights.len()], weights)
}

pub(super) fn transform_vector_curve(
    curve: &Curve3,
    transform: &Matrix4,
) -> Result<Curve3, GeometryError> {
    let Curve3ExactData::RationalBezier {
        control_points,
        weights,
    } = curve.exact_data()
    else {
        return Err(GeometryError::UnsupportedTransform);
    };
    Curve3::rational_bezier(
        control_points
            .into_iter()
            .map(|point| Point3::from(transform.transform_direction3(&Vector3::from(point))))
            .collect(),
        weights,
    )
}

pub(super) fn translated_rational_bezier_surface_equal(
    actual_points: &[Vec<Point3>],
    actual_weights: &[Vec<Real>],
    path_points: &[Point3],
    path_weights: &[Real],
    start_offset: &Vector3,
    end_offset: &Vector3,
) -> Result<bool, BuildError> {
    if actual_points.len() != path_points.len()
        || actual_weights.len() != path_weights.len()
        || path_points.is_empty()
        || actual_points.iter().any(|row| row.len() != 2)
        || actual_weights.iter().any(|row| row.len() != 2)
    {
        return Ok(false);
    }
    for reverse_path in [false, true] {
        for swap_profile in [false, true] {
            let path_index = |row: usize| {
                if reverse_path {
                    path_points.len() - 1 - row
                } else {
                    row
                }
            };
            let first_path_index = path_index(0);
            let weight_scale = (&actual_weights[0][0] / &path_weights[first_path_index])
                .map_err(GeometryError::Numeric)?;
            let mut equal = true;
            for row in 0..path_points.len() {
                let expected_row = path_index(row);
                for column in 0..2 {
                    let expected_column = if swap_profile { 1 - column } else { column };
                    let offset = if expected_column == 0 {
                        start_offset
                    } else {
                        end_offset
                    };
                    if !points_equal(
                        &actual_points[row][column],
                        &(path_points[expected_row].clone() + offset),
                    )? || !real_values_equal(
                        &actual_weights[row][column],
                        &(&path_weights[expected_row] * &weight_scale),
                    )? {
                        equal = false;
                        break;
                    }
                }
                if !equal {
                    break;
                }
            }
            if equal {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

#[derive(Clone, Copy)]
pub(super) struct RationalBezierSweepControlView<'a> {
    pub(super) path_points: &'a [Point3],
    pub(super) path_weights: &'a [Real],
    pub(super) u_controls: &'a [Vector3],
    pub(super) v_controls: &'a [Vector3],
}

pub(super) fn framed_rational_bezier_surface_equal(
    actual_points: &[Vec<Point3>],
    actual_weights: &[Vec<Real>],
    frame: RationalBezierSweepControlView<'_>,
    start: &CurvePoint2,
    end: &CurvePoint2,
) -> Result<bool, BuildError> {
    let RationalBezierSweepControlView {
        path_points,
        path_weights,
        u_controls,
        v_controls,
    } = frame;
    if actual_points.len() != path_points.len()
        || actual_weights.len() != path_weights.len()
        || u_controls.len() != path_points.len()
        || v_controls.len() != path_points.len()
        || path_points.is_empty()
        || actual_points.iter().any(|row| row.len() != 2)
        || actual_weights.iter().any(|row| row.len() != 2)
    {
        return Ok(false);
    }
    for reverse_path in [false, true] {
        for swap_profile in [false, true] {
            let path_index = |row: usize| {
                if reverse_path {
                    path_points.len() - 1 - row
                } else {
                    row
                }
            };
            let first_path_index = path_index(0);
            let weight_scale = (&actual_weights[0][0] / &path_weights[first_path_index])
                .map_err(GeometryError::Numeric)?;
            let mut equal = true;
            for row in 0..path_points.len() {
                let expected_row = path_index(row);
                for column in 0..2 {
                    let coordinate = if (column == 0) ^ swap_profile {
                        start
                    } else {
                        end
                    };
                    let expected = path_points[expected_row].clone()
                        + u_controls[expected_row].clone() * coordinate.x()
                        + v_controls[expected_row].clone() * coordinate.y();
                    if !points_equal(&actual_points[row][column], &expected)?
                        || !real_values_equal(
                            &actual_weights[row][column],
                            &(&path_weights[expected_row] * &weight_scale),
                        )?
                    {
                        equal = false;
                        break;
                    }
                }
                if !equal {
                    break;
                }
            }
            if equal {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

pub(super) fn project_point_to_surface_plane(
    point: &Point3,
    surface: &Surface,
) -> Result<CurvePoint2, BuildError> {
    let origin = surface
        .plane_origin()
        .expect("planar certificate prevalidates plane surfaces");
    let (u, v) = surface
        .plane_directions()
        .expect("planar certificate prevalidates plane surfaces");
    project_point_to_plane_frame(point, origin, u, v)
}

pub(super) fn planar_surface_value(surface: &Surface, point: &Point3) -> Real {
    let origin = surface
        .plane_origin()
        .expect("planar certificate prevalidates plane surfaces");
    let (u, v) = surface
        .plane_directions()
        .expect("planar certificate prevalidates plane surfaces");
    u.cross(v).dot(&(point - origin))
}

pub(super) fn line_parameter(
    candidate: &Point3,
    point: &Point3,
    direction: &Vector3,
) -> Result<Real, BuildError> {
    ((candidate - point).dot(direction) / direction.norm_squared())
        .map_err(GeometryError::Numeric)
        .map_err(BuildError::from)
}

pub(super) fn minimum_real(first: &Real, second: &Real) -> Result<Real, BuildError> {
    Ok(
        if decided_model_order(compare_reals(first, second, crate::STRICT_PREDICATES))?
            == std::cmp::Ordering::Greater
        {
            second.clone()
        } else {
            first.clone()
        },
    )
}

pub(super) fn maximum_real(first: &Real, second: &Real) -> Result<Real, BuildError> {
    Ok(
        if decided_model_order(compare_reals(first, second, crate::STRICT_PREDICATES))?
            == std::cmp::Ordering::Less
        {
            second.clone()
        } else {
            first.clone()
        },
    )
}

pub(super) fn parameter_in_line_material(
    parameter: &Real,
    intervals: &[(Real, Real)],
    contacts: &[Real],
) -> Result<bool, BuildError> {
    for interval in intervals {
        if decided_model_order(compare_reals(
            parameter,
            &interval.0,
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Less
            && decided_model_order(compare_reals(
                parameter,
                &interval.1,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
        {
            return Ok(true);
        }
    }
    for contact in contacts {
        if decided_model_order(compare_reals(parameter, contact, crate::STRICT_PREDICATES))?
            == std::cmp::Ordering::Equal
        {
            return Ok(true);
        }
    }
    Ok(false)
}

pub(super) fn vectors_equal(left: &Vector3, right: &Vector3) -> Result<bool, BuildError> {
    for axis in 0..3 {
        if (&left.0[axis] - &right.0[axis]).zero_status() == hyperreal::ZeroKnowledge::Zero {
            continue;
        }
        match compare_reals(&left.0[axis], &right.0[axis], crate::STRICT_PREDICATES) {
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Equal,
                ..
            } => {}
            PredicateOutcome::Decided { .. } => return Ok(false),
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                }));
            }
        }
    }
    Ok(true)
}

pub(super) fn point3_component(point: &Point3, axis: usize) -> &Real {
    match axis {
        0 => &point.x,
        1 => &point.y,
        2 => &point.z,
        _ => unreachable!("three-dimensional component index"),
    }
}

pub(super) fn points_equal(left: &Point3, right: &Point3) -> Result<bool, BuildError> {
    match point3_equal(left, right, crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                needed,
                stage,
            }))
        }
    }
}

pub(super) fn curve_points_equal(
    left: &CurvePoint2,
    right: &CurvePoint2,
) -> Result<bool, BuildError> {
    Ok(real_values_equal(left.x(), right.x())? && real_values_equal(left.y(), right.y())?)
}

pub(super) fn validate_rational_pcurve_controls(
    actual: &RationalBezier2,
    expected_points: &[CurvePoint2],
    expected_weights: &[Real],
) -> Result<(), BuildError> {
    if expected_points.len() != expected_weights.len() {
        return Err(BuildError::EdgeUseSupportMismatch);
    }
    let expected: Vec<_> = expected_points
        .iter()
        .zip(expected_weights)
        .map(|(point, weight)| hypercurve::HomogeneousControl2::from_affine(point, weight.clone()))
        .collect();
    validate_homogeneous_pcurve_controls(actual.homogeneous_controls(), &expected)
}

pub(super) fn validate_homogeneous_pcurve_controls(
    actual: &[hypercurve::HomogeneousControl2],
    expected: &[hypercurve::HomogeneousControl2],
) -> Result<(), BuildError> {
    if actual.len() != expected.len() || actual.is_empty() {
        return Err(BuildError::EdgeUseSupportMismatch);
    }
    // Unclamped NURBS may begin with an infinite or zero control. Choose a
    // nonzero coefficient of the full net, without requiring affine projection.
    let mut gauge = None;
    'controls: for (a, b) in actual.iter().zip(expected) {
        for (a, b) in [(a.weight(), b.weight()), (a.x(), b.x()), (a.y(), b.y())] {
            if !real_values_equal(a, &Real::zero())? {
                if real_values_equal(b, &Real::zero())? {
                    return Err(BuildError::EdgeUseSupportMismatch);
                }
                gauge = Some((a, b));
                break 'controls;
            }
        }
    }
    let (actual_weight, expected_weight) = gauge.ok_or(BuildError::EdgeUseSupportMismatch)?;
    for (a, b) in actual.iter().zip(expected) {
        for (a, b) in [(a.x(), b.x()), (a.y(), b.y()), (a.weight(), b.weight())] {
            if !real_values_equal(&(a * expected_weight), &(b * actual_weight))? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
        }
    }
    Ok(())
}

pub(super) fn curve2_from_segment(segment: Segment2) -> Curve2 {
    match segment {
        Segment2::Line(line) => Curve2::from(line),
        Segment2::Arc(arc) => Curve2::from(arc),
    }
}

pub(super) fn planar_rational_bezier_curve(
    control_points: Vec<CurvePoint2>,
    weights: Vec<Real>,
) -> Result<Curve2, BuildError> {
    if control_points.len() != weights.len() || control_points.len() < 2 {
        return Err(GeometryError::PlanarCurveConstruction(
            hypercurve::CurveError::InvalidRationalBezier,
        )
        .into());
    }
    let uniform = weights.iter().all(|weight| weight == &weights[0]);
    match (uniform, control_points.as_slice(), weights.as_slice()) {
        (true, [start, control, end], _) => Ok(Curve2::from(QuadraticBezier2::new(
            start.clone(),
            control.clone(),
            end.clone(),
        ))),
        (true, [start, first, second, end], _) => Ok(Curve2::from(CubicBezier2::new(
            start.clone(),
            first.clone(),
            second.clone(),
            end.clone(),
        ))),
        (false, [start, control, end], [start_weight, control_weight, end_weight]) => {
            Ok(Curve2::from(
                RationalQuadraticBezier2::try_new(
                    start.clone(),
                    control.clone(),
                    end.clone(),
                    start_weight.clone(),
                    control_weight.clone(),
                    end_weight.clone(),
                )
                .map_err(GeometryError::from)?,
            ))
        }
        _ => Ok(Curve2::from(
            RationalBezier2::try_new(control_points, weights).map_err(GeometryError::from)?,
        )),
    }
}

pub(super) fn normalize_single_span_nurbs_pcurve(
    curve: &Curve2,
    u_start: &Real,
    u_end: &Real,
    v_start: &Real,
    v_end: &Real,
) -> Result<Curve2, BuildError> {
    let Some(CurveGeometry2::RationalBezier(curve)) = curve.geometry() else {
        return Err(BuildError::EdgeUseSupportMismatch);
    };
    let u_span = u_end - u_start;
    let v_span = v_end - v_start;
    let controls = curve
        .homogeneous_controls()
        .iter()
        .map(|point| {
            Ok(hypercurve::HomogeneousControl2::new(
                ((point.x() - u_start * point.weight()) / &u_span)
                    .map_err(GeometryError::Numeric)?,
                ((point.y() - v_start * point.weight()) / &v_span)
                    .map_err(GeometryError::Numeric)?,
                point.weight().clone(),
            ))
        })
        .collect::<Result<Vec<_>, BuildError>>()?;
    let Classification::Decided(curve) =
        crate::error::classified(RationalBezier2::from_homogeneous_controls(controls))
            .map_err(GeometryError::from)?
    else {
        return Err(BuildError::EdgeUseSupportMismatch);
    };
    Ok(Curve2::from(curve))
}

pub(super) fn validate_projective_pcurve_equal(
    actual: &Curve2,
    expected: &Curve2,
) -> Result<(), BuildError> {
    match (actual.geometry(), expected.geometry()) {
        (
            Some(CurveGeometry2::RationalBezier(actual)),
            Some(CurveGeometry2::RationalBezier(expected)),
        ) => validate_homogeneous_pcurve_controls(
            actual.homogeneous_controls(),
            expected.homogeneous_controls(),
        ),
        (Some(CurveGeometry2::Nurbs(actual)), Some(CurveGeometry2::Nurbs(expected))) => {
            if actual.degree() != expected.degree()
                || !real_slices_equal(actual.knots(), expected.knots())?
            {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
            validate_homogeneous_pcurve_controls(
                actual.homogeneous_controls(),
                expected.homogeneous_controls(),
            )
        }
        _ => Err(BuildError::EdgeUseSupportMismatch),
    }
}

pub(super) fn real_slices_equal(left: &[Real], right: &[Real]) -> Result<bool, BuildError> {
    if left.len() != right.len() {
        return Ok(false);
    }
    for (left, right) in left.iter().zip(right) {
        if !real_values_equal(left, right)? {
            return Ok(false);
        }
    }
    Ok(true)
}

type TensorControlNet = (Vec<Vec<Point3>>, Vec<Vec<Real>>);

pub(super) struct TrimmedRationalTensorProfile {
    pub(super) points: Vec<Vec<Point3>>,
    pub(super) weights: Vec<Vec<Real>>,
    pub(super) start: Real,
    pub(super) end: Real,
}

pub(super) struct TrimmedNurbsTensorProfile {
    pub(super) points: Vec<Vec<Point3>>,
    pub(super) weights: Vec<Vec<Real>>,
    pub(super) knots: Vec<Real>,
}

pub(super) fn translation_tensor_rows(
    control_points: &[Point3],
    direction: &Vector3,
) -> Vec<Vec<Point3>> {
    control_points
        .iter()
        .map(|point| vec![point.clone(), point.clone() + direction])
        .collect()
}

pub(super) fn duplicated_tensor_weights(weights: &[Real]) -> Vec<Vec<Real>> {
    weights
        .iter()
        .map(|weight| vec![weight.clone(), weight.clone()])
        .collect()
}

pub(super) fn transpose_two_row_tensor(
    points: &[Vec<Point3>],
    weights: &[Vec<Real>],
) -> Option<TensorControlNet> {
    if points.len() != 2
        || weights.len() != 2
        || points[0].is_empty()
        || points[0].len() != points[1].len()
        || weights[0].len() != points[0].len()
        || weights[1].len() != points[0].len()
    {
        return None;
    }
    let mut transposed_points = Vec::with_capacity(points[0].len());
    let mut transposed_weights = Vec::with_capacity(points[0].len());
    for index in 0..points[0].len() {
        transposed_points.push(vec![points[0][index].clone(), points[1][index].clone()]);
        transposed_weights.push(vec![weights[0][index].clone(), weights[1][index].clone()]);
    }
    Some((transposed_points, transposed_weights))
}

pub(super) fn rational_tensor_graph_controls(
    weights: &[Real],
    coefficients: &[Real],
    parameter_start: &Real,
    parameter_span: &Real,
    profile_axis: SurfaceIsoAxis,
) -> Result<(Vec<CurvePoint2>, Vec<Real>), BuildError> {
    if weights.len() < 2 || weights.len() != coefficients.len() {
        return Err(BuildError::EdgeUseSupportMismatch);
    }
    let control_count = weights.len();
    let denominator =
        Real::from(u128::try_from(control_count).map_err(|_| GeometryError::InvalidDegree)?);
    let coefficient_homogeneous = weights
        .iter()
        .zip(coefficients)
        .map(|(weight, coefficient)| weight * coefficient)
        .collect::<Vec<_>>();
    let mut elevated_points = Vec::with_capacity(control_count + 1);
    let mut elevated_weights = Vec::with_capacity(control_count + 1);
    for index in 0..=control_count {
        let alpha = (Real::from(u128::try_from(index).map_err(|_| GeometryError::InvalidDegree)?)
            / &denominator)
            .map_err(GeometryError::Numeric)?;
        let one_minus_alpha = Real::one() - &alpha;
        let previous_weight = (index > 0).then(|| weights[index - 1].clone());
        let next_weight = (index < control_count).then(|| weights[index].clone());
        let elevated_weight = previous_weight
            .as_ref()
            .map_or_else(Real::zero, |weight| &alpha * weight)
            + next_weight
                .as_ref()
                .map_or_else(Real::zero, |weight| &one_minus_alpha * weight);
        let local_parameter_homogeneous = previous_weight
            .as_ref()
            .map_or_else(Real::zero, |weight| &alpha * weight);
        let parameter_homogeneous =
            parameter_start * &elevated_weight + parameter_span * local_parameter_homogeneous;
        let previous_coefficient = if index > 0 {
            &alpha * &coefficient_homogeneous[index - 1]
        } else {
            Real::zero()
        };
        let next_coefficient = if index < control_count {
            &one_minus_alpha * &coefficient_homogeneous[index]
        } else {
            Real::zero()
        };
        let coefficient = ((previous_coefficient + next_coefficient) / &elevated_weight)
            .map_err(GeometryError::Numeric)?;
        let parameter =
            (parameter_homogeneous / &elevated_weight).map_err(GeometryError::Numeric)?;
        elevated_points.push(match profile_axis {
            SurfaceIsoAxis::U => CurvePoint2::new(parameter, coefficient),
            SurfaceIsoAxis::V => CurvePoint2::new(coefficient, parameter),
        });
        elevated_weights.push(elevated_weight);
    }
    Ok((elevated_points, elevated_weights))
}

pub(super) fn build_error_geometry(error: BuildError) -> GeometryError {
    match error {
        BuildError::Geometry(error) => error,
        _ => unreachable!("exact scalar comparison only returns geometry errors"),
    }
}

pub(super) fn tensor_curve_images_equal(
    actual: &Curve3,
    expected: &Curve3,
) -> Result<bool, BuildError> {
    if let Curve3ExactData::Line(line) = actual.exact_data() {
        let (points, weights, degree) = match expected.exact_data() {
            Curve3ExactData::RationalBezier {
                control_points,
                weights,
            } => (control_points, weights, 1),
            Curve3ExactData::Nurbs {
                degree,
                control_points,
                weights,
                ..
            } => (control_points, weights, degree),
            _ => return Ok(false),
        };
        if degree != 1 || points.len() != 2 || weights.len() != 2 {
            return Ok(false);
        }
        return Ok(real_values_equal(&weights[0], &weights[1])?
            && points_equal(&line.start, &points[0])?
            && points_equal(&line.end, &points[1])?);
    }
    let unpack = |curve: &Curve3| match curve.exact_data() {
        Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } => Some((control_points, weights, None, None)),
        Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => Some((control_points, weights, Some(degree), Some(knots))),
        _ => None,
    };
    let Some((actual_points, actual_weights, actual_degree, actual_knots)) = unpack(actual) else {
        return Ok(false);
    };
    let Some((expected_points, expected_weights, expected_degree, expected_knots)) =
        unpack(expected)
    else {
        return Ok(false);
    };
    if actual_degree != expected_degree
        || actual_points.len() != expected_points.len()
        || actual_weights.len() != expected_weights.len()
    {
        return Ok(false);
    }
    match (actual_knots.as_ref(), expected_knots.as_ref()) {
        (Some(actual), Some(expected)) if actual.len() == expected.len() => {
            for (actual, expected) in actual.iter().zip(expected) {
                if !real_values_equal(actual, expected)? {
                    return Ok(false);
                }
            }
        }
        (None, None) => {}
        _ => return Ok(false),
    }
    for (actual, expected) in actual_points.iter().zip(&expected_points) {
        if !points_equal(actual, expected)? {
            return Ok(false);
        }
    }
    let weight_scale =
        (&actual_weights[0] / &expected_weights[0]).map_err(GeometryError::Numeric)?;
    for (actual, expected) in actual_weights.iter().zip(&expected_weights) {
        if !real_values_equal(actual, &(expected * &weight_scale))? {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn curve_parameterizations_equal(
    actual: &Curve3,
    expected: &Curve3,
) -> Result<bool, BuildError> {
    if actual.kind() != expected.kind()
        || !real_values_equal(actual.domain().start(), expected.domain().start())?
        || !real_values_equal(actual.domain().end(), expected.domain().end())?
    {
        return Ok(false);
    }
    match (actual.exact_data(), expected.exact_data()) {
        (Curve3ExactData::Line(actual), Curve3ExactData::Line(expected)) => {
            Ok(points_equal(&actual.start, &expected.start)?
                && points_equal(&actual.end, &expected.end)?)
        }
        (
            Curve3ExactData::RationalBezier { .. } | Curve3ExactData::Nurbs { .. },
            Curve3ExactData::RationalBezier { .. } | Curve3ExactData::Nurbs { .. },
        ) => tensor_curve_images_equal(actual, expected),
        (Curve3ExactData::EllipseArc(actual), Curve3ExactData::EllipseArc(expected)) => {
            Ok(actual.circle == expected.circle
                && actual.direction == expected.direction
                && points_equal(&actual.center, &expected.center)?
                && vectors_equal(&actual.x, &expected.x)?
                && vectors_equal(&actual.y, &expected.y)?
                && real_values_equal(&actual.x_radius, &expected.x_radius)?
                && real_values_equal(&actual.y_radius, &expected.y_radius)?
                && real_values_equal(&actual.angle_at_start, &expected.angle_at_start)?)
        }
        _ => Ok(false),
    }
}

pub(super) fn circle_parameterizations_equal(
    actual: &Curve3,
    expected: &Curve3,
) -> Result<bool, BuildError> {
    let Curve3ExactData::EllipseArc(actual) = actual.exact_data() else {
        return Ok(false);
    };
    let Curve3ExactData::EllipseArc(expected) = expected.exact_data() else {
        return Ok(false);
    };
    circle_data_parameterizations_equal(&actual, &expected)
}

pub(super) fn circle_data_parameterizations_equal(
    actual: &EllipseArcExactData,
    expected: &EllipseArcExactData,
) -> Result<bool, BuildError> {
    if !actual.circle
        || !expected.circle
        || !real_values_equal(&actual.domain_start, &expected.domain_start)?
        || !real_values_equal(&actual.domain_end, &expected.domain_end)?
        || !points_equal(&actual.center, &expected.center)?
        || !real_values_equal(&actual.x_radius, &expected.x_radius)?
        || !real_values_equal(&actual.y_radius, &expected.y_radius)?
    {
        return Ok(false);
    }

    // Put both arcs in the unique direct-domain form
    // `center + radius * (x*cos(t) + y*sin(t))`. This compares the complete
    // analytic parameterizations while allowing reflection to reverse the
    // surface's angular chart without rewriting the spatial edge domain.
    let direct_frame = |arc: &EllipseArcExactData| {
        let phase = if arc.direction > 0 {
            &arc.angle_at_start - &arc.domain_start
        } else {
            &arc.angle_at_start + &arc.domain_start
        };
        let sine = phase.clone().sin();
        let cosine = phase.cos();
        let x = arc.x.clone() * &cosine + arc.y.clone() * &sine;
        let y = if arc.direction > 0 {
            -arc.x.clone() * sine + arc.y.clone() * cosine
        } else {
            arc.x.clone() * sine - arc.y.clone() * cosine
        };
        (x, y)
    };
    let (actual_x, actual_y) = direct_frame(actual);
    let (expected_x, expected_y) = direct_frame(expected);
    Ok(vectors_equal(&actual_x, &expected_x)? && vectors_equal(&actual_y, &expected_y)?)
}

pub(super) fn real_values_equal(left: &Real, right: &Real) -> Result<bool, BuildError> {
    match compare_reals(left, right, crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided { value, .. } => Ok(value == std::cmp::Ordering::Equal),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                needed,
                stage,
            }))
        }
    }
}

pub(super) fn insert_exact_split_parameter(
    parameters: &mut Vec<Real>,
    candidate: Real,
) -> Result<(), GeometryError> {
    let mut insertion = parameters.len();
    for (index, parameter) in parameters.iter().enumerate() {
        match decided_model_order(compare_reals(
            &candidate,
            parameter,
            crate::STRICT_PREDICATES,
        ))? {
            std::cmp::Ordering::Less => {
                insertion = index;
                break;
            }
            std::cmp::Ordering::Equal => return Ok(()),
            std::cmp::Ordering::Greater => {}
        }
    }
    parameters.insert(insertion, candidate);
    Ok(())
}

pub(super) fn split_parameter_correspondence(
    source: &ParameterCorrespondence,
    pcurve: &Pcurve,
    edge_domain: &ParameterRange,
    direction: Direction,
) -> Result<ParameterCorrespondence, BuildError> {
    match source {
        ParameterCorrespondence::AngularSweep => Ok(ParameterCorrespondence::AngularSweep),
        ParameterCorrespondence::Affine { scale, offset } => {
            let (edge_start, edge_end) = match direction {
                Direction::Forward => (edge_domain.start(), edge_domain.end()),
                Direction::Reversed => (edge_domain.end(), edge_domain.start()),
            };
            let (local_start, local_end) = pcurve.curve().parameter_endpoints();
            if let (Some(edge_start), Some(edge_end), Some(local_start), Some(local_end)) = (
                edge_start.try_real(),
                edge_end.try_real(),
                local_start.scalar(),
                local_end.scalar(),
            ) {
                // Native subdivisions can reparameterize their pieces. Keep
                // the existing scalar formula and avoid preparing root maps.
                let scale = ((edge_end - edge_start) / (local_end - local_start))
                    .map_err(GeometryError::Numeric)?;
                let offset = edge_start - &scale * local_start;
                return ParameterCorrespondence::affine(scale, offset);
            }
            let pcurve_start = pcurve.domain_start()?;
            let pcurve_end = pcurve.domain_end()?;
            if pcurve_start
                .affine_image(scale, offset)?
                .compare(edge_start)?
                == Ordering::Equal
                && pcurve_end.affine_image(scale, offset)?.compare(edge_end)? == Ordering::Equal
            {
                return Ok(source.clone());
            }
            Err(GeometryError::UnrepresentableParameter.into())
        }
    }
}

pub(super) fn exact_real_min_max(values: &[Real]) -> Result<(Real, Real), BuildError> {
    let Some(first) = values.first() else {
        return Err(BuildError::Geometry(GeometryError::InvalidParameterDomain));
    };
    let mut min = first.clone();
    let mut max = first.clone();
    for value in values.iter().skip(1) {
        if decided_model_order(compare_reals(value, &min, crate::STRICT_PREDICATES))?
            == std::cmp::Ordering::Less
        {
            min = value.clone();
        }
        if decided_model_order(compare_reals(value, &max, crate::STRICT_PREDICATES))?
            == std::cmp::Ordering::Greater
        {
            max = value.clone();
        }
    }
    Ok((min, max))
}

pub(super) fn exact_intervals_cover(
    intervals: &mut Vec<(Real, Real)>,
    domain_start: &Real,
    domain_end: &Real,
) -> Result<bool, BuildError> {
    let mut cursor = domain_start.clone();
    while !intervals.is_empty() {
        let mut continuation = None;
        for (index, (start, _)) in intervals.iter().enumerate() {
            if real_values_equal(start, &cursor)? {
                if continuation.is_some() {
                    return Ok(false);
                }
                continuation = Some(index);
            }
        }
        let Some(index) = continuation else {
            return Ok(false);
        };
        let (_, end) = intervals.swap_remove(index);
        cursor = end;
    }
    real_values_equal(&cursor, domain_end)
}

pub(super) fn torus_parameter_coordinates_can_share_representative(
    left: &Real,
    right: &Real,
) -> bool {
    match compare_reals(left, right, crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Equal,
            ..
        } => true,
        PredicateOutcome::Decided { .. } => false,
        PredicateOutcome::Unknown { .. } => {
            // The caller has already certified identical spatial curve
            // parameterizations on one torus. Such constant coordinates are
            // congruent modulo tau. Keeping both strictly inside one period
            // rules out the distinct 0/tau seam representatives.
            let zero = Real::zero();
            let tau = Real::tau();
            [left, right].into_iter().all(|value| {
                compare_reals(value, &zero, crate::STRICT_PREDICATES).value()
                    == Some(std::cmp::Ordering::Greater)
                    && compare_reals(value, &tau, crate::STRICT_PREDICATES).value()
                        == Some(std::cmp::Ordering::Less)
            })
        }
    }
}

pub(super) fn insert_sorted_real(values: &mut Vec<Real>, value: &Real) -> Result<(), BuildError> {
    for index in 0..values.len() {
        match decided_model_order(compare_reals(
            value,
            &values[index],
            crate::STRICT_PREDICATES,
        ))? {
            std::cmp::Ordering::Less => {
                values.insert(index, value.clone());
                return Ok(());
            }
            std::cmp::Ordering::Equal => return Ok(()),
            std::cmp::Ordering::Greater => {}
        }
    }
    values.push(value.clone());
    Ok(())
}

pub(super) fn exact_real_index(values: &[Real], value: &Real) -> Result<usize, BuildError> {
    for (index, candidate) in values.iter().enumerate() {
        if real_values_equal(candidate, value)? {
            return Ok(index);
        }
    }
    unreachable!("value was inserted into the exact grid")
}

pub(super) fn certified_periodic_longitudinal_half_coverage(
    u_values: &[Real],
    v_values: &[Real],
    cells: &HashSet<(usize, usize)>,
    x: &Vector3,
    y: &Vector3,
    interior_normal: &Vector3,
) -> Result<bool, BuildError> {
    if u_values.len() < 2 || v_values.len() < 2 {
        return Ok(false);
    }
    let angular_span = u_values.last().expect("nonempty torus u grid") - &u_values[0];
    if decided_model_order(compare_reals(
        &angular_span,
        &Real::tau(),
        crate::STRICT_PREDICATES,
    ))? == std::cmp::Ordering::Greater
    {
        return Ok(false);
    }

    let quarter = (Real::pi() / Real::from(2)).map_err(GeometryError::Numeric)?;
    let signed_radial = |angle: &Real| {
        let radial = x.clone() * angle.clone().cos() + y.clone() * angle.clone().sin();
        radial.dot(interior_normal)
    };
    let mut selected_span = Real::zero();
    for u_cell in 0..u_values.len() - 1 {
        let width = &u_values[u_cell + 1] - &u_values[u_cell];
        if decided_model_order(compare_reals(
            &width,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(false);
        }
        let complete = (0..v_values.len() - 1).all(|v_cell| cells.contains(&(u_cell, v_cell)));
        let partial = (0..v_values.len() - 1).any(|v_cell| cells.contains(&(u_cell, v_cell)));
        if partial && !complete {
            return Ok(false);
        }
        let midpoint = ((&u_values[u_cell] + &u_values[u_cell + 1]) / Real::from(2))
            .map_err(GeometryError::Numeric)?;
        let start_order = decided_model_order(compare_reals(
            &signed_radial(&u_values[u_cell]),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))?;
        let midpoint_order = decided_model_order(compare_reals(
            &signed_radial(&midpoint),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))?;
        let end_order = decided_model_order(compare_reals(
            &signed_radial(&u_values[u_cell + 1]),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))?;
        if complete {
            if decided_model_order(compare_reals(&width, &quarter, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Greater
                || start_order == std::cmp::Ordering::Less
                || midpoint_order != std::cmp::Ordering::Greater
                || end_order == std::cmp::Ordering::Less
            {
                return Ok(false);
            }
            selected_span += width;
        } else {
            if decided_model_order(compare_reals(&width, &Real::pi(), crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Greater
                || start_order == std::cmp::Ordering::Greater
                || midpoint_order != std::cmp::Ordering::Less
                || end_order == std::cmp::Ordering::Greater
            {
                return Ok(false);
            }
        }
    }
    if !real_values_equal(&selected_span, &Real::pi())? {
        return Ok(false);
    }

    if decided_model_order(compare_reals(
        &angular_span,
        &Real::tau(),
        crate::STRICT_PREDICATES,
    ))? == std::cmp::Ordering::Less
    {
        let complement_start = u_values.last().expect("nonempty torus u grid");
        let complement_end = &u_values[0] + Real::tau();
        let midpoint = ((complement_start + &complement_end) / Real::from(2))
            .map_err(GeometryError::Numeric)?;
        if decided_model_order(compare_reals(
            &signed_radial(complement_start),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(
                &signed_radial(&midpoint),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &signed_radial(&complement_end),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Greater
        {
            return Ok(false);
        }
    }
    Ok(true)
}

pub(super) fn require_real_equal(
    left: &Real,
    right: &Real,
    mismatch: BuildError,
) -> Result<(), BuildError> {
    if (left - right).zero_status() == hyperreal::ZeroKnowledge::Zero {
        return Ok(());
    }
    match compare_reals(left, right, crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Equal,
            ..
        } => Ok(()),
        PredicateOutcome::Decided { .. } => Err(mismatch),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                needed,
                stage,
            }))
        }
    }
}

pub(super) fn curve_point_coordinate(point: &CurvePoint2, axis: usize) -> &Real {
    if axis == 0 { point.x() } else { point.y() }
}

pub(super) fn require_vector_equal(
    left: &Vector3,
    right: &Vector3,
    mismatch: BuildError,
) -> Result<(), BuildError> {
    if vectors_equal(left, right)? {
        Ok(())
    } else {
        Err(mismatch)
    }
}

pub(super) fn require_point_equal(
    left: &Point3,
    right: &Point3,
    mismatch: BuildError,
) -> Result<(), BuildError> {
    if [&left.x - &right.x, &left.y - &right.y, &left.z - &right.z]
        .into_iter()
        .all(|difference| difference.zero_status() == hyperreal::ZeroKnowledge::Zero)
    {
        return Ok(());
    }
    match point3_equal(left, right, crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided { value: true, .. } => Ok(()),
        PredicateOutcome::Decided { value: false, .. } => Err(mismatch),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                needed,
                stage,
            }))
        }
    }
}

pub(super) fn require_curve_point_at_parameter(
    curve: &Curve3,
    parameter: &Parameter,
    point: &GeometricPoint3,
    mismatch: BuildError,
) -> Result<(), BuildError> {
    let evaluated = curve.point_at(parameter)?;
    let unresolved = match evaluated.coincides_with(point) {
        Ok(true) => return Ok(()),
        Ok(false) => return Err(mismatch),
        Err(error) => error,
    };
    let Some(coordinates) = point.try_coordinates() else {
        return Err(unresolved.into());
    };
    match curve.parameters_of(coordinates)? {
        CurveParameterLocation::EntireDomain => Ok(()),
        CurveParameterLocation::None => Err(mismatch),
        CurveParameterLocation::Parameters(parameters) => {
            let mut unresolved = None;
            for candidate in parameters {
                match candidate.compare(parameter) {
                    Ok(std::cmp::Ordering::Equal) => return Ok(()),
                    Ok(_) => {}
                    Err(error) => {
                        unresolved.get_or_insert(error);
                    }
                }
            }
            if let Some(error) = unresolved {
                Err(BuildError::Geometry(error))
            } else {
                Err(mismatch)
            }
        }
    }
}

pub(super) fn sphere_volume(radius: &Real) -> Result<Real, GeometryError> {
    (Real::from(4) * Real::pi() * radius * radius * radius / Real::from(3))
        .map_err(GeometryError::Numeric)
}

pub(super) fn sphere_finite_cylinder_overlap_volume(
    sphere: &CertifiedSphereShell,
    cylinder: &CertifiedSphereFiniteCylinderRegion,
) -> Result<Real, GeometryError> {
    let center_parameter = (&sphere.center - &cylinder.origin).dot(&cylinder.axis);
    let relative_min = &cylinder.v_min - &center_parameter;
    let relative_max = &cylinder.v_max - center_parameter;
    let maximum = |first: &Real, second: &Real| -> Result<Real, GeometryError> {
        Ok(
            if decided_model_order(compare_reals(first, second, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Less
            {
                second.clone()
            } else {
                first.clone()
            },
        )
    };
    let minimum = |first: &Real, second: &Real| -> Result<Real, GeometryError> {
        Ok(
            if decided_model_order(compare_reals(first, second, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Greater
            {
                second.clone()
            } else {
                first.clone()
            },
        )
    };
    let lower = maximum(&relative_min, &-sphere.radius.clone())?;
    let upper = minimum(&relative_max, &sphere.radius)?;
    if decided_model_order(compare_reals(&lower, &upper, crate::STRICT_PREDICATES))?
        != std::cmp::Ordering::Less
    {
        return Ok(Real::zero());
    }
    let half_height = (&sphere.radius * &sphere.radius - &cylinder.radius * &cylinder.radius)
        .sqrt()
        .map_err(GeometryError::Numeric)?;
    let sphere_primitive = |height: &Real| -> Result<Real, GeometryError> {
        let cubic = (height * height * height / Real::from(3)).map_err(GeometryError::Numeric)?;
        Ok(Real::pi() * (&sphere.radius * &sphere.radius * height - cubic))
    };
    let cylinder_primitive =
        |height: &Real| Real::pi() * &cylinder.radius * &cylinder.radius * height;
    let mut volume = Real::zero();
    let lower_cap_end = minimum(&upper, &-half_height.clone())?;
    if decided_model_order(compare_reals(
        &lower,
        &lower_cap_end,
        crate::STRICT_PREDICATES,
    ))? == std::cmp::Ordering::Less
    {
        volume += sphere_primitive(&lower_cap_end)? - sphere_primitive(&lower)?;
    }
    let core_start = maximum(&lower, &-half_height.clone())?;
    let core_end = minimum(&upper, &half_height)?;
    if decided_model_order(compare_reals(
        &core_start,
        &core_end,
        crate::STRICT_PREDICATES,
    ))? == std::cmp::Ordering::Less
    {
        volume += cylinder_primitive(&core_end) - cylinder_primitive(&core_start);
    }
    let upper_cap_start = maximum(&lower, &half_height)?;
    if decided_model_order(compare_reals(
        &upper_cap_start,
        &upper,
        crate::STRICT_PREDICATES,
    ))? == std::cmp::Ordering::Less
    {
        volume += sphere_primitive(&upper)? - sphere_primitive(&upper_cap_start)?;
    }
    Ok(volume)
}

pub(super) fn sphere_pair_intersection_volume(
    pair: &CertifiedSpherePairShell,
) -> Result<Real, GeometryError> {
    let distance = (&pair.second_center - &pair.first_center)
        .norm_squared()
        .sqrt()
        .map_err(GeometryError::Numeric)?;
    let radius_sum = &pair.first_radius + &pair.second_radius;
    let radius_difference = &pair.first_radius - &pair.second_radius;
    let overlap_depth = &radius_sum - &distance;
    let bracket = &distance * &distance + Real::from(2) * &distance * &radius_sum
        - Real::from(3) * &radius_difference * &radius_difference;
    (Real::pi() * &overlap_depth * overlap_depth * bracket / (Real::from(12) * distance))
        .map_err(GeometryError::Numeric)
}

pub(super) fn compute_bounds(
    vertices: &[Vertex],
    curves: &[Curve3],
    spheres: &[Option<CertifiedSphereShell>],
    sphere_pairs: &[Option<CertifiedSpherePairShell>],
) -> Result<Option<Aabb>, GeometryError> {
    let mut bounds = None;
    for vertex in vertices {
        if let Some(coordinates) = vertex.point.try_coordinates() {
            union_point_bounds(&mut bounds, coordinates, coordinates)?;
        } else {
            let enclosure = vertex.point.bounds()?;
            union_point_bounds(&mut bounds, &enclosure.mins, &enclosure.maxs)?;
        }
    }
    for curve in curves {
        let curve_bounds = curve.bounds()?;
        union_point_bounds(&mut bounds, &curve_bounds.mins, &curve_bounds.maxs)?;
    }
    for sphere in spheres.iter().flatten() {
        union_sphere_bounds(&mut bounds, &sphere.center, &sphere.radius)?;
    }
    for pair in sphere_pairs.iter().flatten() {
        union_sphere_bounds(&mut bounds, &pair.first_center, &pair.first_radius)?;
        union_sphere_bounds(&mut bounds, &pair.second_center, &pair.second_radius)?;
    }
    Ok(bounds.map(|(mins, maxs)| Aabb::new(mins, maxs)))
}

fn union_point_bounds(
    bounds: &mut Option<(Point3, Point3)>,
    lower: &Point3,
    upper: &Point3,
) -> Result<(), GeometryError> {
    let Some((mins, maxs)) = bounds else {
        *bounds = Some((lower.clone(), upper.clone()));
        return Ok(());
    };
    update_min(&mut mins.x, &lower.x)?;
    update_min(&mut mins.y, &lower.y)?;
    update_min(&mut mins.z, &lower.z)?;
    update_max(&mut maxs.x, &upper.x)?;
    update_max(&mut maxs.y, &upper.y)?;
    update_max(&mut maxs.z, &upper.z)?;
    Ok(())
}

pub(super) fn union_sphere_bounds(
    bounds: &mut Option<(Point3, Point3)>,
    center: &Point3,
    radius: &Real,
) -> Result<(), GeometryError> {
    let sphere_min = Point3::new(&center.x - radius, &center.y - radius, &center.z - radius);
    let sphere_max = Point3::new(&center.x + radius, &center.y + radius, &center.z + radius);
    let (mins, maxs) = bounds.get_or_insert_with(|| (sphere_min.clone(), sphere_max.clone()));
    update_min(&mut mins.x, &sphere_min.x)?;
    update_min(&mut mins.y, &sphere_min.y)?;
    update_min(&mut mins.z, &sphere_min.z)?;
    update_max(&mut maxs.x, &sphere_max.x)?;
    update_max(&mut maxs.y, &sphere_max.y)?;
    update_max(&mut maxs.z, &sphere_max.z)?;
    Ok(())
}

pub(super) fn update_min(current: &mut Real, candidate: &Real) -> Result<(), GeometryError> {
    if decided_model_order(compare_reals(candidate, current, crate::STRICT_PREDICATES))?
        == std::cmp::Ordering::Less
    {
        *current = candidate.clone();
    }
    Ok(())
}

pub(super) fn update_max(current: &mut Real, candidate: &Real) -> Result<(), GeometryError> {
    if decided_model_order(compare_reals(candidate, current, crate::STRICT_PREDICATES))?
        == std::cmp::Ordering::Greater
    {
        *current = candidate.clone();
    }
    Ok(())
}

pub(super) fn curve_path_signed_area(path: &CurvePath2) -> Result<Real, GeometryError> {
    let area = crate::error::classified(path.boundary_loop()?.signed_area())?;
    resolve_planar_classification(area)?.ok_or(GeometryError::UnsupportedMeasurement)
}

pub(super) fn decided_model_order(
    outcome: PredicateOutcome<std::cmp::Ordering>,
) -> Result<std::cmp::Ordering, GeometryError> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage })
        }
    }
}

pub(super) fn exact_point_order(
    left: &Point3,
    right: &Point3,
) -> Result<std::cmp::Ordering, GeometryError> {
    match compare_point3_lexicographic(left, right, crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage })
        }
    }
}

pub(crate) fn compare_curve3_exact_data(
    left: &Curve3ExactData,
    right: &Curve3ExactData,
) -> Result<std::cmp::Ordering, GeometryError> {
    let rank = |data: &Curve3ExactData| match data {
        Curve3ExactData::Line(_) => 0_u8,
        Curve3ExactData::RationalBezier { .. } => 1,
        Curve3ExactData::Nurbs { .. } => 2,
        Curve3ExactData::EllipseArc(data) if data.circle => 3,
        Curve3ExactData::EllipseArc(_) => 4,
    };
    let family = rank(left).cmp(&rank(right));
    if family != std::cmp::Ordering::Equal {
        return Ok(family);
    }
    match (left, right) {
        (Curve3ExactData::Line(left), Curve3ExactData::Line(right)) => {
            compare_point_pairs(&left.start, &left.end, &right.start, &right.end)
        }
        (
            Curve3ExactData::RationalBezier {
                control_points: left_points,
                weights: left_weights,
            },
            Curve3ExactData::RationalBezier {
                control_points: right_points,
                weights: right_weights,
            },
        ) => {
            let controls = compare_point3_slices(left_points, right_points)?;
            if controls != std::cmp::Ordering::Equal {
                return Ok(controls);
            }
            compare_real_slices(left_weights, right_weights)
        }
        (
            Curve3ExactData::Nurbs {
                degree: left_degree,
                control_points: left_points,
                weights: left_weights,
                knots: left_knots,
            },
            Curve3ExactData::Nurbs {
                degree: right_degree,
                control_points: right_points,
                weights: right_weights,
                knots: right_knots,
            },
        ) => {
            let degree = left_degree.cmp(right_degree);
            if degree != std::cmp::Ordering::Equal {
                return Ok(degree);
            }
            let controls = compare_point3_slices(left_points, right_points)?;
            if controls != std::cmp::Ordering::Equal {
                return Ok(controls);
            }
            let weights = compare_real_slices(left_weights, right_weights)?;
            if weights != std::cmp::Ordering::Equal {
                return Ok(weights);
            }
            compare_real_slices(left_knots, right_knots)
        }
        (Curve3ExactData::EllipseArc(left), Curve3ExactData::EllipseArc(right)) => {
            let center = exact_point_order(&left.center, &right.center)?;
            if center != std::cmp::Ordering::Equal {
                return Ok(center);
            }
            let x = compare_vector3(&left.x, &right.x)?;
            if x != std::cmp::Ordering::Equal {
                return Ok(x);
            }
            let y = compare_vector3(&left.y, &right.y)?;
            if y != std::cmp::Ordering::Equal {
                return Ok(y);
            }
            for (left_value, right_value) in [
                (&left.x_radius, &right.x_radius),
                (&left.y_radius, &right.y_radius),
                (&left.domain_start, &right.domain_start),
                (&left.domain_end, &right.domain_end),
                (&left.angle_at_start, &right.angle_at_start),
            ] {
                let order = decided_model_order(compare_reals(
                    left_value,
                    right_value,
                    crate::STRICT_PREDICATES,
                ))?;
                if order != std::cmp::Ordering::Equal {
                    return Ok(order);
                }
            }
            Ok(left.direction.cmp(&right.direction))
        }
        _ => unreachable!("equal exact-curve family ranks have matching variants"),
    }
}

pub(super) fn compare_point_pairs(
    left_first: &Point3,
    left_second: &Point3,
    right_first: &Point3,
    right_second: &Point3,
) -> Result<std::cmp::Ordering, GeometryError> {
    let first = exact_point_order(left_first, right_first)?;
    if first != std::cmp::Ordering::Equal {
        return Ok(first);
    }
    exact_point_order(left_second, right_second)
}

pub(super) fn compare_point3_slices(
    left: &[Point3],
    right: &[Point3],
) -> Result<std::cmp::Ordering, GeometryError> {
    for (left_point, right_point) in left.iter().zip(right) {
        let order = exact_point_order(left_point, right_point)?;
        if order != std::cmp::Ordering::Equal {
            return Ok(order);
        }
    }
    Ok(left.len().cmp(&right.len()))
}

pub(super) fn compare_real_slices(
    left: &[Real],
    right: &[Real],
) -> Result<std::cmp::Ordering, GeometryError> {
    for (left_value, right_value) in left.iter().zip(right) {
        let order = decided_model_order(compare_reals(
            left_value,
            right_value,
            crate::STRICT_PREDICATES,
        ))?;
        if order != std::cmp::Ordering::Equal {
            return Ok(order);
        }
    }
    Ok(left.len().cmp(&right.len()))
}

pub(super) fn compare_vector3(
    left: &Vector3,
    right: &Vector3,
) -> Result<std::cmp::Ordering, GeometryError> {
    for index in 0..3 {
        let order = decided_model_order(compare_reals(
            &left[index],
            &right[index],
            crate::STRICT_PREDICATES,
        ))?;
        if order != std::cmp::Ordering::Equal {
            return Ok(order);
        }
    }
    Ok(std::cmp::Ordering::Equal)
}

pub(super) fn compare_ordered_face_split_traces(
    left: &OrderedFaceSplitTrace,
    right: &OrderedFaceSplitTrace,
) -> Result<std::cmp::Ordering, GeometryError> {
    let lower = exact_point_order(&left.lower, &right.lower)?;
    if lower != std::cmp::Ordering::Equal {
        return Ok(lower);
    }
    exact_point_order(&left.upper, &right.upper)
}

pub(super) fn compare_ordered_surface_curve_traces(
    left: &OrderedSurfaceCurveTrace,
    right: &OrderedSurfaceCurveTrace,
) -> Result<std::cmp::Ordering, GeometryError> {
    match compare_curve3_exact_data(&left.exact_key.exact_data(), &right.exact_key.exact_data()) {
        Ok(order) => Ok(order),
        Err(GeometryError::PredicateUnresolved { .. }) => {
            Ok(left.source_index.cmp(&right.source_index))
        }
        Err(error) => Err(error),
    }
}

pub(super) fn projected_face_split_line(
    surface: &Surface,
    trace: &OrderedFaceSplitTrace,
) -> Result<LineSeg2, TopologyEditError> {
    let project = |point: &Point3| {
        let origin = surface
            .plane_origin()
            .expect("face-split projection prevalidates a plane");
        let (u, v) = surface
            .plane_directions()
            .expect("face-split projection prevalidates a plane");
        let displacement = point - origin;
        let uu = u.dot(u);
        let uv = u.dot(v);
        let vv = v.dot(v);
        let du = displacement.dot(u);
        let dv = displacement.dot(v);
        let determinant = &uu * &vv - &uv * &uv;
        Ok::<_, GeometryError>(CurvePoint2::new(
            ((&du * &vv - &dv * &uv) / &determinant).map_err(GeometryError::Numeric)?,
            ((&dv * &uu - &du * &uv) / determinant).map_err(GeometryError::Numeric)?,
        ))
    };
    LineSeg2::try_new(project(&trace.lower)?, project(&trace.upper)?)
        .map_err(GeometryError::from)
        .map_err(TopologyEditError::from)
}

pub(super) fn arranged_face_split_segments(
    trace: &OrderedFaceSplitTrace,
    planar: &LineSeg2,
    prior_lines: &[(usize, LineSeg2)],
) -> Result<Vec<Curve3>, TopologyEditError> {
    let mut cuts = Vec::new();
    for (prior_source, prior) in prior_lines {
        match planar.intersect_line(prior).map_err(GeometryError::from)? {
            LineLineIntersection::None => {}
            LineLineIntersection::Point { a_param, .. } => {
                insert_face_split_cut(&mut cuts, a_param)?;
            }
            LineLineIntersection::Overlap { .. } => {
                return Err(TopologyEditError::OverlappingFaceSplitTraces {
                    first: *prior_source,
                    second: trace.source_index,
                });
            }
            LineLineIntersection::Uncertain { reason } => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        }
    }

    let mut boundaries = Vec::with_capacity(cuts.len() + 2);
    boundaries.push(Real::zero());
    boundaries.extend(cuts);
    boundaries.push(Real::one());
    boundaries
        .windows(2)
        .map(|range| {
            Curve3::line(
                trace.curve.evaluate_real(&range[0])?,
                trace.curve.evaluate_real(&range[1])?,
            )
            .map_err(TopologyEditError::from)
        })
        .collect()
}

pub(super) fn insert_face_split_cut(
    cuts: &mut Vec<Real>,
    parameter: Real,
) -> Result<(), TopologyEditError> {
    if decided_model_order(compare_reals(
        &parameter,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
        || decided_model_order(compare_reals(
            &parameter,
            &Real::one(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Less
    {
        return Ok(());
    }
    for index in 0..cuts.len() {
        match decided_model_order(compare_reals(
            &parameter,
            &cuts[index],
            crate::STRICT_PREDICATES,
        ))? {
            std::cmp::Ordering::Less => {
                cuts.insert(index, parameter);
                return Ok(());
            }
            std::cmp::Ordering::Equal => return Ok(()),
            std::cmp::Ordering::Greater => {}
        }
    }
    cuts.push(parameter);
    Ok(())
}
