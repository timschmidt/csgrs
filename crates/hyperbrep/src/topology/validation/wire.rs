//! Validated construction: validation wire.

use crate::topology::*;

impl ModelBuilder {
    pub(in crate::topology) fn validate_wire_orientation(
        &self,
        wire: WireId,
        orientation: Orientation,
        outer: bool,
        surface: SurfaceId,
    ) -> Result<(), BuildError> {
        let signed_area_order = match self.signed_wire_double_area(wire) {
            Ok(signed_double_area) => decided_model_order(compare_reals(
                &signed_double_area,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))?,
            Err(BuildError::Geometry(GeometryError::UnsupportedPcurveContour)) => {
                let path = self.build_wire_curve_path(wire)?;
                let area = crate::error::classified(
                    path.boundary_loop()
                        .map_err(GeometryError::from)?
                        .signed_area(),
                )
                .map_err(GeometryError::from)?;
                let area = resolve_planar_classification(area)?;
                match area {
                    Some(area) => decided_model_order(compare_reals(
                        &area,
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))?,
                    None => self.tensor_graph_wire_orientation(wire, surface)?,
                }
            }
            Err(error) => return Err(error),
        };
        match signed_area_order {
            std::cmp::Ordering::Equal => Err(BuildError::DegenerateWireArea(wire)),
            value => {
                let expected = match (orientation, outer) {
                    (Orientation::Forward, true) | (Orientation::Reversed, false) => {
                        std::cmp::Ordering::Greater
                    }
                    (Orientation::Reversed, true) | (Orientation::Forward, false) => {
                        std::cmp::Ordering::Less
                    }
                };
                if value == expected {
                    Ok(())
                } else {
                    Err(BuildError::InconsistentWireOrientation(wire))
                }
            }
        }
    }

    pub(in crate::topology) fn tensor_graph_wire_orientation(
        &self,
        wire: WireId,
        surface: SurfaceId,
    ) -> Result<std::cmp::Ordering, BuildError> {
        let surface = self.surface_ref(surface)?;
        if !matches!(
            surface.kind(),
            SurfaceKind::RationalBezier | SurfaceKind::Nurbs
        ) {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        }
        let SurfaceParameterDomain::Closed(u_domain) = surface.domain().u() else {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        };
        let SurfaceParameterDomain::Closed(v_domain) = surface.domain().v() else {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        };
        let wire = self.wire_ref(wire)?;
        let mut graph_endpoints = None;
        let mut has_left_boundary = false;
        let mut has_right_boundary = false;
        let mut has_bottom_boundary = false;
        let mut has_top_boundary = false;
        for edge_use in &wire.edge_uses {
            let pcurve = self.pcurve_ref(self.edge_use_ref(*edge_use)?.pcurve)?;
            match pcurve.curve().geometry() {
                Some(CurveGeometry2::RationalBezier(_)) | Some(CurveGeometry2::Nurbs(_)) => {
                    if graph_endpoints.is_some() {
                        return Err(GeometryError::UnsupportedPcurveContour.into());
                    }
                    let start = pcurve.curve().start();
                    let end = pcurve.curve().end();
                    graph_endpoints = Some((
                        start.clone(),
                        end.clone(),
                        decided_model_order(compare_reals(
                            (end)
                                .coordinates()
                                .ok_or(GeometryError::UnsupportedPcurveContour)?
                                .x(),
                            (start)
                                .coordinates()
                                .ok_or(GeometryError::UnsupportedPcurveContour)?
                                .x(),
                            crate::STRICT_PREDICATES,
                        ))?,
                        decided_model_order(compare_reals(
                            (end)
                                .coordinates()
                                .ok_or(GeometryError::UnsupportedPcurveContour)?
                                .y(),
                            (start)
                                .coordinates()
                                .ok_or(GeometryError::UnsupportedPcurveContour)?
                                .y(),
                            crate::STRICT_PREDICATES,
                        ))?,
                    ));
                }
                Some(CurveGeometry2::Line(line))
                    if real_values_equal(line.start().x(), line.end().x())? =>
                {
                    has_left_boundary |= real_values_equal(line.start().x(), u_domain.start())?;
                    has_right_boundary |= real_values_equal(line.start().x(), u_domain.end())?;
                }
                Some(CurveGeometry2::Line(line))
                    if real_values_equal(line.start().y(), line.end().y())? =>
                {
                    has_bottom_boundary |= real_values_equal(line.start().y(), v_domain.start())?;
                    has_top_boundary |= real_values_equal(line.start().y(), v_domain.end())?;
                }
                Some(CurveGeometry2::Line(_)) => {
                    return Err(GeometryError::UnsupportedPcurveContour.into());
                }
                _ => return Err(GeometryError::UnsupportedPcurveContour.into()),
            }
        }
        let (graph_start, graph_end, graph_u_direction, graph_v_direction) =
            graph_endpoints.ok_or(GeometryError::UnsupportedPcurveContour)?;
        let mut candidates = Vec::with_capacity(2);
        let graph_on_left = real_values_equal(
            (graph_start)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .x(),
            u_domain.start(),
        )? && real_values_equal(
            (graph_end)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .x(),
            u_domain.start(),
        )?;
        let graph_on_right = real_values_equal(
            (graph_start)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .x(),
            u_domain.end(),
        )? && real_values_equal(
            (graph_end)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .x(),
            u_domain.end(),
        )?;
        let effective_left = if graph_on_left != graph_on_right {
            Some(if graph_on_left {
                !has_right_boundary
            } else {
                has_left_boundary
            })
        } else if has_left_boundary != has_right_boundary {
            Some(has_left_boundary)
        } else {
            None
        };
        if graph_v_direction != std::cmp::Ordering::Equal
            && let Some(effective_left) = effective_left
        {
            candidates.push(match (graph_v_direction, effective_left) {
                (std::cmp::Ordering::Greater, true) | (std::cmp::Ordering::Less, false) => {
                    std::cmp::Ordering::Greater
                }
                (std::cmp::Ordering::Less, true) | (std::cmp::Ordering::Greater, false) => {
                    std::cmp::Ordering::Less
                }
                (std::cmp::Ordering::Equal, _) => unreachable!("equal graph span rejected"),
            });
        }
        let graph_on_bottom = real_values_equal(
            (graph_start)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .y(),
            v_domain.start(),
        )? && real_values_equal(
            (graph_end)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .y(),
            v_domain.start(),
        )?;
        let graph_on_top = real_values_equal(
            (graph_start)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .y(),
            v_domain.end(),
        )? && real_values_equal(
            (graph_end)
                .coordinates()
                .ok_or(GeometryError::UnsupportedPcurveContour)?
                .y(),
            v_domain.end(),
        )?;
        let effective_bottom = if graph_on_bottom != graph_on_top {
            Some(if graph_on_bottom {
                !has_top_boundary
            } else {
                has_bottom_boundary
            })
        } else if has_bottom_boundary != has_top_boundary {
            Some(has_bottom_boundary)
        } else {
            None
        };
        if graph_u_direction != std::cmp::Ordering::Equal
            && let Some(effective_bottom) = effective_bottom
        {
            candidates.push(match (graph_u_direction, effective_bottom) {
                (std::cmp::Ordering::Less, true) | (std::cmp::Ordering::Greater, false) => {
                    std::cmp::Ordering::Greater
                }
                (std::cmp::Ordering::Greater, true) | (std::cmp::Ordering::Less, false) => {
                    std::cmp::Ordering::Less
                }
                (std::cmp::Ordering::Equal, _) => unreachable!("equal graph span rejected"),
            });
        }
        let Some(first) = candidates.first().copied() else {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        };
        if candidates.iter().any(|candidate| *candidate != first) {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        }
        Ok(first)
    }

    pub(in crate::topology) fn validate_spherical_trim(
        &self,
        wire: WireId,
        _orientation: Orientation,
    ) -> Result<(), BuildError> {
        let wire_record = self.wire_ref(wire)?;
        let mut first_u = None;
        let mut previous_u = None;
        let mut latitude = None;
        let mut direction = None;
        for edge_use_id in &wire_record.edge_uses {
            let edge_use = self.edge_use_ref(*edge_use_id)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;
            let Some(line) = pcurve.line_segment() else {
                return Err(BuildError::UnsupportedSphericalTrim(wire));
            };
            if !real_values_equal(line.start().y(), line.end().y())? {
                return Err(BuildError::UnsupportedSphericalTrim(wire));
            }
            if let Some(expected) = &latitude {
                if !real_values_equal(line.start().y(), expected)? {
                    return Err(BuildError::UnsupportedSphericalTrim(wire));
                }
            } else {
                latitude = Some(line.start().y().clone());
            }
            if let Some(previous) = &previous_u {
                if !real_values_equal(line.start().x(), previous)? {
                    return Err(BuildError::InvalidSphericalTrim(wire));
                }
            } else {
                first_u = Some(line.start().x().clone());
            }
            let segment_direction = decided_model_order(compare_reals(
                line.end().x(),
                line.start().x(),
                crate::STRICT_PREDICATES,
            ))?;
            if !matches!(
                segment_direction,
                std::cmp::Ordering::Less | std::cmp::Ordering::Greater
            ) || direction.is_some_and(|expected| expected != segment_direction)
            {
                return Err(BuildError::InvalidSphericalTrim(wire));
            }
            direction = Some(segment_direction);
            previous_u = Some(line.end().x().clone());
        }
        let latitude = latitude.ok_or(BuildError::UnsupportedSphericalTrim(wire))?;
        let half_pi =
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
        if decided_model_order(compare_reals(
            &latitude,
            &-half_pi.clone(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(&latitude, &half_pi, crate::STRICT_PREDICATES))?
                != std::cmp::Ordering::Less
        {
            return Err(BuildError::InvalidSphericalTrim(wire));
        }
        let total = previous_u.expect("nonempty wire") - first_u.expect("nonempty wire");
        let expected = match direction.expect("nonempty wire") {
            std::cmp::Ordering::Greater => Real::tau(),
            std::cmp::Ordering::Less => -Real::tau(),
            std::cmp::Ordering::Equal => unreachable!("zero spans rejected"),
        };
        if !real_values_equal(&total, &expected)? {
            return Err(BuildError::InvalidSphericalTrim(wire));
        }
        Ok(())
    }

    pub(in crate::topology) fn spherical_trim_coordinates(
        &self,
        wire_id: WireId,
    ) -> Result<(Real, std::cmp::Ordering), BuildError> {
        let wire = self.wire_ref(wire_id)?;
        let first = self.pcurve_ref(self.edge_use_ref(wire.edge_uses[0])?.pcurve)?;
        let last = self.pcurve_ref(
            self.edge_use_ref(*wire.edge_uses.last().expect("validated nonempty wire"))?
                .pcurve,
        )?;
        let first = first
            .line_segment()
            .ok_or(BuildError::UnsupportedSphericalTrim(wire_id))?;
        let last = last
            .line_segment()
            .ok_or(BuildError::UnsupportedSphericalTrim(wire_id))?;
        Ok((
            first.start().y().clone(),
            decided_model_order(compare_reals(
                last.end().x(),
                first.start().x(),
                crate::STRICT_PREDICATES,
            ))?,
        ))
    }

    pub(in crate::topology) fn validate_wire_nesting(
        &self,
        outer: WireId,
        inner: &[WireId],
    ) -> Result<(), BuildError> {
        if inner.is_empty() {
            return Ok(());
        }
        let outer_path = self.build_wire_curve_path(outer)?;
        for wire in inner {
            let path = self.build_wire_curve_path(*wire)?;
            let intersection = outer_path
                .intersect_path(&path)
                .map_err(GeometryError::from)?;
            if !intersection.is_disjoint() {
                return Err(BuildError::IntersectingFaceWires {
                    first: outer,
                    second: *wire,
                });
            }
            match crate::error::classified(outer_path.classify_point(&path.start()))
                .map_err(GeometryError::from)?
            {
                Classification::Decided(ContourPointLocation::Inside) => {}
                Classification::Decided(_) => return Err(BuildError::InnerWireOutside(*wire)),
                Classification::Uncertain(reason) => {
                    return Err(BuildError::Geometry(
                        GeometryError::PlanarClassificationUnresolved(reason),
                    ));
                }
            }
        }
        for (index, first) in inner.iter().enumerate() {
            let first_path = self.build_wire_curve_path(*first)?;
            for second in &inner[(index + 1)..] {
                let second_path = self.build_wire_curve_path(*second)?;
                let intersection = first_path
                    .intersect_path(&second_path)
                    .map_err(GeometryError::from)?;
                if !intersection.is_disjoint() {
                    return Err(BuildError::IntersectingFaceWires {
                        first: *first,
                        second: *second,
                    });
                }
                let nested = classification_is_inside(
                    crate::error::classified(first_path.classify_point(&second_path.start()))
                        .map_err(GeometryError::from)?,
                )? || classification_is_inside(
                    crate::error::classified(second_path.classify_point(&first_path.start()))
                        .map_err(GeometryError::from)?,
                )?;
                if nested {
                    return Err(BuildError::NestedInnerWires {
                        first: *first,
                        second: *second,
                    });
                }
            }
        }
        Ok(())
    }

    pub(in crate::topology) fn validate_wire_simplicity(
        &self,
        wire: WireId,
    ) -> Result<(), BuildError> {
        if let Some(simple) = self.axis_aligned_line_wire_is_simple(wire)? {
            return if simple {
                Ok(())
            } else {
                Err(BuildError::SelfIntersectingWire(wire))
            };
        }
        if let Some(simple) = self.circular_arc_chord_wire_is_simple(wire)? {
            return if simple {
                Ok(())
            } else {
                Err(BuildError::SelfIntersectingWire(wire))
            };
        }
        if self.rational_quadratic_control_hull_wire_is_simple(wire)? == Some(true) {
            return Ok(());
        }
        self.validate_curve_path_simplicity(wire)
    }

    pub(in crate::topology) fn rational_quadratic_control_hull_wire_is_simple(
        &self,
        wire: WireId,
    ) -> Result<Option<bool>, BuildError> {
        let path = self.build_wire_curve_path(wire)?;
        if path.curves().len() < 2
            || path.curves().iter().any(|curve| {
                !matches!(
                    curve.geometry(),
                    Some(CurveGeometry2::RationalQuadraticBezier(_))
                )
            })
        {
            return Ok(None);
        }

        for curve in path.curves() {
            let fragments = curve
                .native_bezier_fragments()
                .map_err(GeometryError::from)?;
            if fragments.len() != 1
                || !fragments[0]
                    .has_certified_injective_axis()
                    .map_err(GeometryError::from)?
            {
                return Ok(None);
            }
            let Some(CurveGeometry2::RationalQuadraticBezier(curve)) = curve.geometry() else {
                unreachable!("family checked above");
            };
            let mut sign = None;
            for weight in curve.weights() {
                let order = decided_model_order(compare_reals(
                    weight,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))?;
                if order == std::cmp::Ordering::Equal {
                    return Ok(None);
                }
                if sign.replace(order).is_some_and(|prior| prior != order) {
                    return Ok(None);
                }
            }
        }

        let controls = path
            .curves()
            .iter()
            .map(|curve| {
                let Some(CurveGeometry2::RationalQuadraticBezier(curve)) = curve.geometry() else {
                    unreachable!("family checked above");
                };
                curve.control_points()
            })
            .collect::<Vec<_>>();
        let exact_order = |left: &Real, right: &Real| match compare_reals(
            left,
            right,
            crate::STRICT_PREDICATES,
        ) {
            PredicateOutcome::Decided { value, .. } => Some(value),
            PredicateOutcome::Unknown { .. } => None,
        };
        let strictly_separated =
            |left: &[&CurvePoint2; 3], right: &[&CurvePoint2; 3], axis: usize| {
                left.iter().all(|left| {
                    right.iter().all(|right| {
                        exact_order(
                            curve_point_coordinate(left, axis),
                            curve_point_coordinate(right, axis),
                        ) == Some(std::cmp::Ordering::Less)
                    })
                })
            };
        let separated_at_shared_endpoint =
            |left: &[&CurvePoint2; 3],
             left_endpoint: usize,
             right: &[&CurvePoint2; 3],
             right_endpoint: usize,
             axis: usize,
             left_side: std::cmp::Ordering| {
                let boundary = curve_point_coordinate(left[left_endpoint], axis);
                if exact_order(
                    boundary,
                    curve_point_coordinate(right[right_endpoint], axis),
                ) != Some(std::cmp::Ordering::Equal)
                {
                    return false;
                }
                left.iter().enumerate().all(|(index, point)| {
                    exact_order(curve_point_coordinate(point, axis), boundary)
                        == Some(if index == left_endpoint {
                            std::cmp::Ordering::Equal
                        } else {
                            left_side
                        })
                }) && right.iter().enumerate().all(|(index, point)| {
                    exact_order(curve_point_coordinate(point, axis), boundary)
                        == Some(if index == right_endpoint {
                            std::cmp::Ordering::Equal
                        } else {
                            left_side.reverse()
                        })
                })
            };
        let separated_at_distinct_extremes =
            |left: &[&CurvePoint2; 3],
             right: &[&CurvePoint2; 3],
             axis: usize,
             left_side: std::cmp::Ordering| {
                let other_axis = 1 - axis;
                for left_extreme in 0..left.len() {
                    let boundary = curve_point_coordinate(left[left_extreme], axis);
                    for right_extreme in 0..right.len() {
                        if exact_order(boundary, curve_point_coordinate(right[right_extreme], axis))
                            != Some(std::cmp::Ordering::Equal)
                        {
                            continue;
                        }
                        let left_touches_once = left.iter().enumerate().all(|(index, point)| {
                            exact_order(curve_point_coordinate(point, axis), boundary)
                                == Some(if index == left_extreme {
                                    std::cmp::Ordering::Equal
                                } else {
                                    left_side
                                })
                        });
                        let right_touches_once = right.iter().enumerate().all(|(index, point)| {
                            exact_order(curve_point_coordinate(point, axis), boundary)
                                == Some(if index == right_extreme {
                                    std::cmp::Ordering::Equal
                                } else {
                                    left_side.reverse()
                                })
                        });
                        if left_touches_once
                            && right_touches_once
                            && exact_order(
                                curve_point_coordinate(left[left_extreme], other_axis),
                                curve_point_coordinate(right[right_extreme], other_axis),
                            )
                            .is_some_and(|order| order != std::cmp::Ordering::Equal)
                        {
                            return true;
                        }
                    }
                }
                false
            };

        for first in 0..controls.len() {
            for second in (first + 1)..controls.len() {
                let adjacent = second == first + 1 || (first == 0 && second + 1 == controls.len());
                let expected_controls = if second == first + 1 {
                    Some((2_usize, 0_usize))
                } else if adjacent {
                    Some((0_usize, 2_usize))
                } else {
                    None
                };
                let mut certified = false;
                for axis in 0..2 {
                    if !adjacent
                        && (strictly_separated(&controls[first], &controls[second], axis)
                            || strictly_separated(&controls[second], &controls[first], axis)
                            || [std::cmp::Ordering::Less, std::cmp::Ordering::Greater]
                                .into_iter()
                                .any(|side| {
                                    separated_at_distinct_extremes(
                                        &controls[first],
                                        &controls[second],
                                        axis,
                                        side,
                                    )
                                }))
                    {
                        certified = true;
                        break;
                    }
                    let Some((first_endpoint, second_endpoint)) = expected_controls else {
                        continue;
                    };
                    for first_side in [std::cmp::Ordering::Less, std::cmp::Ordering::Greater] {
                        if separated_at_shared_endpoint(
                            &controls[first],
                            first_endpoint,
                            &controls[second],
                            second_endpoint,
                            axis,
                            first_side,
                        ) {
                            certified = true;
                            break;
                        }
                    }
                    if certified {
                        break;
                    }
                }
                if !certified {
                    return Ok(None);
                }
            }
        }
        Ok(Some(true))
    }

    pub(in crate::topology) fn circular_arc_chord_wire_is_simple(
        &self,
        wire: WireId,
    ) -> Result<Option<bool>, BuildError> {
        let wire = self.wire_ref(wire)?;
        let mut arcs = Vec::with_capacity(wire.edge_uses.len().saturating_sub(1));
        let mut chords = Vec::new();
        let mut endpoints = Vec::with_capacity(wire.edge_uses.len());
        for (index, edge_use_id) in wire.edge_uses.iter().enumerate() {
            let edge_use = self.edge_use_ref(*edge_use_id)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;
            if let Some(arc) = pcurve.circular_arc() {
                arcs.push((index, arc));
                endpoints.push((arc.start().clone(), arc.end().clone()));
            } else if let Some(line) = pcurve.line_segment() {
                chords.push((index, line));
                endpoints.push((line.start().clone(), line.end().clone()));
            } else {
                return Ok(None);
            }
        }
        let Some((_, first_arc)) = arcs.first().copied() else {
            return Ok(None);
        };
        if chords.is_empty() {
            return Ok(None);
        }
        let point_equal = |left: &CurvePoint2, right: &CurvePoint2| -> Result<bool, BuildError> {
            Ok(real_values_equal(left.x(), right.x())? && real_values_equal(left.y(), right.y())?)
        };
        for index in 0..endpoints.len() {
            if !point_equal(
                &endpoints[index].1,
                &endpoints[(index + 1) % endpoints.len()].0,
            )? {
                return Ok(Some(false));
            }
        }

        let mut total_sweep = Real::zero();
        for (_, arc) in &arcs {
            if arc.is_clockwise() != first_arc.is_clockwise()
                || !point_equal(arc.center(), first_arc.center())?
                || !real_values_equal(arc.radius_squared_ref(), first_arc.radius_squared_ref())?
            {
                return Ok(Some(false));
            }
            let sweep = match crate::error::classified(arc.directed_sweep_angle())
                .map_err(GeometryError::from)?
            {
                Classification::Decided(sweep) => sweep,
                Classification::Uncertain(_) => return Ok(None),
            };
            total_sweep += sweep.abs();
        }
        if decided_model_order(compare_reals(
            &total_sweep,
            &Real::tau(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Less
        {
            return Ok(Some(false));
        }

        let adjacent = |first: usize, second: usize| {
            first + 1 == second
                || second + 1 == first
                || (first == 0 && second + 1 == endpoints.len())
                || (second == 0 && first + 1 == endpoints.len())
        };
        let allowed_shared_endpoint =
            |first: usize, second: usize, point: &CurvePoint2| -> Result<bool, BuildError> {
                if first + 1 == second {
                    point_equal(point, &endpoints[first].1)
                } else if second + 1 == first || (first == 0 && second + 1 == endpoints.len()) {
                    point_equal(point, &endpoints[first].0)
                } else if second == 0 && first + 1 == endpoints.len() {
                    point_equal(point, &endpoints[first].1)
                } else {
                    Ok(false)
                }
            };

        for (arc_index, arc) in &arcs {
            for (segment_index, (start, end)) in endpoints.iter().enumerate() {
                if segment_index == *arc_index {
                    continue;
                }
                for point in [start, end] {
                    let on_sweep = match crate::error::classified(arc.contains_point(point))
                        .map_err(GeometryError::from)?
                    {
                        Classification::Decided(value) => value,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                    if on_sweep
                        && (!adjacent(*arc_index, segment_index)
                            || !allowed_shared_endpoint(*arc_index, segment_index, point)?)
                    {
                        return Ok(Some(false));
                    }
                }
            }
        }

        // Every chord endpoint must stay in the closed disk. Convexity then
        // proves that the chord interior cannot cross the supporting circle;
        // the explicit chord/chord checks below handle the remaining possible
        // self-intersections. Allowing strict interior endpoints is essential
        // for radial sector wires produced by concurrent circular-face splits.
        for (_, chord) in &chords {
            for endpoint in [chord.start(), chord.end()] {
                if decided_model_order(compare_reals(
                    &endpoint.distance_squared(first_arc.center()),
                    first_arc.radius_squared_ref(),
                    crate::STRICT_PREDICATES,
                ))? == std::cmp::Ordering::Greater
                {
                    return Ok(Some(false));
                }
            }
        }
        for first in 0..chords.len() {
            for second in (first + 1)..chords.len() {
                let (first_index, first_chord) = chords[first];
                let (second_index, second_chord) = chords[second];
                match first_chord
                    .intersect_line(second_chord)
                    .map_err(GeometryError::from)?
                {
                    LineLineIntersection::None => {}
                    LineLineIntersection::Point {
                        a_param, b_param, ..
                    } if adjacent(first_index, second_index) => {
                        let (expected_first, expected_second) = if first_index + 1 == second_index {
                            (Real::one(), Real::zero())
                        } else if second_index + 1 == first_index || first_index == 0 {
                            (Real::zero(), Real::one())
                        } else {
                            (Real::one(), Real::zero())
                        };
                        if !real_values_equal(&a_param, &expected_first)?
                            || !real_values_equal(&b_param, &expected_second)?
                        {
                            return Ok(Some(false));
                        }
                    }
                    LineLineIntersection::Point { .. } | LineLineIntersection::Overlap { .. } => {
                        return Ok(Some(false));
                    }
                    LineLineIntersection::Uncertain { .. } => return Ok(None),
                }
            }
        }
        Ok(Some(true))
    }

    pub(in crate::topology) fn axis_aligned_line_wire_is_simple(
        &self,
        wire: WireId,
    ) -> Result<Option<bool>, BuildError> {
        enum AxisLine {
            Horizontal { y: Real, min: Real, max: Real },
            Vertical { x: Real, min: Real, max: Real },
        }

        let ordered = |first: &Real, second: &Real| -> Result<(Real, Real), BuildError> {
            Ok(
                if decided_model_order(compare_reals(first, second, crate::STRICT_PREDICATES))?
                    == std::cmp::Ordering::Less
                {
                    (first.clone(), second.clone())
                } else {
                    (second.clone(), first.clone())
                },
            )
        };
        let wire_record = self.wire_ref(wire)?;
        let mut lines = Vec::with_capacity(wire_record.edge_uses.len());
        for edge_use in &wire_record.edge_uses {
            let edge_use = self.edge_use_ref(*edge_use)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;
            let Some(line) = pcurve.line_segment() else {
                return Ok(None);
            };
            if real_values_equal(line.start().y(), line.end().y())? {
                let (min, max) = ordered(line.start().x(), line.end().x())?;
                lines.push(AxisLine::Horizontal {
                    y: line.start().y().clone(),
                    min,
                    max,
                });
            } else if real_values_equal(line.start().x(), line.end().x())? {
                let (min, max) = ordered(line.start().y(), line.end().y())?;
                lines.push(AxisLine::Vertical {
                    x: line.start().x().clone(),
                    min,
                    max,
                });
            } else {
                return Ok(None);
            }
        }

        let contains = |min: &Real, value: &Real, max: &Real| -> Result<bool, BuildError> {
            Ok(
                decided_model_order(compare_reals(min, value, crate::STRICT_PREDICATES))?
                    != std::cmp::Ordering::Greater
                    && decided_model_order(compare_reals(value, max, crate::STRICT_PREDICATES))?
                        != std::cmp::Ordering::Greater,
            )
        };
        for first in 0..lines.len() {
            for second in (first + 1)..lines.len() {
                if second == first + 1 || (first == 0 && second + 1 == lines.len()) {
                    continue;
                }
                let intersects = match (&lines[first], &lines[second]) {
                    (
                        AxisLine::Horizontal {
                            y: first_y,
                            min: first_min,
                            max: first_max,
                        },
                        AxisLine::Horizontal {
                            y: second_y,
                            min: second_min,
                            max: second_max,
                        },
                    ) => {
                        real_values_equal(first_y, second_y)?
                            && (contains(first_min, second_min, first_max)?
                                || contains(second_min, first_min, second_max)?)
                    }
                    (
                        AxisLine::Vertical {
                            x: first_x,
                            min: first_min,
                            max: first_max,
                        },
                        AxisLine::Vertical {
                            x: second_x,
                            min: second_min,
                            max: second_max,
                        },
                    ) => {
                        real_values_equal(first_x, second_x)?
                            && (contains(first_min, second_min, first_max)?
                                || contains(second_min, first_min, second_max)?)
                    }
                    (
                        AxisLine::Horizontal { y, min, max },
                        AxisLine::Vertical {
                            x,
                            min: vertical_min,
                            max: vertical_max,
                        },
                    )
                    | (
                        AxisLine::Vertical {
                            x,
                            min: vertical_min,
                            max: vertical_max,
                        },
                        AxisLine::Horizontal { y, min, max },
                    ) => contains(min, x, max)? && contains(vertical_min, y, vertical_max)?,
                };
                if intersects {
                    return Ok(Some(false));
                }
            }
        }
        Ok(Some(true))
    }

    pub(in crate::topology) fn validate_curve_path_simplicity(
        &self,
        wire: WireId,
    ) -> Result<(), BuildError> {
        let path = self.build_wire_curve_path(wire)?;
        let curves = path.curves();
        if curves.len() == 2 {
            let relation = curves[0]
                .intersect_curve(&curves[1])
                .map_err(GeometryError::from)?;
            if !relation.is_complete()
                || !relation.overlaps().is_empty()
                || relation.contacts().len() != 2
            {
                return Err(BuildError::SelfIntersectingWire(wire));
            }
            let expected = [
                (
                    curves[0].parameter_domain().start(),
                    curves[1].parameter_domain().end(),
                ),
                (
                    curves[0].parameter_domain().end(),
                    curves[1].parameter_domain().start(),
                ),
            ];
            let mut matched = [false; 2];
            for contact in relation.contacts() {
                let first_parameter = resolve_planar_classification(
                    crate::error::classified(contact.first().parameter())
                        .map_err(GeometryError::from)?,
                )?;
                let second_parameter = resolve_planar_classification(
                    crate::error::classified(contact.second().parameter())
                        .map_err(GeometryError::from)?,
                )?;
                let mut found = None;
                for (index, (expected_first, expected_second)) in expected.iter().enumerate() {
                    if !matched[index]
                        && resolve_planar_classification(
                            crate::error::classified(first_parameter.compare(expected_first))
                                .map_err(GeometryError::from)?,
                        )?
                        .is_eq()
                        && resolve_planar_classification(
                            crate::error::classified(second_parameter.compare(expected_second))
                                .map_err(GeometryError::from)?,
                        )?
                        .is_eq()
                    {
                        found = Some(index);
                        break;
                    }
                }
                let Some(index) = found else {
                    return Err(BuildError::SelfIntersectingWire(wire));
                };
                matched[index] = true;
            }
            return if matched.into_iter().all(|value| value) {
                Ok(())
            } else {
                Err(BuildError::SelfIntersectingWire(wire))
            };
        }
        for first_index in 0..curves.len() {
            for second_index in (first_index + 1)..curves.len() {
                let relation = curves[first_index]
                    .intersect_curve(&curves[second_index])
                    .map_err(GeometryError::from)?;
                if !relation.is_complete() {
                    return Err(GeometryError::UnsupportedIntersection.into());
                }
                let adjacent = second_index == first_index + 1
                    || (first_index == 0 && second_index + 1 == curves.len());
                if !relation.overlaps().is_empty() {
                    return Err(BuildError::SelfIntersectingWire(wire));
                }
                if !adjacent {
                    if !relation.contacts().is_empty() {
                        return Err(BuildError::SelfIntersectingWire(wire));
                    }
                    continue;
                }
                let (expected_first, expected_second) = if second_index == first_index + 1 {
                    (
                        curves[first_index].parameter_domain().end(),
                        curves[second_index].parameter_domain().start(),
                    )
                } else {
                    (
                        curves[first_index].parameter_domain().start(),
                        curves[second_index].parameter_domain().end(),
                    )
                };
                if relation.contacts().len() != 1 {
                    return Err(BuildError::SelfIntersectingWire(wire));
                }
                let contact = &relation.contacts()[0];
                let first_parameter = resolve_planar_classification(
                    crate::error::classified(contact.first().parameter())
                        .map_err(GeometryError::from)?,
                )?;
                let second_parameter = resolve_planar_classification(
                    crate::error::classified(contact.second().parameter())
                        .map_err(GeometryError::from)?,
                )?;
                if !resolve_planar_classification(
                    crate::error::classified(first_parameter.compare(expected_first))
                        .map_err(GeometryError::from)?,
                )?
                .is_eq()
                    || !resolve_planar_classification(
                        crate::error::classified(second_parameter.compare(expected_second))
                            .map_err(GeometryError::from)?,
                    )?
                    .is_eq()
                {
                    return Err(BuildError::SelfIntersectingWire(wire));
                }
            }
        }
        Ok(())
    }

    pub(in crate::topology) fn build_wire_curve_path(
        &self,
        wire: WireId,
    ) -> Result<CurvePath2, BuildError> {
        let wire = self.wire_ref(wire)?;
        Ok(CurvePath2::try_new(
            wire.edge_uses
                .iter()
                .map(|edge_use| {
                    let edge_use = self.edge_use_ref(*edge_use)?;
                    Ok(self.pcurve_ref(edge_use.pcurve)?.curve().clone())
                })
                .collect::<Result<Vec<_>, BuildError>>()?,
        )
        .map_err(GeometryError::from)?)
    }

    pub(in crate::topology) fn build_wire_contour(
        &self,
        wire: WireId,
    ) -> Result<Contour2, BuildError> {
        let wire = self.wire_ref(wire)?;
        let mut segments = Vec::with_capacity(wire.edge_uses.len());
        for edge_use in &wire.edge_uses {
            let edge_use = self.edge_use_ref(*edge_use)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;
            segments.push(pcurve.segment()?);
        }
        Ok(Contour2::try_new(segments).map_err(GeometryError::from)?)
    }

    pub(in crate::topology) fn signed_wire_double_area(
        &self,
        wire: WireId,
    ) -> Result<Real, BuildError> {
        let wire_record = self.wire_ref(wire)?;
        let mut line_area = Real::zero();
        let mut all_lines = true;
        for edge_use in &wire_record.edge_uses {
            let edge_use = self.edge_use_ref(*edge_use)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;
            let Some(line) = pcurve.line_segment() else {
                all_lines = false;
                break;
            };
            line_area += line.start().x() * line.end().y() - line.start().y() * line.end().x();
        }
        if all_lines {
            return Ok(line_area);
        }
        self.build_wire_contour(wire)?
            .signed_area()
            .map_err(GeometryError::from)?
            .map(|area| Real::from(2) * area)
            .ok_or(BuildError::DegenerateWireArea(wire))
    }
}
