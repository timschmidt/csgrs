//! Full-turn product-domain Booleans preserve the owning planar region.
//!
//! A common exact axis is proved before projecting either profile. The same
//! Hypercurve operation handles native and retained curves; the ordinary solid
//! builder validates every resulting component and cavity.

use super::*;

pub(super) fn coaxial_revolution_boolean(
    first_model: &ArenaModel,
    first_solid: SolidId,
    second_model: &ArenaModel,
    second_solid: SolidId,
    operation: BooleanOp,
) -> Result<Option<ArenaBooleanResult>, BooleanError> {
    let Some(first) = first_model.certified_revolution_profile(first_solid) else {
        return Ok(None);
    };
    let Some(second) = second_model.certified_revolution_profile(second_solid) else {
        return Ok(None);
    };
    let opposite = if vectors_exactly_equal(&first.axis, &second.axis)? {
        false
    } else if vectors_exactly_opposite(&first.axis, &second.axis)? {
        true
    } else {
        return Ok(None);
    };
    let origin_offset = &second.axis_origin - &first.axis_origin;
    let axial_offset = origin_offset.dot(&first.axis);
    let radial_offset = origin_offset - first.axis.clone() * &axial_offset;
    if exact_order(&radial_offset.norm_squared(), &Real::zero())? != Ordering::Equal {
        return Ok(None);
    }
    let (first_region, second_region) =
        if let (Some((first_outer, first_holes)), Some((second_outer, second_holes))) =
            (first.native_polygons(), second.native_polygons())
        {
            // Keep the established native polygon maps and carrier ordering.
            let second_outer = remap_revolution_profile(&second_outer, &axial_offset, opposite)?;
            let second_holes = second_holes
                .iter()
                .map(|hole| remap_revolution_profile(hole, &axial_offset, opposite))
                .collect::<Result<Vec<_>, _>>()?;
            (
                CurveRegion2::try_from_native_contours(vec![first_outer], first_holes)
                    .map_err(GeometryError::from)?,
                CurveRegion2::try_from_native_contours(vec![second_outer], second_holes)
                    .map_err(GeometryError::from)?,
            )
        } else {
            // A full turn has no distinguished meridian ray. Only the axial
            // coordinate changes when origins or unit-axis directions differ.
            let map = hypercurve::Similarity2::try_from_real_affine(
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::from(if opposite { -1 } else { 1 }),
                Real::zero(),
                axial_offset,
            )
            .map_err(GeometryError::from)?;
            (
                first.region()?,
                second
                    .region()?
                    .transform_similarity(&map)
                    .map_err(GeometryError::from)?,
            )
        };
    let result = first_region
        .boolean_region(&second_region, operation)
        .map_err(GeometryError::from)?;
    if result.is_empty() {
        return Ok(Some(ArenaBooleanResult::Empty));
    }
    let native_polygons = result
        .native_contours_fast_path()
        .map_err(GeometryError::from)?
        .is_some_and(|native| {
            native
                .material_contours()
                .iter()
                .chain(native.hole_contours())
                .all(|contour| {
                    contour
                        .segments()
                        .iter()
                        .all(|segment| matches!(segment, Segment2::Line(_)))
                })
        });
    if !native_polygons {
        let (model, solids) = crate::construction::revolve_curve_region(&result)?;
        let model = orient_revolution_model(model, &first.axis_origin, &first.axis)?;
        return Ok(Some(match solids.as_slice() {
            [] => ArenaBooleanResult::Empty,
            &[solid] => ArenaBooleanResult::Solid { model, solid },
            _ => ArenaBooleanResult::Solids { model, solids },
        }));
    }
    let profiles = match native_contour_profiles(&result)? {
        Classification::Decided(profiles) => profiles,
        Classification::Uncertain(reason) => return Err(BooleanError::Unresolved(reason)),
    };
    let mut solids = Vec::with_capacity(profiles.len());
    for profile in profiles {
        let outer = line_profile_points(&profile.material)?;
        let holes = profile
            .holes
            .iter()
            .map(line_profile_points)
            .collect::<Result<Vec<_>, _>>()?;
        solids.push(oriented_revolution_region(
            &outer,
            &holes,
            &first.axis_origin,
            &first.axis,
        )?);
    }
    Ok(Some(if solids.len() == 1 {
        let (model, solid) = solids.pop().expect("one revolution result");
        ArenaBooleanResult::Solid { model, solid }
    } else {
        merge_owned_solids(solids)?
    }))
}

fn remap_revolution_profile(
    profile: &Contour2,
    axial_offset: &Real,
    opposite: bool,
) -> Result<Contour2, BooleanError> {
    let map = |point: &hypercurve::Point2| {
        hypercurve::Point2::new(
            point.x().clone(),
            if opposite {
                axial_offset - point.y()
            } else {
                axial_offset + point.y()
            },
        )
    };
    let mut segments = profile
        .segments()
        .iter()
        .map(|segment| match segment {
            Segment2::Line(line) => LineSeg2::try_new(map(line.start()), map(line.end()))
                .map(Segment2::Line)
                .map_err(GeometryError::from)
                .map_err(BooleanError::from),
            Segment2::Arc(_) => Err(BooleanError::UnsupportedOperand),
        })
        .collect::<Result<Vec<_>, _>>()?;
    if opposite {
        segments.reverse();
        segments = segments
            .into_iter()
            .map(|segment| segment.reversed())
            .collect();
    }
    Contour2::try_new(segments)
        .map_err(GeometryError::from)
        .map_err(BooleanError::from)
}

fn line_profile_points(contour: &Contour2) -> Result<Vec<crate::Point2>, BooleanError> {
    contour
        .segments()
        .iter()
        .map(|segment| match segment {
            Segment2::Line(line) => Ok(crate::Point2::new(
                line.start().x().clone(),
                line.start().y().clone(),
            )),
            Segment2::Arc(_) => Err(BooleanError::UnsupportedOperand),
        })
        .collect()
}

fn oriented_revolution_region(
    outer: &[crate::Point2],
    holes: &[Vec<crate::Point2>],
    origin: &Point3,
    axis: &crate::Vector3,
) -> Result<(ArenaModel, SolidId), BooleanError> {
    let (model, solid) = crate::construction::revolve_region(outer, holes)?;
    Ok((orient_revolution_model(model, origin, axis)?, solid))
}

fn orient_revolution_model(
    model: ArenaModel,
    origin: &Point3,
    axis: &crate::Vector3,
) -> Result<ArenaModel, BooleanError> {
    let (x, y) = axis
        .orthonormal_basis_checked()
        .map_err(GeometryError::Numeric)?;
    let transform = Matrix4::affine_orthonormal(
        orthonormal_linear(&x, &y, axis),
        [origin.x.clone(), origin.y.clone(), origin.z.clone()],
    );
    Ok(model.transformed(&transform)?)
}
