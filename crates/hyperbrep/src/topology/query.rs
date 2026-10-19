//! Certified classification and retained family queries.

use super::*;

impl CertifiedAngularSector {
    fn classify_offset(&self, offset: &Vector3) -> Result<SolidPointLocation, QueryError> {
        let mut boundary = false;
        for normal in &self.interior_normals {
            match decided_model_order(compare_reals(
                &offset.dot(normal),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? {
                std::cmp::Ordering::Less => return Ok(SolidPointLocation::Outside),
                std::cmp::Ordering::Equal => boundary = true,
                std::cmp::Ordering::Greater => {}
            }
        }
        Ok(if boundary {
            SolidPointLocation::Boundary
        } else {
            SolidPointLocation::Inside
        })
    }
}

impl ArenaModel {
    /// Classifies a point against a validated planar line-bounded solid.
    ///
    /// Face contours are materialized once from retained pcurves. Ray
    /// directions that hit an edge or vertex are discarded and retried; if
    /// every deterministic direction is degenerate, the result stays an
    /// explicit unresolved error.
    pub fn classify_point(
        &self,
        solid: SolidId,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        crate::context::strict(|| {
            let solid_id = solid;
            let solid = self.solid(solid_id).ok_or(QueryError::InvalidReference {
                kind: EntityKind::Solid,
                index: solid_id.index(),
            })?;
            if let Some(cylinder) = &self.data.certified_cylinders[solid_id.index()] {
                return self.classify_point_against_cylinder(cylinder, point);
            }
            if let Some(sphere) = &self.data.certified_spheres[solid_id.index()] {
                return self.classify_point_against_sphere(sphere, point);
            }
            if let Some(pair) = &self.data.certified_sphere_pairs[solid_id.index()] {
                return self.classify_point_against_sphere_pair(pair, point);
            }
            if let Some(torus) = &self.data.certified_tori[solid_id.index()] {
                return self.classify_point_against_torus(torus, point);
            }
            if let Some(frustum) = &self.data.certified_cone_frustums[solid_id.index()] {
                return self.classify_point_against_cone_frustum(frustum, point);
            }
            if let Some(revolution) = &self.data.certified_revolutions[solid_id.index()] {
                return self.classify_point_against_revolution(revolution, point);
            }
            if let Some(loft) = &self.data.certified_lofts[solid_id.index()] {
                return self.classify_point_against_loft(loft, point);
            }
            if let Some(sweep) = &self.data.certified_curve_sweeps[solid_id.index()] {
                return self.classify_point_against_curve_sweep(sweep, point);
            }
            if let Some(prism) = &self.data.certified_prisms[solid_id.index()] {
                return self.classify_point_against_prism(prism, point);
            }
            if let Some(axial) = &self.data.certified_axial_shells[solid_id.index()] {
                return Self::classify_point_against_axial(axial, point);
            }
            for shell in std::iter::once(&solid.outer).chain(solid.voids.iter()) {
                for face in &self.shell(*shell).expect("validated shell ID").faces {
                    let signed = self.face_plane_value(*face, point)?;
                    if decided_model_order(compare_reals(
                        &signed,
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Equal
                    {
                        match self.classify_point_on_face(*face, point)? {
                            Classification::Decided(ContourPointLocation::Inside)
                            | Classification::Decided(ContourPointLocation::Boundary) => {
                                return Ok(SolidPointLocation::Boundary);
                            }
                            Classification::Decided(ContourPointLocation::Outside) => {}
                            Classification::Uncertain(reason) => {
                                return Err(
                                    GeometryError::PlanarClassificationUnresolved(reason).into()
                                );
                            }
                        }
                    }
                }
            }

            if self.classify_point_against_shell(solid.outer, point)? == SolidPointLocation::Outside
            {
                return Ok(SolidPointLocation::Outside);
            }
            for void_shell in &solid.voids {
                if self.classify_point_against_shell(*void_shell, point)?
                    == SolidPointLocation::Inside
                {
                    return Ok(SolidPointLocation::Outside);
                }
            }
            Ok(SolidPointLocation::Inside)
        })
    }

    pub(in crate::topology) fn classify_point_against_cylinder(
        &self,
        cylinder: &CertifiedCylinderShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &cylinder.origin;
        let axial = offset.dot(&cylinder.axis);
        let below = decided_model_order(compare_reals(
            &axial,
            &cylinder.v_min,
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less;
        let above = decided_model_order(compare_reals(
            &axial,
            &cylinder.v_max,
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Greater;
        if below || above {
            return Ok(SolidPointLocation::Outside);
        }
        let radial = offset - cylinder.axis.clone() * &axial;
        let radial_squared = radial.norm_squared();
        let radius_squared = &cylinder.radius * &cylinder.radius;
        let radial_order = decided_model_order(compare_reals(
            &radial_squared,
            &radius_squared,
            crate::STRICT_PREDICATES,
        ))?;
        if radial_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        if let Some(subtraction) = &cylinder.sphere_subtraction {
            let (center, radius) = match subtraction {
                CertifiedCylinderSphereSubtraction::Void { center, radius }
                | CertifiedCylinderSphereSubtraction::Component { center, radius, .. } => {
                    (center, radius)
                }
            };
            match decided_model_order(compare_reals(
                &(point - center).norm_squared(),
                &(radius * radius),
                crate::STRICT_PREDICATES,
            ))? {
                std::cmp::Ordering::Less => return Ok(SolidPointLocation::Outside),
                std::cmp::Ordering::Equal => return Ok(SolidPointLocation::Boundary),
                std::cmp::Ordering::Greater => {}
            }
        }
        let on_cap = decided_model_order(compare_reals(
            &axial,
            &cylinder.v_min,
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Equal
            || decided_model_order(compare_reals(
                &axial,
                &cylinder.v_max,
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Equal;
        if on_cap || radial_order == std::cmp::Ordering::Equal {
            Ok(SolidPointLocation::Boundary)
        } else {
            Ok(SolidPointLocation::Inside)
        }
    }

    pub(in crate::topology) fn classify_point_against_revolution(
        &self,
        revolution: &CertifiedRevolutionShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &revolution.axis_origin;
        let axial = offset.dot(&revolution.axis);
        let radial = offset - revolution.axis.clone() * &axial;
        let radius = radial
            .norm_squared()
            .sqrt()
            .map_err(GeometryError::Numeric)?;
        let profile_point = CurvePoint2::new(radius, axial);
        let location = match revolution.profile.classify_point(&profile_point)? {
            Classification::Decided(location) => location,
            Classification::Uncertain(reason) => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        };
        match location {
            ContourPointLocation::Outside => return Ok(SolidPointLocation::Outside),
            ContourPointLocation::Boundary => return Ok(SolidPointLocation::Boundary),
            ContourPointLocation::Inside => {}
        }
        for void in &revolution.voids {
            match void.classify_point(&profile_point)? {
                Classification::Decided(ContourPointLocation::Inside) => {
                    return Ok(SolidPointLocation::Outside);
                }
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(SolidPointLocation::Boundary);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        Ok(SolidPointLocation::Inside)
    }

    pub(in crate::topology) fn classify_point_against_loft(
        &self,
        loft: &CertifiedLoftShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let displacement = point - &loft.origin;
        let determinant = loft.u.dot(&loft.v.cross(&loft.height_axis));
        let parameter_u = (displacement.dot(&loft.v.cross(&loft.height_axis)) / &determinant)
            .map_err(GeometryError::Numeric)?;
        let parameter_v = (loft.u.dot(&displacement.cross(&loft.height_axis)) / &determinant)
            .map_err(GeometryError::Numeric)?;
        let parameter_t = (loft.u.dot(&loft.v.cross(&displacement)) / determinant)
            .map_err(GeometryError::Numeric)?;
        let t_order_min = decided_model_order(compare_reals(
            &parameter_t,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))?;
        let t_order_max = decided_model_order(compare_reals(
            &parameter_t,
            &Real::one(),
            crate::STRICT_PREDICATES,
        ))?;
        if t_order_min == std::cmp::Ordering::Less || t_order_max == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let mut selected_span = None;
        for span in &loft.spans {
            let at_or_above = decided_model_order(compare_reals(
                &parameter_t,
                &span.start,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less;
            let at_or_below = decided_model_order(compare_reals(
                &parameter_t,
                &span.end,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater;
            if at_or_above && at_or_below {
                selected_span = Some(span);
                break;
            }
        }
        let span = selected_span.ok_or(GeometryError::InvalidParameterDomain)?;
        let span_width = &span.end - &span.start;
        let local_parameter =
            ((&parameter_t - &span.start) / span_width).map_err(GeometryError::Numeric)?;
        let profile_point = CurvePoint2::new(parameter_u, parameter_v);
        let profile_location = match &span.interpolation {
            CertifiedLoftInterpolation::Homothetic {
                profile,
                scale,
                translation,
            } => {
                let factor = Real::one() + &local_parameter * (scale - Real::one());
                let normalized = CurvePoint2::new(
                    ((profile_point.x() - &local_parameter * translation.x()) / &factor)
                        .map_err(GeometryError::Numeric)?,
                    ((profile_point.y() - &local_parameter * translation.y()) / factor)
                        .map_err(GeometryError::Numeric)?,
                );
                match crate::error::classified(profile.classify_point(&normalized))
                    .map_err(GeometryError::from)?
                {
                    Classification::Decided(location) => location,
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
            CertifiedLoftInterpolation::ConvexCorresponding { lower, upper } => {
                classify_convex_loft_section(lower, upper, &local_parameter, &profile_point)?
            }
        };
        match profile_location {
            ContourPointLocation::Outside => Ok(SolidPointLocation::Outside),
            ContourPointLocation::Boundary => Ok(SolidPointLocation::Boundary),
            ContourPointLocation::Inside
                if t_order_min == std::cmp::Ordering::Equal
                    || t_order_max == std::cmp::Ordering::Equal =>
            {
                Ok(SolidPointLocation::Boundary)
            }
            ContourPointLocation::Inside => Ok(SolidPointLocation::Inside),
        }
    }

    pub(in crate::topology) fn classify_point_against_curve_sweep(
        &self,
        sweep: &CertifiedCurveSweepShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let start = sweep
            .path
            .evaluate_real(sweep.path.domain().represented_start()?)?;
        let end = sweep
            .path
            .evaluate_real(sweep.path.domain().represented_end()?)?;
        let start_u = Vector3::from(
            sweep
                .u_path
                .evaluate_real(sweep.u_path.domain().represented_start()?)?,
        );
        let start_v = Vector3::from(
            sweep
                .v_path
                .evaluate_real(sweep.v_path.domain().represented_start()?)?,
        );
        let normal = start_u.cross(&start_v);
        let progress = normal.dot(&(end - &start));
        let parameter = (normal.dot(&(point - &start)) / &progress)
            .map_err(|problem| QueryError::from(GeometryError::Numeric(problem)))?;
        let lower_order = decided_model_order(compare_reals(
            &parameter,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))?;
        let upper_order = decided_model_order(compare_reals(
            &parameter,
            &Real::one(),
            crate::STRICT_PREDICATES,
        ))?;
        if lower_order == std::cmp::Ordering::Less || upper_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let path_point = sweep.path.evaluate_real(&parameter)?;
        let u = Vector3::from(sweep.u_path.evaluate_real(&parameter)?);
        let v = Vector3::from(sweep.v_path.evaluate_real(&parameter)?);
        let profile_point = project_point_to_plane_frame(point, &path_point, &u, &v)
            .map_err(build_error_geometry)?;
        let location = match crate::error::classified(sweep.profile.classify_point(&profile_point))
            .map_err(GeometryError::from)?
        {
            Classification::Decided(location) => location,
            Classification::Uncertain(reason) => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        };
        match location {
            ContourPointLocation::Outside => return Ok(SolidPointLocation::Outside),
            ContourPointLocation::Boundary => return Ok(SolidPointLocation::Boundary),
            ContourPointLocation::Inside => {}
        }
        for hole in &sweep.holes {
            match crate::error::classified(hole.classify_point(&profile_point))
                .map_err(GeometryError::from)?
            {
                Classification::Decided(ContourPointLocation::Inside) => {
                    return Ok(SolidPointLocation::Outside);
                }
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(SolidPointLocation::Boundary);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        if lower_order == std::cmp::Ordering::Equal || upper_order == std::cmp::Ordering::Equal {
            Ok(SolidPointLocation::Boundary)
        } else {
            Ok(SolidPointLocation::Inside)
        }
    }

    pub(in crate::topology) fn classify_point_against_sphere(
        &self,
        sphere: &CertifiedSphereShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        if let CertifiedSphereRegion::FiniteCylinder(region) = &sphere.region {
            return self.classify_point_against_sphere_finite_cylinder(sphere, region, point);
        }
        let radial_boundary = if let CertifiedSphereRegion::Radial(clip) = &sphere.region {
            let offset = point - &sphere.center;
            let axial = offset.dot(&clip.axis);
            let radial = offset - clip.axis.clone() * axial;
            let radial_order = decided_model_order(compare_reals(
                &radial.norm_squared(),
                &(&clip.radius * &clip.radius),
                crate::STRICT_PREDICATES,
            ))?;
            if matches!(
                (clip.side, radial_order),
                (
                    CertifiedSphereRadialSide::Inside,
                    std::cmp::Ordering::Greater
                ) | (CertifiedSphereRadialSide::Outside, std::cmp::Ordering::Less)
            ) {
                return Ok(SolidPointLocation::Outside);
            }
            match radial_order {
                std::cmp::Ordering::Equal => true,
                std::cmp::Ordering::Less | std::cmp::Ordering::Greater => false,
            }
        } else {
            false
        };
        let clip_orders = if let CertifiedSphereRegion::Axial(clip) = &sphere.region {
            let height = (point - &sphere.center).dot(&clip.axis);
            let min =
                decided_model_order(compare_reals(&height, &clip.min, crate::STRICT_PREDICATES))?;
            let max =
                decided_model_order(compare_reals(&height, &clip.max, crate::STRICT_PREDICATES))?;
            if min == std::cmp::Ordering::Less || max == std::cmp::Ordering::Greater {
                return Ok(SolidPointLocation::Outside);
            }
            Some((min, max))
        } else {
            None
        };
        let distance_squared = (point - &sphere.center).norm_squared();
        let radius_squared = &sphere.radius * &sphere.radius;
        match decided_model_order(compare_reals(
            &distance_squared,
            &radius_squared,
            crate::STRICT_PREDICATES,
        ))? {
            std::cmp::Ordering::Greater => return Ok(SolidPointLocation::Outside),
            std::cmp::Ordering::Equal => return Ok(SolidPointLocation::Boundary),
            std::cmp::Ordering::Less => {}
        }
        if radial_boundary
            || clip_orders.is_some_and(|(min, max)| {
                min == std::cmp::Ordering::Equal || max == std::cmp::Ordering::Equal
            })
        {
            return Ok(SolidPointLocation::Boundary);
        }
        for void in &sphere.voids {
            let location = match void {
                CertifiedSphereVoid::Sphere { center, radius } => {
                    match decided_model_order(compare_reals(
                        &(point - center).norm_squared(),
                        &(radius * radius),
                        crate::STRICT_PREDICATES,
                    ))? {
                        std::cmp::Ordering::Less => SolidPointLocation::Inside,
                        std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
                        std::cmp::Ordering::Greater => SolidPointLocation::Outside,
                    }
                }
                CertifiedSphereVoid::Cylinder(cylinder) => {
                    let offset = point - &cylinder.origin;
                    let axial = offset.dot(&cylinder.axis);
                    let min = decided_model_order(compare_reals(
                        &axial,
                        &cylinder.v_min,
                        crate::STRICT_PREDICATES,
                    ))?;
                    let max = decided_model_order(compare_reals(
                        &axial,
                        &cylinder.v_max,
                        crate::STRICT_PREDICATES,
                    ))?;
                    if min == std::cmp::Ordering::Less || max == std::cmp::Ordering::Greater {
                        SolidPointLocation::Outside
                    } else {
                        let radial = offset - cylinder.axis.clone() * &axial;
                        match decided_model_order(compare_reals(
                            &radial.norm_squared(),
                            &(&cylinder.radius * &cylinder.radius),
                            crate::STRICT_PREDICATES,
                        ))? {
                            std::cmp::Ordering::Greater => SolidPointLocation::Outside,
                            std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
                            std::cmp::Ordering::Less
                                if min == std::cmp::Ordering::Equal
                                    || max == std::cmp::Ordering::Equal =>
                            {
                                SolidPointLocation::Boundary
                            }
                            std::cmp::Ordering::Less => SolidPointLocation::Inside,
                        }
                    }
                }
            };
            match location {
                SolidPointLocation::Inside => return Ok(SolidPointLocation::Outside),
                SolidPointLocation::Boundary => return Ok(SolidPointLocation::Boundary),
                SolidPointLocation::Outside => {}
            }
        }
        Ok(SolidPointLocation::Inside)
    }

    pub(in crate::topology) fn classify_point_against_sphere_finite_cylinder(
        &self,
        sphere: &CertifiedSphereShell,
        cylinder: &CertifiedSphereFiniteCylinderRegion,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let sphere_order = decided_model_order(compare_reals(
            &(point - &sphere.center).norm_squared(),
            &(&sphere.radius * &sphere.radius),
            crate::STRICT_PREDICATES,
        ))?;
        let offset = point - &cylinder.origin;
        let axial = offset.dot(&cylinder.axis);
        let axial_min = decided_model_order(compare_reals(
            &axial,
            &cylinder.v_min,
            crate::STRICT_PREDICATES,
        ))?;
        let axial_max = decided_model_order(compare_reals(
            &axial,
            &cylinder.v_max,
            crate::STRICT_PREDICATES,
        ))?;
        let cylinder_location =
            if axial_min == std::cmp::Ordering::Less || axial_max == std::cmp::Ordering::Greater {
                SolidPointLocation::Outside
            } else {
                let radial = offset - cylinder.axis.clone() * &axial;
                match decided_model_order(compare_reals(
                    &radial.norm_squared(),
                    &(&cylinder.radius * &cylinder.radius),
                    crate::STRICT_PREDICATES,
                ))? {
                    std::cmp::Ordering::Greater => SolidPointLocation::Outside,
                    std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
                    std::cmp::Ordering::Less
                        if axial_min == std::cmp::Ordering::Equal
                            || axial_max == std::cmp::Ordering::Equal =>
                    {
                        SolidPointLocation::Boundary
                    }
                    std::cmp::Ordering::Less => SolidPointLocation::Inside,
                }
            };
        let sphere_location = match sphere_order {
            std::cmp::Ordering::Less => SolidPointLocation::Inside,
            std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
            std::cmp::Ordering::Greater => SolidPointLocation::Outside,
        };
        Ok(match cylinder.operation {
            CertifiedSphereFiniteCylinderOperation::Union => {
                match (sphere_location, cylinder_location) {
                    (SolidPointLocation::Inside, _) | (_, SolidPointLocation::Inside) => {
                        SolidPointLocation::Inside
                    }
                    (SolidPointLocation::Boundary, _) | (_, SolidPointLocation::Boundary) => {
                        SolidPointLocation::Boundary
                    }
                    (SolidPointLocation::Outside, SolidPointLocation::Outside) => {
                        SolidPointLocation::Outside
                    }
                }
            }
            CertifiedSphereFiniteCylinderOperation::Intersection => {
                match (sphere_location, cylinder_location) {
                    (SolidPointLocation::Outside, _) | (_, SolidPointLocation::Outside) => {
                        SolidPointLocation::Outside
                    }
                    (SolidPointLocation::Boundary, _) | (_, SolidPointLocation::Boundary) => {
                        SolidPointLocation::Boundary
                    }
                    (SolidPointLocation::Inside, SolidPointLocation::Inside) => {
                        SolidPointLocation::Inside
                    }
                }
            }
            CertifiedSphereFiniteCylinderOperation::Difference => {
                match (sphere_location, cylinder_location) {
                    (SolidPointLocation::Outside, _) | (_, SolidPointLocation::Inside) => {
                        SolidPointLocation::Outside
                    }
                    (SolidPointLocation::Boundary, _)
                    | (SolidPointLocation::Inside, SolidPointLocation::Boundary) => {
                        SolidPointLocation::Boundary
                    }
                    (SolidPointLocation::Inside, SolidPointLocation::Outside) => {
                        SolidPointLocation::Inside
                    }
                }
            }
        })
    }

    pub(in crate::topology) fn classify_point_against_sphere_pair(
        &self,
        pair: &CertifiedSpherePairShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let first = decided_model_order(compare_reals(
            &(point - &pair.first_center).norm_squared(),
            &(&pair.first_radius * &pair.first_radius),
            crate::STRICT_PREDICATES,
        ))?;
        let second = decided_model_order(compare_reals(
            &(point - &pair.second_center).norm_squared(),
            &(&pair.second_radius * &pair.second_radius),
            crate::STRICT_PREDICATES,
        ))?;
        Ok(match pair.kind {
            CertifiedSpherePairKind::Union => {
                if first == std::cmp::Ordering::Less || second == std::cmp::Ordering::Less {
                    SolidPointLocation::Inside
                } else if first == std::cmp::Ordering::Equal || second == std::cmp::Ordering::Equal
                {
                    SolidPointLocation::Boundary
                } else {
                    SolidPointLocation::Outside
                }
            }
            CertifiedSpherePairKind::Intersection => {
                if first == std::cmp::Ordering::Greater || second == std::cmp::Ordering::Greater {
                    SolidPointLocation::Outside
                } else if first == std::cmp::Ordering::Equal || second == std::cmp::Ordering::Equal
                {
                    SolidPointLocation::Boundary
                } else {
                    SolidPointLocation::Inside
                }
            }
            CertifiedSpherePairKind::Difference => match (first, second) {
                (std::cmp::Ordering::Greater, _) => SolidPointLocation::Outside,
                (std::cmp::Ordering::Equal, std::cmp::Ordering::Less) => {
                    SolidPointLocation::Outside
                }
                (std::cmp::Ordering::Equal, _) => SolidPointLocation::Boundary,
                (std::cmp::Ordering::Less, std::cmp::Ordering::Less) => SolidPointLocation::Outside,
                (std::cmp::Ordering::Less, std::cmp::Ordering::Equal) => {
                    SolidPointLocation::Boundary
                }
                (std::cmp::Ordering::Less, std::cmp::Ordering::Greater) => {
                    SolidPointLocation::Inside
                }
            },
        })
    }

    pub(in crate::topology) fn classify_point_against_torus(
        &self,
        torus: &CertifiedTorusShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &torus.center;
        let axial = offset.dot(&torus.axis);
        let region_boundary = match &torus.region {
            CertifiedTorusRegion::Whole => false,
            CertifiedTorusRegion::Axial { min, max }
            | CertifiedTorusRegion::SectorBand { min, max, .. } => {
                let min_order =
                    decided_model_order(compare_reals(&axial, min, crate::STRICT_PREDICATES))?;
                let max_order =
                    decided_model_order(compare_reals(&axial, max, crate::STRICT_PREDICATES))?;
                if min_order == std::cmp::Ordering::Less || max_order == std::cmp::Ordering::Greater
                {
                    return Ok(SolidPointLocation::Outside);
                }
                let angular_boundary =
                    if let CertifiedTorusRegion::SectorBand { angular, .. } = &torus.region {
                        match angular.classify_offset(&offset)? {
                            SolidPointLocation::Outside => return Ok(SolidPointLocation::Outside),
                            SolidPointLocation::Inside => false,
                            SolidPointLocation::Boundary => true,
                        }
                    } else {
                        false
                    };
                angular_boundary
                    || min_order == std::cmp::Ordering::Equal
                    || max_order == std::cmp::Ordering::Equal
            }
            CertifiedTorusRegion::LongitudinalHalf { interior_normal } => {
                match decided_model_order(compare_reals(
                    &offset.dot(interior_normal),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? {
                    std::cmp::Ordering::Less => return Ok(SolidPointLocation::Outside),
                    std::cmp::Ordering::Equal => true,
                    std::cmp::Ordering::Greater => false,
                }
            }
        };
        let radial = offset - torus.axis.clone() * &axial;
        let radial_squared = radial.norm_squared();
        let major_squared = &torus.major_radius * &torus.major_radius;
        let minor_squared = &torus.minor_radius * &torus.minor_radius;
        let implicit_base = &radial_squared + &axial * &axial + &major_squared - &minor_squared;
        let left = &implicit_base * &implicit_base;
        let right = Real::from(4) * major_squared * radial_squared;
        Ok(
            match decided_model_order(compare_reals(&left, &right, crate::STRICT_PREDICATES))? {
                std::cmp::Ordering::Less if region_boundary => SolidPointLocation::Boundary,
                std::cmp::Ordering::Less => SolidPointLocation::Inside,
                std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
                std::cmp::Ordering::Greater => SolidPointLocation::Outside,
            },
        )
    }

    fn classify_point_against_axial(
        axial: &CertifiedAxialShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &axial.origin;
        let determinant = axial.u.dot(&axial.v.cross(&axial.axis));
        let x = (offset.dot(&axial.v.cross(&axial.axis)) / &determinant)
            .map_err(GeometryError::Numeric)?;
        let y = (axial.u.dot(&offset.cross(&axial.axis)) / &determinant)
            .map_err(GeometryError::Numeric)?;
        let height =
            (axial.u.dot(&axial.v.cross(&offset)) / determinant).map_err(GeometryError::Numeric)?;
        let point = hypercurve::CurvePoint2::from(CurvePoint2::new(x, y));
        let classify = |index: Option<usize>| -> Result<SolidPointLocation, QueryError> {
            let Some(band) = index.and_then(|i| axial.bands.get(i)) else {
                return Ok(SolidPointLocation::Outside);
            };
            Ok(
                match band
                    .region
                    .classify_point(&point)
                    .map_err(GeometryError::from)?
                {
                    RegionPointLocation::Inside => SolidPointLocation::Inside,
                    RegionPointLocation::Outside => SolidPointLocation::Outside,
                    RegionPointLocation::Boundary => SolidPointLocation::Boundary,
                },
            )
        };
        for (index, event) in axial.heights.iter().enumerate() {
            match decided_model_order(compare_reals(&height, event, crate::STRICT_PREDICATES))? {
                std::cmp::Ordering::Less => return classify(index.checked_sub(1)),
                std::cmp::Ordering::Equal => {
                    let a = classify(index.checked_sub(1))?;
                    let b = classify(Some(index))?;
                    return Ok(if a == b {
                        a
                    } else {
                        SolidPointLocation::Boundary
                    });
                }
                std::cmp::Ordering::Greater => {}
            }
        }
        Ok(SolidPointLocation::Outside)
    }

    pub(in crate::topology) fn classify_point_against_prism(
        &self,
        prism: &CertifiedPrismShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let displacement = point - &prism.origin;
        let determinant = prism.u.dot(&prism.v.cross(&prism.extrusion));
        let planar_u = (displacement.dot(&prism.v.cross(&prism.extrusion)) / &determinant)
            .map_err(GeometryError::Numeric)?;
        let planar_v = (prism.u.dot(&displacement.cross(&prism.extrusion)) / &determinant)
            .map_err(GeometryError::Numeric)?;
        let extrusion_parameter = (prism.u.dot(&prism.v.cross(&displacement)) / determinant)
            .map_err(GeometryError::Numeric)?;
        let min_order = decided_model_order(compare_reals(
            &extrusion_parameter,
            &prism.parameter_min,
            crate::STRICT_PREDICATES,
        ))?;
        let max_order = decided_model_order(compare_reals(
            &extrusion_parameter,
            &prism.parameter_max,
            crate::STRICT_PREDICATES,
        ))?;
        if min_order == std::cmp::Ordering::Less || max_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let planar = hypercurve::CurvePoint2::from(CurvePoint2::new(planar_u, planar_v));
        let planar_location = match &prism.profile {
            CertifiedPrismProfile::Paths { outer, holes, .. } => {
                match crate::error::classified(outer.classify_point(&planar))
                    .map_err(GeometryError::from)?
                {
                    Classification::Decided(ContourPointLocation::Outside) => {
                        RegionPointLocation::Outside
                    }
                    Classification::Decided(ContourPointLocation::Boundary) => {
                        RegionPointLocation::Boundary
                    }
                    Classification::Decided(ContourPointLocation::Inside) => {
                        let mut location = RegionPointLocation::Inside;
                        for hole in holes {
                            match crate::error::classified(hole.classify_point(&planar))
                                .map_err(GeometryError::from)?
                            {
                                Classification::Decided(ContourPointLocation::Inside) => {
                                    location = RegionPointLocation::Outside;
                                    break;
                                }
                                Classification::Decided(ContourPointLocation::Boundary) => {
                                    location = RegionPointLocation::Boundary;
                                    break;
                                }
                                Classification::Decided(ContourPointLocation::Outside) => {}
                                Classification::Uncertain(reason) => {
                                    return Err(GeometryError::PlanarClassificationUnresolved(
                                        reason,
                                    )
                                    .into());
                                }
                            }
                        }
                        location
                    }
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
            CertifiedPrismProfile::CurveRegion { region, .. } => {
                match crate::error::classified(region.classify_point(&planar))
                    .map_err(GeometryError::from)?
                {
                    Classification::Decided(location) => location,
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
        };
        match planar_location {
            RegionPointLocation::Outside => return Ok(SolidPointLocation::Outside),
            RegionPointLocation::Boundary => return Ok(SolidPointLocation::Boundary),
            RegionPointLocation::Inside => {}
        }
        if min_order == std::cmp::Ordering::Equal || max_order == std::cmp::Ordering::Equal {
            Ok(SolidPointLocation::Boundary)
        } else {
            Ok(SolidPointLocation::Inside)
        }
    }

    /// Maps one face-local pcurve parameter to the shared edge's canonical
    /// curve parameter.
    ///
    /// The result is exact for both affine relations and native circular
    /// angular-sweep relations. No inverse floating-point projection or
    /// tolerance search is performed.
    pub fn edge_parameter_at(
        &self,
        edge_use: EdgeUseId,
        pcurve_parameter: &Parameter,
    ) -> Result<Parameter, QueryError> {
        crate::context::strict(|| {
            let edge_use_record = self
                .edge_use(edge_use)
                .ok_or(QueryError::InvalidReference {
                    kind: EntityKind::EdgeUse,
                    index: edge_use.index(),
                })?;
            let edge = self
                .edge(edge_use_record.edge)
                .expect("validated edge-use edge ID");
            let pcurve = self
                .pcurve(edge_use_record.pcurve)
                .expect("validated edge-use pcurve ID");
            edge_use_record
                .parameter_correspondence
                .edge_parameter(
                    pcurve,
                    &edge.domain,
                    edge_use_record.direction,
                    pcurve_parameter,
                )
                .map_err(QueryError::from)
        })
    }

    /// Maps one shared edge parameter back to a face-local pcurve parameter.
    ///
    /// Native circular pcurves replay directed angular fraction through
    /// Hypercurve's exact rational-conic inverse; no sampled projection is
    /// used.
    pub fn pcurve_parameter_at(
        &self,
        edge_use: EdgeUseId,
        edge_parameter: &Parameter,
    ) -> Result<Parameter, QueryError> {
        crate::context::strict(|| {
            let edge_use_record = self
                .edge_use(edge_use)
                .ok_or(QueryError::InvalidReference {
                    kind: EntityKind::EdgeUse,
                    index: edge_use.index(),
                })?;
            let edge = self
                .edge(edge_use_record.edge)
                .expect("validated edge-use edge ID");
            let pcurve = self
                .pcurve(edge_use_record.pcurve)
                .expect("validated edge-use pcurve ID");
            edge_use_record
                .parameter_correspondence
                .pcurve_parameter(
                    pcurve,
                    &edge.domain,
                    edge_use_record.direction,
                    edge_parameter,
                )
                .map_err(QueryError::from)
        })
    }

    pub(in crate::topology) fn classify_point_with_ray(
        &self,
        shell: ShellId,
        point: &Point3,
        direction: &Vector3,
    ) -> Result<Option<SolidPointLocation>, QueryError> {
        let mut crossings = 0_usize;
        for face_id in &self.shell(shell).expect("validated shell ID").faces {
            let face = self.face(*face_id).expect("validated face ID");
            let surface = self.surface(face.surface).expect("validated surface ID");
            let (u, v) = surface
                .plane_directions()
                .expect("the current validated face matrix contains only planes");
            let denominator = u.cross(v).dot(direction);
            if decided_model_order(compare_reals(
                &denominator,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Equal
            {
                continue;
            }
            let plane_value = self.face_plane_value(*face_id, point)?;
            let parameter = ((-plane_value) / denominator).map_err(GeometryError::Numeric)?;
            match decided_model_order(compare_reals(
                &parameter,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? {
                std::cmp::Ordering::Less | std::cmp::Ordering::Equal => continue,
                std::cmp::Ordering::Greater => {}
            }
            let intersection = point.clone() + direction.clone() * parameter;
            match self.classify_point_on_face(*face_id, &intersection)? {
                Classification::Decided(ContourPointLocation::Inside) => crossings += 1,
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary)
                | Classification::Uncertain(hypercurve::UncertaintyReason::Boundary) => {
                    return Ok(None);
                }
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        Ok(Some(if crossings.is_multiple_of(2) {
            SolidPointLocation::Outside
        } else {
            SolidPointLocation::Inside
        }))
    }

    pub(in crate::topology) fn classify_point_against_shell(
        &self,
        shell: ShellId,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let directions = [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::from_xyz(Real::one(), Real::from(2), Real::from(4)),
        ];
        for direction in directions {
            if let Some(location) = self.classify_point_with_ray(shell, point, &direction)? {
                return Ok(location);
            }
        }
        Err(
            GeometryError::PlanarClassificationUnresolved(hypercurve::UncertaintyReason::Boundary)
                .into(),
        )
    }

    pub(in crate::topology) fn classify_point_against_cone_frustum(
        &self,
        frustum: &CertifiedConeFrustumShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &frustum.apex;
        let axial = offset.dot(&frustum.axis);
        let cosine = frustum.semi_angle.clone().cos();
        let v = (&axial / cosine).map_err(GeometryError::Numeric)?;
        let min_order =
            decided_model_order(compare_reals(&v, &frustum.v_min, crate::STRICT_PREDICATES))?;
        let max_order =
            decided_model_order(compare_reals(&v, &frustum.v_max, crate::STRICT_PREDICATES))?;
        if min_order == std::cmp::Ordering::Less || max_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let radial = offset.clone() - frustum.axis.clone() * axial;
        let radius = v * frustum.semi_angle.clone().sin();
        let radial_order = decided_model_order(compare_reals(
            &radial.norm_squared(),
            &(&radius * &radius),
            crate::STRICT_PREDICATES,
        ))?;
        if radial_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let region_boundary = match &frustum.region {
            CertifiedConeFrustumRegion::Whole => false,
            CertifiedConeFrustumRegion::Sector(angular) => {
                match angular.classify_offset(&offset)? {
                    SolidPointLocation::Outside => return Ok(SolidPointLocation::Outside),
                    SolidPointLocation::Inside => false,
                    SolidPointLocation::Boundary => true,
                }
            }
            CertifiedConeFrustumRegion::LongitudinalHalf { interior_normal } => {
                match decided_model_order(compare_reals(
                    &offset.dot(interior_normal),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? {
                    std::cmp::Ordering::Less => return Ok(SolidPointLocation::Outside),
                    std::cmp::Ordering::Equal => true,
                    std::cmp::Ordering::Greater => false,
                }
            }
        };
        if radial_order == std::cmp::Ordering::Equal
            || min_order == std::cmp::Ordering::Equal
            || max_order == std::cmp::Ordering::Equal
            || region_boundary
        {
            Ok(SolidPointLocation::Boundary)
        } else {
            Ok(SolidPointLocation::Inside)
        }
    }

    pub(in crate::topology) fn face_plane_value(
        &self,
        face: FaceId,
        point: &Point3,
    ) -> Result<Real, GeometryError> {
        let face = self.face(face).expect("validated face ID");
        let surface = self.surface(face.surface).expect("validated surface ID");
        let origin = surface
            .plane_origin()
            .expect("the current validated face matrix contains only planes");
        let (u, v) = surface
            .plane_directions()
            .expect("the current validated face matrix contains only planes");
        Ok(u.cross(v).dot(&(point - origin)))
    }

    pub(in crate::topology) fn classify_point_on_face(
        &self,
        face: FaceId,
        point: &Point3,
    ) -> Result<Classification<ContourPointLocation>, GeometryError> {
        let face_record = self.face(face).expect("validated face ID");
        let surface = self
            .surface(face_record.surface)
            .expect("validated surface ID");
        let origin = surface
            .plane_origin()
            .expect("the current validated face matrix contains only planes");
        let (u, v) = surface
            .plane_directions()
            .expect("the current validated face matrix contains only planes");
        let displacement = point - origin;
        let uu = u.dot(u);
        let uv = u.dot(v);
        let vv = v.dot(v);
        let du = displacement.dot(u);
        let dv = displacement.dot(v);
        let determinant = &uu * &vv - &uv * &uv;
        let parameter_u =
            ((&du * &vv - &dv * &uv) / &determinant).map_err(GeometryError::Numeric)?;
        let parameter_v =
            ((&dv * &uu - &du * &uv) / determinant).map_err(GeometryError::Numeric)?;
        let point = CurvePoint2::new(parameter_u, parameter_v);
        self.classify_surface_parameter_on_face(face, &point.into())
    }

    pub(crate) fn face_contours(&self, face: FaceId) -> Result<&[Contour2], GeometryError> {
        crate::cache::get_or_try_init(&self.data.face_contours[face.index()], || {
            self.build_face_contours(face)
        })
        .map(Vec::as_slice)
    }

    pub(in crate::topology) fn build_face_contours(
        &self,
        face: FaceId,
    ) -> Result<Vec<Contour2>, GeometryError> {
        let face = self.face(face).expect("validated face ID");
        face.boundary_wires()
            .map(|wire| self.build_model_wire_contour(*wire))
            .collect()
    }

    pub(in crate::topology) fn build_model_wire_contour(
        &self,
        wire: WireId,
    ) -> Result<Contour2, GeometryError> {
        let native = self
            .wire(wire)
            .expect("validated wire ID")
            .edge_uses
            .iter()
            .map(|edge_use| {
                let edge_use = self.edge_use(*edge_use).expect("validated edge-use ID");
                self.pcurve(edge_use.pcurve)
                    .expect("validated pcurve ID")
                    .segment()
            })
            .collect::<Result<Vec<_>, _>>();
        match native {
            Ok(segments) => return Contour2::try_new(segments).map_err(GeometryError::from),
            Err(GeometryError::UnsupportedPcurveContour) => {}
            Err(error) => return Err(error),
        }
        let segments = self
            .model_wire_curves(wire)?
            .into_iter()
            .map(|curve| Pcurve::new(curve).segment())
            .collect::<Result<Vec<_>, _>>()?;
        Contour2::try_new(segments).map_err(GeometryError::from)
    }

    pub(in crate::topology) fn build_model_wire_curve_path(
        &self,
        wire: WireId,
    ) -> Result<CurvePath2, GeometryError> {
        CurvePath2::try_new(self.model_wire_curves(wire)?).map_err(GeometryError::from)
    }

    fn model_wire_curves(&self, wire: WireId) -> Result<Vec<Curve2>, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        let curves = wire
            .edge_uses
            .iter()
            .map(|edge_use| {
                let edge_use = self.edge_use(*edge_use).expect("validated edge-use ID");
                self.pcurve(edge_use.pcurve)
                    .expect("validated pcurve ID")
                    .curve()
                    .clone()
            })
            .collect();
        crate::topology::wire_geometry::coalesce_restrictions(curves)
    }

    pub(crate) fn classify_surface_parameter_on_face(
        &self,
        face: FaceId,
        point: &hypercurve::CurvePoint2,
    ) -> Result<Classification<ContourPointLocation>, GeometryError> {
        let face = self.face(face).expect("validated face ID");
        let FaceBoundary::Trimmed { outer, inner } = &face.boundary else {
            return Ok(Classification::Decided(ContourPointLocation::Inside));
        };
        match crate::error::classified(
            self.build_model_wire_curve_path(*outer)?
                .classify_point(point),
        )
        .map_err(GeometryError::from)?
        {
            Classification::Decided(ContourPointLocation::Outside) => {
                return Ok(Classification::Decided(ContourPointLocation::Outside));
            }
            Classification::Decided(ContourPointLocation::Boundary) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
            Classification::Decided(ContourPointLocation::Inside) => {}
        }
        for wire in inner {
            match crate::error::classified(
                self.build_model_wire_curve_path(*wire)?
                    .classify_point(point),
            )
            .map_err(GeometryError::from)?
            {
                Classification::Decided(ContourPointLocation::Inside) => {
                    return Ok(Classification::Decided(ContourPointLocation::Outside));
                }
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Decided(ContourPointLocation::Boundary));
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(ContourPointLocation::Inside))
    }

    /// A validated right-prism axis, including holes and native cylinder solids.
    /// This is a view of admission evidence, not recognition from sampled bounds.
    pub(crate) fn certified_right_prism_frame(
        &self,
        solid_id: SolidId,
    ) -> Result<Option<(Point3, Vector3)>, GeometryError> {
        let Some(solid) = self.solid(solid_id) else {
            return Ok(None);
        };
        if !solid.voids.is_empty() {
            return Ok(None);
        }
        if let Some(prism) = self
            .data
            .certified_prisms
            .get(solid_id.index())
            .and_then(Option::as_ref)
        {
            for direction in [&prism.u, &prism.v] {
                if decided_model_order(compare_reals(
                    &direction.dot(&prism.extrusion),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Equal
                {
                    return Ok(None);
                }
            }
            return Ok(Some((
                prism.origin.clone(),
                prism
                    .extrusion
                    .clone()
                    .normalize_checked()
                    .map_err(GeometryError::Numeric)?,
            )));
        }
        Ok(self
            .certified_cylinder_profile(solid_id)
            .map(|cylinder| (cylinder.origin, cylinder.axis)))
    }

    pub(crate) fn certified_z_prism_profile(
        &self,
        solid_id: SolidId,
    ) -> Result<Option<CertifiedZPrismProfile>, GeometryError> {
        let Some(solid) = self.solid(solid_id) else {
            return Ok(None);
        };
        if !solid.voids.is_empty() {
            return Ok(None);
        }
        let shell = self
            .shell(solid.outer)
            .expect("validated solid shell reference");
        let certified_translation_family = self
            .data
            .certified_prisms
            .get(solid_id.index())
            .is_some_and(Option::is_some)
            || self
                .data
                .certified_cylinders
                .get(solid_id.index())
                .and_then(Option::as_ref)
                .is_some_and(|cylinder| cylinder.sphere_subtraction.is_none())
            || shell.faces.iter().all(|face| {
                matches!(
                    self.surface(self.face(*face).expect("validated shell face").surface())
                        .expect("validated face surface")
                        .kind(),
                    SurfaceKind::Plane | SurfaceKind::Extrusion
                )
            });
        if !certified_translation_family {
            return Ok(None);
        }
        let prism = self
            .data
            .certified_prisms
            .get(solid_id.index())
            .and_then(Option::as_ref);
        // Cold admission derives complete cap paths, while construction may
        // retain a CurveRegion directly. Both are shell certificates; neither
        // may be replaced by an endpoint polygon when a cap is nonlinear.
        let certified_region = || -> Result<Option<CurveRegion2>, GeometryError> {
            let Some(CertifiedPrismShell {
                profile,
                origin,
                u,
                v,
                extrusion,
                ..
            }) = prism
            else {
                return Ok(None);
            };
            let zero = Real::zero();
            let horizontal_profile =
                decided_model_order(compare_reals(&u.0[2], &zero, crate::STRICT_PREDICATES))?
                    == std::cmp::Ordering::Equal
                    && decided_model_order(compare_reals(
                        &v.0[2],
                        &zero,
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Equal;
            let vertical_extrusion = decided_model_order(compare_reals(
                &extrusion.0[0],
                &zero,
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Equal
                && decided_model_order(compare_reals(
                    &extrusion.0[1],
                    &zero,
                    crate::STRICT_PREDICATES,
                ))? == std::cmp::Ordering::Equal;
            if horizontal_profile && vertical_extrusion {
                let region = match profile {
                    CertifiedPrismProfile::CurveRegion { region, .. } => {
                        std::borrow::Cow::Borrowed(region)
                    }
                    CertifiedPrismProfile::Paths { outer, holes, .. } => {
                        let paths = std::iter::once(outer)
                            .chain(holes)
                            .cloned()
                            .collect::<Vec<_>>();
                        let roles = std::iter::once(hypercurve::CurveRegionLoopRole::Material)
                            .chain(std::iter::repeat_n(
                                hypercurve::CurveRegionLoopRole::Hole,
                                holes.len(),
                            ))
                            .collect::<Vec<_>>();
                        std::borrow::Cow::Owned(
                            CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                                &paths,
                                &roles,
                                &vec![hypercurve::FillRule::NonZero; paths.len()],
                            )
                            .map_err(GeometryError::from)?,
                        )
                    }
                };
                let region = region
                    .transform_affine(&u.0[0], &v.0[0], &u.0[1], &v.0[1], &origin.x, &origin.y)
                    .map_err(GeometryError::from)?;
                return Ok(Some(region));
            }
            Ok(None)
        };
        let mut vertex_ids = HashSet::new();
        for face_id in &shell.faces {
            let face = self.face(*face_id).expect("validated shell face");
            for wire_id in face.boundary_wires() {
                let wire = self.wire(*wire_id).expect("validated face wire");
                for edge_use_id in &wire.edge_uses {
                    let edge_use = self
                        .edge_use(*edge_use_id)
                        .expect("validated wire edge use");
                    let edge = self.edge(edge_use.edge).expect("validated edge reference");
                    vertex_ids.insert(edge.start);
                    vertex_ids.insert(edge.end);
                    if edge_use.direction == Direction::Forward {
                        let endpoints = [edge.start, edge.end]
                            .map(|vertex| self.vertex(vertex).expect("validated vertex").point());
                        if endpoints
                            .iter()
                            .any(|point| point.try_coordinates().is_none())
                        {
                            // The complete admitted prism, its horizontal frame
                            // and vertical extrusion prove R x [z0,z1]. Selected
                            // cap subdivisions need no independent vertex scalars.
                            let (Some(prism), Some(region)) = (prism, certified_region()?) else {
                                return Ok(None);
                            };
                            let z0 = prism.origin.z.clone();
                            let z1 = &z0 + &prism.extrusion.0[2];
                            let (z_min, z_max) = match decided_model_order(compare_reals(
                                &z0,
                                &z1,
                                crate::STRICT_PREDICATES,
                            ))? {
                                std::cmp::Ordering::Less => (z0, z1),
                                std::cmp::Ordering::Greater => (z1, z0),
                                std::cmp::Ordering::Equal => return Ok(None),
                            };
                            return Ok(Some(CertifiedZPrismProfile {
                                region,
                                z_min,
                                z_max,
                            }));
                        }
                        let start = self
                            .vertex(edge.start)
                            .expect("validated vertex")
                            .represented_point()?;
                        let end = self
                            .vertex(edge.end)
                            .expect("validated vertex")
                            .represented_point()?;
                        // Two endpoint levels alone also describe a skew prism.
                        // A world-Z product requires its connecting edges to
                        // retain x/y, in addition to the admitted shell proof.
                        if !real_values_equal(&start.z, &end.z).map_err(build_error_geometry)?
                            && (!real_values_equal(&start.x, &end.x)
                                .map_err(build_error_geometry)?
                                || !real_values_equal(&start.y, &end.y)
                                    .map_err(build_error_geometry)?)
                        {
                            return Ok(None);
                        }
                    }
                }
            }
        }
        let Some(first_id) = vertex_ids.iter().next() else {
            return Ok(None);
        };
        let first = self
            .vertex(*first_id)
            .expect("validated vertex")
            .represented_point()?;
        let mut z_min = first.z.clone();
        let mut z_max = first.z.clone();
        for vertex_id in &vertex_ids {
            let z = &self
                .vertex(*vertex_id)
                .expect("validated vertex")
                .represented_point()?
                .z;
            if decided_model_order(compare_reals(z, &z_min, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Less
            {
                z_min = z.clone();
            }
            if decided_model_order(compare_reals(z, &z_max, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Greater
            {
                z_max = z.clone();
            }
        }
        if decided_model_order(compare_reals(&z_min, &z_max, crate::STRICT_PREDICATES))?
            != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        for vertex_id in &vertex_ids {
            let z = &self
                .vertex(*vertex_id)
                .expect("validated vertex")
                .represented_point()?
                .z;
            let at_min = decided_model_order(compare_reals(z, &z_min, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Equal;
            let at_max = decided_model_order(compare_reals(z, &z_max, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Equal;
            if !at_min && !at_max {
                return Ok(None);
            }
        }

        if prism
            .is_some_and(|prism| matches!(prism.profile, CertifiedPrismProfile::CurveRegion { .. }))
            && let Some(region) = certified_region()?
        {
            return Ok(Some(CertifiedZPrismProfile {
                region,
                z_min,
                z_max,
            }));
        }

        let mut top_region = None::<CurveRegion2>;
        for face_id in &shell.faces {
            let face = self.face(*face_id).expect("validated shell face");
            let mut profiles = Vec::with_capacity(face.inner().len() + 1);
            let mut is_top = true;
            for wire_id in face.boundary_wires() {
                let wire = self.wire(*wire_id).expect("validated cap wire");
                let mut segments = Vec::with_capacity(wire.edge_uses.len());
                for edge_use_id in &wire.edge_uses {
                    let (start, end) = self.directed_vertices(*edge_use_id);
                    let point = self
                        .vertex(start)
                        .expect("validated vertex")
                        .represented_point()?;
                    let endpoint = self
                        .vertex(end)
                        .expect("validated vertex")
                        .represented_point()?;
                    // A reversed side can start at the top cap. Reject its
                    // axial edge before projection could collapse it to a point.
                    if decided_model_order(compare_reals(
                        &point.z,
                        &z_max,
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Equal
                        || decided_model_order(compare_reals(
                            &endpoint.z,
                            &z_max,
                            crate::STRICT_PREDICATES,
                        ))? != std::cmp::Ordering::Equal
                    {
                        is_top = false;
                        break;
                    }
                    let segment = self.projected_model_cap_segment(*edge_use_id, point, endpoint);
                    match segment {
                        Ok(segment) => segments.push(segment),
                        Err(GeometryError::UnsupportedPcurveContour) => {
                            let region = certified_region()?
                                .ok_or(GeometryError::UnsupportedPcurveContour)?;
                            return Ok(Some(CertifiedZPrismProfile {
                                region,
                                z_min,
                                z_max,
                            }));
                        }
                        Err(error) => return Err(error),
                    }
                }
                if !is_top {
                    break;
                }
                profiles.push(Contour2::try_new(segments).map_err(GeometryError::from)?);
            }
            if is_top && !profiles.is_empty() {
                let outer = profiles.remove(0);
                let region = CurveRegion2::try_from_native_contours(vec![outer], profiles)
                    .map_err(GeometryError::from)?;
                // A shape-preserving partition can split a cap into several
                // faces. The profile is their complete material cover, not
                // whichever face happens to occur first in the arena.
                top_region = Some(match top_region {
                    None => region,
                    Some(previous) => previous
                        .boolean_region(&region, BooleanOp::Union)
                        .map_err(GeometryError::from)?,
                });
            }
        }
        Ok(top_region.map(|region| CertifiedZPrismProfile {
            region,
            z_min,
            z_max,
        }))
    }

    pub(crate) fn certified_sphere_profile(
        &self,
        solid: SolidId,
    ) -> Option<CertifiedSphereProfile> {
        self.data
            .certified_spheres
            .get(solid.index())
            .and_then(Option::as_ref)
            .filter(|sphere| {
                sphere.voids.is_empty() && matches!(sphere.region, CertifiedSphereRegion::Whole)
            })
            .map(|sphere| CertifiedSphereProfile {
                center: sphere.center.clone(),
                radius: sphere.radius.clone(),
            })
    }

    pub(crate) fn certified_cylinder_profile(
        &self,
        solid: SolidId,
    ) -> Option<CertifiedCylinderProfile> {
        self.data
            .certified_cylinders
            .get(solid.index())
            .and_then(Option::as_ref)
            .filter(|cylinder| cylinder.sphere_subtraction.is_none())
            .map(|cylinder| CertifiedCylinderProfile {
                origin: cylinder.origin.clone(),
                axis: cylinder.axis.clone(),
                radius: cylinder.radius.clone(),
                v_min: cylinder.v_min.clone(),
                v_max: cylinder.v_max.clone(),
            })
    }

    pub(crate) fn certified_cone_frustum_profile(
        &self,
        solid: SolidId,
    ) -> Option<CertifiedConeFrustumProfile> {
        self.data
            .certified_cone_frustums
            .get(solid.index())
            .and_then(Option::as_ref)
            .filter(|frustum| matches!(frustum.region, CertifiedConeFrustumRegion::Whole))
            .map(|frustum| CertifiedConeFrustumProfile {
                apex: frustum.apex.clone(),
                axis: frustum.axis.clone(),
                semi_angle: frustum.semi_angle.clone(),
                v_min: frustum.v_min.clone(),
                v_max: frustum.v_max.clone(),
            })
    }

    pub(crate) fn certified_torus_profile(&self, solid: SolidId) -> Option<CertifiedTorusProfile> {
        self.data
            .certified_tori
            .get(solid.index())
            .and_then(Option::as_ref)
            .filter(|torus| matches!(torus.region, CertifiedTorusRegion::Whole))
            .map(|torus| CertifiedTorusProfile {
                center: torus.center.clone(),
                axis: torus.axis.clone(),
                major_radius: torus.major_radius.clone(),
                minor_radius: torus.minor_radius.clone(),
            })
    }

    pub(crate) fn certified_revolution_profile(
        &self,
        solid: SolidId,
    ) -> Option<CertifiedRevolutionProfile> {
        let revolution = self
            .data
            .certified_revolutions
            .get(solid.index())
            .and_then(Option::as_ref)?;
        Some(CertifiedRevolutionProfile {
            axis_origin: revolution.axis_origin.clone(),
            axis: revolution.axis.clone(),
            profile: revolution.profile.clone(),
            holes: revolution.voids.clone(),
        })
    }

    pub(in crate::topology) fn projected_model_cap_segment(
        &self,
        edge_use_id: EdgeUseId,
        start: &Point3,
        end: &Point3,
    ) -> Result<Segment2, GeometryError> {
        let edge_use = self
            .edge_use(edge_use_id)
            .expect("validated cap edge-use reference");
        let edge = self.edge(edge_use.edge).expect("validated cap edge");
        let curve = self.curve(edge.curve).expect("validated cap curve");
        match curve.kind() {
            Curve3Kind::Line => Ok(Segment2::Line(LineSeg2::try_new(
                CurvePoint2::new(start.x.clone(), start.y.clone()),
                CurvePoint2::new(end.x.clone(), end.y.clone()),
            )?)),
            Curve3Kind::CircleArc => {
                let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
                    unreachable!("circle kind carries ellipse-arc exact data");
                };
                let parameter = match edge_use.direction {
                    Direction::Forward => edge.domain.start(),
                    Direction::Reversed => edge.domain.end(),
                };
                let mut tangent = curve
                    .derivative_at(
                        (parameter)
                            .try_real()
                            .ok_or(crate::GeometryError::UnrepresentableParameter)?,
                        1,
                    )?
                    .vector()
                    .clone();
                if edge_use.direction == Direction::Reversed {
                    tangent = -tangent;
                }
                let radial_x = &start.x - &data.center.x;
                let radial_y = &start.y - &data.center.y;
                let cross = radial_x * &tangent.0[1] - radial_y * &tangent.0[0];
                let clockwise = match decided_model_order(compare_reals(
                    &cross,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? {
                    std::cmp::Ordering::Less => true,
                    std::cmp::Ordering::Greater => false,
                    std::cmp::Ordering::Equal => return Err(GeometryError::InvalidArcSweep),
                };
                Ok(Segment2::Arc(CircularArc2::try_from_center(
                    CurvePoint2::new(start.x.clone(), start.y.clone()),
                    CurvePoint2::new(end.x.clone(), end.y.clone()),
                    CurvePoint2::new(data.center.x, data.center.y),
                    clockwise,
                )?))
            }
            _ => Err(GeometryError::UnsupportedPcurveContour),
        }
    }
}
