//! Exact model measure operations.

use crate::topology::*;

impl ArenaModel {
    /// Returns the exact area of a supported validated face.
    ///
    /// Whole spheres and periodic latitude caps use analytic spherical area.
    /// Other active carriers integrate their exact parameter-space boundary
    /// and Jacobian without tessellation.
    pub fn face_area(&self, id: FaceId) -> Result<Real, QueryError> {
        crate::context::strict(|| {
            let face = self.face(id).ok_or(QueryError::InvalidReference {
                kind: EntityKind::Face,
                index: id.index(),
            })?;
            let surface = self
                .surface(face.surface)
                .expect("validated face surface ID");
            if let SurfaceExactData::Sphere { radius, .. } = surface.exact_data() {
                if face.is_whole_surface() {
                    return Ok(Real::from(4) * Real::pi() * &radius * radius);
                }
                let outer = face.outer().ok_or(GeometryError::UnsupportedMeasurement)?;
                if let [upper_wire] = face.inner() {
                    let lower = self
                        .pcurve(
                            self.edge_use(self.wire(outer).expect("validated wire").edge_uses[0])
                                .expect("validated use")
                                .pcurve,
                        )
                        .expect("validated pcurve")
                        .line_segment()
                        .ok_or(GeometryError::UnsupportedPcurveContour)?
                        .start()
                        .y()
                        .clone();
                    let upper = self
                        .pcurve(
                            self.edge_use(
                                self.wire(*upper_wire).expect("validated wire").edge_uses[0],
                            )
                            .expect("validated use")
                            .pcurve,
                        )
                        .expect("validated pcurve")
                        .line_segment()
                        .ok_or(GeometryError::UnsupportedPcurveContour)?
                        .start()
                        .y()
                        .clone();
                    return Ok(Real::tau() * &radius * radius * (upper.sin() - lower.sin()));
                }
                let wire = self.wire(outer).expect("validated spherical face wire");
                let mut latitude_cap = true;
                for edge_use in &wire.edge_uses {
                    let edge_use = self.edge_use(*edge_use).expect("validated spherical use");
                    let line = self
                        .pcurve(edge_use.pcurve)
                        .expect("validated spherical pcurve")
                        .line_segment()
                        .ok_or(GeometryError::UnsupportedPcurveContour)?;
                    if !real_values_equal(line.start().y(), line.end().y())
                        .map_err(build_error_geometry)?
                    {
                        latitude_cap = false;
                        break;
                    }
                }
                if !latitude_cap {
                    return Ok(self.signed_sphere_wire_area(outer, &radius)?.abs());
                }
                let first_use = self
                    .edge_use(wire.edge_uses[0])
                    .expect("validated spherical edge use");
                let pcurve = self
                    .pcurve(first_use.pcurve)
                    .expect("validated spherical pcurve");
                let line = pcurve
                    .line_segment()
                    .ok_or(GeometryError::UnsupportedMeasurement)?;
                let increasing = decided_model_order(compare_reals(
                    line.end().x(),
                    line.start().x(),
                    crate::STRICT_PREDICATES,
                ))? == std::cmp::Ordering::Greater;
                let upper = match face.orientation {
                    Orientation::Forward => increasing,
                    Orientation::Reversed => !increasing,
                };
                let sine = line.start().y().clone().sin();
                let height_factor = if upper {
                    Real::one() - sine
                } else {
                    Real::one() + sine
                };
                return Ok(Real::from(2) * Real::pi() * &radius * radius * height_factor);
            }
            let outer = face.outer().ok_or(GeometryError::UnsupportedMeasurement)?;
            if let SurfaceExactData::Torus {
                major_radius,
                minor_radius,
                ..
            } = surface.exact_data()
            {
                let magnitude = |wire| -> Result<Real, GeometryError> {
                    let signed = self.signed_torus_wire_area(wire, &major_radius, &minor_radius)?;
                    Ok(area_magnitude(signed))
                };
                let mut area = magnitude(outer)?;
                for inner in face.inner() {
                    area -= magnitude(*inner)?;
                }
                return Ok(area);
            }
            if let SurfaceExactData::Cone { semi_angle, .. } = surface.exact_data() {
                let mut area = area_magnitude(self.signed_cone_wire_area(outer, &semi_angle)?);
                for inner in face.inner() {
                    area -= area_magnitude(self.signed_cone_wire_area(*inner, &semi_angle)?);
                }
                return Ok(area);
            }
            if let SurfaceExactData::Revolution {
                profile,
                axis_origin,
                axis,
            } = surface.exact_data()
            {
                if !face.inner().is_empty() {
                    return Err(GeometryError::UnsupportedMeasurement.into());
                }
                let profile = Curve3::from_exact_data(*profile)?;
                let wire = self.wire(outer).expect("validated revolution face wire");
                if wire.edge_uses.len() != 4 {
                    return Err(GeometryError::UnsupportedMeasurement.into());
                }
                let mut u_values = Vec::with_capacity(2);
                let mut v_values = Vec::with_capacity(2);
                for use_id in &wire.edge_uses {
                    let pcurve = self
                        .pcurve(
                            self.edge_use(*use_id)
                                .expect("validated revolution edge use")
                                .pcurve,
                        )
                        .expect("validated revolution pcurve");
                    let segment = pcurve
                        .line_segment()
                        .ok_or(GeometryError::UnsupportedMeasurement)?;
                    if !real_values_equal(segment.start().x(), segment.end().x())
                        .map_err(build_error_geometry)?
                        && !real_values_equal(segment.start().y(), segment.end().y())
                            .map_err(build_error_geometry)?
                    {
                        return Err(GeometryError::UnsupportedMeasurement.into());
                    }
                    insert_sorted_real(&mut u_values, segment.start().x())
                        .map_err(build_error_geometry)?;
                    insert_sorted_real(&mut u_values, segment.end().x())
                        .map_err(build_error_geometry)?;
                    insert_sorted_real(&mut v_values, segment.start().y())
                        .map_err(build_error_geometry)?;
                    insert_sorted_real(&mut v_values, segment.end().y())
                        .map_err(build_error_geometry)?;
                }
                if u_values.len() != 2 || v_values.len() != 2 {
                    return Err(GeometryError::UnsupportedMeasurement.into());
                }
                let profile = profile.subcurve(&v_values[0], &v_values[1])?;
                let radial_at = |point: &Point3| {
                    let relative = point - &axis_origin;
                    let axial = axis.dot(&relative);
                    relative - axis.clone() * axial
                };
                let angular_span = &u_values[1] - &u_values[0];
                let line_image_area = || -> Result<Real, QueryError> {
                    let start = profile.evaluate_real(profile.domain().start())?;
                    let end = profile.evaluate_real(profile.domain().end())?;
                    let start_radial = radial_at(&start);
                    let end_radial = radial_at(&end);
                    if decided_model_order(compare_reals(
                        &start_radial.cross(&end_radial).norm_squared(),
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Equal
                        || decided_model_order(compare_reals(
                            &start_radial.dot(&end_radial),
                            &Real::zero(),
                            crate::STRICT_PREDICATES,
                        ))? != std::cmp::Ordering::Greater
                    {
                        return Err(GeometryError::UnsupportedMeasurement.into());
                    }
                    let start_radius = start_radial
                        .norm_squared()
                        .sqrt()
                        .map_err(GeometryError::Numeric)?;
                    let end_radius = end_radial
                        .norm_squared()
                        .sqrt()
                        .map_err(GeometryError::Numeric)?;
                    let profile_length = (&end - &start)
                        .norm_squared()
                        .sqrt()
                        .map_err(GeometryError::Numeric)?;
                    (angular_span.clone() * profile_length * (start_radius + end_radius)
                        / Real::from(2))
                    .map_err(GeometryError::Numeric)
                    .map_err(QueryError::from)
                };
                return match profile.exact_data() {
                    Curve3ExactData::Line(_) => line_image_area(),
                    Curve3ExactData::RationalBezier { .. } | Curve3ExactData::Nurbs { .. }
                        if certified_monotone_line_curve_image(&profile)
                            .map_err(build_error_geometry)? =>
                    {
                        line_image_area()
                    }
                    Curve3ExactData::EllipseArc(data) if data.circle => {
                        let start_point = profile.evaluate_real(profile.domain().start())?;
                        let start_radial = radial_at(&start_point);
                        let start_radius = start_radial
                            .norm_squared()
                            .sqrt()
                            .map_err(GeometryError::Numeric)?;
                        let radial_unit =
                            (start_radial / &start_radius).map_err(GeometryError::Numeric)?;
                        let center_radial = radial_unit.dot(&radial_at(&data.center));
                        let x_radial = radial_unit.dot(&data.x);
                        let y_radial = radial_unit.dot(&data.y);
                        let start_angle = &data.angle_at_start
                            + Real::from(data.direction)
                                * (profile.domain().start() - &data.domain_start);
                        let end_angle = &data.angle_at_start
                            + Real::from(data.direction)
                                * (profile.domain().end() - &data.domain_start);
                        let direction = Real::from(data.direction);
                        let integral_cos =
                            &direction * (end_angle.clone().sin() - start_angle.clone().sin());
                        let integral_sin =
                            direction * (start_angle.clone().cos() - end_angle.clone().cos());
                        let parameter_span = profile.domain().end() - profile.domain().start();
                        let radial_integral = center_radial * parameter_span
                            + &data.x_radius * (x_radial * integral_cos + y_radial * integral_sin);
                        Ok(angular_span * data.x_radius * radial_integral)
                    }
                    _ => Err(GeometryError::UnsupportedMeasurement.into()),
                };
            }
            if let SurfaceExactData::Plane { origin, u, v } = surface.exact_data()
                && let Some(area) = self.analytic_planar_face_area(face, &origin, &u, &v)?
            {
                return Ok(area);
            }
            let mut parameter_area = area_magnitude(self.signed_model_wire_double_area(outer)?);
            for inner in face.inner() {
                parameter_area -= area_magnitude(self.signed_model_wire_double_area(*inner)?);
            }
            let surface_scale = match surface.exact_data() {
                SurfaceExactData::Plane { u, v, .. } => u
                    .cross(&v)
                    .norm_squared()
                    .sqrt()
                    .map_err(GeometryError::Numeric)?,
                SurfaceExactData::Cylinder { radius, .. } => radius,
                SurfaceExactData::Extrusion { profile, direction } => {
                    let profile = Curve3::from_exact_data(*profile)?;
                    return self.extrusion_face_area(
                        face,
                        outer,
                        &profile,
                        &direction,
                        &parameter_area,
                    );
                }
                data @ (SurfaceExactData::RationalBezier { .. }
                | SurfaceExactData::Nurbs { .. }) => {
                    return affine_tensor_face_area(&data, &parameter_area)
                        .map_err(build_error_geometry)?
                        .ok_or(GeometryError::UnsupportedMeasurement)
                        .map_err(QueryError::from);
                }
                _ => return Err(GeometryError::UnsupportedMeasurement.into()),
            };
            let double_area = parameter_area * surface_scale;
            (double_area / Real::from(2))
                .map_err(GeometryError::Numeric)
                .map_err(QueryError::from)
        })
    }

    pub(in crate::topology) fn extrusion_face_area(
        &self,
        face: &Face,
        outer: WireId,
        profile: &Curve3,
        direction: &Vector3,
        parameter_double_area: &Real,
    ) -> Result<Real, QueryError> {
        if let Some(scale) =
            extrusion_constant_area_scale(profile, direction).map_err(build_error_geometry)?
        {
            return (parameter_double_area * scale / Real::from(2))
                .map_err(GeometryError::Numeric)
                .map_err(QueryError::from);
        }
        if !face.inner().is_empty()
            || !(certified_monotone_line_curve_image(profile).map_err(build_error_geometry)?
                || certified_monotone_planar_extrusion_image(profile, direction)
                    .map_err(build_error_geometry)?)
        {
            return Err(GeometryError::UnsupportedMeasurement.into());
        }
        let Some((u_min, u_max, v_min, v_max)) = self.axis_aligned_parameter_rectangle(outer)?
        else {
            return Err(GeometryError::UnsupportedMeasurement.into());
        };
        let rectangle_double_area = Real::from(2) * (&u_max - &u_min) * (&v_max - &v_min);
        if !real_values_equal(parameter_double_area, &rectangle_double_area)
            .map_err(build_error_geometry)?
            || !profile.domain().contains(&u_min)?
            || !profile.domain().contains(&u_max)?
        {
            return Err(GeometryError::UnsupportedMeasurement.into());
        }
        let restricted = profile.subcurve(&u_min, &u_max)?;
        let start = restricted.evaluate_real(restricted.domain().start())?;
        let end = restricted.evaluate_real(restricted.domain().end())?;
        let swept_area = (&end - &start)
            .cross(direction)
            .norm_squared()
            .sqrt()
            .map_err(GeometryError::Numeric)?
            * (v_max - v_min);
        Ok(swept_area)
    }

    pub(in crate::topology) fn axis_aligned_parameter_rectangle(
        &self,
        wire: WireId,
    ) -> Result<Option<(Real, Real, Real, Real)>, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        if wire.edge_uses.len() != 4 {
            return Ok(None);
        }
        let mut u_values = Vec::with_capacity(2);
        let mut v_values = Vec::with_capacity(2);
        for edge_use in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use).expect("validated edge-use ID");
            let pcurve = self.pcurve(edge_use.pcurve).expect("validated pcurve ID");
            let Some(line) = pcurve.line_segment() else {
                return Ok(None);
            };
            let constant_u = real_values_equal(line.start().x(), line.end().x())
                .map_err(build_error_geometry)?;
            let constant_v = real_values_equal(line.start().y(), line.end().y())
                .map_err(build_error_geometry)?;
            if constant_u == constant_v {
                return Ok(None);
            }
            insert_sorted_real(&mut u_values, line.start().x()).map_err(build_error_geometry)?;
            insert_sorted_real(&mut u_values, line.end().x()).map_err(build_error_geometry)?;
            insert_sorted_real(&mut v_values, line.start().y()).map_err(build_error_geometry)?;
            insert_sorted_real(&mut v_values, line.end().y()).map_err(build_error_geometry)?;
        }
        if u_values.len() != 2 || v_values.len() != 2 {
            return Ok(None);
        }
        Ok(Some((
            u_values.remove(0),
            u_values.remove(0),
            v_values.remove(0),
            v_values.remove(0),
        )))
    }

    fn analytic_planar_face_area(
        &self,
        face: &Face,
        origin: &Point3,
        u: &Vector3,
        v: &Vector3,
    ) -> Result<Option<Real>, GeometryError> {
        let wires = || face.outer().into_iter().chain(face.inner().iter().copied());
        let mut has_arc = false;
        for wire in wires() {
            for use_id in &self.wire(wire).expect("validated wire").edge_uses {
                let edge_use = self.edge_use(*use_id).expect("validated use");
                let edge = self.edge(edge_use.edge).expect("validated edge");
                if edge.domain.start().try_real().is_none()
                    || edge.domain.end().try_real().is_none()
                {
                    return Ok(None);
                }
                match self.curve(edge.curve).expect("validated curve").kind() {
                    Curve3Kind::Line => {}
                    Curve3Kind::CircleArc | Curve3Kind::EllipseArc => has_arc = true,
                    _ => return Ok(None),
                }
            }
        }
        // Pure polygons keep the existing native exact product-sum path.
        if !has_arc {
            return Ok(None);
        }
        let normal = u.cross(v).normalize().map_err(GeometryError::Numeric)?;
        let mut area = Real::zero();
        for (index, wire) in wires().enumerate() {
            let edge_uses = &self.wire(wire).expect("validated wire").edge_uses;
            // A closed line integral is translation invariant. Centering it
            // on a retained conic avoids introducing large translated endpoint
            // terms that must cancel again after subdivision and replay.
            let origin = edge_uses
                .iter()
                .find_map(|use_id| {
                    let edge_use = self.edge_use(*use_id).expect("validated use");
                    let edge = self.edge(edge_use.edge).expect("validated edge");
                    self.curve(edge.curve)
                        .expect("validated curve")
                        .analytic_area_origin()
                })
                .unwrap_or(origin);
            let mut doubled = Real::zero();
            for use_id in edge_uses {
                let edge_use = self.edge_use(*use_id).expect("validated use");
                let edge = self.edge(edge_use.edge).expect("validated edge");
                let contribution = self
                    .curve(edge.curve)
                    .expect("validated curve")
                    .analytic_area_vector(
                        edge.domain.represented_start()?,
                        edge.domain.represented_end()?,
                        origin,
                    )?
                    .ok_or(GeometryError::UnsupportedMeasurement)?
                    .dot(&normal);
                match edge_use.direction {
                    Direction::Forward => doubled += contribution,
                    Direction::Reversed => doubled -= contribution,
                }
            }
            let magnitude = area_magnitude(doubled);
            if index == 0 {
                area += magnitude;
            } else {
                area -= magnitude;
            }
        }
        Ok(Some(
            (area / Real::from(2)).map_err(GeometryError::Numeric)?,
        ))
    }

    pub(in crate::topology) fn signed_model_wire_double_area(
        &self,
        wire: WireId,
    ) -> Result<Real, GeometryError> {
        match self.signed_wire_double_area(wire) {
            Ok(area) => Ok(area),
            Err(GeometryError::UnsupportedPcurveContour) => {
                let area = crate::error::classified(
                    self.build_model_wire_curve_path(wire)?
                        .boundary_loop()
                        .map_err(GeometryError::from)?
                        .signed_area(),
                )
                .map_err(GeometryError::from)?;
                let area = resolve_planar_classification(area)?
                    .ok_or(GeometryError::UnsupportedMeasurement)?;
                Ok(Real::from(2) * area)
            }
            Err(error) => Err(error),
        }
    }

    /// Returns the exact volume of a validated solid.
    pub fn solid_volume(&self, id: SolidId) -> Result<Real, QueryError> {
        crate::context::strict(|| {
            let solid = self.solid(id).ok_or(QueryError::InvalidReference {
                kind: EntityKind::Solid,
                index: id.index(),
            })?;
            if let Some(cylinder) = &self.data.certified_cylinders[id.index()] {
                let volume = Real::pi()
                    * &cylinder.radius
                    * &cylinder.radius
                    * (&cylinder.v_max - &cylinder.v_min);
                let Some(subtraction) = &cylinder.sphere_subtraction else {
                    return Ok(volume);
                };
                let excluded = match subtraction {
                    CertifiedCylinderSphereSubtraction::Void { radius, .. } => {
                        sphere_volume(radius)?
                    }
                    CertifiedCylinderSphereSubtraction::Component {
                        center,
                        radius,
                        side,
                    } => {
                        let center_parameter = (center - &cylinder.origin).dot(&cylinder.axis);
                        let antiderivative = |height: &Real| -> Result<Real, GeometryError> {
                            let cubic = (height * height * height / Real::from(3))
                                .map_err(GeometryError::Numeric)?;
                            Ok(Real::pi() * (radius * radius * height - cubic))
                        };
                        match side {
                            CertifiedCylinderSphereComponentSide::Lower => {
                                let upper = &cylinder.v_max - &center_parameter;
                                antiderivative(&upper)? - antiderivative(&-radius.clone())?
                            }
                            CertifiedCylinderSphereComponentSide::Upper => {
                                let lower = &cylinder.v_min - &center_parameter;
                                antiderivative(radius)? - antiderivative(&lower)?
                            }
                        }
                    }
                };
                return Ok(volume - excluded);
            }
            if let Some(sphere) = &self.data.certified_spheres[id.index()] {
                if let CertifiedSphereRegion::Axial(clip) = &sphere.region {
                    let antiderivative = |height: &Real| -> Result<Real, GeometryError> {
                        let cubic = (height * height * height / Real::from(3))
                            .map_err(GeometryError::Numeric)?;
                        Ok(Real::pi() * (&sphere.radius * &sphere.radius * height - cubic))
                    };
                    return Ok(antiderivative(&clip.max)? - antiderivative(&clip.min)?);
                }
                if let CertifiedSphereRegion::Radial(clip) = &sphere.region {
                    let axial_half_height = (&sphere.radius * &sphere.radius
                        - &clip.radius * &clip.radius)
                        .sqrt()
                        .map_err(GeometryError::Numeric)?;
                    let axial_cubic = &axial_half_height * &axial_half_height * axial_half_height;
                    let retained_cubic = match clip.side {
                        CertifiedSphereRadialSide::Inside => {
                            &sphere.radius * &sphere.radius * &sphere.radius - axial_cubic
                        }
                        CertifiedSphereRadialSide::Outside => axial_cubic,
                    };
                    return (Real::from(4) * Real::pi() * retained_cubic / Real::from(3))
                        .map_err(GeometryError::Numeric)
                        .map_err(QueryError::from);
                }
                if let CertifiedSphereRegion::FiniteCylinder(region) = &sphere.region {
                    let cylinder_volume = Real::pi()
                        * &region.radius
                        * &region.radius
                        * (&region.v_max - &region.v_min);
                    let sphere_volume = sphere_volume(&sphere.radius)?;
                    let overlap = sphere_finite_cylinder_overlap_volume(sphere, region)?;
                    return Ok(match region.operation {
                        CertifiedSphereFiniteCylinderOperation::Union => {
                            sphere_volume + cylinder_volume - overlap
                        }
                        CertifiedSphereFiniteCylinderOperation::Intersection => overlap,
                        CertifiedSphereFiniteCylinderOperation::Difference => {
                            sphere_volume - overlap
                        }
                    });
                }
                let mut volume = sphere_volume(&sphere.radius)?;
                for void in &sphere.voids {
                    volume -= match void {
                        CertifiedSphereVoid::Sphere { radius, .. } => sphere_volume(radius)?,
                        CertifiedSphereVoid::Cylinder(cylinder) => {
                            Real::pi()
                                * &cylinder.radius
                                * &cylinder.radius
                                * (&cylinder.v_max - &cylinder.v_min)
                        }
                    };
                }
                return Ok(volume);
            }
            if let Some(pair) = &self.data.certified_sphere_pairs[id.index()] {
                let first_volume = sphere_volume(&pair.first_radius)?;
                let second_volume = sphere_volume(&pair.second_radius)?;
                let overlap = sphere_pair_intersection_volume(pair)?;
                return Ok(match pair.kind {
                    CertifiedSpherePairKind::Union => first_volume + second_volume - overlap,
                    CertifiedSpherePairKind::Intersection => overlap,
                    CertifiedSpherePairKind::Difference => first_volume - overlap,
                });
            }
            if let Some(torus) = &self.data.certified_tori[id.index()] {
                return match &torus.region {
                    CertifiedTorusRegion::Whole => Ok(Real::from(2)
                        * Real::pi()
                        * Real::pi()
                        * &torus.major_radius
                        * &torus.minor_radius
                        * &torus.minor_radius),
                    CertifiedTorusRegion::Axial { min, max }
                    | CertifiedTorusRegion::SectorBand { min, max, .. } => {
                        let antiderivative = |height: &Real| -> Result<Real, GeometryError> {
                            // Normalize before taking the square root. The
                            // dimensionless disk segment is invariant under a
                            // similarity, avoiding newly mixed radicals in a
                            // scaled body's exact volume certificate.
                            let normalized =
                                (height / &torus.minor_radius).map_err(GeometryError::Numeric)?;
                            let normalized =
                                normalized.compact_quadratic_tower().unwrap_or(normalized);
                            let radial = (Real::one() - &normalized * &normalized)
                                .sqrt()
                                .map_err(GeometryError::Numeric)?;
                            let angle =
                                normalized.clone().asin().map_err(GeometryError::Numeric)?;
                            Ok(Real::from(2)
                                * Real::pi()
                                * &torus.major_radius
                                * &torus.minor_radius
                                * &torus.minor_radius
                                * (normalized * radial + angle))
                        };
                        let volume = antiderivative(max)? - antiderivative(min)?;
                        if let CertifiedTorusRegion::SectorBand { angular, .. } = &torus.region {
                            (volume * &angular.sweep / Real::tau())
                                .map_err(GeometryError::Numeric)
                                .map_err(QueryError::from)
                        } else {
                            Ok(volume)
                        }
                    }
                    CertifiedTorusRegion::LongitudinalHalf { .. } => Ok(Real::pi()
                        * Real::pi()
                        * &torus.major_radius
                        * &torus.minor_radius
                        * &torus.minor_radius),
                };
            }
            if let Some(frustum) = &self.data.certified_cone_frustums[id.index()] {
                let sine = frustum.semi_angle.clone().sin();
                let cosine = frustum.semi_angle.clone().cos();
                let min_radius = &frustum.v_min * &sine;
                let max_radius = &frustum.v_max * sine;
                let height = (&frustum.v_max - &frustum.v_min) * cosine;
                let volume = (Real::pi()
                    * height
                    * (&min_radius * &min_radius
                        + &min_radius * &max_radius
                        + &max_radius * &max_radius)
                    / Real::from(3))
                .map_err(GeometryError::Numeric)
                .map_err(QueryError::from)?;
                return match &frustum.region {
                    CertifiedConeFrustumRegion::Whole => Ok(volume),
                    CertifiedConeFrustumRegion::LongitudinalHalf { .. } => (volume / Real::from(2))
                        .map_err(GeometryError::Numeric)
                        .map_err(QueryError::from),
                    CertifiedConeFrustumRegion::Sector(angular) => (volume * &angular.sweep
                        / Real::tau())
                    .map_err(GeometryError::Numeric)
                    .map_err(QueryError::from),
                };
            }
            if let Some(revolution) = &self.data.certified_revolutions[id.index()] {
                let mut first_moment = revolution
                    .profile
                    .signed_x_first_moment()?
                    .ok_or(GeometryError::UnsupportedMeasurement)?;
                for void in &revolution.voids {
                    first_moment -= void
                        .signed_x_first_moment()?
                        .ok_or(GeometryError::UnsupportedMeasurement)?;
                }
                return Ok(Real::from(2) * Real::pi() * first_moment);
            }
            if let Some(loft) = &self.data.certified_lofts[id.index()] {
                let jacobian = loft.u.dot(&loft.v.cross(&loft.height_axis)).abs();
                return Ok(jacobian * &loft.parameter_volume);
            }
            if let Some(sweep) = &self.data.certified_curve_sweeps[id.index()] {
                let mut area = sweep
                    .profile
                    .signed_area()
                    .map_err(GeometryError::from)?
                    .ok_or(GeometryError::UnsupportedMeasurement)?
                    .abs();
                for hole in &sweep.holes {
                    area -= hole
                        .signed_area()
                        .map_err(GeometryError::from)?
                        .ok_or(GeometryError::UnsupportedMeasurement)?
                        .abs();
                }
                let path_start = sweep.path.evaluate_real(sweep.path.domain().start())?;
                let path_end = sweep.path.evaluate_real(sweep.path.domain().end())?;
                let u = Vector3::from(sweep.u_path.evaluate_real(sweep.u_path.domain().start())?);
                let v = Vector3::from(sweep.v_path.evaluate_real(sweep.v_path.domain().start())?);
                let progress = u.cross(&v).dot(&(path_end - path_start)).abs();
                return Ok(area * progress * &sweep.area_scale_integral);
            }
            if let Some(prism) = &self.data.certified_prisms[id.index()] {
                let area = match &prism.profile {
                    CertifiedPrismProfile::Paths { area, .. }
                    | CertifiedPrismProfile::CurveRegion { area, .. } => area.clone(),
                };
                let jacobian = prism.u.dot(&prism.v.cross(&prism.extrusion)).abs();
                return Ok(area * jacobian * (&prism.parameter_max - &prism.parameter_min));
            }
            let signed_six_volume = std::iter::once(&solid.outer)
                .chain(solid.voids.iter())
                .map(|shell| self.signed_shell_six_volume(*shell))
                .try_fold(Real::zero(), |sum, volume| {
                    volume.map(|volume| sum + volume)
                })?;
            (signed_six_volume / Real::from(6))
                .map_err(GeometryError::Numeric)
                .map_err(QueryError::from)
        })
    }

    pub(in crate::topology) fn signed_wire_double_area(
        &self,
        wire: WireId,
    ) -> Result<Real, GeometryError> {
        self.build_model_wire_contour(wire)?
            .signed_area()
            .map_err(GeometryError::from)?
            .map(|area| Real::from(2) * area)
            .ok_or(GeometryError::UnsupportedMeasurement)
    }

    pub(in crate::topology) fn signed_torus_wire_area(
        &self,
        wire: WireId,
        major_radius: &Real,
        minor_radius: &Real,
    ) -> Result<Real, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        let mut area = Real::zero();
        for edge_use_id in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use_id).expect("validated edge-use ID");
            let pcurve = self.pcurve(edge_use.pcurve).expect("validated pcurve ID");
            let line = pcurve
                .line_segment()
                .ok_or(GeometryError::UnsupportedMeasurement)?;
            let u_constant = real_values_equal(line.start().x(), line.end().x())
                .map_err(build_error_geometry)?;
            let v_constant = real_values_equal(line.start().y(), line.end().y())
                .map_err(build_error_geometry)?;
            if u_constant == v_constant {
                return Err(GeometryError::UnsupportedMeasurement);
            }
            if u_constant {
                let delta_v = line.end().y() - line.start().y();
                let delta_sin = line.end().y().clone().sin() - line.start().y().clone().sin();
                area += line.start().x().clone()
                    * minor_radius
                    * (major_radius * delta_v + minor_radius * delta_sin);
            }
        }
        Ok(area)
    }

    pub(in crate::topology) fn signed_sphere_wire_area(
        &self,
        wire: WireId,
        radius: &Real,
    ) -> Result<Real, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        let mut area = Real::zero();
        for edge_use_id in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use_id).expect("validated edge-use ID");
            let line = self
                .pcurve(edge_use.pcurve)
                .expect("validated pcurve ID")
                .line_segment()
                .ok_or(GeometryError::UnsupportedMeasurement)?;
            if real_values_equal(line.start().y(), line.end().y()).map_err(build_error_geometry)? {
                area += radius
                    * radius
                    * line.start().y().clone().sin()
                    * (line.end().x() - line.start().x());
            } else if !real_values_equal(line.start().x(), line.end().x())
                .map_err(build_error_geometry)?
            {
                return Err(GeometryError::UnsupportedMeasurement);
            }
        }
        Ok(area)
    }

    pub(in crate::topology) fn signed_cone_wire_area(
        &self,
        wire: WireId,
        semi_angle: &Real,
    ) -> Result<Real, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        let mut doubled_area = Real::zero();
        for edge_use_id in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use_id).expect("validated edge-use ID");
            let pcurve = self.pcurve(edge_use.pcurve).expect("validated pcurve ID");
            let line = pcurve
                .line_segment()
                .ok_or(GeometryError::UnsupportedMeasurement)?;
            let u_constant = real_values_equal(line.start().x(), line.end().x())
                .map_err(build_error_geometry)?;
            let v_constant = real_values_equal(line.start().y(), line.end().y())
                .map_err(build_error_geometry)?;
            if u_constant == v_constant {
                return Err(GeometryError::UnsupportedMeasurement);
            }
            if u_constant {
                doubled_area += line.start().x().clone()
                    * semi_angle.clone().sin()
                    * (line.end().y() * line.end().y() - line.start().y() * line.start().y());
            }
        }
        (doubled_area / Real::from(2)).map_err(GeometryError::Numeric)
    }

    pub(in crate::topology) fn signed_shell_six_volume(
        &self,
        shell: ShellId,
    ) -> Result<Real, GeometryError> {
        let mut sum = Real::zero();
        for face_id in &self.shell(shell).expect("validated shell ID").faces {
            let face = self.face(*face_id).expect("validated face ID");
            for wire_id in face.boundary_wires() {
                let wire = self.wire(*wire_id).expect("validated wire ID");
                let points = wire.edge_uses.iter().map(|edge_use| {
                    let start = self.directed_vertices(*edge_use).0;
                    Ok::<_, GeometryError>(self.vertex(start).expect("validated vertex ID").point())
                });
                sum += match wire_geometry::scalar_fan_six_volume(points)? {
                    Some(volume) => volume,
                    None => wire_geometry::planar_wire_six_volume(
                        self.surface(face.surface).expect("validated surface ID"),
                        &self.build_model_wire_curve_path(*wire_id)?,
                    )?,
                };
            }
        }
        Ok(sum)
    }
}

/// Consume a strict sign before introducing sqrt(area^2), which can obscure
/// cancellation between small analytic patches. Compact only through the
/// native scalar recognizer. An unavailable proof preserves the original
/// exact absolute value; it never selects an orientation by approximation.
fn area_magnitude(signed: Real) -> Real {
    let signed = signed.compact_pi_laurent().unwrap_or(signed);
    match compare_reals(&signed, &Real::zero(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Less,
            ..
        } => -signed,
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Equal,
            ..
        } => Real::zero(),
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater,
            ..
        } => signed,
        PredicateOutcome::Unknown { .. } => signed.abs(),
    }
}
