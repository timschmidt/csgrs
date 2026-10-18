//! Native support surfaces and continuous periodic charts for direct 3D curves.
use super::*;
use hyperbrep::Surface;
use hypercurve::CircularArc2;

#[derive(Clone)]
pub(super) struct Cylinder {
    axis: Vector3,
    /// Exact zero-height circle used by the native point-location query.
    iso: Curve3,
}

#[derive(Clone)]
pub(super) enum Support {
    Plane,
    Cylinder(Cylinder),
    Torus(torus::Torus),
    Cone(hyperbrep::ConicalRevolutionChart),
    Extrusion(Surface),
}

impl FaceSurface {
    pub(super) fn is_plane(&self) -> bool {
        matches!(self.support, Support::Plane)
    }

    pub(super) fn angular_coordinates(&self) -> [bool; 2] {
        match self.support {
            Support::Plane | Support::Extrusion(_) => [false, false],
            Support::Cylinder(_) => [true, false],
            Support::Torus(_) => [true, true],
            Support::Cone(_) => [true, false],
        }
    }
}

impl Builder {
    pub(super) fn surface(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
        branch: Option<ConeBranch>,
    ) -> Result<FaceSurface, Error> {
        let entity = resolver.entity(id)?;
        let site = Site::entity(entity);
        let record = entity.components().next().expect("syntax component");
        if !entity.is_complex() && record.name() == "SURFACE_OF_REVOLUTION" {
            return self.conical_surface(resolver, id, scale, branch, site);
        }
        if !entity.is_complex() && record.name() == "SURFACE_OF_LINEAR_EXTRUSION" {
            if let Some(surface) = self.by_surface.get(&(id, None)) {
                return Ok(surface.clone());
            }
            let mut args = Args::new(record, site);
            resolver.text(args.next()?, site)?;
            let profile = resolver.spline3(args.reference()?, scale)?;
            let direction = resolver.vector(args.reference()?, 3)? * scale;
            args.finish()?;
            let native = Surface::extrusion(profile, direction)
                .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
            let key = self
                .model
                .surface(native.clone())
                .map_err(|e| site.error(ErrorKind::Build(e)))?;
            let surface = FaceSurface {
                geometry: None,
                key,
                support: Support::Extrusion(native),
                parameters: SourceSurfaceParameters {
                    surface: id,
                    origin: hyperbrep::Point2::origin(),
                    u: hyperbrep::Vector2::x(),
                    v: hyperbrep::Vector2::y(),
                },
            };
            cache(&mut self.by_surface, (id, None), surface.clone(), site)?;
            return Ok(surface);
        }
        if entity.is_complex()
            || !matches!(
                record.name(),
                "PLANE" | "CYLINDRICAL_SURFACE" | "TOROIDAL_SURFACE"
            )
        {
            return Err(site.error(ErrorKind::Unsupported("advanced support surface")));
        }
        if let Some(surface) = self.by_surface.get(&(id, None)) {
            return Ok(surface.clone());
        }
        let mut args = Args::new(record, site);
        resolver.text(args.next()?, site)?;
        let geometry = resolver.placement(args.reference()?, scale)?;
        let (native, support) = if record.name() == "CYLINDRICAL_SURFACE" {
            self.angular_scale(site)?;
            let radius = resolver.scalar(args.next()?, site)? * scale;
            let axis = geometry.x.cross(&geometry.y);
            let surface = Surface::cylinder(
                geometry.origin.clone(),
                geometry.x.clone(),
                geometry.y.clone(),
                axis.clone(),
                radius.clone(),
            )
            .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
            let iso = Curve3::circle_arc(
                geometry.origin.clone(),
                geometry.x.clone(),
                geometry.y.clone(),
                radius,
                Real::zero(),
                Real::tau(),
            )
            .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
            (surface, Support::Cylinder(Cylinder { axis, iso }))
        } else if record.name() == "TOROIDAL_SURFACE" {
            self.angular_scale(site)?;
            let major = resolver.scalar(args.next()?, site)? * scale;
            let minor = resolver.scalar(args.next()?, site)? * scale;
            let (surface, torus) = torus::Torus::new(&geometry, major, minor, site)?;
            (surface, Support::Torus(torus))
        } else {
            (geometry.surface.clone(), Support::Plane)
        };
        args.finish()?;
        let key = self
            .model
            .surface(native)
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let inverse =
            |value: &Real| (Real::one() / value).map_err(|e| site.error(ErrorKind::Numeric(e)));
        let u_scale = if matches!(support, Support::Plane) {
            scale
        } else {
            self.angular_scale(site)?
        };
        let v_scale = if matches!(support, Support::Torus(_)) {
            self.angular_scale(site)?
        } else {
            scale
        };
        let surface = FaceSurface {
            geometry: Some(geometry),
            key,
            support,
            parameters: SourceSurfaceParameters {
                surface: id,
                origin: hyperbrep::Point2::origin(),
                u: hyperbrep::Vector2::x() * inverse(u_scale)?,
                v: hyperbrep::Vector2::y() * inverse(v_scale)?,
            },
        };
        cache(&mut self.by_surface, (id, None), surface.clone(), site)?;
        Ok(surface)
    }
}

impl FaceSurface {
    pub(super) fn spline_boundary_curve(
        &self,
        curve: &Curve3,
        forward: bool,
        site: Site,
    ) -> Result<(Curve2, ParameterCorrespondence), Error> {
        let plane = self
            .geometry
            .as_ref()
            .filter(|_| self.is_plane())
            .ok_or_else(|| site.error(ErrorKind::Unsupported("spline pcurve on this support")))?;
        let directed = if forward {
            curve.clone()
        } else {
            curve
                .reversed()
                .map_err(|e| site.error(ErrorKind::Geometry(e)))?
        };
        let materialized = plane
            .surface
            .planar_pcurve(&directed)
            .map_err(|e| site.error(ErrorKind::Geometry(e)))?
            .ok_or_else(|| site.invalid("spline is not contained in face plane"))?;
        let (scale, offset) = materialized
            .correspondence()
            .affine_coefficients()
            .ok_or_else(|| {
                site.error(ErrorKind::Unsupported(
                    "nonaffine spline pcurve correspondence",
                ))
            })?;
        let (scale, offset) = if forward {
            (scale.clone(), offset.clone())
        } else {
            (
                -scale,
                curve.domain().start() + curve.domain().end() - offset,
            )
        };
        let correspondence = ParameterCorrespondence::affine(scale, offset)
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        Ok((materialized.curve().clone(), correspondence))
    }

    pub(super) fn boundary_segment(
        &self,
        edge: &Edge,
        from: &Point3,
        to: &Point3,
        forward: bool,
        previous: Option<&Point2>,
        site: Site,
    ) -> Result<(Segment2, ParameterCorrespondence), Error> {
        let direction = if forward { Real::one() } else { -Real::one() };
        let segment = if let Support::Extrusion(surface) = &self.support {
            let locate = |point: &Point3| {
                let (u, v) = surface
                    .extrusion_parameters_of(point)
                    .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
                Ok(Point2::new(spline::unique_parameter(u, site)?, v))
            };
            let from = locate(from)?;
            let to = locate(to)?;
            if order(site, from.x(), to.x())? != Ordering::Equal
                && order(site, from.y(), to.y())? != Ordering::Equal
            {
                return Err(site.error(ErrorKind::Unsupported(
                    "nonisoparametric extrusion boundary",
                )));
            }
            Segment2::Line(
                LineSeg2::try_new(from, to)
                    .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?,
            )
        } else if edge.spline.is_some() {
            return Err(site.error(ErrorKind::Unsupported(
                "spline boundary segment on this support",
            )));
        } else if let Support::Cone(chart) = &self.support {
            cone::boundary_segment(chart, edge, from, to, &direction, previous, site)?
        } else {
            let plane = self.geometry.as_ref().expect("elementary support frame");
            if let Support::Cylinder(cylinder) = &self.support {
                let from_height = (from - &plane.origin).dot(&cylinder.axis);
                let to_height = (to - &plane.origin).dot(&cylinder.axis);
                let angle = if let Some(previous) = previous {
                    // The loop has already checked shared vertex identity. The
                    // preceding use and native image validation certify this lift.
                    previous.x().clone()
                } else {
                    let projected = from.clone() + cylinder.axis.clone() * -&from_height;
                    circles::circle_parameter(&cylinder.iso, &projected, site)?
                };
                let end_angle = if let Some(circle) = &edge.circle {
                    let normal = aligned_normals(&circle.normal, &cylinder.axis, site)?;
                    &angle + &circle.sweep * normal * &direction
                } else {
                    angle.clone()
                };
                Segment2::Line(
                    LineSeg2::try_new(
                        Point2::new(angle, from_height),
                        Point2::new(end_angle, to_height),
                    )
                    .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?,
                )
            } else if let Support::Torus(torus) = &self.support {
                torus.boundary_segment(plane, edge, from, &direction, previous, site)?
            } else {
                let project = |point: &Point3| {
                    let delta = point - &plane.origin;
                    Point2::new(delta.dot(&plane.x), delta.dot(&plane.y))
                };
                if let Some(circle) = &edge.circle {
                    let normal = aligned_normals(&circle.normal, &plane.x.cross(&plane.y), site)?;
                    let clockwise =
                        order(site, &(&circle.sweep * normal * &direction), &Real::zero())?
                            == Ordering::Less;
                    Segment2::Arc(
                        CircularArc2::try_from_center(
                            project(from),
                            project(to),
                            project(&circle.center),
                            clockwise,
                        )
                        .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?,
                    )
                } else {
                    Segment2::Line(
                        LineSeg2::try_new(project(from), project(to))
                            .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?,
                    )
                }
            }
        };
        let correspondence = if matches!(segment, Segment2::Arc(_)) {
            ParameterCorrespondence::angular_sweep()
        } else {
            let domain = edge
                .circle
                .as_ref()
                .map(|circle| circle.domain.clone())
                .or_else(|| edge.spline.as_ref().map(|curve| curve.domain().clone()))
                .unwrap_or_else(ParameterDomain::unit);
            ParameterCorrespondence::affine(
                (domain.end() - domain.start()) * direction,
                if forward {
                    domain.start().clone()
                } else {
                    domain.end().clone()
                },
            )
            .map_err(|e| site.error(ErrorKind::Build(e)))?
        };
        Ok((segment, correspondence))
    }
}

pub(super) fn aligned_normals(a: &Vector3, b: &Vector3, site: Site) -> Result<Real, Error> {
    let sign = a.dot(b);
    if order(site, &sign, &Real::one())? == Ordering::Equal
        || order(site, &sign, &-Real::one())? == Ordering::Equal
    {
        Ok(sign)
    } else {
        Err(site.invalid("circle plane is not aligned with its face support"))
    }
}
