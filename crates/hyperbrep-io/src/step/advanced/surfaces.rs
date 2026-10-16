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

impl Builder {
    pub(super) fn surface(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
    ) -> Result<FaceSurface, Error> {
        let entity = resolver.entity(id)?;
        let site = Site::entity(entity);
        let record = entity.components().next().expect("syntax component");
        if entity.is_complex() || !matches!(record.name(), "PLANE" | "CYLINDRICAL_SURFACE") {
            return Err(site.error(ErrorKind::Unsupported("advanced support surface")));
        }
        if let Some(surface) = self.by_surface.get(&id) {
            return Ok(surface.clone());
        }
        let mut args = Args::new(record, site);
        resolver.text(args.next()?, site)?;
        let geometry = resolver.placement(args.reference()?, scale)?;
        let (native, cylinder) = if record.name() == "CYLINDRICAL_SURFACE" {
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
            (surface, Some(Cylinder { axis, iso }))
        } else {
            (geometry.surface.clone(), None)
        };
        args.finish()?;
        let key = self
            .model
            .surface(native)
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let surface = FaceSurface {
            geometry,
            key,
            cylinder,
        };
        cache(&mut self.by_surface, id, surface.clone(), site)?;
        Ok(surface)
    }

    pub(super) fn plane(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
    ) -> Result<FaceSurface, Error> {
        let surface = self.surface(resolver, id, scale)?;
        if surface.cylinder.is_some() {
            return Err(Site::entity(resolver.entity(id)?)
                .error(ErrorKind::Unsupported("advanced pcurve support surface")));
        }
        Ok(surface)
    }
}

impl FaceSurface {
    pub(super) fn boundary_segment(
        &self,
        edge: &Edge,
        from: &Point3,
        to: &Point3,
        forward: bool,
        previous: Option<&Point2>,
        site: Site,
    ) -> Result<(Segment2, ParameterCorrespondence), Error> {
        let plane = &self.geometry;
        let direction = if forward { Real::one() } else { -Real::one() };
        let segment = if let Some(cylinder) = &self.cylinder {
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
        } else {
            let project = |point: &Point3| {
                let delta = point - &plane.origin;
                Point2::new(delta.dot(&plane.x), delta.dot(&plane.y))
            };
            if let Some(circle) = &edge.circle {
                let normal = aligned_normals(&circle.normal, &plane.x.cross(&plane.y), site)?;
                let clockwise = order(site, &(&circle.sweep * normal * &direction), &Real::zero())?
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
        };
        let correspondence = if matches!(segment, Segment2::Arc(_)) {
            ParameterCorrespondence::angular_sweep()
        } else {
            let domain = edge
                .circle
                .as_ref()
                .map(|circle| circle.domain.clone())
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

fn aligned_normals(a: &Vector3, b: &Vector3, site: Site) -> Result<Real, Error> {
    let sign = a.dot(b);
    if order(site, &sign, &Real::one())? == Ordering::Equal
        || order(site, &sign, &-Real::one())? == Ordering::Equal
    {
        Ok(sign)
    } else {
        Err(site.invalid("circle plane is not aligned with its face support"))
    }
}
