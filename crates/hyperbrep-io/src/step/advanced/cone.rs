//! STEP syntax and unit handling around the native conical-revolution proof.
use super::*;
use crate::part21::{ValueKind, Values};
use hyperbrep::{ConicalRevolutionChart, Vector2};

impl Builder {
    /// Successful proofs live only for this body import. Source identity, units
    /// and policy are fixed by that import; failures are never cached. The
    /// separate surface cache also keys the selected branch. Parser visit and
    /// entity limits bound entry counts, not retained scalar heap bytes.
    pub(super) fn conical_source(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
    ) -> Result<Option<ConicalRevolution>, Error> {
        let entity = resolver.entity(id)?;
        let site = Site::entity(entity);
        let record = entity.components().next().expect("syntax component");
        if entity.is_complex() || record.name() != "SURFACE_OF_REVOLUTION" {
            return Ok(None);
        }
        if let Some(source) = self.by_revolution.get(&id) {
            return Ok(Some(source.clone()));
        }
        self.angular_scale(site)?;
        let mut args = Args::new(record, site);
        resolver.text(args.next()?, site)?;
        let line = resolver.line3(args.reference()?, scale)?;
        let placement = args.reference()?;
        args.finish()?;
        let mut args = resolver.simple(placement, "AXIS1_PLACEMENT")?;
        let placement_site = args.site;
        resolver.text(args.next()?, placement_site)?;
        let point = resolver.point(args.reference()?)?;
        let point = Point3::new(point.x * scale, point.y * scale, point.z * scale);
        let axis = match args.optional_reference()? {
            Some(id) => resolver.vector_direction(id, 3)?,
            None => Vector3::z(),
        };
        args.finish()?;
        let source = ConicalRevolution::from_line(line.origin, line.velocity, point, axis)
            .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
        cache(&mut self.by_revolution, id, source.clone(), site)?;
        Ok(Some(source))
    }

    /// Only vertex coordinates are read here. Constructing an edge could
    /// recurse through a supplied PCURVE on this same support. Each lookup is
    /// charged to the ordinary resolver limits; edge-use admission is charged
    /// once, by the actual loop builder.
    pub(super) fn conical_branch(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        source: &ConicalRevolution,
        bounds: Values<'_, '_>,
        scale: &Real,
        site: Site,
    ) -> Result<ConeBranch, Error> {
        for bound in bounds {
            let entity = resolver.entity(reference(bound, site)?)?;
            let bound_site = Site::entity(entity);
            let record = entity.components().next().expect("syntax component");
            if entity.is_complex() || !matches!(record.name(), "FACE_BOUND" | "FACE_OUTER_BOUND") {
                return Err(bound_site.error(ErrorKind::Expected("face bound")));
            }
            let mut args = Args::new(record, bound_site);
            resolver.text(args.next()?, bound_site)?;
            let loop_id = args.reference()?;
            args.boolean()?;
            args.finish()?;
            let mut args = resolver.simple(loop_id, "EDGE_LOOP")?;
            let loop_site = args.site;
            resolver.text(args.next()?, loop_site)?;
            let edges = args.list()?;
            args.finish()?;
            for value in edges {
                let mut args = resolver.simple(reference(value, loop_site)?, "ORIENTED_EDGE")?;
                let edge_site = args.site;
                resolver.text(args.next()?, edge_site)?;
                if args.next()?.kind() != ValueKind::Derived
                    || args.next()?.kind() != ValueKind::Derived
                {
                    return Err(
                        edge_site.error(ErrorKind::Expected("derived oriented-edge endpoints"))
                    );
                }
                let edge = args.reference()?;
                args.boolean()?;
                args.finish()?;
                let mut args = resolver.simple(edge, "EDGE_CURVE")?;
                resolver.text(args.next()?, args.site)?;
                let vertices = [args.reference()?, args.reference()?];
                args.reference()?;
                args.boolean()?;
                args.finish()?;
                for id in vertices {
                    let vertex = self.vertex(resolver, id, scale)?;
                    match source.branch_at(&vertex.point) {
                        Ok(branch) => return Ok(branch),
                        Err(GeometryError::SingularSurfaceParameter) => {}
                        Err(e) => return Err(site.error(ErrorKind::Geometry(e))),
                    }
                }
            }
        }
        Err(site.error(ErrorKind::Geometry(GeometryError::SingularSurfaceParameter)))
    }

    pub(super) fn conical_surface(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
        branch: Option<ConeBranch>,
        site: Site,
    ) -> Result<FaceSurface, Error> {
        let branch =
            branch.ok_or_else(|| site.invalid("cone branch requires a non-apex witness"))?;
        if let Some(surface) = self.by_surface.get(&(id, Some(branch))) {
            return Ok(surface.clone());
        }
        let source = self
            .conical_source(resolver, id, scale)?
            .expect("revolution dispatch");
        let chart = source
            .chart(branch)
            .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
        let origin = chart
            .source_parameters(&hyperbrep::Point2::origin())
            .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
        let u = (chart.angular_scale() / self.angular_scale(site)?)
            .map_err(|e| site.error(ErrorKind::Numeric(e)))?;
        let v =
            (Real::one() / chart.slant_scale()).map_err(|e| site.error(ErrorKind::Numeric(e)))?;
        let key = self
            .model
            .surface(chart.surface().clone())
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let surface = FaceSurface {
            geometry: None,
            key,
            support: surfaces::Support::Cone(chart),
            parameters: SourceSurfaceParameters {
                surface: id,
                origin,
                u: Vector2::new([u, Real::zero()]),
                v: Vector2::new([Real::zero(), v]),
            },
        };
        cache(
            &mut self.by_surface,
            (id, Some(branch)),
            surface.clone(),
            site,
        )?;
        Ok(surface)
    }
}

pub(super) fn boundary_segment(
    chart: &ConicalRevolutionChart,
    edge: &Edge,
    from: &Point3,
    to: &Point3,
    direction: &Real,
    previous: Option<&Point2>,
    site: Site,
) -> Result<Segment2, Error> {
    let parameters = |point| match chart.canonical_parameters_of(point) {
        Ok(uv) => Ok(Some(uv)),
        Err(GeometryError::SingularSurfaceParameter) => Ok(None),
        Err(e) => Err(site.error(ErrorKind::Geometry(e))),
    };
    let first = parameters(from)?;
    let second = parameters(to)?;
    let witness = first
        .as_ref()
        .or(second.as_ref())
        .ok_or_else(|| site.invalid("apex-only cone edge"))?;
    let angle = previous
        .map(|uv| uv.x().clone())
        .unwrap_or_else(|| witness.x.clone());
    let end_angle = if let Some(circle) = &edge.circle {
        let data = chart
            .surface()
            .conical_geometry()
            .expect("native cone chart");
        let normal = surfaces::aligned_normals(&circle.normal, data.axis, site)?;
        &angle + &circle.sweep * normal * direction
    } else {
        angle.clone()
    };
    let from_v = first.map(|uv| uv.y).unwrap_or_else(Real::zero);
    let to_v = second.map(|uv| uv.y).unwrap_or_else(Real::zero);
    // This is a candidate parameter-space line, not a whole-edge proof.
    // The model builder must prove its image against the retained 3D curve.
    Ok(Segment2::Line(
        LineSeg2::try_new(Point2::new(angle, from_v), Point2::new(end_angle, to_v))
            .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?,
    ))
}
