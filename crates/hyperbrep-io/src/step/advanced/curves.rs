//! Whole-image linear certificates and source parameterizations, including PCURVE.
use super::super::decode::{component, parts};
use super::*;
use crate::part21::ValueKind;

struct Line {
    source: u64,
    origin: Point3,
    velocity: Vector3,
}
impl Line {
    fn parameter(&self, point: &Point3, site: Site) -> Result<Real, Error> {
        let delta = point - &self.origin;
        let parameter = (delta.dot(&self.velocity) / self.velocity.dot(&self.velocity))
            .map_err(|e| site.error(ErrorKind::Numeric(e)))?;
        if !same_vector(site, &delta, &(self.velocity.clone() * &parameter))? {
            return Err(site.invalid("point is not on the exact source line"));
        }
        Ok(parameter)
    }
    fn same_oriented_image(&self, other: &Self, site: Site) -> Result<(), Error> {
        self.parameter(&other.origin, site)?;
        let scale = (other.velocity.dot(&self.velocity) / self.velocity.dot(&self.velocity))
            .map_err(|e| site.error(ErrorKind::Numeric(e)))?;
        if !same_vector(site, &other.velocity, &(self.velocity.clone() * &scale))?
            || order(site, &scale, &Real::zero())? != Ordering::Greater
        {
            return Err(site.invalid("surface-curve representations disagree in image or sense"));
        }
        Ok(())
    }
    fn parameters(
        &self,
        start: &Point3,
        end: &Point3,
        site: Site,
    ) -> Result<SourceCurveParameters, Error> {
        Ok(SourceCurveParameters {
            curve: self.source,
            start: self.parameter(start, site)?,
            end: self.parameter(end, site)?,
        })
    }
}
impl Resolver<'_, '_, '_> {
    fn vector(&mut self, id: u64, dimension: usize) -> Result<Vector3, Error> {
        let mut args = self.simple(id, "VECTOR")?;
        let site = args.site;
        self.text(args.next()?, site)?;
        let direction = args.reference()?;
        let magnitude = self.scalar(args.next()?, site)?;
        args.finish()?;
        if order(site, &magnitude, &Real::zero())? != Ordering::Greater {
            return Err(site.invalid("line vector must have positive magnitude"));
        }
        let mut args = self.simple(direction, "DIRECTION")?;
        let site = args.site;
        self.text(args.next()?, site)?;
        let mut values = args.list()?;
        let mut coordinates = [Real::zero(), Real::zero(), Real::zero()];
        for coordinate in coordinates.iter_mut().take(dimension) {
            let value = values
                .next()
                .ok_or_else(|| site.error(ErrorKind::Expected("direction dimension")))?;
            *coordinate = self.scalar(value, site)?;
        }
        if values.next().is_some() {
            return Err(site.error(ErrorKind::Expected("direction dimension")));
        }
        args.finish()?;
        Ok(Vector3::new(coordinates)
            .normalize_checked()
            .map_err(|e| site.error(ErrorKind::Numeric(e)))?
            * magnitude)
    }
    fn point2(&mut self, id: u64) -> Result<Point2, Error> {
        let mut args = self.simple(id, "CARTESIAN_POINT")?;
        let site = args.site;
        self.text(args.next()?, site)?;
        let mut values = args.list()?;
        let mut next = || {
            self.scalar(
                values
                    .next()
                    .ok_or_else(|| site.error(ErrorKind::Expected("two coordinates")))?,
                site,
            )
        };
        let result = Point2::new(next()?, next()?);
        if values.next().is_some() {
            return Err(site.error(ErrorKind::Expected("two coordinates")));
        }
        args.finish()?;
        Ok(result)
    }
    fn parametric_context(&mut self, id: u64) -> Result<(), Error> {
        let entity = self.entity(id)?;
        let site = Site::entity(entity);
        let record = entity.components().next().expect("syntax component");
        if !entity.is_complex() && record.name() == "PARAMETRIC_REPRESENTATION_CONTEXT" {
            let mut args = Args::new(record, site);
            self.text(args.next()?, site)?;
            self.text(args.next()?, site)?;
            return args.finish();
        }
        parts(
            entity,
            &[
                "PARAMETRIC_REPRESENTATION_CONTEXT",
                "REPRESENTATION_CONTEXT",
            ],
            &["GEOMETRIC_REPRESENTATION_CONTEXT"],
        )?;
        if let Some(record) = component(entity, "GEOMETRIC_REPRESENTATION_CONTEXT") {
            let mut args = Args::new(record, site);
            let dimension = args.next()?;
            if !matches!(dimension.kind(), ValueKind::Integer(_)) {
                return Err(site.error(ErrorKind::Expected("integer context dimension")));
            }
            if self.scalar(dimension, site)? != Real::from(2) {
                return Err(site.invalid("pcurve context must have dimension two"));
            }
            args.finish()?;
        }
        Args::new(
            component(entity, "PARAMETRIC_REPRESENTATION_CONTEXT").expect("checked component"),
            site,
        )
        .finish()?;
        let mut args = Args::new(
            component(entity, "REPRESENTATION_CONTEXT").expect("checked component"),
            site,
        );
        self.text(args.next()?, site)?;
        self.text(args.next()?, site)?;
        args.finish()
    }
    fn line3(&mut self, id: u64, scale: &Real) -> Result<Line, Error> {
        let entity = self.entity(id)?;
        let site = Site::entity(entity);
        let record = entity.components().next().expect("syntax component");
        if entity.is_complex() || record.name() != "LINE" {
            return Err(site.error(ErrorKind::Unsupported("advanced spatial curve")));
        }
        let mut args = Args::new(record, site);
        self.text(args.next()?, site)?;
        let point = self.point(args.reference()?)?;
        let velocity = self.vector(args.reference()?, 3)? * scale;
        args.finish()?;
        Ok(Line {
            source: id,
            origin: Point3::new(&point.x * scale, &point.y * scale, &point.z * scale),
            velocity,
        })
    }
}
impl Builder {
    fn pcurve(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
    ) -> Result<Line, Error> {
        let mut args = resolver.simple(id, "PCURVE")?;
        let site = args.site;
        resolver.text(args.next()?, site)?;
        let plane = self.plane(resolver, args.reference()?, scale)?.geometry;
        let representation = args.reference()?;
        args.finish()?;
        let mut args = resolver.simple(representation, "DEFINITIONAL_REPRESENTATION")?;
        let site = args.site;
        resolver.text(args.next()?, site)?;
        let mut items = args.list()?;
        let curve = reference(
            items
                .next()
                .ok_or_else(|| site.error(ErrorKind::Expected("one pcurve basis curve")))?,
            site,
        )?;
        if items.next().is_some() {
            return Err(site.error(ErrorKind::Expected("one pcurve basis curve")));
        }
        resolver.parametric_context(args.reference()?)?;
        args.finish()?;
        let entity = resolver.entity(curve)?;
        let site = Site::entity(entity);
        let record = entity.components().next().expect("syntax component");
        if entity.is_complex() || record.name() != "LINE" {
            return Err(site.error(ErrorKind::Unsupported("advanced pcurve basis")));
        }
        let mut args = Args::new(record, site);
        resolver.text(args.next()?, site)?;
        let point = resolver.point2(args.reference()?)?;
        let velocity = resolver.vector(args.reference()?, 2)?;
        args.finish()?;
        Ok(Line {
            source: id,
            origin: plane.origin
                + plane.x.clone() * (point.x() * scale)
                + plane.y.clone() * (point.y() * scale),
            velocity: plane.x * (&velocity[0] * scale) + plane.y * (&velocity[1] * scale),
        })
    }
    pub(super) fn edge(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
    ) -> Result<Edge, Error> {
        let mut args = resolver.simple(id, "EDGE_CURVE")?;
        if let Some(edge) = self.by_edge.get(&id) {
            return Ok(edge.clone());
        }
        let site = args.site;
        resolver.text(args.next()?, site)?;
        let start = args.reference()?;
        let end = args.reference()?;
        let geometry = args.reference()?;
        let same_sense = args.boolean()?;
        args.finish()?;
        let first = self.vertex(resolver, start, scale)?;
        let second = self.vertex(resolver, end, scale)?;
        let entity = resolver.entity(geometry)?;
        let record = entity.components().next().expect("syntax component");
        let geometry_site = Site::entity(entity);
        if entity.is_complex() {
            return Err(geometry_site.error(ErrorKind::Unsupported("complex edge geometry")));
        }
        let mut parameters = Vec::new();
        let master;
        if record.name() == "SURFACE_CURVE" {
            let mut args = Args::new(record, geometry_site);
            resolver.text(args.next()?, geometry_site)?;
            let line = resolver.line3(args.reference()?, scale)?;
            let mut pcurves: [Option<Line>; 2] = [None, None];
            let mut count = 0;
            for value in args.list()? {
                if count == 2 {
                    return Err(
                        geometry_site.invalid("surface curve requires one or two associations")
                    );
                }
                let associated = reference(value, geometry_site)?;
                // ADVANCED_FACE WR9 requires every association to be a PCURVE.
                let pcurve = self.pcurve(resolver, associated, scale)?;
                line.same_oriented_image(&pcurve, geometry_site)?;
                pcurves[count] = Some(pcurve);
                count += 1;
            }
            if count == 0 {
                return Err(geometry_site.invalid("surface curve requires one or two associations"));
            }
            let chosen = match args.next()?.kind() {
                ValueKind::Enumeration("CURVE_3D") => &line,
                ValueKind::Enumeration("PCURVE_S1") => {
                    pcurves[0].as_ref().expect("nonempty associations")
                }
                ValueKind::Enumeration("PCURVE_S2") => pcurves[1]
                    .as_ref()
                    .ok_or_else(|| geometry_site.invalid("missing master pcurve"))?,
                _ => {
                    return Err(geometry_site
                        .error(ErrorKind::Expected("surface-curve master representation")));
                }
            };
            args.finish()?;
            master = chosen.source;
            let mut master_parameters = chosen.parameters(&first.point, &second.point, site)?;
            master_parameters.curve = geometry;
            push(&mut parameters, master_parameters).map_err(|kind| site.error(kind))?;
            push(
                &mut parameters,
                line.parameters(&first.point, &second.point, site)?,
            )
            .map_err(|kind| site.error(kind))?;
            for pcurve in pcurves.iter().flatten() {
                push(
                    &mut parameters,
                    pcurve.parameters(&first.point, &second.point, site)?,
                )
                .map_err(|kind| site.error(kind))?;
            }
        } else {
            let line = resolver.line3(geometry, scale)?;
            master = line.source;
            push(
                &mut parameters,
                line.parameters(&first.point, &second.point, site)?,
            )
            .map_err(|kind| site.error(kind))?;
        }
        let sense = order(site, &parameters[0].start, &parameters[0].end)?;
        if sense
            != if same_sense {
                Ordering::Less
            } else {
                Ordering::Greater
            }
        {
            return Err(site.invalid("edge same_sense disagrees with source curve parameters"));
        }
        let curve = Curve3::line(first.point, second.point)
            .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
        let curve = self
            .model
            .curve(curve)
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let key = self
            .model
            .edge(
                &first.key,
                &second.key,
                &curve,
                ParameterDomain::unit().into(),
            )
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let edge = Edge {
            key: key.clone(),
            start,
            end,
        };
        cache(&mut self.by_edge, id, edge.clone(), site)?;
        push(
            &mut self.edges,
            EdgeSource {
                source: id,
                geometry,
                master,
                parameters,
                key,
            },
        )
        .map_err(|kind| site.error(kind))?;
        Ok(edge)
    }
}
