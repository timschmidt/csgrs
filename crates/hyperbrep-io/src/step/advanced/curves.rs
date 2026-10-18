//! Whole-image linear certificates and source parameterizations, including PCURVE.
use super::super::decode::{component, parts};
use super::*;
use crate::part21::ValueKind;

pub(super) struct Line {
    pub source: u64,
    pub origin: Point3,
    pub velocity: Vector3,
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
    pub(super) fn same_oriented_image(&self, other: &Self, site: Site) -> Result<(), Error> {
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
    pub(super) fn parameters(
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
    pub(super) fn vector(&mut self, id: u64, dimension: usize) -> Result<Vector3, Error> {
        let mut args = self.simple(id, "VECTOR")?;
        let site = args.site;
        self.text(args.next()?, site)?;
        let direction = args.reference()?;
        let magnitude = self.scalar(args.next()?, site)?;
        args.finish()?;
        if order(site, &magnitude, &Real::zero())? != Ordering::Greater {
            return Err(site.invalid("line vector must have positive magnitude"));
        }
        Ok(self.vector_direction(direction, dimension)? * magnitude)
    }
    pub(super) fn vector_direction(&mut self, id: u64, dimension: usize) -> Result<Vector3, Error> {
        let mut args = self.simple(id, "DIRECTION")?;
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
        Vector3::new(coordinates)
            .normalize_checked()
            .map_err(|e| site.error(ErrorKind::Numeric(e)))
    }
    pub(super) fn point2(&mut self, id: u64) -> Result<Point2, Error> {
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
    pub(super) fn parametric_context(&mut self, id: u64) -> Result<(), Error> {
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
    pub(super) fn line3(&mut self, id: u64, scale: &Real) -> Result<Line, Error> {
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
pub(super) fn require_sense(
    parameters: &SourceCurveParameters,
    same_sense: bool,
    site: Site,
) -> Result<(), Error> {
    if order(site, &parameters.start, &parameters.end)?
        != if same_sense {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    {
        return Err(site.invalid("edge same_sense disagrees with source curve parameters"));
    }
    Ok(())
}

impl Builder {
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
        if entity.is_complex() && !spline::is_spline(entity) {
            return Err(geometry_site.error(ErrorKind::Unsupported("complex edge geometry")));
        }
        let mut associations = [None, None];
        let mut count = 0;
        let seam = record.name() == "SEAM_CURVE";
        let (spatial, master_index) = if !entity.is_complex()
            && matches!(record.name(), "SURFACE_CURVE" | "SEAM_CURVE")
        {
            let mut args = Args::new(record, geometry_site);
            resolver.text(args.next()?, geometry_site)?;
            let spatial = args.reference()?;
            for value in args.list()? {
                if count == 2 {
                    return Err(
                        geometry_site.invalid("surface curve requires one or two associations")
                    );
                }
                associations[count] = Some(reference(value, geometry_site)?);
                count += 1;
            }
            if count == 0 {
                return Err(geometry_site.invalid("surface curve requires one or two associations"));
            }
            let master = match args.next()?.kind() {
                ValueKind::Enumeration("CURVE_3D") => 0,
                ValueKind::Enumeration("PCURVE_S1") => 1,
                ValueKind::Enumeration("PCURVE_S2") if count == 2 => 2,
                ValueKind::Enumeration("PCURVE_S2") => {
                    return Err(geometry_site.invalid("missing master pcurve"));
                }
                _ => {
                    return Err(geometry_site
                        .error(ErrorKind::Expected("surface-curve master representation")));
                }
            };
            args.finish()?;
            (spatial, master)
        } else {
            (geometry, 0)
        };
        let spatial_entity = if spatial == geometry {
            entity
        } else {
            resolver.entity(spatial)?
        };
        let spatial_record = spatial_entity
            .components()
            .next()
            .expect("syntax component");
        let mut source_line = None;
        let (edge, spatial_parameters) =
            if !spatial_entity.is_complex() && spatial_record.name() == "CIRCLE" {
                self.circular_edge(resolver, spatial, start, end, same_sense, scale)?
            } else if spline::is_spline(spatial_entity) {
                self.spline_edge(resolver, spatial, start, end, same_sense, scale)?
            } else {
                let line = resolver.line3(spatial, scale)?;
                let parameters = line.parameters(&first.point, &second.point, site)?;
                require_sense(&parameters, same_sense, site)?;
                let curve = Curve3::line(first.point.clone(), second.point.clone())
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
                source_line = Some(line);
                (
                    Edge {
                        key,
                        start,
                        end,
                        circle: None,
                        spline: None,
                    },
                    parameters,
                )
            };
        let mut parameters = Vec::new();
        push(&mut parameters, spatial_parameters).map_err(|kind| site.error(kind))?;
        let mut checked = [None, None];
        for (index, id) in associations.into_iter().flatten().enumerate() {
            let pcurve = self.supplied_pcurve(
                resolver,
                id,
                &edge,
                source_line.as_ref(),
                same_sense,
                scale,
                geometry_site,
            )?;
            push(&mut parameters, pcurve.parameters.clone()).map_err(|kind| site.error(kind))?;
            checked[index] = Some(pcurve);
        }
        if seam {
            pcurves::validate_seam(&checked, geometry_site)?;
        }
        let master = parameters[master_index].curve;
        if count != 0 {
            let mut selected = parameters[master_index].clone();
            selected.curve = geometry;
            push(&mut parameters, selected).map_err(|kind| site.error(kind))?;
            parameters.rotate_right(1);
        }
        cache(&mut self.by_edge, id, edge.clone(), site)?;
        push(
            &mut self.edges,
            EdgeSource {
                source: id,
                geometry,
                master,
                parameters,
                key: edge.key.clone(),
            },
        )
        .map_err(|kind| site.error(kind))?;
        Ok(edge)
    }
}
