//! Supplied UV representations and native whole-image admission.
use super::*;
use hypercurve::CircularArc2;

pub(super) struct CheckedPcurve {
    pub parameters: SourceCurveParameters,
    surface: u64,
    /// Constant angular coordinate and its axis (0 for U, 1 for V).
    seam: Option<(usize, Real)>,
}

pub(super) fn validate_seam(pcurves: &[Option<CheckedPcurve>; 2], site: Site) -> Result<(), Error> {
    let [Some(first), Some(second)] = pcurves else {
        return Err(site.invalid("seam curve requires two pcurves"));
    };
    if first.surface != second.surface {
        return Err(site.invalid("seam pcurves must share their basis surface"));
    }
    let (Some((first_axis, a)), Some((second_axis, b))) = (&first.seam, &second.seam) else {
        return Err(site.error(ErrorKind::Unsupported("seam parameter boundary")));
    };
    if first_axis != second_axis {
        return Err(site.invalid("seam pcurves must bound the same angular coordinate"));
    }
    let zero = Real::zero();
    let tau = Real::tau();
    if !((order(site, a, &zero)? == Ordering::Equal && order(site, b, &tau)? == Ordering::Equal)
        || (order(site, b, &zero)? == Ordering::Equal && order(site, a, &tau)? == Ordering::Equal))
    {
        return Err(site.invalid("seam pcurves are not at opposite exact parameter bounds"));
    }
    Ok(())
}

impl Builder {
    #[allow(clippy::too_many_arguments)]
    pub(super) fn supplied_pcurve(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        edge: &Edge,
        source_line: Option<&curves::Line>,
        same_sense: bool,
        scale: &Real,
        geometry_site: Site,
    ) -> Result<CheckedPcurve, Error> {
        let mut args = resolver.simple(id, "PCURVE")?;
        let site = args.site;
        resolver.text(args.next()?, site)?;
        let surface_id = args.reference()?;
        let representation = args.reference()?;
        args.finish()?;
        let branch = match self.conical_source(resolver, surface_id, scale)? {
            Some(source) => {
                let first = source.branch_at(&self.by_vertex[&edge.start].point);
                let branch = match first {
                    Err(GeometryError::SingularSurfaceParameter) => {
                        source.branch_at(&self.by_vertex[&edge.end].point)
                    }
                    other => other,
                }
                .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
                Some(branch)
            }
            None => None,
        };
        let surface = self.surface(resolver, surface_id, scale, branch)?;
        let mut args = resolver.simple(representation, "DEFINITIONAL_REPRESENTATION")?;
        let representation_site = args.site;
        resolver.text(args.next()?, representation_site)?;
        let mut items = args.list()?;
        let curve = reference(
            items.next().ok_or_else(|| {
                representation_site.error(ErrorKind::Expected("one pcurve basis curve"))
            })?,
            representation_site,
        )?;
        if items.next().is_some() {
            return Err(representation_site.error(ErrorKind::Expected("one pcurve basis curve")));
        }
        resolver.parametric_context(args.reference()?)?;
        args.finish()?;
        let entity = resolver.entity(curve)?;
        let curve_site = Site::entity(entity);
        let record = entity.components().next().expect("syntax component");
        if entity.is_complex() {
            return Err(curve_site.error(ErrorKind::Unsupported("complex pcurve basis")));
        }
        let first = &self.by_vertex[&edge.start].point;
        let second = &self.by_vertex[&edge.end].point;
        let mut args = Args::new(record, curve_site);
        resolver.text(args.next()?, curve_site)?;
        let mut seam = None;
        let (pcurve, correspondence, parameters) = match record.name() {
            "LINE" => {
                let origin = resolver.point2(args.reference()?)?;
                let velocity = resolver.vector(args.reference()?, 2)?;
                args.finish()?;
                if let Some(line) = source_line.filter(|_| surface.is_plane()) {
                    // Preserve the exact infinite affine-line certificate and
                    // its source maps. Curved images use the native query below.
                    let plane = surface.geometry.as_ref().expect("plane support frame");
                    let mapped = curves::Line {
                        source: id,
                        origin: plane.origin.clone()
                            + plane.x.clone() * (origin.x() * scale)
                            + plane.y.clone() * (origin.y() * scale),
                        velocity: plane.x.clone() * (&velocity[0] * scale)
                            + plane.y.clone() * (&velocity[1] * scale),
                    };
                    line.same_oriented_image(&mapped, geometry_site)?;
                    return Ok(CheckedPcurve {
                        parameters: mapped.parameters(first, second, site)?,
                        surface: surface_id,
                        seam,
                    });
                }
                let angular = surface.angular_coordinates();
                let unit_parameter = Real::one();
                let u_scale = if matches!(surface.support, surfaces::Support::Extrusion(_)) {
                    &unit_parameter
                } else if angular[0] {
                    self.angular_scale(site)?
                } else {
                    scale
                };
                let v_scale = if matches!(
                    surface.support,
                    surfaces::Support::Cone(_) | surfaces::Support::Extrusion(_)
                ) {
                    &unit_parameter
                } else if angular[1] {
                    self.angular_scale(site)?
                } else {
                    scale
                };
                let mut origin = Point2::new(origin.x() * u_scale, origin.y() * v_scale);
                let mut velocity = [&velocity[0] * u_scale, &velocity[1] * v_scale];
                if angular[0] && order(site, &velocity[0], &Real::zero())? == Ordering::Equal {
                    seam = Some((0, origin.x().clone()));
                } else if angular[1] && order(site, &velocity[1], &Real::zero())? == Ordering::Equal
                {
                    seam = Some((1, origin.y().clone()));
                }
                if let surfaces::Support::Cone(chart) = &surface.support {
                    // Seam evidence above is in the authored angular frame.
                    // A reversed native chart need not put that seam at +tau.
                    let mapped = chart.native_parameters(&hyperbrep::Point2::new(
                        origin.x().clone(),
                        origin.y().clone(),
                    ));
                    origin = Point2::new(mapped.x, mapped.y);
                    velocity = [
                        &velocity[0] * chart.angular_scale(),
                        &velocity[1] * chart.slant_scale(),
                    ];
                }
                let (segment, correspondence) =
                    surface.boundary_segment(edge, first, second, true, None, site)?;
                let Segment2::Line(target) = segment else {
                    return Err(site.error(ErrorKind::Unsupported(
                        "linear pcurve for a planar circular edge",
                    )));
                };
                // Derive candidate parameters from one nonzero affine component;
                // the native proof checks every coordinate over the whole edge.
                let component = if order(site, &velocity[0], &Real::zero())? != Ordering::Equal {
                    0
                } else {
                    1
                };
                let coordinate = |point: &Point2| {
                    if component == 0 {
                        point.x().clone()
                    } else {
                        point.y().clone()
                    }
                };
                let parameter = |point: &Point2| {
                    ((coordinate(point) - coordinate(&origin)) / &velocity[component])
                        .map_err(|e| site.error(ErrorKind::Numeric(e)))
                };
                let a = parameter(target.start())?;
                let b = parameter(target.end())?;
                let at = |t: &Real| {
                    Point2::new(origin.x() + &velocity[0] * t, origin.y() + &velocity[1] * t)
                };
                let native = LineSeg2::try_new(at(&a), at(&b))
                    .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?;
                (
                    Pcurve::new(native.into()),
                    correspondence,
                    SourceCurveParameters {
                        curve: id,
                        start: a,
                        end: b,
                    },
                )
            }
            "CIRCLE" if surface.is_plane() => {
                let placement = args.reference()?;
                let radius = resolver.scalar(args.next()?, curve_site)? * scale;
                args.finish()?;
                let mut args = resolver.simple(placement, "AXIS2_PLACEMENT_2D")?;
                let placement_site = args.site;
                resolver.text(args.next()?, placement_site)?;
                let center = resolver.point2(args.reference()?)?;
                let center = Point2::new(center.x() * scale, center.y() * scale);
                let x = match args.optional_reference()? {
                    Some(id) => resolver.vector_direction(id, 2)?,
                    None => Vector3::x(),
                };
                args.finish()?;
                let y = Vector3::new([-x[1].clone(), x[0].clone(), Real::zero()]);
                let point3 = |p: &Point2| Point3::new(p.x().clone(), p.y().clone(), Real::zero());
                let full =
                    Curve3::circle_arc(point3(&center), x, y, radius, Real::zero(), Real::tau())
                        .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
                let project = |point: &Point3| {
                    let plane = surface.geometry.as_ref().expect("plane support frame");
                    let delta = point - &plane.origin;
                    Point2::new(delta.dot(&plane.x), delta.dot(&plane.y))
                };
                let from = project(first);
                let to = project(second);
                let [a, b] = circles::directed_parameters(
                    &full,
                    &point3(&from),
                    &point3(&to),
                    edge.start == edge.end,
                    same_sense,
                    site,
                )?;
                let angle = self.angular_scale(site)?;
                let parameters = SourceCurveParameters {
                    curve: id,
                    start: (a / angle).map_err(|e| site.error(ErrorKind::Numeric(e)))?,
                    end: (b / angle).map_err(|e| site.error(ErrorKind::Numeric(e)))?,
                };
                let arc = CircularArc2::try_from_center(from, to, center, !same_sense)
                    .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?;
                (
                    Pcurve::new(arc.into()),
                    ParameterCorrespondence::angular_sweep(),
                    parameters,
                )
            }
            _ => {
                return Err(curve_site.error(ErrorKind::Unsupported(
                    "advanced pcurve basis/surface combination",
                )));
            }
        };
        curves::require_sense(&parameters, same_sense, geometry_site)?;
        let pcurve = self
            .model
            .pcurve(pcurve)
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        self.model
            .validate_edge_image(
                &edge.key,
                Direction::Forward,
                &pcurve,
                correspondence,
                &surface.key,
            )
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        Ok(CheckedPcurve {
            parameters,
            surface: surface_id,
            seam,
        })
    }
}
