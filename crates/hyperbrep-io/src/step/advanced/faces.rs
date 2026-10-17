use super::*;
use crate::part21::ValueKind;

impl Builder {
    pub(super) fn face(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        id: u64,
        scale: &Real,
    ) -> Result<FaceKey, Error> {
        let mut args = resolver.simple(id, "ADVANCED_FACE")?;
        let site = args.site;
        resolver.text(args.next()?, site)?;
        let bounds = args.list()?;
        let surface = self.surface(resolver, args.reference()?, scale)?;
        let forward = args.boolean()?;
        args.finish()?;
        let orientation = if forward {
            Orientation::Forward
        } else {
            Orientation::Reversed
        };
        let mut bound_ids = HashSet::new();
        let mut outer = None;
        let mut holes = Vec::new();
        for value in bounds {
            let bound_id = reference(value, site)?;
            if !insert(&mut bound_ids, bound_id).map_err(|kind| site.error(kind))? {
                return Err(site.invalid("duplicate face bound"));
            }
            let entity = resolver.entity(bound_id)?;
            let bound_site = Site::entity(entity);
            let record = entity.components().next().expect("syntax component");
            if entity.is_complex() || !matches!(record.name(), "FACE_BOUND" | "FACE_OUTER_BOUND") {
                return Err(bound_site.error(ErrorKind::Expected("face bound")));
            }
            let declared_outer = record.name() == "FACE_OUTER_BOUND";
            let mut args = Args::new(record, bound_site);
            resolver.text(args.next()?, bound_site)?;
            let loop_id = args.reference()?;
            let bound_forward = args.boolean()?;
            args.finish()?;
            let (wire, area) =
                self.edge_loop(resolver, bound_id, loop_id, bound_forward, &surface, scale)?;
            let sign = order(bound_site, &area, &Real::zero())?;
            if sign == Ordering::Equal {
                return Err(bound_site.invalid("zero-area parameter-space bound"));
            }
            let is_outer = sign
                == if forward {
                    Ordering::Greater
                } else {
                    Ordering::Less
                };
            if declared_outer && !is_outer {
                return Err(bound_site.invalid("declared outer bound has inner orientation"));
            }
            push(&mut self.loops, (bound_id, loop_id, wire.clone()))
                .map_err(|kind| site.error(kind))?;
            if is_outer {
                if outer.replace(wire).is_some() {
                    return Err(site.invalid("more than one material outer bound"));
                }
            } else {
                push(&mut holes, wire).map_err(|kind| site.error(kind))?;
            }
        }
        let outer = outer.ok_or_else(|| {
            site.error(ErrorKind::Expected(
                "one finite parameter-space outer bound",
            ))
        })?;
        self.model
            .face(&surface.key, orientation, &outer, &holes)
            .map_err(|e| site.error(ErrorKind::Build(e)))
    }
    fn edge_loop(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        bound: u64,
        id: u64,
        forward: bool,
        surface: &FaceSurface,
        scale: &Real,
    ) -> Result<(WireKey, Real), Error> {
        let mut args = resolver.simple(id, "EDGE_LOOP")?;
        let site = args.site;
        resolver.text(args.next()?, site)?;
        let mut ordered = Vec::new();
        let mut oriented_ids = HashSet::new();
        for value in args.list()? {
            resolver.edge_use(site)?;
            let oriented = reference(value, site)?;
            if !insert(&mut oriented_ids, oriented).map_err(|kind| site.error(kind))? {
                return Err(site.invalid("edge loop repeats one oriented-edge entity"));
            }
            let mut args = resolver.simple(oriented, "ORIENTED_EDGE")?;
            let edge_site = args.site;
            resolver.text(args.next()?, edge_site)?;
            if args.next()?.kind() != ValueKind::Derived
                || args.next()?.kind() != ValueKind::Derived
            {
                return Err(edge_site.error(ErrorKind::Expected("derived oriented-edge endpoints")));
            }
            let edge = args.reference()?;
            let sense = args.boolean()?;
            args.finish()?;
            push(&mut ordered, (oriented, edge, sense == forward))
                .map_err(|kind| site.error(kind))?;
        }
        args.finish()?;
        if !forward {
            ordered.reverse();
        }
        let mut torus_segments = if matches!(surface.support, surfaces::Support::Torus(_)) {
            Some(
                self.torus_loop(resolver, &ordered, surface, scale, site)?
                    .into_iter(),
            )
        } else {
            None
        };
        let mut uses = Vec::new();
        let mut segments = Vec::new();
        let mut previous_vertex = None;
        let mut previous_parameter = None;
        for (oriented, edge_id, forward) in ordered {
            let edge = self.edge(resolver, edge_id, scale)?;
            let (from, to) = if forward {
                (edge.start, edge.end)
            } else {
                (edge.end, edge.start)
            };
            if previous_vertex.is_some_and(|id| id != from) {
                return Err(site.invalid("edge loop has disconnected vertex identities"));
            }
            let (segment, correspondence) = if let Some(prepared) = &mut torus_segments {
                prepared
                    .next()
                    .expect("one prepared segment per ordered use")
            } else {
                surface.boundary_segment(
                    &edge,
                    &self.by_vertex[&from].point,
                    &self.by_vertex[&to].point,
                    forward,
                    previous_parameter.as_ref(),
                    site,
                )?
            };
            previous_vertex = Some(to);
            previous_parameter = Some(segment.end().clone());
            let curve = match &segment {
                Segment2::Line(line) => Curve2::from(line.clone()),
                Segment2::Arc(arc) => Curve2::from(arc.clone()),
            };
            let pcurve = self
                .model
                .pcurve(Pcurve::new(curve))
                .map_err(|e| site.error(ErrorKind::Build(e)))?;
            let direction = if forward {
                Direction::Forward
            } else {
                Direction::Reversed
            };
            let key = self
                .model
                .edge_use(&edge.key, direction, &pcurve, correspondence)
                .map_err(|e| site.error(ErrorKind::Build(e)))?;
            push(
                &mut self.edge_uses,
                UseSource {
                    bound,
                    loop_id: id,
                    source: EdgeUseSource::OrientedEdge {
                        oriented_edge: oriented,
                        edge_curve: edge_id,
                        from_vertex: from,
                        to_vertex: to,
                    },
                    key: key.clone(),
                },
            )
            .map_err(|kind| site.error(kind))?;
            push(&mut uses, key).map_err(|kind| site.error(kind))?;
            push(&mut segments, segment).map_err(|kind| site.error(kind))?;
        }
        let wire = self
            .model
            .wire(&uses)
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let contour =
            Contour2::try_new(segments).map_err(|e| site.error(ErrorKind::Geometry(e.into())))?;
        let area = contour
            .signed_area()
            .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?
            .ok_or_else(|| site.error(ErrorKind::Unsupported("bound signed area")))?;
        Ok((wire, area))
    }
}
