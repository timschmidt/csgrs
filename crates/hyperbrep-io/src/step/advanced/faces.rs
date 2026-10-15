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
        let plane = self.plane(resolver, args.reference()?, scale)?;
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
            let (wire, area) = self.edge_loop(
                resolver,
                bound_id,
                loop_id,
                bound_forward,
                &plane.geometry,
                scale,
            )?;
            let sign = order(bound_site, &area, &Real::zero())?;
            if sign == Ordering::Equal {
                return Err(bound_site.invalid("zero-area planar bound"));
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
        let outer = outer
            .ok_or_else(|| site.error(ErrorKind::Expected("one finite planar outer bound")))?;
        self.model
            .face(&plane.key, orientation, &outer, &holes)
            .map_err(|e| site.error(ErrorKind::Build(e)))
    }
    fn edge_loop(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        bound: u64,
        id: u64,
        forward: bool,
        plane: &Plane,
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
        let mut uses = Vec::new();
        let mut segments = Vec::new();
        for (oriented, edge_id, forward) in ordered {
            let edge = self.edge(resolver, edge_id, scale)?;
            let (from, to) = if forward {
                (edge.start, edge.end)
            } else {
                (edge.end, edge.start)
            };
            let uv = |id| {
                let point = &self.by_vertex[&id].point;
                let delta = point - &plane.origin;
                Point2::new(delta.dot(&plane.x), delta.dot(&plane.y))
            };
            let line = LineSeg2::try_new(uv(from), uv(to))
                .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?;
            let pcurve = self
                .model
                .pcurve(Pcurve::new(Curve2::from(line.clone())))
                .map_err(|e| site.error(ErrorKind::Build(e)))?;
            let (direction, correspondence) = if forward {
                (Direction::Forward, ParameterCorrespondence::identity())
            } else {
                (
                    Direction::Reversed,
                    ParameterCorrespondence::affine(-Real::one(), Real::one())
                        .map_err(|e| site.error(ErrorKind::Build(e)))?,
                )
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
            push(&mut segments, Segment2::Line(line)).map_err(|kind| site.error(kind))?;
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
