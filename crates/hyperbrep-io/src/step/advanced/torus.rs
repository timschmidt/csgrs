//! STEP latitude/meridian chart construction, admitted by native image proofs.
use super::*;
use hyperbrep::Surface;

#[derive(Clone)]
pub(super) struct Torus {
    axis: Vector3,
    major: Curve3,
    minor_radius: Real,
}

impl Torus {
    pub(super) fn new(
        frame: &Plane,
        major_radius: Real,
        minor_radius: Real,
        site: Site,
    ) -> Result<(Surface, Self), Error> {
        // Part 42 permits positive horn/spindle radii. Native BREP currently
        // admits ring tori; preserve unsupported versus invalid input here.
        if order(site, &major_radius, &Real::zero())? == Ordering::Greater
            && order(site, &minor_radius, &Real::zero())? == Ordering::Greater
            && order(site, &major_radius, &minor_radius)? != Ordering::Greater
        {
            return Err(site.error(ErrorKind::Unsupported("non-ring toroidal surface")));
        }
        let axis = frame.x.cross(&frame.y);
        let surface = Surface::torus(
            frame.origin.clone(),
            frame.x.clone(),
            frame.y.clone(),
            axis.clone(),
            major_radius.clone(),
            minor_radius.clone(),
        )
        .map_err(|error| site.error(ErrorKind::Geometry(error)))?;
        let major = full_circle(
            frame.origin.clone(),
            frame.x.clone(),
            frame.y.clone(),
            major_radius,
            site,
        )?;
        Ok((
            surface,
            Self {
                axis,
                major,
                minor_radius,
            },
        ))
    }

    pub(super) fn boundary_segment(
        &self,
        frame: &Plane,
        edge: &Edge,
        from: &Point3,
        direction: &Real,
        previous: Option<&Point2>,
        site: Site,
    ) -> Result<Segment2, Error> {
        let circle = edge
            .circle
            .as_ref()
            .ok_or_else(|| site.error(ErrorKind::Unsupported("noncircular torus boundary")))?;
        let major_radius = self.major.circular_geometry().expect("major circle").radius;
        let latitude = order(
            site,
            &circle.normal.cross(&self.axis).norm_squared(),
            &Real::zero(),
        )? == Ordering::Equal;
        let (canonical, normal) = if latitude {
            let iso = full_circle(
                circle.center.clone(),
                frame.x.clone(),
                frame.y.clone(),
                circle.radius.clone(),
                site,
            )?;
            let minor = full_circle(
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                self.minor_radius.clone(),
                site,
            )?;
            let section = Point3::new(
                &circle.radius - major_radius,
                (&circle.center - &frame.origin).dot(&self.axis),
                Real::zero(),
            );
            (
                Point2::new(
                    circles::circle_parameter(&iso, from, site)?,
                    circles::circle_parameter(&minor, &section, site)?,
                ),
                surfaces::aligned_normals(&circle.normal, &self.axis, site)?,
            )
        } else {
            // Native circle location proves that the meridian center lies on
            // the major circle. The resulting radial vector is therefore unit.
            let u = circles::circle_parameter(&self.major, &circle.center, site)?;
            let radial = (&circle.center - &frame.origin)
                * (Real::one() / major_radius).map_err(|e| site.error(ErrorKind::Numeric(e)))?;
            let iso = full_circle(
                circle.center.clone(),
                radial.clone(),
                self.axis.clone(),
                self.minor_radius.clone(),
                site,
            )?;
            (
                Point2::new(u, circles::circle_parameter(&iso, from, site)?),
                surfaces::aligned_normals(&circle.normal, &radial.cross(&self.axis), site)?,
            )
        };
        // Preserve the preceding endpoint's periodic lift. Shared topology and
        // native whole-image validation, not endpoint sampling, admit this UV.
        let start = previous.cloned().unwrap_or(canonical);
        let sweep = &circle.sweep * normal * direction;
        let end = if latitude {
            Point2::new(start.x() + sweep, start.y().clone())
        } else {
            Point2::new(start.x().clone(), start.y() + sweep)
        };
        Ok(Segment2::Line(
            LineSeg2::try_new(start, end).map_err(|e| site.error(ErrorKind::Geometry(e.into())))?,
        ))
    }
}

fn full_circle(
    center: Point3,
    x: Vector3,
    y: Vector3,
    radius: Real,
    site: Site,
) -> Result<Curve3, Error> {
    Curve3::circle_arc(center, x, y, radius, Real::zero(), Real::tau())
        .map_err(|error| site.error(ErrorKind::Geometry(error)))
}

impl Builder {
    pub(super) fn torus_loop(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        ordered: &[(u64, u64, bool)],
        surface: &FaceSurface,
        scale: &Real,
        site: Site,
    ) -> Result<Vec<(Segment2, ParameterCorrespondence)>, Error> {
        let mut segments = Vec::new();
        let mut previous_vertex = None;
        let mut previous_parameter = None;
        for &(_, edge_id, forward) in ordered {
            let edge = self.edge(resolver, edge_id, scale)?;
            let (from, to) = if forward {
                (edge.start, edge.end)
            } else {
                (edge.end, edge.start)
            };
            if previous_vertex.is_some_and(|id| id != from) {
                return Err(site.invalid("edge loop has disconnected vertex identities"));
            }
            let (segment, correspondence) = surface.boundary_segment(
                &edge,
                &self.by_vertex[&from].point,
                &self.by_vertex[&to].point,
                forward,
                previous_parameter.as_ref(),
                site,
            )?;
            previous_vertex = Some(to);
            previous_parameter = Some(segment.end().clone());
            push(&mut segments, (segment, correspondence)).map_err(|kind| site.error(kind))?;
        }
        let Some((first, _)) = segments.first() else {
            return Ok(segments);
        };
        let mut min = [first.start().x().clone(), first.start().y().clone()];
        let mut max = min.clone();
        for (segment, _) in &segments {
            for point in [segment.start(), segment.end()] {
                for (axis, value) in [point.x(), point.y()].into_iter().enumerate() {
                    if order(site, value, &min[axis])? == Ordering::Less {
                        min[axis] = value.clone();
                    }
                    if order(site, value, &max[axis])? == Ordering::Greater {
                        max[axis] = value.clone();
                    }
                }
            }
        }
        let zero = Real::zero();
        let tau = Real::tau();
        let mut shift = [Real::zero(), Real::zero()];
        for axis in 0..2 {
            shift[axis] = if order(site, &min[axis], &zero)? == Ordering::Less {
                tau.clone()
            } else if order(site, &max[axis], &tau)? == Ordering::Greater {
                -&tau
            } else {
                Real::zero()
            };
            if order(site, &(&min[axis] + &shift[axis]), &zero)? == Ordering::Less
                || order(site, &(&max[axis] + &shift[axis]), &tau)? == Ordering::Greater
            {
                return Err(site.error(ErrorKind::Unsupported(
                    "torus loop requires periodic subdivision",
                )));
            }
        }
        // The first vertex may denote either side of a seam. Translate the
        // entire continuous loop by an exact period into the native chart;
        // never identify its distinct 0/tau boundary coordinates. Lines attain
        // their coordinate extrema at their endpoints. Whole-image admission
        // below still checks each translated pcurve against its spatial edge.
        if shift != [Real::zero(), Real::zero()] {
            let translate = |p: &Point2| Point2::new(p.x() + &shift[0], p.y() + &shift[1]);
            for (segment, _) in &mut segments {
                *segment = Segment2::Line(
                    LineSeg2::try_new(translate(segment.start()), translate(segment.end()))
                        .map_err(|e| site.error(ErrorKind::Geometry(e.into())))?,
                );
            }
        }
        Ok(segments)
    }
}
