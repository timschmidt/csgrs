//! Source circles. Native geometry owns radius, incidence and angle proofs.
use super::*;
use hyperbrep::CurveParameterLocation;

#[derive(Clone)]
pub(super) struct CircleEdge {
    pub center: Point3,
    pub normal: Vector3,
    pub radius: Real,
    /// Physical angular traversal in the source frame, including edge same_sense.
    pub sweep: Real,
    /// Increasing native parameter interval, even when physical traversal reverses.
    pub domain: ParameterDomain,
}

/// Choose the first parameter of the canonical full circle [0, tau]. The seam's
/// second parameter remains available when constructing the directed edge range.
pub(super) fn circle_parameter(curve: &Curve3, point: &Point3, site: Site) -> Result<Real, Error> {
    let CurveParameterLocation::Parameters(parameters) = curve
        .parameters_of(point)
        .map_err(|e| site.error(ErrorKind::Geometry(e)))?
    else {
        return Err(site.invalid("point is not on the exact source circle"));
    };
    parameters
        .first()
        .and_then(|parameter| parameter.try_real().cloned())
        .ok_or_else(|| site.error(ErrorKind::Geometry(GeometryError::UnrepresentableParameter)))
}

pub(super) fn directed_parameters(
    full: &Curve3,
    first: &Point3,
    second: &Point3,
    closed: bool,
    same_sense: bool,
    site: Site,
) -> Result<[Real; 2], Error> {
    let a = circle_parameter(full, first, site)?;
    let b = if closed {
        if same_sense {
            &a + Real::tau()
        } else {
            &a - Real::tau()
        }
    } else {
        let mut b = circle_parameter(full, second, site)?;
        let sense = order(site, &a, &b)?;
        if sense == Ordering::Equal {
            return Err(site.invalid("distinct circle vertices have the same exact position"));
        }
        if same_sense && sense == Ordering::Greater {
            b += Real::tau();
        } else if !same_sense && sense == Ordering::Less {
            b -= Real::tau();
        }
        b
    };
    Ok([a, b])
}

impl Builder {
    pub(super) fn angular_scale(&self, site: Site) -> Result<&Real, Error> {
        self.angle_scale
            .as_ref()
            .ok_or_else(|| site.error(ErrorKind::Expected("plane-angle unit for angular mapping")))
    }

    pub(super) fn circular_edge(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        geometry: u64,
        start: u64,
        end: u64,
        same_sense: bool,
        scale: &Real,
    ) -> Result<(Edge, SourceCurveParameters), Error> {
        let mut args = resolver.simple(geometry, "CIRCLE")?;
        let site = args.site;
        let angle_scale = self.angular_scale(site)?.clone();
        resolver.text(args.next()?, site)?;
        let frame = resolver.placement(args.reference()?, scale)?;
        let radius = resolver.scalar(args.next()?, site)? * scale;
        args.finish()?;
        let full = Curve3::circle_arc(
            frame.origin.clone(),
            frame.x.clone(),
            frame.y.clone(),
            radius.clone(),
            Real::zero(),
            Real::tau(),
        )
        .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
        let first = self.by_vertex[&start].clone();
        let second = self.by_vertex[&end].clone();
        let [a, b] = directed_parameters(
            &full,
            &first.point,
            &second.point,
            start == end,
            same_sense,
            site,
        )?;
        // Preserve the source frame and an increasing native angular domain.
        // Reversal changes traversal, while the directed source endpoints below
        // retain the source's phase, sense and declared angular unit.
        let normal = frame.x.cross(&frame.y);
        let sweep = &b - &a;
        let curve = Curve3::circle_arc(
            frame.origin.clone(),
            frame.x,
            frame.y,
            radius.clone(),
            if same_sense { a.clone() } else { b.clone() },
            if same_sense { b.clone() } else { a.clone() },
        )
        .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
        let curve = if same_sense {
            curve
        } else {
            curve
                .reversed()
                .map_err(|e| site.error(ErrorKind::Geometry(e)))?
        };
        let circle = CircleEdge {
            radius,
            center: frame.origin,
            normal,
            sweep,
            domain: curve.domain().clone(),
        };
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
                circle.domain.clone().into(),
            )
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let edge = Edge {
            key: key.clone(),
            start,
            end,
            circle: Some(circle),
        };
        Ok((
            edge,
            SourceCurveParameters {
                curve: geometry,
                start: (a / &angle_scale).map_err(|e| site.error(ErrorKind::Numeric(e)))?,
                end: (b / &angle_scale).map_err(|e| site.error(ErrorKind::Numeric(e)))?,
            },
        ))
    }
}
