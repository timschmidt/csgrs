//! Full-size 5-to-2, height-6 frustum with one periodic side and two circle caps.
use hyperbrep::*;
use hypercurve::{CircularArc2, Curve2, LineSeg2, Point2 as UV};

pub fn build(angle: Real, turns: i32, reversed: bool) -> Result<Model, BuildError> {
    with_span(angle, turns, reversed, Real::tau())
}

pub fn with_span(angle: Real, turns: i32, reversed: bool, span: Real) -> Result<Model, BuildError> {
    let mut b = ModelBuilder::new();
    let end = &angle + Real::tau();
    let point = |top| {
        let radius = Real::from(if top { 2 } else { 5 });
        Point3::new(
            &radius * angle.clone().cos(),
            radius * angle.clone().sin(),
            Real::from(if top { 6 } else { 0 }),
        )
    };
    let vertices = [
        b.vertex(point(false).into())?,
        b.vertex(point(true).into())?,
    ];
    let mut circles = Vec::new();
    let mut faces = Vec::new();
    for (i, top) in [false, true].into_iter().enumerate() {
        let z = Real::from(if top { 6 } else { 0 });
        let curve = Curve3::circle_arc(
            Point3::new(0.into(), 0.into(), z.clone()),
            Vector3::x(),
            Vector3::y(),
            Real::from(if top { 2 } else { 5 }),
            angle.clone(),
            end.clone(),
        )?;
        let curve = b.curve(if reversed { curve.reversed()? } else { curve })?;
        let edge = b.edge(
            &vertices[i],
            &vertices[i],
            &curve,
            ParameterDomain::new(angle.clone(), end.clone())?.into(),
        )?;
        let p = point(top);
        let p = UV::new(p.x, p.y);
        let arc = CircularArc2::try_from_center(p.clone(), p, UV::new(0.into(), 0.into()), !top)
            .map_err(GeometryError::from)?;
        let pc = b.pcurve(Pcurve::new(Curve2::from(arc)))?;
        let edge_use = b.edge_use(
            &edge,
            if top != reversed {
                Direction::Forward
            } else {
                Direction::Reversed
            },
            &pc,
            ParameterCorrespondence::angular_sweep(),
        )?;
        let wire = b.wire(&[edge_use])?;
        let plane = b.surface(Surface::plane(
            Point3::new(0.into(), 0.into(), z),
            Vector3::x(),
            Vector3::y(),
        )?)?;
        faces.push(b.face(
            &plane,
            if top {
                Orientation::Forward
            } else {
                Orientation::Reversed
            },
            &wire,
            &[],
        )?);
        circles.push(edge);
    }
    let line = b.curve(Curve3::line(point(false), point(true))?)?;
    let seam = b.edge(
        &vertices[0],
        &vertices[1],
        &line,
        ParameterDomain::unit().into(),
    )?;
    let low = -angle.clone() + Real::from(turns) * Real::tau();
    let high = &low + span;
    let root = Real::from(5).sqrt().map_err(GeometryError::Numeric)?;
    let near = Real::from(2) * &root;
    let far = Real::from(5) * root;
    let mut uses = Vec::new();
    for (edge, from, to, physical_forward, circular) in [
        (
            &circles[1],
            UV::new(low.clone(), near.clone()),
            UV::new(high.clone(), near.clone()),
            false,
            true,
        ),
        (
            &seam,
            UV::new(high.clone(), near.clone()),
            UV::new(high.clone(), far.clone()),
            false,
            false,
        ),
        (
            &circles[0],
            UV::new(high, far.clone()),
            UV::new(low.clone(), far),
            true,
            true,
        ),
        (
            &seam,
            UV::new(
                low.clone(),
                Real::from(5) * Real::from(5).sqrt().map_err(GeometryError::Numeric)?,
            ),
            UV::new(low, near),
            true,
            false,
        ),
    ] {
        let forward = physical_forward != (circular && reversed);
        let pc = b.pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(from, to).map_err(GeometryError::from)?,
        )))?;
        let (scale, offset) = if circular {
            (
                if forward { Real::tau() } else { -Real::tau() },
                if forward { angle.clone() } else { end.clone() },
            )
        } else {
            (
                if forward { Real::one() } else { -Real::one() },
                if forward { Real::zero() } else { Real::one() },
            )
        };
        uses.push(b.edge_use(
            edge,
            if forward {
                Direction::Forward
            } else {
                Direction::Reversed
            },
            &pc,
            ParameterCorrespondence::affine(scale, offset)?,
        )?);
    }
    let wire = b.wire(&uses)?;
    let cone = b.surface(Surface::cone(
        Point3::new(0.into(), 0.into(), 10.into()),
        Vector3::x(),
        -Vector3::y(),
        -Vector3::z(),
        (Real::one() / Real::from(2))
            .map_err(GeometryError::Numeric)?
            .atan()
            .map_err(GeometryError::Numeric)?,
    )?)?;
    faces.push(b.face(&cone, Orientation::Forward, &wire, &[])?);
    let shell = b.shell(&faces)?;
    b.solid(&shell, &[])?;
    Ok(b.finish().expect("constructed closed cone must replay"))
}
