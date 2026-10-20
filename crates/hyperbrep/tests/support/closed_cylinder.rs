//! A cylinder with two vertices, three spatial edges and one periodic side face.
//! Each cap has one full-circle edge. The seam has two distinct face-local uses.
use hyperbrep::*;
use hypercurve::{CircularArc2, Curve2, LineSeg2, Point2 as UV};

pub fn build(angle: Real, turns: i32, reversed_curves: bool) -> Result<Model, BuildError> {
    let mut b = ModelBuilder::new();
    let end = &angle + Real::tau();
    let point = |z| {
        Point3::new(
            Real::from(2) * angle.clone().cos(),
            Real::from(2) * angle.clone().sin(),
            Real::from(z),
        )
    };
    let vertices = [b.vertex(point(0).into())?, b.vertex(point(4).into())?];
    let mut circles = Vec::new();
    for (i, z) in [0, 4].into_iter().enumerate() {
        let curve = Curve3::circle_arc(
            Point3::new(Real::zero(), Real::zero(), Real::from(z)),
            Vector3::x(),
            Vector3::y(),
            Real::from(2),
            angle.clone(),
            end.clone(),
        )?;
        let curve = b.curve(if reversed_curves {
            curve.reversed()?
        } else {
            curve
        })?;
        circles.push(b.edge(
            &vertices[i],
            &vertices[i],
            &curve,
            ParameterDomain::new(angle.clone(), end.clone())?.into(),
        )?);
    }
    let line = b.curve(Curve3::line(point(0), point(4))?)?;
    let seam = b.edge(
        &vertices[0],
        &vertices[1],
        &line,
        ParameterDomain::unit().into(),
    )?;
    let mut faces = Vec::new();
    for (i, z) in [0, 4].into_iter().enumerate() {
        let top = i == 1;
        let p = point(z);
        let p = UV::new(p.x, p.y);
        let arc =
            CircularArc2::try_from_center(p.clone(), p, UV::new(Real::zero(), Real::zero()), !top)
                .map_err(GeometryError::from)?;
        let pcurve = b.pcurve(Pcurve::new(Curve2::from(arc)))?;
        let use_key = b.edge_use(
            &circles[i],
            if top != reversed_curves {
                Direction::Forward
            } else {
                Direction::Reversed
            },
            &pcurve,
            ParameterCorrespondence::angular_sweep(),
        )?;
        let wire = b.wire(&[use_key])?;
        let plane = b.surface(Surface::plane(
            Point3::new(Real::zero(), Real::zero(), Real::from(z)),
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
    }
    let lo = &angle + Real::tau() * Real::from(turns);
    let hi = &lo + Real::tau();
    let mut uses = Vec::new();
    for (edge, from, to, physical_forward, circular) in [
        (
            &circles[0],
            UV::new(lo.clone(), Real::zero()),
            UV::new(hi.clone(), Real::zero()),
            true,
            true,
        ),
        (
            &seam,
            UV::new(hi.clone(), Real::zero()),
            UV::new(hi.clone(), Real::from(4)),
            true,
            false,
        ),
        (
            &circles[1],
            UV::new(hi.clone(), Real::from(4)),
            UV::new(lo.clone(), Real::from(4)),
            false,
            true,
        ),
        (
            &seam,
            UV::new(lo.clone(), Real::from(4)),
            UV::new(lo.clone(), Real::zero()),
            false,
            false,
        ),
    ] {
        let forward = physical_forward != (circular && reversed_curves);
        let pcurve = b.pcurve(Pcurve::new(Curve2::from(
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
            &pcurve,
            ParameterCorrespondence::affine(scale, offset)?,
        )?);
    }
    let wire = b.wire(&uses)?;
    let surface = b.surface(Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Real::from(2),
    )?)?;
    faces.push(b.face(&surface, Orientation::Forward, &wire, &[])?);
    let shell = b.shell(&faces)?;
    b.solid(&shell, &[])?;
    Ok(b.finish().expect("constructed topology must replay"))
}

pub fn check(model: &Model) {
    let solid = model.solids().next().unwrap();
    assert_eq!(solid.volume().unwrap(), Real::from(16) * Real::pi());
    let area = model
        .faces()
        .map(|f| f.area().unwrap())
        .fold(Real::zero(), |a, b| a + b);
    assert_eq!(area, Real::from(24) * Real::pi());
    for (p, expected) in [
        (
            Point3::new(0.into(), 0.into(), 2.into()),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(3.into(), 0.into(), 2.into()),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(2.into(), 0.into(), 2.into()),
            SolidPointLocation::Boundary,
        ),
    ] {
        assert_eq!(solid.classify_point(&p).unwrap(), expected);
    }
}

pub fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let loaded = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(loaded.counts(), model.counts());
    assert_eq!(loaded.to_json().unwrap(), json);
    loaded
}
