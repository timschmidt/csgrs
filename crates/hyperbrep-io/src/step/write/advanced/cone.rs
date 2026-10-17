//! Exact conical revolution encoding with finite generator/axis anchors.
use super::super::number;
use super::{Output, WriteError, WriteErrorKind, WrittenSurfaceParameters, geometry};
use hyperbrep::{Direction, FaceRef, Point2, Point3, Real, Surface, Vector2, Vector3};

struct Anchor {
    point: Point3,
    axis_point: Point3,
    velocity: Vector3,
    parameters: WrittenSurfaceParameters,
}

fn representability(error: &WriteError) -> bool {
    matches!(
        error.kind,
        WriteErrorKind::NonRational | WriteErrorKind::NonTerminatingDecimal
    )
}

fn anchor(
    out: &Output,
    surface: &Surface,
    uv: &Point2,
    point: &Point3,
    scale: &Real,
) -> Result<Anchor, WriteError> {
    let data = surface.conical_geometry().expect("native cone dispatch");
    let point = geometry::converted(point, scale, out)?;
    let center = data.apex.clone() + data.axis.clone() * (&uv.y * data.semi_angle.clone().cos());
    let axis_point = geometry::converted(&center, scale, out)?;
    // Coordinate conversion proves rationality and size, not finite-decimal
    // representability. Reject an unencodable anchor before selecting it.
    for point in [&point, &axis_point] {
        for coordinate in [&point.x, &point.y, &point.z] {
            number::decimal(coordinate, &out.limits, out.site)?;
        }
    }
    let velocity = surface
        .partials_at(uv)
        .map_err(|error| out.site.error(WriteErrorKind::Geometry(error)))?
        .v()
        .clone();
    let ratios = geometry::direction_values(&velocity, out)?;
    for i in 0..3 {
        number::decimal(&ratios[i], &out.limits, out.site)?;
    }
    let inverse_scale =
        (Real::one() / scale).map_err(|error| out.site.error(WriteErrorKind::Numeric(error)))?;
    Ok(Anchor {
        point,
        axis_point,
        velocity,
        parameters: WrittenSurfaceParameters {
            origin: Point2::new(-&uv.x, -(&uv.y * &inverse_scale)),
            u: Vector2::x(),
            v: Vector2::new([Real::zero(), inverse_scale]),
        },
    })
}

fn select(out: &Output, face: FaceRef<'_>, scale: &Real) -> Result<Anchor, WriteError> {
    let surface = face.surface();
    let data = surface.conical_geometry().expect("native cone dispatch");
    // The axis direction is independent of the selected anchor.
    geometry::direction_values(data.axis, out)?;
    let first_error = match anchor(out, &surface, &Point2::origin(), data.apex, scale) {
        Ok(anchor) => return Ok(anchor),
        Err(error) if representability(&error) => error,
        Err(error) => return Err(error),
    };
    for wire in face.outer().into_iter().chain(face.inner()) {
        for edge_use in wire.edge_uses() {
            let pcurve = edge_use.pcurve();
            let edge = edge_use.edge();
            for (parameter, first) in [(pcurve.domain_start(), true), (pcurve.domain_end(), false)]
            {
                let parameter =
                    parameter.map_err(|error| out.site.error(WriteErrorKind::Geometry(error)))?;
                let image = pcurve
                    .point_at(&parameter)
                    .map_err(|error| out.site.error(WriteErrorKind::Geometry(error)))?;
                let uv = image.coordinates().ok_or_else(|| {
                    out.site.error(WriteErrorKind::Geometry(
                        hyperbrep::GeometryError::UnrepresentableParameter,
                    ))
                })?;
                let uv = Point2::new(uv.x().clone(), uv.y().clone());
                // Prefer the authored longitude-zero generator. It keeps the
                // frame consistent when several faces share one native support.
                let canonical = Point2::new(Real::zero(), uv.y.clone());
                let point = surface
                    .point_at(&canonical)
                    .map_err(|error| out.site.error(WriteErrorKind::Geometry(error)))?;
                match anchor(out, &surface, &canonical, &point, scale) {
                    Ok(anchor) => return Ok(anchor),
                    Err(error) if representability(&error) => {}
                    Err(error) => return Err(error),
                }
                // Preserve the owned endpoint image when the canonical
                // generator has no exact finite encoding. Native edge-image
                // validation binds this UV endpoint to the directed vertex.
                let vertex = if first == (edge_use.direction() == Direction::Forward) {
                    edge.start()
                } else {
                    edge.end()
                };
                let point = vertex
                    .point()
                    .try_coordinates()
                    .ok_or_else(|| out.site.error(WriteErrorKind::UnmaterializedPoint))?;
                match anchor(out, &surface, &uv, point, scale) {
                    Ok(anchor) => return Ok(anchor),
                    Err(error) if representability(&error) => {}
                    Err(error) => return Err(error),
                }
            }
        }
    }
    Err(first_error)
}

pub(super) fn surface(
    out: &mut Output,
    face: FaceRef<'_>,
    scale: &Real,
) -> Result<(u64, WrittenSurfaceParameters, bool), WriteError> {
    // Selection and all number/direction proofs precede entity emission.
    // A successful anchor parameterizes the entire selected cone branch, so
    // the writer may reuse this encoding for every face sharing that support.
    let anchor = select(out, face, scale)?;
    let data = face.surface();
    let cone = data.conical_geometry().expect("native cone dispatch");
    let origin = geometry::point(out, &anchor.point)?;
    let generator = geometry::line(out, origin, &anchor.velocity)?;
    let point = geometry::point(out, &anchor.axis_point)?;
    let axis = geometry::direction(out, cone.axis)?;
    let placement = out.entity("AXIS1_PLACEMENT", |out| {
        out.put("'',")?;
        out.reference(point)?;
        out.put(",")?;
        out.reference(axis)
    })?;
    let surface = out.entity("SURFACE_OF_REVOLUTION", |out| {
        out.put("'',")?;
        out.reference(generator)?;
        out.put(",")?;
        out.reference(placement)
    })?;
    Ok((surface, anchor.parameters, false))
}
