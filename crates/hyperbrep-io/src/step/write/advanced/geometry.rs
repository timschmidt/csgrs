use super::super::{WriteError, WriteErrorKind, error::Site, number, output::Output};
use super::WrittenSurfaceParameters;
use hyperbrep::{GeometryError, Point3, Real, Surface, SurfaceKind, Vector2, Vector3};
use hyperlimit::{PredicateOutcome, compare_reals};
use std::cmp::Ordering;

fn order(a: &Real, b: &Real, site: Site) -> Result<Ordering, WriteError> {
    match compare_reals(a, b, hyperbrep::STRICT_PREDICATES) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => Err(site.error(WriteErrorKind::Geometry(
            GeometryError::PredicateUnresolved { needed, stage },
        ))),
    }
}

/// The scalar layer owns this value certificate. Materialization here selects
/// an exact STEP encoding; it never substitutes a nearby rational.
fn rational(value: &Real, out: &Output) -> Result<Real, WriteError> {
    let value = if value.exact_rational_ref().is_some() {
        value.clone()
    } else {
        Real::new(
            value
                .exact_rational_normal_form()
                .ok_or_else(|| out.site.error(WriteErrorKind::NonRational))?,
        )
    };
    number::exact(&value, &out.limits, out.site)?;
    Ok(value)
}

pub(super) fn length(value: &Real, scale: &Real, out: &Output) -> Result<Real, WriteError> {
    let value = rational(value, out)?;
    let value =
        (&value / scale).map_err(|problem| out.site.error(WriteErrorKind::Numeric(problem)))?;
    rational(&value, out)
}

pub(super) fn converted(point: &Point3, scale: &Real, out: &Output) -> Result<Point3, WriteError> {
    Ok(Point3::new(
        length(&point.x, scale, out)?,
        length(&point.y, scale, out)?,
        length(&point.z, scale, out)?,
    ))
}

/// Emits already converted coordinates.
pub(super) fn point(out: &mut Output, point: &Point3) -> Result<u64, WriteError> {
    out.entity("CARTESIAN_POINT", |out| {
        out.put("'',(")?;
        out.number(&point.x)?;
        out.put(",")?;
        out.number(&point.y)?;
        out.put(",")?;
        out.number(&point.z)?;
        out.put(")")
    })
}

fn direction(out: &mut Output, vector: &Vector3) -> Result<u64, WriteError> {
    // Check every directly retained rational component before any projective
    // division. Normalized rational directions may share an irrational factor.
    for i in 0..3 {
        if vector[i].exact_rational_ref().is_some() {
            number::exact(&vector[i], &out.limits, out.site)?;
        }
    }
    let ratios = match number::primitive_direction(vector, &out.limits, out.site) {
        Ok(value) => value,
        Err(WriteError {
            kind: WriteErrorKind::NonRational,
            ..
        }) => {
            let mut pivot = None;
            for i in 0..3 {
                if order(&vector[i], &Real::zero(), out.site)? != Ordering::Equal {
                    pivot = Some(vector[i].abs());
                    break;
                }
            }
            let pivot =
                pivot.ok_or_else(|| out.site.error(WriteErrorKind::Invalid("zero direction")))?;
            let mut ratios = [Real::zero(), Real::zero(), Real::zero()];
            for i in 0..3 {
                ratios[i] = rational(
                    &(&vector[i] / &pivot)
                        .map_err(|problem| out.site.error(WriteErrorKind::Numeric(problem)))?,
                    out,
                )?;
            }
            number::primitive_direction(&Vector3::new(ratios), &out.limits, out.site)?
        }
        Err(error) => return Err(error),
    };
    out.entity("DIRECTION", |out| {
        out.put("'',(")?;
        out.number(&ratios[0])?;
        out.put(",")?;
        out.number(&ratios[1])?;
        out.put(",")?;
        out.number(&ratios[2])?;
        out.put(")")
    })
}

pub(super) fn placement(
    out: &mut Output,
    origin: &Point3,
    axis: &Vector3,
    x: &Vector3,
    scale: &Real,
) -> Result<u64, WriteError> {
    let origin = converted(origin, scale, out)?;
    let origin = point(out, &origin)?;
    let axis = direction(out, axis)?;
    let x = direction(out, x)?;
    out.entity("AXIS2_PLACEMENT_3D", |out| {
        out.put("'',")?;
        out.reference(origin)?;
        out.put(",")?;
        out.reference(axis)?;
        out.put(",")?;
        out.reference(x)
    })
}

pub(super) fn surface(
    out: &mut Output,
    source: &Surface,
    scale: &Real,
) -> Result<(u64, WrittenSurfaceParameters), WriteError> {
    let divide = |value: Real| {
        (value / scale).map_err(|problem| out.site.error(WriteErrorKind::Numeric(problem)))
    };
    match source.kind() {
        SurfaceKind::Plane => {
            let origin = source.plane_origin().expect("plane origin");
            let (u, v) = source.plane_directions().expect("plane directions");
            let normal = u.cross(v);
            let x = u
                .normalize_checked()
                .map_err(|problem| out.site.error(WriteErrorKind::Numeric(problem)))?;
            let axis = normal
                .normalize_checked()
                .map_err(|problem| out.site.error(WriteErrorKind::Numeric(problem)))?;
            let y = axis
                .cross(&x)
                .normalize_checked()
                .map_err(|problem| out.site.error(WriteErrorKind::Numeric(problem)))?;
            let parameters = WrittenSurfaceParameters {
                u: Vector2::new([divide(u.dot(&x))?, divide(u.dot(&y))?]),
                v: Vector2::new([divide(v.dot(&x))?, divide(v.dot(&y))?]),
            };
            let placement = placement(out, origin, &normal, u, scale)?;
            let id = out.entity("PLANE", |out| {
                out.put("'',")?;
                out.reference(placement)
            })?;
            Ok((id, parameters))
        }
        SurfaceKind::Cylinder => {
            let cylinder = source.cylindrical_geometry().expect("cylinder data");
            let parameters = WrittenSurfaceParameters {
                u: Vector2::new([Real::one(), Real::zero()]),
                v: Vector2::new([Real::zero(), divide(Real::one())?]),
            };
            let radius = length(cylinder.radius, scale, out)?;
            let placement = placement(out, cylinder.origin, cylinder.axis, cylinder.x, scale)?;
            let id = out.entity("CYLINDRICAL_SURFACE", |out| {
                out.put("'',")?;
                out.reference(placement)?;
                out.put(",")?;
                out.number(&radius)
            })?;
            Ok((id, parameters))
        }
        kind => Err(out.site.error(WriteErrorKind::UnsupportedSurface(kind))),
    }
}

pub(super) fn line(out: &mut Output, origin: u64, velocity: &Vector3) -> Result<u64, WriteError> {
    let direction = direction(out, velocity)?;
    let vector = out.entity("VECTOR", |out| {
        out.put("'',")?;
        out.reference(direction)?;
        out.put(",1.")
    })?;
    out.entity("LINE", |out| {
        out.put("'',")?;
        out.reference(origin)?;
        out.put(",")?;
        out.reference(vector)
    })
}
