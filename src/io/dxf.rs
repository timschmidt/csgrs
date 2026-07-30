//! Strict DXF import and triangle export.

use crate::io::{IoError, finite_f64, finite_triangle_normal, triangulate_planar_face};
use chrono::{DateTime, Local, Utc};
use dxf::Drawing;
use dxf::entities::{Entity, EntityType, Face3D};
use hyperlattice::{Point3, Real, Vector3};
use hypermesh::{Triangle, TriangleMesh};
use std::cmp::Ordering;
use std::io::Cursor;

// `Drawing::new()` initializes its creation and update dates from the wall
// clock. Besides making otherwise identical exports differ, the decimal DXF
// representation of those dates can change length from one second to the next.
// Use a documented sentinel so `to_dxf` is reproducible.
const DXF_EXPORT_TIMESTAMP_SECONDS: i64 = 946_728_000; // 2000-01-01 12:00:00 UTC

fn reproducible_drawing() -> Drawing {
    let mut drawing = Drawing::new();
    let timestamp = DateTime::<Utc>::from_timestamp(DXF_EXPORT_TIMESTAMP_SECONDS, 0)
        .expect("the fixed DXF export timestamp is valid");
    let local_timestamp = timestamp.with_timezone(&Local);
    drawing.header.creation_date = local_timestamp;
    drawing.header.update_date = local_timestamp;
    drawing.header.creation_date_universal = timestamp;
    drawing.header.update_date_universal = timestamp;
    drawing
}

fn real(value: f64, field: &'static str) -> Result<Real, IoError> {
    Real::try_from(value).map_err(|error| {
        IoError::MalformedInput(format!("DXF {field} is not finite: {error}"))
    })
}

fn ocs_basis(normal: dxf::Vector) -> Result<(Vector3, Vector3, Vector3), IoError> {
    let normal = Vector3::from_xyz(
        real(normal.x, "normal x")?,
        real(normal.y, "normal y")?,
        real(normal.z, "normal z")?,
    )
    .normalize_checked()
    .map_err(|error| IoError::MalformedInput(format!("invalid DXF OCS normal: {error}")))?;
    let threshold = (Real::one() / Real::from(64_u8)).map_err(|_| IoError::Geometry {
        format: "DXF",
        detail: "could not construct OCS basis threshold".into(),
    })?;
    let component_is_small = |component: &Real| {
        hyperlimit::compare_reals(&component.abs(), &threshold, crate::PREDICATE_POLICY)
            .value()
            .map(|ordering| ordering == Ordering::Less)
            .ok_or_else(|| IoError::Geometry {
                format: "DXF",
                detail: "OCS reference-axis selection is indeterminate".into(),
            })
    };
    let reference = if component_is_small(&normal.0[0])? && component_is_small(&normal.0[1])? {
        Vector3::y()
    } else {
        Vector3::z()
    };
    let x_axis = reference
        .cross(&normal)
        .normalize_checked()
        .map_err(|error| IoError::MalformedInput(format!("invalid DXF OCS basis: {error}")))?;
    let y_axis = normal.cross(&x_axis);
    Ok((x_axis, y_axis, normal))
}

fn ocs_point(
    point: dxf::Point,
    basis: &(Vector3, Vector3, Vector3),
) -> Result<Point3, IoError> {
    let x = real(point.x, "point x")?;
    let y = real(point.y, "point y")?;
    let z = real(point.z, "point z")?;
    let vector = basis.0.clone() * x + basis.1.clone() * y + basis.2.clone() * z;
    Ok(Point3::new(
        vector.0[0].clone(),
        vector.0[1].clone(),
        vector.0[2].clone(),
    ))
}

fn push_polygon(points: Vec<Point3>) -> Result<(Vec<Point3>, Vec<Triangle>), IoError> {
    let normal = (&points[1] - &points[0])
        .unit_cross_checked(&(&points[2] - &points[0]))
        .map_err(|error| IoError::Geometry {
            format: "DXF",
            detail: format!("entity has a degenerate surface normal: {error}"),
        })?;
    let _ = normal;
    let face = (0..points.len()).collect::<Vec<_>>();
    let triangles = triangulate_planar_face(&points, &face, "DXF")?;
    Ok((points, triangles))
}

/// Imports supported DXF surface entities as native triangle geometry.
pub fn from_dxf(data: &[u8]) -> Result<TriangleMesh, IoError> {
    let drawing = Drawing::load(&mut Cursor::new(data))?;
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut append_polygon = |points: Vec<Point3>| -> Result<(), IoError> {
        let base = positions.len();
        let (new_positions, new_triangles) = push_polygon(points)?;
        positions.extend(new_positions);
        triangles.extend(new_triangles.into_iter().map(|triangle| {
            let [a, b, c] = triangle.indices();
            Triangle::new(base + a, base + b, base + c)
        }));
        Ok(())
    };

    for entity in drawing.entities() {
        match &entity.specific {
            EntityType::Polyline(polyline) if polyline.is_closed() => {
                if polyline.thickness != 0.0 {
                    return Err(IoError::Unsupported {
                        format: "DXF",
                        detail: "closed POLYLINE thickness is not yet supported".into(),
                    });
                }
                let basis = ocs_basis(polyline.normal.clone())?;
                let points = polyline
                    .vertices()
                    .map(|vertex| ocs_point(vertex.location.clone(), &basis))
                    .collect::<Result<Vec<_>, _>>()?;
                if points.len() < 3 {
                    return Err(IoError::MalformedInput(
                        "DXF closed POLYLINE has fewer than three vertices".into(),
                    ));
                }
                append_polygon(points)?;
            },
            EntityType::Circle(circle) => {
                if circle.thickness != 0.0 {
                    return Err(IoError::Unsupported {
                        format: "DXF",
                        detail: "CIRCLE thickness is not yet supported".into(),
                    });
                }
                if !circle.radius.is_finite() || circle.radius <= 0.0 {
                    return Err(IoError::MalformedInput(
                        "DXF CIRCLE radius must be finite and positive".into(),
                    ));
                }
                let basis = ocs_basis(circle.normal.clone())?;
                let center = ocs_point(circle.center.clone(), &basis)?;
                let radius = real(circle.radius, "circle radius")?;
                let mut points = Vec::with_capacity(64);
                for index in 0..64 {
                    let angle = std::f64::consts::TAU * index as f64 / 64.0;
                    let offset = basis.0.clone()
                        * (radius.clone() * real(angle.cos(), "cosine")?)
                        + basis.1.clone() * (radius.clone() * real(angle.sin(), "sine")?);
                    points.push(center.clone() + offset);
                }
                append_polygon(points)?;
            },
            EntityType::Solid(solid) => {
                let basis = ocs_basis(solid.extrusion_direction.clone())?;
                let bottom = vec![
                    ocs_point(solid.first_corner.clone(), &basis)?,
                    ocs_point(solid.second_corner.clone(), &basis)?,
                    ocs_point(solid.fourth_corner.clone(), &basis)?,
                    ocs_point(solid.third_corner.clone(), &basis)?,
                ];
                if solid.thickness == 0.0 {
                    append_polygon(bottom)?;
                } else {
                    let height = real(solid.thickness, "SOLID thickness")?;
                    let translation = basis.2.clone() * height;
                    let top = bottom
                        .iter()
                        .map(|point| point.clone() + translation.clone())
                        .collect::<Vec<_>>();
                    let mut reversed = bottom.clone();
                    reversed.reverse();
                    append_polygon(reversed)?;
                    append_polygon(top.clone())?;
                    for index in 0..4 {
                        let next = (index + 1) % 4;
                        append_polygon(vec![
                            bottom[index].clone(),
                            bottom[next].clone(),
                            top[next].clone(),
                            top[index].clone(),
                        ])?;
                    }
                }
            },
            EntityType::Face3D(face) => {
                let mut points = vec![
                    point_from_wcs(face.first_corner.clone())?,
                    point_from_wcs(face.second_corner.clone())?,
                    point_from_wcs(face.third_corner.clone())?,
                ];
                let fourth = point_from_wcs(face.fourth_corner.clone())?;
                let fourth_limit = hyperlimit::Point3::new(
                    fourth.x.clone(),
                    fourth.y.clone(),
                    fourth.z.clone(),
                );
                let third_limit = hyperlimit::Point3::new(
                    points[2].x.clone(),
                    points[2].y.clone(),
                    points[2].z.clone(),
                );
                let same_as_third = hyperlimit::point3_equal(
                    &fourth_limit,
                    &third_limit,
                    crate::PREDICATE_POLICY,
                )
                .value()
                .ok_or_else(|| IoError::Geometry {
                    format: "DXF",
                    detail: "3DFACE fourth-corner incidence is indeterminate".into(),
                })?;
                if !same_as_third {
                    points.push(fourth);
                }
                append_polygon(points)?;
            },
            other => {
                return Err(IoError::Unsupported {
                    format: "DXF",
                    detail: format!("entity {other:?}"),
                });
            },
        }
    }
    Ok(TriangleMesh::new(positions, triangles))
}

fn point_from_wcs(point: dxf::Point) -> Result<Point3, IoError> {
    Ok(Point3::new(
        real(point.x, "point x")?,
        real(point.y, "point y")?,
        real(point.z, "point z")?,
    ))
}

/// Export triangles as DXF `3DFACE` entities.
pub fn to_dxf(mesh: &TriangleMesh) -> Result<Vec<u8>, IoError> {
    let mut drawing = reproducible_drawing();
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices();
        let points = [&mesh.positions[a], &mesh.positions[b], &mesh.positions[c]];
        let result = (|| {
            finite_triangle_normal(points, "DXF")?;
            let points = points
                .iter()
                .map(|point| {
                    Ok(dxf::Point::new(
                        finite_f64(&point.x, "DXF", "vertex x")?,
                        finite_f64(&point.y, "DXF", "vertex y")?,
                        finite_f64(&point.z, "DXF", "vertex z")?,
                    ))
                })
                .collect::<Result<Vec<_>, IoError>>()?;
            Ok::<_, IoError>(Face3D::new(
                points[0].clone(),
                points[1].clone(),
                points[2].clone(),
                points[2].clone(),
            ))
        })();
        drawing.add_entity(Entity::new(EntityType::Face3D(result?)));
    }
    let mut buffer = Vec::new();
    drawing.save(&mut buffer)?;
    Ok(buffer)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sampled_sphere_is_representable_at_the_dxf_boundary() {
        let sphere = crate::solid::sphere(Real::from(4_u8), 20, 10);
        let bytes = to_dxf(&sphere).unwrap();
        assert_eq!(
            from_dxf(&bytes).unwrap().triangles.len(),
            sphere.triangles.len()
        );
    }
}
