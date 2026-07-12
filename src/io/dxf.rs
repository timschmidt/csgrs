//! Strict DXF import and triangle export.

use crate::io::{IoError, finite_f64};
use crate::mesh::Mesh;
use crate::mesh::Polygon;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;
use dxf::Drawing;
use dxf::entities::{Entity, EntityType, Face3D};
use hyperlattice::{Point3, Real, Vector3};
use std::fmt::Debug;
use std::io::Cursor;

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
    let reference = if normal.0[0].abs() < threshold && normal.0[1].abs() < threshold {
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

fn polygon<M: Clone + Send + Sync>(
    points: Vec<Point3>,
    metadata: M,
) -> Result<Polygon<M>, IoError> {
    let normal = (&points[1] - &points[0])
        .unit_cross_checked(&(&points[2] - &points[0]))
        .map_err(|error| IoError::Geometry {
            format: "DXF",
            detail: format!("entity has a degenerate surface normal: {error}"),
        })?;
    Ok(Polygon::new(
        points
            .into_iter()
            .map(|point| Vertex::new(point, normal.clone()))
            .collect(),
        metadata,
    ))
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Import supported DXF surface entities, rejecting unsupported entities.
    pub fn from_dxf(data: &[u8], metadata: M) -> Result<Self, IoError> {
        let drawing = Drawing::load(&mut Cursor::new(data))?;
        let mut polygons = Vec::new();

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
                    polygons.push(polygon(points, metadata.clone())?);
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
                    polygons.push(polygon(points, metadata.clone())?);
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
                        polygons.push(polygon(bottom, metadata.clone())?);
                    } else {
                        let height = real(solid.thickness, "SOLID thickness")?;
                        let translation = basis.2.clone() * height;
                        let top = bottom
                            .iter()
                            .map(|point| point.clone() + translation.clone())
                            .collect::<Vec<_>>();
                        let mut reversed = bottom.clone();
                        reversed.reverse();
                        polygons.push(polygon(reversed, metadata.clone())?);
                        polygons.push(polygon(top.clone(), metadata.clone())?);
                        for index in 0..4 {
                            let next = (index + 1) % 4;
                            polygons.push(polygon(
                                vec![
                                    bottom[index].clone(),
                                    bottom[next].clone(),
                                    top[next].clone(),
                                    top[index].clone(),
                                ],
                                metadata.clone(),
                            )?);
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
                    if fourth != points[2] {
                        points.push(fourth);
                    }
                    polygons.push(polygon(points, metadata.clone())?);
                },
                other => {
                    return Err(IoError::Unsupported {
                        format: "DXF",
                        detail: format!("entity {other:?}"),
                    });
                },
            }
        }
        Ok(Mesh::from_polygons(&polygons))
    }

    pub fn to_dxf(&self) -> Result<Vec<u8>, IoError> {
        to_dxf(self)
    }
}

fn point_from_wcs(point: dxf::Point) -> Result<Point3, IoError> {
    Ok(Point3::new(
        real(point.x, "point x")?,
        real(point.y, "point y")?,
        real(point.z, "point z")?,
    ))
}

/// Export triangles as DXF `3DFACE` entities.
pub fn to_dxf<T: Triangulated3D>(shape: &T) -> Result<Vec<u8>, IoError> {
    let mut drawing = Drawing::new();
    let mut failure = None;
    shape.visit_triangles(|triangle| {
        if failure.is_some() {
            return;
        }
        let result = (|| {
            let first_edge = &triangle[1].position - &triangle[0].position;
            let second_edge = &triangle[2].position - &triangle[0].position;
            first_edge.unit_cross_checked(&second_edge).map_err(|error| {
                IoError::Geometry {
                    format: "DXF",
                    detail: format!("degenerate export triangle: {error}"),
                }
            })?;
            let points = triangle
                .iter()
                .map(|vertex| {
                    Ok(dxf::Point::new(
                        finite_f64(&vertex.position.x, "DXF", "vertex x")?,
                        finite_f64(&vertex.position.y, "DXF", "vertex y")?,
                        finite_f64(&vertex.position.z, "DXF", "vertex z")?,
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
        match result {
            Ok(face) => {
                drawing.add_entity(Entity::new(EntityType::Face3D(face)));
            },
            Err(error) => failure = Some(error),
        }
    });
    if let Some(error) = failure {
        return Err(error);
    }
    let mut buffer = Vec::new();
    drawing.save(&mut buffer)?;
    Ok(buffer)
}

#[cfg(feature = "sketch")]
impl crate::sketch::Profile {
    pub fn to_dxf(&self) -> Result<Vec<u8>, IoError> {
        to_dxf(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DegenerateTriangle;

    impl Triangulated3D for DegenerateTriangle {
        fn visit_triangles<F>(&self, mut visit: F)
        where
            F: FnMut([Vertex; 3]),
        {
            let point = Point3::new(Real::zero(), Real::zero(), Real::zero());
            let vertex = Vertex::new(point, Vector3::z());
            visit([vertex.clone(), vertex.clone(), vertex]);
        }
    }

    #[test]
    fn arbitrary_normal_circle_is_built_in_its_ocs_plane() {
        let mut drawing = Drawing::new();
        let mut circle = dxf::entities::Circle::new(dxf::Point::origin(), 2.0);
        circle.normal = dxf::Vector::x_axis();
        drawing.add_entity(Entity::new(EntityType::Circle(circle)));
        let mut bytes = Vec::new();
        drawing.save(&mut bytes).unwrap();

        let mesh = Mesh::from_dxf(&bytes, ()).unwrap();
        assert!(
            mesh.vertices()
                .iter()
                .all(|vertex| vertex.position.x == Real::zero())
        );
    }

    #[test]
    fn face_export_round_trips_through_dxf_parser() {
        let cube = Mesh::<()>::cube(Real::one(), ());
        let bytes = cube.to_dxf().unwrap();
        let imported = Mesh::from_dxf(&bytes, ()).unwrap();
        assert_eq!(imported.polygons.len(), 12);
    }

    #[test]
    fn unsupported_entities_are_reported() {
        let mut drawing = Drawing::new();
        let line =
            dxf::entities::Line::new(dxf::Point::origin(), dxf::Point::new(1.0, 0.0, 0.0));
        drawing.add_entity(Entity::new(EntityType::Line(line)));
        let mut bytes = Vec::new();
        drawing.save(&mut bytes).unwrap();
        let error = Mesh::<()>::from_dxf(&bytes, ()).unwrap_err();
        assert!(matches!(error, IoError::Unsupported { format: "DXF", .. }));
    }

    #[test]
    fn malformed_input_and_degenerate_export_are_rejected() {
        assert!(Mesh::<()>::from_dxf(b"not a DXF", ()).is_err());
        assert!(to_dxf(&DegenerateTriangle).is_err());
    }
}
