//! DXF import support for converting 2D CAD entities into `csgrs` geometry.

use crate::float_types::{
    Real, hreal_f64_gt, hreal_from_f64, hunit_vector3, hvector3_from_point3, tolerance,
};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::sketch::Profile;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::error::Error;
use std::fmt::Debug;

use dxf::Drawing;
use dxf::entities::*;
use hypercurve::Point2;
use std::io::Cursor;

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    #[doc = " Import a Mesh object from DXF data."]
    #[doc = ""]
    #[doc = " ## Parameters"]
    #[doc = " - `dxf_data`: A byte slice containing the DXF file data."]
    #[doc = " - `metadata`: metadata that will be attached to all polygons of the resulting `Mesh`"]
    #[doc = ""]
    #[doc = " ## Returns"]
    #[doc = " A `Result` containing the Mesh object or an error if parsing fails."]
    ///
    /// DXF entity coordinates are treated as primitive file-boundary data.
    /// Points and normals are promoted through hyperlattice adapters before
    /// becoming mesh vertices, so malformed or non-finite entity data is
    /// skipped instead of being sanitized into CAD topology. This follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_dxf(dxf_data: &[u8], metadata: M) -> Result<Mesh<M>, Box<dyn Error>> {
        let drawing = Drawing::load(&mut Cursor::new(dxf_data))?;
        let mut polygons = Vec::new();

        for entity in drawing.entities() {
            match &entity.specific {
                EntityType::Line(_line) => {},
                EntityType::Polyline(polyline) => {
                    if polyline.is_closed() {
                        let mut verts = Vec::new();
                        for vertex in polyline.vertices() {
                            let point = Point3::new(
                                vertex.location.x as Real,
                                vertex.location.y as Real,
                                vertex.location.z as Real,
                            );
                            if hvector3_from_point3(&point).is_some() {
                                verts.push(Vertex::new(point, Vector3::z()));
                            }
                        }
                        if verts.len() >= 3 {
                            polygons.push(Polygon::new(verts, metadata.clone()));
                        }
                    }
                },
                EntityType::Circle(circle) => {
                    let center = Point3::new(
                        circle.center.x as Real,
                        circle.center.y as Real,
                        circle.center.z as Real,
                    );
                    if hvector3_from_point3(&center).is_none() {
                        continue;
                    }
                    let radius = circle.radius as Real;
                    if !hreal_f64_gt(radius, tolerance()) {
                        continue;
                    }
                    let segments = 32;
                    let mut verts = Vec::with_capacity(segments + 1);
                    let Some(normal) = hunit_vector3(&Vector3::new(
                        circle.normal.x as Real,
                        circle.normal.y as Real,
                        circle.normal.z as Real,
                    )) else {
                        continue;
                    };
                    for i in 0..segments {
                        let theta =
                            2.0 * crate::float_types::PI * (i as Real) / (segments as Real);
                        let x = center.x as Real + radius * theta.cos();
                        let y = center.y as Real + radius * theta.sin();
                        let z = center.z as Real;
                        verts.push(Vertex::new(Point3::new(x, y, z), normal));
                    }
                    polygons.push(Polygon::new(verts, metadata.clone()));
                },
                EntityType::Solid(solid) => {
                    let thickness = solid.thickness as Real;
                    let extrusion_direction = Vector3::new(
                        solid.extrusion_direction.x as Real,
                        solid.extrusion_direction.y as Real,
                        solid.extrusion_direction.z as Real,
                    );
                    let points = [
                        Point2::new(
                            hreal_from_f64(solid.first_corner.x as Real)?,
                            hreal_from_f64(solid.first_corner.y as Real)?,
                        ),
                        Point2::new(
                            hreal_from_f64(solid.second_corner.x as Real)?,
                            hreal_from_f64(solid.second_corner.y as Real)?,
                        ),
                        Point2::new(
                            hreal_from_f64(solid.third_corner.x as Real)?,
                            hreal_from_f64(solid.third_corner.y as Real)?,
                        ),
                        Point2::new(
                            hreal_from_f64(solid.fourth_corner.x as Real)?,
                            hreal_from_f64(solid.fourth_corner.y as Real)?,
                        ),
                    ];
                    let ring = [
                        points[0].clone(),
                        points[1].clone(),
                        points[2].clone(),
                        points[3].clone(),
                        points[0].clone(),
                    ];
                    let extruded = Profile::polygon_points(&ring, metadata.clone())
                        .extrude_vector(extrusion_direction * thickness)
                        .polygons;
                    polygons.extend(extruded);
                },
                _ => {},
            }
        }

        Ok(Mesh::from_polygons(&polygons, metadata))
    }
}

#[doc = " Export any `Triangulated3D` shape to DXF format."]
#[doc = ""]
#[doc = " # Returns"]
#[doc = " A `Result` containing the DXF file as a byte vector or an error if exporting fails."]
pub fn to_dxf<T: Triangulated3D>(shape: &T) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut drawing = Drawing::new();

    shape.visit_triangles(|tri| {
        #[allow(clippy::unnecessary_cast)]
        let face = dxf::entities::Face3D::new(
            dxf::Point::new(
                tri[0].position.x as f64,
                tri[0].position.y as f64,
                tri[0].position.z as f64,
            ),
            dxf::Point::new(
                tri[1].position.x as f64,
                tri[1].position.y as f64,
                tri[1].position.z as f64,
            ),
            dxf::Point::new(
                tri[2].position.x as f64,
                tri[2].position.y as f64,
                tri[2].position.z as f64,
            ),
            // OBJ/CSG triangles have 3 distinct vertices; DXF Face3D needs 4, so repeat the last.
            dxf::Point::new(
                tri[2].position.x as f64,
                tri[2].position.y as f64,
                tri[2].position.z as f64,
            ),
        );
        let entity = dxf::entities::Entity::new(dxf::entities::EntityType::Face3D(face));
        drawing.add_entity(entity);
    });

    let mut buffer = Vec::new();
    drawing.save(&mut buffer)?;
    Ok(buffer)
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        self::to_dxf(self)
    }
}

impl<M: Clone + Debug + Send + Sync> Profile<M> {
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        self::to_dxf(self)
    }
}
