use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;
use crate::sketch::Sketch;
use geo::{Polygon as GeoPolygon, line_string};
use nalgebra::{Point3, Vector3};
use std::error::Error;
use std::fmt::Debug;

use core2::io::Cursor;
use dxf::Drawing;
use dxf::entities::*;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    #[doc = " Import a Mesh object from DXF data."]
    #[doc = ""]
    #[doc = " ## Parameters"]
    #[doc = " - `dxf_data`: A byte slice containing the DXF file data."]
    #[doc = " - `metadata`: metadata that will be attached to all polygons of the resulting `Mesh`"]
    #[doc = ""]
    #[doc = " ## Returns"]
    #[doc = " A `Result` containing the Mesh object or an error if parsing fails."]
    pub fn from_dxf(
        dxf_data: &[u8],
        metadata: Option<S>,
    ) -> Result<Mesh<S>, Box<dyn Error>> {
        let drawing = Drawing::load(&mut Cursor::new(dxf_data))?;
        let mut polygons = Vec::new();

        for entity in drawing.entities() {
            match &entity.specific {
                EntityType::Line(_line) => {}
                EntityType::Polyline(polyline) => {
                    if polyline.is_closed() {
                        let mut verts = Vec::new();
                        for vertex in polyline.vertices() {
                            verts.push(Vertex::new(
                                Point3::new(
                                    vertex.location.x as Real,
                                    vertex.location.y as Real,
                                    vertex.location.z as Real,
                                ),
                                Vector3::z(),
                            ));
                        }
                        if verts.len() >= 3 {
                            polygons.push(Polygon::new(verts, None));
                        }
                    }
                }
                EntityType::Circle(circle) => {
                    let center = Point3::new(
                        circle.center.x as Real,
                        circle.center.y as Real,
                        circle.center.z as Real,
                    );
                    let radius = circle.radius as Real;
                    let segments = 32;
                    let mut verts = Vec::with_capacity(segments + 1);
                    let normal = Vector3::new(
                        circle.normal.x as Real,
                        circle.normal.y as Real,
                        circle.normal.z as Real,
                    )
                    .normalize();
                    for i in 0..segments {
                        let theta = 2.0 * crate::float_types::PI * (i as Real) / (segments as Real);
                        let x = center.x as Real + radius * theta.cos();
                        let y = center.y as Real + radius * theta.sin();
                        let z = center.z as Real;
                        verts.push(Vertex::new(Point3::new(x, y, z), normal));
                    }
                    polygons.push(Polygon::new(verts, metadata.clone()));
                }
                EntityType::Solid(solid) => {
                    let thickness = solid.thickness as Real;
                    let extrusion_direction = Vector3::new(
                        solid.extrusion_direction.x as Real,
                        solid.extrusion_direction.y as Real,
                        solid.extrusion_direction.z as Real,
                    );
                    let extruded = Sketch::from_geo(
                        GeoPolygon::new(
                            line_string![
                                (x: solid.first_corner.x  as Real, y: solid.first_corner.y  as Real),
                                (x: solid.second_corner.x as Real, y: solid.second_corner.y as Real),
                                (x: solid.third_corner.x  as Real, y: solid.third_corner.y  as Real),
                                (x: solid.fourth_corner.x as Real, y: solid.fourth_corner.y as Real),
                                (x: solid.first_corner.x  as Real, y: solid.first_corner.y  as Real),
                            ],
                            Vec::new(),
                        )
                        .into(),
                        None,
                    )
                    .extrude_vector(extrusion_direction * thickness)
                    .polygons;
                    polygons.extend(extruded);
                }
                _ => {}
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

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        self::to_dxf(self)
    }
}

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        self::to_dxf(self)
    }
}

impl<S: Clone + Debug + Send + Sync> crate::bmesh::BMesh<S> {
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        self::to_dxf(self)
    }
}
