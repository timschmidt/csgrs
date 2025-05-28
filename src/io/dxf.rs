use crate::{CSG, Vertex};
use crate::polygon::Polygon;
use crate::float_types::Real;

use std::error::Error;
use std::fmt::Debug;
use nalgebra::{Point3, Vector3};

#[cfg(any(feature = "stl-io", feature = "dxf-io"))]
use core2::io::Cursor;

#[cfg(feature = "dxf-io")]
use dxf::Drawing;
#[cfg(feature = "dxf-io")]
use dxf::entities::*;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Import a CSG object from DXF data.
    ///
    /// ## Parameters
    /// - `dxf_data`: A byte slice containing the DXF file data.
    /// - `metadata`: metadata that will be attached to all polygons of the resulting `CSG`
    ///
    /// ## Returns
    /// A `Result` containing the CSG object or an error if parsing fails.
    #[cfg(feature = "dxf-io")]
    pub fn from_dxf(dxf_data: &[u8], metadata: Option<S>) -> Result<CSG<S>, Box<dyn Error>> {
        use geo::{line_string, Polygon as GeoPolygon};

        // Load the DXF drawing from the provided data
        let drawing = Drawing::load(&mut Cursor::new(dxf_data))?;

        let mut polygons = Vec::new();

        for entity in drawing.entities() {
            match &entity.specific {
                EntityType::Line(_line) => {
                    // Convert a line to a thin rectangular polygon (optional)
                    // Alternatively, skip lines if they don't form closed loops
                    // Here, we'll skip standalone lines
                    // To form polygons from lines, you'd need to group connected lines into loops
                }
                EntityType::Polyline(polyline) => {
                    // Handle POLYLINE entities (which can be 2D or 3D)
                    if polyline.is_closed() {
                        let mut verts = Vec::new();
                        for vertex in polyline.vertices() {
                            verts.push(Vertex::new(
                                Point3::new(
                                    vertex.location.x as Real,
                                    vertex.location.y as Real,
                                    vertex.location.z as Real,
                                ),
                                Vector3::z(), // Assuming flat in XY
                            ));
                        }
                        // Create a polygon from the polyline vertices
                        if verts.len() >= 3 {
                            polygons.push(Polygon::new(verts, None));
                        }
                    }
                }
                EntityType::Circle(circle) => {
                    // Approximate circles with regular polygons
                    let center = Point3::new(
                        circle.center.x as Real,
                        circle.center.y as Real,
                        circle.center.z as Real,
                    );
                    let radius = circle.radius as Real;
                    // FIXME: this seems a bit low maybe make it relative to the radius
                    let segments = 32; // Number of segments to approximate the circle

                    let mut verts = Vec::with_capacity(segments + 1);
                    let normal = Vector3::new(
                        circle.normal.x as Real,
                        circle.normal.y as Real,
                        circle.normal.z as Real
                    ).normalize();

                    for i in 0..segments {
                        let theta = 2.0 * crate::float_types::PI * (i as Real) / (segments as Real);
                        let x = center.x as Real + radius * theta.cos();
                        let y = center.y as Real + radius * theta.sin();
                        let z = center.z as Real;
                        verts.push(Vertex::new(Point3::new(x, y, z), normal));
                    }

                    // Create a polygon from the approximated circle vertices
                    polygons.push(Polygon::new(verts, metadata.clone()));
                }
                EntityType::Solid(solid) => {
                    let thickness = solid.thickness as Real;
                    let extrusion_direction = Vector3::new(
                        solid.extrusion_direction.x as Real,
                        solid.extrusion_direction.y as Real,
                        solid.extrusion_direction.z as Real
                    );

                    let extruded = CSG::from_geo(
                        GeoPolygon::new(line_string![
                            (x: solid.first_corner.x as Real, y: solid.first_corner.y as Real),
                            (x: solid.second_corner.x as Real, y: solid.second_corner.y as Real),
                            (x: solid.third_corner.x as Real, y: solid.third_corner.y as Real),
                            (x: solid.fourth_corner.x as Real, y: solid.fourth_corner.y as Real),
                            (x: solid.first_corner.x as Real, y: solid.first_corner.y as Real),
                        ], Vec::new()).into(),
                        None,
                        )
                            .extrude_vector(extrusion_direction * thickness).polygons;

                        polygons.extend(extruded);
                }
                // todo convert image to work with `from_image`
                // EntityType::Image(image) => {}
                // todo convert image to work with `text`, also try using system fonts for a better chance of having the font
                // EntityType::Text(text) => {}
                // Handle other entity types as needed (e.g., Line, Spline)
                _ => {
                    // Ignore unsupported entity types for now
                }
            }
        }

        Ok(CSG::from_polygons(&polygons))
    }

    /// Export the CSG object to DXF format.
    ///
    /// # Returns
    ///
    /// A `Result` containing the DXF file as a byte vector or an error if exporting fails.
    #[cfg(feature = "dxf-io")]
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        let mut drawing = Drawing::new();

        for poly in &self.polygons {
            // Triangulate the polygon if it has more than 3 vertices
            let triangles = if poly.vertices.len() > 3 {
                poly.tessellate()
            } else {
                vec![[
                    poly.vertices[0].clone(),
                    poly.vertices[1].clone(),
                    poly.vertices[2].clone(),
                ]]
            };

            for tri in triangles {
                // Create a 3DFACE entity for each triangle
                let face = dxf::entities::Face3D::new(
                    // 3DFACE expects four vertices, but for triangles, the fourth is the same as the third
                    dxf::Point::new(
                        tri[0].pos.x as f64,
                        tri[0].pos.y as f64,
                        tri[0].pos.z as f64,
                    ),
                    dxf::Point::new(
                        tri[1].pos.x as f64,
                        tri[1].pos.y as f64,
                        tri[1].pos.z as f64,
                    ),
                    dxf::Point::new(
                        tri[2].pos.x as f64,
                        tri[2].pos.y as f64,
                        tri[2].pos.z as f64,
                    ),
                    dxf::Point::new(
                        tri[2].pos.x as f64,
                        tri[2].pos.y as f64,
                        tri[2].pos.z as f64,
                    ), // Duplicate for triangular face
                );

                let entity = dxf::entities::Entity::new(dxf::entities::EntityType::Face3D(face));

                // Add the 3DFACE entity to the drawing
                drawing.add_entity(entity);
            }
        }

        // Serialize the DXF drawing to bytes
        let mut buffer = Vec::new();
        drawing.save(&mut buffer)?;

        Ok(buffer)
    }
    
}
