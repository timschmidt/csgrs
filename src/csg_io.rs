//! File import/export of `CSG`s in [DXF], and [STL]
//!
//! [DXF]: https://en.wikipedia.org/wiki/AutoCAD_DXF
//! [STL]: https://en.wikipedia.org/wiki/STL_(file_format)
// todo move to io mod

use crate::csg::{CSG, CSGError};
use crate::float_types::{PI, Real};
use crate::polygon::Polygon;
use crate::vertex::Vertex;

use std::error::Error;
use std::fmt::Debug;

use geo::CoordsIter;
use nalgebra::{Point3, Vector3};

#[cfg(any(feature = "stl-io", feature = "dxf-io"))]
use core2::io::Cursor;

#[cfg(feature = "dxf-io")]
use dxf::Drawing;
#[cfg(feature = "dxf-io")]
use dxf::entities::*;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Export to ASCII STL
    /// 1) 3D polygons in `self.polygons`,
    /// 2) any 2D Polygons or MultiPolygons in `self.geometry` (tessellated in XY).
    ///
    /// Convert this CSG to an **ASCII STL** string with the given `name`.
    ///
    /// ```
    /// # use csgrs::csg::CSG;
    /// let csg = CSG::<()>::cube(1.4, 5.0, 8.9, None);
    /// let stl_text = csg.to_stl_ascii("my_solid");
    /// println!("{}", stl_text);
    /// ```
    pub fn to_stl_ascii(&self, name: &str) -> String {
        let mut out = String::new();
        out.push_str(&format!("solid {}\n", name));

        //
        // (A) Write out all *3D* polygons
        //
        for poly in &self.polygons {
            // Ensure the polygon is tessellated, since STL is triangle-based.
            let triangles = poly.tessellate().expect("expected at least three vertces");
            // A typical STL uses the face normal; we can take the polygon’s plane normal:
            let normal = poly.plane.normal.normalize();

            for tri in triangles {
                out.push_str(&format!(
                    "  facet normal {:.6} {:.6} {:.6}\n",
                    normal.x, normal.y, normal.z
                ));
                out.push_str("    outer loop\n");
                for vertex in &tri {
                    out.push_str(&format!(
                        "      vertex {:.6} {:.6} {:.6}\n",
                        vertex.pos.x, vertex.pos.y, vertex.pos.z
                    ));
                }
                out.push_str("    endloop\n");
                out.push_str("  endfacet\n");
            }
        }

        //
        // (B) Write out all *2D* geometry from `self.geometry`
        //     We only handle Polygon and MultiPolygon.  We tessellate in XY, set z=0.
        //
        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Outer ring (in CCW for a typical “positive” polygon)
                    let outer = poly2d
                        .exterior()
                        .coords_iter()
                        .map(|c| [c.x, c.y])
                        .collect::<Vec<[Real; 2]>>();

                    // Collect holes
                    let holes_vec = poly2d
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect::<Vec<_>>())
                        .collect::<Vec<_>>();
                    let hole_refs = holes_vec
                        .iter()
                        .map(|hole_coords| &hole_coords[..])
                        .collect::<Vec<_>>();

                    // Triangulate with our existing helper:
                    let triangles_2d = Self::tessellate_2d(&outer, &hole_refs);

                    // Write each tri as a facet in ASCII STL, with a normal of (0,0,1)
                    for tri in triangles_2d {
                        out.push_str("  facet normal 0.000000 0.000000 1.000000\n");
                        out.push_str("    outer loop\n");
                        for pt in &tri {
                            out.push_str(&format!(
                                "      vertex {:.6} {:.6} {:.6}\n",
                                pt.x, pt.y, pt.z
                            ));
                        }
                        out.push_str("    endloop\n");
                        out.push_str("  endfacet\n");
                    }
                }

                geo::Geometry::MultiPolygon(mp) => {
                    // Each polygon inside the MultiPolygon
                    for poly2d in &mp.0 {
                        let outer = poly2d
                            .exterior()
                            .coords_iter()
                            .map(|c| [c.x, c.y])
                            .collect::<Vec<[Real; 2]>>();

                        // Holes
                        let holes_vec = poly2d
                            .interiors()
                            .iter()
                            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect::<Vec<_>>())
                            .collect::<Vec<_>>();
                        let hole_refs = holes_vec
                            .iter()
                            .map(|hole_coords| &hole_coords[..])
                            .collect::<Vec<_>>();

                        let triangles_2d = Self::tessellate_2d(&outer, &hole_refs);

                        for tri in triangles_2d {
                            out.push_str("  facet normal 0.000000 0.000000 1.000000\n");
                            out.push_str("    outer loop\n");
                            for pt in &tri {
                                out.push_str(&format!(
                                    "      vertex {:.6} {:.6} {:.6}\n",
                                    pt.x, pt.y, pt.z
                                ));
                            }
                            out.push_str("    endloop\n");
                            out.push_str("  endfacet\n");
                        }
                    }
                }

                // Skip all other geometry types (LineString, Point, etc.)
                // You can optionally handle them if you like, or ignore them.
                _ => {}
            }
        }

        out.push_str(&format!("endsolid {}\n", name));
        out
    }

    /// Export to BINARY STL (returns `Vec<u8>`)
    ///
    /// Convert this CSG to a **binary STL** byte vector with the given `name`.
    ///
    /// The resulting `Vec<u8>` can then be written to a file or handled in memory:
    ///
    /// ```
    /// # use csgrs::csg::CSG;
    /// # let csg = CSG::<()>::new();
    /// let bytes = csg.to_stl_binary("my_solid").unwrap(); // you should handle errors
    /// std::fs::write("my_solid.stl", bytes);
    /// ```
    #[cfg(feature = "stl-io")]
    pub fn to_stl_binary(&self, _name: &str) -> std::io::Result<Vec<u8>> {
        use core2::io::Cursor;
        use stl_io::{Normal, Triangle, Vertex, write_stl};

        let mut triangles = Vec::new();

        // Triangulate all 3D polygons in self.polygons
        for poly in &self.polygons {
            let normal = poly.plane.normal.normalize();
            // Convert polygon to triangles
            let tri_list = poly.tessellate().expect("expected at least three vertces");
            for tri in tri_list {
                triangles.push(Triangle {
                    normal: Normal::new([normal.x as f32, normal.y as f32, normal.z as f32]),
                    vertices: [
                        Vertex::new([
                            tri[0].pos.x as f32,
                            tri[0].pos.y as f32,
                            tri[0].pos.z as f32,
                        ]),
                        Vertex::new([
                            tri[1].pos.x as f32,
                            tri[1].pos.y as f32,
                            tri[1].pos.z as f32,
                        ]),
                        Vertex::new([
                            tri[2].pos.x as f32,
                            tri[2].pos.y as f32,
                            tri[2].pos.z as f32,
                        ]),
                    ],
                });
            }
        }

        //
        // (B) Triangulate any 2D geometry from self.geometry (Polygon, MultiPolygon).
        //     We treat these as lying in the XY plane, at Z=0, with a default normal of +Z.
        //
        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Gather outer ring as [x,y]
                    let outer: Vec<[Real; 2]> = poly2d
                        .exterior().coords_iter()
                        .map(|c| [c.x, c.y])
                        .collect();

                    // Gather holes
                    let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                        .collect();

                    // Convert each hole to a slice-reference for triangulation
                    let hole_refs: Vec<&[[Real; 2]]> = holes_vec.iter().map(|h| &h[..]).collect();

                    // Triangulate using our geo-based helper
                    let tri_2d = Self::tessellate_2d(&outer, &hole_refs);

                    // Each triangle is in XY, so normal = (0,0,1)
                    for tri_pts in tri_2d {
                        triangles.push(Triangle {
                            normal: Normal::new([0.0, 0.0, 1.0]),
                            vertices: [
                                Vertex::new([
                                    tri_pts[0].x as f32,
                                    tri_pts[0].y as f32,
                                    tri_pts[0].z as f32,
                                ]),
                                Vertex::new([
                                    tri_pts[1].x as f32,
                                    tri_pts[1].y as f32,
                                    tri_pts[1].z as f32,
                                ]),
                                Vertex::new([
                                    tri_pts[2].x as f32,
                                    tri_pts[2].y as f32,
                                    tri_pts[2].z as f32,
                                ]),
                            ],
                        });
                    }
                }

                geo::Geometry::MultiPolygon(mpoly) => {
                    // Same approach, but each Polygon in the MultiPolygon
                    for poly2d in &mpoly.0 {
                        let outer: Vec<[Real; 2]> = poly2d
                            .exterior().coords_iter()
                            .map(|c| [c.x, c.y])
                            .collect();

                        let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
                            .interiors()
                            .iter()
                            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                            .collect();

                        let hole_refs: Vec<&[[Real; 2]]> = holes_vec.iter().map(|h| &h[..]).collect();
                        let tri_2d = Self::tessellate_2d(&outer, &hole_refs);

                        for tri_pts in tri_2d {
                            triangles.push(Triangle {
                                normal: Normal::new([0.0, 0.0, 1.0]),
                                vertices: [
                                    Vertex::new([
                                        tri_pts[0].x as f32,
                                        tri_pts[0].y as f32,
                                        tri_pts[0].z as f32,
                                    ]),
                                    Vertex::new([
                                        tri_pts[1].x as f32,
                                        tri_pts[1].y as f32,
                                        tri_pts[1].z as f32,
                                    ]),
                                    Vertex::new([
                                        tri_pts[2].x as f32,
                                        tri_pts[2].y as f32,
                                        tri_pts[2].z as f32,
                                    ]),
                                ],
                            });
                        }
                    }
                }

                // Skip other geometry types: lines, points, etc.
                _ => {}
            }
        }

        //
        // (C) Encode into a binary STL buffer
        //
        let mut cursor = Cursor::new(Vec::new());
        write_stl(&mut cursor, triangles.iter())?;
        Ok(cursor.into_inner())
    }

    /// Create a CSG object from STL data using `stl_io`.
    #[cfg(feature = "stl-io")]
    pub fn from_stl(stl_data: &[u8], metadata: Option<S>) -> Result<CSG<S>, std::io::Error> {
        // Create an in-memory cursor from the STL data
        let mut cursor = Cursor::new(stl_data);

        // Create an STL reader from the cursor
        let stl_reader = stl_io::create_stl_reader(&mut cursor)?;

        let mut polygons = Vec::new();

        for tri_result in stl_reader {
            // Handle potential errors from the STL reader
            let tri = tri_result?;

            // Construct vertices and a polygon
            let vertices = &[
                Vertex::new(
                    Point3::new(
                        tri.vertices[0][0] as Real,
                        tri.vertices[0][1] as Real,
                        tri.vertices[0][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
                Vertex::new(
                    Point3::new(
                        tri.vertices[1][0] as Real,
                        tri.vertices[1][1] as Real,
                        tri.vertices[1][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
                Vertex::new(
                    Point3::new(
                        tri.vertices[2][0] as Real,
                        tri.vertices[2][1] as Real,
                        tri.vertices[2][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
            ];
            polygons.push(Polygon::from_tri(vertices, metadata.clone()));
        }

        Ok(CSG::from_polygons(&polygons))
    }

    /// Import a CSG object from DXF data.
    ///
    /// ## Parameters
    /// - `dxf_data`: A byte slice containing the DXF file data.
    /// - `metadata`: metadata that will be attached to all polygons of the resulting `CSG`
    ///
    /// ## Returns
    /// A `Result` containing the CSG object or an error if parsing fails.
    #[cfg(feature = "dxf-io")]
    pub fn from_dxf(dxf_data: &[u8], metadata: Option<S>) -> Result<CSG<S>, either::Either<CSGError, dxf::DxfError>> {
        use geo::{line_string, Polygon as GeoPolygon};

        // Load the DXF drawing from the provided data
        let drawing = match Drawing::load(&mut Cursor::new(dxf_data)) {
            Ok(drawing) => drawing,
            Err(e) => return Err(either::Right(e))
        };

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
                        let normal = Vector3::new(polyline.normal.x, polyline.normal.y, polyline.normal.z);

                        let mut verts = Vec::new();
                        for vertex in polyline.vertices() {
                            verts.push(Vertex::new(
                                Point3::new(
                                    vertex.location.x as Real,
                                    vertex.location.y as Real,
                                    vertex.location.z as Real,
                                ),
                                normal,
                            ));
                        }
                        // Create a polygon from the polyline vertices
                        if verts.len() >= 3 {
                            let poly = match Polygon::new(verts, metadata.clone()) {
                                Ok(p) => p,
                                Err(e) => return Err(either::Either::Left(e)),
                            };
                            polygons.push(poly);
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
                        let theta = 2.0 * PI * (i as Real) / (segments as Real);
                        let x = center.x as Real + radius * theta.cos();
                        let y = center.y as Real + radius * theta.sin();
                        let z = center.z as Real;
                        verts.push(Vertex::new(Point3::new(x, y, z), normal));
                    }

                    // Create a polygon from the approximated circle vertices
                    let poly = match Polygon::new(verts, metadata.clone()) {
                        Ok(p) => p,
                        Err(e) => return Err(either::Either::Left(e)),
                    };
                    polygons.push(poly);
                }
                EntityType::Solid(solid) => {
                    let thickness = solid.thickness as Real;
                    let extrusion_direction = Vector3::new(
                        solid.extrusion_direction.x as Real,
                        solid.extrusion_direction.y as Real,
                        solid.extrusion_direction.z as Real
                    );

                    let shape_2d = CSG::from_geo(
                        GeoPolygon::new(line_string![
                            (x: solid.first_corner.x as Real, y: solid.first_corner.y as Real),
                            (x: solid.second_corner.x as Real, y: solid.second_corner.y as Real),
                            (x: solid.third_corner.x as Real, y: solid.third_corner.y as Real),
                            (x: solid.fourth_corner.x as Real, y: solid.fourth_corner.y as Real),
                            (x: solid.first_corner.x as Real, y: solid.first_corner.y as Real),
                        ], Vec::new()).into(),
                        None,
                    );
                    let extruded = match shape_2d.extrude_vector(extrusion_direction * thickness) {
                        Ok(csg) => csg.polygons,
                        Err(e) => return Err(either::Either::Left(e)),
                    };

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
                poly.tessellate()?
            } else {
                vec![[
                    poly.vertices[0].clone(),
                    poly.vertices[1].clone(),
                    poly.vertices[2].clone(),
                ]]
            };

            for tri in triangles {
                // Create a 3DFACE entity for each triangle
                #[allow(clippy::unnecessary_cast)]
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