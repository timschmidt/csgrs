use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::sketch::Sketch;

use geo::CoordsIter;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

use core2::io::Cursor;

use stl_io;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// Export to ASCII STL
    /// Convert this Mesh to an **ASCII STL** string with the given `name`.
    ///
    /// ```rust
    /// # use csgrs::mesh::Mesh;
    /// # use std::error::Error;
    /// # fn main() -> Result<(), Box<dyn Error>> {
    /// let mesh  = Mesh::<()>::cube(1.0, None);
    /// let bytes = mesh.to_stl_ascii("my_solid");
    /// std::fs::write("stl/my_solid.stl", bytes)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_stl_ascii(&self, name: &str) -> String {
        // Use mesh-level triangulation so T-junction fixing is applied.
        let tri_mesh = self.triangulate();

        let mut out = String::new();
        out.push_str(&format!("solid {name}\n"));

        // Each polygon in tri_mesh should be a triangle
        for poly in &tri_mesh.polygons {
            // Skip any degenerate polygons just in case
            if poly.vertices.len() != 3 {
                continue;
            }

            // Face normal from the polygon's plane
            let normal = poly.plane.normal().normalize();

            out.push_str(&format!(
                "  facet normal {:.6} {:.6} {:.6}\n",
                normal.x, normal.y, normal.z
            ));
            out.push_str("    outer loop\n");

            for v in &poly.vertices {
                out.push_str(&format!(
                    "      vertex {:.6} {:.6} {:.6}\n",
                    v.pos.x, v.pos.y, v.pos.z
                ));
            }

            out.push_str("    endloop\n");
            out.push_str("  endfacet\n");
        }

        out.push_str(&format!("endsolid {name}\n"));
        out
    }

    /// Export to BINARY STL (returns `Vec<u8>`)
    ///
    /// Convert this Mesh to a **binary STL** byte vector with the given `name`.
    ///
    /// The resulting `Vec<u8>` can then be written to a file or handled in memory:
    ///
    /// ```rust
    /// # use csgrs::mesh::Mesh;
    /// # use std::error::Error;
    /// # fn main() -> Result<(), Box<dyn Error>> {
    /// let object = Mesh::<()>::cube(1.0, None);
    /// let bytes  = object.to_stl_binary("my_solid")?;
    /// std::fs::write("stl/my_solid.stl", bytes)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_stl_binary(&self, _name: &str) -> std::io::Result<Vec<u8>> {
        use core2::io::Cursor;
        use stl_io::{Normal, Triangle, Vertex, write_stl};

        // Use mesh-level triangulation so T-junction fixing is applied.
        let tri_mesh = self.triangulate();

        let mut triangles = Vec::new();

        for poly in &tri_mesh.polygons {
            // Skip any degenerate polygons just in case
            if poly.vertices.len() != 3 {
                continue;
            }

            let normal = poly.plane.normal().normalize();

            #[allow(clippy::unnecessary_cast)]
            {
                triangles.push(Triangle {
                    normal: Normal::new([normal.x as f32, normal.y as f32, normal.z as f32]),
                    vertices: [
                        Vertex::new([
                            poly.vertices[0].pos.x as f32,
                            poly.vertices[0].pos.y as f32,
                            poly.vertices[0].pos.z as f32,
                        ]),
                        Vertex::new([
                            poly.vertices[1].pos.x as f32,
                            poly.vertices[1].pos.y as f32,
                            poly.vertices[1].pos.z as f32,
                        ]),
                        Vertex::new([
                            poly.vertices[2].pos.x as f32,
                            poly.vertices[2].pos.y as f32,
                            poly.vertices[2].pos.z as f32,
                        ]),
                    ],
                });
            }
        }

        let mut cursor = Cursor::new(Vec::new());
        write_stl(&mut cursor, triangles.iter())?;
        Ok(cursor.into_inner())
    }

    /// Create a Mesh object from STL data using 'stl_io'.
    #[cfg(feature = "stl-io")]
    pub fn from_stl(stl_data: &[u8], metadata: Option<S>) -> Result<Mesh<S>, std::io::Error> {
        // Create an in-memory cursor from the STL data
        let mut cursor = Cursor::new(stl_data);

        // Create an STL reader from the cursor
        let stl_reader = stl_io::create_stl_reader(&mut cursor)?;

        let mut polygons = Vec::new();

        for tri_result in stl_reader {
            // Handle potential errors from the STL reader
            let tri = tri_result?;

            // Construct vertices and a polygon
            let vertices = vec![
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
            polygons.push(Polygon::new(vertices, metadata.clone()));
        }

        Ok(Mesh::from_polygons(&polygons, metadata))
    }
}

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Export to ASCII STL
    /// Convert this Sketch to an **ASCII STL** string with the given 'name'.
    ///
    /// ```
    /// # use csgrs::sketch::Sketch;
    /// # use std::error::Error;
    /// # fn main() -> Result<(), Box<dyn Error>> {
    /// let sketch: Sketch<()> = Sketch::square(2.0, None);
    /// let bytes = sketch.to_stl_ascii("my_sketch");
    /// std::fs::write("stl/my_sketch.stl", bytes)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_stl_ascii(&self, name: &str) -> String {
        let mut out = String::new();
        out.push_str(&format!("solid {name}\n"));

        // Write out all *2D* geometry from `self.geometry`
        // We only handle Polygon and MultiPolygon.  We tessellate in XY, set z=0.
        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Outer ring (in CCW for a typical "positive" polygon)
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
                    let triangles_2d =
                        Sketch::<()>::triangulate_with_holes(&outer, &hole_refs);

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
                },

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
                            .map(|ring| {
                                ring.coords_iter().map(|c| [c.x, c.y]).collect::<Vec<_>>()
                            })
                            .collect::<Vec<_>>();
                        let hole_refs = holes_vec
                            .iter()
                            .map(|hole_coords| &hole_coords[..])
                            .collect::<Vec<_>>();

                        let triangles_2d =
                            Sketch::<()>::triangulate_with_holes(&outer, &hole_refs);

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
                },

                // Skip all other geometry types (LineString, Point, etc.)
                // You can optionally handle them if you like, or ignore them.
                _ => {},
            }
        }

        out.push_str(&format!("endsolid {name}\n"));
        out
    }

    /// Export to BINARY STL (returns `Vec<u8>`)
    ///
    /// Convert this Sketch to a **binary STL** byte vector with the given 'name'.
    ///
    /// The resulting `Vec<u8>` can then be written to a file or handled in memory:
    ///
    /// ```rust
    /// # use csgrs::sketch::Sketch;
    /// # use std::error::Error;
    /// # fn main() -> Result<(), Box<dyn Error>> {
    /// let object = Sketch::<()>::square(1.0, None);
    /// let bytes  = object.to_stl_binary("my_sketch")?;
    /// std::fs::write("stl/my_sketch.stl", bytes)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_stl_binary(&self, _name: &str) -> std::io::Result<Vec<u8>> {
        use core2::io::Cursor;
        use stl_io::{Normal, Triangle, Vertex, write_stl};

        let mut triangles = Vec::new();

        // Triangulate any 2D geometry from self.geometry (Polygon, MultiPolygon).
        // We treat these as lying in the XY plane, at Z=0, with a default normal of +Z.
        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Gather outer ring as [x,y]
                    let outer: Vec<[Real; 2]> =
                        poly2d.exterior().coords_iter().map(|c| [c.x, c.y]).collect();

                    // Gather holes
                    let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                        .collect();

                    // Convert each hole to a slice-reference for triangulation
                    let hole_refs: Vec<&[[Real; 2]]> =
                        holes_vec.iter().map(|h| &h[..]).collect();

                    // Triangulate using our geo-based helper
                    let tri_2d = Sketch::<()>::triangulate_with_holes(&outer, &hole_refs);

                    // Each triangle is in XY, so normal = (0,0,1)
                    #[allow(clippy::unnecessary_cast)]
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
                },

                geo::Geometry::MultiPolygon(mpoly) => {
                    // Same approach, but each Polygon in the MultiPolygon
                    for poly2d in &mpoly.0 {
                        let outer: Vec<[Real; 2]> =
                            poly2d.exterior().coords_iter().map(|c| [c.x, c.y]).collect();

                        let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
                            .interiors()
                            .iter()
                            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                            .collect();

                        let hole_refs: Vec<&[[Real; 2]]> =
                            holes_vec.iter().map(|h| &h[..]).collect();
                        let tri_2d = Sketch::<()>::triangulate_with_holes(&outer, &hole_refs);

                        #[allow(clippy::unnecessary_cast)]
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
                },

                // Skip other geometry types: lines, points, etc.
                _ => {},
            }
        }

        // (C) Encode into a binary STL buffer
        //
        let mut cursor = Cursor::new(Vec::new());
        write_stl(&mut cursor, triangles.iter())?;
        Ok(cursor.into_inner())
    }
}
