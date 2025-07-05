//! AMF file format support for Mesh objects
//!
//! This module provides export functionality for AMF (Additive Manufacturing File Format),
//! an XML-based format specifically designed for 3D printing and additive manufacturing.
use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::sketch::Sketch;
use geo::CoordsIter;
use nalgebra::Point3;
use std::fmt::Debug;
use std::io::Write;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// Export this Mesh to AMF format as a string
    ///
    /// Creates an AMF (Additive Manufacturing File Format) file containing:
    /// 1. All 3D polygons from Mesh (tessellated to triangles)
    ///
    /// AMF is an XML-based format designed for 3D printing with support for:
    /// - Complex 3D geometries
    /// - Multiple materials and colors
    /// - Metadata and manufacturing information
    ///
    /// # Arguments
    /// * `object_name` - Name for the object in the AMF file
    /// * `units` - Units for the geometry (e.g., "millimeter", "inch")
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// let csg: Mesh<()> = Mesh::cube(10.0, None);
    /// let amf_content = csg.to_amf("my_cube", "millimeter");
    /// println!("{}", amf_content);
    /// ```
    pub fn to_amf(&self, object_name: &str, units: &str) -> String {
        let mut amf_content = String::new();

        // AMF XML header
        amf_content.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        amf_content.push_str("<amf unit=\"");
        amf_content.push_str(units);
        amf_content.push_str("\" version=\"1.1\">\n");

        // Metadata
        amf_content.push_str("  <metadata type=\"producer\">csgrs library</metadata>\n");
        amf_content
            .push_str("  <metadata type=\"cad\">Constructive Solid Geometry</metadata>\n");
        amf_content.push_str(&format!(
            "  <metadata type=\"description\">{}</metadata>\n",
            object_name
        ));

        let mut vertices = Vec::new();
        let mut triangles = Vec::new();

        // Process 3D polygons
        for poly in &self.polygons {
            // Tessellate polygon to triangles
            let poly_triangles = poly.triangulate();

            for triangle in poly_triangles {
                let mut triangle_indices = Vec::new();

                for vertex in triangle {
                    let vertex_idx = add_unique_vertex_amf(&mut vertices, vertex.pos);
                    triangle_indices.push(vertex_idx);
                }

                if triangle_indices.len() == 3 {
                    triangles.push(triangle_indices);
                }
            }
        }

        // Start object definition
        amf_content.push_str(&format!("  <object id=\"{}\">\n", object_name));
        amf_content.push_str("    <mesh>\n");

        // Write vertices
        amf_content.push_str("      <vertices>\n");
        for (i, vertex) in vertices.iter().enumerate() {
            amf_content.push_str(&format!("        <vertex id=\"{}\">\n", i));
            amf_content.push_str("          <coordinates>\n");
            amf_content.push_str(&format!("            <x>{:.6}</x>\n", vertex.x));
            amf_content.push_str(&format!("            <y>{:.6}</y>\n", vertex.y));
            amf_content.push_str(&format!("            <z>{:.6}</z>\n", vertex.z));
            amf_content.push_str("          </coordinates>\n");
            amf_content.push_str("        </vertex>\n");
        }
        amf_content.push_str("      </vertices>\n");

        // Write triangles (volume definition)
        amf_content.push_str("      <volume>\n");
        for (i, triangle) in triangles.iter().enumerate() {
            amf_content.push_str(&format!("        <triangle id=\"{}\">\n", i));
            amf_content.push_str(&format!("          <v1>{}</v1>\n", triangle[0]));
            amf_content.push_str(&format!("          <v2>{}</v2>\n", triangle[1]));
            amf_content.push_str(&format!("          <v3>{}</v3>\n", triangle[2]));
            amf_content.push_str("        </triangle>\n");
        }
        amf_content.push_str("      </volume>\n");

        // Close mesh and object
        amf_content.push_str("    </mesh>\n");
        amf_content.push_str("  </object>\n");

        // Close AMF
        amf_content.push_str("</amf>\n");

        amf_content
    }

    /// Export this Mesh to an AMF file
    ///
    /// # Arguments
    /// * `writer` - Where to write the AMF data
    /// * `object_name` - Name for the object in the AMF file
    /// * `units` - Units for the geometry (e.g., "millimeter", "inch")
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use std::fs::File;
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let csg: Mesh<()> = Mesh::cube(10.0, None);
    /// let mut file = File::create("output.amf")?;
    /// csg.write_amf(&mut file, "my_cube", "millimeter")?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn write_amf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
        units: &str,
    ) -> std::io::Result<()> {
        let amf_content = self.to_amf(object_name, units);
        writer.write_all(amf_content.as_bytes())
    }

    /// Export this Mesh to AMF format with color information
    ///
    /// Creates an AMF file with color/material information for enhanced 3D printing.
    ///
    /// # Arguments
    /// * `object_name` - Name for the object in the AMF file
    /// * `units` - Units for the geometry (e.g., "millimeter", "inch")
    /// * `color` - RGB color as (red, green, blue) where each component is 0.0-1.0
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// let csg: Mesh<()> = Mesh::cube(10.0, None);
    /// let amf_content = csg.to_amf_with_color("red_cube", "millimeter", (1.0, 0.0, 0.0));
    /// println!("{}", amf_content);
    /// ```
    pub fn to_amf_with_color(
        &self,
        object_name: &str,
        units: &str,
        color: (Real, Real, Real),
    ) -> String {
        let mut amf_content = String::new();

        // AMF XML header
        amf_content.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        amf_content.push_str("<amf unit=\"");
        amf_content.push_str(units);
        amf_content.push_str("\" version=\"1.1\">\n");

        // Metadata
        amf_content.push_str("  <metadata type=\"producer\">csgrs library</metadata>\n");
        amf_content
            .push_str("  <metadata type=\"cad\">Constructive Solid Geometry</metadata>\n");
        amf_content.push_str(&format!(
            "  <metadata type=\"description\">{}</metadata>\n",
            object_name
        ));

        // Material definition with color
        amf_content.push_str("  <material id=\"material1\">\n");
        amf_content.push_str("    <metadata type=\"name\">Default Material</metadata>\n");
        amf_content.push_str("    <color>\n");
        amf_content.push_str(&format!("      <r>{:.3}</r>\n", color.0));
        amf_content.push_str(&format!("      <g>{:.3}</g>\n", color.1));
        amf_content.push_str(&format!("      <b>{:.3}</b>\n", color.2));
        amf_content.push_str("      <a>1.0</a>\n"); // Alpha (opacity)
        amf_content.push_str("    </color>\n");
        amf_content.push_str("  </material>\n");

        let mut vertices = Vec::new();
        let mut triangles = Vec::new();

        // Process 3D polygons
        for poly in &self.polygons {
            let poly_triangles = poly.triangulate();

            for triangle in poly_triangles {
                let mut triangle_indices = Vec::new();

                for vertex in triangle {
                    let vertex_idx = add_unique_vertex_amf(&mut vertices, vertex.pos);
                    triangle_indices.push(vertex_idx);
                }

                if triangle_indices.len() == 3 {
                    triangles.push(triangle_indices);
                }
            }
        }

        // Start object definition
        amf_content.push_str(&format!("  <object id=\"{}\">\n", object_name));
        amf_content.push_str("    <mesh>\n");

        // Write vertices
        amf_content.push_str("      <vertices>\n");
        for (i, vertex) in vertices.iter().enumerate() {
            amf_content.push_str(&format!("        <vertex id=\"{}\">\n", i));
            amf_content.push_str("          <coordinates>\n");
            amf_content.push_str(&format!("            <x>{:.6}</x>\n", vertex.x));
            amf_content.push_str(&format!("            <y>{:.6}</y>\n", vertex.y));
            amf_content.push_str(&format!("            <z>{:.6}</z>\n", vertex.z));
            amf_content.push_str("          </coordinates>\n");
            amf_content.push_str("        </vertex>\n");
        }
        amf_content.push_str("      </vertices>\n");

        // Write triangles with material reference
        amf_content.push_str("      <volume materialid=\"material1\">\n");
        for (i, triangle) in triangles.iter().enumerate() {
            amf_content.push_str(&format!("        <triangle id=\"{}\">\n", i));
            amf_content.push_str(&format!("          <v1>{}</v1>\n", triangle[0]));
            amf_content.push_str(&format!("          <v2>{}</v2>\n", triangle[1]));
            amf_content.push_str(&format!("          <v3>{}</v3>\n", triangle[2]));
            amf_content.push_str("        </triangle>\n");
        }
        amf_content.push_str("      </volume>\n");

        // Close mesh and object
        amf_content.push_str("    </mesh>\n");
        amf_content.push_str("  </object>\n");

        // Close AMF
        amf_content.push_str("</amf>\n");

        amf_content
    }
}

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Export this Mesh to AMF format as a string
    ///
    /// Creates an AMF (Additive Manufacturing File Format) file containing:
    /// 2. Any 2D geometry from Sketch (extruded/projected to 3D)
    ///
    /// AMF is an XML-based format designed for 3D printing with support for:
    /// - Complex 3D geometries
    /// - Multiple materials and colors
    /// - Metadata and manufacturing information
    ///
    /// # Arguments
    /// * `object_name` - Name for the object in the AMF file
    /// * `units` - Units for the geometry (e.g., "millimeter", "inch")
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// let csg: Mesh<()> = Mesh::cube(10.0, None);
    /// let amf_content = csg.to_amf("my_cube", "millimeter");
    /// println!("{}", amf_content);
    /// ```
    pub fn to_amf(&self, object_name: &str, units: &str) -> String {
        let mut amf_content = String::new();

        // AMF XML header
        amf_content.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        amf_content.push_str("<amf unit=\"");
        amf_content.push_str(units);
        amf_content.push_str("\" version=\"1.1\">\n");

        // Metadata
        amf_content.push_str("  <metadata type=\"producer\">csgrs library</metadata>\n");
        amf_content
            .push_str("  <metadata type=\"cad\">Constructive Solid Geometry</metadata>\n");
        amf_content.push_str(&format!(
            "  <metadata type=\"description\">{}</metadata>\n",
            object_name
        ));

        let mut vertices = Vec::new();
        let mut triangles = Vec::new();

        // Process 2D geometry (project to XY plane at Z=0)
        for geom in &self.geometry.0 {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    self.add_2d_polygon_to_amf(poly2d, &mut vertices, &mut triangles);
                },
                geo::Geometry::MultiPolygon(mp) => {
                    for poly2d in &mp.0 {
                        self.add_2d_polygon_to_amf(poly2d, &mut vertices, &mut triangles);
                    }
                },
                _ => {}, // Skip other geometry types
            }
        }

        // Start object definition
        amf_content.push_str(&format!("  <object id=\"{}\">\n", object_name));
        amf_content.push_str("    <mesh>\n");

        // Write vertices
        amf_content.push_str("      <vertices>\n");
        for (i, vertex) in vertices.iter().enumerate() {
            amf_content.push_str(&format!("        <vertex id=\"{}\">\n", i));
            amf_content.push_str("          <coordinates>\n");
            amf_content.push_str(&format!("            <x>{:.6}</x>\n", vertex.x));
            amf_content.push_str(&format!("            <y>{:.6}</y>\n", vertex.y));
            amf_content.push_str(&format!("            <z>{:.6}</z>\n", vertex.z));
            amf_content.push_str("          </coordinates>\n");
            amf_content.push_str("        </vertex>\n");
        }
        amf_content.push_str("      </vertices>\n");

        // Write triangles (volume definition)
        amf_content.push_str("      <volume>\n");
        for (i, triangle) in triangles.iter().enumerate() {
            amf_content.push_str(&format!("        <triangle id=\"{}\">\n", i));
            amf_content.push_str(&format!("          <v1>{}</v1>\n", triangle[0]));
            amf_content.push_str(&format!("          <v2>{}</v2>\n", triangle[1]));
            amf_content.push_str(&format!("          <v3>{}</v3>\n", triangle[2]));
            amf_content.push_str("        </triangle>\n");
        }
        amf_content.push_str("      </volume>\n");

        // Close mesh and object
        amf_content.push_str("    </mesh>\n");
        amf_content.push_str("  </object>\n");

        // Close AMF
        amf_content.push_str("</amf>\n");

        amf_content
    }

    /// Export this Mesh to AMF format with color information
    ///
    /// Creates an AMF file with color/material information for enhanced 3D printing.
    ///
    /// # Arguments
    /// * `object_name` - Name for the object in the AMF file
    /// * `units` - Units for the geometry (e.g., "millimeter", "inch")
    /// * `color` - RGB color as (red, green, blue) where each component is 0.0-1.0
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// let csg: Mesh<()> = Mesh::cube(10.0, None);
    /// let amf_content = csg.to_amf_with_color("red_cube", "millimeter", (1.0, 0.0, 0.0));
    /// println!("{}", amf_content);
    /// ```
    pub fn to_amf_with_color(
        &self,
        object_name: &str,
        units: &str,
        color: (Real, Real, Real),
    ) -> String {
        let mut amf_content = String::new();

        // AMF XML header
        amf_content.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        amf_content.push_str("<amf unit=\"");
        amf_content.push_str(units);
        amf_content.push_str("\" version=\"1.1\">\n");

        // Metadata
        amf_content.push_str("  <metadata type=\"producer\">csgrs library</metadata>\n");
        amf_content
            .push_str("  <metadata type=\"cad\">Constructive Solid Geometry</metadata>\n");
        amf_content.push_str(&format!(
            "  <metadata type=\"description\">{}</metadata>\n",
            object_name
        ));

        // Material definition with color
        amf_content.push_str("  <material id=\"material1\">\n");
        amf_content.push_str("    <metadata type=\"name\">Default Material</metadata>\n");
        amf_content.push_str("    <color>\n");
        amf_content.push_str(&format!("      <r>{:.3}</r>\n", color.0));
        amf_content.push_str(&format!("      <g>{:.3}</g>\n", color.1));
        amf_content.push_str(&format!("      <b>{:.3}</b>\n", color.2));
        amf_content.push_str("      <a>1.0</a>\n"); // Alpha (opacity)
        amf_content.push_str("    </color>\n");
        amf_content.push_str("  </material>\n");

        let mut vertices = Vec::new();
        let mut triangles = Vec::new();

        // Process 2D geometry
        for geom in &self.geometry.0 {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    self.add_2d_polygon_to_amf(poly2d, &mut vertices, &mut triangles);
                },
                geo::Geometry::MultiPolygon(mp) => {
                    for poly2d in &mp.0 {
                        self.add_2d_polygon_to_amf(poly2d, &mut vertices, &mut triangles);
                    }
                },
                _ => {},
            }
        }

        // Start object definition
        amf_content.push_str(&format!("  <object id=\"{}\">\n", object_name));
        amf_content.push_str("    <mesh>\n");

        // Write vertices
        amf_content.push_str("      <vertices>\n");
        for (i, vertex) in vertices.iter().enumerate() {
            amf_content.push_str(&format!("        <vertex id=\"{}\">\n", i));
            amf_content.push_str("          <coordinates>\n");
            amf_content.push_str(&format!("            <x>{:.6}</x>\n", vertex.x));
            amf_content.push_str(&format!("            <y>{:.6}</y>\n", vertex.y));
            amf_content.push_str(&format!("            <z>{:.6}</z>\n", vertex.z));
            amf_content.push_str("          </coordinates>\n");
            amf_content.push_str("        </vertex>\n");
        }
        amf_content.push_str("      </vertices>\n");

        // Write triangles with material reference
        amf_content.push_str("      <volume materialid=\"material1\">\n");
        for (i, triangle) in triangles.iter().enumerate() {
            amf_content.push_str(&format!("        <triangle id=\"{}\">\n", i));
            amf_content.push_str(&format!("          <v1>{}</v1>\n", triangle[0]));
            amf_content.push_str(&format!("          <v2>{}</v2>\n", triangle[1]));
            amf_content.push_str(&format!("          <v3>{}</v3>\n", triangle[2]));
            amf_content.push_str("        </triangle>\n");
        }
        amf_content.push_str("      </volume>\n");

        // Close mesh and object
        amf_content.push_str("    </mesh>\n");
        amf_content.push_str("  </object>\n");

        // Close AMF
        amf_content.push_str("</amf>\n");

        amf_content
    }

    // Helper function to add 2D polygon to AMF data
    fn add_2d_polygon_to_amf(
        &self,
        poly2d: &geo::Polygon<Real>,
        vertices: &mut Vec<Point3<Real>>,
        triangles: &mut Vec<Vec<usize>>,
    ) {
        // Get the exterior ring
        let exterior: Vec<[Real; 2]> =
            poly2d.exterior().coords_iter().map(|c| [c.x, c.y]).collect();

        // Get holes
        let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
            .interiors()
            .iter()
            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
            .collect();

        let hole_refs: Vec<&[[Real; 2]]> = holes_vec.iter().map(|h| &h[..]).collect();

        // Tessellate the 2D polygon
        let triangles_2d = Self::triangulate_2d(&exterior, &hole_refs);

        for triangle in triangles_2d {
            let mut triangle_indices = Vec::new();

            for point in triangle {
                let vertex_3d = Point3::new(point.x, point.y, point.z);
                let vertex_idx = add_unique_vertex_amf(vertices, vertex_3d);
                triangle_indices.push(vertex_idx);
            }

            if triangle_indices.len() == 3 {
                triangles.push(triangle_indices);
            }
        }
    }

    /// Export this Mesh to an AMF file
    ///
    /// # Arguments
    /// * `writer` - Where to write the AMF data
    /// * `object_name` - Name for the object in the AMF file
    /// * `units` - Units for the geometry (e.g., "millimeter", "inch")
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use std::fs::File;
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let csg: Mesh<()> = Mesh::cube(10.0, None);
    /// let mut file = File::create("stl/output.amf")?;
    /// csg.write_amf(&mut file, "my_cube", "millimeter")?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn write_amf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
        units: &str,
    ) -> std::io::Result<()> {
        let amf_content = self.to_amf(object_name, units);
        writer.write_all(amf_content.as_bytes())
    }
}

// Helper function to add unique vertex and return its index for AMF
fn add_unique_vertex_amf(vertices: &mut Vec<Point3<Real>>, vertex: Point3<Real>) -> usize {
    const EPSILON: Real = 1e-6;

    // Check if vertex already exists (within tolerance)
    for (i, existing) in vertices.iter().enumerate() {
        if (existing.coords - vertex.coords).norm() < EPSILON {
            return i;
        }
    }

    // Add new vertex
    vertices.push(vertex);
    vertices.len() - 1
}
