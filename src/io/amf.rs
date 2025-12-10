#![doc = " AMF file format support for Mesh, Sketch, and BMesh objects"]
#![doc = ""]
#![doc = " This module provides export functionality for AMF (Additive Manufacturing File Format),"]
#![doc = " an XML-based format specifically designed for 3D printing and additive manufacturing."]

use crate::float_types::{Real, tolerance};
use crate::triangulated::Triangulated3D;
use nalgebra::Point3;
use std::fmt::Debug;
use std::io::Write;

/// Add a vertex to the list, reusing an existing one if position is within `tolerance()`.
fn add_unique_vertex_amf(vertices: &mut Vec<Point3<Real>>, vertex: Point3<Real>) -> usize {
    for (i, existing) in vertices.iter().enumerate() {
        if (existing.coords - vertex.coords).norm() < tolerance() {
            return i;
        }
    }
    vertices.push(vertex);
    vertices.len() - 1
}

fn build_amf_buffers<T: Triangulated3D>(shape: &T) -> (Vec<Point3<Real>>, Vec<[usize; 3]>) {
    let mut vertices = Vec::<Point3<Real>>::new();
    let mut triangles = Vec::<[usize; 3]>::new();

    shape.visit_triangles(|tri| {
        let i0 = add_unique_vertex_amf(&mut vertices, tri[0].position);
        let i1 = add_unique_vertex_amf(&mut vertices, tri[1].position);
        let i2 = add_unique_vertex_amf(&mut vertices, tri[2].position);
        triangles.push([i0, i1, i2]);
    });

    (vertices, triangles)
}

#[doc = " Export any `Triangulated3D` shape to AMF format as a string"]
#[doc = ""]
#[doc = " Creates an AMF (Additive Manufacturing File Format) file containing:"]
#[doc = " 1. All triangles visited via `Triangulated3D::visit_triangles`"]
#[doc = ""]
#[doc = " # Arguments"]
#[doc = " * `object_name` - Name for the object in the AMF file"]
#[doc = " * `units` - Units for the geometry (e.g., \"millimeter\", \"inch\")"]
pub fn to_amf<T: Triangulated3D>(shape: &T, object_name: &str, units: &str) -> String {
    let (vertices, triangles) = build_amf_buffers(shape);

    let mut amf_content = String::new();
    amf_content.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    amf_content.push_str("<amf unit=\"");
    amf_content.push_str(units);
    amf_content.push_str("\" version=\"1.1\">\n");
    amf_content.push_str("  <metadata type=\"producer\">csgrs library</metadata>\n");
    amf_content.push_str("  <metadata type=\"cad\">Constructive Solid Geometry</metadata>\n");
    amf_content.push_str(&format!(
        "  <metadata type=\"description\">{object_name}</metadata>\n"
    ));

    amf_content.push_str(&format!("  <object id=\"{object_name}\">\n"));
    amf_content.push_str("    <mesh>\n");
    amf_content.push_str("      <vertices>\n");
    for (i, vertex) in vertices.iter().enumerate() {
        amf_content.push_str(&format!("        <vertex id=\"{i}\">\n"));
        amf_content.push_str("          <coordinates>\n");
        amf_content.push_str(&format!("            <x>{:.6}</x>\n", vertex.x));
        amf_content.push_str(&format!("            <y>{:.6}</y>\n", vertex.y));
        amf_content.push_str(&format!("            <z>{:.6}</z>\n", vertex.z));
        amf_content.push_str("          </coordinates>\n");
        amf_content.push_str("        </vertex>\n");
    }
    amf_content.push_str("      </vertices>\n");
    amf_content.push_str("      <volume>\n");
    for (i, triangle) in triangles.iter().enumerate() {
        amf_content.push_str(&format!("        <triangle id=\"{i}\">\n"));
        amf_content.push_str(&format!("          <v1>{}</v1>\n", triangle[0]));
        amf_content.push_str(&format!("          <v2>{}</v2>\n", triangle[1]));
        amf_content.push_str(&format!("          <v3>{}</v3>\n", triangle[2]));
        amf_content.push_str("        </triangle>\n");
    }
    amf_content.push_str("      </volume>\n");
    amf_content.push_str("    </mesh>\n");
    amf_content.push_str("  </object>\n");
    amf_content.push_str("</amf>\n");
    amf_content
}

#[doc = " Export any `Triangulated3D` shape to AMF format with color information"]
#[doc = ""]
#[doc = " Creates an AMF file with color/material information for enhanced 3D printing."]
#[doc = ""]
#[doc = " # Arguments"]
#[doc = " * `object_name` - Name for the object in the AMF file"]
#[doc = " * `units` - Units for the geometry (e.g., \"millimeter\", \"inch\")"]
#[doc = " * `color` - RGB color as (red, green, blue) where each component is 0.0-1.0"]
pub fn to_amf_with_color<T: Triangulated3D>(
    shape: &T,
    object_name: &str,
    units: &str,
    color: (Real, Real, Real),
) -> String {
    let (vertices, triangles) = build_amf_buffers(shape);

    let mut amf_content = String::new();
    amf_content.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    amf_content.push_str("<amf unit=\"");
    amf_content.push_str(units);
    amf_content.push_str("\" version=\"1.1\">\n");
    amf_content.push_str("  <metadata type=\"producer\">csgrs library</metadata>\n");
    amf_content.push_str("  <metadata type=\"cad\">Constructive Solid Geometry</metadata>\n");
    amf_content.push_str(&format!(
        "  <metadata type=\"description\">{object_name}</metadata>\n"
    ));

    amf_content.push_str("  <material id=\"material1\">\n");
    amf_content.push_str("    <metadata type=\"name\">Default Material</metadata>\n");
    amf_content.push_str("    <color>\n");
    amf_content.push_str(&format!("      <r>{:.3}</r>\n", color.0));
    amf_content.push_str(&format!("      <g>{:.3}</g>\n", color.1));
    amf_content.push_str(&format!("      <b>{:.3}</b>\n", color.2));
    amf_content.push_str("      <a>1.0</a>\n");
    amf_content.push_str("    </color>\n");
    amf_content.push_str("  </material>\n");

    amf_content.push_str(&format!("  <object id=\"{object_name}\">\n"));
    amf_content.push_str("    <mesh>\n");
    amf_content.push_str("      <vertices>\n");
    for (i, vertex) in vertices.iter().enumerate() {
        amf_content.push_str(&format!("        <vertex id=\"{i}\">\n"));
        amf_content.push_str("          <coordinates>\n");
        amf_content.push_str(&format!("            <x>{:.6}</x>\n", vertex.x));
        amf_content.push_str(&format!("            <y>{:.6}</y>\n", vertex.y));
        amf_content.push_str(&format!("            <z>{:.6}</z>\n", vertex.z));
        amf_content.push_str("          </coordinates>\n");
        amf_content.push_str("        </vertex>\n");
    }
    amf_content.push_str("      </vertices>\n");
    amf_content.push_str("      <volume materialid=\"material1\">\n");
    for (i, triangle) in triangles.iter().enumerate() {
        amf_content.push_str(&format!("        <triangle id=\"{i}\">\n"));
        amf_content.push_str(&format!("          <v1>{}</v1>\n", triangle[0]));
        amf_content.push_str(&format!("          <v2>{}</v2>\n", triangle[1]));
        amf_content.push_str(&format!("          <v3>{}</v3>\n", triangle[2]));
        amf_content.push_str("        </triangle>\n");
    }
    amf_content.push_str("      </volume>\n");
    amf_content.push_str("    </mesh>\n");
    amf_content.push_str("  </object>\n");
    amf_content.push_str("</amf>\n");
    amf_content
}

#[doc = " Export any `Triangulated3D` shape to an AMF file"]
#[doc = ""]
#[doc = " # Arguments"]
#[doc = " * `writer` - Where to write the AMF data"]
#[doc = " * `object_name` - Name for the object in the AMF file"]
#[doc = " * `units` - Units for the geometry (e.g., \"millimeter\", \"inch\")"]
pub fn write_amf<T: Triangulated3D, W: Write>(
    shape: &T,
    writer: &mut W,
    object_name: &str,
    units: &str,
) -> std::io::Result<()> {
    let amf_content = to_amf(shape, object_name, units);
    writer.write_all(amf_content.as_bytes())
}

impl<S: Clone + Debug + Send + Sync> crate::mesh::Mesh<S> {
    pub fn to_amf(&self, object_name: &str, units: &str) -> String {
        self::to_amf(self, object_name, units)
    }

    pub fn write_amf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
        units: &str,
    ) -> std::io::Result<()> {
        self::write_amf(self, writer, object_name, units)
    }

    pub fn to_amf_with_color(
        &self,
        object_name: &str,
        units: &str,
        color: (Real, Real, Real),
    ) -> String {
        self::to_amf_with_color(self, object_name, units, color)
    }
}

impl<S: Clone + Debug + Send + Sync> crate::sketch::Sketch<S> {
    pub fn to_amf(&self, object_name: &str, units: &str) -> String {
        self::to_amf(self, object_name, units)
    }

    pub fn write_amf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
        units: &str,
    ) -> std::io::Result<()> {
        self::write_amf(self, writer, object_name, units)
    }

    pub fn to_amf_with_color(
        &self,
        object_name: &str,
        units: &str,
        color: (Real, Real, Real),
    ) -> String {
        self::to_amf_with_color(self, object_name, units, color)
    }
}

impl<S: Clone + Debug + Send + Sync> crate::bmesh::BMesh<S> {
    pub fn to_amf(&self, object_name: &str, units: &str) -> String {
        self::to_amf(self, object_name, units)
    }

    pub fn write_amf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
        units: &str,
    ) -> std::io::Result<()> {
        self::write_amf(self, writer, object_name, units)
    }

    pub fn to_amf_with_color(
        &self,
        object_name: &str,
        units: &str,
        color: (Real, Real, Real),
    ) -> String {
        self::to_amf_with_color(self, object_name, units, color)
    }
}
