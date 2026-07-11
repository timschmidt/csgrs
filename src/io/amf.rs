#![doc = " AMF file format support for Mesh and Profile objects"]
#![doc = ""]
#![doc = " This module provides export functionality for AMF (Additive Manufacturing File Format),"]
#![doc = " an XML-based format specifically designed for 3D printing and additive manufacturing."]

use crate::triangulated::IndexedTriangulated3D;
use hyperlattice::{Point3, Real};
use std::fmt::Debug;
use std::io::Write;

fn real_f64(value: &Real) -> f64 {
    value
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .unwrap_or(0.0)
}

fn build_amf_buffers<T: IndexedTriangulated3D>(shape: &T) -> (Vec<Point3>, Vec<[usize; 3]>) {
    let indexed = shape.indexed_triangles();
    let triangles = indexed
        .faces
        .into_iter()
        .map(|face| face.map(|(position, _normal)| position))
        .collect();
    (indexed.positions, triangles)
}

#[doc = " Export any `Triangulated3D` shape to AMF format as a string"]
#[doc = ""]
#[doc = " Creates an AMF (Additive Manufacturing File Format) file containing:"]
#[doc = " 1. All triangles visited via `Triangulated3D::visit_triangles`"]
#[doc = ""]
#[doc = " # Arguments"]
#[doc = " * `object_name` - Name for the object in the AMF file"]
#[doc = " * `units` - Units for the geometry (e.g., \"millimeter\", \"inch\")"]
pub fn to_amf<T: IndexedTriangulated3D>(shape: &T, object_name: &str, units: &str) -> String {
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
        amf_content.push_str(&format!("            <x>{:.6}</x>\n", real_f64(&vertex.x)));
        amf_content.push_str(&format!("            <y>{:.6}</y>\n", real_f64(&vertex.y)));
        amf_content.push_str(&format!("            <z>{:.6}</z>\n", real_f64(&vertex.z)));
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
pub fn to_amf_with_color<T: IndexedTriangulated3D>(
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
    amf_content.push_str(&format!("      <r>{:.3}</r>\n", real_f64(&color.0)));
    amf_content.push_str(&format!("      <g>{:.3}</g>\n", real_f64(&color.1)));
    amf_content.push_str(&format!("      <b>{:.3}</b>\n", real_f64(&color.2)));
    amf_content.push_str("      <a>1.0</a>\n");
    amf_content.push_str("    </color>\n");
    amf_content.push_str("  </material>\n");

    amf_content.push_str(&format!("  <object id=\"{object_name}\">\n"));
    amf_content.push_str("    <mesh>\n");
    amf_content.push_str("      <vertices>\n");
    for (i, vertex) in vertices.iter().enumerate() {
        amf_content.push_str(&format!("        <vertex id=\"{i}\">\n"));
        amf_content.push_str("          <coordinates>\n");
        amf_content.push_str(&format!("            <x>{:.6}</x>\n", real_f64(&vertex.x)));
        amf_content.push_str(&format!("            <y>{:.6}</y>\n", real_f64(&vertex.y)));
        amf_content.push_str(&format!("            <z>{:.6}</z>\n", real_f64(&vertex.z)));
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
pub fn write_amf<T: IndexedTriangulated3D, W: Write>(
    shape: &T,
    writer: &mut W,
    object_name: &str,
    units: &str,
) -> std::io::Result<()> {
    let amf_content = to_amf(shape, object_name, units);
    writer.write_all(amf_content.as_bytes())
}

impl<M: Clone + Debug + Send + Sync> crate::mesh::Mesh<M> {
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

#[cfg(feature = "sketch")]
impl<M: Clone + Debug + Send + Sync> crate::sketch::Profile<M> {
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
