//! AMF export for indexed triangulated geometry.

use crate::io::{IoError, finite_f64, xml_metadata};
use crate::triangulated::IndexedTriangulated3D;
use hyperlattice::Real;
use std::fmt::Debug;
use std::io::Write;

fn validate_units(units: &str) -> Result<&str, IoError> {
    match units {
        "meter" | "millimeter" | "micrometer" | "feet" | "inch" => Ok(units),
        _ => Err(IoError::Unsupported {
            format: "AMF",
            detail: format!("unknown unit {units:?}"),
        }),
    }
}

fn finite_color(color: &(Real, Real, Real)) -> Result<[f64; 3], IoError> {
    let values = [
        finite_f64(&color.0, "AMF", "color red")?,
        finite_f64(&color.1, "AMF", "color green")?,
        finite_f64(&color.2, "AMF", "color blue")?,
    ];
    if values.iter().any(|value| !(0.0..=1.0).contains(value)) {
        return Err(IoError::MalformedInput(
            "AMF colors must be in the inclusive range 0..=1".into(),
        ));
    }
    Ok(values)
}

fn serialize_amf<T: IndexedTriangulated3D>(
    shape: &T,
    object_name: &str,
    units: &str,
    color: Option<(Real, Real, Real)>,
) -> Result<String, IoError> {
    let units = validate_units(units)?;
    let object_name = xml_metadata(object_name, "AMF", "object description")?;
    let indexed = shape.indexed_triangles();
    let color = color.as_ref().map(finite_color).transpose()?;

    let mut output = String::new();
    output.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    output.push_str(&format!("<amf unit=\"{units}\" version=\"1.1\">\n"));
    output.push_str("  <metadata type=\"producer\">csgrs</metadata>\n");
    output.push_str(&format!(
        "  <metadata type=\"description\">{object_name}</metadata>\n"
    ));
    if let Some([red, green, blue]) = color {
        output.push_str("  <material id=\"1\"><color>\n");
        output.push_str(&format!(
            "    <r>{red:.17}</r><g>{green:.17}</g><b>{blue:.17}</b><a>1</a>\n"
        ));
        output.push_str("  </color></material>\n");
    }
    output.push_str("  <object id=\"0\"><mesh><vertices>\n");
    for vertex in &indexed.positions {
        output.push_str("    <vertex><coordinates>");
        output.push_str(&format!(
            "<x>{:.17}</x><y>{:.17}</y><z>{:.17}</z>",
            finite_f64(&vertex.x, "AMF", "vertex x")?,
            finite_f64(&vertex.y, "AMF", "vertex y")?,
            finite_f64(&vertex.z, "AMF", "vertex z")?,
        ));
        output.push_str("</coordinates></vertex>\n");
    }
    output.push_str("  </vertices>");
    if color.is_some() {
        output.push_str("<volume materialid=\"1\">\n");
    } else {
        output.push_str("<volume>\n");
    }
    for face in indexed.faces {
        let [first, second, third] = face.map(|(position, _)| position);
        if [first, second, third]
            .iter()
            .any(|index| *index >= indexed.positions.len())
        {
            return Err(IoError::Geometry {
                format: "AMF",
                detail: "indexed triangle references a missing position".into(),
            });
        }
        output.push_str(&format!(
            "    <triangle><v1>{first}</v1><v2>{second}</v2><v3>{third}</v3></triangle>\n"
        ));
    }
    output.push_str("  </volume></mesh></object>\n</amf>\n");
    Ok(output)
}

pub fn to_amf<T: IndexedTriangulated3D>(
    shape: &T,
    object_name: &str,
    units: &str,
) -> Result<String, IoError> {
    serialize_amf(shape, object_name, units, None)
}

pub fn to_amf_with_color<T: IndexedTriangulated3D>(
    shape: &T,
    object_name: &str,
    units: &str,
    color: (Real, Real, Real),
) -> Result<String, IoError> {
    serialize_amf(shape, object_name, units, Some(color))
}

pub fn write_amf<T: IndexedTriangulated3D, W: Write>(
    shape: &T,
    writer: &mut W,
    object_name: &str,
    units: &str,
) -> Result<(), IoError> {
    writer.write_all(to_amf(shape, object_name, units)?.as_bytes())?;
    Ok(())
}

macro_rules! impl_amf_export {
    ($type:ty) => {
        impl<M: Clone + Debug + Send + Sync> $type {
            pub fn to_amf(&self, name: &str, units: &str) -> Result<String, IoError> {
                to_amf(self, name, units)
            }

            pub fn write_amf<W: Write>(
                &self,
                writer: &mut W,
                name: &str,
                units: &str,
            ) -> Result<(), IoError> {
                write_amf(self, writer, name, units)
            }

            pub fn to_amf_with_color(
                &self,
                name: &str,
                units: &str,
                color: (Real, Real, Real),
            ) -> Result<String, IoError> {
                to_amf_with_color(self, name, units, color)
            }
        }
    };
}

impl_amf_export!(crate::mesh::Mesh<M>);
#[cfg(feature = "sketch")]
impl_amf_export!(crate::sketch::Profile<M>);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::Mesh;
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    #[test]
    fn metadata_is_xml_escaped() {
        let mesh = Mesh::<()>::cube(Real::one(), ());
        let output = mesh.to_amf("a<&\"b", "millimeter").unwrap();
        assert!(output.contains("a&lt;&amp;&quot;b"));
        assert!(mesh.to_amf("bad\0name", "millimeter").is_err());

        let mut reader = Reader::from_str(&output);
        let mut vertices = 0;
        let mut triangles = 0;
        loop {
            match reader.read_event().unwrap() {
                Event::Start(element) if element.name().as_ref() == b"vertex" => vertices += 1,
                Event::Start(element) if element.name().as_ref() == b"triangle" => {
                    triangles += 1
                },
                Event::Eof => break,
                _ => {},
            }
        }
        assert_eq!(vertices, 8);
        assert_eq!(triangles, 12);
    }

    #[test]
    fn invalid_units_and_colors_are_rejected() {
        let mesh = Mesh::<()>::cube(Real::one(), ());
        assert!(mesh.to_amf("cube", "parsec").is_err());
        assert!(
            mesh.to_amf_with_color(
                "cube",
                "millimeter",
                (Real::from(2_u8), Real::zero(), Real::zero()),
            )
            .is_err()
        );
    }
}
