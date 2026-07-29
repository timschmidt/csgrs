//! AMF export for native Hypermesh triangles.

use crate::io::{IoError, finite_f64, xml_metadata};
use hyperlattice::Real;
use hypermesh::TriangleMesh;
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
    values
        .iter()
        .all(|value| (0.0..=1.0).contains(value))
        .then_some(values)
        .ok_or_else(|| IoError::MalformedInput("AMF colors must be in 0..=1".into()))
}

fn serialize(
    mesh: &TriangleMesh,
    object_name: &str,
    units: &str,
    color: Option<(Real, Real, Real)>,
) -> Result<String, IoError> {
    let units = validate_units(units)?;
    let object_name = xml_metadata(object_name, "AMF", "object description")?;
    let color = color.as_ref().map(finite_color).transpose()?;
    let mut output = format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
         <amf unit=\"{units}\" version=\"1.1\">\n\
         <metadata type=\"producer\">csgrs</metadata>\n\
         <metadata type=\"description\">{object_name}</metadata>\n"
    );
    if let Some([red, green, blue]) = color {
        output.push_str(&format!(
            "<material id=\"1\"><color><r>{red:.17}</r><g>{green:.17}</g>\
             <b>{blue:.17}</b><a>1</a></color></material>\n"
        ));
    }
    output.push_str("<object id=\"0\"><mesh><vertices>\n");
    for point in mesh.positions.iter() {
        output.push_str(&format!(
            "<vertex><coordinates><x>{:.17}</x><y>{:.17}</y><z>{:.17}</z>\
             </coordinates></vertex>\n",
            finite_f64(&point.x, "AMF", "vertex x")?,
            finite_f64(&point.y, "AMF", "vertex y")?,
            finite_f64(&point.z, "AMF", "vertex z")?,
        ));
    }
    output.push_str(if color.is_some() {
        "</vertices><volume materialid=\"1\">\n"
    } else {
        "</vertices><volume>\n"
    });
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices();
        if [a, b, c]
            .into_iter()
            .any(|index| index >= mesh.positions.len())
        {
            return Err(IoError::Geometry {
                format: "AMF",
                detail: "triangle references a missing position".into(),
            });
        }
        output.push_str(&format!(
            "<triangle><v1>{a}</v1><v2>{b}</v2><v3>{c}</v3></triangle>\n"
        ));
    }
    output.push_str("</volume></mesh></object></amf>\n");
    Ok(output)
}

pub fn to_amf(mesh: &TriangleMesh, object_name: &str, units: &str) -> Result<String, IoError> {
    serialize(mesh, object_name, units, None)
}

pub fn to_amf_with_color(
    mesh: &TriangleMesh,
    object_name: &str,
    units: &str,
    color: (Real, Real, Real),
) -> Result<String, IoError> {
    serialize(mesh, object_name, units, Some(color))
}

pub fn write_amf<W: Write>(
    mesh: &TriangleMesh,
    writer: &mut W,
    object_name: &str,
    units: &str,
) -> Result<(), IoError> {
    writer.write_all(to_amf(mesh, object_name, units)?.as_bytes())?;
    Ok(())
}
