//! AMF export for native Hypermesh triangles.

use crate::io::{IoError, decimal_f64, finite_f64, xml_metadata};
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
            "<material id=\"1\"><color><r>{}</r><g>{}</g><b>{}</b><a>1</a></color></material>\n",
            decimal_f64(red),
            decimal_f64(green),
            decimal_f64(blue),
        ));
    }
    output.push_str("<object id=\"0\"><mesh><vertices>\n");
    for point in mesh.positions.iter() {
        output.push_str(&format!(
            "<vertex><coordinates><x>{}</x><y>{}</y><z>{}</z>\
             </coordinates></vertex>\n",
            decimal_f64(finite_f64(&point.x, "AMF", "vertex x")?),
            decimal_f64(finite_f64(&point.y, "AMF", "vertex y")?),
            decimal_f64(finite_f64(&point.z, "AMF", "vertex z")?),
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

/// Export native triangle geometry as AMF.
///
/// Coordinates are projected to finite binary64 values and each is spelled
/// exactly in decimal, with the same formatter as OBJ, PLY and ASCII STL: the
/// existing 17 fractional digits when they are exact, and up to 1074 where a
/// value needs them. The file therefore preserves the selected projection, not
/// arbitrary exact source coordinates. `units` must be an AMF unit name; it
/// labels the coordinates and does not scale them. Topology validation and
/// fidelity to the source are separate claims.
pub fn to_amf(mesh: &TriangleMesh, object_name: &str, units: &str) -> Result<String, IoError> {
    serialize(mesh, object_name, units, None)
}

/// [`to_amf`] with one material whose color components lie in `0..=1`.
///
/// Color components are projected and spelled like coordinates.
pub fn to_amf_with_color(
    mesh: &TriangleMesh,
    object_name: &str,
    units: &str,
    color: (Real, Real, Real),
) -> Result<String, IoError> {
    serialize(mesh, object_name, units, Some(color))
}

/// Write [`to_amf`] output to a stream after the complete text is formed.
pub fn write_amf<W: Write>(
    mesh: &TriangleMesh,
    writer: &mut W,
    object_name: &str,
    units: &str,
) -> Result<(), IoError> {
    writer.write_all(to_amf(mesh, object_name, units)?.as_bytes())?;
    Ok(())
}
