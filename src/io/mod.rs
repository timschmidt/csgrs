//! Optional import and export modules for mesh and sketch file formats.

#[cfg(feature = "svg-io")]
pub mod svg;

#[cfg(feature = "stl-io")]
pub mod stl;

#[cfg(feature = "dxf-io")]
pub mod dxf;

#[cfg(feature = "obj-io")]
pub mod obj;

#[cfg(feature = "ply-io")]
pub mod ply;

#[cfg(feature = "amf-io")]
pub mod amf;

#[cfg(feature = "gltf-io")]
pub mod gltf;

#[cfg(feature = "gerber-io")]
pub mod gerber;

#[cfg(any(
    feature = "obj-io",
    feature = "ply-io",
    feature = "amf-io",
    feature = "stl-io",
    feature = "gltf-io",
    feature = "dxf-io"
))]
use hyperlattice::Real;

/// Error produced while parsing or serializing a geometry interchange format.
#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error(transparent)]
    StdIo(#[from] std::io::Error),
    #[error("could not parse a finite number: {0}")]
    ParseFloat(#[from] std::num::ParseFloatError),

    #[error("input is malformed: {0}")]
    MalformedInput(String),
    #[error("{format} cannot represent {field} as a finite {target}")]
    UnrepresentableCoordinate {
        format: &'static str,
        field: &'static str,
        target: &'static str,
    },
    #[error("{format} metadata field {field} contains forbidden control characters")]
    InvalidMetadata {
        format: &'static str,
        field: &'static str,
    },
    #[error("{format} output exceeds the supported {limit} limit")]
    SizeOverflow {
        format: &'static str,
        limit: &'static str,
    },
    #[error("unsupported {format} input: {detail}")]
    Unsupported {
        format: &'static str,
        detail: String,
    },
    #[error("{format} geometry conversion failed: {detail}")]
    Geometry {
        format: &'static str,
        detail: String,
    },

    #[cfg(feature = "dxf-io")]
    #[error("DXF processing failed: {0}")]
    Dxf(#[from] ::dxf::DxfError),

    #[cfg(feature = "gerber-io")]
    /// Error during Gerber file processing.
    #[error("Gerber parsing failed: {0}")]
    GerberParsing(String),

    #[cfg(feature = "gerber-io")]
    /// Error during Gerber code generation.
    #[error("Gerber generation failed: {0}")]
    GerberCodegen(#[from] ::gerber_types::GerberError),
}

#[cfg(any(
    feature = "obj-io",
    feature = "ply-io",
    feature = "amf-io",
    feature = "stl-io",
    feature = "dxf-io"
))]
pub(crate) fn finite_f64(
    value: &Real,
    format: &'static str,
    field: &'static str,
) -> Result<f64, IoError> {
    value.to_f64_lossy().filter(|value| value.is_finite()).ok_or(
        IoError::UnrepresentableCoordinate {
            format,
            field,
            target: "f64",
        },
    )
}

#[cfg(any(feature = "stl-io", feature = "gltf-io"))]
pub(crate) fn finite_f32(
    value: &Real,
    format: &'static str,
    field: &'static str,
) -> Result<f32, IoError> {
    // Reuse hyperreal's cached f64 boundary view, which is shared by render,
    // bounds, and checksum paths. Computing a separate f32 refinement for the
    // same exact scalar is substantially more expensive and provides no extra
    // topology information at this explicitly lossy IO boundary.
    value
        .to_f64_lossy()
        .map(|value| value as f32)
        .filter(|value| value.is_finite())
        .ok_or(IoError::UnrepresentableCoordinate {
            format,
            field,
            target: "f32",
        })
}

#[cfg(any(feature = "obj-io", feature = "ply-io", feature = "stl-io"))]
pub(crate) fn single_line_metadata<'a>(
    value: &'a str,
    format: &'static str,
    field: &'static str,
) -> Result<&'a str, IoError> {
    if value.chars().any(char::is_control) {
        return Err(IoError::InvalidMetadata { format, field });
    }
    Ok(value)
}

#[cfg(feature = "amf-io")]
pub(crate) fn xml_metadata(
    value: &str,
    format: &'static str,
    field: &'static str,
) -> Result<String, IoError> {
    if value.chars().any(|character| {
        !matches!(character, '\u{9}' | '\u{a}' | '\u{d}')
            && (character < '\u{20}' || matches!(character, '\u{fffe}' | '\u{ffff}'))
    }) {
        return Err(IoError::InvalidMetadata { format, field });
    }
    Ok(value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;"))
}

#[cfg(test)]
pub(crate) mod test_support {
    use crate::triangulated::{IndexedTriangleMesh3D, IndexedTriangulated3D, Triangulated3D};
    use crate::vertex::Vertex;
    use hyperlattice::{Point3, Real, Vector3};

    pub(crate) struct InvalidIndexed;

    impl Triangulated3D for InvalidIndexed {
        fn visit_triangles<F>(&self, _visit: F)
        where
            F: FnMut([Vertex; 3]),
        {
        }
    }

    impl IndexedTriangulated3D for InvalidIndexed {
        fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
            IndexedTriangleMesh3D {
                positions: vec![Point3::new(Real::zero(), Real::zero(), Real::zero())],
                normals: vec![Vector3::z()],
                faces: vec![[(1, 0), (0, 0), (0, 0)]],
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{finite_f32, finite_f64};
    use hyperlattice::Real;

    #[test]
    fn finite_boundary_helpers_reject_unrepresentable_exact_values() {
        let huge = format!("1{}", "0".repeat(1000)).parse::<Real>().unwrap();
        assert!(finite_f64(&huge, "test", "coordinate").is_err());
        assert!(finite_f32(&huge, "test", "coordinate").is_err());
    }
}
