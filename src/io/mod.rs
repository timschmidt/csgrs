//! Optional import and export modules for mesh and curve file formats.
//!
//! Adapters define their own represented input/output models. Finite projection,
//! mesh validity, predicate certainty and source provenance are separate claims;
//! a successful parse or serialization does not establish exact native replay.
//! Shared helpers provide boundary conversion and diagnostics, not a second
//! geometry kernel.

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

#[cfg(feature = "vrml-io")]
pub mod vrml;

#[cfg(feature = "gerber-io")]
pub mod gerber;

#[cfg(any(feature = "stl-io", feature = "dxf-io"))]
use hyperlattice::Point3;
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
    /// Underlying stream/read/write failure; a writer may already have accepted bytes.
    #[error(transparent)]
    StdIo(
        /// Original standard I/O cause.
        #[from]
        std::io::Error,
    ),
    /// A textual floating-point field could not be parsed; finite range checks are separate.
    #[error("could not parse a finite number: {0}")]
    ParseFloat(
        /// Original numeric-parser cause.
        #[from]
        std::num::ParseFloatError,
    ),

    /// Format syntax or a parser-side conversion was rejected with textual context.
    #[error("input is malformed: {0}")]
    MalformedInput(
        /// Diagnostic text, possibly containing a line number; no typed nested source.
        String,
    ),
    /// A selected finite coordinate view is unavailable or overflows its target type.
    #[error("{format} cannot represent {field} as a finite {target}")]
    UnrepresentableCoordinate {
        /// Adapter format label.
        format: &'static str,
        /// Coordinate, normal or color component being projected.
        field: &'static str,
        /// Requested finite scalar type, such as f32 or f64.
        target: &'static str,
    },
    /// Metadata violates the adapter's character rules, including format delimiters where checked.
    #[error("{format} metadata field {field} contains forbidden control characters")]
    InvalidMetadata {
        /// Adapter format label.
        format: &'static str,
        /// Rejected metadata field's descriptive name.
        field: &'static str,
    },
    /// A checked format size/count limit cannot represent the requested output.
    #[error("{format} output exceeds the supported {limit} limit")]
    SizeOverflow {
        /// Adapter format label.
        format: &'static str,
        /// Name of the exceeded encoding limit, not a universal work budget.
        limit: &'static str,
    },
    /// Well-identified input or an option lies outside the adapter's implemented subset.
    #[error("unsupported {format} input: {detail}")]
    Unsupported {
        /// Adapter format label.
        format: &'static str,
        /// Unsupported construct or option, retained as text.
        detail: String,
    },
    /// Geometry conversion failed, with a textual rather than typed nested cause.
    /// May describe invalid geometry or an indeterminate predicate; inspect the
    /// adapter's contract rather than treating every instance as proved invalid input.
    #[error("{format} geometry conversion failed: {detail}")]
    Geometry {
        /// Adapter format label.
        format: &'static str,
        /// Conversion diagnostic; unlike `Construction`, this has no error source.
        detail: String,
    },
    /// A native modeling operation or validation failed at a format boundary.
    /// Inspect `source.kind()` for the category and its native cause for context.
    /// Includes graphics construction during PLY/glTF export and vertex-reference
    /// validation during OBJ export; no serialized geometry is returned on failure.
    #[error("{format} construction failed: {source}")]
    Construction {
        /// Adapter at which the native failure was encountered.
        format: &'static str,
        /// Typed modeling error retaining its native category and nested source.
        #[source]
        source: crate::errors::ValidationError,
    },

    /// Native DXF parser/writer failure.
    #[cfg(feature = "dxf-io")]
    #[error("DXF processing failed: {0}")]
    Dxf(
        /// Original DXF library cause.
        #[from]
        ::dxf::DxfError,
    ),

    #[cfg(feature = "gerber-io")]
    /// Error during Gerber file processing.
    #[error("Gerber parsing failed: {0}")]
    GerberParsing(
        /// Parser diagnostic retained as text, without a typed nested source.
        String,
    ),

    #[cfg(feature = "gerber-io")]
    /// Error during Gerber code generation.
    #[error("Gerber generation failed: {0}")]
    GerberCodegen(
        /// Original Gerber generation cause.
        #[from]
        ::gerber_types::GerberError,
    ),
}

/// Converts one interchange face into triangles, using native planar earcut when needed.
///
/// Rejects fewer than three entries or out-of-range indices. Three-entry faces
/// pass through without a geometric degeneracy check. Larger faces search
/// first-vertex/consecutive pairs for a nondegenerate support, stopping on any
/// native error. Planarity visits every point even after a nonzero result; an
/// undecided later point still fails. A decided nonplanar face uses a first-vertex
/// fan rather than rejecting or fitting the polygon to a plane.
///
/// Planar faces use a Newell normal and cyclic coordinate projection. Strict
/// dominant-axis ordering is a preference; if unresolved, the shared projection
/// helper chooses a component classified nonzero under the supplied policy.
/// Matching nonzero projected turns select a fan that retains collinear boundary
/// vertices and may include zero-area triangles. This local turn test does not
/// prove an arbitrary ring simple. Remaining faces use native earcut with no
/// holes, restoring negative input winding when mapping indices back.
///
/// The caller owns whole-mesh validity and resource bounds. Native mesh and
/// triangulation certainty is accumulated in `decisions`, while failures here
/// become string-valued `IoError::Geometry`, not retained native error sources.
#[cfg(any(feature = "obj-io", feature = "dxf-io", feature = "vrml-io"))]
pub(crate) fn triangulate_planar_face(
    positions: &[hyperlattice::Point3],
    face: &[usize],
    format: &'static str,
    decisions: &crate::context::GeometryDecisions,
) -> Result<Vec<hypermesh::Triangle>, IoError> {
    if face.len() < 3 || face.iter().any(|&index| index >= positions.len()) {
        return Err(IoError::Geometry {
            format,
            detail: "face has too few vertices or an invalid position index".into(),
        });
    }
    if face.len() == 3 {
        return Ok(vec![hypermesh::Triangle::new(face[0], face[1], face[2])]);
    }
    let points = face
        .iter()
        .map(|&index| &positions[index])
        .collect::<Vec<_>>();
    let mut support = None;
    for index in 1..points.len() - 1 {
        if hypermesh::Plane::points_are_nondegenerate(
            decisions.mesh_context(),
            points[0],
            points[index],
            points[index + 1],
        )
        .map_err(|error| IoError::Geometry {
            format,
            detail: error.to_string(),
        })
        .map(|outcome| decisions.consume_mesh(outcome))?
        {
            support = Some(index);
            break;
        }
    }
    let support = support.ok_or_else(|| IoError::Geometry {
        format,
        detail: "face is degenerate".into(),
    })?;
    let plane = hypermesh::Plane::from_points(points[0], points[support], points[support + 1]);
    let planar = points.iter().try_fold(true, |planar, point| {
        match decisions.probe(hyperlimit::classify_real_sign(
            &plane.expression_at_point(point),
            decisions.predicate_policy(),
        )) {
            Some(hyperlimit::Sign::Zero) => Ok(planar),
            Some(hyperlimit::Sign::Negative | hyperlimit::Sign::Positive) => Ok(false),
            None => Err(IoError::Geometry {
                format,
                detail: "face planarity is indeterminate".into(),
            }),
        }
    })?;
    if !planar {
        return Ok((1..face.len() - 1)
            .map(|index| hypermesh::Triangle::new(face[0], face[index], face[index + 1]))
            .collect());
    }
    let normal = points.iter().zip(points.iter().cycle().skip(1)).fold(
        [
            hyperlattice::Real::zero(),
            hyperlattice::Real::zero(),
            hyperlattice::Real::zero(),
        ],
        |mut sum, (a, b)| {
            sum[0] += (&a.y - &b.y) * (&a.z + &b.z);
            sum[1] += (&a.z - &b.z) * (&a.x + &b.x);
            sum[2] += (&a.x - &b.x) * (&a.y + &b.y);
            sum
        },
    );
    // Any proved nonzero normal component gives an invertible projection.
    // The dominant component is kept as a preference, so faces whose strict
    // comparisons decide it keep their triangulation, but magnitudes too close
    // to order neither block the projection nor consume an approximate
    // terminal decision.
    let dominant = (1..normal.len()).try_fold(0, |axis, candidate| {
        match hyperlimit::compare_reals(
            &normal[candidate].clone().abs(),
            &normal[axis].clone().abs(),
            hyperlimit::PredicatePolicy::STRICT,
        ) {
            hyperlimit::PredicateOutcome::Decided {
                value: std::cmp::Ordering::Greater,
                ..
            } => Some(candidate),
            hyperlimit::PredicateOutcome::Decided { .. } => Some(axis),
            hyperlimit::PredicateOutcome::Unknown { .. } => None,
        }
    });
    let axis = match dominant {
        Some(axis) => axis,
        None => {
            crate::solid::certified_projection_axis(
                &normal,
                "face projection normal",
                decisions,
            )
            .map_err(|error| IoError::Geometry {
                format,
                detail: format!("face projection normal is indeterminate: {error}"),
            })?
            .ok_or_else(|| IoError::Geometry {
                format,
                detail: "face has a zero projection normal".into(),
            })?
            .0
        },
    };
    let projected = points
        .iter()
        .map(|point| match axis {
            0 => hypertri::ExactPoint::new(point.y.clone(), point.z.clone()),
            1 => hypertri::ExactPoint::new(point.z.clone(), point.x.clone()),
            _ => hypertri::ExactPoint::new(point.x.clone(), point.y.clone()),
        })
        .collect::<Vec<_>>();
    let mut winding = None;
    let mut weakly_convex = true;
    for index in 0..projected.len() {
        let [a, b, c] = [
            &projected[index],
            &projected[(index + 1) % projected.len()],
            &projected[(index + 2) % projected.len()],
        ];
        let determinant = (&b.x - &a.x) * (&c.y - &a.y) - (&b.y - &a.y) * (&c.x - &a.x);
        let turn = decisions
            .probe(hyperlimit::classify_real_sign(
                &determinant,
                decisions.predicate_policy(),
            ))
            .ok_or_else(|| IoError::Geometry {
                format,
                detail: "face orientation is indeterminate".into(),
            })?;
        match winding {
            None if turn != hyperlimit::Sign::Zero => winding = Some(turn),
            Some(expected) if turn != hyperlimit::Sign::Zero && turn != expected => {
                weakly_convex = false;
            },
            _ => {},
        }
    }
    // Preserve collinear boundary vertices in a weakly convex interchange
    // face. Earcut may legally discard them, but doing so can crack an
    // otherwise closed polygon mesh along a neighboring face.
    if weakly_convex && winding.is_some() {
        return Ok((1..face.len() - 1)
            .map(|index| hypermesh::Triangle::new(face[0], face[index], face[index + 1]))
            .collect());
    }
    let signed_area = projected.iter().zip(projected.iter().cycle().skip(1)).fold(
        hyperlattice::Real::zero(),
        |area, (current, next)| {
            area + current.x.clone() * next.y.clone() - next.x.clone() * current.y.clone()
        },
    );
    let reverse_output = match decisions.probe(hyperlimit::classify_real_sign(
        &signed_area,
        decisions.predicate_policy(),
    )) {
        Some(hyperlimit::Sign::Negative) => true,
        Some(hyperlimit::Sign::Positive) => false,
        Some(hyperlimit::Sign::Zero) => {
            return Err(IoError::Geometry {
                format,
                detail: "projected face has zero signed area".into(),
            });
        },
        None => {
            return Err(IoError::Geometry {
                format,
                detail: "projected face winding is indeterminate".into(),
            });
        },
    };
    hypertri::earcut(&decisions.triangulation_context(), &projected, &[])
        .map_err(|error| IoError::Geometry {
            format,
            detail: format!("face triangulation failed: {error}"),
        })
        .map(|outcome| {
            let indices = decisions.consume_triangulation(outcome);
            indices
                .as_chunks::<3>()
                .0
                .iter()
                .map(|row| {
                    let [a, mut b, mut c] = [face[row[0]], face[row[1]], face[row[2]]];
                    if reverse_output {
                        std::mem::swap(&mut b, &mut c);
                    }
                    hypermesh::Triangle::new(a, b, c)
                })
                .collect()
        })
}

/// Selects the scalar's lossy binary64 view and rejects unavailable/nonfinite results.
/// Underflow to finite zero is allowed; success is not an exact-value equality,
/// topology, error-bound or provenance certificate. No source scalar is mutated.
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

/// Exact decimal spelling of an already finite binary64 boundary value.
///
/// Retain the legacy 17 fractional digits whenever sufficient. A dyadic
/// significand times 2^e needs at most -e fractional decimal digits after its
/// trailing binary zeros are removed; binary64 bounds this precision at 1074.
/// The caller must already have checked finiteness. This formats the selected
/// binary64 value exactly, not the original arbitrary-precision scalar; it does
/// not choose the shortest decimal spelling or perform projection validation.
#[cfg(any(
    feature = "obj-io",
    feature = "ply-io",
    feature = "stl-io",
    feature = "amf-io"
))]
pub(crate) fn decimal_f64(value: f64) -> impl std::fmt::Display {
    /// Local fixed-decimal display wrapper for the selected finite value.
    struct Decimal(
        /// Caller-validated binary64 scalar whose exact decimal value is emitted.
        f64,
    );

    impl std::fmt::Display for Decimal {
        /// Derives required decimal places from the exponent and trailing significand zeros.
        /// Uses at least 17 places and preserves standard formatting failure propagation.
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            let bits = self.0.to_bits();
            let exponent = ((bits >> 52) & 0x7ff) as i32;
            let significand =
                (bits & ((1_u64 << 52) - 1)) | if exponent == 0 { 0 } else { 1_u64 << 52 };
            let fractional_bits = if significand == 0 {
                0
            } else {
                1075 - exponent.max(1) - significand.trailing_zeros() as i32
            };
            let precision = fractional_bits.max(17) as usize;
            write!(f, "{:.precision$}", self.0)
        }
    }

    Decimal(value)
}

/// Computes a unit normal from binary64-projected points using finite cross-product arithmetic.
/// Coordinates are projected in point/X/Y/Z order. A zero or nonfinite computed
/// length is a geometry error, including arithmetic underflow/overflow; this does
/// not establish degeneracy of the original exact triangle or a certified normal.
#[cfg(any(feature = "stl-io", feature = "dxf-io"))]
pub(crate) fn finite_triangle_normal(
    points: [&Point3; 3],
    format: &'static str,
) -> Result<[f64; 3], IoError> {
    let project = |point: &Point3| {
        Ok::<_, IoError>([
            finite_f64(&point.x, format, "vertex x")?,
            finite_f64(&point.y, format, "vertex y")?,
            finite_f64(&point.z, format, "vertex z")?,
        ])
    };
    let points = [
        project(points[0])?,
        project(points[1])?,
        project(points[2])?,
    ];
    let left = [
        points[1][0] - points[0][0],
        points[1][1] - points[0][1],
        points[1][2] - points[0][2],
    ];
    let right = [
        points[2][0] - points[0][0],
        points[2][1] - points[0][1],
        points[2][2] - points[0][2],
    ];
    let cross = [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ];
    let length = cross.iter().map(|value| value * value).sum::<f64>().sqrt();
    if !length.is_finite() || length == 0.0 {
        return Err(IoError::Geometry {
            format,
            detail: "degenerate triangle after finite format projection".into(),
        });
    }
    Ok(cross.map(|value| value / length))
}

/// Rounds the shared lossy binary64 view to binary32 and rejects unavailable/nonfinite output.
/// This is a two-stage conversion, not a direct correctly-rounded exact-to-f32
/// guarantee. Finite underflow to zero is admitted without a fidelity receipt.
#[cfg(any(feature = "stl-io", feature = "gltf-io"))]
pub(crate) fn finite_f32(
    value: &Real,
    format: &'static str,
    field: &'static str,
) -> Result<f32, IoError> {
    // Reuse hyperreal's cached f64 boundary view, which is shared by render,
    // bounds, and checksum paths. This intentionally casts that view rather
    // than requesting separate f32 refinement; neither choice supplies a
    // topology certificate at this explicitly lossy IO boundary.
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

/// Returns metadata unchanged only when it contains no Unicode control character.
/// Empty strings and ordinary Unicode are allowed; format-specific delimiters
/// still require caller checks. No escaping or line normalization is performed.
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

/// Rejects forbidden XML scalar characters and escapes the five markup delimiters.
/// Permits tab, newline and carriage return; rejects other characters below
/// U+0020 and U+FFFE/U+FFFF. Escapes ampersands first so newly inserted entities
/// are not escaped again. This is text encoding, not XML document validation.
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

/// Finite-boundary rejection witness shared by interchange exporters.
#[cfg(all(test, feature = "stl-io"))]
mod tests {
    use super::{finite_f32, finite_f64};
    use hyperlattice::Real;

    /// An exact 10^1000 scalar fails both finite target conversions explicitly.
    #[test]
    fn finite_boundary_helpers_reject_unrepresentable_exact_values() {
        let huge = format!("1{}", "0".repeat(1000)).parse::<Real>().unwrap();
        assert!(finite_f64(&huge, "test", "coordinate").is_err());
        assert!(finite_f32(&huge, "test", "coordinate").is_err());
    }
}
