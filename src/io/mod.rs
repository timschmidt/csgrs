//! Optional import and export modules for mesh and curve file formats.

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

/// Parses an exact decimal, including scientific notation, without routing
/// through a binary floating-point approximation.
#[cfg(any(feature = "obj-io", feature = "vrml-io"))]
pub(crate) fn parse_real_decimal(
    text: &str,
) -> Result<hyperlattice::Real, hyperlattice::Problem> {
    const MAX_EXPANDED_DECIMAL_LEN: usize = 1_000_000;

    if !text.contains(['e', 'E']) {
        return text.parse();
    }
    let (mantissa, exponent) = text
        .split_once(['e', 'E'])
        .ok_or(hyperlattice::Problem::BadDecimal)?;
    let exponent = exponent
        .parse::<i64>()
        .map_err(|_| hyperlattice::Problem::BadDecimal)?;
    let (sign, magnitude) = if let Some(magnitude) = mantissa.strip_prefix('-') {
        ("-", magnitude)
    } else if let Some(magnitude) = mantissa.strip_prefix('+') {
        ("", magnitude)
    } else {
        ("", mantissa)
    };
    let (whole, fraction) = magnitude
        .split_once('.')
        .map_or((magnitude, ""), |parts| parts);
    if (whole.is_empty() && fraction.is_empty())
        || !whole.bytes().all(|byte| byte.is_ascii_digit())
        || !fraction.bytes().all(|byte| byte.is_ascii_digit())
    {
        return Err(hyperlattice::Problem::BadDecimal);
    }

    let mut digits = String::with_capacity(whole.len().saturating_add(fraction.len()));
    digits.push_str(whole);
    digits.push_str(fraction);
    let decimal_position = i64::try_from(whole.len())
        .ok()
        .and_then(|whole_len| whole_len.checked_add(exponent))
        .ok_or(hyperlattice::Problem::OutOfRange)?;
    let digits_len =
        i64::try_from(digits.len()).map_err(|_| hyperlattice::Problem::OutOfRange)?;
    let mut normalized = String::from(sign);
    if decimal_position <= 0 {
        normalized.push_str("0.");
        let zero_count = usize::try_from(
            decimal_position
                .checked_neg()
                .ok_or(hyperlattice::Problem::OutOfRange)?,
        )
        .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        if zero_count.saturating_add(digits.len()) > MAX_EXPANDED_DECIMAL_LEN {
            return Err(hyperlattice::Problem::OutOfRange);
        }
        normalized.extend(std::iter::repeat_n('0', zero_count));
        normalized.push_str(&digits);
    } else if decimal_position >= digits_len {
        normalized.push_str(&digits);
        let zero_count = usize::try_from(decimal_position - digits_len)
            .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        if zero_count.saturating_add(digits.len()) > MAX_EXPANDED_DECIMAL_LEN {
            return Err(hyperlattice::Problem::OutOfRange);
        }
        normalized.extend(std::iter::repeat_n('0', zero_count));
    } else {
        let decimal_position = usize::try_from(decimal_position)
            .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        normalized.push_str(&digits[..decimal_position]);
        normalized.push('.');
        normalized.push_str(&digits[decimal_position..]);
    }
    normalized.parse()
}

/// Triangulate one planar 3D index ring through Hypertri after dominant-axis
/// projection. This is an interchange-boundary conversion, not a second mesh
/// or polygon carrier.
#[cfg(any(feature = "obj-io", feature = "dxf-io", feature = "vrml-io"))]
pub(crate) fn triangulate_planar_face(
    positions: &[hyperlattice::Point3],
    face: &[usize],
    format: &'static str,
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
            &crate::MESH_CONTEXT,
            points[0],
            points[index],
            points[index + 1],
        )
        .map_err(|error| IoError::Geometry {
            format,
            detail: error.to_string(),
        })?
        .into_value()
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
    let planar =
        points
            .iter()
            .try_fold(true, |planar, point| {
                match hyperlimit::classify_real_sign(
                    &plane.expression_at_point(point),
                    crate::PREDICATE_POLICY,
                )
                .value()
                {
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
    let mut axis = 0;
    for candidate in 1..normal.len() {
        match hyperlimit::compare_reals(
            &normal[candidate].clone().abs(),
            &normal[axis].clone().abs(),
            crate::PREDICATE_POLICY,
        )
        .value()
        {
            Some(std::cmp::Ordering::Greater) => axis = candidate,
            Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Less) => {},
            None => {
                return Err(IoError::Geometry {
                    format,
                    detail: "dominant projection axis is indeterminate".into(),
                });
            },
        }
    }
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
        let turn = hyperlimit::classify_real_sign(&determinant, crate::PREDICATE_POLICY)
            .value()
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
    let reverse_output =
        match hyperlimit::classify_real_sign(&signed_area, crate::PREDICATE_POLICY).value() {
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
    hypertri::earcut(&crate::TRIANGULATION_CONTEXT, &projected, &[])
        .map_err(|error| IoError::Geometry {
            format,
            detail: format!("face triangulation failed: {error}"),
        })
        .map(|outcome| {
            let indices = outcome.into_value();
            indices
                .chunks_exact(3)
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

#[cfg(all(test, feature = "stl-io"))]
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
