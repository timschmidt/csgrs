//! Native fidelity interchange and scalar-free admission of its graph envelope.
use super::*;

/// Syntactic size of a native parallel-fidelity graph, before scalar decoding.
/// Counts include every definition and reference, including unused definitions.
/// These are admission units, not arithmetic-work or heap bounds.
#[derive(Clone, Copy, Debug, Default)]
pub struct NativeParallelFidelityShape2 {
    /// Definitions and explicit backwards graph-reference occurrences.
    pub records: usize,
    /// Largest polynomial/spline degree in a native support.
    pub degree: usize,
    /// Variable controls, weights and knots; fixed analytic coordinates excluded.
    pub geometry_elements: usize,
    /// Longest backwards dependency chain; leaves have depth zero.
    pub dependency_depth: usize,
}

impl Curve2 {
    /// Encodes a native curve and its fidelity through the existing exact graph.
    /// This avoids display projection and is restricted to native carriers;
    /// selected outer restrictions retain their ordinary native interchange.
    pub fn to_native_parallel_fidelity_definition(&self) -> SvgResult<String> {
        if self.geometry().is_none() || self.parallel_fidelity().is_none() {
            return Err(malformed(
                "native parallel fidelity requires an annotated native curve",
            ));
        }
        let options = crate::SvgOptions::default();
        let path = CurvePath2::try_new(vec![self.clone()]).map_err(SvgError::Exact)?;
        let encoded = graph::encode_path(
            &path,
            options.max_extension_bytes,
            options.max_point_definition_depth,
        )?;
        Self::inspect_native_parallel_fidelity_definition(&encoded, options.max_extension_bytes)?;
        Ok(encoded)
    }

    /// Replays the construction with strict verification and caller limits.
    /// Neither admission counts nor serialized flags certify the construction.
    pub fn from_native_parallel_fidelity_definition(
        encoded: &str,
        options: crate::SvgOptions,
    ) -> SvgResult<Self> {
        let options = options.validate()?;
        Self::inspect_native_parallel_fidelity_definition(encoded, options.max_extension_bytes)?;
        let hex = encoded
            .strip_prefix(graph::PREFIX)
            .ok_or_else(|| malformed("native fidelity graph version"))?;
        let bytes = decode_bytes(hex, options.max_extension_bytes)?;
        let path = graph::decode_path(
            &bytes,
            options.max_extension_bytes,
            options.max_point_definition_depth,
            options.max_parallel_verification_depth,
        )?;
        let [curve] = path.curves() else {
            return Err(malformed("native fidelity requires exactly one output"));
        };
        if curve.geometry().is_none() || curve.parallel_fidelity().is_none() {
            return Err(malformed(
                "native fidelity requires a checked native output",
            ));
        }
        Ok(curve.clone())
    }

    /// Inspects only graph syntax/array sizes; no `Real` or geometry is decoded.
    /// `max_bytes` bounds decoded metadata bytes, separately from arithmetic.
    /// The accepted envelope is the native supports and fidelity edits emitted
    /// by this interchange. Other exact graph families use their own carrier.
    pub fn inspect_native_parallel_fidelity_definition(
        encoded: &str,
        max_bytes: usize,
    ) -> SvgResult<NativeParallelFidelityShape2> {
        let hex = encoded
            .strip_prefix(graph::PREFIX)
            .ok_or_else(|| malformed("native fidelity graph version"))?;
        let limit = max_bytes;
        let bytes = decode_bytes(hex, limit)?;
        let mut reader = ExactReader::new(&bytes, limit);
        if reader.take(4)? != b"HCP4" {
            return Err(malformed("native fidelity graph header"));
        }
        let count = reader.read_len()?;
        reader.check_collection_len(count, 1)?;
        let mut depths = Vec::new();
        let mut shape = NativeParallelFidelityShape2::default();
        for _ in 0..count {
            shape.records += 1;
            let depth = match reader.read_u8()? {
                0 => {
                    native(&mut reader, &mut shape)?;
                    0
                }
                31 => {
                    native(&mut reader, &mut shape)?;
                    reals(&mut reader, 3)?;
                    0
                }
                33 => {
                    let count = reader.read_len()?;
                    reader.check_collection_len(count, 4)?;
                    let mut depth = 0;
                    for _ in 0..count {
                        depth = depth.max(reference(&mut reader, &depths, &mut shape)?);
                    }
                    reals(&mut reader, 1)?;
                    reader.read_len()?;
                    reader.read_bool()?;
                    depth + 1
                }
                34 => {
                    let a = reference(&mut reader, &depths, &mut shape)?;
                    reader.read_len()?;
                    let b = reference(&mut reader, &depths, &mut shape)?;
                    reals(&mut reader, 2)?;
                    reader.read_bool()?;
                    reals(&mut reader, 1)?;
                    a.max(b) + 1
                }
                tag @ (35..=39) => {
                    let depth = reference(&mut reader, &depths, &mut shape)?;
                    match tag {
                        35 => reals(&mut reader, 2)?,
                        36 => {}
                        37 | 39 => reals(&mut reader, 6)?,
                        38 => {
                            reader.read_len()?;
                        }
                        _ => unreachable!(),
                    };
                    depth + 1
                }
                _ => {
                    return Err(SvgError::Unsupported(
                        "non-native definition in parallel-fidelity envelope".into(),
                    ));
                }
            };
            shape.dependency_depth = shape.dependency_depth.max(depth);
            depths.push(depth);
        }
        if reader.read_len()? != 1 {
            return Err(malformed("native fidelity requires one curve reference"));
        }
        reference(&mut reader, &depths, &mut shape)?;
        if !reader.is_empty() {
            return Err(malformed("trailing native fidelity bytes"));
        }
        Ok(shape)
    }
}

fn reference(
    reader: &mut ExactReader<'_>,
    depths: &[usize],
    shape: &mut NativeParallelFidelityShape2,
) -> SvgResult<usize> {
    let index = reader.read_len()?;
    shape.records += 1;
    depths
        .get(index)
        .copied()
        .ok_or_else(|| malformed("native fidelity reference is missing or forward"))
}
fn reals(reader: &mut ExactReader<'_>, count: usize) -> SvgResult<()> {
    reader.check_collection_len(count, 4)?;
    for _ in 0..count {
        let len = reader.read_len()?;
        reader.take(len)?;
    }
    Ok(())
}
fn array(
    reader: &mut ExactReader<'_>,
    width: usize,
    shape: &mut NativeParallelFidelityShape2,
) -> SvgResult<usize> {
    let count = reader.read_len()?;
    reader.check_collection_len(count, width * 4)?;
    reals(reader, count * width)?;
    shape.geometry_elements += count;
    Ok(count)
}
fn native(reader: &mut ExactReader<'_>, shape: &mut NativeParallelFidelityShape2) -> SvgResult<()> {
    match reader.read_u8()? {
        0 => reals(reader, 4)?,
        1 => {
            reals(reader, 6)?;
            reader.read_bool()?;
            if reader.read_bool()? {
                reals(reader, 1)?;
            }
        }
        2 => {
            reals(reader, 6)?;
            shape.degree = shape.degree.max(2);
            shape.geometry_elements += 3;
        }
        3 => {
            reals(reader, 8)?;
            shape.degree = shape.degree.max(3);
            shape.geometry_elements += 4;
        }
        4 => {
            reals(reader, 9)?;
            shape.degree = shape.degree.max(2);
            shape.geometry_elements += 6;
        }
        5 => {
            let n = array(reader, 3, shape)?;
            shape.degree = shape.degree.max(n.saturating_sub(1));
        }
        tag @ (6 | 7) => {
            shape.degree = shape.degree.max(reader.read_len()?);
            array(reader, if tag == 6 { 2 } else { 3 }, shape)?;
            array(reader, 1, shape)?;
            if reader.read_bool()? {
                reals(reader, 1)?;
            }
        }
        _ => return Err(malformed("unknown native support in fidelity envelope")),
    }
    Ok(())
}
