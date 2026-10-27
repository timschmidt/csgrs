//! Retained source and parameter definitions, in dependency order.
//!
//! References share replayed objects; neither an address nor a stored success
//! flag establishes mathematical identity. Every definition is checked before
//! any path is published. Native geometry continues to use the version 3 codec.

use super::{ExactReader, ExactWriter, encode_bytes, extension_overflow, malformed};
use crate::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, Curve2, CurveContext, CurveGeometry2,
    CurveOperation2, CurveParameterRange2, CurvePath2, CurveResult, ExactCurveError, SvgError,
    SvgResult,
};
use std::collections::HashMap;

pub(super) const PREFIX: &str = "4:";
const MAGIC: &[u8; 4] = b"HCP4";
const NATIVE_CURVE: u8 = 0;
const SCALAR_PARAMETER: u8 = 1;
const SELECTED_ROOT: u8 = 2;
const PARAMETER_IMAGE: u8 = 3;
const SOURCE_RESTRICTION: u8 = 4;

pub(super) fn encode_path(path: &CurvePath2, max_bytes: usize) -> SvgResult<String> {
    if path.curves().len() > max_bytes / 4 {
        return Err(extension_overflow());
    }
    let mut graph = GraphWriter {
        writer: ExactWriter::new(max_bytes),
        count: 0,
        sources: HashMap::new(),
        parameters: HashMap::new(),
    };
    graph.writer.bytes.extend_from_slice(MAGIC);
    graph.writer.write_len(0)?;
    let references = path
        .curves()
        .iter()
        .map(|curve| graph.curve(curve))
        .collect::<SvgResult<Vec<_>>>()?;
    graph.writer.bytes[4..8].copy_from_slice(&graph.count.to_be_bytes());
    graph.writer.write_len(references.len())?;
    for reference in references {
        graph.writer.write_u32(reference)?;
    }
    Ok(encode_bytes(PREFIX, &graph.writer.bytes))
}

struct GraphWriter {
    writer: ExactWriter,
    count: u32,
    // These pointers memoize live immutable storage, never leave the encoder,
    // and do not merge independently authored definitions by geometric fit.
    sources: HashMap<*const CurveGeometry2, u32>,
    parameters: HashMap<*const BezierParameterPolynomial, u32>,
}

impl GraphWriter {
    fn begin(&mut self, tag: u8) -> SvgResult<u32> {
        let index = self.count;
        self.count = self.count.checked_add(1).ok_or_else(extension_overflow)?;
        self.writer.write_u8(tag)?;
        Ok(index)
    }

    fn curve(&mut self, curve: &Curve2) -> SvgResult<u32> {
        if let Some((source, range, reversed)) = curve.source_restriction() {
            let Some((start, end)) = range.as_bezier_parameters() else {
                return Err(SvgError::Unsupported(
                    "exact SVG source restrictions require a retained Bezier parameter definition"
                        .into(),
                ));
            };
            let source = self.curve(source)?;
            let start = self.parameter(start)?;
            let end = self.parameter(end)?;
            let index = self.begin(SOURCE_RESTRICTION)?;
            self.writer.write_u32(source)?;
            self.writer.write_u32(start)?;
            self.writer.write_u32(end)?;
            self.writer.write_bool(reversed)?;
            return Ok(index);
        }
        let Some(geometry) = curve.geometry() else {
            return Err(SvgError::Unsupported(
                "exact SVG metadata does not encode this generated curve definition".into(),
            ));
        };
        let key = std::ptr::from_ref(geometry);
        if let Some(index) = self.sources.get(&key) {
            return Ok(*index);
        }
        let index = self.begin(NATIVE_CURVE)?;
        self.writer.write_curve(curve)?;
        self.sources.insert(key, index);
        Ok(index)
    }

    fn parameter(&mut self, parameter: &BezierParameter2) -> SvgResult<u32> {
        let BezierParameter2::Algebraic(root) = parameter else {
            let BezierParameter2::Exact(value) = parameter else {
                unreachable!()
            };
            let index = self.begin(SCALAR_PARAMETER)?;
            self.writer.write_real(value)?;
            return Ok(index);
        };
        let key = std::ptr::from_ref(root.polynomial());
        if let Some(index) = self.parameters.get(&key) {
            return Ok(*index);
        }
        let index = if let Some((source, numerator, denominator)) = root.source_chart() {
            let source = self.parameter(&BezierParameter2::Algebraic(source.clone()))?;
            let index = self.begin(PARAMETER_IMAGE)?;
            self.writer.write_u32(source)?;
            for value in numerator.iter().chain(denominator) {
                self.writer.write_real(value)?;
            }
            index
        } else {
            let index = self.begin(SELECTED_ROOT)?;
            self.writer.write_reals(root.polynomial().coefficients())?;
            self.writer.write_real(root.interval().start())?;
            self.writer.write_real(root.interval().end())?;
            index
        };
        self.parameters.insert(key, index);
        Ok(index)
    }
}

enum Definition {
    Curve(Curve2),
    Parameter(BezierParameter2),
}

fn curve_reference(definitions: &[Definition], index: usize) -> SvgResult<&Curve2> {
    match definitions.get(index) {
        Some(Definition::Curve(curve)) => Ok(curve),
        _ => Err(malformed(
            "exact path extension has an invalid source reference",
        )),
    }
}

fn parameter_reference(definitions: &[Definition], index: usize) -> SvgResult<&BezierParameter2> {
    match definitions.get(index) {
        Some(Definition::Parameter(parameter)) => Ok(parameter),
        _ => Err(malformed(
            "exact path extension has an invalid parameter reference",
        )),
    }
}

fn certified<T>(value: CurveResult<Classification<T>>) -> SvgResult<T> {
    let value = value.map_err(|cause| {
        SvgError::Exact(ExactCurveError::invalid_unattributed(
            CurveOperation2::Construction,
            cause,
        ))
    })?;
    ExactCurveError::decided(CurveOperation2::Construction, value).map_err(SvgError::Exact)
}

pub(super) fn decode_path(bytes: &[u8], max_bytes: usize) -> SvgResult<CurvePath2> {
    let mut reader = ExactReader::new(bytes, max_bytes);
    if reader.take(MAGIC.len())? != MAGIC {
        return Err(malformed(
            "exact path extension has an invalid graph header",
        ));
    }
    let count = reader.read_len()?;
    reader.check_collection_len(count, 1)?;
    let mut definitions = Vec::new();
    for _ in 0..count {
        let definition = match reader.read_u8()? {
            NATIVE_CURVE => Definition::Curve(reader.read_curve()?),
            SCALAR_PARAMETER => Definition::Parameter(BezierParameter2::Exact(reader.read_real()?)),
            SELECTED_ROOT => {
                let coefficients = reader.read_reals()?;
                let polynomial =
                    certified(BezierParameterPolynomial::try_new_power_basis_with_policy(
                        coefficients,
                        &CurveContext::STRICT,
                    ))?;
                let lower = reader.read_real()?;
                let upper = reader.read_real()?;
                let interval = certified(BezierParameterInterval::try_new_with_policy(
                    lower,
                    upper,
                    &CurveContext::STRICT,
                ))?;
                let root = certified(BezierAlgebraicParameter2::try_isolate_with_policy(
                    polynomial,
                    interval,
                    &CurveContext::STRICT,
                ))?;
                Definition::Parameter(BezierParameter2::Algebraic(root))
            }
            PARAMETER_IMAGE => {
                let source = parameter_reference(&definitions, reader.read_len()?)?;
                let numerator = [reader.read_real()?, reader.read_real()?];
                let denominator = [reader.read_real()?, reader.read_real()?];
                Definition::Parameter(certified(source.projective_image_unbounded(
                    &numerator,
                    &denominator,
                    &CurveContext::STRICT,
                ))?)
            }
            SOURCE_RESTRICTION => {
                let source = curve_reference(&definitions, reader.read_len()?)?;
                if source.geometry().is_none() {
                    return Err(malformed(
                        "exact path restriction must reference a native source",
                    ));
                }
                let start = parameter_reference(&definitions, reader.read_len()?)?.clone();
                let end = parameter_reference(&definitions, reader.read_len()?)?.clone();
                let reversed = reader.read_bool()?;
                let range = certified(CurveParameterRange2::try_new_with_policy(
                    start.into(),
                    end.into(),
                    &CurveContext::STRICT,
                ))?;
                let curve = source.restrict_source(range).map_err(SvgError::Exact)?;
                Definition::Curve(if reversed {
                    curve.reversed().map_err(SvgError::Exact)?
                } else {
                    curve
                })
            }
            tag => {
                return Err(SvgError::Unsupported(format!(
                    "unknown exact Hypercurve definition tag {tag}"
                )));
            }
        };
        definitions.push(definition);
    }
    let count = reader.read_len()?;
    reader.check_collection_len(count, 4)?;
    let curves = (0..count)
        .map(|_| Ok(curve_reference(&definitions, reader.read_len()?)?.clone()))
        .collect::<SvgResult<Vec<_>>>()?;
    if !reader.is_empty() {
        return Err(malformed("exact path extension has trailing graph bytes"));
    }
    CurvePath2::try_new_raw(curves, &CurveContext::STRICT).map_err(SvgError::Exact)
}
