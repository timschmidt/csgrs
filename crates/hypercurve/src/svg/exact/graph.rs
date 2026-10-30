//! Retained source and parameter definitions, in dependency order.
//!
//! References share replayed objects; neither an address nor a stored success
//! flag establishes mathematical identity. Every definition is checked before
//! any path is published. Single native paths continue to use version 3.
//! Version 5 shares the same definitions across multiple boundary paths.

use super::{ExactReader, ExactWriter, encode_bytes, extension_overflow, malformed};
use crate::{
    BezierAlgebraicParameter2, BezierParallel2, BezierParallelSource2, BezierParameter2,
    BezierParameterInterval, BezierParameterPolynomial, BezierParameterRange2, Classification,
    Curve2, CurveContext, CurveGeometry2, CurveOperation2, CurveParameterRange2, CurvePath2,
    CurveResult, ExactCurveError, Real, SvgError, SvgResult,
};
use std::collections::HashMap;

#[cfg(test)]
mod circle_certificate_tests;
mod circles;
mod points;
mod polynomial;

pub(super) const PREFIX: &str = "4:";
const MAGIC: &[u8; 4] = b"HCP4";
pub(super) const PATHS_PREFIX: &str = "5:";
const PATHS_MAGIC: &[u8; 4] = b"HCP5";
const NATIVE_CURVE: u8 = 0;
const SCALAR_PARAMETER: u8 = 1;
const SELECTED_ROOT: u8 = 2;
const PARAMETER_IMAGE: u8 = 3;
const SOURCE_RESTRICTION: u8 = 4;
const PARALLEL_SUPPORT: u8 = 5;
const PARALLEL_RANGE: u8 = 6;
const CIRCULAR_CONIC: u8 = 31;

pub(super) fn encode_path(
    path: &CurvePath2,
    max_bytes: usize,
    max_point_depth: usize,
) -> SvgResult<String> {
    if path.curves().len() > max_bytes / 4 {
        return Err(extension_overflow());
    }
    let mut graph = GraphWriter::new(max_bytes, max_point_depth);
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
    parameters: HashMap<*const BezierParameterPolynomial, (BezierAlgebraicParameter2, u32)>,
    parallel_sources: HashMap<*const BezierParallelSource2, (BezierParallel2, u32)>,
    parallels: HashMap<*const Real, (BezierParallel2, u32)>,
    point_sources: HashMap<*const crate::HomogeneousControl2, (crate::RationalBezier2, u32)>,
    points: HashMap<(u8, *const ()), (crate::CurvePoint2, u32)>,
    chords: HashMap<*const crate::CurvePoint2, (crate::BezierAlgebraicChord2, u32)>,
    retained_curves: HashMap<*const crate::BezierSplitFragment2, (Curve2, u32)>,
    displacements: HashMap<*const (), (crate::CurvePoint2, u32)>,
    point_depths: Vec<usize>,
    max_point_depth: usize,
    query_parameters:
        HashMap<(bool, *const ()), (crate::selected_query_scalar::SelectedQueryScalar2, u32)>,
    polynomial_sources: HashMap<
        *const hypersolve::bivariate_topology::BivariateCurveTopology,
        (crate::PolynomialCurveBranch2, u32),
    >,
    circles: HashMap<*const (), (crate::bezier_offset::BezierAlgebraicCuspSemicircle2, u32)>,
    circle_parameters: HashMap<
        (u32, *const ()),
        (
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
            u32,
        ),
    >,
}

impl GraphWriter {
    fn new(max_bytes: usize, max_point_depth: usize) -> Self {
        Self {
            writer: ExactWriter::new(max_bytes),
            count: 0,
            sources: HashMap::new(),
            parameters: HashMap::new(),
            parallel_sources: HashMap::new(),
            parallels: HashMap::new(),
            point_sources: HashMap::new(),
            points: HashMap::new(),
            chords: HashMap::new(),
            retained_curves: HashMap::new(),
            displacements: HashMap::new(),
            point_depths: Vec::new(),
            max_point_depth,
            query_parameters: HashMap::new(),
            circles: HashMap::new(),
            polynomial_sources: HashMap::new(),
            circle_parameters: HashMap::new(),
        }
    }

    fn begin(&mut self, tag: u8) -> SvgResult<u32> {
        let index = self.count;
        self.count = self.count.checked_add(1).ok_or_else(extension_overflow)?;
        self.writer.write_u8(tag)?;
        self.point_depths.push(0);
        Ok(index)
    }

    fn curve(&mut self, curve: &Curve2) -> SvgResult<u32> {
        let key = curve.retained_fragment().map(std::ptr::from_ref);
        if let Some((_, index)) = key.and_then(|key| self.retained_curves.get(&key)) {
            return Ok(*index);
        }
        let index = self.curve_definition(curve)?;
        if let Some(key) = key {
            self.retained_curves.insert(key, (curve.clone(), index));
        }
        Ok(index)
    }

    fn curve_definition(&mut self, curve: &Curve2) -> SvgResult<u32> {
        if let Some(branch) = curve.polynomial_branch() {
            return self.polynomial_branch(branch);
        }
        if let Some(crate::BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)) =
            curve.retained_fragment()
        {
            return self.circle_fragment(fragment);
        }
        if let Some(crate::BezierSplitFragment2::AlgebraicChord(chord)) = curve.retained_fragment()
        {
            return self.chord(chord);
        }
        if let Some(view) = curve.analytic_parallel() {
            let Some((start, end)) = view.range().as_bezier_parameters() else {
                return Err(SvgError::Unsupported(
                    "exact SVG parallels require a retained Bezier parameter definition".into(),
                ));
            };
            let parallel = self.parallel(view.parallel())?;
            let start = self.parameter(start)?;
            let end = self.parameter(end)?;
            let index = self.begin(PARALLEL_RANGE)?;
            self.writer.write_u32(parallel)?;
            self.writer.write_u32(start)?;
            self.writer.write_u32(end)?;
            self.writer.write_bool(view.is_reversed())?;
            return Ok(index);
        }
        if let Some((source, range, reversed)) = curve.source_restriction() {
            let source = self.curve(source)?;
            let start = self.scalar_parameter(range.start())?;
            let end = self.scalar_parameter(range.end())?;
            let index = self.begin(SOURCE_RESTRICTION)?;
            self.point_depths[index as usize] =
                self.point_depths[start as usize].max(self.point_depths[end as usize]);
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
        // A retained circle is construction evidence, not a cache: later
        // offsets use its exact center and radius instead of reconstructing
        // them from the controls. Replay checks it against the native curve.
        let circle = match geometry {
            CurveGeometry2::RationalQuadraticBezier(conic) => conic.retained_circular_conic(),
            _ => None,
        };
        let index = self.begin(if circle.is_some() {
            CIRCULAR_CONIC
        } else {
            NATIVE_CURVE
        })?;
        self.writer.write_curve(curve)?;
        if let Some(circle) = circle {
            self.writer.write_point(&circle.center)?;
            self.writer.write_real(&circle.radius_squared)?;
        }
        self.sources.insert(key, index);
        Ok(index)
    }

    fn parallel(&mut self, parallel: &BezierParallel2) -> SvgResult<u32> {
        // Distance belongs to the immutable parallel allocation and identifies
        // that complete support, including its source. Its address stays local.
        let key = std::ptr::from_ref(parallel.distance());
        if let Some((_, index)) = self.parallels.get(&key) {
            return Ok(*index);
        }
        let source = parallel.source();
        let source_key = std::ptr::from_ref(source);
        let source_index = if let Some((_, index)) = self.parallel_sources.get(&source_key) {
            *index
        } else {
            let native = match source {
                BezierParallelSource2::Quadratic(curve) => Curve2::from(curve.clone()),
                BezierParallelSource2::Cubic(curve) => Curve2::from(curve.clone()),
                BezierParallelSource2::Rational(curve) => Curve2::from(curve.clone()),
            };
            let index = self.begin(NATIVE_CURVE)?;
            // Never memoize the temporary native wrapper's address.
            self.writer.write_curve(&native)?;
            self.parallel_sources
                .insert(source_key, (parallel.clone(), index));
            index
        };
        let index = self.begin(PARALLEL_SUPPORT)?;
        self.writer.write_u32(source_index)?;
        self.writer.write_real(parallel.distance())?;
        self.parallels.insert(key, (parallel.clone(), index));
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
        if let Some((_, index)) = self.parameters.get(&key) {
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
        self.parameters.insert(key, (root.clone(), index));
        Ok(index)
    }
}

enum Definition {
    PolynomialSource(crate::PolynomialCurveSet2),
    Curve(Curve2),
    Parameter(BezierParameter2),
    QueryParameter(crate::CurveParameter2),
    Parallel(BezierParallel2),
    Point(crate::CurvePoint2),
    Chord(crate::BezierAlgebraicChord2),
    Displacement(crate::BezierAlgebraicChordParallelPoint2),
    Circle(crate::bezier_offset::BezierAlgebraicCuspSemicircle2),
    CircleParameter {
        circle: u32,
        parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    },
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

fn scalar_parameter_reference(
    definitions: &[Definition],
    index: usize,
) -> SvgResult<crate::CurveParameter2> {
    match definitions.get(index) {
        Some(Definition::Parameter(parameter)) => Ok(parameter.clone().into()),
        Some(Definition::QueryParameter(parameter)) => Ok(parameter.clone()),
        _ => Err(malformed(
            "exact path extension has an invalid scalar parameter reference",
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

pub(super) fn decode_path(
    bytes: &[u8],
    max_bytes: usize,
    max_point_depth: usize,
) -> SvgResult<CurvePath2> {
    let mut reader = ExactReader::new(bytes, max_bytes);
    if reader.take(MAGIC.len())? != MAGIC {
        return Err(malformed(
            "exact path extension has an invalid graph header",
        ));
    }
    let definitions = read_definitions(&mut reader, max_point_depth)?;
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

/// Decode definitions once so references across boundary paths retain shared
/// source selections. Every node is checked, including unreferenced nodes.
fn read_definitions(
    reader: &mut ExactReader<'_>,
    max_point_depth: usize,
) -> SvgResult<Vec<Definition>> {
    let count = reader.read_len()?;
    reader.check_collection_len(count, 1)?;
    let mut definitions = Vec::new();
    let mut point_depths: Vec<usize> = Vec::new();
    // Different distances on one serialized source retain one source owner.
    // Regularity belongs to each finite range, not to this unbounded recipe.
    let mut parallel_sources: HashMap<usize, BezierParallel2> = HashMap::new();
    for _ in 0..count {
        let mut point_depth = 0;
        let definition = match reader.read_u8()? {
            NATIVE_CURVE => Definition::Curve(reader.read_curve()?),
            CIRCULAR_CONIC => {
                let curve = reader.read_curve()?;
                let Some(CurveGeometry2::RationalQuadraticBezier(conic)) = curve.geometry() else {
                    return Err(malformed(
                        "exact circle certificate requires a rational quadratic",
                    ));
                };
                let center = reader.read_point()?;
                let radius_squared = reader.read_real()?;
                let conic = certified(Ok(conic.with_checked_circle_certificate(
                    center,
                    radius_squared,
                    &CurveContext::STRICT,
                )))?
                .ok_or_else(|| malformed("exact circle certificate does not hold for its curve"))?;
                Definition::Curve(Curve2::from(conic))
            }
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
                let start_index = reader.read_len()?;
                let end_index = reader.read_len()?;
                let start = scalar_parameter_reference(&definitions, start_index)?;
                let end = scalar_parameter_reference(&definitions, end_index)?;
                point_depth = point_depths[start_index].max(point_depths[end_index]);
                let reversed = reader.read_bool()?;
                let range = certified(CurveParameterRange2::try_new_with_policy(
                    start,
                    end,
                    &CurveContext::STRICT,
                ))?;
                let curve = source.restrict_source(range).map_err(SvgError::Exact)?;
                Definition::Curve(if reversed {
                    curve.reversed().map_err(SvgError::Exact)?
                } else {
                    curve
                })
            }
            PARALLEL_SUPPORT => {
                let source_index = reader.read_len()?;
                let source = curve_reference(&definitions, source_index)?;
                let distance = reader.read_real()?;
                let parallel = if let Some(source) = parallel_sources.get(&source_index) {
                    source.with_distance(distance)
                } else {
                    let source = match source.geometry() {
                        Some(CurveGeometry2::QuadraticBezier(curve)) => {
                            BezierParallelSource2::Quadratic(curve.clone())
                        }
                        Some(CurveGeometry2::CubicBezier(curve)) => {
                            BezierParallelSource2::Cubic(curve.clone())
                        }
                        Some(CurveGeometry2::RationalBezier(curve)) => {
                            BezierParallelSource2::Rational(curve.clone())
                        }
                        _ => {
                            return Err(malformed(
                                "exact parallel must reference a native Bezier source",
                            ));
                        }
                    };
                    let parallel = BezierParallel2::from_source(source, distance);
                    parallel_sources.insert(source_index, parallel.clone());
                    parallel
                };
                Definition::Parallel(parallel)
            }
            PARALLEL_RANGE => {
                let parallel = match definitions.get(reader.read_len()?) {
                    Some(Definition::Parallel(parallel)) => parallel.clone(),
                    _ => return Err(malformed("exact path has an invalid parallel reference")),
                };
                let start = parameter_reference(&definitions, reader.read_len()?)?.clone();
                let end = parameter_reference(&definitions, reader.read_len()?)?.clone();
                let reversed = reader.read_bool()?;
                if certified(start.cmp_by_refinement_with_policy(&end, &CurveContext::STRICT))?
                    != std::cmp::Ordering::Less
                {
                    return Err(SvgError::Exact(ExactCurveError::invalid_unattributed(
                        CurveOperation2::Construction,
                        crate::CurveError::InvalidBezierRange,
                    )));
                }
                let range = BezierParameterRange2::new_validated(start, end);
                let curve = certified(Curve2::try_analytic_parallel_with_policy(
                    parallel,
                    range,
                    &CurveContext::STRICT,
                ))?;
                Definition::Curve(if reversed {
                    curve.reversed().map_err(SvgError::Exact)?
                } else {
                    curve
                })
            }
            tag @ (points::COORDINATES
            | points::RATIONAL_POINT
            | points::RATIONAL_EXPRESSION
            | points::PARALLEL_POINT
            | points::CURVE_ENDPOINT
            | points::CHORD_SUPPORT
            | points::CHORD_RANGE
            | points::POLYNOMIAL_FIBER
            | points::RATIONAL_IMAGE
            | points::SIMILARITY_POINT
            | points::SUPPORTING_LINE_POINT
            | points::CHORD_DISPLACEMENT
            | points::DISPLACEMENT_POINT
            | points::CHORD_UNIT_TANGENT
            | points::SELECTED_POLYNOMIAL_ROOT
            | points::RATIONAL_QUERY_PAIR
            | points::QUERY_PARAMETER) => {
                let (definition, depth) = points::read_definition(
                    tag,
                    reader,
                    &definitions,
                    &point_depths,
                    max_point_depth,
                )?;
                point_depth = depth;
                definition
            }
            tag @ (circles::CHORD_NORMAL_CIRCLE
            | circles::SCALAR_CIRCLE_PARAMETER
            | circles::CHORD_NORMAL_CONTACT
            | circles::CIRCLE_RANGE
            | circles::INCIDENT_CIRCLE_POINT) => {
                let (definition, depth) = circles::read_definition(
                    tag,
                    reader,
                    &definitions,
                    &point_depths,
                    max_point_depth,
                )?;
                point_depth = depth;
                definition
            }
            tag @ (polynomial::POLYNOMIAL_SOURCE | polynomial::POLYNOMIAL_BRANCH) => {
                let (definition, depth) =
                    polynomial::read_definition(tag, reader, &definitions, &point_depths)?;
                point_depth = depth;
                definition
            }
            tag => {
                return Err(SvgError::Unsupported(format!(
                    "unknown exact Hypercurve definition tag {tag}"
                )));
            }
        };
        definitions.push(definition);
        point_depths.push(point_depth);
    }
    Ok(definitions)
}

/// Compound boundaries use the existing definition graph with several roots.
/// No stored region role, winding result or validity flag is trusted on replay.
pub(super) fn encode_paths(
    paths: &[CurvePath2],
    max_bytes: usize,
    max_point_depth: usize,
) -> SvgResult<String> {
    let roots = paths.iter().try_fold(paths.len(), |count, path| {
        count
            .checked_add(path.curves().len())
            .ok_or_else(extension_overflow)
    })?;
    if roots > max_bytes / 4 {
        return Err(extension_overflow());
    }
    let mut graph = GraphWriter::new(max_bytes, max_point_depth);
    graph.writer.bytes.extend_from_slice(PATHS_MAGIC);
    graph.writer.write_len(0)?;
    let references = paths
        .iter()
        .map(|path| {
            path.curves()
                .iter()
                .map(|curve| graph.curve(curve))
                .collect::<SvgResult<Vec<_>>>()
        })
        .collect::<SvgResult<Vec<_>>>()?;
    graph.writer.bytes[4..8].copy_from_slice(&graph.count.to_be_bytes());
    graph.writer.write_len(references.len())?;
    for path in references {
        graph.writer.write_len(path.len())?;
        for reference in path {
            graph.writer.write_u32(reference)?;
        }
    }
    Ok(encode_bytes(PATHS_PREFIX, &graph.writer.bytes))
}

pub(super) fn decode_paths(
    bytes: &[u8],
    max_bytes: usize,
    max_point_depth: usize,
) -> SvgResult<Vec<CurvePath2>> {
    let mut reader = ExactReader::new(bytes, max_bytes);
    if reader.take(PATHS_MAGIC.len())? != PATHS_MAGIC {
        return Err(malformed(
            "exact path extension has an invalid compound graph header",
        ));
    }
    let definitions = read_definitions(&mut reader, max_point_depth)?;
    let count = reader.read_len()?;
    reader.check_collection_len(count, 4)?;
    let paths = (0..count)
        .map(|_| {
            let count = reader.read_len()?;
            reader.check_collection_len(count, 4)?;
            let curves = (0..count)
                .map(|_| Ok(curve_reference(&definitions, reader.read_len()?)?.clone()))
                .collect::<SvgResult<Vec<_>>>()?;
            CurvePath2::try_new_raw(curves, &CurveContext::STRICT).map_err(SvgError::Exact)
        })
        .collect::<SvgResult<Vec<_>>>()?;
    if !reader.is_empty() {
        return Err(malformed(
            "exact path extension has trailing compound graph bytes",
        ));
    }
    Ok(paths)
}
