//! Selected point dependencies and straight support/range records.

use super::*;
use crate::{Axis2, BezierAlgebraicChord2, CurvePoint2, CurvePointDefinition2, RationalBezier2};

pub(super) const COORDINATES: u8 = 7;
pub(super) const RATIONAL_POINT: u8 = 8;
pub(super) const RATIONAL_EXPRESSION: u8 = 9;
pub(super) const PARALLEL_POINT: u8 = 10;
pub(super) const CHORD_SUPPORT: u8 = 11;
pub(super) const CHORD_RANGE: u8 = 12;
pub(super) const CURVE_ENDPOINT: u8 = 13;
pub(super) const POLYNOMIAL_FIBER: u8 = 14;
pub(super) const RATIONAL_IMAGE: u8 = 15;
pub(super) const SIMILARITY_POINT: u8 = 16;
pub(super) const SUPPORTING_LINE_POINT: u8 = 17;
pub(super) const CHORD_DISPLACEMENT: u8 = 18;
pub(super) const DISPLACEMENT_POINT: u8 = 19;
pub(super) const CHORD_UNIT_TANGENT: u8 = 25;
pub(super) const RATIONAL_QUERY_PAIR: u8 = 26;
pub(super) const QUERY_PARAMETER: u8 = 27;

fn depth_overflow() -> SvgError {
    SvgError::SizeOverflow {
        limit: "configured exact point-definition depth",
    }
}

pub(super) fn check_depth(depth: usize, limit: usize) -> SvgResult<usize> {
    if depth > limit {
        Err(depth_overflow())
    } else {
        Ok(depth)
    }
}

pub(super) fn next_depth(depth: usize, limit: usize) -> SvgResult<usize> {
    check_depth(depth.checked_add(1).ok_or_else(depth_overflow)?, limit)
}

enum PointTask {
    Visit(CurvePoint2, usize),
    Emit(CurvePoint2, CurvePointDefinition2, Option<(Curve2, bool)>),
}

impl GraphWriter {
    fn point_source(&mut self, source: &RationalBezier2) -> SvgResult<u32> {
        let key = source.homogeneous_controls().as_ptr();
        if let Some((_, index)) = self.point_sources.get(&key) {
            return Ok(*index);
        }
        let index = self.begin(NATIVE_CURVE)?;
        self.writer.write_curve(&Curve2::from(source.clone()))?;
        self.point_sources.insert(key, (source.clone(), index));
        Ok(index)
    }

    pub(super) fn scalar_parameter(
        &mut self,
        parameter: &crate::CurveParameter2,
    ) -> SvgResult<u32> {
        if let Some(parameter) = parameter.as_bezier_parameter() {
            return self.parameter(parameter);
        }
        let query = parameter.as_polynomial_query().ok_or_else(|| {
            SvgError::Unsupported(
                "exact parameter replay requires a retained source definition".into(),
            )
        })?;
        let key = query.replay_storage_key();
        if let Some((_, index)) = self.query_parameters.get(&key) {
            return Ok(*index);
        }
        let form = query.query();
        let source = self.point(&form.source)?;
        let depth = next_depth(self.point_depths[source as usize], self.max_point_depth)?;
        let index = self.begin(QUERY_PARAMETER)?;
        self.writer.write_u32(source)?;
        self.polynomial(&form.numerator)?;
        self.polynomial(&form.denominator)?;
        self.point_depths[index as usize] = depth;
        self.query_parameters.insert(key, (query.clone(), index));
        Ok(index)
    }

    pub(super) fn point(&mut self, point: &CurvePoint2) -> SvgResult<u32> {
        // Postorder traversal keeps the native dependency DAG shared without
        // consuming the Rust call stack for every nested point construction.
        let mut tasks = vec![PointTask::Visit(point.clone(), 0)];
        let mut active = std::collections::HashSet::new();
        while let Some(task) = tasks.pop() {
            match task {
                PointTask::Visit(point, ancestors) => {
                    let key = point.replay_storage_key();
                    if let Some((_, index)) = key.and_then(|key| self.points.get(&key)) {
                        check_depth(
                            ancestors
                                .checked_add(self.point_depths[*index as usize])
                                .ok_or_else(depth_overflow)?,
                            self.max_point_depth,
                        )?;
                        continue;
                    }
                    let definition = point.definition().map_err(SvgError::Exact)?;
                    let key = key.ok_or_else(|| {
                        SvgError::Unsupported(
                            "exact point replay requires an owned definition identity".into(),
                        )
                    })?;
                    if !active.insert(key) {
                        return Err(malformed("exact point definitions contain a cycle"));
                    }
                    let endpoint = point.endpoint_source();
                    let dependencies = if endpoint.is_some() {
                        Vec::new()
                    } else {
                        match &definition {
                            CurvePointDefinition2::Rational { parameter, .. }
                            | CurvePointDefinition2::Parallel { parameter, .. } => parameter
                                .as_polynomial_query()
                                .map(|query| vec![query.query().source])
                                .unwrap_or_default(),
                            CurvePointDefinition2::RationalQueryPair { sources, .. } => {
                                sources.to_vec()
                            }
                            CurvePointDefinition2::RationalImage { source, .. }
                            | CurvePointDefinition2::Similarity { source, .. } => {
                                vec![source.clone()]
                            }
                            CurvePointDefinition2::SupportingLineIntersection { first, second } => {
                                first.iter().chain(second).cloned().collect()
                            }
                            CurvePointDefinition2::ChordDisplacement { source, origin, .. } => {
                                [source.start(), source.end()]
                                    .into_iter()
                                    .chain(origin.iter().cloned())
                                    .collect()
                            }
                            _ => Vec::new(),
                        }
                    };
                    let next = if dependencies.is_empty() {
                        check_depth(ancestors, self.max_point_depth)?
                    } else {
                        next_depth(ancestors, self.max_point_depth)?
                    };
                    tasks.push(PointTask::Emit(point, definition, endpoint));
                    tasks.extend(
                        dependencies
                            .into_iter()
                            .rev()
                            .map(|point| PointTask::Visit(point, next)),
                    );
                }
                PointTask::Emit(point, definition, endpoint) => {
                    self.write_point_definition(&point, definition, endpoint)?;
                    active.remove(&point.replay_storage_key().expect("checked point identity"));
                }
            }
        }
        self.point_index(point)
    }

    fn point_index(&self, point: &CurvePoint2) -> SvgResult<u32> {
        point
            .replay_storage_key()
            .and_then(|key| self.points.get(&key))
            .map(|(_, index)| *index)
            .ok_or_else(|| malformed("exact point dependency was not emitted"))
    }

    fn polynomial(&mut self, polynomial: &hypersolve::BivariatePolynomial) -> SvgResult<()> {
        self.writer.write_len(polynomial.coefficients.len())?;
        for row in &polynomial.coefficients {
            self.writer.write_reals(row)?;
        }
        Ok(())
    }

    fn write_point_definition(
        &mut self,
        point: &CurvePoint2,
        definition: CurvePointDefinition2,
        endpoint: Option<(Curve2, bool)>,
    ) -> SvgResult<u32> {
        let key = point.replay_storage_key();
        if let Some((_, index)) = key.and_then(|key| self.points.get(&key)) {
            return Ok(*index);
        }
        // The traversal already checked the strict point definition before
        // retaining any endpoint reference.
        let mut depth = 0;
        let index = if let Some((source, start)) = endpoint {
            let source = self.curve(&source)?;
            depth = self.point_depths[source as usize];
            check_depth(depth, self.max_point_depth)?;
            let index = self.begin(CURVE_ENDPOINT)?;
            self.writer.write_u32(source)?;
            self.writer.write_bool(start)?;
            index
        } else {
            match definition {
                CurvePointDefinition2::Coordinates(point) => {
                    let index = self.begin(COORDINATES)?;
                    self.writer.write_point(&point)?;
                    index
                }
                CurvePointDefinition2::Rational { source, parameter } => {
                    let source = self.point_source(&source)?;
                    let parameter = self.scalar_parameter(&parameter)?;
                    depth = self.point_depths[parameter as usize];
                    let index = self.begin(RATIONAL_POINT)?;
                    self.writer.write_u32(source)?;
                    self.writer.write_u32(parameter)?;
                    index
                }
                CurvePointDefinition2::RationalExpression {
                    parameter,
                    x_numerator,
                    y_numerator,
                    denominator,
                } => {
                    let parameter = self.parameter(&parameter)?;
                    let index = self.begin(RATIONAL_EXPRESSION)?;
                    self.writer.write_u32(parameter)?;
                    for coefficients in [x_numerator, y_numerator, denominator] {
                        self.writer.write_reals(&coefficients)?;
                    }
                    index
                }
                CurvePointDefinition2::Parallel { source, parameter } => {
                    let source = self.parallel(&source)?;
                    let parameter = self.scalar_parameter(&parameter)?;
                    depth = self.point_depths[parameter as usize];
                    let index = self.begin(PARALLEL_POINT)?;
                    self.writer.write_u32(source)?;
                    self.writer.write_u32(parameter)?;
                    index
                }
                CurvePointDefinition2::PolynomialFiber {
                    equation,
                    parameter,
                    root,
                } => {
                    let parameter = self.parameter(&parameter)?;
                    let index = self.begin(POLYNOMIAL_FIBER)?;
                    self.writer.write_u32(parameter)?;
                    self.polynomial(&equation)?;
                    self.writer.write_real(&root.lower)?;
                    self.writer.write_real(&root.upper)?;
                    self.writer.write_bool(root.exact_root.is_some())?;
                    if let Some(exact) = root.exact_root {
                        self.writer.write_real(&exact)?;
                    }
                    index
                }
                CurvePointDefinition2::RationalQueryPair {
                    sources,
                    numerators,
                    denominators,
                } => {
                    let sources = sources
                        .each_ref()
                        .map(|source| self.point_index(source))
                        .into_iter()
                        .collect::<SvgResult<Vec<_>>>()?;
                    depth = next_depth(
                        sources
                            .iter()
                            .map(|source| self.point_depths[*source as usize])
                            .max()
                            .unwrap_or(0),
                        self.max_point_depth,
                    )?;
                    let index = self.begin(RATIONAL_QUERY_PAIR)?;
                    for ((source, numerator), denominator) in
                        sources.into_iter().zip(numerators).zip(denominators)
                    {
                        self.writer.write_u32(source)?;
                        self.polynomial(&numerator)?;
                        self.polynomial(&denominator)?;
                    }
                    index
                }
                CurvePointDefinition2::RationalImage { source, map } => {
                    let source = self.point_index(&source)?;
                    depth = next_depth(self.point_depths[source as usize], self.max_point_depth)?;
                    let index = self.begin(RATIONAL_IMAGE)?;
                    self.writer.write_u32(source)?;
                    for polynomial in map.numerators().iter().chain([map.denominator()]) {
                        self.polynomial(polynomial)?;
                    }
                    index
                }
                CurvePointDefinition2::Similarity { source, transform } => {
                    let source = self.point_index(&source)?;
                    depth = next_depth(self.point_depths[source as usize], self.max_point_depth)?;
                    let index = self.begin(SIMILARITY_POINT)?;
                    self.writer.write_u32(source)?;
                    let (a, b, d, e, x, y) = transform.affine_components();
                    for value in [a, b, d, e, x, y] {
                        self.writer.write_real(value)?;
                    }
                    index
                }
                CurvePointDefinition2::SupportingLineIntersection { first, second } => {
                    let references = first
                        .iter()
                        .chain(&second)
                        .map(|point| self.point_index(point))
                        .collect::<SvgResult<Vec<_>>>()?;
                    let source_depth = references
                        .iter()
                        .map(|index| self.point_depths[*index as usize])
                        .max()
                        .unwrap_or(0);
                    depth = next_depth(source_depth, self.max_point_depth)?;
                    let index = self.begin(SUPPORTING_LINE_POINT)?;
                    for reference in references {
                        self.writer.write_u32(reference)?;
                    }
                    index
                }
                CurvePointDefinition2::ChordDisplacement {
                    source,
                    origin,
                    at_end,
                    distance,
                    translation,
                    left_normal,
                } => {
                    let (_, key) = key.expect("checked displacement identity");
                    let support = if let Some((_, index)) = self.displacements.get(&key) {
                        *index
                    } else {
                        let source = self.curve(&source)?;
                        let origin = origin
                            .as_ref()
                            .map(|point| self.point_index(point))
                            .transpose()?;
                        let source_depth =
                            origin.map_or(self.point_depths[source as usize], |origin| {
                                self.point_depths[source as usize]
                                    .max(self.point_depths[origin as usize])
                            });
                        let depth = next_depth(source_depth, self.max_point_depth)?;
                        let index = self.begin(CHORD_DISPLACEMENT)?;
                        self.writer.write_u32(source)?;
                        self.writer.write_bool(origin.is_some())?;
                        if let Some(origin) = origin {
                            self.writer.write_u32(origin)?;
                        }
                        self.writer.write_real(&distance)?;
                        for value in translation {
                            self.writer.write_real(&value)?;
                        }
                        self.writer.write_bool(left_normal)?;
                        self.point_depths[index as usize] = depth;
                        self.displacements.insert(key, (point.clone(), index));
                        index
                    };
                    depth = self.point_depths[support as usize];
                    let index = self.begin(DISPLACEMENT_POINT)?;
                    self.writer.write_u32(support)?;
                    self.writer.write_bool(at_end)?;
                    index
                }
            }
        };
        self.point_depths[index as usize] = depth;
        if let Some(key) = key {
            self.points.insert(key, (point.clone(), index));
        }
        Ok(index)
    }

    fn chord_support(&mut self, chord: &BezierAlgebraicChord2) -> SvgResult<u32> {
        let chord = chord.retained_support();
        let key = std::ptr::from_ref(chord.start());
        if let Some((_, index)) = self.chords.get(&key) {
            return Ok(*index);
        }
        certified(
            chord
                .validate_policy(&CurveContext::STRICT)
                .map(Classification::Decided),
        )?;
        let (axis, increases) = chord.native_replay_chart().ok_or_else(|| {
            SvgError::Unsupported("exact chord replay requires retained contact definitions".into())
        })?;
        let start = self.point(chord.start())?;
        let end = self.point(chord.end())?;
        let index = self.begin(CHORD_SUPPORT)?;
        self.writer.write_u32(start)?;
        self.writer.write_u32(end)?;
        self.writer.write_u8(match axis {
            Axis2::X => 0,
            Axis2::Y => 1,
        })?;
        self.writer.write_bool(increases)?;
        self.point_depths[index as usize] =
            self.point_depths[start as usize].max(self.point_depths[end as usize]);
        let index = self.chord_unit_tangent(chord, index)?;
        self.chords.insert(key, (chord.clone(), index));
        Ok(index)
    }

    fn chord_unit_tangent(&mut self, chord: &BezierAlgebraicChord2, source: u32) -> SvgResult<u32> {
        let Some(tangent) = chord.native_replay_unit_tangent() else {
            return Ok(source);
        };
        let index = self.begin(CHORD_UNIT_TANGENT)?;
        self.writer.write_u32(source)?;
        for component in tangent {
            self.writer.write_real(&component)?;
        }
        self.point_depths[index as usize] = self.point_depths[source as usize];
        Ok(index)
    }

    pub(super) fn chord(&mut self, chord: &BezierAlgebraicChord2) -> SvgResult<u32> {
        certified(
            chord
                .validate_policy(&CurveContext::STRICT)
                .map(Classification::Decided),
        )?;
        let (axis, increases) = chord.native_replay_chart().ok_or_else(|| {
            SvgError::Unsupported("exact chord replay requires retained contact definitions".into())
        })?;
        let support = self.chord_support(chord)?;
        let start = self.point(chord.start())?;
        let end = self.point(chord.end())?;
        let index = self.begin(CHORD_RANGE)?;
        self.writer.write_u32(support)?;
        self.writer.write_u32(start)?;
        self.writer.write_u32(end)?;
        self.writer.write_u8(match axis {
            Axis2::X => 0,
            Axis2::Y => 1,
        })?;
        self.writer.write_bool(increases)?;
        self.point_depths[index as usize] = [support, start, end]
            .into_iter()
            .map(|index| self.point_depths[index as usize])
            .max()
            .unwrap_or(0);
        self.chord_unit_tangent(chord, index)
    }
}

pub(super) fn point_reference(definitions: &[Definition], index: usize) -> SvgResult<&CurvePoint2> {
    match definitions.get(index) {
        Some(Definition::Point(point)) => Ok(point),
        _ => Err(malformed("exact path has an invalid point reference")),
    }
}

fn read_axis(reader: &mut ExactReader<'_>) -> SvgResult<Axis2> {
    match reader.read_u8()? {
        0 => Ok(Axis2::X),
        1 => Ok(Axis2::Y),
        _ => Err(malformed("exact chord has an invalid coordinate axis")),
    }
}

fn read_polynomial(reader: &mut ExactReader<'_>) -> SvgResult<hypersolve::BivariatePolynomial> {
    let count = reader.read_len()?;
    reader.check_collection_len(count, 4)?;
    let coefficients = (0..count)
        .map(|_| reader.read_reals())
        .collect::<SvgResult<Vec<_>>>()?;
    Ok(hypersolve::BivariatePolynomial::new(coefficients))
}

pub(super) fn read_definition(
    tag: u8,
    reader: &mut ExactReader<'_>,
    definitions: &[Definition],
    point_depths: &[usize],
    max_point_depth: usize,
) -> SvgResult<(Definition, usize)> {
    let mut depth = 0;
    if tag == QUERY_PARAMETER {
        let index = reader.read_len()?;
        let source = point_reference(definitions, index)?;
        let numerator = read_polynomial(reader)?;
        let denominator = read_polynomial(reader)?;
        let depth = next_depth(point_depths[index], max_point_depth)?;
        // Replay the declared query authority even when an older scalar
        // specialization could evaluate it. Otherwise its source/guard recipe
        // can be lost and the imported restriction may cease to be exportable.
        let parameter = certified(crate::selected_query_scalar::SelectedQueryScalar2::replay(
            crate::bezier_offset::PointFormSource2::new(source.clone(), numerator, denominator),
            &CurveContext::STRICT,
        ))?;
        return Ok((
            Definition::QueryParameter(crate::CurveParameter2::from_polynomial_query(parameter)),
            depth,
        ));
    }
    let point = match tag {
        CHORD_UNIT_TANGENT => {
            let source = reader.read_len()?;
            let (chord, support) = match definitions.get(source) {
                Some(Definition::Chord(chord)) => (chord, true),
                Some(Definition::Curve(curve)) => {
                    let Some(crate::BezierSplitFragment2::AlgebraicChord(chord)) =
                        curve.retained_fragment()
                    else {
                        return Err(malformed("unit tangent must reference a retained chord"));
                    };
                    (chord, false)
                }
                _ => return Err(malformed("unit tangent must reference a retained chord")),
            };
            let tangent = [reader.read_real()?, reader.read_real()?];
            return certified(chord.replay_unit_tangent(tangent, &CurveContext::STRICT)).map(
                |chord| {
                    (
                        if support {
                            Definition::Chord(chord)
                        } else {
                            Definition::Curve(chord.into())
                        },
                        point_depths[source],
                    )
                },
            );
        }
        CURVE_ENDPOINT => {
            let index = reader.read_len()?;
            let source = curve_reference(definitions, index)?;
            depth = check_depth(point_depths[index], max_point_depth)?;
            let start = reader.read_bool()?;
            return Ok((
                Definition::Point(if start { source.start() } else { source.end() }),
                depth,
            ));
        }
        COORDINATES => CurvePointDefinition2::Coordinates(reader.read_point()?),
        RATIONAL_POINT => {
            let source = curve_reference(definitions, reader.read_len()?)?;
            let Some(CurveGeometry2::RationalBezier(source)) = source.geometry() else {
                return Err(malformed(
                    "exact point must reference a native rational source",
                ));
            };
            CurvePointDefinition2::Rational {
                source: source.clone(),
                parameter: {
                    let index = reader.read_len()?;
                    let parameter = scalar_parameter_reference(definitions, index)?;
                    depth = point_depths[index];
                    parameter
                },
            }
        }
        RATIONAL_EXPRESSION => CurvePointDefinition2::RationalExpression {
            parameter: parameter_reference(definitions, reader.read_len()?)?.clone(),
            x_numerator: reader.read_reals()?,
            y_numerator: reader.read_reals()?,
            denominator: reader.read_reals()?,
        },
        PARALLEL_POINT => {
            let Some(Definition::Parallel(source)) = definitions.get(reader.read_len()?) else {
                return Err(malformed("exact point has an invalid parallel reference"));
            };
            CurvePointDefinition2::Parallel {
                source: source.clone(),
                parameter: {
                    let index = reader.read_len()?;
                    let parameter = scalar_parameter_reference(definitions, index)?;
                    depth = point_depths[index];
                    parameter
                },
            }
        }
        POLYNOMIAL_FIBER => {
            let parameter = parameter_reference(definitions, reader.read_len()?)?.clone();
            let equation = read_polynomial(reader)?;
            let lower = reader.read_real()?;
            let upper = reader.read_real()?;
            let exact_root = if reader.read_bool()? {
                Some(reader.read_real()?)
            } else {
                None
            };
            CurvePointDefinition2::PolynomialFiber {
                equation,
                parameter,
                root: hypersolve::IsolatedRootInterval {
                    lower,
                    upper,
                    exact_root,
                    // This requests a singleton. The point owner proves it;
                    // the archive cannot supply a trusted root-count flag.
                    distinct_root_count: 1,
                },
            }
        }
        RATIONAL_QUERY_PAIR => {
            let mut sources = Vec::with_capacity(2);
            let mut numerators = Vec::with_capacity(2);
            let mut denominators = Vec::with_capacity(2);
            for _ in 0..2 {
                let index = reader.read_len()?;
                sources.push(point_reference(definitions, index)?.clone());
                depth = depth.max(next_depth(point_depths[index], max_point_depth)?);
                numerators.push(read_polynomial(reader)?);
                denominators.push(read_polynomial(reader)?);
            }
            CurvePointDefinition2::RationalQueryPair {
                sources: sources.try_into().expect("two sources"),
                numerators: numerators.try_into().expect("two numerators"),
                denominators: denominators.try_into().expect("two denominators"),
            }
        }
        RATIONAL_IMAGE => {
            let index = reader.read_len()?;
            let source = point_reference(definitions, index)?.clone();
            depth = next_depth(point_depths[index], max_point_depth)?;
            CurvePointDefinition2::RationalImage {
                source,
                map: crate::RationalMap2::new(
                    [read_polynomial(reader)?, read_polynomial(reader)?],
                    read_polynomial(reader)?,
                ),
            }
        }
        SIMILARITY_POINT => {
            let index = reader.read_len()?;
            let source = point_reference(definitions, index)?.clone();
            depth = next_depth(point_depths[index], max_point_depth)?;
            let transform = certified(
                crate::Similarity2::try_from_real_affine(
                    reader.read_real()?,
                    reader.read_real()?,
                    reader.read_real()?,
                    reader.read_real()?,
                    reader.read_real()?,
                    reader.read_real()?,
                )
                .map(Classification::Decided),
            )?;
            CurvePointDefinition2::Similarity { source, transform }
        }
        SUPPORTING_LINE_POINT => {
            let mut endpoint = || -> SvgResult<CurvePoint2> {
                let index = reader.read_len()?;
                let point = point_reference(definitions, index)?.clone();
                depth = depth.max(point_depths[index]);
                Ok(point)
            };
            let first = [endpoint()?, endpoint()?];
            let second = [endpoint()?, endpoint()?];
            depth = next_depth(depth, max_point_depth)?;
            CurvePointDefinition2::SupportingLineIntersection { first, second }
        }
        CHORD_DISPLACEMENT => {
            let source_index = reader.read_len()?;
            let source = curve_reference(definitions, source_index)?.clone();
            depth = point_depths[source_index];
            let origin = if reader.read_bool()? {
                let index = reader.read_len()?;
                let point = point_reference(definitions, index)?.clone();
                depth = depth.max(point_depths[index]);
                Some(point)
            } else {
                None
            };
            depth = next_depth(depth, max_point_depth)?;
            let distance = reader.read_real()?;
            let translation = [reader.read_real()?, reader.read_real()?];
            let left_normal = reader.read_bool()?;
            let point = certified(
                crate::BezierAlgebraicChordParallelPoint2::from_replay_definition(
                    source,
                    origin,
                    false,
                    distance,
                    translation,
                    left_normal,
                    &CurveContext::STRICT,
                ),
            )?;
            return Ok((Definition::Displacement(point), depth));
        }
        DISPLACEMENT_POINT => {
            let index = reader.read_len()?;
            let Some(Definition::Displacement(point)) = definitions.get(index) else {
                return Err(malformed(
                    "exact point has an invalid displacement reference",
                ));
            };
            let at_end = reader.read_bool()?;
            return Ok((
                Definition::Point(point.replay_endpoint(at_end).into()),
                check_depth(point_depths[index], max_point_depth)?,
            ));
        }
        CHORD_SUPPORT => {
            let start_index = reader.read_len()?;
            let start = point_reference(definitions, start_index)?.clone();
            let end_index = reader.read_len()?;
            let end = point_reference(definitions, end_index)?.clone();
            depth = check_depth(
                point_depths[start_index].max(point_depths[end_index]),
                max_point_depth,
            )?;
            let axis = read_axis(reader)?;
            let increases = reader.read_bool()?;
            return certified(BezierAlgebraicChord2::try_from_replay_definition(
                start,
                end,
                axis,
                increases,
                &CurveContext::STRICT,
            ))
            .map(|chord| (Definition::Chord(chord), depth));
        }
        CHORD_RANGE => {
            let support = reader.read_len()?;
            let Some(Definition::Chord(chord)) = definitions.get(support) else {
                return Err(malformed("exact range has an invalid chord reference"));
            };
            let start_index = reader.read_len()?;
            let start = point_reference(definitions, start_index)?.clone();
            let end_index = reader.read_len()?;
            let end = point_reference(definitions, end_index)?.clone();
            depth = check_depth(
                point_depths[support]
                    .max(point_depths[start_index])
                    .max(point_depths[end_index]),
                max_point_depth,
            )?;
            let axis = read_axis(reader)?;
            let increases = reader.read_bool()?;
            return certified(chord.replay_selected_range(
                start,
                end,
                axis,
                increases,
                &CurveContext::STRICT,
            ))
            .map(|chord| (Definition::Curve(chord.into()), depth));
        }
        _ => unreachable!("caller selected a point or chord definition tag"),
    };
    point
        .into_point_with_checked_sources()
        .map(|point| (Definition::Point(point), depth))
        .map_err(SvgError::Exact)
}
