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

    fn point_parameter(&mut self, parameter: &crate::CurveParameter2) -> SvgResult<u32> {
        let parameter = parameter.as_bezier_parameter().ok_or_else(|| {
            SvgError::Unsupported("exact point replay requires a retained Bezier parameter".into())
        })?;
        self.parameter(parameter)
    }

    fn point(&mut self, point: &CurvePoint2) -> SvgResult<u32> {
        let key = point.replay_storage_key();
        if let Some((_, index)) = key.and_then(|key| self.points.get(&key)) {
            return Ok(*index);
        }
        // Keeping an endpoint reference must not bypass strict point-policy
        // validation or turn inherited qualification into an exact claim.
        let definition = point.definition().map_err(SvgError::Exact)?;
        let index = if let Some((source, start)) = point.endpoint_source() {
            let source = self.curve(&source)?;
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
                    let parameter = self.point_parameter(&parameter)?;
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
                    let parameter = self.point_parameter(&parameter)?;
                    let index = self.begin(PARALLEL_POINT)?;
                    self.writer.write_u32(source)?;
                    self.writer.write_u32(parameter)?;
                    index
                }
                _ => {
                    return Err(SvgError::Unsupported(
                        "exact SVG metadata does not encode this selected point definition".into(),
                    ));
                }
            }
        };
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
        self.chords.insert(key, (chord.clone(), index));
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
        Ok(index)
    }
}

fn point_reference(definitions: &[Definition], index: usize) -> SvgResult<&CurvePoint2> {
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

pub(super) fn read_definition(
    tag: u8,
    reader: &mut ExactReader<'_>,
    definitions: &[Definition],
) -> SvgResult<Definition> {
    let point = match tag {
        CURVE_ENDPOINT => {
            let source = curve_reference(definitions, reader.read_len()?)?;
            let start = reader.read_bool()?;
            return Ok(Definition::Point(if start {
                source.start()
            } else {
                source.end()
            }));
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
                parameter: parameter_reference(definitions, reader.read_len()?)?
                    .clone()
                    .into(),
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
                parameter: parameter_reference(definitions, reader.read_len()?)?
                    .clone()
                    .into(),
            }
        }
        CHORD_SUPPORT => {
            let start = point_reference(definitions, reader.read_len()?)?.clone();
            let end = point_reference(definitions, reader.read_len()?)?.clone();
            let axis = read_axis(reader)?;
            let increases = reader.read_bool()?;
            return certified(BezierAlgebraicChord2::try_from_replay_definition(
                start,
                end,
                axis,
                increases,
                &CurveContext::STRICT,
            ))
            .map(Definition::Chord);
        }
        CHORD_RANGE => {
            let Some(Definition::Chord(chord)) = definitions.get(reader.read_len()?) else {
                return Err(malformed("exact range has an invalid chord reference"));
            };
            let start = point_reference(definitions, reader.read_len()?)?.clone();
            let end = point_reference(definitions, reader.read_len()?)?.clone();
            let axis = read_axis(reader)?;
            let increases = reader.read_bool()?;
            return certified(chord.replay_selected_range(
                start,
                end,
                axis,
                increases,
                &CurveContext::STRICT,
            ))
            .map(|chord| Definition::Curve(chord.into()));
        }
        _ => unreachable!("caller selected a point or chord definition tag"),
    };
    point
        .into_point()
        .map(Definition::Point)
        .map_err(SvgError::Exact)
}
