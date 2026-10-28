//! Source-preserving selected-circle supports, contacts and finite ranges.

use super::*;
use crate::bezier_offset::{
    BezierAlgebraicCuspSemicircle2, BezierAlgebraicCuspSemicircleFragment2,
    BezierAlgebraicCuspSemicircleParameter2, CircleChordNormalDefinition2,
    CircleParameterDefinition2,
};

pub(super) const CHORD_NORMAL_CIRCLE: u8 = 20;
pub(super) const SCALAR_CIRCLE_PARAMETER: u8 = 21;
pub(super) const CHORD_NORMAL_CONTACT: u8 = 22;
pub(super) const CIRCLE_RANGE: u8 = 23;
pub(super) const INCIDENT_CIRCLE_POINT: u8 = 24;

impl GraphWriter {
    fn circle(&mut self, circle: &BezierAlgebraicCuspSemicircle2) -> SvgResult<u32> {
        let key = circle.replay_storage_key();
        if let Some((_, index)) = self.circles.get(&key) {
            return Ok(*index);
        }
        let definition = circle.native_chord_normal_definition().ok_or_else(|| {
            SvgError::Unsupported(
                "exact circle replay requires a retained chord-normal frame".into(),
            )
        })?;
        let center = self.point(&definition.center)?;
        let anchor = self.curve(&definition.anchor.into())?;
        let depth = points::next_depth(
            self.point_depths[center as usize].max(self.point_depths[anchor as usize]),
            self.max_point_depth,
        )?;
        let index = self.begin(CHORD_NORMAL_CIRCLE)?;
        self.writer.write_u32(center)?;
        self.writer.write_u32(anchor)?;
        self.writer.write_real(&definition.radial_distance)?;
        self.writer.write_bool(definition.clockwise)?;
        self.point_depths[index as usize] = depth;
        self.circles.insert(key, (circle.clone(), index));
        Ok(index)
    }

    fn circle_parameter(
        &mut self,
        circle: &BezierAlgebraicCuspSemicircle2,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
    ) -> SvgResult<u32> {
        let support = self.circle(circle)?;
        let key = parameter.replay_storage_key().map(|key| (support, key));
        if let Some((_, index)) = key.and_then(|key| self.circle_parameters.get(&key)) {
            return Ok(*index);
        }
        let definition =
            certified(circle.native_parameter_definition(parameter))?.ok_or_else(|| {
                SvgError::Unsupported(
                    "exact circle replay requires a retained angular contact definition".into(),
                )
            })?;
        let mut depth = self.point_depths[support as usize];
        let index = match definition {
            CircleParameterDefinition2::IncidentPoint(point) => {
                let point = self.point(&point)?;
                depth = depth.max(self.point_depths[point as usize]);
                points::check_depth(depth, self.max_point_depth)?;
                let index = self.begin(INCIDENT_CIRCLE_POINT)?;
                self.writer.write_u32(support)?;
                self.writer.write_u32(point)?;
                index
            }
            CircleParameterDefinition2::Exact(value) => {
                let index = self.begin(SCALAR_CIRCLE_PARAMETER)?;
                self.writer.write_u32(support)?;
                self.writer.write_real(&value)?;
                index
            }
            CircleParameterDefinition2::ChordNormal {
                chord,
                point,
                radial_distance,
            } => {
                let chord = self.curve(&chord.into())?;
                let point = self.point(&point)?;
                depth = depth
                    .max(self.point_depths[chord as usize])
                    .max(self.point_depths[point as usize]);
                points::check_depth(depth, self.max_point_depth)?;
                let index = self.begin(CHORD_NORMAL_CONTACT)?;
                for reference in [support, chord, point] {
                    self.writer.write_u32(reference)?;
                }
                self.writer.write_real(&radial_distance)?;
                index
            }
        };
        self.point_depths[index as usize] = depth;
        if let Some(key) = key {
            self.circle_parameters
                .insert(key, (parameter.clone(), index));
        }
        Ok(index)
    }

    pub(super) fn circle_fragment(
        &mut self,
        fragment: &BezierAlgebraicCuspSemicircleFragment2,
    ) -> SvgResult<u32> {
        certified(
            fragment
                .validate_policy(&CurveContext::STRICT)
                .map(Classification::Decided),
        )?;
        let circle = self.circle(fragment.semicircle())?;
        let start = self.circle_parameter(fragment.semicircle(), fragment.start_parameter())?;
        let end = self.circle_parameter(fragment.semicircle(), fragment.end_parameter())?;
        let index = self.begin(CIRCLE_RANGE)?;
        for reference in [circle, start, end] {
            self.writer.write_u32(reference)?;
        }
        self.writer.write_bool(fragment.is_reversed())?;
        self.point_depths[index as usize] = [circle, start, end]
            .into_iter()
            .map(|index| self.point_depths[index as usize])
            .max()
            .unwrap_or(0);
        Ok(index)
    }
}

fn circle_reference(
    definitions: &[Definition],
    index: usize,
) -> SvgResult<&BezierAlgebraicCuspSemicircle2> {
    match definitions.get(index) {
        Some(Definition::Circle(circle)) => Ok(circle),
        _ => Err(malformed("exact circle has an invalid support reference")),
    }
}

fn chord_reference(
    definitions: &[Definition],
    index: usize,
) -> SvgResult<&crate::BezierAlgebraicChord2> {
    match curve_reference(definitions, index)?.retained_fragment() {
        Some(crate::BezierSplitFragment2::AlgebraicChord(chord)) => Ok(chord),
        _ => Err(malformed("exact circle must reference a retained chord")),
    }
}

fn parameter_reference(
    definitions: &[Definition],
    index: usize,
    support: usize,
) -> SvgResult<&BezierAlgebraicCuspSemicircleParameter2> {
    match definitions.get(index) {
        Some(Definition::CircleParameter { circle, parameter }) if *circle as usize == support => {
            Ok(parameter)
        }
        _ => Err(malformed(
            "exact circle has an invalid angular parameter reference",
        )),
    }
}

pub(super) fn read_definition(
    tag: u8,
    reader: &mut ExactReader<'_>,
    definitions: &[Definition],
    point_depths: &[usize],
    max_point_depth: usize,
) -> SvgResult<(Definition, usize)> {
    match tag {
        CHORD_NORMAL_CIRCLE => {
            let center_index = reader.read_len()?;
            let center = points::point_reference(definitions, center_index)?.clone();
            let anchor_index = reader.read_len()?;
            let anchor = chord_reference(definitions, anchor_index)?.clone();
            let depth = points::next_depth(
                point_depths[center_index].max(point_depths[anchor_index]),
                max_point_depth,
            )?;
            let definition = CircleChordNormalDefinition2 {
                center,
                anchor,
                radial_distance: reader.read_real()?,
                clockwise: reader.read_bool()?,
            };
            certified(
                BezierAlgebraicCuspSemicircle2::replay_chord_normal_definition(
                    definition,
                    &CurveContext::STRICT,
                ),
            )
            .map(|circle| (Definition::Circle(circle), depth))
        }
        SCALAR_CIRCLE_PARAMETER | CHORD_NORMAL_CONTACT | INCIDENT_CIRCLE_POINT => {
            let support = reader.read_len()?;
            let circle = circle_reference(definitions, support)?;
            let mut depth = point_depths[support];
            let definition = if tag == INCIDENT_CIRCLE_POINT {
                let point_index = reader.read_len()?;
                let point = points::point_reference(definitions, point_index)?.clone();
                depth = depth.max(point_depths[point_index]);
                CircleParameterDefinition2::IncidentPoint(point)
            } else if tag == SCALAR_CIRCLE_PARAMETER {
                CircleParameterDefinition2::Exact(reader.read_real()?)
            } else {
                let chord_index = reader.read_len()?;
                let chord = chord_reference(definitions, chord_index)?.clone();
                let point_index = reader.read_len()?;
                let point = points::point_reference(definitions, point_index)?.clone();
                depth = depth
                    .max(point_depths[chord_index])
                    .max(point_depths[point_index]);
                CircleParameterDefinition2::ChordNormal {
                    chord,
                    point,
                    radial_distance: reader.read_real()?,
                }
            };
            points::check_depth(depth, max_point_depth)?;
            certified(circle.replay_parameter_definition(definition, &CurveContext::STRICT)).map(
                |parameter| {
                    (
                        Definition::CircleParameter {
                            circle: support as u32,
                            parameter,
                        },
                        depth,
                    )
                },
            )
        }
        CIRCLE_RANGE => {
            let support = reader.read_len()?;
            let circle = circle_reference(definitions, support)?.clone();
            let start_index = reader.read_len()?;
            let start = parameter_reference(definitions, start_index, support)?.clone();
            let end_index = reader.read_len()?;
            let end = parameter_reference(definitions, end_index, support)?.clone();
            let reversed = reader.read_bool()?;
            let depth = points::check_depth(
                point_depths[support]
                    .max(point_depths[start_index])
                    .max(point_depths[end_index]),
                max_point_depth,
            )?;
            // Range and contact authority is reconstructed. Construction-time
            // adjacency flags are not serialized as trusted proof.
            certified(BezierAlgebraicCuspSemicircleFragment2::try_new(
                circle,
                start,
                end,
                reversed,
                &CurveContext::STRICT,
            ))
            .map(|fragment| (Definition::Curve(fragment.into()), depth))
        }
        _ => unreachable!("caller selected a circle definition tag"),
    }
}
