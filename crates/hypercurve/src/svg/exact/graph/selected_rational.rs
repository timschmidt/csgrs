//! Finite rational source ranges with retained endpoint definitions.

use super::*;
use crate::bezier_split::{BezierSelectedFiberFragment2, BezierSelectedFiberSource2};
use crate::{CurveFamily2, RationalBezier2};
use std::cmp::Ordering;

pub(super) const RANGE: u8 = 40;

impl GraphWriter {
    pub(super) fn selected_rational_range(
        &mut self,
        fragment: &BezierSelectedFiberFragment2,
        source: &RationalBezier2,
    ) -> SvgResult<u32> {
        let source = self.point_source(source)?;
        let range = fragment.range();
        let start = self.scalar_parameter(range.start())?;
        let end = self.scalar_parameter(range.end())?;
        let (lower, upper) = if fragment.is_reversed() {
            (fragment.end_point(), fragment.start_point())
        } else {
            (fragment.start_point(), fragment.end_point())
        };
        let lower = self.point(lower)?;
        let upper = self.point(upper)?;
        let references = [source, start, end, lower, upper];
        let depth = points::next_depth(
            references
                .map(|index| self.point_depths[index as usize])
                .into_iter()
                .max()
                .unwrap(),
            self.max_point_depth,
        )?;
        let index = self.begin(RANGE)?;
        for reference in references {
            self.writer.write_u32(reference)?;
        }
        self.writer.write_bool(fragment.is_reversed())?;
        self.point_depths[index as usize] = depth;
        Ok(index)
    }
}

pub(super) fn read_range(
    reader: &mut ExactReader<'_>,
    definitions: &[Definition],
    depths: &[usize],
    max_point_depth: usize,
) -> SvgResult<(Curve2, usize)> {
    let source_index = reader.read_len()?;
    let Some(CurveGeometry2::RationalBezier(source)) =
        curve_reference(definitions, source_index)?.geometry()
    else {
        return Err(malformed(
            "exact selected rational range requires a native rational source",
        ));
    };
    let start_index = reader.read_len()?;
    let end_index = reader.read_len()?;
    let start = scalar_parameter_reference(definitions, start_index)?;
    let end = scalar_parameter_reference(definitions, end_index)?;
    let lower_index = reader.read_len()?;
    let upper_index = reader.read_len()?;
    let lower = points::point_reference(definitions, lower_index)?;
    let upper = points::point_reference(definitions, upper_index)?;
    let reversed = reader.read_bool()?;
    let depth = points::next_depth(
        [
            source_index,
            start_index,
            end_index,
            lower_index,
            upper_index,
        ]
        .map(|index| depths[index])
        .into_iter()
        .max()
        .unwrap(),
        max_point_depth,
    )?;

    let strict = &CurveContext::STRICT;
    if certified(start.cmp_by_refinement(&end, strict))? != Ordering::Less {
        return Err(malformed(
            "exact selected rational range must be ascending and nonempty",
        ));
    }
    let range = CurveParameterRange2::new_validated(start, end);
    let weight = &certified(
        source
            .homogeneous_power_basis()
            .map(Classification::Decided),
    )?
    .weight;
    if !certified(
        crate::bezier_offset::polynomial_is_nonzero_on_parameter_range(weight, &range, strict),
    )? {
        return Err(malformed(
            "exact selected rational range contains a source pole",
        ));
    }

    let parallel = BezierParallel2::from_source(
        BezierParallelSource2::Rational(source.clone()),
        Real::zero(),
    );
    for (parameter, point) in [(range.start(), lower), (range.end(), upper)] {
        let expected = if matches!(
            parameter.as_bezier_parameter(),
            Some(BezierParameter2::Algebraic(_))
        ) {
            parallel.checked_native_point(parameter, strict)
        } else {
            crate::curve::curve_evaluation::rational_point(
                source,
                parameter,
                CurveFamily2::RationalBezier,
                strict,
            )
        }
        .map_err(SvgError::Exact)?;
        if !certified(Ok(expected.same_point(point, strict)))? {
            return Err(malformed(
                "exact selected rational endpoint disagrees with its source parameter",
            ));
        }
    }

    let fragment = BezierSelectedFiberFragment2::new(
        BezierSelectedFiberSource2::Rational(source.clone()),
        range,
        lower.clone(),
        upper.clone(),
    );
    let fragment = if reversed {
        fragment.reversed()
    } else {
        fragment
    };
    Ok((fragment.into(), depth))
}
