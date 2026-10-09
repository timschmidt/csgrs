//! Native circle envelopes shared by drills and fixture probes.

use crate::Scalar;
use crate::geometry::{Coord, Rect};
use crate::kicad::DrillFeature;

use super::envelope::EnvelopeSpatialIndex;

/// Each circle retains its original source index. Missing or invalid diameters
/// remain unindexed candidates; callers decide their required-field semantics.
/// Unknown, unrepresentable and
/// oversized circle envelopes use the shared fallback, never a zero radius.
pub(in crate::checks) struct NativeCircleSpatialIndex {
    envelopes: EnvelopeSpatialIndex,
}

impl NativeCircleSpatialIndex {
    pub(in crate::checks) fn new<'a>(
        circles: impl IntoIterator<Item = (&'a [Scalar; 2], Option<&'a Scalar>)>,
        nominal_spacing: &Scalar,
    ) -> Self {
        Self {
            envelopes: EnvelopeSpatialIndex::new(
                circles
                    .into_iter()
                    .map(|(center, diameter)| {
                        diameter.and_then(|diameter| circle_enclosure(center, diameter))
                    })
                    .collect(),
                super::native_radius_upper(nominal_spacing),
            ),
        }
    }

    pub(in crate::checks) fn bucket_count(&self) -> usize {
        self.envelopes.bucket_count()
    }

    pub(in crate::checks) fn later_candidates_within_spacing(
        &self,
        index: usize,
        spacing: &Scalar,
    ) -> Vec<usize> {
        self.envelopes
            .near_index(index, super::native_radius_upper(spacing))
            .into_iter()
            .filter(|&other| other > index)
            .collect()
    }
}

fn circle_enclosure(center: &[Scalar; 2], diameter: &Scalar) -> Option<Rect<f64>> {
    crate::operations::evaluate("drill-bound-enclosure", None, || {
        let radius = crate::checks::measurements::drill_radius(diameter)?;
        let [x, y] = center;
        let bounds = [x - &radius, y - &radius, x + &radius, y + &radius];
        let [Some(x0), Some(y0), Some(x1), Some(y1)] =
            bounds.each_ref().map(crate::projection::finite_enclosure)
        else {
            return Ok(None);
        };
        Ok(Some(Rect::new(
            Coord { x: x0[0], y: y0[0] },
            Coord { x: x1[1], y: y1[1] },
        )))
    })
    .ok()
    .flatten()
}

/// Existing drill consumers share the same native circle-envelope scheduler.
pub(in crate::checks) struct DrillSpatialIndex(NativeCircleSpatialIndex);

impl DrillSpatialIndex {
    pub(in crate::checks) fn new(drills: &[DrillFeature], nominal_spacing: &Scalar) -> Self {
        Self(NativeCircleSpatialIndex::new(
            drills
                .iter()
                .map(|drill| (&drill.location, Some(&drill.diameter))),
            nominal_spacing,
        ))
    }
    pub(in crate::checks) fn bucket_count(&self) -> usize {
        self.0.bucket_count()
    }
    pub(in crate::checks) fn later_candidates_within_spacing(
        &self,
        index: usize,
        spacing: &Scalar,
    ) -> Vec<usize> {
        self.0.later_candidates_within_spacing(index, spacing)
    }
}
