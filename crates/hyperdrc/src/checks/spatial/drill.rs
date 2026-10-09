//! Native circular drill envelopes for conservative spacing candidates.

use crate::Scalar;
use crate::geometry::{Coord, Rect};
use crate::kicad::DrillFeature;

use super::envelope::EnvelopeSpatialIndex;

/// Each drill retains its original source index. Unknown, unrepresentable and
/// oversized circle envelopes use the shared fallback, never a zero radius.
pub(in crate::checks) struct DrillSpatialIndex {
    envelopes: EnvelopeSpatialIndex,
}

impl DrillSpatialIndex {
    pub(in crate::checks) fn new(drills: &[DrillFeature], nominal_spacing: &Scalar) -> Self {
        Self {
            envelopes: EnvelopeSpatialIndex::new(
                drills.iter().map(drill_enclosure).collect(),
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

fn drill_enclosure(drill: &DrillFeature) -> Option<Rect<f64>> {
    crate::operations::evaluate("drill-bound-enclosure", None, || {
        let radius = crate::checks::measurements::drill_radius(&drill.diameter)?;
        let [x, y] = &drill.location;
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
