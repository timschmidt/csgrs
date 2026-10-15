//! Lossless line-contour views of already classified material/hole profiles.

use crate::{
    Classification, Contour2, CurveOperation2, CurveRegionBoundaryLoop2, CurveRegionProfile2,
    ExactCurveError, ExactCurveResult, Segment2, UncertaintyReason,
};

impl CurveRegionProfile2<'_> {
    /// Retain exact line segments and their material/hole ownership.
    ///
    /// Returns `None` if any boundary lacks a native line certificate. This is
    /// a representation query, not permission to replace a curved boundary by
    /// display chords. No coordinate passes through a finite projection.
    pub fn linear_contours(&self) -> ExactCurveResult<Option<(Contour2, Vec<Contour2>)>> {
        fn contour(boundary: &CurveRegionBoundaryLoop2) -> ExactCurveResult<Option<Contour2>> {
            let policy = crate::policy::principal_context();
            let mut segments = Vec::with_capacity(boundary.len());
            for fragment in boundary.fragments() {
                // Reuse the native line-image and selected-endpoint authority,
                // including authored lines stored as degree-elevated Beziers.
                let line = crate::bezier_region::retained_line_fragment_segment(fragment, &policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid_unattributed(
                            CurveOperation2::NativeTopology,
                            cause,
                        )
                    })?;
                match line {
                    Classification::Decided(line) => segments.push(Segment2::Line(line)),
                    Classification::Uncertain(UncertaintyReason::Unsupported) => return Ok(None),
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked_unattributed(
                            CurveOperation2::NativeTopology,
                            reason,
                        ));
                    }
                }
            }
            Contour2::try_new(segments)
                .map(Some)
                .map_err(|cause| ExactCurveError::Invalid {
                    operation: CurveOperation2::NativeTopology,
                    family: Some(crate::CurveFamily2::Line),
                    cause,
                })
        }
        let Some(material) = contour(self.material())? else {
            return Ok(None);
        };
        let Some(holes) = self
            .holes()
            .iter()
            .map(|boundary| contour(boundary))
            .collect::<ExactCurveResult<Option<Vec<_>>>>()?
        else {
            return Ok(None);
        };
        Ok(Some((material, holes)))
    }
}
