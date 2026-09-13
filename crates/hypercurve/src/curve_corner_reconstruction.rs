//! Path publication of exact corner cuts through the shared chain machinery.

use super::*;
use crate::BezierSplitFragment2;
use crate::bezier_region::curve_corner_chain::CurveCornerChain2;

pub(super) fn corner_has_native_reconstruction(curve: &Curve2, cut: &CornerCut2) -> bool {
    curve.geometry().is_some()
        && cut.point.coordinates().is_some()
        && (cut.placement == CornerPlacement2::Corner
            || matches!(curve.geometry(), Some(CurveGeometry2::CircularArc(_)))
            || cut.exact_parameter().is_some())
}

/// The solver selects a cut in the complete authored domain before this
/// conversion. Only reconstruction uses the source's rational chart partition.
struct CornerSourceFragments2 {
    fragments: Vec<BezierSplitFragment2>,
    cut_index: usize,
    cut: CornerTrimCut2,
}

impl CornerSourceFragments2 {
    fn new(
        curve: &Curve2,
        cut: CornerCut2,
        previous: bool,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if let Some(fragment) = curve.retained_fragment() {
            return Ok(Self {
                fragments: vec![fragment.clone()],
                cut_index: 0,
                cut: cut.into_retained_evidence().ok_or_else(|| {
                    ExactCurveError::blocked(
                        operation,
                        curve.family(),
                        crate::UncertaintyReason::Unsupported,
                    )
                })?,
            });
        }

        // Native circle contacts already carry exact Cartesian incidence and
        // full-sweep placement. Keep that authority when the other side needs
        // selected reconstruction; choosing an endpoint chart first would
        // discard valid contacts on a major sweep or its extension.
        let arc_side;
        let mut cut = cut;
        let source = if matches!(curve.geometry(), Some(CurveGeometry2::CircularArc(_))) {
            arc_side = materialize_corner_cut(curve, &cut, previous, operation, policy)?;
            cut.placement = CornerPlacement2::Corner;
            cut.parameter = Some(if previous { Real::one() } else { Real::zero() }.into());
            &arc_side
        } else {
            curve
        };
        let native = source.native_bezier_fragments_for_operation(policy, operation)?;
        let cut_index = if previous {
            native.len().checked_sub(1)
        } else {
            (!native.is_empty()).then_some(0)
        }
        .ok_or_else(|| {
            ExactCurveError::invalid(
                operation,
                curve.family(),
                CurveError::Topology("a corner source has no incident chart".into()),
            )
        })?;
        let mut cut = cut.into_retained_evidence().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            )
        })?;
        if !matches!(curve.geometry(), Some(CurveGeometry2::CircularArc(_))) {
            let (start, end) = native[cut_index].parameter_range();
            if start != &Real::zero() || end != &Real::one() {
                let scale = (Real::one() / (end - start)).map_err(|cause| {
                    ExactCurveError::invalid(operation, curve.family(), cause.into())
                })?;
                cut.parameter = match cut
                    .parameter
                    .affine_image_unbounded(&scale, &(-start * &scale), policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
                {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(operation, curve.family(), reason));
                    }
                };
            }
        }
        Ok(Self {
            fragments: native
                .iter()
                .map(|fragment| BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: fragment.curve().clone(),
                })
                .collect(),
            cut_index,
            cut,
        })
    }
}

impl CurvePath2 {
    pub(super) fn reconstruct_selected_chamfer(
        &self,
        previous_index: usize,
        next_index: usize,
        solution: ChamferCorner2,
        previous_retained_arc: Option<&CircularArc2>,
        next_retained_arc: Option<&CircularArc2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Self>> {
        let operation = CurveOperation2::Chamfer;
        let previous = CornerSourceFragments2::new(
            &self.data.curves[previous_index],
            solution.previous,
            true,
            operation,
            policy,
        )?;
        let next = CornerSourceFragments2::new(
            &self.data.curves[next_index],
            solution.next,
            false,
            operation,
            policy,
        )?;
        let same_curve = previous_index == next_index;
        let next_cut_index = if same_curve {
            next.cut_index
        } else {
            previous.fragments.len() + next.cut_index
        };
        let mut fragments = previous.fragments;
        if !same_curve {
            fragments.extend(next.fragments);
        }
        let Some(rebuilt) = CurveCornerChain2::new(&fragments, same_curve).reconstruct_chamfer(
            previous.cut_index,
            next_cut_index,
            previous.cut,
            next.cut,
            previous_retained_arc,
            next_retained_arc,
            policy,
        )?
        else {
            return Ok(None);
        };
        let mut curves = Vec::with_capacity(self.data.curves.len() + rebuilt.len());
        if previous_index < next_index {
            curves.extend(self.data.curves[..previous_index].iter().cloned());
        }
        curves.extend(rebuilt.into_iter().map(Curve2::from_retained_fragment));
        if previous_index < next_index {
            curves.extend(self.data.curves[next_index + 1..].iter().cloned());
        } else if !same_curve {
            // At the closing vertex the reconstructed pair starts at the last
            // curve's retained start. Preserve the same cyclic traversal.
            curves.extend(
                self.data.curves[next_index + 1..previous_index]
                    .iter()
                    .cloned(),
            );
        }
        Self::try_new_raw(curves, policy)
            .map(Some)
            .map_err(|error| remap_operation(error, operation))
    }
}
