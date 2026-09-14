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
        defer_arc: bool,
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

        if let Some(spans) = curve.restricted_source_spans(policy, operation)? {
            let cut_index = if previous { spans.len() - 1 } else { 0 };
            let span = &spans[cut_index];
            let mut cut = cut.into_retained_evidence().ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    curve.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
            let inverse = (Real::one() / &span.source_scale).map_err(|cause| {
                ExactCurveError::invalid(operation, curve.family(), cause.into())
            })?;
            cut.parameter = match cut
                .parameter
                .affine_image_unbounded(&inverse, &(-&span.source_offset * &inverse), policy)
                .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(operation, curve.family(), reason));
                }
            };
            return Ok(Self {
                fragments: spans.iter().map(|span| span.fragment.clone()).collect(),
                cut_index,
                cut,
            });
        }

        // Native circle contacts already carry exact Cartesian incidence and
        // full-sweep placement. Keep that authority when the other side needs
        // selected reconstruction; choosing an endpoint chart first would
        // discard valid contacts on a major sweep or its extension.
        let arc_side;
        let mut cut = cut;
        let source =
            if !defer_arc && matches!(curve.geometry(), Some(CurveGeometry2::CircularArc(_))) {
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
        self.reconstruct_corner(
            previous_index,
            next_index,
            solution.previous,
            solution.next,
            CurveOperation2::Chamfer,
            None,
            policy,
            |chain, [previous_index, next_index], [previous_cut, next_cut], _| {
                chain.reconstruct_chamfer(
                    previous_index,
                    next_index,
                    previous_cut,
                    next_cut,
                    previous_retained_arc,
                    next_retained_arc,
                    policy,
                )
            },
        )
    }

    pub(super) fn reconstruct_selected_fillet(
        &self,
        previous_index: usize,
        next_index: usize,
        solution: FilletCorner2,
        radius: &Real,
        mode: CurveCornerMode2,
        retained_arcs: [Option<&CircularArc2>; 2],
        promoted_parallels: [Option<&crate::BezierParallelFragment2>; 2],
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Self>> {
        let deferred_arc = solution
            .retained_frame
            .as_ref()
            .and_then(|frame| frame.anchor_evidence.as_ref())
            .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
            .map(|contact| contact.arc_is_previous);
        self.reconstruct_corner(
            previous_index,
            next_index,
            solution.previous,
            solution.next,
            CurveOperation2::Fillet,
            deferred_arc,
            policy,
            |chain, [previous_index, next_index], [previous_cut, next_cut], domains| {
                chain.reconstruct_fillet(
                    previous_index,
                    next_index,
                    previous_cut,
                    next_cut,
                    solution.center,
                    solution.clockwise,
                    solution.retained_frame,
                    radius,
                    mode,
                    retained_arcs,
                    promoted_parallels,
                    domains,
                    policy,
                )
            },
        )
    }

    fn reconstruct_corner(
        &self,
        previous_index: usize,
        next_index: usize,
        previous_cut: CornerCut2,
        next_cut: CornerCut2,
        operation: CurveOperation2,
        deferred_arc: Option<bool>,
        policy: &CurveContext,
        rebuild: impl FnOnce(
            &CurveCornerChain2<'_>,
            [usize; 2],
            [CornerTrimCut2; 2],
            [std::ops::Range<usize>; 2],
        ) -> ExactCurveResult<Option<Vec<BezierSplitFragment2>>>,
    ) -> ExactCurveResult<Option<Self>> {
        let previous = CornerSourceFragments2::new(
            &self.data.curves[previous_index],
            previous_cut,
            true,
            deferred_arc == Some(true),
            operation,
            policy,
        )?;
        let next = CornerSourceFragments2::new(
            &self.data.curves[next_index],
            next_cut,
            false,
            deferred_arc == Some(false),
            operation,
            policy,
        )?;
        let same_curve = previous_index == next_index;
        let next_cut_index = if same_curve {
            next.cut_index
        } else {
            previous.fragments.len() + next.cut_index
        };
        let domains = [
            0..previous.fragments.len(),
            if same_curve {
                0..previous.fragments.len()
            } else {
                previous.fragments.len()..previous.fragments.len() + next.fragments.len()
            },
        ];
        let mut fragments = previous.fragments;
        if !same_curve {
            fragments.extend(next.fragments);
        }
        let Some(rebuilt) = rebuild(
            &CurveCornerChain2::new(&fragments, same_curve),
            [previous.cut_index, next_cut_index],
            [previous.cut, next.cut],
            domains,
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
