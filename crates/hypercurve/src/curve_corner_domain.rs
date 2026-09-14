//! Corner contacts in a complete authored parameter domain.

use super::*;
use crate::CurveParameterRange2;
use std::cmp::Ordering;

fn decided<T>(value: Classification<T>, family: CurveFamily2) -> ExactCurveResult<T> {
    match value {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Chamfer,
            family,
            reason,
        )),
    }
}

fn order(
    first: &CurveParameter2,
    second: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    decided(
        first
            .cmp_by_refinement(second, policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Chamfer, family, cause))?,
        family,
    )
}

pub(super) fn fixed_distance_point(
    parallel: &BezierParallel2,
    parameter: crate::bezier_offset::BezierParallelFixedDistanceParameter2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<(CurveParameter2, CurvePoint2)> {
    use crate::bezier_offset::BezierParallelFixedDistanceParameter2;
    Ok(match parameter {
        BezierParallelFixedDistanceParameter2::Bezier(parameter) => {
            let point =
                analytic_parallel_point_evidence(parallel, &parameter, operation, family, policy)?;
            (parameter.into(), point)
        }
        BezierParallelFixedDistanceParameter2::SelectedFiber(parameter) => {
            let point = CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                parallel.clone(),
                parameter.clone(),
                policy,
            ));
            (CurveParameter2::from_selected_fiber(parameter), point)
        }
        BezierParallelFixedDistanceParameter2::RecursiveProjective(parameter) => {
            let point = CurvePoint2::from(
                crate::BezierAnalyticParallelPoint2::new_recursive_projective(
                    parallel.clone(),
                    parameter.clone(),
                    policy,
                ),
            );
            (CurveParameter2::from_recursive_projective(parameter), point)
        }
    })
}

impl Curve2 {
    /// Two cuts on one closed authored curve must leave a nonempty interval
    /// between them. Enumeration over several charts can also produce the
    /// opposite pairing, whose trims have already passed each other.
    pub(super) fn chamfer_cuts_leave_authored_interval(
        &self,
        previous: &CornerCut2,
        next: &CornerCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        if (self.source_range().is_none()
            && !matches!(
                self.geometry(),
                Some(CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_))
            ))
            || previous.placement == CornerPlacement2::Extension
            || next.placement == CornerPlacement2::Extension
        {
            return Ok(true);
        }
        let previous = previous.parameter.as_ref().expect("authored cut parameter");
        let next = next.parameter.as_ref().expect("authored cut parameter");
        let ordering = order(next, previous, self.family(), policy)?;
        Ok(if self.source_range().is_some_and(|range| range.reversed) {
            ordering.is_gt()
        } else {
            ordering.is_lt()
        })
    }

    /// A rational chart partitions enumeration, never the authored trim domain.
    /// The incident chart still owns an extension ray. Every finite chart is
    /// searched from the same exact corner location before global placement.
    pub(super) fn chamfer_cuts_in_authored_domain(
        &self,
        incident: ExactCornerCarrier2<'_>,
        incident_chart: Option<(&Real, &Real)>,
        setback: &Real,
        setback_sign: RealSign,
        previous: bool,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CornerCuts2> {
        let operation = CurveOperation2::Chamfer;
        let family = self.family();
        let incident_cuts = || {
            let mut cuts = corner_chamfer_cuts(
                incident,
                setback,
                setback_sign,
                previous,
                mode,
                false,
                operation,
                family,
                policy,
            )?;
            for cut in cuts.iter_mut() {
                cut.map_source_parameter(incident_chart, operation, family, policy)?;
            }
            Ok(cuts)
        };
        let source = match self.source_range() {
            Some(range) => &range.source,
            None if matches!(
                self.geometry(),
                Some(CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_))
            ) =>
            {
                self
            }
            _ => return incident_cuts(),
        };
        let charts = source.native_bezier_fragments_for_operation(policy, operation)?;
        if charts.len() == 1 || setback_sign == RealSign::Zero {
            return incident_cuts();
        }
        let range = self.parameter_domain();
        let corner_is_upper = previous != self.source_range().is_some_and(|range| range.reversed);
        let corner_parameter = if corner_is_upper {
            range.end()
        } else {
            range.start()
        };
        let mut center_chart = None;
        for (index, chart) in charts.iter().enumerate() {
            let (start, end) = chart.parameter_range();
            let lower = order(corner_parameter, &start.clone().into(), family, policy)?;
            let upper = order(corner_parameter, &end.clone().into(), family, policy)?;
            if lower.is_lt()
                || upper.is_gt()
                || (lower.is_eq() && corner_is_upper && index > 0)
                || (upper.is_eq() && !corner_is_upper && index + 1 < charts.len())
            {
                continue;
            }
            let inverse = (Real::one() / (end - start))
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
            let local = decided(
                corner_parameter
                    .affine_image_unbounded(&inverse, &(-start * &inverse), policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                family,
            )?;
            center_chart = Some((
                exact_corner_bezier_parallel(
                    ExactCornerBezier2::NativeSpan(chart),
                    Real::zero(),
                    operation,
                    family,
                )?,
                local,
            ));
            break;
        }
        let (center, center_parameter) = center_chart.ok_or_else(|| {
            ExactCurveError::invalid(operation, family, CurveError::InvalidCurveParameter)
        })?;
        let unit_range =
            CurveParameterRange2::new_validated(Real::zero().into(), Real::one().into());
        let mut cuts = CornerCuts2::default();
        let mut seam: Option<(CurveParameter2, CurvePoint2)> = None;
        for chart in charts {
            let previous_seam = seam.take();
            let (start, end) = chart.parameter_range();
            if !order(range.end(), &start.clone().into(), family, policy)?.is_gt()
                || !order(range.start(), &end.clone().into(), family, policy)?.is_lt()
            {
                continue;
            }
            let parallel = exact_corner_bezier_parallel(
                ExactCornerBezier2::NativeSpan(chart),
                Real::zero(),
                operation,
                family,
            )?;
            let parameters = decided(
                parallel
                    .fixed_distance_incidence(
                        &center,
                        &center_parameter,
                        setback,
                        &unit_range,
                        None,
                        policy,
                    )
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                family,
            )?;
            for parameter in parameters {
                let (parameter, point) =
                    fixed_distance_point(&parallel, parameter, operation, family, policy)?;
                let parameter = decided(
                    parameter
                        .affine_image_unbounded(&(end - start), start, policy)
                        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                    family,
                )?;
                if !order(&parameter, range.start(), family, policy)?.is_gt()
                    || !order(&parameter, range.end(), family, policy)?.is_lt()
                {
                    continue;
                }
                // Only adjacent closed cells can duplicate a source location.
                // Retain that seam witness instead of comparing every new
                // root with every previously published cut. Distinct source
                // parameters at a self-contact remain distinct solutions.
                if let Some((previous_parameter, previous_point)) = &previous_seam
                    && order(&parameter, previous_parameter, family, policy)?.is_eq()
                    && decided(point.same_point(previous_point, policy), family)?
                {
                    continue;
                }
                if order(&parameter, &end.clone().into(), family, policy)?.is_eq() {
                    seam = Some((parameter.clone(), point.clone()));
                }
                cuts.push(CornerCut2 {
                    parameter: Some(parameter),
                    point,
                    placement: CornerPlacement2::Trim,
                });
            }
        }
        if mode == CurveCornerMode2::TrimOrExtend {
            for cut in incident_cuts()?.iter() {
                if cut.placement == CornerPlacement2::Extension {
                    cuts.push(cut.clone());
                }
            }
        }
        Ok(cuts)
    }
}
