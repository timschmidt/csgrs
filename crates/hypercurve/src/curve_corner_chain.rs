//! Exact corner reconstruction for connected open or closed curve chains.
//!
//! The solver returns a connected fragment sequence. Region fill rules, loop
//! roles, and regularization belong to the caller that publishes that sequence.

use super::*;

pub(crate) struct CurveCornerChain2<'a> {
    fragments: &'a [BezierSplitFragment2],
    closed: bool,
}

impl<'a> CurveCornerChain2<'a> {
    /// Borrows a nonempty chain whose joins the caller has already certified.
    pub(crate) fn new(fragments: &'a [BezierSplitFragment2], closed: bool) -> Self {
        debug_assert!(!fragments.is_empty());
        Self { fragments, closed }
    }

    pub(super) fn fragments(&self) -> &'a [BezierSplitFragment2] {
        self.fragments
    }

    pub(super) fn neighbor(&self, index: usize, previous: bool) -> Option<usize> {
        if previous {
            index
                .checked_sub(1)
                .or_else(|| self.closed.then_some(self.fragments.len() - 1))
        } else if index + 1 < self.fragments.len() {
            Some(index + 1)
        } else {
            self.closed.then_some(0)
        }
    }

    pub(super) fn chamfer_vertex_by_setbacks(
        &self,
        vertex_index: usize,
        previous_setback: Real,
        next_setback: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Vec<BezierSplitFragment2>>> {
        let fragment_count = self.fragments().len();
        if vertex_index >= fragment_count || (vertex_index == 0 && !self.closed) {
            return Err(curve_region_edit_error(
                CurveOperation2::Chamfer,
                CurveError::InvalidCurveRange,
            ));
        }
        let previous_index = if vertex_index == 0 {
            fragment_count - 1
        } else {
            vertex_index - 1
        };
        let next_index = vertex_index;
        let previous_fragment = &self.fragments()[previous_index];
        let next_fragment = &self.fragments()[next_index];
        let mut previous_source = CornerCarrierPreparation2::admit(previous_fragment);
        let mut next_source = CornerCarrierPreparation2::admit(next_fragment);
        let previous_family = previous_source.family();
        let next_family = next_source.family();
        let previous_sign = validate_corner_design_value(
            &previous_setback,
            CurveOperation2::Chamfer,
            previous_family,
            policy,
        )?;
        let next_sign = validate_corner_design_value(
            &next_setback,
            CurveOperation2::Chamfer,
            next_family,
            policy,
        )?;
        let previous_has_smooth_run = previous_sign != RealSign::Zero
            && retained_cusp_smooth_run_neighbor(
                self,
                previous_index,
                true,
                CurveOperation2::Chamfer,
                policy,
            )?
            .is_some();
        let next_has_smooth_run = next_sign != RealSign::Zero
            && retained_cusp_smooth_run_neighbor(
                self,
                next_index,
                false,
                CurveOperation2::Chamfer,
                policy,
            )?
            .is_some();
        previous_source.prepare(CurveOperation2::Chamfer, policy)?;
        next_source.prepare(CurveOperation2::Chamfer, policy)?;
        let previous_carrier =
            previous_source.exact_carrier(true, CurveOperation2::Chamfer, policy)?;
        let next_carrier = next_source.exact_carrier(false, CurveOperation2::Chamfer, policy)?;
        let previous_retained_arc = previous_carrier.retained_rational_arc().cloned();
        let next_retained_arc = next_carrier.retained_rational_arc().cloned();
        let solutions = solve_exact_chamfer_corner(
            previous_carrier,
            next_carrier,
            &previous_setback,
            &next_setback,
            previous_sign,
            next_sign,
            mode,
            previous_has_smooth_run,
            next_has_smooth_run,
            previous_family,
            next_family,
            policy,
        )?;
        let solutions = try_map_corner_solutions(solutions, |solution| {
            let (mut previous_cut, mut next_cut) =
                solution.into_retained_cut_evidence().ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Chamfer,
                        previous_family,
                        UncertaintyReason::Unsupported,
                    )
                })?;
            let mut previous_cut_index = previous_index;
            let mut next_cut_index = next_index;
            if previous_has_smooth_run {
                match rebind_retained_cusp_run_cut(
                    self,
                    previous_index,
                    previous_index,
                    true,
                    true,
                    &mut previous_cut,
                    CurveOperation2::Chamfer,
                    policy,
                )? {
                    Some(index) => previous_cut_index = index,
                    None if mode == CurveCornerMode2::TrimOrExtend
                        && previous_cut.placement == CornerPlacement2::Extension => {}
                    None => return Ok(None),
                }
            }
            if next_has_smooth_run {
                match rebind_retained_cusp_run_cut(
                    self,
                    next_index,
                    next_index,
                    false,
                    true,
                    &mut next_cut,
                    CurveOperation2::Chamfer,
                    policy,
                )? {
                    Some(index) => next_cut_index = index,
                    None if mode == CurveCornerMode2::TrimOrExtend
                        && next_cut.placement == CornerPlacement2::Extension => {}
                    None => return Ok(None),
                }
            }
            if (previous_has_smooth_run || next_has_smooth_run)
                && previous_cut_index == next_cut_index
            {
                return Ok(None);
            }
            self.reconstruct_chamfer(
                previous_cut_index,
                next_cut_index,
                previous_cut,
                next_cut,
                previous_retained_arc.as_deref(),
                next_retained_arc.as_deref(),
                policy,
            )
        })?;
        Ok(compact_optional_corner_solutions(solutions))
    }

    /// Replays already selected cuts; contact solving remains with the source
    /// domains, which may contain more than one rational chart.
    pub(crate) fn reconstruct_chamfer(
        &self,
        previous_index: usize,
        next_index: usize,
        mut previous_cut: CornerTrimCut2,
        mut next_cut: CornerTrimCut2,
        previous_retained_arc: Option<&crate::curve::RetainedRationalCornerArc2>,
        next_retained_arc: Option<&crate::curve::RetainedRationalCornerArc2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Vec<BezierSplitFragment2>>> {
        let fragment_count = self.fragments().len();
        let previous_fragment = &self.fragments()[previous_index];
        // The corner solver has already certified these two point
        // witnesses distinct. Capture the chord's exact monotone axis
        // before extension canonicalization reparameterizes either point
        // onto a finite local envelope. The carrier switch preserves the
        // points but can otherwise turn a cheap one-field direction proof
        // into an unnecessary Cartesian compositum.
        let chord_authority = if previous_cut.point.coordinates().is_some()
            && next_cut.point.coordinates().is_some()
        {
            None
        } else {
            match policy
                .strict_predicate_pass(|| {
                    crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                        previous_cut.point.clone(),
                        next_cut.point.clone(),
                        policy,
                    )
                })
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Chamfer, cause))?
            {
                Classification::Decided(chord) => Some(chord),
                Classification::Uncertain(_) => None,
            }
        };
        // Interior cuts retain the authored source parameter and circle
        // certificate. Only extensions need charts beyond that domain.
        let distinct_fragments = previous_index != next_index;
        let previous_replacement = match (
            distinct_fragments,
            previous_retained_arc,
            previous_cut.placement,
        ) {
            (true, Some(support), CornerPlacement2::Extension) => {
                Some(Self::retained_arc_extension_fragments(
                    std::slice::from_ref(&self.fragments()[previous_index]),
                    support,
                    &previous_cut,
                    None,
                    true,
                    CurveOperation2::Chamfer,
                    policy,
                )?)
            }
            _ => None,
        };
        let next_replacement = match (distinct_fragments, next_retained_arc, next_cut.placement) {
            (true, Some(support), CornerPlacement2::Extension) => {
                Some(Self::retained_arc_extension_fragments(
                    std::slice::from_ref(&self.fragments()[next_index]),
                    support,
                    &next_cut,
                    None,
                    false,
                    CurveOperation2::Chamfer,
                    policy,
                )?)
            }
            _ => None,
        };
        if fragment_count == 1
            && (previous_cut.placement == CornerPlacement2::Extension
                || next_cut.placement == CornerPlacement2::Extension)
        {
            if previous_replacement.is_some() || next_replacement.is_some() {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Chamfer,
                    CurveFamily2::CircularArc,
                    UncertaintyReason::Unsupported,
                ));
            }
            Self::canonicalize_retained_single_fragment_extension_cuts(
                previous_fragment,
                &mut previous_cut,
                &mut next_cut,
                CurveOperation2::Chamfer,
                policy,
            )?;
        } else {
            if previous_replacement.is_none() {
                Self::canonicalize_retained_corner_cut(
                    &self.fragments()[previous_index],
                    &mut previous_cut,
                    true,
                    CurveOperation2::Chamfer,
                    policy,
                )?;
            }
            if next_replacement.is_none() {
                Self::canonicalize_retained_corner_cut(
                    &self.fragments()[next_index],
                    &mut next_cut,
                    false,
                    CurveOperation2::Chamfer,
                    policy,
                )?;
            }
        }
        if fragment_count == 1
            && !retained_single_fragment_corner_cuts_are_separated(
                previous_fragment,
                &previous_cut,
                &next_cut,
                CurveOperation2::Chamfer,
                policy,
            )?
        {
            return Ok(None);
        }
        let rebuilt = self.rebuild_retained_chamfer(
            previous_index,
            next_index,
            previous_cut,
            next_cut,
            chord_authority,
            previous_replacement,
            next_replacement,
            policy,
        )?;
        Ok(Some(rebuilt))
    }

    fn rebuild_retained_chamfer(
        &self,
        previous_index: usize,
        next_index: usize,
        previous_cut: CornerTrimCut2,
        next_cut: CornerTrimCut2,
        chord_authority: Option<crate::BezierAlgebraicChord2>,
        previous_replacement: Option<Vec<BezierSplitFragment2>>,
        next_replacement: Option<Vec<BezierSplitFragment2>>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        // A finite selected envelope retains both its extended cut and the
        // authored corner in one parameter field. When the opposite setback
        // is zero, use that correlated corner witness for the chamfer chord.
        // The boundary adjacency and envelope construction already prove it
        // is the same corner; mixing in the old carrier's endpoint would throw
        // away this proof and force an unrelated Cartesian compositum.
        let mut previous_chord_point = previous_cut.point.clone();
        let mut next_chord_point = next_cut.point.clone();
        if previous_cut.placement == CornerPlacement2::Corner
            && next_cut.placement == CornerPlacement2::Extension
            && let Some(CornerReplacement2::SelectedFiber { fragment, .. }) =
                next_cut.replacement.as_ref()
        {
            previous_chord_point = fragment.end_point().clone();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-retained-chamfer",
                "selected-envelope-corner-witness",
            );
        }
        if next_cut.placement == CornerPlacement2::Corner
            && previous_cut.placement == CornerPlacement2::Extension
            && let Some(CornerReplacement2::SelectedFiber { fragment, .. }) =
                previous_cut.replacement.as_ref()
        {
            next_chord_point = fragment.start_point().clone();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-retained-chamfer",
                "selected-envelope-corner-witness",
            );
        }
        let chord = if let (Some(previous_point), Some(next_point)) = (
            previous_chord_point.coordinates(),
            next_chord_point.coordinates(),
        ) {
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(previous_point.clone(), next_point.clone()).map_err(
                        |cause| curve_region_edit_error(CurveOperation2::Chamfer, cause),
                    )?,
                )),
            }
        } else if let Some(authority) = chord_authority {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-retained-chamfer",
                "precanonical-chord-authority",
            );
            BezierSplitFragment2::AlgebraicChord(
                authority
                    .with_certified_equivalent_endpoints(
                        previous_chord_point,
                        next_chord_point,
                        policy,
                    )
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Chamfer, cause))?,
            )
        } else {
            match crate::BezierAlgebraicChord2::try_new(
                previous_chord_point,
                next_chord_point,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Chamfer, cause))?
            {
                Classification::Decided(chord) => BezierSplitFragment2::AlgebraicChord(chord),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Chamfer,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            }
        };
        self.rebuild_retained_corner(
            previous_index,
            next_index,
            previous_cut,
            next_cut,
            vec![chord],
            previous_replacement,
            next_replacement,
            CurveOperation2::Chamfer,
            policy,
        )
    }

    fn retained_arc_extension_fragments(
        source_fragments: &[BezierSplitFragment2],
        arc: &crate::curve::RetainedRationalCornerArc2,
        cut: &CornerTrimCut2,
        contact: Option<&crate::curve::RetainedArcFilletContactSeed2>,
        previous: bool,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        use crate::bezier_split::BezierSelectedFiberFragment2;
        let family = CurveFamily2::CircularArc;
        let source = &arc.fragment;
        let rational = source.rational_curve().expect("a rational circle chart");
        let compare = |left: &CurveParameter2, right: &CurveParameter2| {
            retained_corner_decision(
                left.cmp_by_refinement(right, policy)
                    .map_err(|cause| curve_region_edit_error(operation, cause))?,
                operation,
            )
        };
        // The complement of a selected interval consists of its remaining
        // parent suffix, the parent's circular complement, and its prefix.
        // Each cell keeps its own parameter chart and exact endpoint identity.
        let cell = |curve: &RationalBezier2,
                    start: CurveParameter2,
                    end: CurveParameter2,
                    start_point: CurvePoint2,
                    end_point: CurvePoint2|
         -> ExactCurveResult<Option<BezierSelectedFiberFragment2>> {
            Ok(match compare(&start, &end)? {
                std::cmp::Ordering::Equal => None,
                std::cmp::Ordering::Less => Some(BezierSelectedFiberFragment2::new(
                    BezierSelectedFiberSource2::Rational(curve.clone()),
                    CurveParameterRange2::new_validated(start, end),
                    start_point,
                    end_point,
                )),
                std::cmp::Ordering::Greater => Some(
                    BezierSelectedFiberFragment2::new(
                        BezierSelectedFiberSource2::Rational(curve.clone()),
                        CurveParameterRange2::new_validated(end, start),
                        end_point,
                        start_point,
                    )
                    .reversed(),
                ),
            })
        };
        let (source_start, source_end, parent_start, parent_end) = if source.is_reversed() {
            (
                source.range().end(),
                source.range().start(),
                Real::one(),
                Real::zero(),
            )
        } else {
            (
                source.range().start(),
                source.range().end(),
                Real::zero(),
                Real::one(),
            )
        };
        let support = arc.support();
        let mut cells = Vec::with_capacity(6);
        cells.extend(cell(
            rational,
            source_end.clone(),
            parent_end.into(),
            source.end_point().clone(),
            support.end().clone().into(),
        )?);
        let complement_start = cells.len();
        for span in crate::curve::retained_arc_complement_projective_spans(
            support, operation, family, policy,
        )? {
            let start = span.start().clone().into();
            let end = span.end().clone().into();
            cells.push(BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::Rational(span.into()),
                CurveParameterRange2::new_validated(Real::zero().into(), Real::one().into()),
                start,
                end,
            ));
        }
        cells.extend(cell(
            rational,
            parent_start.into(),
            source_start.clone(),
            support.start().clone().into(),
            source.start_point().clone(),
        )?);
        let mut selected = None;
        for (index, candidate) in cells.iter().enumerate() {
            let parameter = if let Some(crate::curve::RetainedArcFilletContactSeed2 {
                cell: crate::curve::RetainedArcFilletContactCell2::Complement(cell_index),
                parameter,
            }) = contact
            {
                if index != complement_start + cell_index {
                    continue;
                }
                parameter.clone()
            } else {
                let Some(parameter) =
                    crate::curve::RetainedRationalCornerArc2::parameter_at_incident_point(
                        candidate.rational_curve().expect("a circular cell"),
                        &cut.point,
                        operation,
                        family,
                        policy,
                    )?
                else {
                    continue;
                };
                parameter
            };
            let start_order = compare(&parameter, candidate.range().start())?;
            let end_order = compare(&parameter, candidate.range().end())?;
            if start_order.is_lt() || end_order.is_gt() {
                continue;
            }
            let (at_start, at_end) = if candidate.is_reversed() {
                (end_order.is_eq(), start_order.is_eq())
            } else {
                (start_order.is_eq(), end_order.is_eq())
            };
            // Earlier cells own shared endpoints; the retained finite source
            // owns the two outer endpoints of this complementary interval.
            if at_start || (at_end && index + 1 == cells.len()) {
                continue;
            }
            if selected.replace((index, parameter)).is_some() {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "one circular extension contact belongs to multiple cells".into(),
                    ),
                ));
            }
        }
        let (index, parameter) = selected.ok_or_else(|| {
            ExactCurveError::blocked(operation, family, UncertaintyReason::Predicate)
        })?;
        let selected = &cells[index];
        let (start, end) = if selected.is_reversed() {
            (selected.range().end(), selected.range().start())
        } else {
            (selected.range().start(), selected.range().end())
        };
        let partial = if previous {
            cell(
                selected.rational_curve().unwrap(),
                start.clone(),
                parameter,
                selected.start_point().clone(),
                cut.point.clone(),
            )?
        } else {
            cell(
                selected.rational_curve().unwrap(),
                parameter,
                end.clone(),
                cut.point.clone(),
                selected.end_point().clone(),
            )?
        };
        let publish = |fragment: BezierSelectedFiberFragment2| {
            if fragment.range().start().scalar() == Some(&Real::zero())
                && fragment.range().end().scalar() == Some(&Real::one())
            {
                let curve = fragment.rational_curve().unwrap();
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Rational(if fragment.is_reversed() {
                        curve.reversed()
                    } else {
                        curve.clone()
                    }),
                }
            } else {
                BezierSplitFragment2::SelectedFiber(fragment)
            }
        };
        let mut fragments = Vec::with_capacity(source_fragments.len() + cells.len());
        if previous {
            fragments.extend_from_slice(source_fragments);
            fragments.extend(cells[..index].iter().cloned().map(publish));
            fragments.extend(partial.map(publish));
        } else {
            fragments.extend(partial.map(publish));
            fragments.extend(cells[index + 1..].iter().cloned().map(publish));
            fragments.extend_from_slice(source_fragments);
        }
        Ok(fragments)
    }

    fn materialized_corner_arc_fragments(
        arc: &CircularArc2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        let decomposition = match crate::arc_bezier::decompose_circular_arc(arc, policy)
            .map_err(|error| error.with_operation(operation))?
        {
            Classification::Decided(decomposition) => decomposition,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::CircularArc,
                    reason,
                ));
            }
        };
        Ok(decomposition
            .spans()
            .iter()
            .map(|span| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::RationalQuadratic(span.curve().clone()),
            })
            .collect())
    }

    #[allow(clippy::too_many_arguments)]
    fn rebuild_retained_corner(
        &self,
        previous_index: usize,
        next_index: usize,
        previous_cut: CornerTrimCut2,
        next_cut: CornerTrimCut2,
        inserted: Vec<BezierSplitFragment2>,
        previous_replacement: Option<Vec<BezierSplitFragment2>>,
        next_replacement: Option<Vec<BezierSplitFragment2>>,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        let fragment_count = self.fragments().len();
        let same_fragment = previous_index == next_index;
        if same_fragment && (previous_replacement.is_some() || next_replacement.is_some()) {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            ));
        }
        let has_extension = previous_cut.placement == CornerPlacement2::Extension
            || next_cut.placement == CornerPlacement2::Extension;
        if same_fragment && has_extension {
            match (
                previous_cut.replacement.as_ref(),
                next_cut.replacement.as_ref(),
            ) {
                (Some(previous), Some(next)) if previous == next => {}
                (None, None)
                    if matches!(
                        self.fragments()[previous_index],
                        BezierSplitFragment2::RetainedBezier { .. }
                            | BezierSplitFragment2::AnalyticParallel(_)
                            | BezierSplitFragment2::SelectedFiber(_)
                    ) => {}
                _ => {
                    return Err(ExactCurveError::blocked(
                        operation,
                        CurveFamily2::RationalBezier,
                        UncertaintyReason::Unsupported,
                    ));
                }
            }
        }
        let middle_trim = if same_fragment
            && ((previous_cut.placement == CornerPlacement2::Trim
                && next_cut.placement == CornerPlacement2::Trim)
                || has_extension)
        {
            Some(retained_corner_fragment_between_cuts(
                &self.fragments()[previous_index],
                &previous_cut,
                &next_cut,
                operation,
                policy,
            )?)
        } else {
            None
        };
        let previous_trim = if same_fragment
            && (middle_trim.is_some() || previous_cut.placement != CornerPlacement2::Trim)
        {
            None
        } else if let Some(replacement) = previous_replacement {
            Some(replacement)
        } else {
            Some(match previous_cut.placement {
                CornerPlacement2::Trim => vec![retained_corner_fragment_trim(
                    &self.fragments()[previous_index],
                    previous_cut.parameter,
                    &previous_cut.point,
                    previous_cut
                        .replacement
                        .as_ref()
                        .and_then(CornerReplacement2::as_curve),
                    true,
                    operation,
                    policy,
                )?],
                CornerPlacement2::Corner => {
                    vec![self.fragments()[previous_index].clone()]
                }
                CornerPlacement2::Extension => retained_corner_fragment_extension(
                    &self.fragments()[previous_index],
                    previous_cut.parameter,
                    &previous_cut.point,
                    previous_cut.replacement.as_ref(),
                    true,
                    operation,
                    policy,
                )?,
            })
        };
        let next_trim = if same_fragment
            && (middle_trim.is_some() || next_cut.placement != CornerPlacement2::Trim)
        {
            None
        } else if let Some(replacement) = next_replacement {
            Some(replacement)
        } else {
            Some(match next_cut.placement {
                CornerPlacement2::Trim => vec![retained_corner_fragment_trim(
                    &self.fragments()[next_index],
                    next_cut.parameter,
                    &next_cut.point,
                    next_cut
                        .replacement
                        .as_ref()
                        .and_then(CornerReplacement2::as_curve),
                    false,
                    operation,
                    policy,
                )?],
                CornerPlacement2::Corner => {
                    vec![self.fragments()[next_index].clone()]
                }
                CornerPlacement2::Extension => retained_corner_fragment_extension(
                    &self.fragments()[next_index],
                    next_cut.parameter,
                    &next_cut.point,
                    next_cut.replacement.as_ref(),
                    false,
                    operation,
                    policy,
                )?,
            })
        };

        let mut fragments = Vec::with_capacity(fragment_count + inserted.len());
        if same_fragment {
            fragments.extend(inserted);
            if let Some(middle_trim) = middle_trim {
                fragments.push(middle_trim);
            } else {
                if let Some(next_trim) = next_trim {
                    fragments.extend(next_trim);
                }
                if let Some(previous_trim) = previous_trim {
                    fragments.extend(previous_trim);
                }
            }
        } else if previous_index > next_index {
            fragments.extend(inserted);
            fragments.extend(next_trim.expect("a distinct next fragment is retained"));
            fragments.extend(
                self.fragments()[next_index + 1..previous_index]
                    .iter()
                    .cloned(),
            );
            fragments.extend(previous_trim.expect("a distinct previous fragment is retained"));
        } else {
            fragments.extend(self.fragments()[..previous_index].iter().cloned());
            fragments.extend(previous_trim.expect("a distinct previous fragment is retained"));
            fragments.extend(inserted);
            fragments.extend(next_trim.expect("a distinct next fragment is retained"));
            fragments.extend(self.fragments()[next_index + 1..].iter().cloned());
        }
        Ok(fragments)
    }

    pub(super) fn fillet_vertex_by_radius(
        &self,
        vertex_index: usize,
        radius: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Vec<BezierSplitFragment2>>> {
        let fragment_count = self.fragments().len();
        if vertex_index >= fragment_count || (vertex_index == 0 && !self.closed) {
            return Err(curve_region_edit_error(
                CurveOperation2::Fillet,
                CurveError::InvalidCurveRange,
            ));
        }
        let previous_index = if vertex_index == 0 {
            fragment_count - 1
        } else {
            vertex_index - 1
        };
        let next_index = vertex_index;
        let previous_run_authority = if mode == CurveCornerMode2::TrimOnly {
            retained_cusp_smooth_run_authority(
                self,
                previous_index,
                true,
                CurveOperation2::Fillet,
                policy,
            )?
        } else {
            None
        };
        let next_run_authority = if mode == CurveCornerMode2::TrimOnly {
            retained_cusp_smooth_run_authority(
                self,
                next_index,
                false,
                CurveOperation2::Fillet,
                policy,
            )?
        } else {
            None
        };
        let has_smooth_run = previous_run_authority.is_some() || next_run_authority.is_some();
        let previous_solve_index = previous_run_authority.unwrap_or(previous_index);
        let next_solve_index = next_run_authority.unwrap_or(next_index);
        let previous_solve_fragment = &self.fragments()[previous_solve_index];
        let next_solve_fragment = &self.fragments()[next_solve_index];
        let mut previous_source = CornerCarrierPreparation2::admit(previous_solve_fragment);
        let mut next_source = CornerCarrierPreparation2::admit(next_solve_fragment);
        let previous_family = previous_source.family();
        let next_family = next_source.family();
        let radius_sign = validate_corner_design_value(
            &radius,
            CurveOperation2::Fillet,
            previous_family,
            policy,
        )?;
        if radius_sign == RealSign::Zero {
            return Ok(CurveCornerSolutions2::NoSolution(
                crate::CurveCornerNoSolution2::ZeroDesignValue,
            ));
        }
        previous_source.prepare(CurveOperation2::Fillet, policy)?;
        next_source.prepare(CurveOperation2::Fillet, policy)?;
        let previous_carrier =
            previous_source.exact_carrier(true, CurveOperation2::Fillet, policy)?;
        let next_carrier = next_source.exact_carrier(false, CurveOperation2::Fillet, policy)?;
        let previous_retained_arc = previous_carrier.retained_rational_arc().cloned();
        let next_retained_arc = next_carrier.retained_rational_arc().cloned();
        let solve_mode = if has_smooth_run {
            CurveCornerMode2::TrimOrExtend
        } else {
            mode
        };
        let solutions = solve_exact_fillet_corner(
            previous_carrier,
            next_carrier,
            &radius,
            radius_sign,
            solve_mode,
            has_smooth_run,
            previous_family,
            next_family,
            policy,
        )?;
        let solutions = try_map_corner_solutions(solutions, |solution| {
            let (mut previous_cut, mut next_cut, center, clockwise, retained_frame) =
                solution.into_retained_cut_evidence().ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        UncertaintyReason::Unsupported,
                    )
                })?;
            let mut previous_cut_index = previous_index;
            let mut next_cut_index = next_index;
            if has_smooth_run {
                let Some(index) = rebind_retained_cusp_run_cut(
                    self,
                    previous_index,
                    previous_solve_index,
                    true,
                    false,
                    &mut previous_cut,
                    CurveOperation2::Fillet,
                    policy,
                )?
                else {
                    return Ok(None);
                };
                previous_cut_index = index;
                let Some(index) = rebind_retained_cusp_run_cut(
                    self,
                    next_index,
                    next_solve_index,
                    false,
                    false,
                    &mut next_cut,
                    CurveOperation2::Fillet,
                    policy,
                )?
                else {
                    return Ok(None);
                };
                next_cut_index = index;
                if previous_cut_index == next_cut_index {
                    return Ok(None);
                }
            }
            self.reconstruct_fillet(
                previous_cut_index,
                next_cut_index,
                previous_cut,
                next_cut,
                center,
                clockwise,
                retained_frame,
                &radius,
                [
                    previous_retained_arc.as_deref(),
                    next_retained_arc.as_deref(),
                ],
                [
                    (previous_cut_index == previous_index)
                        .then(|| previous_source.promoted_parallel())
                        .flatten(),
                    (next_cut_index == next_index)
                        .then(|| next_source.promoted_parallel())
                        .flatten(),
                ],
                [
                    previous_cut_index..previous_cut_index + 1,
                    next_cut_index..next_cut_index + 1,
                ],
                policy,
            )
        })?;
        Ok(compact_optional_corner_solutions(solutions))
    }

    /// Reconstructs already solved cuts over their complete source domains.
    pub(crate) fn reconstruct_fillet(
        &self,
        previous_index: usize,
        next_index: usize,
        mut previous_cut: CornerTrimCut2,
        mut next_cut: CornerTrimCut2,
        center: CurvePoint2,
        clockwise: bool,
        retained_frame: Option<RetainedFilletFrame2>,
        radius: &Real,
        retained_arcs: [Option<&crate::curve::RetainedRationalCornerArc2>; 2],
        promoted_parallels: [Option<&crate::BezierParallelFragment2>; 2],
        source_domains: [std::ops::Range<usize>; 2],
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Vec<BezierSplitFragment2>>> {
        let fragment_count = self.fragments().len();
        let previous_fragment = &self.fragments()[previous_index];
        debug_assert!(source_domains[0].contains(&previous_index));
        debug_assert!(source_domains[1].contains(&next_index));
        let deferred_arc_is_previous = retained_frame
            .as_ref()
            .and_then(|frame| frame.anchor_evidence.as_ref())
            .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
            .map(|deferred| deferred.arc_is_previous);
        let mut replacements = [None, None];
        for (index, cut) in [&previous_cut, &next_cut].into_iter().enumerate() {
            if let Some(support) = retained_arcs[index]
                && deferred_arc_is_previous != Some(index == 0)
                && cut.placement == CornerPlacement2::Extension
            {
                replacements[index] = Some(Self::retained_arc_extension_fragments(
                    &self.fragments()[source_domains[index].clone()],
                    support,
                    cut,
                    None,
                    index == 0,
                    CurveOperation2::Fillet,
                    policy,
                )?);
            }
        }
        let [mut previous_replacement, mut next_replacement] = replacements;
        if fragment_count == 1
            && deferred_arc_is_previous.is_none()
            && (previous_cut.placement == CornerPlacement2::Extension
                || next_cut.placement == CornerPlacement2::Extension)
        {
            Self::canonicalize_retained_single_fragment_extension_cuts(
                previous_fragment,
                &mut previous_cut,
                &mut next_cut,
                CurveOperation2::Fillet,
                policy,
            )?;
        } else {
            if deferred_arc_is_previous != Some(true) && previous_replacement.is_none() {
                Self::canonicalize_retained_corner_cut(
                    &self.fragments()[previous_index],
                    &mut previous_cut,
                    true,
                    CurveOperation2::Fillet,
                    policy,
                )?;
            }
            if deferred_arc_is_previous != Some(false) && next_replacement.is_none() {
                Self::canonicalize_retained_corner_cut(
                    &self.fragments()[next_index],
                    &mut next_cut,
                    false,
                    CurveOperation2::Fillet,
                    policy,
                )?;
            }
        }
        if fragment_count == 1
            && !retained_single_fragment_corner_cuts_are_separated(
                previous_fragment,
                &previous_cut,
                &next_cut,
                CurveOperation2::Fillet,
                policy,
            )?
        {
            return Ok(None);
        }
        let mut candidate_valid = true;
        let inserted = Self::retained_fillet_fragments(
            &self.fragments()[source_domains[0].clone()],
            &self.fragments()[source_domains[1].clone()],
            &mut previous_cut,
            &mut next_cut,
            center,
            clockwise,
            retained_frame,
            &radius,
            false,
            promoted_parallels[0],
            promoted_parallels[1],
            &mut previous_replacement,
            &mut next_replacement,
            &mut candidate_valid,
            policy,
        )?;
        if !candidate_valid {
            return Ok(None);
        }
        // A deferred native arc may span several charts. Its replacement
        // owns the complete authored side, including any complement cells.
        let previous_index = if previous_replacement.is_some() {
            source_domains[0].start
        } else {
            previous_index
        };
        let next_index = if next_replacement.is_some() {
            source_domains[1].end - 1
        } else {
            next_index
        };
        let rebuilt = self.rebuild_retained_corner(
            previous_index,
            next_index,
            previous_cut,
            next_cut,
            inserted,
            previous_replacement,
            next_replacement,
            CurveOperation2::Fillet,
            policy,
        )?;
        Ok(Some(rebuilt))
    }

    fn retained_deferred_arc_contact_on_rational(
        fillet: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        source: &RationalBezier2,
        deferred: &crate::curve::RetainedDeferredArcFilletContact2,
        allow_source_start: bool,
        allow_source_end: bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<RetainedDeferredArcContact2>> {
        let zero = CurveParameter2::from(BezierParameter2::Exact(Real::zero()));
        let one = CurveParameter2::from(BezierParameter2::Exact(Real::one()));
        let circles = [fillet.clone(), fillet.complementary_half()];
        for (fillet_half, circle) in circles.iter().enumerate() {
            let intersections = circle.certified_tangent_rational_intersections(
                source,
                &deferred.source.support(),
                &deferred.source_radius,
                &deferred.signed_center_radius,
                policy,
            );
            let (intersections, parameter_map) = match intersections
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            };
            let mut retained = None;
            let mut retain_contact = |
                source_parameter: CurveParameter2,
                location: crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2,
                tangent_cross_sign: RealSign,
                point: CurvePoint2,
                fillet_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
            | -> ExactCurveResult<()> {
                use crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2;

                if tangent_cross_sign != RealSign::Zero {
                    return Err(curve_region_edit_error(
                        CurveOperation2::Fillet,
                        CurveError::Topology(
                            "a certified tangent arc contact had nonzero tangent cross".into(),
                        ),
                    ));
                }
                if (fillet_half == 0
                    && location == BezierAlgebraicCuspSemicircleContactLocation2::Start)
                    || (fillet_half == 1
                        && location == BezierAlgebraicCuspSemicircleContactLocation2::End)
                {
                    return Ok(());
                }
                let order = |boundary: &CurveParameter2| {
                    source_parameter
                        .cmp_by_refinement(boundary, policy)
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))
                        .and_then(|ordering| match ordering {
                            Classification::Decided(ordering) => Ok(ordering),
                            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::CircularArc,
                                reason,
                            )),
                        })
                };
                let zero_order = order(&zero)?;
                let one_order = order(&one)?;
                let source_at_start = zero_order == std::cmp::Ordering::Equal;
                let source_at_end = one_order == std::cmp::Ordering::Equal;
                if zero_order == std::cmp::Ordering::Less
                    || one_order == std::cmp::Ordering::Greater
                    || (source_at_start && !allow_source_start)
                    || (source_at_end && !allow_source_end)
                {
                    return Ok(());
                }
                if retained.is_some() {
                    return Err(curve_region_edit_error(
                        CurveOperation2::Fillet,
                        CurveError::Topology(
                            "one tangent arc/Bezier fillet retained multiple circular contacts"
                                .into(),
                        ),
                    ));
                }
                retained = Some(RetainedDeferredArcContact2 {
                    source_parameter,
                    source_at_start,
                    source_at_end,
                    point,
                    fillet_parameter,
                    fillet_half: fillet_half as u8,
                });
                Ok(())
            };
            match intersections {
                crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberContacts(contacts) => {
                    for contact in contacts {
                        retain_contact(
                            CurveParameter2::from_selected_fiber(
                                contact.other_parameter().clone(),
                            ),
                            contact.location(),
                            contact.tangent_cross_sign(),
                            contact.point_evidence(),
                            contact.cusp_parameter(),
                        )?;
                    }
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Contacts(contacts) => {
                    for contact in contacts {
                        use crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2;

                        let fillet_parameter = match contact.location {
                            BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                                    Real::zero(),
                                )
                            }
                            BezierAlgebraicCuspSemicircleContactLocation2::End => {
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                                    Real::one(),
                                )
                            }
                            BezierAlgebraicCuspSemicircleContactLocation2::Interior => parameter_map
                                .as_ref()
                                .ok_or_else(|| {
                                    curve_region_edit_error(
                                        CurveOperation2::Fillet,
                                        CurveError::Topology(
                                            "an ordinary retained arc contact lost its circle parameter map"
                                                .into(),
                                        ),
                                    )
                                })?
                                .contact_parameter(&contact),
                        };
                        retain_contact(
                            contact.other_parameter.clone(),
                            contact.location,
                            contact.tangent_cross_sign,
                            contact.point,
                            fillet_parameter,
                        )?;
                    }
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberOverlaps(_)
                | crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Overlaps(_) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        UncertaintyReason::Boundary,
                    ));
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        UncertaintyReason::Predicate,
                    ));
                }
            }
            if retained.is_some() {
                return Ok(retained);
            }
        }
        Ok(None)
    }

    /// Replays the source-circle parameter carried through an exact affine
    /// radial offset. The center solve has already certified incidence on the
    /// offset chart, so the same projective parameter names the source-circle
    /// tangency without asking a resultant solver to rediscover a double root.
    fn retained_preselected_arc_fillet_contact(
        source_fragments: &[BezierSplitFragment2],
        deferred: &crate::curve::RetainedDeferredArcFilletContact2,
        seed: &crate::curve::RetainedArcFilletContactSeed2,
        arc_cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<RetainedPreselectedArcFilletContact2> {
        let (source_spans, span_index, authored) = match seed.cell {
            crate::curve::RetainedArcFilletContactCell2::Authored(index) => {
                let decomposition = match deferred
                    .source
                    .support()
                    .rational_bezier_decomposition_with_policy(policy)
                    .map_err(|error| error.with_operation(CurveOperation2::Fillet))?
                {
                    Classification::Decided(decomposition) => decomposition,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::CircularArc,
                            reason,
                        ));
                    }
                };
                (
                    decomposition
                        .spans()
                        .iter()
                        .map(|span| span.curve().clone())
                        .collect::<Vec<_>>(),
                    index,
                    true,
                )
            }
            crate::curve::RetainedArcFilletContactCell2::Complement(index) => (
                crate::curve::retained_arc_complement_projective_spans(
                    &deferred.source.support(),
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    policy,
                )?,
                index,
                false,
            ),
        };
        let span = source_spans.get(span_index).ok_or_else(|| {
            curve_region_edit_error(
                CurveOperation2::Fillet,
                CurveError::Topology("a retained arc contact named a missing circle cell".into()),
            )
        })?;
        let source_curve = RationalBezier2::from(span.clone());
        let source_parallel = source_curve
            .parallel_left(Real::zero())
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
        let source_direction = match real_sign(&deferred.signed_center_radius, policy) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) => {
                return Err(curve_region_edit_error(
                    CurveOperation2::Fillet,
                    CurveError::Topology(
                        "a preselected arc contact retained a zero offset radius".into(),
                    ),
                ));
            }
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    UncertaintyReason::RealSign,
                ));
            }
        };
        // The center solve already certified this projective parameter and
        // `fillet_cut_from_center` mapped its exact center point radially back
        // to the source circle. Reuse both pieces of evidence, including the
        // compact selected-fiber parameter, rather than solving the same
        // incidence again in a larger field.
        let point = arc_cut.point.clone();
        if let Some(arc) = deferred.source.retained_rational_arc() {
            let replacement = if arc_cut.placement == CornerPlacement2::Extension {
                Some(Self::retained_arc_extension_fragments(
                    source_fragments,
                    arc,
                    arc_cut,
                    Some(seed),
                    deferred.arc_is_previous,
                    CurveOperation2::Fillet,
                    policy,
                )?)
            } else {
                None
            };
            return Ok(RetainedPreselectedArcFilletContact2 {
                replacement,
                source_parallel,
                source_parameter: seed.parameter.clone(),
                source_direction,
            });
        }
        arc_cut.parameter = seed.parameter.clone();
        arc_cut.placement = if authored {
            CornerPlacement2::Trim
        } else {
            CornerPlacement2::Extension
        };
        if authored && source_spans.len() == 1 {
            arc_cut.replacement = Some(CornerReplacement2::Curve(BezierSubcurve2::Rational(
                source_curve,
            )));
            return Ok(RetainedPreselectedArcFilletContact2 {
                replacement: None,
                source_parallel,
                source_parameter: seed.parameter.clone(),
                source_direction,
            });
        }
        arc_cut.replacement = None;
        let boundary_order = |boundary: Real| {
            seed.parameter
                .cmp_by_refinement(
                    &CurveParameter2::from(BezierParameter2::Exact(boundary)),
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))
                .and_then(|order| match order {
                    Classification::Decided(order) => Ok(order),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    )),
                })
        };
        let endpoint = if boundary_order(Real::zero())? == std::cmp::Ordering::Equal {
            Some(BezierEndpoint::Start)
        } else if boundary_order(Real::one())? == std::cmp::Ordering::Equal {
            Some(BezierEndpoint::End)
        } else {
            None
        };
        let replacement = retained_circular_cut_fragments(
            (!authored).then_some(source_fragments),
            &source_spans,
            span_index,
            &seed.parameter,
            &point,
            endpoint,
            deferred.arc_is_previous,
        );
        Ok(RetainedPreselectedArcFilletContact2 {
            replacement: Some(replacement),
            source_parallel,
            source_parameter: seed.parameter.clone(),
            source_direction,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn retained_deferred_arc_fillet_fragments(
        frame: &RetainedFilletFrame2,
        fillet: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        center: &CurvePoint2,
        source_fragments: &[BezierSplitFragment2],
        deferred: &crate::curve::RetainedDeferredArcFilletContact2,
        anchor_cut: &mut CornerTrimCut2,
        arc_cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<RetainedDeferredArcFilletResult2>> {
        if source_fragments.is_empty() {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                CurveFamily2::CircularArc,
                UncertaintyReason::Unsupported,
            ));
        }
        let rational_arc = deferred.source.retained_rational_arc();
        let source_chart = matches!(
            deferred.domain,
            crate::curve::FilletContactDomain2::SourceChart(_)
        );
        let decomposition = match deferred
            .source
            .support()
            .rational_bezier_decomposition_with_policy(policy)
            .map_err(|error| error.with_operation(CurveOperation2::Fillet))?
        {
            Classification::Decided(decomposition) => decomposition,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    reason,
                ));
            }
        };
        // The retained center lies on the signed concentric offset circle.
        // Its radial direction locates the source contact without adjoining a
        // second selected parameter just to reject another circular chart.
        // Each canonical chart is shorter than a semicircle. Unknown sides
        // leave the contact to the full certified intersection replay below.
        let radial_sign =
            policy.strict_predicate_pass(|| real_sign(&deferred.signed_center_radius, policy));
        let inside_side = match radial_sign {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => Some(
                if deferred.source.support().is_clockwise() ^ (sign == RealSign::Negative) {
                    crate::LineSide::Right
                } else {
                    crate::LineSide::Left
                },
            ),
            _ => None,
        };
        let center_side = |endpoint: &Point2| -> CurveResult<Option<crate::LineSide>> {
            policy.strict_predicate_pass(|| {
                let chord = match crate::bezier_offset::BezierAlgebraicChord2::try_new(
                    deferred.source.support().center().clone().into(),
                    endpoint.clone().into(),
                    policy,
                )? {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(_) => return Ok(None),
                };
                chord
                    .oriented_support_side(center, policy)
                    .map(|side| match side {
                        Classification::Decided(side) => Some(side),
                        Classification::Uncertain(_) => None,
                    })
            })
        };
        // Canonical circular charts retain the full authored sweep. An internal
        // chart boundary belongs to its following chart; both outer source
        // endpoints remain excluded from a strict corner contact.
        let select_contact =
            |spans: &[RationalQuadraticBezier2],
             authored: bool|
             -> ExactCurveResult<Option<(usize, RetainedDeferredArcContact2)>> {
                for (index, span) in spans.iter().enumerate() {
                    let include_start = rational_arc.is_some()
                        || index != 0
                        || (authored && source_chart && !deferred.arc_is_previous);
                    let include_end = rational_arc.is_some()
                        || (authored
                            && source_chart
                            && deferred.arc_is_previous
                            && index + 1 == spans.len());
                    if let Some(inside_side) = inside_side {
                        let outside_side = if inside_side == crate::LineSide::Left {
                            crate::LineSide::Right
                        } else {
                            crate::LineSide::Left
                        };
                        if center_side(span.start())
                            .map_err(|cause| {
                                curve_region_edit_error(CurveOperation2::Fillet, cause)
                            })?
                            .is_some_and(|side| {
                                side != inside_side
                                    && !(include_start && side == crate::LineSide::On)
                            })
                            || center_side(span.end())
                                .map_err(|cause| {
                                    curve_region_edit_error(CurveOperation2::Fillet, cause)
                                })?
                                .is_some_and(|side| {
                                    side != outside_side
                                        && !(include_end && side == crate::LineSide::On)
                                })
                        {
                            continue;
                        }
                    }
                    let rational = RationalBezier2::from(span.clone());
                    let Some(contact) = Self::retained_deferred_arc_contact_on_rational(
                        &fillet,
                        &rational,
                        deferred,
                        include_start,
                        include_end,
                        policy,
                    )?
                    else {
                        continue;
                    };
                    // The signed-radius tangency certificate has one contact on
                    // the source circle; replaying other charts cannot add one.
                    return Ok(Some((index, contact)));
                }
                Ok(None)
            };
        let mut spans = decomposition
            .spans()
            .iter()
            .map(|span| span.curve().clone())
            .collect::<Vec<_>>();
        let mut selected = if rational_arc.is_none()
            && source_chart
            && arc_cut.placement == CornerPlacement2::Extension
        {
            None
        } else {
            select_contact(&spans, true)?
        };
        let mut placement = CornerPlacement2::Trim;
        if selected.is_none() {
            if source_chart && arc_cut.placement != CornerPlacement2::Extension {
                // The source inverse already certified this finite location.
                // A coincident fillet endpoint cannot become an extension.
                return Ok(None);
            }
            if deferred.domain.mode() != CurveCornerMode2::TrimOrExtend
                || deferred.source.support().start() == deferred.source.support().end()
            {
                return Ok(None);
            }
            spans = crate::curve::retained_arc_complement_projective_spans(
                &deferred.source.support(),
                CurveOperation2::Fillet,
                CurveFamily2::CircularArc,
                policy,
            )?;
            selected = select_contact(&spans, false)?;
            placement = CornerPlacement2::Extension;
        }
        let Some((span_index, contact)) = selected else {
            return Ok(None);
        };
        // This is the terminal fillet parameter's own point witness. Bind the
        // source cut to it before assembling any circular extension, so path
        // admission reuses the certified tangency instead of comparing two
        // separately evaluated images of the same selected center.
        arc_cut.point = contact.point.clone();
        let retains_source_parameter =
            rational_arc.is_some() || (source_chart && placement == CornerPlacement2::Trim);
        let contact_seed = crate::curve::RetainedArcFilletContactSeed2 {
            cell: if placement == CornerPlacement2::Extension {
                crate::curve::RetainedArcFilletContactCell2::Complement(span_index)
            } else {
                crate::curve::RetainedArcFilletContactCell2::Authored(span_index)
            },
            parameter: contact.source_parameter.clone(),
        };
        let arc_replacement = if let Some(arc) = rational_arc {
            if arc_cut.placement == CornerPlacement2::Extension {
                Some(Self::retained_arc_extension_fragments(
                    source_fragments,
                    arc,
                    arc_cut,
                    Some(&contact_seed),
                    deferred.arc_is_previous,
                    CurveOperation2::Fillet,
                    policy,
                )?)
            } else {
                None
            }
        } else if retains_source_parameter {
            None
        } else if placement == CornerPlacement2::Trim && spans.len() == 1 {
            arc_cut.replacement = Some(CornerReplacement2::Curve(BezierSubcurve2::Rational(
                RationalBezier2::from(spans[0].clone()),
            )));
            None
        } else {
            let endpoint = if contact.source_at_start {
                Some(BezierEndpoint::Start)
            } else if contact.source_at_end {
                Some(BezierEndpoint::End)
            } else {
                None
            };
            arc_cut.replacement = None;
            Some(retained_circular_cut_fragments(
                (placement == CornerPlacement2::Extension).then_some(source_fragments),
                &spans,
                span_index,
                &contact.source_parameter,
                &contact.point,
                endpoint,
                deferred.arc_is_previous,
            ))
        };
        if !retains_source_parameter {
            arc_cut.parameter = contact.source_parameter;
            arc_cut.placement = placement;
        }
        let crosses_complementary_half = contact.fillet_half != 0;
        let terminal_circle = if crosses_complementary_half {
            fillet.complementary_half()
        } else {
            fillet.clone()
        };
        let fillet_fragments = Self::publish_retained_circle_fillet(
            frame.anchor_is_previous,
            fillet,
            terminal_circle,
            contact.fillet_parameter,
            crosses_complementary_half,
            Some(contact.point),
            anchor_cut,
            arc_cut,
            policy,
        )?;
        Ok(Some(RetainedDeferredArcFilletResult2 {
            fillet_fragments,
            arc_replacement,
        }))
    }

    fn retained_fillet_sweep(
        frame: &RetainedFilletFrame2,
        other_parallel: &BezierParallel2,
        other_parameter: &BezierParameter2,
        other_reversed: bool,
        fillet_clockwise: bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<(u8, RealSign, RealSign)> {
        let (tangent_cross, tangent_dot) = if let Some(relation) = frame.anchor_evidence.as_ref() {
            match (relation.cross, relation.dot) {
                (Some(cross), dot) => (cross, dot.unwrap_or(RealSign::Zero)),
                (None, Some(dot)) => (RealSign::Zero, dot),
                (None, None) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        UncertaintyReason::Predicate,
                    ));
                }
            }
        } else {
            let relation = |cross: bool| -> ExactCurveResult<RealSign> {
                let classification = match &frame.radial_frame {
                    RetainedFilletRadialFrame2::RepresentedUnitNormal(unit_normal) => {
                        let line_tangent = (unit_normal.1.clone(), -unit_normal.0.clone());
                        other_parallel
                            .vector_tangent_cross_and_dot_signs(
                                other_parameter,
                                &line_tangent.0,
                                &line_tangent.1,
                                policy,
                            )
                            .map(|classification| {
                                classification.map(
                                    |(cross_sign, dot_sign)| {
                                        if cross { cross_sign } else { dot_sign }
                                    },
                                )
                            })
                    }
                    RetainedFilletRadialFrame2::ChordNormal {
                        anchor,
                        policy: frame_policy,
                    } => {
                        if frame_policy != policy {
                            return Err(curve_region_edit_error(
                                CurveOperation2::Fillet,
                                CurveError::Topology(
                                    "a chord-normal fillet sweep crossed predicate policies".into(),
                                ),
                            ));
                        }
                        if let Some(tangent) = anchor.certified_unit_tangent().or_else(|| {
                            anchor
                                .certified_axis_direction()
                                .map(BezierAlgebraicChordAxisDirection2::unit_tangent)
                        }) {
                            other_parallel
                                .vector_tangent_cross_and_dot_signs(
                                    other_parameter,
                                    &tangent.0,
                                    &tangent.1,
                                    policy,
                                )
                                .map(|classification| {
                                    classification.map(
                                        |(cross_sign, dot_sign)| {
                                            if cross { cross_sign } else { dot_sign }
                                        },
                                    )
                                })
                        } else {
                            let zero = Real::zero();
                            let one = Real::one();
                            anchor.tangent_cross_dot_parallel_linear_combination_sign(
                                other_parallel,
                                other_parameter,
                                if cross { &one } else { &zero },
                                if cross { &zero } else { &one },
                                policy,
                            )
                        }
                    }
                    _ => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::RationalBezier,
                            UncertaintyReason::Unsupported,
                        ));
                    }
                }
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
                match classification {
                    Classification::Decided(sign) => Ok(sign),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    )),
                }
            };
            let (mut tangent_cross, mut tangent_dot) = (relation(true)?, relation(false)?);
            if other_reversed {
                tangent_cross = exact_sign_reverse(tangent_cross);
                tangent_dot = exact_sign_reverse(tangent_dot);
            }
            (tangent_cross, tangent_dot)
        };
        Self::retained_fillet_sweep_from_tangent_relation(
            tangent_cross,
            tangent_dot,
            fillet_clockwise,
        )
    }

    fn retained_fillet_sweep_from_tangent_relation(
        tangent_cross: RealSign,
        tangent_dot: RealSign,
        fillet_clockwise: bool,
    ) -> ExactCurveResult<(u8, RealSign, RealSign)> {
        let sweep_halves = match (fillet_clockwise, tangent_cross) {
            (false, RealSign::Positive) | (true, RealSign::Negative) => 1_u8,
            (false, RealSign::Negative) | (true, RealSign::Positive) => 2_u8,
            (_, RealSign::Zero) if tangent_dot == RealSign::Negative => 1_u8,
            (_, RealSign::Zero) if tangent_dot == RealSign::Positive => {
                return Err(curve_region_edit_error(
                    CurveOperation2::Fillet,
                    CurveError::Topology(
                        "distinct fillet contacts retained the same oriented tangent".into(),
                    ),
                ));
            }
            (_, RealSign::Zero) => {
                return Err(curve_region_edit_error(
                    CurveOperation2::Fillet,
                    CurveError::Topology(
                        "regular fillet tangents had zero cross and dot products".into(),
                    ),
                ));
            }
        };
        Ok((sweep_halves, tangent_cross, tangent_dot))
    }

    #[allow(clippy::too_many_arguments)]
    /// Publishes every retained selected-circle fillet through one exact
    /// one- or two-half representation. A supplied terminal point preserves
    /// correlated source-contact evidence; otherwise the terminal circle owns
    /// exact point recovery from its selected angular parameter.
    fn publish_retained_circle_fillet(
        anchor_is_previous: bool,
        fillet: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        terminal_circle: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        terminal_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
        crosses_complementary_half: bool,
        terminal_point: Option<CurvePoint2>,
        anchor_cut: &mut CornerTrimCut2,
        terminal_cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        terminal_cut.point = if let Some(point) = terminal_point {
            point
        } else {
            match terminal_parameter
                .coincident_point_evidence(&terminal_circle, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        UncertaintyReason::Unsupported,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            }
        };
        anchor_cut.point = match fillet
            .start_point_evidence(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    reason,
                ));
            }
        };

        let zero =
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero());
        let mut fragments = if crosses_complementary_half {
            vec![
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    fillet,
                    zero.clone(),
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                        Real::one(),
                    ),
                    false,
                    policy,
                ),
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    terminal_circle,
                    zero,
                    terminal_parameter,
                    false,
                    policy,
                ),
            ]
        } else {
            vec![
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    fillet,
                    zero,
                    terminal_parameter,
                    false,
                    policy,
                ),
            ]
        };
        if !anchor_is_previous {
            fragments.reverse();
            for fragment in &mut fragments {
                *fragment = fragment.reversed();
            }
        }
        Ok(fragments
            .into_iter()
            .map(|fragment| fragment.with_certified_tangent_endpoints())
            .map(BezierSplitFragment2::AlgebraicCuspSemicircle)
            .collect())
    }

    #[allow(clippy::too_many_arguments)]
    fn retained_parallel_fillet_fragments(
        frame: &RetainedFilletFrame2,
        fillet: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        other_parallel: &BezierParallel2,
        other_parameter: BezierParameter2,
        other_reversed: bool,
        fillet_clockwise: bool,
        anchor_cut: &mut CornerTrimCut2,
        other_cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record("hypercurve", "curve-region-fillet-parallel", "entered");
        other_cut.parameter = CurveParameter2::from(other_parameter.clone());
        let (sweep_halves, tangent_cross, tangent_dot) = Self::retained_fillet_sweep(
            frame,
            other_parallel,
            &other_parameter,
            other_reversed,
            fillet_clockwise,
            policy,
        )?;
        #[cfg(feature = "dispatch-trace")]
        {
            let sign = |sign| match sign {
                RealSign::Negative => "negative",
                RealSign::Zero => "zero",
                RealSign::Positive => "positive",
            };
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-parallel-sweep",
                if sweep_halves == 1 {
                    "one-half"
                } else {
                    "two-halves"
                },
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-parallel-cross",
                sign(tangent_cross),
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-parallel-dot",
                sign(tangent_dot),
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-parallel-anchor",
                if frame.anchor_is_previous {
                    "previous"
                } else {
                    "next"
                },
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-parallel-direction",
                match (fillet_clockwise, other_reversed) {
                    (false, false) => "ccw-forward",
                    (false, true) => "ccw-reversed",
                    (true, false) => "cw-forward",
                    (true, true) => "cw-reversed",
                },
            );
        }
        let terminal_circle = if sweep_halves == 2 {
            fillet.complementary_half()
        } else {
            fillet.clone()
        };
        let fillet_parameter = if terminal_circle.uses_selected_parallel_normal_frame()
            || terminal_circle.uses_selected_chord_normal_frame()
        {
            let derivative_scale = match other_parallel
                .parallel_derivative_scale_sign(&other_parameter, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        UncertaintyReason::Boundary,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
            let traversal_agrees_with_source =
                (derivative_scale == RealSign::Positive) != other_reversed;
            // `fillet_clockwise` follows the locally published circle from
            // anchor to companion. Anchoring on the next boundary carrier
            // reverses that traversal, but it does not reverse the common
            // offset side on which the center was solved.
            let common_offset_clockwise = if frame.anchor_is_previous {
                fillet_clockwise
            } else {
                !fillet_clockwise
            };
            let signed_offset_sign = if common_offset_clockwise {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            let other_radial_sign = if traversal_agrees_with_source {
                exact_sign_reverse(signed_offset_sign)
            } else {
                signed_offset_sign
            };
            #[cfg(feature = "dispatch-trace")]
            {
                let sign = |value| match value {
                    RealSign::Negative => "negative",
                    RealSign::Zero => "zero",
                    RealSign::Positive => "positive",
                };
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-fillet-parallel-derivative-scale",
                    sign(derivative_scale),
                );
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-fillet-parallel-other-radial",
                    sign(other_radial_sign),
                );
            }
            let frame_cross = if frame.anchor_is_previous {
                tangent_cross
            } else {
                exact_sign_reverse(tangent_cross)
            };
            match terminal_circle
                .certified_selected_parallel_contact_parameter(
                    other_parallel.clone(),
                    other_parameter.clone(),
                    if tangent_cross == RealSign::Zero {
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End
                    } else {
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior
                    },
                    other_radial_sign,
                    frame_cross,
                    tangent_dot,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            }
        } else if tangent_cross == RealSign::Zero {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        } else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-parallel",
                "general-parameter-map",
            );
            let parameter_map = match terminal_circle
                .parallel_parameter_map(other_parallel, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(parameter_map) => parameter_map,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-parallel",
                "parameter-map-decided",
            );
            parameter_map.certified_interior_tangent_parameter(other_parameter)
        };
        Self::publish_retained_circle_fillet(
            frame.anchor_is_previous,
            fillet,
            terminal_circle,
            fillet_parameter,
            sweep_halves == 2,
            None,
            anchor_cut,
            other_cut,
            policy,
        )
    }

    /// Reconstructs a selected line, analytic, or circular/chord fillet from
    /// the center contact retained by the offset solver.
    ///
    /// Represented and analytic selected-normal frames map the chord cut
    /// directly. A concentric circular frame rejoins the authoritative
    /// selected-circle/chord incidence kernel and selects the already-authored
    /// chord cut; it does not introduce another family solver.
    fn retained_chord_fillet_fragments(
        frame: &RetainedFilletFrame2,
        fillet: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        chord: &crate::BezierAlgebraicChord2,
        fillet_clockwise: bool,
        anchor_cut: &mut CornerTrimCut2,
        chord_cut: &mut CornerTrimCut2,
        candidate_valid: &mut bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record("hypercurve", "curve-region-fillet-chord", "entered");
        let Some(relation) = frame.anchor_evidence.as_ref() else {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Predicate,
            ));
        };
        let (tangent_cross, tangent_dot) = match (relation.cross, relation.dot) {
            (Some(cross), Some(dot)) => (cross, dot),
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Predicate,
                ));
            }
        };
        let (sweep_halves, tangent_cross, _) = Self::retained_fillet_sweep_from_tangent_relation(
            tangent_cross,
            tangent_dot,
            fillet_clockwise,
        )?;
        let terminal_circle = if sweep_halves == 2 {
            fillet.complementary_half()
        } else {
            fillet.clone()
        };
        let fillet_parameter = if tangent_cross == RealSign::Zero {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        } else {
            let parameter = match &frame.radial_frame {
                RetainedFilletRadialFrame2::RepresentedUnitNormal(unit_normal) => {
                    let anchor_tangent = (unit_normal.1.clone(), -unit_normal.0.clone());
                    terminal_circle.certified_chord_normal_contact_parameter(
                        crate::bezier_offset::BezierSelectedChordNormalAnchor2::Represented(
                            anchor_tangent,
                        ),
                        chord.clone(),
                        chord_cut.point.clone(),
                        frame.radial_distance.clone(),
                        false,
                        policy,
                    )
                    .map(|classification| classification.map(Some))
                }
                RetainedFilletRadialFrame2::ChordNormal { anchor, .. } => terminal_circle
                    .certified_chord_normal_contact_parameter(
                        crate::bezier_offset::BezierSelectedChordNormalAnchor2::RetainedChord(
                            anchor.clone(),
                        ),
                        chord.clone(),
                        chord_cut.point.clone(),
                        frame.radial_distance.clone(),
                        false,
                        policy,
                    )
                    .map(|classification| classification.map(Some)),
                RetainedFilletRadialFrame2::ParallelNormal { .. } => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-fillet-chord-frame",
                        "parallel-normal",
                    );
                    terminal_circle.certified_selected_chord_parallel_normal_contact_parameter(
                        chord.clone(),
                        chord_cut.point.clone(),
                        frame.radial_distance.clone(),
                        tangent_cross,
                        policy,
                    )
                    .map(|classification| classification.map(Some))
                }
                RetainedFilletRadialFrame2::SelectedConcentric { .. } => terminal_circle
                    .certified_selected_chord_retained_contact_parameter(
                        chord.clone(),
                        chord_cut.point.clone(),
                        frame.radial_distance.clone(),
                        policy,
                    ),
                RetainedFilletRadialFrame2::ConcentricArc { .. }
                    if terminal_circle.uses_selected_radial_frame() =>
                {
                    terminal_circle.certified_selected_chord_retained_contact_parameter(
                        chord.clone(),
                        chord_cut.point.clone(),
                        frame.radial_distance.clone(),
                        policy,
                    )
                }
                RetainedFilletRadialFrame2::ConcentricArc { .. } => {
                    let expected_parameter = chord_cut
                        .parameter
                        .as_algebraic_chord()
                        .cloned()
                        .ok_or_else(|| {
                            ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::RationalBezier,
                                UncertaintyReason::Unsupported,
                            )
                        })?;
                    let contacts = match terminal_circle
                        .chord_intersections(chord, policy)
                        .map_err(|cause| {
                            curve_region_edit_error(CurveOperation2::Fillet, cause)
                        })? {
                        Classification::Decided(
                            crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2::Contacts(
                                contacts,
                            ),
                        ) => contacts,
                        Classification::Decided(
                            crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2::NoContacts,
                        ) => Vec::new(),
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::RationalBezier,
                                reason,
                            ));
                        }
                    };
                    let mut selected = None;
                    for contact in contacts {
                        let parameter_order = contact
                            .chord_parameter
                            .cmp_by_refinement(&expected_parameter, policy)
                            .map_err(|cause| {
                                curve_region_edit_error(CurveOperation2::Fillet, cause)
                            })?;
                        let same_point = policy.strict_predicate_pass(|| {
                            contact.point.same_point(&chord_cut.point, policy)
                        });
                        match (parameter_order, same_point) {
                            (
                                Classification::Decided(std::cmp::Ordering::Equal),
                                _,
                            )
                            | (_, Classification::Decided(true)) => {
                                if selected.replace(contact.cusp_parameter).is_some() {
                                    return Err(curve_region_edit_error(
                                        CurveOperation2::Fillet,
                                        CurveError::Topology(
                                            "one circular/chord fillet cut mapped to multiple circle contacts"
                                                .into(),
                                        ),
                                    ));
                                }
                                chord_cut.parameter = CurveParameter2::from_algebraic_chord(
                                    contact.chord_parameter,
                                );
                            }
                            (Classification::Decided(_), Classification::Decided(false)) => {}
                            (Classification::Uncertain(reason), _)
                            | (_, Classification::Uncertain(reason)) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    CurveFamily2::RationalBezier,
                                    reason,
                                ));
                            }
                        }
                    }
                    Ok(match selected {
                        Some(parameter) => Classification::Decided(Some(parameter)),
                        None => {
                            return Err(curve_region_edit_error(
                                CurveOperation2::Fillet,
                                CurveError::Topology(
                                    "the authored circular/chord fillet cut was absent from the selected circle"
                                        .into(),
                                ),
                            ));
                        }
                    })
                }
            }
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
            match parameter {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => {
                    *candidate_valid = false;
                    return Ok(Vec::new());
                }
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-fillet-chord-parameter",
                        match reason {
                            UncertaintyReason::Unsupported => "unsupported",
                            UncertaintyReason::Predicate => "predicate",
                            UncertaintyReason::Ordering => "ordering",
                            UncertaintyReason::RealSign => "real-sign",
                            UncertaintyReason::Boundary => "boundary",
                        },
                    );
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            }
        };
        Self::publish_retained_circle_fillet(
            frame.anchor_is_previous,
            fillet,
            terminal_circle,
            fillet_parameter,
            sweep_halves == 2,
            None,
            anchor_cut,
            chord_cut,
            policy,
        )
    }

    #[allow(clippy::too_many_arguments)]
    /// Reconstructs the selected analytic/circular carrier-switch fillet from
    /// the same mapped contact that solved its center.
    ///
    /// The analytic anchor supplies the fillet's selected-normal frame. The
    /// trimmed circular companion keeps the original two-normal contact map at
    /// its endpoint, so angular ordering is replayed without invoking the
    /// general two-circle resultant or materializing either selected point.
    fn retained_parallel_cusp_fillet_fragments(
        frame: &RetainedFilletFrame2,
        fillet: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        companion: &crate::BezierAlgebraicCuspSemicircleFragment2,
        companion_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
        companion_at_start: bool,
        anchor_parallel: &BezierParallel2,
        anchor_parameter: &CurveParameter2,
        source_direction: RealSign,
        companion_radial_sign: RealSign,
        fillet_clockwise: bool,
        anchor_cut: &mut CornerTrimCut2,
        companion_cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        let (tangent_cross, tangent_dot) = match frame.anchor_evidence.as_ref() {
            Some(relation) => match (relation.cross, relation.dot) {
                (Some(cross), dot) => (cross, dot.unwrap_or(RealSign::Zero)),
                (None, Some(dot)) => (RealSign::Zero, dot),
                (None, None) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        UncertaintyReason::Predicate,
                    ));
                }
            },
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    UncertaintyReason::Predicate,
                ));
            }
        };
        let (sweep_halves, tangent_cross, _) = Self::retained_fillet_sweep_from_tangent_relation(
            tangent_cross,
            tangent_dot,
            fillet_clockwise,
        )?;
        let terminal_circle = if sweep_halves == 2 {
            fillet.complementary_half()
        } else {
            fillet.clone()
        };
        let contact_parameter = if tangent_cross == RealSign::Zero {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        } else {
            let complementary_companion;
            let zero =
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero());
            let one =
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one());
            let companion_is_complement = companion_cut.parameter.is_algebraic_cusp_complement();
            let (companion_circle, companion_start, companion_end) = if companion_is_complement {
                complementary_companion = companion.semicircle().complementary_half();
                (&complementary_companion, &zero, &one)
            } else {
                (
                    companion.semicircle(),
                    companion.start_parameter(),
                    companion.end_parameter(),
                )
            };
            let (start, end) = match (companion_at_start, companion.is_reversed()) {
                (true, false) | (false, true) => {
                    (companion_parameter.clone(), companion_end.clone())
                }
                (true, true) | (false, false) => {
                    (companion_start.clone(), companion_parameter.clone())
                }
            };
            let trimmed_companion =
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    companion_circle.clone(),
                    start,
                    end,
                    companion.is_reversed(),
                    policy,
                );
            let terminal_radial_sign = match real_sign(terminal_circle.radial_distance(), policy) {
                Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Some(RealSign::Zero) => {
                    return Err(curve_region_edit_error(
                        CurveOperation2::Fillet,
                        CurveError::Topology(
                            "a retained carrier-switch fillet terminal circle collapsed".into(),
                        ),
                    ));
                }
                None => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        UncertaintyReason::RealSign,
                    ));
                }
            };
            let radial_product_sign = exact_sign_product(
                exact_sign_product(terminal_radial_sign, source_direction),
                companion_radial_sign,
            );
            match terminal_circle
                .certified_selected_circular_tangent_contact_parameter(
                    trimmed_companion,
                    companion_at_start,
                    anchor_parallel.clone(),
                    anchor_parameter.clone(),
                    source_direction,
                    radial_product_sign,
                    companion_cut.point.clone(),
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            }
        };
        Self::publish_retained_circle_fillet(
            frame.anchor_is_previous,
            fillet,
            terminal_circle,
            contact_parameter,
            sweep_halves == 2,
            None,
            anchor_cut,
            companion_cut,
            policy,
        )
    }

    /// Publishes a fillet whose center is a genuinely two-field selected
    /// circle-pair contact. The start radial is retained by the fillet circle
    /// frame and the terminal angular parameter reuses that same pair map;
    /// neither endpoint requires a reconstructed Cartesian field.
    fn retained_pair_cusp_fillet_fragments(
        frame: &RetainedFilletFrame2,
        fillet: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        companion: &crate::BezierAlgebraicCuspSemicircleFragment2,
        fillet_clockwise: bool,
        anchor_cut: &mut CornerTrimCut2,
        companion_cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        let relation = frame.anchor_evidence.as_ref().ok_or_else(|| {
            ExactCurveError::blocked(
                CurveOperation2::Fillet,
                CurveFamily2::CircularArc,
                UncertaintyReason::Predicate,
            )
        })?;
        let mut tangent_cross = relation.cross.ok_or_else(|| {
            ExactCurveError::blocked(
                CurveOperation2::Fillet,
                CurveFamily2::CircularArc,
                UncertaintyReason::Predicate,
            )
        })?;
        let tangent_dot = match (tangent_cross, relation.dot) {
            (_, Some(dot)) => dot,
            (RealSign::Positive | RealSign::Negative, None) => RealSign::Zero,
            (RealSign::Zero, None) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    UncertaintyReason::Predicate,
                ));
            }
        };
        if !frame.anchor_is_previous {
            tangent_cross = exact_sign_reverse(tangent_cross);
        }
        let (sweep_halves, tangent_cross, _) = Self::retained_fillet_sweep_from_tangent_relation(
            tangent_cross,
            tangent_dot,
            fillet_clockwise,
        )?;
        let terminal_circle = if sweep_halves == 2 {
            fillet.complementary_half()
        } else {
            fillet.clone()
        };
        let complementary_companion;
        let companion_circle = if companion_cut.parameter.is_algebraic_cusp_complement() {
            complementary_companion = companion.semicircle().complementary_half();
            &complementary_companion
        } else {
            companion.semicircle()
        };
        let contact_parameter = if tangent_cross == RealSign::Zero {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        } else {
            match terminal_circle
                .certified_selected_pair_contact_parameter(companion_circle, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => {
                    // The retained pair contact belongs to the other rational
                    // half-chart. Fall back to the exact full-circle selector,
                    // which owns complementary-half and diameter assignment.
                    return Self::retained_concentric_cusp_fillet_fragments(
                        frame,
                        fillet,
                        companion,
                        anchor_cut,
                        companion_cut,
                        policy,
                    );
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            }
        };
        Self::publish_retained_circle_fillet(
            frame.anchor_is_previous,
            fillet,
            terminal_circle,
            contact_parameter,
            sweep_halves == 2,
            None,
            anchor_cut,
            companion_cut,
            policy,
        )
    }

    /// Publishes a fillet anchored on a represented circular carrier and
    /// terminating on a selected circle. Both half charts are queried, with
    /// deterministic diameter ownership, so major sweeps and complementary
    /// source-circle extension use the same exact circle-pair authority.
    fn retained_concentric_cusp_fillet_fragments(
        frame: &RetainedFilletFrame2,
        fillet: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        companion: &crate::BezierAlgebraicCuspSemicircleFragment2,
        anchor_cut: &mut CornerTrimCut2,
        companion_cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        let complementary_companion;
        let companion_is_complement = companion_cut.parameter.is_algebraic_cusp_complement();
        let companion_circle = if companion_is_complement {
            complementary_companion = companion.semicircle().complementary_half();
            &complementary_companion
        } else {
            companion.semicircle()
        };
        if companion_cut.parameter.as_algebraic_cusp().is_none() {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                CurveFamily2::CircularArc,
                UncertaintyReason::Unsupported,
            ));
        }
        let fillet_halves = [fillet.clone(), fillet.complementary_half()];
        let endpoint = |location| match location {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero())
            }
            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => {
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
            }
            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                unreachable!("endpoint-only pair contacts are not interior")
            }
        };
        let mut selected = None;
        for (fillet_half, circle) in fillet_halves.iter().enumerate() {
            let intersections = match circle
                .pair_intersections(companion_circle, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            };
            let mut contacts = Vec::new();
            match intersections {
                crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                    contacts: pair_contacts,
                    parameter_map,
                } => {
                    contacts.reserve(pair_contacts.len());
                    for contact in pair_contacts {
                        contacts.push((
                            parameter_map.first_contact_parameter(&contact),
                            parameter_map.second_contact_parameter(&contact),
                            contact.first_location,
                        ));
                    }
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(
                    pair_contacts,
                ) => {
                    contacts.reserve(pair_contacts.len());
                    for contact in pair_contacts {
                        contacts.push((
                            endpoint(contact.first_location),
                            endpoint(contact.second_location),
                            contact.first_location,
                        ));
                    }
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts => {}
                crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(_) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        UncertaintyReason::Boundary,
                    ));
                }
            }
            for (fillet_parameter, companion_parameter, fillet_location) in contacts {
                // The base half owns the non-anchor diameter point; the
                // complement owns neither shared endpoint.
                if fillet_location
                    == crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start
                    || (fillet_half == 1
                        && fillet_location
                            == crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End)
                {
                    continue;
                }
                if selected.is_some() {
                    return Err(curve_region_edit_error(
                        CurveOperation2::Fillet,
                        CurveError::Topology(
                            "one retained circle tangency occupied multiple full-circle cells"
                                .into(),
                        ),
                    ));
                }
                selected = Some((fillet_half, fillet_parameter, companion_parameter));
            }
        }
        let Some((fillet_half, fillet_parameter, companion_parameter)) = selected else {
            return Err(curve_region_edit_error(
                CurveOperation2::Fillet,
                CurveError::Topology(
                    "a retained fillet lost its certified selected-circle tangency".into(),
                ),
            ));
        };
        companion_cut.parameter = if companion_is_complement {
            CurveParameter2::from_algebraic_cusp_complement(companion_parameter)
        } else {
            CurveParameter2::from_algebraic_cusp(companion_parameter)
        };
        Self::publish_retained_circle_fillet(
            frame.anchor_is_previous,
            fillet,
            fillet_halves[fillet_half].clone(),
            fillet_parameter,
            fillet_half != 0,
            None,
            anchor_cut,
            companion_cut,
            policy,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn retained_fillet_fragments(
        previous_fragments: &[BezierSplitFragment2],
        next_fragments: &[BezierSplitFragment2],
        previous_cut: &mut CornerTrimCut2,
        next_cut: &mut CornerTrimCut2,
        center: CurvePoint2,
        clockwise: bool,
        retained_frame: Option<RetainedFilletFrame2>,
        radius: &Real,
        allow_boundary_contact: bool,
        previous_promoted_parallel: Option<&crate::BezierParallelFragment2>,
        next_promoted_parallel: Option<&crate::BezierParallelFragment2>,
        previous_replacement: &mut Option<Vec<BezierSplitFragment2>>,
        next_replacement: &mut Option<Vec<BezierSplitFragment2>>,
        candidate_valid: &mut bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        let previous_fragment = previous_fragments
            .last()
            .expect("a nonempty previous source");
        let next_fragment = next_fragments.first().expect("a nonempty next source");
        // Prefer the ordinary exact-Real arc authority whenever every retained
        // point already has a STRICT standalone witness.  This is not an
        // approximation or a field flattening: algebraic images enter only
        // when both Cartesian coordinates reduce exactly to the canonical
        // scalar.  Genuinely selected or correlated points continue through
        // the procedural circle publisher below.
        let exact_point = |point: &CurvePoint2| match point {
            CurvePoint2(CurvePointData2::Exact(point)) => Some(point.clone()),
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                point.exact_point(&CurveContext::STRICT)
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        let represented_previous = exact_point(&previous_cut.point);
        let represented_next = exact_point(&next_cut.point);
        let represented_center = exact_point(&center);
        let deferred_arc_contact = retained_frame
            .as_ref()
            .and_then(|frame| frame.anchor_evidence.as_ref())
            .and_then(|evidence| evidence.deferred_arc_contact.as_ref());
        if let (Some(deferred), Some(previous_point), Some(next_point), Some(center)) = (
            deferred_arc_contact,
            represented_previous.as_ref(),
            represented_next.as_ref(),
            represented_center.as_ref(),
        ) {
            let arc_contact = if deferred.arc_is_previous {
                previous_point
            } else {
                next_point
            };
            let fillet = CircularArc2::new_with_certified_radius(
                previous_point.clone(),
                next_point.clone(),
                center.clone(),
                radius * radius,
                clockwise,
                None,
            );
            if let Some(arc) = deferred.source.retained_rational_arc() {
                let (cut, source, replacement) = if deferred.arc_is_previous {
                    (
                        &*previous_cut,
                        previous_fragments,
                        &mut *previous_replacement,
                    )
                } else {
                    (&*next_cut, next_fragments, &mut *next_replacement)
                };
                if cut.placement == CornerPlacement2::Extension {
                    *replacement = Some(Self::retained_arc_extension_fragments(
                        source,
                        arc,
                        cut,
                        None,
                        deferred.arc_is_previous,
                        CurveOperation2::Fillet,
                        policy,
                    )?);
                }
                return Self::materialized_corner_arc_fragments(
                    &fillet,
                    CurveOperation2::Fillet,
                    policy,
                );
            }
            let Some(cut) = crate::curve::arc_fillet_cut_from_incident_point(
                &deferred.source,
                arc_contact.clone(),
                true,
                deferred.arc_is_previous,
                deferred.domain,
                CurveFamily2::CircularArc,
                policy,
            )?
            else {
                *candidate_valid = false;
                return Ok(Vec::new());
            };
            let cut = cut
                .into_retained_evidence()
                .expect("a native deferred arc cut retains its endpoint parameter marker");
            let arc_cut = if deferred.arc_is_previous {
                &mut *previous_cut
            } else {
                &mut *next_cut
            };
            arc_cut.point = cut.point;
            arc_cut.placement = cut.placement;
            let retained_arc = CircularArc2::new_with_certified_radius(
                if deferred.arc_is_previous {
                    deferred.source.support().start().clone()
                } else {
                    arc_contact.clone()
                },
                if deferred.arc_is_previous {
                    arc_contact.clone()
                } else {
                    deferred.source.support().end().clone()
                },
                deferred.source.support().center().clone(),
                deferred.source.support().radius_squared(),
                deferred.source.support().is_clockwise(),
                None,
            );
            let replacement = Some(Self::materialized_corner_arc_fragments(
                &retained_arc,
                CurveOperation2::Fillet,
                policy,
            )?);
            if deferred.arc_is_previous {
                *previous_replacement = replacement;
            } else {
                *next_replacement = replacement;
            }
            return Self::materialized_corner_arc_fragments(
                &fillet,
                CurveOperation2::Fillet,
                policy,
            );
        }
        let has_deferred_arc_contact = deferred_arc_contact.is_some();
        if !has_deferred_arc_contact
            && let (Some(previous_point), Some(next_point), Some(center)) = (
                represented_previous.as_ref(),
                represented_next.as_ref(),
                represented_center.as_ref(),
            )
        {
            let arc = CircularArc2::new_with_certified_radius(
                previous_point.clone(),
                next_point.clone(),
                center.clone(),
                radius * radius,
                clockwise,
                None,
            );
            return Self::materialized_corner_arc_fragments(&arc, CurveOperation2::Fillet, policy);
        }

        {
            let frame = retained_frame.ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
            let (anchor_cut, other_cut, other_fragment, other_promoted_parallel) =
                if frame.anchor_is_previous {
                    (
                        &mut *previous_cut,
                        &mut *next_cut,
                        next_fragment,
                        next_promoted_parallel,
                    )
                } else {
                    (
                        &mut *next_cut,
                        &mut *previous_cut,
                        previous_fragment,
                        previous_promoted_parallel,
                    )
                };
            let canonical_anchor_tangent = if matches!(
                &frame.radial_frame,
                RetainedFilletRadialFrame2::ParallelNormal { .. }
            ) {
                anchor_cut
                    .replacement_parallel_fragment()
                    .zip(anchor_cut.parameter.as_bezier_parameter())
                    .zip(anchor_cut.replacement_parallel_source_parameter_map())
                    .map(
                        |((replacement, parameter), (source_scale, source_offset))| {
                            (
                                replacement.parallel().clone(),
                                parameter.clone(),
                                source_scale.clone(),
                                source_offset.clone(),
                            )
                        },
                    )
            } else {
                None
            };
            if let Some(replacement) = frame
                .anchor_evidence
                .as_ref()
                .and_then(|relation| relation.canonical_anchor_curve.clone())
            {
                anchor_cut.replacement = Some(CornerReplacement2::Curve(
                    BezierSubcurve2::Rational(replacement),
                ));
            }
            if !matches!(
                other_fragment,
                BezierSplitFragment2::RetainedBezier { .. }
                    | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    | BezierSplitFragment2::AlgebraicChord(_)
                    | BezierSplitFragment2::AnalyticParallel(_)
                    | BezierSplitFragment2::SelectedFiber(_)
                    | BezierSplitFragment2::Materialized { .. }
            ) {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                ));
            }
            let fillet_clockwise = if frame.anchor_is_previous {
                clockwise
            } else {
                !clockwise
            };
            let fillet = match match &frame.radial_frame {
                RetainedFilletRadialFrame2::RepresentedUnitNormal(unit_normal) => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_certified_unit_normal(
                        &center,
                        unit_normal.clone(),
                        frame.radial_distance.clone(),
                        fillet_clockwise,
                        policy,
                    )
                }
                RetainedFilletRadialFrame2::ChordNormal {
                    anchor,
                    policy: frame_policy,
                } => {
                    if frame_policy != policy {
                        return Err(curve_region_edit_error(
                            CurveOperation2::Fillet,
                            CurveError::Topology(
                                "a chord-normal fillet frame crossed predicate policies".into(),
                            ),
                        ));
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
                        center.clone(),
                        anchor.clone(),
                        frame.radial_distance.clone(),
                        fillet_clockwise,
                        policy,
                    )
                }
                RetainedFilletRadialFrame2::ConcentricArc {
                    support_center,
                    normal_denominator,
                } => crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_certified_concentric_normal(
                    &center,
                    support_center,
                    normal_denominator.clone(),
                    frame.radial_distance.clone(),
                    fillet_clockwise,
                    policy,
                ),
                RetainedFilletRadialFrame2::SelectedConcentric {
                    support,
                    center_parameter,
                    normal_denominator,
                } => crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_circle_radial(
                    support,
                    center_parameter.clone(),
                    normal_denominator.clone(),
                    frame.radial_distance.clone(),
                    fillet_clockwise,
                    policy,
                ),
                RetainedFilletRadialFrame2::ParallelNormal {
                    center_support,
                    center_parameter,
                    policy: frame_policy,
                } => {
                    if frame_policy != policy {
                        return Err(curve_region_edit_error(
                            CurveOperation2::Fillet,
                            CurveError::Topology(
                                "a parallel-normal fillet frame crossed predicate policies".into(),
                            ),
                        ));
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                        center_support.clone(),
                        center_parameter.clone(),
                        frame.radial_distance.clone(),
                        fillet_clockwise,
                        policy,
                    )
                }
            }
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))? {
                Classification::Decided(Some(fillet)) => fillet,
                Classification::Decided(None) => {
                    return Err(curve_region_edit_error(
                        CurveOperation2::Fillet,
                        CurveError::Topology("a positive-radius retained fillet collapsed".into()),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            };
            let fillet = if let Some((support, parameter, source_scale, source_offset)) =
                canonical_anchor_tangent
            {
                fillet
                    .with_certified_parallel_normal_tangent_authority(
                        support,
                        parameter,
                        source_scale,
                        source_offset,
                        policy,
                    )
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            } else {
                fillet
            };
            let mut preselected_arc_contact = None;
            if let Some(deferred) = frame
                .anchor_evidence
                .as_ref()
                .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
            {
                if let Some(seed) = &deferred.contact_seed {
                    if frame.anchor_is_previous != deferred.arc_is_previous {
                        return Err(curve_region_edit_error(
                            CurveOperation2::Fillet,
                            CurveError::Topology(
                                "a preselected circular fillet contact lost its arc anchor".into(),
                            ),
                        ));
                    }
                    let replay = Self::retained_preselected_arc_fillet_contact(
                        if frame.anchor_is_previous {
                            previous_fragments
                        } else {
                            next_fragments
                        },
                        deferred,
                        seed,
                        anchor_cut,
                        policy,
                    )?;
                    let RetainedPreselectedArcFilletContact2 {
                        replacement,
                        source_parallel,
                        source_parameter,
                        source_direction,
                    } = replay;
                    if deferred.arc_is_previous {
                        *previous_replacement = replacement;
                    } else {
                        *next_replacement = replacement;
                    }
                    preselected_arc_contact =
                        Some((source_parallel, source_parameter, source_direction));
                } else {
                    if frame.anchor_is_previous == deferred.arc_is_previous {
                        return Err(curve_region_edit_error(
                            CurveOperation2::Fillet,
                            CurveError::Topology(
                                "a deferred circular fillet contact was assigned to its anchor carrier"
                                    .into(),
                            ),
                        ));
                    }
                    let Some(result) = Self::retained_deferred_arc_fillet_fragments(
                        &frame,
                        fillet,
                        &center,
                        if frame.anchor_is_previous {
                            next_fragments
                        } else {
                            previous_fragments
                        },
                        deferred,
                        anchor_cut,
                        other_cut,
                        policy,
                    )?
                    else {
                        *candidate_valid = false;
                        return Ok(Vec::new());
                    };
                    if deferred.arc_is_previous {
                        *previous_replacement = result.arc_replacement;
                    } else {
                        *next_replacement = result.arc_replacement;
                    }
                    return Ok(result.fillet_fragments);
                }
            }
            if !matches!(
                &frame.radial_frame,
                RetainedFilletRadialFrame2::SelectedConcentric { .. }
            ) && let (
                Some((source_parallel, source_parameter, source_direction)),
                BezierSplitFragment2::AlgebraicCuspSemicircle(companion),
                Some(companion_parameter),
            ) = (
                preselected_arc_contact.as_ref(),
                other_fragment,
                other_cut.parameter.as_algebraic_cusp().cloned(),
            ) {
                // The center solve already retained the exact selected-circle
                // endpoint and its rational arc tangent. Publish that same
                // correlation directly; a second general circle-pair solve
                // would enlarge the field and can lose a major-cell root.
                let companion_radial_sign = if clockwise {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                };
                return Self::retained_parallel_cusp_fillet_fragments(
                    &frame,
                    fillet,
                    companion,
                    companion_parameter,
                    frame.anchor_is_previous,
                    source_parallel,
                    source_parameter,
                    *source_direction,
                    companion_radial_sign,
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    policy,
                );
            }
            if let (
                BezierSplitFragment2::AlgebraicCuspSemicircle(companion),
                RetainedFilletRadialFrame2::SelectedConcentric { .. },
            ) = (other_fragment, &frame.radial_frame)
            {
                return Self::retained_pair_cusp_fillet_fragments(
                    &frame,
                    fillet,
                    companion,
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    policy,
                );
            }
            if let (
                BezierSplitFragment2::AlgebraicCuspSemicircle(companion),
                Some(source_direction),
                Some(companion_parameter),
            ) = (
                other_fragment,
                frame
                    .anchor_evidence
                    .as_ref()
                    .and_then(|evidence| evidence.source_direction),
                other_cut.parameter.as_algebraic_cusp().cloned(),
            ) && let Some((center_support, center_parameter)) = match &frame.radial_frame {
                RetainedFilletRadialFrame2::ParallelNormal {
                    center_support,
                    center_parameter,
                    ..
                } => Some((
                    center_support.clone(),
                    CurveParameter2::from(center_parameter.clone()),
                )),
                RetainedFilletRadialFrame2::ChordNormal { .. } => frame
                    .anchor_evidence
                    .as_ref()
                    .and_then(|evidence| evidence.center_parallel.as_ref())
                    .and_then(|center| {
                        center
                            .parameter
                            .as_ref()
                            .map(|parameter| (center.support.clone(), parameter.clone()))
                    }),
                RetainedFilletRadialFrame2::RepresentedUnitNormal(_)
                | RetainedFilletRadialFrame2::ConcentricArc { .. }
                | RetainedFilletRadialFrame2::SelectedConcentric { .. } => None,
            } {
                // The solved offset center lies on the opposite side of the
                // circular source from its tangency point.
                let companion_radial_sign = if clockwise {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                };
                return Self::retained_parallel_cusp_fillet_fragments(
                    &frame,
                    fillet,
                    companion,
                    companion_parameter,
                    frame.anchor_is_previous,
                    &center_support,
                    &center_parameter,
                    source_direction,
                    companion_radial_sign,
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    policy,
                );
            }
            if let BezierSplitFragment2::AlgebraicChord(chord) = other_fragment {
                let chord = other_cut
                    .parameter
                    .as_algebraic_chord()
                    .map_or_else(|| chord.clone(), |parameter| parameter.chord().clone());
                return Self::retained_chord_fillet_fragments(
                    &frame,
                    fillet,
                    &chord,
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    candidate_valid,
                    policy,
                );
            }
            if matches!(other_fragment, BezierSplitFragment2::RetainedBezier { .. })
                && let Some(parameter) = other_cut.parameter.as_algebraic_chord()
            {
                let chord = parameter.chord().clone();
                return Self::retained_chord_fillet_fragments(
                    &frame,
                    fillet,
                    &chord,
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    candidate_valid,
                    policy,
                );
            }
            if let BezierSplitFragment2::Materialized {
                curve: BezierSubcurve2::Quadratic(line),
                ..
            } = other_fragment
                && line.retained_exact_line_image().is_some()
                && (other_cut.parameter.as_algebraic_chord().is_some()
                    || other_cut.parameter.is_retained_scalar())
            {
                let support = if let Some(parameter) = other_cut.parameter.as_algebraic_chord() {
                    parameter.chord().clone()
                } else {
                    retained_algebraic_line_support(
                        line.retained_exact_line_image()
                            .expect("the retained fillet companion is an exact line"),
                        CurveOperation2::Fillet,
                        policy,
                    )?
                };
                return Self::retained_chord_fillet_fragments(
                    &frame,
                    fillet,
                    &support,
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    candidate_valid,
                    policy,
                );
            }
            if let BezierSplitFragment2::SelectedFiber(other_fragment) = other_fragment
                && let Some(expected_parameter) = other_cut.parameter.as_bezier_parameter().cloned()
            {
                let expected = CurveParameter2::from(expected_parameter.clone());
                let compare = |boundary: &CurveParameter2| {
                    retained_corner_decision(
                        policy
                            .strict_predicate_pass(|| expected.cmp_by_refinement(boundary, policy))
                            .map_err(|cause| {
                                curve_region_edit_error(CurveOperation2::Fillet, cause)
                            })?,
                        CurveOperation2::Fillet,
                    )
                };
                let start_order = compare(other_fragment.range().start())?;
                let end_order = compare(other_fragment.range().end())?;
                let admissible = match other_cut.placement {
                    CornerPlacement2::Trim => start_order.is_gt() && end_order.is_lt(),
                    CornerPlacement2::Corner => {
                        (start_order.is_eq() || start_order.is_gt())
                            && (end_order.is_eq() || end_order.is_lt())
                    }
                    CornerPlacement2::Extension => {
                        retained_selected_corner_parameter_is_in_native_chart(
                            &expected,
                            CurveOperation2::Fillet,
                            policy,
                        )?
                    }
                };
                if !admissible {
                    return Err(curve_region_edit_error(
                        CurveOperation2::Fillet,
                        CurveError::Topology(
                            "a selected fillet companion lost its certified cut parameter".into(),
                        ),
                    ));
                }
                return Self::retained_parallel_fillet_fragments(
                    &frame,
                    fillet,
                    &other_fragment.parallel_carrier(),
                    expected_parameter,
                    other_fragment.is_reversed(),
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    policy,
                );
            }
            let other_parallel_fragment = match other_fragment {
                BezierSplitFragment2::AnalyticParallel(fragment) => Some(fragment),
                BezierSplitFragment2::RetainedBezier { .. }
                | BezierSplitFragment2::SelectedFiber(_) => {
                    Some(other_promoted_parallel.ok_or_else(|| {
                        ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::RationalBezier,
                            UncertaintyReason::Unsupported,
                        )
                    })?)
                }
                _ => None,
            };
            if let Some(other_fragment) = other_parallel_fragment {
                let expected_parameter = other_cut
                    .parameter
                    .as_bezier_parameter()
                    .cloned()
                    .ok_or_else(|| {
                        ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::RationalBezier,
                            UncertaintyReason::Unsupported,
                        )
                    })?;
                match crate::bezier_offset::overlap_parameter_is_in_range(
                    &expected_parameter,
                    other_fragment.range(),
                    allow_boundary_contact,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
                {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Err(curve_region_edit_error(
                            CurveOperation2::Fillet,
                            CurveError::Topology(
                                "a retained parallel fillet lost its interior trim parameter"
                                    .into(),
                            ),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::RationalBezier,
                            reason,
                        ));
                    }
                }
                return Self::retained_parallel_fillet_fragments(
                    &frame,
                    fillet,
                    other_fragment.parallel(),
                    expected_parameter,
                    other_fragment.is_reversed(),
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    policy,
                );
            }
            if let BezierSplitFragment2::AlgebraicCuspSemicircle(companion) = other_fragment {
                return Self::retained_concentric_cusp_fillet_fragments(
                    &frame, fillet, companion, anchor_cut, other_cut, policy,
                );
            }
            if let BezierSplitFragment2::Materialized { curve, .. } = other_fragment {
                let expected_parameter = other_cut
                    .parameter
                    .as_bezier_parameter()
                    .cloned()
                    .ok_or_else(|| {
                        ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::RationalBezier,
                            UncertaintyReason::Unsupported,
                        )
                    })?;
                let rational = RationalBezier2::try_from_subcurve(curve)
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
                let source_parallel = rational
                    .parallel_left(Real::zero())
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
                return Self::retained_parallel_fillet_fragments(
                    &frame,
                    fillet,
                    &source_parallel,
                    expected_parameter,
                    false,
                    fillet_clockwise,
                    anchor_cut,
                    other_cut,
                    policy,
                );
            }
            unreachable!("the retained fillet companion family was checked")
        }
    }

    pub(super) fn canonicalize_retained_single_fragment_extension_cuts(
        fragment: &BezierSplitFragment2,
        previous_cut: &mut CornerTrimCut2,
        next_cut: &mut CornerTrimCut2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<()> {
        if previous_cut.replacement.is_some() || next_cut.replacement.is_some() {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "a one-fragment extension arrived with an independent replacement carrier"
                        .into(),
                ),
            ));
        }
        if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. })
            && previous_cut.parameter.as_algebraic_chord().is_some()
            && next_cut.parameter.as_algebraic_chord().is_some()
        {
            return Ok(());
        }

        if matches!(fragment, BezierSplitFragment2::SelectedFiber(_))
            && retained_selected_corner_parameter_is_in_native_chart(
                &previous_cut.parameter,
                operation,
                policy,
            )?
            && retained_selected_corner_parameter_is_in_native_chart(
                &next_cut.parameter,
                operation,
                policy,
            )?
        {
            // Both cuts already name one exact interval in the selected
            // source chart. The shared rebuilder can retain that interval
            // directly; a finite-envelope carrier switch would only enlarge
            // the algebraic representation.
            return Ok(());
        }

        let promoted;
        let carrier = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                RetainedCornerExtensionCarrier2::Curve(curve)
            }
            BezierSplitFragment2::RetainedBezier { .. } => {
                promoted = promoted_endpoint_image_corner_fragment(fragment, operation)?;
                RetainedCornerExtensionCarrier2::AnalyticParallel(&promoted)
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => {
                RetainedCornerExtensionCarrier2::AnalyticParallel(fragment)
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                RetainedCornerExtensionCarrier2::SelectedFiber(fragment)
            }
            BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                ));
            }
        };
        canonicalize_retained_extension_on_finite_envelope(
            carrier,
            previous_cut,
            next_cut,
            operation,
            policy,
        )
    }

    pub(super) fn canonicalize_retained_corner_cut(
        fragment: &BezierSplitFragment2,
        cut: &mut CornerTrimCut2,
        previous: bool,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<()> {
        if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            if cut
                .parameter
                .as_algebraic_chord()
                .is_some_and(|parameter| parameter.chord() == chord)
            {
                // The corner solver already transported this cut from the
                // offset carrier onto the authored chord and classified its
                // finite-domain placement. Keep that structural parameter;
                // rebuilding it from the multi-field Cartesian point repeats
                // the same incidence proof and can require an unnecessary
                // compositum merely to rediscover the retained identity.
                return Ok(());
            }
            cut.parameter = if cut.placement == CornerPlacement2::Extension {
                CurveParameter2::from_algebraic_chord(
                    chord
                        .parameter_at_certified_support_point(cut.point.clone(), policy)
                        .map_err(|cause| curve_region_edit_error(operation, cause))?,
                )
            } else {
                match chord
                    .parameter_at_certified_point(cut.point.clone(), policy)
                    .map_err(|cause| curve_region_edit_error(operation, cause))?
                {
                    Classification::Decided(Some(parameter)) => {
                        CurveParameter2::from_algebraic_chord(parameter)
                    }
                    Classification::Decided(None) => {
                        return Err(curve_region_edit_error(
                            operation,
                            CurveError::Topology(
                                "an exact retained corner cut lay outside its chord".into(),
                            ),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            operation,
                            CurveFamily2::RationalBezier,
                            reason,
                        ));
                    }
                }
            };
            return Ok(());
        }
        if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. })
            && cut.parameter.as_algebraic_chord().is_some()
        {
            return Ok(());
        }
        if cut.placement != CornerPlacement2::Extension {
            return Ok(());
        }

        if matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
            && cut.parameter.is_algebraic_cusp()
        {
            // The retained parameter already names either the authored half
            // or its complementary chart. Reconstruction expands that exact
            // chart path into the required half-circle fragment sequence.
            return Ok(());
        }

        if cut.parameter.as_algebraic_chord().is_some()
            && matches!(
                fragment,
                BezierSplitFragment2::Materialized {
                    curve: BezierSubcurve2::Quadratic(curve),
                    ..
                } if curve.retained_exact_line_image().is_some()
            )
        {
            return Ok(());
        }

        if matches!(fragment, BezierSplitFragment2::SelectedFiber(_))
            && retained_selected_corner_parameter_is_in_native_chart(
                &cut.parameter,
                operation,
                policy,
            )?
        {
            // The original source chart already contains this extension.
            // Reconstruction can enlarge the compact selected range directly;
            // no finite-envelope reparameterization is needed.
            return Ok(());
        }

        let promoted;
        let carrier = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                RetainedCornerExtensionCarrier2::Curve(curve)
            }
            BezierSplitFragment2::RetainedBezier { .. } => {
                promoted = promoted_endpoint_image_corner_fragment(fragment, operation)?;
                RetainedCornerExtensionCarrier2::AnalyticParallel(&promoted)
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => {
                RetainedCornerExtensionCarrier2::AnalyticParallel(fragment)
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                RetainedCornerExtensionCarrier2::SelectedFiber(fragment)
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::AlgebraicChord(_) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                ));
            }
        };
        let mut retained_endpoint = CornerTrimCut2 {
            parameter: carrier.retained_endpoint(previous),
            point: cut.point.clone(),
            placement: CornerPlacement2::Corner,
            replacement: None,
        };
        if previous {
            canonicalize_retained_extension_on_finite_envelope(
                carrier,
                cut,
                &mut retained_endpoint,
                operation,
                policy,
            )
        } else {
            canonicalize_retained_extension_on_finite_envelope(
                carrier,
                &mut retained_endpoint,
                cut,
                operation,
                policy,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn native(curve: QuadraticBezier2) -> BezierSplitFragment2 {
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(curve),
        }
    }

    fn line(start: Point2, end: Point2) -> BezierSplitFragment2 {
        native(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(start, end).unwrap(),
        ))
    }

    fn assert_endpoints(
        fragments: &[BezierSplitFragment2],
        start: &Point2,
        end: &Point2,
        policy: &CurveContext,
    ) {
        let first = Curve2::from_retained_fragment(fragments[0].clone());
        let last = Curve2::from_retained_fragment(fragments.last().unwrap().clone());
        for (actual, expected) in [(first.start(), start), (last.end(), end)] {
            let equality = actual.coincides_with(&CurvePoint2::from(expected.clone()), policy);
            assert_eq!(equality.certainty, crate::CurveCertainty::Certified);
            assert_eq!(equality.value, Classification::Decided(true));
        }
    }

    #[test]
    fn open_selected_chamfers_reenter_without_region_fill_semantics() {
        let start = Point2::from_values(-4, 0);
        let end = Point2::from_values(1, 2);
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = vec![
                line(start.clone(), Point2::from_values(0, 0)),
                native(QuadraticBezier2::new(
                    Point2::from_values(0, 0),
                    Point2::from_values(0, 1),
                    end.clone(),
                )),
            ];
            let outcome = resolve_certified_operation(&policy, |attempt| {
                CurveCornerChain2::new(&source, false).chamfer_vertex_by_setbacks(
                    1,
                    Real::one(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    attempt,
                )
            })
            .unwrap();
            assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(chamfered) = outcome.value else {
                panic!("the open algebraic setback has one exact solution")
            };
            assert_eq!(chamfered.len(), 3);
            assert!(chamfered.iter().any(|fragment| {
                Curve2::from_retained_fragment(fragment.clone())
                    .geometry()
                    .is_none()
            }));
            assert_endpoints(&chamfered, &start, &end, &policy);
            let quarter = (Real::one() / Real::from(4_u8)).unwrap();
            let repeated = resolve_certified_operation(&policy, |attempt| {
                CurveCornerChain2::new(&chamfered, false).chamfer_vertex_by_setbacks(
                    1,
                    quarter.clone(),
                    quarter,
                    CurveCornerMode2::TrimOnly,
                    attempt,
                )
            })
            .unwrap();
            assert_eq!(repeated.certainty, crate::CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(repeated) = repeated.value else {
                panic!("a retained open chamfer must accept another exact setback")
            };
            assert_eq!(repeated.len(), 4);
            assert_endpoints(&repeated, &start, &end, &policy);
        }
    }

    #[test]
    fn open_fillet_keeps_terminal_endpoints_and_exact_connectivity() {
        let start = Point2::from_values(0, 0);
        let end = Point2::from_values(4, 4);
        let source = vec![
            line(start.clone(), Point2::from_values(4, 0)),
            line(Point2::from_values(4, 0), end.clone()),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let outcome = resolve_certified_operation(&policy, |attempt| {
                CurveCornerChain2::new(&source, false).fillet_vertex_by_radius(
                    1,
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    attempt,
                )
            })
            .unwrap();
            assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(filleted) = outcome.value else {
                panic!("the open right-angle fillet has one exact solution")
            };
            assert_endpoints(&filleted, &start, &end, &policy);
            for pair in filleted.windows(2) {
                assert_eq!(
                    curve_fragment_endpoints_equal(&pair[0], false, &pair[1], true, &policy),
                    Classification::Decided(true),
                );
            }
        }
    }
}
