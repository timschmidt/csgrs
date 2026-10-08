from pathlib import Path
R=Path('/home/tim/Documents/GitHub/workspace/hypercurve/src')
files={name:(R/name).read_text() for name in ['curve.rs','bezier_region.rs','curve_corner_chain.rs']}
def one(s,old,new,count=1):
    assert s.count(old)==count,(old[:100],s.count(old),count)
    return s.replace(old,new)
def section(s,start,end,new):
    a=s.index(start); b=s.index(end,a+len(start))
    return s[:a]+new+s[b:]
s=files['curve.rs']
s=section(s,'#[derive(Clone, Debug, PartialEq)]\npub(crate) enum CornerReplacement2 {','#[derive(Clone, Debug)]\npub(crate) struct ChamferCorner2 {','''#[derive(Clone, Debug)]
pub(crate) struct CornerTrimCut2 {
    pub(crate) parameter: CurveParameter2,
    pub(crate) point: CurvePoint2,
    pub(crate) placement: CornerPlacement2,
    /// The exact replacement span, in the same chart as this cut.
    pub(crate) replacement: Option<Arc<crate::BezierSplitFragment2>>,
}

impl CornerTrimCut2 {
    pub(crate) fn replacement_curve(&self) -> Option<&BezierSubcurve2> {
        match self.replacement.as_deref()? {
            crate::BezierSplitFragment2::Materialized { curve, .. } => Some(curve),
            _ => None,
        }
    }
}

''')
files['curve.rs']=s

s=files['bezier_region.rs']
s=one(s,'CornerPlacement2, CornerReplacement2, CornerTrimCut2, RetainedFilletFrame2,','CornerPlacement2, CornerTrimCut2, RetainedFilletFrame2,')
s=one(s,'replacement: Option<&CornerReplacement2>,','replacement: Option<&BezierSplitFragment2>,')
s=one(s,'''        let curve = match replacement {
            CornerReplacement2::Curve(curve) => curve,
            CornerReplacement2::AnalyticParallel { fragment, .. } => {
                return Ok(vec![BezierSplitFragment2::AnalyticParallel(
                    fragment.clone(),
                )]);
            }
            CornerReplacement2::SelectedFiber { fragment, .. } => {
                return Ok(vec![BezierSplitFragment2::SelectedFiber(
                    fragment.as_ref().clone(),
                )]);
            }
        };''','''        let BezierSplitFragment2::Materialized { curve, .. } = replacement else {
            return Ok(vec![replacement.clone()]);
        };''')
s=one(s,'''        (Some(CornerReplacement2::SelectedFiber { fragment, .. }), _)
        | (_, Some(CornerReplacement2::SelectedFiber { fragment, .. })) => {
            return Ok(BezierSplitFragment2::SelectedFiber(
                fragment.as_ref().clone(),
            ));
        }
        (Some(CornerReplacement2::AnalyticParallel { fragment, .. }), _)
        | (_, Some(CornerReplacement2::AnalyticParallel { fragment, .. })) => {
            return Ok(BezierSplitFragment2::AnalyticParallel(fragment.clone()));
        }''','''        (Some(replacement), _) | (_, Some(replacement))
            if !matches!(replacement, BezierSplitFragment2::Materialized { .. }) =>
        {
            return Ok(replacement.clone());
        }''')
s=section(s,'#[derive(Clone, Copy)]\nenum RetainedCornerExtensionCarrier2', 'fn curve_region_boundary_loop_from_native_material_contour(', '''/// Keeps both cuts, their point witnesses and their original support chart.
/// The finite envelope certifies source finiteness; it never becomes a new
/// geometric carrier or a restriction on the exact cut representation.
fn retain_corner_extension_interval(
    fragment: &BezierSplitFragment2,
    previous_cut: &mut CornerTrimCut2,
    next_cut: &mut CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let reversed = fragment.source_is_reversed();
    let (lower, upper) = if reversed {
        (&*previous_cut, &*next_cut)
    } else {
        (&*next_cut, &*previous_cut)
    };
    let order = retained_corner_decision(
        policy.strict_predicate_pass(|| lower.parameter.cmp_by_refinement(&upper.parameter, policy))
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
        operation,
    )?;
    if order != std::cmp::Ordering::Less {
        return Err(curve_region_edit_error(operation, CurveError::Topology(
            "retained extension cuts did not bound one traversal interval".into(),
        )));
    }
    let range = CurveParameterRange2::new_validated(
        lower.parameter.clone(), upper.parameter.clone(),
    );
    let support = CurveSupport2::from_fragment(fragment);
    let source = match &support {
        CurveSupport2::Bezier(curve) => RationalBezier2::try_from_subcurve(curve),
        CurveSupport2::Parallel(parallel) => parallel.source().to_rational_bezier(),
        CurveSupport2::Line(_) | CurveSupport2::Circle(_) => {
            return Err(ExactCurveError::blocked(operation, support.family(), UncertaintyReason::Unsupported));
        }
    }.map_err(|cause| curve_region_edit_error(operation, cause))?;
    retained_corner_decision(
        source.finite_discovery_envelope(&range, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
        operation,
    )?;
    let replacement = Arc::new(support.restrict_certified(
        range, Some([lower.point.clone(), upper.point.clone()]), reversed, policy,
    ).map_err(|cause| curve_region_edit_error(operation, cause))?);
    previous_cut.replacement = Some(replacement.clone());
    next_cut.replacement = Some(replacement);
    Ok(())
}

''')
s=one(s,'''/// authority. Exterior cuts on native and analytic carriers are both mapped
/// onto one finite pole-free envelope before this function runs, so the second
/// cut is never mistaken for a parameter of an independently reparameterized
/// subcurve.''','''/// authority. Exterior cuts retain one certified pole-free source interval,
/// so both point witnesses and every later query keep the same parameter chart.''')
# Replace representation assertions by unchanged parameter and support oracles.
s=one(s,'''                let CornerReplacement2::SelectedFiber {
                    fragment: replacement_fragment,
                    source_scale,
                    source_offset,
                } = replacement''','''                let BezierSplitFragment2::SelectedFiber(replacement_fragment) = replacement''')
a=s.index('                for (mapped, original) in [',s.index('fn one_fragment_selected_projective_extensions_keep_the_local_fiber'))
b=s.index('                let retained = retained_corner_fragment_between_cuts(',a)
s=s[:a]+'''                assert!(replacement_fragment.parallel_carrier() == parallel);
                for (retained, original) in [
                    (replacement_fragment.range().start(), &start_extension),
                    (replacement_fragment.range().end(), &end_extension),
                ] {
                    assert!(retained == &CurveParameter2::from_selected_fiber(original.clone()));
                }
'''+s[b:]
s=one(s,'assert_eq!(&retained, replacement_fragment.as_ref());','assert!(retained == *replacement_fragment);')
s=one(s,'Some(CornerReplacement2::SelectedFiber { .. })','Some(BezierSplitFragment2::SelectedFiber(_))')
files['bezier_region.rs']=s

s=files['curve_corner_chain.rs']
s=one(s,'Some(CornerReplacement2::SelectedFiber { fragment, .. })','Some(BezierSplitFragment2::SelectedFiber(fragment))',2)
s=one(s,'''        // A finite selected envelope retains both its extended cut and the
        // authored corner in one parameter field. When the opposite setback
        // is zero, use that correlated corner witness for the chamfer chord.
        // The boundary adjacency and envelope construction already prove it
        // is the same corner; mixing in the old carrier's endpoint would throw
        // away this proof and force an unrelated Cartesian compositum.''','''        // An extended span retains its cut and unchanged corner on one source.
        // With a zero opposite setback, reuse that correlated corner witness
        // instead of adjoining the adjacent fragment's independent point field.''')
for value in ['source_curve','RationalBezier2::from(spans[0].clone())']:
    s=one(s,f'''Some(CornerReplacement2::Curve(BezierSubcurve2::Rational(
                {value},
            )))''',f'''Some(Arc::new(BezierSplitFragment2::Materialized {{
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Rational({value}),
            }}))''')
s=one(s,'''Some(CornerReplacement2::Curve(
                    BezierSubcurve2::Rational(replacement),
                ))''','''Some(Arc::new(BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Rational(replacement),
                }))''')
s=section(s,'            let canonical_anchor_tangent = if matches!(', '            if let Some(replacement) = frame', '')
s=section(s,'            let fillet = if let Some((support, parameter, source_scale, source_offset)) =', '            let mut preselected_arc_contact = None;', '')
s=section(s,'            // Chord contacts above use the geometric support and exact point.', '            let allow_boundary_contact =', '''            // Companion contacts consume the cut's exact replacement range
            // in its original support chart, including exterior endpoints.
            let replacement_companion = other_cut.replacement.as_deref();
            let other_fragment = replacement_companion.unwrap_or(other_fragment);
''')
s=one(s,'''                    CornerPlacement2::Extension => {
                        retained_selected_corner_parameter_is_in_native_chart(
                            expected,
                            CurveOperation2::Fillet,
                            policy,
                        )?
                    }''','''                    CornerPlacement2::Extension => !start_order.is_lt() && !end_order.is_gt(),''')
# The three non-linear families now use one finite source interval publisher.
a=s.index('        if matches!(fragment, BezierSplitFragment2::SelectedFiber(_))',s.index('pub(super) fn canonicalize_retained_single_fragment_extension_cuts'))
b=s.index('    pub(super) fn canonicalize_retained_corner_cut(',a)
s=s[:a]+'''        retain_corner_extension_interval(fragment, previous_cut, next_cut, operation, policy)
    }

'''+s[b:]
a=s.index('        if matches!(fragment, BezierSplitFragment2::SelectedFiber(_))',s.index('pub(super) fn canonicalize_retained_corner_cut'))
b=s.index('\n}\n\n#[cfg(test)]',a)
s=s[:a]+'''        let reversed = fragment.source_is_reversed();
        let range = if matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
            CurveParameterRange2::unit()
        } else {
            fragment.curve_region_parameter_range()
        };
        let parameter = if previous != reversed { range.start() } else { range.end() };
        let point = retained_corner_decision(
            policy.strict_predicate_pass(|| curve_fragment_endpoint_point(fragment, previous, policy))
                .map_err(|cause| curve_region_edit_error(operation, cause))?,
            operation,
        )?.ok_or_else(|| ExactCurveError::blocked(operation, CurveFamily2::RationalBezier, UncertaintyReason::Unsupported))?;
        let mut retained_endpoint = CornerTrimCut2 {
            parameter: parameter.clone(), point, placement: CornerPlacement2::Corner,
            replacement: None,
        };
        if previous {
            retain_corner_extension_interval(fragment, cut, &mut retained_endpoint, operation, policy)
        } else {
            retain_corner_extension_interval(fragment, &mut retained_endpoint, cut, operation, policy)
        }
    }
'''+s[b:]
s=one(s,'.and_then(CornerReplacement2::as_curve)', '''.and_then(|fragment| match fragment {
                            BezierSplitFragment2::Materialized { curve, .. } => Some(curve),
                            _ => None,
                        })''',2)
files['curve_corner_chain.rs']=s

import re
for name in ['bezier_region.rs','curve_corner_chain.rs']:
    s=files[name]
    s=re.sub(r'(\.replacement\s*)\.as_ref\(\)',r'\1.as_deref()',s)
    assert 'CornerReplacement2' not in s,name
    assert 'RetainedCornerExtensionCarrier2' not in s,name
    files[name]=s
for name,s in files.items(): (R/name).write_text(s)
print('Replaced corner replacement enum and finite-envelope reconstruction with retained source spans')
