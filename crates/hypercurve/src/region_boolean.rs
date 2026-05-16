//! Region-level boolean boundary pipeline.
//!
//! The routines here compose the existing event, split, classify, and boundary
//! traversal stages. Simple boundary-only contacts are regularized here, while
//! shared-boundary cases that also involve interior containment remain explicit
//! uncertainty instead of being guessed through.

use hyperlattice::Backend;

use crate::{
    BooleanBoundaryFragmentSet, BooleanBoundaryLoopSet, BooleanOp, Classification, Contour2,
    ContourIntersection, CurvePolicy, CurveResult, FillRule, IntersectionKind, Region2,
    RegionPointLocation, RegionSide, RegionView2,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BoundaryContactKind {
    PointOnly,
    Overlap,
}

impl<B: Backend> Region2<B> {
    /// Computes closed boolean boundary loops against another owned region.
    ///
    /// This is a convenience wrapper over [`RegionView2::boolean_boundary_loops`].
    pub fn boolean_boundary_loops(
        &self,
        other: &Self,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet<B>>> {
        self.as_view()
            .boolean_boundary_loops(&other.as_view(), op, policy)
    }

    /// Computes checked boolean boundary contours against another owned region.
    ///
    /// The returned contours are closed result boundaries. They are not yet
    /// assigned to material or hole bins; that role assignment belongs to the
    /// later nesting pass.
    pub fn boolean_boundary_contours(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2<B>>>> {
        self.as_view()
            .boolean_boundary_contours(&other.as_view(), op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against another owned region.
    ///
    /// The result is available only when the current boundary pipeline can
    /// produce closed contours and the nesting pass can classify those contours
    /// without boundary ambiguity.
    pub fn boolean_region(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        self.as_view()
            .boolean_region(&other.as_view(), op, fill_rule, policy)
    }
}

impl<B: Backend> RegionView2<'_, B> {
    /// Computes closed boolean boundary loops against another region view.
    ///
    /// Algorithm note: this method wires together the standard polygon clipping
    /// stages: collect intersection events, split input boundaries at those
    /// events, classify each fragment against the opposite operand, and traverse
    /// selected directed fragments into closed loops. Greiner and Hormann
    /// describe split-boundary traversal after entry/exit classification
    /// (G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons,"
    /// ACM Transactions on Graphics 17(2), 71-83, 1998). Martinez, Rueda, and
    /// Feito describe boolean selection from segment classifications for
    /// general polygons (F. Martinez, A. J. Rueda, and F. R. Feito, "A new
    /// algorithm for computing Boolean operations on polygons," Computers &
    /// Geosciences 35(6), 1177-1185, 2009). `hypercurve` keeps each stage
    /// explicit so uncertain tangencies, shared boundaries, and branch vertices
    /// can stop the pipeline instead of being resolved by a global epsilon.
    pub fn boolean_boundary_loops(
        &self,
        other: &RegionView2<'_, B>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet<B>>> {
        boolean_boundary_loops_between(self, other, op, policy)
    }

    /// Computes checked boolean boundary contours against another region view.
    ///
    /// The contours are produced only after every selected boundary chain closes.
    /// Open chains and unresolved shared boundaries are returned as uncertainty.
    pub fn boolean_boundary_contours(
        &self,
        other: &RegionView2<'_, B>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2<B>>>> {
        boolean_boundary_contours_between(self, other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against another region view.
    ///
    /// After boundary traversal, closed output contours are assigned to material
    /// and hole bins by containment depth. Hormann and Agathos discuss the
    /// point-in-polygon classification problem that underlies this nesting test
    /// (K. Hormann and A. Agathos, "The point in polygon problem for arbitrary
    /// polygons," Computational Geometry 20(3), 131-144, 2001). `hypercurve`
    /// treats any boundary result during nesting as explicit uncertainty,
    /// because a boundary touch means the output contour graph still needs a
    /// degeneracy-specific resolver.
    pub fn boolean_region(
        &self,
        other: &RegionView2<'_, B>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Region2<B>>> {
        boolean_region_between(self, other, op, fill_rule, policy)
    }
}

pub(crate) fn boolean_boundary_loops_between<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryLoopSet<B>>> {
    let intersections = first.intersect_region(second, policy)?;

    let fragments = match intersections.split_regions(first, second, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let selection = match fragments.classify_for_boolean(first, second, op, policy)? {
        Classification::Decided(selection) => selection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let emitted = selection.emit_boundary_fragments(&fragments)?;
    let chains = match emitted.assemble_chains(policy) {
        Classification::Decided(chains) => chains,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    Ok(chains.into_closed_loops())
}

pub(crate) fn boolean_boundary_contours_between<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2<B>>>> {
    if same_region_view(first, second) {
        return Ok(Classification::Decided(match op {
            BooleanOp::Union | BooleanOp::Intersection => clone_boundary_contours(first),
            BooleanOp::Difference | BooleanOp::Xor => Vec::new(),
        }));
    }
    if first.is_empty() || second.is_empty() {
        return Ok(Classification::Decided(empty_operand_boundary_contours(
            first, second, op,
        )));
    }
    match boundary_only_contact(first, second, policy)? {
        Classification::Decided(Some(kind)) => {
            return boundary_contact_boundary_contours(first, second, op, fill_rule, policy, kind);
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    if op == BooleanOp::Xor {
        return xor_boundary_contours_by_region(first, second, fill_rule, policy);
    }

    match boolean_boundary_loops_between(first, second, op, policy)? {
        Classification::Decided(loops) => {
            loops.into_contours(fill_rule).map(Classification::Decided)
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn xor_boundary_contours_by_region<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2<B>>>> {
    // The checked-contour API can express the boundary loops of a symmetric
    // difference, but it cannot attach material/hole roles to them. Build the
    // role-aware region first, then expose its checked boundary contours.
    // This follows the set identity used in Martinez, Rueda, and Feito's
    // segment-selection view of polygon booleans (F. Martinez, A. J. Rueda,
    // and F. R. Feito, "A new algorithm for computing Boolean operations on
    // polygons," Computers & Geosciences 35(6), 1177-1185, 2009) while keeping
    // remaining ambiguous shared boundaries out of the direct traversal graph
    // until the general overlap/branch resolver lands.
    match xor_region_by_difference_union(first, second, fill_rule, policy)? {
        Classification::Decided(region) => Ok(Classification::Decided(clone_boundary_contours(
            &region.as_view(),
        ))),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_region_between<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Region2<B>>> {
    if same_region_view(first, second) {
        return Ok(Classification::Decided(match op {
            BooleanOp::Union | BooleanOp::Intersection => clone_region(first),
            BooleanOp::Difference | BooleanOp::Xor => Region2::empty(),
        }));
    }
    if first.is_empty() || second.is_empty() {
        return Ok(Classification::Decided(empty_operand_region(
            first, second, op,
        )));
    }
    match boundary_only_contact(first, second, policy)? {
        Classification::Decided(Some(kind)) => {
            return boundary_contact_region(first, second, op, fill_rule, policy, kind);
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    if op == BooleanOp::Xor {
        return xor_region_by_difference_union(first, second, fill_rule, policy);
    }

    match boolean_boundary_contours_between(first, second, op, fill_rule, policy)? {
        Classification::Decided(contours) => Region2::from_boundary_contours(contours, policy),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn boundary_only_contact<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<BoundaryContactKind>>> {
    let intersections = first.intersect_region(second, policy)?;
    if intersections.is_empty() {
        return Ok(Classification::Decided(None));
    }

    let mut saw_contact = false;
    let mut saw_overlap = false;
    for pair in intersections.pairs() {
        for event in pair.intersections.events() {
            match event {
                ContourIntersection::Point(point) => match point.kind {
                    IntersectionKind::Endpoint | IntersectionKind::Tangent => {
                        saw_contact = true;
                    }
                    IntersectionKind::Crossing | IntersectionKind::Overlap => {
                        return Ok(Classification::Decided(None));
                    }
                },
                ContourIntersection::Overlap(_) => {
                    saw_contact = true;
                    saw_overlap = true;
                }
                ContourIntersection::Uncertain(uncertain) => {
                    return Ok(Classification::Uncertain(uncertain.reason));
                }
            }
        }
    }

    if !saw_contact {
        return Ok(Classification::Decided(None));
    }

    let disjoint_interiors = if saw_overlap {
        split_contact_interiors_are_disjoint(first, second, &intersections, policy)?
    } else {
        unsplit_contact_interiors_are_disjoint(first, second, policy)?
    };
    match disjoint_interiors {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(Some(if saw_overlap {
        BoundaryContactKind::Overlap
    } else {
        BoundaryContactKind::PointOnly
    })))
}

fn split_contact_interiors_are_disjoint<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    intersections: &crate::RegionIntersectionSet<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let fragments = match intersections.split_regions(first, second, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let mut first_has_outside_sample = false;
    let mut second_has_outside_sample = false;
    for contour_fragments in fragments.contours() {
        let opposite = match contour_fragments.key.side {
            RegionSide::First => second,
            RegionSide::Second => first,
        };

        for fragment in contour_fragments.fragments.fragments() {
            let sample = match fragment.segment.representative_point(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match opposite.classify_point(&sample, policy) {
                Classification::Decided(RegionPointLocation::Outside) => {
                    match contour_fragments.key.side {
                        RegionSide::First => first_has_outside_sample = true,
                        RegionSide::Second => second_has_outside_sample = true,
                    }
                }
                Classification::Decided(RegionPointLocation::Boundary) => {}
                Classification::Decided(RegionPointLocation::Inside) => {
                    return Ok(Classification::Decided(false));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }

    Ok(Classification::Decided(
        first_has_outside_sample && second_has_outside_sample,
    ))
}

fn unsplit_contact_interiors_are_disjoint<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let mut first_has_outside_sample = false;
    let mut second_has_outside_sample = false;

    match scan_unsplit_contact_samples(
        first.material_contours(),
        second,
        &mut first_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_contact_samples(
        first.hole_contours(),
        second,
        &mut first_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_contact_samples(
        second.material_contours(),
        first,
        &mut second_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_contact_samples(
        second.hole_contours(),
        first,
        &mut second_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(
        first_has_outside_sample && second_has_outside_sample,
    ))
}

fn scan_unsplit_contact_samples<B: Backend>(
    contours: &[&Contour2<B>],
    opposite: &RegionView2<'_, B>,
    has_outside_sample: &mut bool,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    for contour in contours {
        for segment in contour.segments() {
            let sample = match segment.representative_point(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match opposite.classify_point(&sample, policy) {
                Classification::Decided(RegionPointLocation::Outside) => {
                    *has_outside_sample = true;
                }
                Classification::Decided(RegionPointLocation::Boundary) => {}
                Classification::Decided(RegionPointLocation::Inside) => {
                    return Ok(Classification::Decided(false));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }

    Ok(Classification::Decided(true))
}

fn boundary_contact_boundary_contours<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
    kind: BoundaryContactKind,
) -> CurveResult<Classification<Vec<Contour2<B>>>> {
    // Boundary-only contacts carry no filled area. Foster, Hormann, and Popa
    // identify these contact degeneracies as cases that should be handled
    // separately from ordinary traversal (E. L. Foster, K. Hormann, and R. T.
    // Popa, "Clipping simple polygons with degenerate intersections,"
    // Computers & Graphics: X 2, 100007, 2019). Point-only contacts keep their
    // separate loops; shared-edge contacts must remove the coincident edge for
    // union/xor so the result does not expose an internal seam as boundary.
    Ok(Classification::Decided(match op {
        BooleanOp::Union | BooleanOp::Xor => match kind {
            BoundaryContactKind::PointOnly => {
                let mut contours = clone_boundary_contours(first);
                contours.extend(clone_boundary_contours(second));
                contours
            }
            BoundaryContactKind::Overlap => {
                return boundary_overlap_union_contours(first, second, op, fill_rule, policy);
            }
        },
        BooleanOp::Intersection => Vec::new(),
        BooleanOp::Difference => clone_boundary_contours(first),
    }))
}

fn boundary_overlap_union_contours<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2<B>>>> {
    let intersections = first.intersect_region(second, policy)?;
    let fragments = match intersections.split_regions(first, second, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let selection = match fragments.classify_for_boolean(first, second, op, policy)? {
        Classification::Decided(selection) => selection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let emitted = selection.emit_boundary_fragments(&fragments)?;

    // For a certified boundary-only overlap, the unresolved fragments are
    // exactly the coincident zero-area edges. Dropping them and assembling the
    // remaining directed boundary implements the regularized shared-edge union
    // described by fill-state clipping algorithms such as Vatti's scanline
    // formulation (B. R. Vatti, "A generic solution to polygon clipping,"
    // Communications of the ACM 35(7), 56-63, 1992). The certification above
    // rejects containment and positive-area overlap before this path runs.
    let emitted =
        BooleanBoundaryFragmentSet::new(emitted.directed_fragments().to_vec(), Vec::new());
    let chains = match emitted.assemble_chains(policy) {
        Classification::Decided(chains) => chains,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    match chains.into_closed_loops() {
        Classification::Decided(loops) => {
            loops.into_contours(fill_rule).map(Classification::Decided)
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn boundary_contact_region<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
    kind: BoundaryContactKind,
) -> CurveResult<Classification<Region2<B>>> {
    Ok(Classification::Decided(match op {
        BooleanOp::Union | BooleanOp::Xor => match kind {
            BoundaryContactKind::PointOnly => {
                merge_disjoint_region_bins(clone_region(first), clone_region(second))
            }
            BoundaryContactKind::Overlap => {
                return match boundary_overlap_union_contours(first, second, op, fill_rule, policy)?
                {
                    Classification::Decided(contours) => {
                        Region2::from_boundary_contours(contours, policy)
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
        },
        BooleanOp::Intersection => Region2::empty(),
        BooleanOp::Difference => clone_region(first),
    }))
}

fn xor_region_by_difference_union<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Region2<B>>> {
    // Region XOR is the symmetric difference `(A - B) union (B - A)`. Martinez,
    // Rueda, and Feito describe polygon boolean operations as combinations of
    // selected classified segments (F. Martinez, A. J. Rueda, and F. R. Feito,
    // "A new algorithm for computing Boolean operations on polygons,"
    // Computers & Geosciences 35(6), 1177-1185, 2009); using the set identity
    // here lets the region-level API reuse the better-tested difference and
    // union role-assignment paths while the lower boundary graph still grows a
    // dedicated overlap/branch resolver for direct XOR traversal.
    let first_only =
        match boolean_region_between(first, second, BooleanOp::Difference, fill_rule, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let second_only =
        match boolean_region_between(second, first, BooleanOp::Difference, fill_rule, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    Ok(Classification::Decided(merge_disjoint_region_bins(
        first_only,
        second_only,
    )))
}

fn merge_disjoint_region_bins<B: Backend>(first: Region2<B>, second: Region2<B>) -> Region2<B> {
    // The two symmetric-difference halves are interior-disjoint by set
    // definition. Directly merging their signed contour bins preserves
    // boundary-only contacts that a contour-only nesting pass would reject as
    // ambiguous. This mirrors Vatti's fill-state view of clipping output
    // (B. R. Vatti, "A generic solution to polygon clipping," Communications
    // of the ACM 35(7), 56-63, 1992): after the two difference regions have
    // already crossed the fill-state boundary, their explicit material/hole
    // bins can be concatenated without inventing a new traversal graph.
    let mut material_contours = first.material_contours().to_vec();
    material_contours.extend(second.material_contours().iter().cloned());
    let mut hole_contours = first.hole_contours().to_vec();
    hole_contours.extend(second.hole_contours().iter().cloned());
    Region2::new(material_contours, hole_contours)
}

fn same_region_view<B: Backend>(first: &RegionView2<'_, B>, second: &RegionView2<'_, B>) -> bool {
    same_contour_multiset(first.material_contours(), second.material_contours())
        && same_contour_multiset(first.hole_contours(), second.hole_contours())
}

fn same_contour_multiset<B: Backend>(first: &[&Contour2<B>], second: &[&Contour2<B>]) -> bool {
    if first.len() != second.len() {
        return false;
    }

    let mut matched = vec![false; second.len()];
    for first_contour in first {
        let Some(index) = second
            .iter()
            .enumerate()
            .find_map(|(index, second_contour)| {
                (!matched[index] && first_contour.has_same_exact_boundary(second_contour))
                    .then_some(index)
            })
        else {
            return false;
        };
        matched[index] = true;
    }

    true
}

fn clone_boundary_contours<B: Backend>(region: &RegionView2<'_, B>) -> Vec<Contour2<B>> {
    // Exact contour-bin identity fast paths keep coincident boundaries out of
    // the general traversal graph. Foster, Hormann, and Popa show that
    // degenerate polygon clipping benefits from separating coincident-boundary
    // cases from ordinary entry/exit traversal (E. L. Foster, K. Hormann, and
    // R. T. Popa, "Clipping simple polygons with degenerate intersections,"
    // Computers & Graphics: X 2, 100007, 2019). This fast path handles exact
    // reordered contours, cyclic start-index changes, and reversed traversal
    // within each role bin; split or otherwise equivalent-but-nonidentical
    // boundaries still belong to the future overlap resolver.
    region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
        .map(|contour| (*contour).clone())
        .collect()
}

fn clone_region<B: Backend>(region: &RegionView2<'_, B>) -> Region2<B> {
    // Region-level identity fast paths preserve explicit contour roles without
    // re-entering the nesting pass. Vatti describes boolean output in terms of
    // fill-state transitions (B. R. Vatti, "A generic solution to polygon
    // clipping," Communications of the ACM 35(7), 56-63, 1992); exact identity
    // and empty-set identities reduce those transitions to cloning or dropping
    // an operand. Keeping this at the `Region2` layer matters for valid input
    // regions whose explicit material bins touch and therefore cannot be
    // reconstructed by a boundary-only containment pass.
    Region2::new(
        region
            .material_contours()
            .iter()
            .map(|contour| (*contour).clone())
            .collect(),
        region
            .hole_contours()
            .iter()
            .map(|contour| (*contour).clone())
            .collect(),
    )
}

fn empty_operand_boundary_contours<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    op: BooleanOp,
) -> Vec<Contour2<B>> {
    // Empty-set identities are regularized boolean identities, so they should
    // not enter the clipping graph at all. Vatti's scanline formulation
    // describes boolean construction in terms of fill-state transitions
    // (B. R. Vatti, "A generic solution to polygon clipping," Communications
    // of the ACM 35(7), 56-63, 1992); with one empty operand, those transitions
    // reduce to the nonempty operand or to the empty set.
    match (first.is_empty(), second.is_empty(), op) {
        (true, _, BooleanOp::Union | BooleanOp::Xor) => clone_boundary_contours(second),
        (_, true, BooleanOp::Union | BooleanOp::Xor | BooleanOp::Difference) => {
            clone_boundary_contours(first)
        }
        _ => Vec::new(),
    }
}

fn empty_operand_region<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    op: BooleanOp,
) -> Region2<B> {
    match (first.is_empty(), second.is_empty(), op) {
        (true, _, BooleanOp::Union | BooleanOp::Xor) => clone_region(second),
        (_, true, BooleanOp::Union | BooleanOp::Xor | BooleanOp::Difference) => clone_region(first),
        _ => Region2::empty(),
    }
}
