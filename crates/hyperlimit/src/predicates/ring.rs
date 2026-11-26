//! Shoelace signs, local turns and even-odd classification for polygonal rings.
//!
//! Slices specify cyclic edges; indexed variants borrow vertices in index order.
//! These predicates do not validate simplicity, hole nesting, material side or
//! arrangement topology. Shoelace area and local turn consistency are separate
//! from even-odd membership. Exact coordinates remain authoritative, while the
//! explicit policy controls whether an unresolved decision may use a qualified
//! approximate terminal.
//!
//! Predicate outcomes carry local certainty/stage; bare fact summaries and
//! retained report fields do not. Structural report validation checks internal
//! metadata, and source replay compares results without retaining its receipt.
//! Neither operation upgrades an approximate report into an exact proof. Ring
//! length controls temporary/report allocation; no total resource budget is
//! introduced by these helpers.

use crate::predicate::PredicatePolicy;
use crate::predicates::decision::{DecisionTrace, UnknownDecision, decided};
use core::cmp::Ordering;

use crate::classify::{PointSegmentLocation, RingConvexity, RingPointLocation};
use crate::geometry::Point2;
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::order::compare_reals_with_policy;
use crate::predicates::orient::orient2d_with_policy;
use crate::predicates::segment::classify_collinear_point_segment_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{resolve_real_sign, signed_term_filter};
use hyperreal::{Rational, Real, ZeroKnowledge};

/// Structural edge counts and policy-decided summaries for one cyclic ring.
///
/// These facts are summaries over exact `Real` coordinates. They are
/// useful for algorithm selection, but they are not topology certificates:
/// later containment, visibility, and intersection decisions must still call
/// exact predicates.
/// Area and turn classification can require refinement; their certainty/stage
/// receipts are discarded. Under an approximate policy those fields therefore
/// cannot be treated as certified merely because they appear in this carrier.
/// Public fields have no source binding or consistency validation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ring2Facts {
    /// Number of vertices in the caller-supplied ring.
    pub vertex_count: usize,
    /// Number of cyclic edges structurally known to collapse to a point.
    pub known_degenerate_edges: usize,
    /// Number of non-degenerate cyclic edges structurally known horizontal or vertical.
    pub known_axis_aligned_edges: usize,
    /// Number of cyclic edges with unknown coordinate-zero status.
    pub unknown_edge_zero_status: usize,
    /// Decided shoelace-area sign, with certainty/stage discarded; None means Unknown.
    pub signed_area: Option<Sign>,
    /// Local turn summary, without child certainty receipts or a simplicity proof.
    pub convexity: RingConvexity,
}

/// Failure of retained even-odd report metadata validation or source replay.
///
/// The report is a predicate-layer audit trail, not a polygon arrangement data
/// structure. These errors describe checked metadata inconsistencies or a
/// failed source replay; absence of an error does not certify the source
/// geometry, policy admissibility or all producer invariants.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RingEvenOddValidationError {
    /// Short-ring metadata, non-boundary report length or an edge index violates
    /// the count constraints checked by validate.
    EdgeCountMismatch,
    /// Boundary ray metadata, multiple boundary records or the retained boundary
    /// index disagree with the checked boundary conventions.
    BoundaryMismatch,
    /// An edge report retained a point-on-segment decision inconsistent with
    /// its boundary metadata.
    /// This variant is currently not emitted by the validators in this module.
    SegmentLocationMismatch,
    /// A non-boundary edge lacks above flags, or a straddle lacks upward/orientation.
    MissingStraddleFacts,
    /// A non-straddling edge retained orientation/upward facts or a crossing.
    UnexpectedStraddleFacts,
    /// A straddling edge retained a crossing decision that does not match its
    /// orientation and upward facts.
    /// Also used when the aggregate crossing count disagrees with edge flags.
    CrossingMismatch,
    /// Retained crossing parity derives a different point/ring location.
    LocationMismatch,
    /// Source classification was Unknown or produced a different report value.
    SourceReplayMismatch,
}

/// Retained decision values for one edge visited by an even-odd point/ring test.
///
/// Boundary is decided first through orientation and collinear segment classification. Only
/// non-boundary y-straddling edges retain the orientation and upward facts used
/// by the crossing-number test.
/// Fields carry no individual certainty, source coordinates or source identity;
/// preserve the enclosing classifier outcome. Publicly constructed records are
/// not automatically proofs, even if structural validation accepts them.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RingEvenOddEdgeReport {
    /// Edge position in ring order, not a global vertex index for indexed input.
    pub edge_index: usize,
    /// Decided point/segment relation, qualified by the enclosing outcome.
    pub segment_location: PointSegmentLocation,
    /// Whether the first edge endpoint is strictly above the query point.
    pub a_above: Option<bool>,
    /// Whether the second edge endpoint is strictly above the query point.
    pub b_above: Option<bool>,
    /// Whether the directed edge goes upward in y.
    /// The producer sets this to b_above on a straddle; validate does not check that equality.
    pub upward: Option<bool>,
    /// Decided orientation of `(a, b, point)` for y-straddling non-boundary edges.
    pub orientation: Option<Sign>,
    /// Whether this edge toggles even-odd parity for the positive-x ray.
    pub crosses_right: bool,
}

impl RingEvenOddEdgeReport {
    /// Tests whether the retained relation names an endpoint or segment interior.
    /// Performs no geometric replay or certainty check.
    pub const fn is_boundary(&self) -> bool {
        self.segment_location.is_on_segment()
    }

    /// Tests whether both above flags exist and differ.
    /// Does not check orientation, upward direction or their geometric validity.
    pub const fn is_y_straddling(&self) -> bool {
        matches!((self.a_above, self.b_above), (Some(a), Some(b)) if a != b)
    }
}

/// Report-bearing point-in-ring classification under the even-odd rule.
///
/// The coarse [`RingPointLocation`] remains the compatibility result. This
/// report keeps the per-edge decision values that produced it: boundary
/// point/segment decisions, y-straddle comparisons, orientation signs, crossing
/// toggles, and final parity. The standard crossing-number classifier is
/// evaluated with policy-controlled Real predicates. The report has no own
/// certainty/stage receipt or source geometry; keep its producing outcome.
/// Fields are public, and structural validation does not establish provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RingEvenOddReport {
    /// Coarse point/ring location.
    pub location: RingPointLocation,
    /// Number of cyclic edges in the caller-supplied ring.
    pub edge_count: usize,
    /// Number of retained positive-x ray crossings before classification
    /// terminated.
    pub crossing_count: usize,
    /// Edge position selected by the producer's decided on-segment relation.
    pub boundary_edge: Option<usize>,
    /// Per-edge decisions visited in cyclic order by the producer. A boundary
    /// result stops at the first decided boundary edge; later edges are absent.
    pub edges: Vec<RingEvenOddEdgeReport>,
}

impl RingEvenOddReport {
    /// Validate retained parity, boundary, and edge-crossing facts.
    ///
    /// Checks count constraints, index bounds, edge metadata shapes, at most one
    /// boundary record, crossing count and the resulting coarse location. A
    /// short ring requires an empty Outside report. Non-boundary reports need
    /// `edges.len() == edge_count`; boundary reports may retain a partial list.
    ///
    /// This does not validate source coordinates, index uniqueness/order,
    /// boundary-at-end traversal, or the producer's `upward == b_above` relation.
    /// It also does not reconcile a non-boundary segment label with orientation.
    /// Publicly forged but internally accepted metadata is not geometric proof.
    pub fn validate(&self) -> Result<(), RingEvenOddValidationError> {
        if self.edge_count < 3 {
            if !self.edges.is_empty() || self.crossing_count != 0 || self.boundary_edge.is_some() {
                return Err(RingEvenOddValidationError::EdgeCountMismatch);
            }
            return if self.location == RingPointLocation::Outside {
                Ok(())
            } else {
                Err(RingEvenOddValidationError::LocationMismatch)
            };
        }

        if self.boundary_edge.is_none() && self.edges.len() != self.edge_count {
            return Err(RingEvenOddValidationError::EdgeCountMismatch);
        }

        let mut crossings = 0_usize;
        let mut boundary = None;
        for edge in &self.edges {
            if edge.edge_index >= self.edge_count {
                return Err(RingEvenOddValidationError::EdgeCountMismatch);
            }
            validate_even_odd_edge_report(edge)?;
            if edge.is_boundary() && boundary.replace(edge.edge_index).is_some() {
                return Err(RingEvenOddValidationError::BoundaryMismatch);
            }
            if edge.crosses_right {
                crossings += 1;
            }
        }

        if boundary != self.boundary_edge {
            return Err(RingEvenOddValidationError::BoundaryMismatch);
        }
        if self.crossing_count != crossings {
            return Err(RingEvenOddValidationError::CrossingMismatch);
        }

        let expected = if boundary.is_some() {
            RingPointLocation::Boundary
        } else if crossings % 2 == 1 {
            RingPointLocation::Inside
        } else {
            RingPointLocation::Outside
        };
        if self.location == expected {
            Ok(())
        } else {
            Err(RingEvenOddValidationError::LocationMismatch)
        }
    }

    /// Replay this report against source ring geometry.
    ///
    /// First validates metadata, then requires field-for-field equality with a
    /// newly decided report in the supplied ring order. An Unknown replay or a
    /// different report becomes SourceReplayMismatch. Replay certainty/stage is
    /// discarded, so Ok under an approximate policy is not exact certification.
    /// Equivalent geometry with reordered edges need not reproduce this report.
    /// For indexed input, pass vertices materialized in the original ring order.
    pub fn validate_against_sources(
        &self,
        ring: &[Point2],
        point: &Point2,
        policy: PredicatePolicy,
    ) -> Result<(), RingEvenOddValidationError> {
        self.validate()?;
        match classify_point_ring_even_odd_report_with_policy(ring, point, policy) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(RingEvenOddValidationError::SourceReplayMismatch),
        }
    }
}

/// Build structural facts for a closed polygonal ring with an explicit policy.
///
/// Copies vertex references, evaluates signed area and local turns, then counts
/// raw cyclic-edge structural patterns. Retains a repeated closing vertex in
/// counts but omits one structurally equal closing vertex from the turn scan.
/// The returned area/turn fields discard decision receipts; see [`Ring2Facts`].
pub fn ring2_facts_with_policy(points: &[Point2], policy: PredicatePolicy) -> Ring2Facts {
    let refs: Vec<_> = points.iter().collect();
    ring2_facts_refs(&refs, policy)
}

/// Build structural facts for an indexed closed polygonal ring with an explicit
/// policy.
/// Returns None for any out-of-range index; otherwise retains the supplied
/// order/repetitions and the same receipt-loss behavior as [`ring2_facts_with_policy`].
pub fn indexed_ring2_facts_with_policy(
    points: &[Point2],
    ring: &[usize],
    policy: PredicatePolicy,
) -> Option<Ring2Facts> {
    let refs = indexed_ring_refs(points, ring)?;
    Some(ring2_facts_refs(&refs, policy))
}

/// Builds summaries before scanning raw cyclic edge differences for structural zeros.
/// Area is projected with `.value()` and turns return a bare enum, losing their
/// qualification. Fewer than two vertices has no counted edges; unknown-zero
/// status is distinct from degenerate, axis-aligned and known diagonal edges.
fn ring2_facts_refs(points: &[&Point2], policy: PredicatePolicy) -> Ring2Facts {
    let mut facts = Ring2Facts {
        vertex_count: points.len(),
        known_degenerate_edges: 0,
        known_axis_aligned_edges: 0,
        unknown_edge_zero_status: 0,
        signed_area: ring_area_sign_refs(points, policy).value(),
        convexity: ring_convexity_refs(points, policy),
    };

    if points.len() < 2 {
        return facts;
    }

    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        let dx = sub_ref(&next.x, &current.x);
        let dy = sub_ref(&next.y, &current.y);

        match (dx.structural_facts().zero, dy.structural_facts().zero) {
            (ZeroKnowledge::Zero, ZeroKnowledge::Zero) => facts.known_degenerate_edges += 1,
            (ZeroKnowledge::Zero, ZeroKnowledge::NonZero)
            | (ZeroKnowledge::NonZero, ZeroKnowledge::Zero) => {
                facts.known_axis_aligned_edges += 1;
            }
            (ZeroKnowledge::Unknown, _) | (_, ZeroKnowledge::Unknown) => {
                facts.unknown_edge_zero_status += 1;
            }
            (ZeroKnowledge::NonZero, ZeroKnowledge::NonZero) => {}
        }
    }

    facts
}

/// Classify local turn consistency for a closed polygonal ring with a policy.
///
/// Ignores one structurally repeated closing vertex, then visits local triples
/// until Unknown, both turn signs, or the end. Zero turns are ignored; all zero
/// or fewer than three effective vertices gives Degenerate. This is not a
/// simplicity or global convexity test. Child certainty/stage is discarded,
/// so approximate-policy labels are not unqualified geometric proofs.
pub fn ring_convexity_with_policy(points: &[Point2], policy: PredicatePolicy) -> RingConvexity {
    let refs: Vec<_> = points.iter().collect();
    ring_convexity_refs(&refs, policy)
}

/// Classify local turn consistency for an indexed closed polygonal ring with a
/// policy.
/// None means an invalid vertex index; Some may contain RingConvexity::Unknown.
/// Valid indices preserve the bare enum's loss of child certainty receipts.
pub fn indexed_ring_convexity_with_policy(
    points: &[Point2],
    ring: &[usize],
    policy: PredicatePolicy,
) -> Option<RingConvexity> {
    let refs = indexed_ring_refs(points, ring)?;
    Some(ring_convexity_refs(&refs, policy))
}

/// Return the sign of twice the signed area of a closed polygonal ring.
///
/// The input may repeat its first vertex at the end; the repeated closing edge
/// contributes zero. The function constructs the exact shoelace expression
/// `sum(x_i*y_next - y_i*x_next)` and decides its sign under the policy.
/// For a simple ring, counterclockwise is positive; for a self-intersecting
/// walk this is signed algebraic area, not the area of its even-odd filled set.
/// This function keeps the determinant
/// in `hyperlimit` because orientation/winding is a predicate-level decision;
/// ring storage and material/hole roles belong in `hypercurve` or `hypertri`.
/// Fewer than three vertices gives Exact/Structural Zero. Other rational
/// inputs use Exact/Exact accumulation; symbolic fallback may remain Unknown
/// or return an Approximate sign. The outcome receipt must be retained.
pub fn ring_area_sign_with_policy(
    points: &[Point2],
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let refs: Vec<_> = points.iter().collect();
    ring_area_sign_refs(&refs, policy)
}

/// Return the sign of twice the signed area of an indexed closed polygonal ring
/// with an explicit predicate escalation policy.
/// Every index is checked before applying the short-ring convention. An invalid
/// index returns Unknown/Unsupported at Undecided stage, even in a short ring.
pub fn indexed_ring_area_sign_with_policy(
    points: &[Point2],
    ring: &[usize],
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let Some(refs) = indexed_ring_refs(points, ring) else {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
    };
    ring_area_sign_refs(&refs, policy)
}

/// Handles short rings, then tries rational accumulation before a Real shoelace sum.
/// Retains positive/negative products for the signed-term filter in cyclic
/// order. The symbolic path has no separate exact callback and uses the policy
/// resolver; allocation and scalar costs are not bounded by a total-work budget.
fn ring_area_sign_refs(points: &[&Point2], policy: PredicatePolicy) -> PredicateOutcome<Sign> {
    if points.len() < 3 {
        return PredicateOutcome::decided(
            Sign::Zero,
            crate::predicate::Certainty::Exact,
            crate::predicate::Escalation::Structural,
        );
    }
    if let Some(outcome) = exact_rational_ring_area_sign(points) {
        return outcome;
    }

    let mut terms = Vec::with_capacity(points.len() * 2);
    let mut area: Option<Real> = None;

    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let positive = mul_ref(&points[index].x, &points[next].y);
        let negative = mul_ref(&points[index].y, &points[next].x);
        let wedge = sub_ref(&positive, &negative);
        terms.push((positive, Sign::Positive));
        terms.push((negative, Sign::Negative));
        area = Some(match area {
            Some(current) => add_ref(&current, &wedge),
            None => wedge,
        });
    }

    let area = area.expect("three or more points produce at least one wedge");
    resolve_real_sign(
        &area,
        policy,
        || {
            let refs: Vec<_> = terms.iter().map(|(term, sign)| (term, *sign)).collect();
            signed_term_filter(&refs)
        },
        || None,
        RefinementNeed::RealRefinement,
    )
}

/// Compares one rational signed product sum with zero without materializing its value.
/// Requires every visited coordinate to have rational storage; None declines
/// that path, without proving irrationality or a zero area. Success is Exact/Exact.
fn exact_rational_ring_area_sign(points: &[&Point2]) -> Option<PredicateOutcome<Sign>> {
    let mut signs = Vec::with_capacity(points.len() * 2);
    let mut terms = Vec::<[&Rational; 2]>::with_capacity(points.len() * 2);
    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let x = points[index].x.exact_rational_ref()?;
        let y = points[index].y.exact_rational_ref()?;
        let next_x = points[next].x.exact_rational_ref()?;
        let next_y = points[next].y.exact_rational_ref()?;
        signs.extend([true, false]);
        terms.extend([[x, next_y], [y, next_x]]);
    }
    let sign = match Rational::signed_product_sum2_ordering_slice(&signs, &terms) {
        Ordering::Less => Sign::Negative,
        Ordering::Equal => Sign::Zero,
        Ordering::Greater => Sign::Positive,
    };
    Some(PredicateOutcome::decided(
        sign,
        Certainty::Exact,
        Escalation::Exact,
    ))
}

/// Classify a point against a closed polygonal ring by the even-odd rule with an
/// explicit predicate escalation policy.
///
/// Each edge's boundary test precedes its positive-x ray-crossing test; this is
/// one edge-by-edge pass, not a separate global boundary pass. No intersection
/// coordinates are constructed. Strict-above y comparisons give the vertex
/// crossing convention, and a repeated closing vertex is accepted.
///
/// Fewer than three supplied vertices returns Outside without testing boundary,
/// including a query on a supplied point/segment. Other outcomes retain the
/// weakest consumed certainty and highest stage; the first Unknown stops. This
/// wrapper discards the per-edge report but preserves the outer receipt.
pub fn classify_point_ring_even_odd_with_policy(
    ring: &[Point2],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<RingPointLocation> {
    match classify_point_ring_even_odd_report_with_policy(ring, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.location, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Policy-controlled report-bearing variant of
/// [`classify_point_ring_even_odd_with_policy`].
/// Retains visited edge decisions; the outcome, not the report fields, carries
/// certainty/stage. A boundary result is a prefix; Unknown publishes no partial report.
pub fn classify_point_ring_even_odd_report_with_policy(
    ring: &[Point2],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<RingEvenOddReport> {
    let refs: Vec<_> = ring.iter().collect();
    classify_point_ring_even_odd_report_refs(&refs, point, policy)
}

/// Classify a point against an indexed closed polygonal ring by the even-odd
/// rule with an explicit predicate escalation policy.
/// Invalid indices yield Unknown/Unsupported at Undecided stage. Valid input
/// preserves the indexed report outcome's certainty while projecting its location.
pub fn classify_point_indexed_ring_even_odd_with_policy(
    points: &[Point2],
    ring: &[usize],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<RingPointLocation> {
    match classify_point_indexed_ring_even_odd_report_with_policy(points, ring, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.location, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Policy-controlled report-bearing variant of
/// [`classify_point_indexed_ring_even_odd_with_policy`].
/// Checks all indices before classification, preserving order and repetitions.
/// Report edge indices are positions within `ring`, not indices into `points`.
pub fn classify_point_indexed_ring_even_odd_report_with_policy(
    points: &[Point2],
    ring: &[usize],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<RingEvenOddReport> {
    let Some(refs) = indexed_ring_refs(points, ring) else {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
    };
    classify_point_ring_even_odd_report_refs(&refs, point, policy)
}

/// Visits cyclic edges, retaining a boundary prefix or a complete parity report.
///
/// Decides orientation first for every edge, even if its y range later does not
/// straddle. Zero orientation invokes collinear segment classification; a
/// boundary returns immediately. Otherwise compares a.y then b.y with query y.
/// A straddle sets upward from b_above and toggles on positive/upward or
/// negative/downward orientation. Unknown stops without publishing accumulated
/// edges; successful results carry the joined consumed receipts.
fn classify_point_ring_even_odd_report_refs(
    ring: &[&Point2],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<RingEvenOddReport> {
    if ring.len() < 3 {
        return PredicateOutcome::decided(
            RingEvenOddReport {
                location: RingPointLocation::Outside,
                edge_count: ring.len(),
                crossing_count: 0,
                boundary_edge: None,
                edges: Vec::new(),
            },
            Certainty::Exact,
            Escalation::Structural,
        );
    }

    let mut trace = DecisionTrace::default();
    let mut inside = false;
    let mut crossing_count = 0_usize;
    let mut edges = Vec::with_capacity(ring.len());

    for index in 0..ring.len() {
        let a = &ring[index];
        let b = &ring[(index + 1) % ring.len()];

        // The orientation is needed both for the complete point/segment report
        // and, on y-straddling edges, for crossing parity. Decide it once and
        // retain the result for both decisions.
        let orientation = match decided(orient2d_with_policy(a, b, point, policy), &mut trace) {
            Ok(sign) => sign,
            Err(unknown) => return unknown.into_outcome(),
        };
        let segment_location = match orientation {
            Sign::Zero => match decided(
                classify_collinear_point_segment_with_policy(a, b, point, policy),
                &mut trace,
            ) {
                Ok(location) => location,
                Err(unknown) => return unknown.into_outcome(),
            },
            Sign::Positive | Sign::Negative => PointSegmentLocation::OffLine,
        };
        if segment_location.is_on_segment() {
            edges.push(RingEvenOddEdgeReport {
                edge_index: index,
                segment_location,
                a_above: None,
                b_above: None,
                upward: None,
                orientation: None,
                crosses_right: false,
            });
            return PredicateOutcome::decided(
                RingEvenOddReport {
                    location: RingPointLocation::Boundary,
                    edge_count: ring.len(),
                    crossing_count,
                    boundary_edge: Some(index),
                    edges,
                },
                trace.certainty,
                trace.stage,
            );
        }

        let a_above = match compare_greater(&a.y, &point.y, policy, &mut trace) {
            Ok(value) => value,
            Err(unknown) => return unknown.into_outcome(),
        };
        let b_above = match compare_greater(&b.y, &point.y, policy, &mut trace) {
            Ok(value) => value,
            Err(unknown) => return unknown.into_outcome(),
        };
        if a_above == b_above {
            edges.push(RingEvenOddEdgeReport {
                edge_index: index,
                segment_location,
                a_above: Some(a_above),
                b_above: Some(b_above),
                upward: None,
                orientation: None,
                crosses_right: false,
            });
            continue;
        }

        // For certified comparisons a y-straddle proves endpoint order: if only
        // b is above the query then b > a, otherwise b < a. Reuse the decision;
        // any terminal qualification remains in the enclosing trace receipt.
        let upward = b_above;

        let crosses_right = matches!(
            (upward, orientation),
            (true, Sign::Positive) | (false, Sign::Negative)
        );
        if crosses_right {
            inside = !inside;
            crossing_count += 1;
        }
        edges.push(RingEvenOddEdgeReport {
            edge_index: index,
            segment_location,
            a_above: Some(a_above),
            b_above: Some(b_above),
            upward: Some(upward),
            orientation: Some(orientation),
            crosses_right,
        });
    }

    PredicateOutcome::decided(
        RingEvenOddReport {
            location: if inside {
                RingPointLocation::Inside
            } else {
                RingPointLocation::Outside
            },
            edge_count: ring.len(),
            crossing_count,
            boundary_edge: None,
            edges,
        },
        trace.certainty,
        trace.stage,
    )
}

/// Return whether `point` is inside or on the boundary of `ring` by the
/// even-odd rule with an explicit predicate escalation policy.
/// Maps the location to a Boolean without dropping the outcome's qualification
/// or Unknown. Short rings return false even at a supplied vertex.
pub fn point_in_ring_even_odd_with_policy(
    ring: &[Point2],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_ring_even_odd_with_policy(ring, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_inside_or_boundary(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Return whether `point` is inside or on the boundary of an indexed ring by
/// the even-odd rule with an explicit predicate escalation policy.
/// Preserves invalid-index Unknown and all qualification from the indexed classifier.
pub fn point_in_indexed_ring_even_odd_with_policy(
    points: &[Point2],
    ring: &[usize],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_indexed_ring_even_odd_with_policy(points, ring, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_inside_or_boundary(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Collects checked vertex references in index order; any invalid index gives None.
/// Repeated indices are retained and no topology or coordinate equality is tested.
fn indexed_ring_refs<'a>(points: &'a [Point2], ring: &[usize]) -> Option<Vec<&'a Point2>> {
    ring.iter().map(|&index| points.get(index)).collect()
}

/// Checks boundary/straddle metadata shape and the stored crossing truth table.
///
/// Boundary labels require absent ray metadata; non-boundaries require above
/// flags, and straddles additionally require upward/orientation. It does not
/// require upward to equal b_above, check a segment label against orientation,
/// or replay coordinates. SegmentLocationMismatch is not produced here.
fn validate_even_odd_edge_report(
    edge: &RingEvenOddEdgeReport,
) -> Result<(), RingEvenOddValidationError> {
    if edge.is_boundary() {
        if edge.a_above.is_some()
            || edge.b_above.is_some()
            || edge.upward.is_some()
            || edge.orientation.is_some()
            || edge.crosses_right
        {
            return Err(RingEvenOddValidationError::BoundaryMismatch);
        }
        return Ok(());
    }

    let (Some(a_above), Some(b_above)) = (edge.a_above, edge.b_above) else {
        return Err(RingEvenOddValidationError::MissingStraddleFacts);
    };
    if a_above == b_above {
        if edge.upward.is_some() || edge.orientation.is_some() || edge.crosses_right {
            return Err(RingEvenOddValidationError::UnexpectedStraddleFacts);
        }
        return Ok(());
    }

    let (Some(upward), Some(orientation)) = (edge.upward, edge.orientation) else {
        return Err(RingEvenOddValidationError::MissingStraddleFacts);
    };
    let expected_crosses = matches!(
        (upward, orientation),
        (true, Sign::Positive) | (false, Sign::Negative)
    );
    if edge.crosses_right == expected_crosses {
        Ok(())
    } else {
        Err(RingEvenOddValidationError::CrossingMismatch)
    }
}

/// Summarizes local turn signs, stopping on Unknown or both nonzero sign classes.
/// Drops orientation receipts via `.value()`; zero-only/short rings are Degenerate.
/// It does not test simplicity, and a terminal-qualified sign loses its receipt.
fn ring_convexity_refs(points: &[&Point2], policy: PredicatePolicy) -> RingConvexity {
    let len = open_ring_len(points);
    if len < 3 {
        return RingConvexity::Degenerate;
    }

    let mut saw_positive = false;
    let mut saw_negative = false;
    for index in 0..len {
        let previous = points[(index + len - 1) % len];
        let current = points[index];
        let next = points[(index + 1) % len];
        let Some(sign) = orient2d_with_policy(previous, current, next, policy).value() else {
            return RingConvexity::Unknown;
        };

        match sign {
            Sign::Positive => saw_positive = true,
            Sign::Negative => saw_negative = true,
            Sign::Zero => {}
        }

        if saw_positive && saw_negative {
            return RingConvexity::MixedTurns;
        }
    }

    if saw_positive || saw_negative {
        RingConvexity::LocallyConvex
    } else {
        RingConvexity::Degenerate
    }
}

/// Omits one final vertex only when it is structurally equal to the first.
/// Uses point PartialEq, not policy-decided mathematical coordinate equality;
/// no interior duplicates or additional repeated closing vertices are removed.
fn open_ring_len(points: &[&Point2]) -> usize {
    let mut len = points.len();
    if len > 1 && points[0] == points[len - 1] {
        len -= 1;
    }
    len
}

/// Extracts a strict-greater comparison while joining its decided receipt.
/// Unknown returns its hint/stage without modifying the local trace.
fn compare_greater(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<bool, UnknownDecision> {
    Ok(decided(compare_reals_with_policy(left, right, policy), trace)? == Ordering::Greater)
}

/// Finite ring fixtures, report consistency checks and policy-uncertainty controls.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicates::decision::{certainty_rank, stage_rank};

    /// Allows terminal decisions while ordinary rational fixtures remain exact.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Constructs an exact integer planar fixture.
    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(hyperreal::Real::from(x), hyperreal::Real::from(y))
    }

    /// Constructs rational coordinates with a caller-supplied nonzero denominator.
    fn rational_p2(x: i32, y: i32, denominator: i32) -> Point2 {
        Point2::new(
            (hyperreal::Real::from(x) / hyperreal::Real::from(denominator)).unwrap(),
            (hyperreal::Real::from(y) / hyperreal::Real::from(denominator)).unwrap(),
        )
    }

    /// Retains a proved unit-circle identity before returning a separate
    /// angle-addition zero used to exercise the current unresolved frontier.
    fn terminal_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let original = &sine * &sine + &cosine * &cosine - Real::one();
        // Preserve the original identity as a newly certified proof case.
        assert_eq!(
            original.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
        // A separate angle-addition identity retains the terminal-policy control.
        crate::test_support::terminally_unresolved_zero()
    }

    #[test]
    /// Checks clockwise/counterclockwise rectangle signs and collinear/empty zeros.
    fn ring_area_sign_classifies_winding_and_degenerate_rings() {
        let ccw = [p2(0, 0), p2(4, 0), p2(4, 3), p2(0, 3)];
        let cw = [p2(0, 0), p2(0, 3), p2(4, 3), p2(4, 0)];
        let line = [p2(0, 0), p2(1, 1), p2(2, 2)];

        assert_eq!(
            crate::ring_area_sign(&ccw, APPROX).value(),
            Some(Sign::Positive)
        );
        assert_eq!(
            crate::ring_area_sign(&cw, APPROX).value(),
            Some(Sign::Negative)
        );
        assert_eq!(
            crate::ring_area_sign(&line, APPROX).value(),
            Some(Sign::Zero)
        );
        assert_eq!(crate::ring_area_sign(&[], APPROX).value(), Some(Sign::Zero));
    }

    #[test]
    /// Requires a positive Exact/Exact receipt for rational coordinates.
    /// The assertion checks the receipt, not an allocation or dispatch count.
    fn exact_rational_ring_area_uses_sign_only_accumulation() {
        let ring = [
            rational_p2(0, 0, 11),
            rational_p2(40, 0, 11),
            rational_p2(40, 40, 11),
            rational_p2(0, 40, 11),
        ];
        let outcome = crate::ring_area_sign(&ring, APPROX);
        assert!(matches!(
            outcome,
            PredicateOutcome::Decided {
                value: Sign::Positive,
                certainty: Certainty::Exact,
                stage: Escalation::Exact,
            }
        ));
    }

    #[test]
    /// Checks square location values and closed-boundary Boolean membership.
    fn point_ring_even_odd_classifies_inside_outside_and_boundary() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];

        assert_eq!(
            crate::classify_point_ring_even_odd(&ring, &p2(2, 2), APPROX).value(),
            Some(RingPointLocation::Inside)
        );
        assert_eq!(
            crate::classify_point_ring_even_odd(&ring, &p2(5, 2), APPROX).value(),
            Some(RingPointLocation::Outside)
        );
        assert_eq!(
            crate::classify_point_ring_even_odd(&ring, &p2(4, 2), APPROX).value(),
            Some(RingPointLocation::Boundary)
        );
        assert_eq!(
            crate::point_in_ring_even_odd(&ring, &p2(4, 2), APPROX).value(),
            Some(true)
        );
    }

    #[test]
    /// Checks a square interior's one crossing, all edge records and source replay.
    /// The fixture extracts the report value rather than asserting its outer receipt.
    fn point_ring_even_odd_report_retains_crossing_parity() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let point = p2(2, 2);
        let report = crate::classify_point_ring_even_odd_report(&ring, &point, APPROX)
            .value()
            .expect("axis-aligned square containment should decide exactly");

        assert_eq!(report.location, RingPointLocation::Inside);
        assert_eq!(report.edge_count, 4);
        assert_eq!(report.boundary_edge, None);
        assert_eq!(report.crossing_count, 1);
        assert_eq!(report.edges.len(), 4);
        assert_eq!(
            report
                .edges
                .iter()
                .filter(|edge| edge.crosses_right)
                .count(),
            1
        );
        assert_eq!(report.validate(), Ok(()));
        assert_eq!(
            report.validate_against_sources(&ring, &point, APPROX,),
            Ok(())
        );
    }

    #[test]
    /// Checks a boundary prefix with no crossings and rejection of a removed boundary index.
    fn point_ring_even_odd_report_keeps_boundary_and_parity_distinct() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let point = p2(4, 2);
        let report = crate::classify_point_ring_even_odd_report(&ring, &point, APPROX)
            .value()
            .expect("edge boundary should decide exactly");

        assert_eq!(report.location, RingPointLocation::Boundary);
        assert_eq!(report.boundary_edge, Some(1));
        assert_eq!(report.crossing_count, 0);
        assert!(report.edges.last().is_some_and(|edge| edge.is_boundary()));
        assert_eq!(report.validate(), Ok(()));

        let mut forged = report.clone();
        forged.boundary_edge = None;
        assert_eq!(
            forged.validate(),
            Err(RingEvenOddValidationError::EdgeCountMismatch)
        );
    }

    #[test]
    /// Checks diamond inside/outside parity when the ray passes through vertex heights.
    fn point_ring_even_odd_report_handles_vertex_straddles_without_double_counting() {
        let diamond = [p2(0, 2), p2(2, 4), p2(4, 2), p2(2, 0)];
        let inside = crate::classify_point_ring_even_odd_report(&diamond, &p2(2, 2), APPROX)
            .value()
            .expect("diamond interior should decide exactly");
        let outside = crate::classify_point_ring_even_odd_report(&diamond, &p2(5, 2), APPROX)
            .value()
            .expect("diamond exterior should decide exactly");

        assert_eq!(inside.location, RingPointLocation::Inside);
        assert_eq!(inside.crossing_count, 1);
        assert_eq!(inside.validate(), Ok(()));
        assert_eq!(outside.location, RingPointLocation::Outside);
        assert_eq!(outside.crossing_count, 0);
        assert_eq!(outside.validate(), Ok(()));
    }

    #[test]
    /// Requires CollinearOutside metadata for a query beyond a horizontal square edge.
    fn point_ring_even_odd_report_retains_collinear_outside_edge_evidence() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let report = crate::classify_point_ring_even_odd_report(&ring, &p2(6, 0), APPROX)
            .value()
            .expect("collinear exterior point should decide exactly");

        assert_eq!(report.location, RingPointLocation::Outside);
        assert_eq!(
            report.edges[0].segment_location,
            PointSegmentLocation::CollinearOutside
        );
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    /// Checks an indexed square report's location/parity/metadata and invalid-index decline.
    /// It does not invoke the source-replay validator despite the historical test name.
    fn indexed_ring_even_odd_report_replays_caller_topology() {
        let points = [p2(9, 9), p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let ring = [1, 2, 3, 4];
        let point = p2(2, 2);
        let report =
            crate::classify_point_indexed_ring_even_odd_report(&points, &ring, &point, APPROX)
                .value()
                .expect("indexed square containment should decide exactly");

        assert_eq!(report.location, RingPointLocation::Inside);
        assert_eq!(report.crossing_count, 1);
        assert_eq!(report.validate(), Ok(()));
        assert!(
            crate::classify_point_indexed_ring_even_odd_report(&points, &[1, 99], &point, APPROX)
                .value()
                .is_none()
        );
    }

    #[test]
    /// Checks square interior membership with an explicitly repeated closing vertex.
    fn point_ring_even_odd_accepts_repeated_closing_vertex() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4), p2(0, 0)];

        assert_eq!(
            crate::classify_point_ring_even_odd(&ring, &p2(1, 1), APPROX).value(),
            Some(RingPointLocation::Inside)
        );
    }

    #[test]
    /// Checks raw duplicate-closing edge counts alongside area and local-turn summaries.
    fn ring_facts_count_raw_edges_but_classify_open_turns() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 3), p2(0, 3), p2(0, 0)];
        let facts = crate::ring2_facts(&ring, APPROX);

        assert_eq!(facts.vertex_count, 5);
        assert_eq!(facts.known_degenerate_edges, 1);
        assert_eq!(facts.known_axis_aligned_edges, 4);
        assert_eq!(facts.unknown_edge_zero_status, 0);
        assert_eq!(facts.signed_area, Some(Sign::Positive));
        assert_eq!(facts.convexity, RingConvexity::LocallyConvex);
    }

    #[test]
    /// Checks indexed area, location, turns and Boolean membership plus invalid fact indices.
    fn indexed_ring_predicates_reuse_caller_topology() {
        let points = [p2(9, 9), p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let ring = [1, 2, 3, 4];

        assert_eq!(
            crate::indexed_ring_area_sign(&points, &ring, APPROX).value(),
            Some(Sign::Positive)
        );
        assert_eq!(
            crate::classify_point_indexed_ring_even_odd(&points, &ring, &p2(2, 2), APPROX).value(),
            Some(RingPointLocation::Inside)
        );
        assert_eq!(
            crate::indexed_ring_convexity(&points, &ring, APPROX),
            Some(RingConvexity::LocallyConvex)
        );
        assert!(crate::indexed_ring2_facts(&points, &[1, 99], APPROX).is_none());

        assert_eq!(
            ring_convexity_with_policy(&points[1..], APPROX),
            RingConvexity::LocallyConvex
        );
        assert_eq!(
            point_in_indexed_ring_even_odd_with_policy(
                &points,
                &ring,
                &p2(2, 2),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(true)
        );
    }

    #[test]
    /// Checks Unknown propagation through report, location, indexed and Boolean wrappers.
    fn strict_even_odd_classifier_propagates_unresolved_comparisons() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let point = Point2::new(Real::from(2), terminal_zero());
        assert!(matches!(
            classify_point_ring_even_odd_report_with_policy(&ring, &point, PredicatePolicy::STRICT,),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            classify_point_ring_even_odd_with_policy(&ring, &point, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_point_indexed_ring_even_odd_with_policy(
                &ring,
                &[0, 1, 2, 3],
                &point,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            point_in_ring_even_odd_with_policy(&ring, &point, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            point_in_indexed_ring_even_odd_with_policy(
                &ring,
                &[0, 1, 2, 3],
                &point,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    #[test]
    /// Checks empty/diagonal/unknown-zero summaries and typed invalid-index area failure.
    fn ring_facts_cover_empty_indexed_diagonal_and_unknown_edges() {
        let empty = ring2_facts_with_policy(&[], APPROX);
        assert_eq!(empty.vertex_count, 0);
        assert_eq!(empty.known_degenerate_edges, 0);

        let diagonal = [p2(0, 0), p2(1, 1), p2(2, 3)];
        let facts = indexed_ring2_facts_with_policy(&diagonal, &[0, 1, 2], APPROX)
            .expect("valid indices should retain ring facts");
        assert_eq!(facts.vertex_count, 3);
        assert_eq!(facts.known_axis_aligned_edges, 0);
        assert_eq!(facts.unknown_edge_zero_status, 0);

        let unresolved = [
            p2(0, 0),
            Point2::new(terminal_zero(), Real::from(1)),
            p2(2, 2),
        ];
        let facts = ring2_facts_with_policy(&unresolved, PredicatePolicy::STRICT);
        assert!(facts.unknown_edge_zero_status > 0);

        assert!(indexed_ring2_facts_with_policy(&diagonal, &[0, 9], APPROX).is_none());
        assert!(matches!(
            indexed_ring_area_sign_with_policy(&diagonal, &[0, 9], APPROX),
            PredicateOutcome::Unknown {
                needed: RefinementNeed::Unsupported,
                stage: Escalation::Undecided,
            }
        ));
    }

    #[test]
    /// Checks short-ring Outside report values and strict replay, even at a supplied point.
    /// The test extracts values and does not directly assert the certainty field.
    fn degenerate_ring_reports_are_exact_and_replayable() {
        for ring in [&[][..], &[p2(0, 0)][..], &[p2(0, 0), p2(1, 0)][..]] {
            let report = classify_point_ring_even_odd_report_with_policy(
                ring,
                &p2(0, 0),
                PredicatePolicy::STRICT,
            )
            .value()
            .expect("rings with fewer than three points have an exact outside result");
            assert_eq!(report.location, RingPointLocation::Outside);
            assert_eq!(report.edge_count, ring.len());
            assert_eq!(report.validate(), Ok(()));
            assert_eq!(
                report.validate_against_sources(ring, &p2(0, 0), PredicatePolicy::STRICT),
                Ok(())
            );
        }
    }

    #[test]
    /// Checks metadata rejection when the retained boundary index differs from its edge.
    fn ring_report_validation_rejects_a_different_boundary_index() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let point = p2(4, 2);
        let mut report = classify_point_ring_even_odd_report_with_policy(&ring, &point, APPROX)
            .value()
            .expect("boundary classification should decide");
        report.boundary_edge = Some(0);
        assert_eq!(
            report.validate(),
            Err(RingEvenOddValidationError::BoundaryMismatch)
        );
    }

    #[test]
    /// Checks short, uniform-turn, zero-turn, mixed-turn and strict unresolved summaries.
    fn ring_convexity_covers_turn_classes_and_unresolved_turns() {
        assert_eq!(
            ring_convexity_with_policy(&[p2(0, 0), p2(1, 0)], APPROX),
            RingConvexity::Degenerate
        );
        assert_eq!(
            ring_convexity_with_policy(&[p2(0, 0), p2(0, 2), p2(2, 2), p2(2, 0)], APPROX),
            RingConvexity::LocallyConvex
        );
        assert_eq!(
            ring_convexity_with_policy(&[p2(0, 0), p2(1, 1), p2(2, 2)], APPROX),
            RingConvexity::Degenerate
        );
        assert_eq!(
            ring_convexity_with_policy(&[p2(0, 0), p2(2, 0), p2(1, 1), p2(2, 2), p2(0, 2)], APPROX,),
            RingConvexity::MixedTurns
        );

        let unresolved = [
            p2(0, 0),
            p2(1, 0),
            Point2::new(Real::from(2), terminal_zero()),
        ];
        assert_eq!(
            ring_convexity_with_policy(&unresolved, PredicatePolicy::STRICT),
            RingConvexity::Unknown
        );
    }

    #[test]
    /// Checks receipt joins, all rank constants and unchanged Unknown hint/stage forwarding.
    fn decision_trace_retains_the_least_certain_and_latest_stage() {
        let mut trace = DecisionTrace::default();
        assert!(matches!(
            decided(
                PredicateOutcome::decided(7, Certainty::Filtered, Escalation::Filter,),
                &mut trace,
            ),
            Ok(7)
        ));
        assert_eq!(trace.certainty, Certainty::Filtered);
        assert_eq!(trace.stage, Escalation::Filter);

        assert!(matches!(
            decided(
                PredicateOutcome::decided(9, Certainty::Approximate, Escalation::Refined,),
                &mut trace,
            ),
            Ok(9)
        ));
        assert_eq!(trace.certainty, Certainty::Approximate);
        assert_eq!(trace.stage, Escalation::Refined);
        assert_eq!(certainty_rank(Certainty::Exact), 0);
        assert_eq!(certainty_rank(Certainty::Filtered), 1);
        assert_eq!(certainty_rank(Certainty::Approximate), 2);
        assert_eq!(stage_rank(Escalation::Structural), 0);
        assert_eq!(stage_rank(Escalation::Filter), 1);
        assert_eq!(stage_rank(Escalation::Exact), 2);
        assert_eq!(stage_rank(Escalation::Refined), 3);
        assert_eq!(stage_rank(Escalation::Undecided), 4);

        let unknown = decided::<()>(
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Undecided),
            &mut trace,
        )
        .expect_err("unknown decisions must propagate");
        assert!(matches!(
            unknown.into_outcome::<()>(),
            PredicateOutcome::Unknown {
                needed: RefinementNeed::RealRefinement,
                stage: Escalation::Undecided,
            }
        ));
    }

    #[test]
    /// Rejects two retained boundary records and checks three strict unresolved fixtures.
    fn ring_report_rejects_multiple_boundaries_and_propagates_late_unknowns() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let boundary = classify_point_ring_even_odd_report_with_policy(&ring, &p2(4, 2), APPROX)
            .value()
            .expect("boundary report");
        let boundary_edge = boundary
            .edges
            .last()
            .expect("boundary edge is retained")
            .clone();
        let forged = RingEvenOddReport {
            location: RingPointLocation::Boundary,
            edge_count: 4,
            crossing_count: 0,
            boundary_edge: Some(boundary_edge.edge_index),
            edges: vec![boundary_edge.clone(), boundary_edge],
        };
        assert_eq!(
            forged.validate(),
            Err(RingEvenOddValidationError::BoundaryMismatch)
        );

        let unresolved = terminal_zero();
        assert!(matches!(
            classify_point_ring_even_odd_report_with_policy(
                &[p2(0, 0), p2(1, 0), p2(0, 1)],
                &Point2::new(unresolved.clone(), Real::zero()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_point_ring_even_odd_report_with_policy(
                &[p2(0, 0), p2(0, 1), p2(-1, 1)],
                &Point2::new(Real::one(), unresolved.clone()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_point_ring_even_odd_report_with_policy(
                &[p2(0, -1), p2(0, 0), p2(-1, 0)],
                &Point2::new(Real::one(), unresolved),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }
}
