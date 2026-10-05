//! Chord/chord and collinear intersections and support systems.

use super::*;

impl BezierAlgebraicChord2 {
    /// Intersects two retained exact chords without materializing either
    /// endpoint field. Endpoint-side predicates need at most three selected
    /// roots at once; strict interior contacts retain the two nonparallel
    /// supports as compact correlated point evidence.
    pub(crate) fn chord_intersections(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordPairIntersections2>> {
        if policy.permits_approximate_512() {
            let strict =
                policy.strict_predicate_pass(|| self.chord_intersections_once(other, policy));
            match strict {
                Ok(decided @ Classification::Decided(_)) => return Ok(decided),
                Ok(Classification::Uncertain(_)) => {}
                Err(error) => return Err(error),
            }
        }
        self.chord_intersections_once(other, policy)
    }

    pub(in crate::bezier_offset) fn chord_intersections_once(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordPairIntersections2>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        // Refine each finite chord before entering their infinite-support
        // relation. Disjoint Cartesian projections are an exact rejection
        // certificate under every policy and avoid both a STRICT boundary
        // blocker and an APPROXIMATE_512 endpoint-incidence terminal. The
        // bounded speculative pass leaves this cold refinement to its
        // complete caller.
        // This is an optional rejection filter. Boxes of chords that really
        // meet never separate, and each deeper level of a selected-fiber
        // endpoint costs a larger local Sturm refinement, so the ladder stops
        // where the exact support relation below is cheaper.
        if !policy.has_bounded_exact_predicate_budget() {
            for refinement_steps in [0, 2, 4, 8, 16, 32, 64] {
                let (Classification::Decided(first), Classification::Decided(second)) = (
                    self.conservative_bounds_refined(refinement_steps, policy)?,
                    other.conservative_bounds_refined(refinement_steps, policy)?,
                ) else {
                    continue;
                };
                if first.overlaps_with_policy(&second, &CurveContext::STRICT)
                    == Classification::Decided(false)
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair-side-kernel",
                        "refined-chord-box-disjointness",
                    );
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
                    ));
                }
            }
        }
        // Finite descendants produced by splits, reversals, coalescing, and
        // offset-miter legs retain their common affine support explicitly.
        // That identity is a stronger exact certificate than rebuilding a
        // zero tangent cross from their potentially multi-field endpoints.
        if self.shares_retained_support(other) {
            return self.collinear_chord_intersections(other, policy);
        }
        if self.shared_tangent_orientation(other).is_some()
            && let Some(collinear) = self.retained_parallel_support_collinearity(other, policy)?
        {
            return if collinear {
                self.collinear_chord_intersections(other, policy)
            } else {
                Ok(Classification::Decided(
                    BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
                ))
            };
        }
        let certified_axis_directions = self
            .certified_axis_direction()
            .zip(other.certified_axis_direction());
        if let Some((first_direction, second_direction)) = certified_axis_directions
            && first_direction.axis() == second_direction.axis()
        {
            // Two cardinal supports on the same axis are structurally
            // parallel. Endpoint predicates from independently selected
            // fields must not manufacture a transverse intersection between
            // them: compare the one constant coordinate and dispatch the
            // complete parallel relation directly.
            return match self.support_collinearity(other, policy)? {
                Classification::Decided(true) => self.collinear_chord_intersections(other, policy),
                Classification::Decided(false) => Ok(Classification::Decided(
                    BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
                )),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        let perpendicular_axis_directions =
            certified_axis_directions.filter(|(first, second)| first.axis() != second.axis());
        if perpendicular_axis_directions.is_none() {
            // Collinear supports must first be parallel. Most unary
            // broad-phase survivors are unrelated oblique chords, so decide
            // their direction cross before attempting the substantially more
            // expensive support-equality predicate.
            let tangent_cross =
                policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))?;
            let collinearity = if tangent_cross == Classification::Decided(RealSign::Zero) {
                self.support_collinearity(other, policy)?
            } else {
                // An unresolved optional direction precheck cannot make a
                // support-identity result actionable: the complete endpoint
                // side kernel below must still run. Avoid constructing that
                // redundant four-field collinearity predicate.
                Classification::Decided(false)
            };
            if tangent_cross == Classification::Decided(RealSign::Zero) {
                match collinearity {
                    Classification::Decided(true) => {
                        return self.collinear_chord_intersections(other, policy);
                    }
                    Classification::Decided(false) => {
                        return Ok(Classification::Decided(
                            BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
                        ));
                    }
                    // Collinearity is only the cheap parallel fast path.  The
                    // complete four-endpoint side kernel below can still
                    // distinguish a common support from two disjoint parallel
                    // supports without materializing either endpoint field.
                    Classification::Uncertain(_) => {}
                }
            }
        }

        // Perpendicular cardinal supports need only four scalar coordinate
        // comparisons. This preserves exact endpoint incidence when one band
        // owns a selected-circle-derived point and the other owns the same
        // point in its original selected field; a generic oriented-area
        // predicate would unnecessarily adjoin those independent fields.
        let perpendicular_axis_sides = if let Some((first_direction, second_direction)) =
            perpendicular_axis_directions
        {
            let side = |support: &Self,
                        direction: BezierAlgebraicChordAxisDirection2,
                        point: &CurvePoint2|
             -> CurveResult<Classification<crate::classify::LineSide>> {
                let perpendicular_axis = match direction.axis() {
                    Axis2::X => Axis2::Y,
                    Axis2::Y => Axis2::X,
                };
                Ok(
                    Self::point_axis_order(point, support.start(), perpendicular_axis, policy)?
                        .map(|order| direction.line_side_from_perpendicular_order(order)),
                )
            };
            let mut first_sides = [crate::classify::LineSide::On; 2];
            let mut second_sides = [crate::classify::LineSide::On; 2];
            let mut decided = true;
            for (result, point) in first_sides.iter_mut().zip([self.start(), self.end()]) {
                match side(other, second_direction, point)? {
                    Classification::Decided(side) => *result = side,
                    Classification::Uncertain(_) => decided = false,
                }
            }
            for (result, point) in second_sides.iter_mut().zip([other.start(), other.end()]) {
                match side(self, first_direction, point)? {
                    Classification::Decided(side) => *result = side,
                    Classification::Uncertain(_) => decided = false,
                }
            }
            decided.then_some((first_sides, second_sides))
        } else {
            None
        };
        let (mut first_sides, mut second_sides, mut perpendicular_axis_certificate) =
            if let Some((first_sides, second_sides)) = perpendicular_axis_sides {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "perpendicular-axis-coordinates",
                );
                (first_sides, second_sides, true)
            } else {
                let first = match BezierAlgebraicChordSupportPredicate2::try_new(self, policy)? {
                    Classification::Decided(predicate) => predicate,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let second = match BezierAlgebraicChordSupportPredicate2::try_new(other, policy)? {
                    Classification::Decided(predicate) => predicate,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let (first_sides, second_sides) = if matches!(
                    &first,
                    BezierAlgebraicChordSupportPredicate2::RefinedEndpoint { .. }
                ) || matches!(
                    &second,
                    BezierAlgebraicChordSupportPredicate2::RefinedEndpoint { .. }
                ) {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair-side-kernel",
                        "batched-refined-endpoints",
                    );
                    let result = self.pair_sides_by_refinement(other, true, policy)?;
                    match result {
                        Classification::Decided(BezierAlgebraicChordPairSides2::Complete(
                            first_sides,
                            second_sides,
                        )) => (first_sides, second_sides),
                        Classification::Decided(BezierAlgebraicChordPairSides2::Disjoint) => {
                            return Ok(Classification::Decided(
                                BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                } else {
                    let mut second_sides = [crate::classify::LineSide::On; 2];
                    for (side, point) in second_sides.iter_mut().zip([other.start(), other.end()]) {
                        let result = first.oriented_side(point, policy)?;
                        *side = match result {
                            Classification::Decided(side) => side,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    }
                    let mut first_sides = [crate::classify::LineSide::On; 2];
                    for (side, point) in first_sides.iter_mut().zip([self.start(), self.end()]) {
                        let result = second.oriented_side(point, policy)?;
                        *side = match result {
                            Classification::Decided(side) => side,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    }
                    (first_sides, second_sides)
                };
                (first_sides, second_sides, false)
            };
        let all_on = [crate::classify::LineSide::On; 2];
        if first_sides == all_on || second_sides == all_on {
            let both_on = first_sides == all_on && second_sides == all_on;
            let tangent_cross =
                policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))?;
            if both_on && tangent_cross == Classification::Decided(RealSign::Zero) {
                return self.collinear_chord_intersections(other, policy);
            }

            // Retained intersection-point provenance is an exact incidence
            // fast path, but it can name an ancestral support that no longer
            // matches a rebuilt finite chord.  A nonzero tangent cross (or an
            // asymmetric all-on result) contradicts the inferred line
            // identity.  Recompute all four sides from endpoint geometry,
            // retaining only direct endpoint equality, before deciding
            // the finite pair.
            let (repaired_first, repaired_second) =
                match self.pair_sides_by_refinement(other, false, policy)? {
                    Classification::Decided(BezierAlgebraicChordPairSides2::Complete(
                        first_sides,
                        second_sides,
                    )) => (first_sides, second_sides),
                    Classification::Decided(BezierAlgebraicChordPairSides2::Disjoint) => {
                        return Ok(Classification::Decided(
                            BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            if repaired_first == all_on && repaired_second == all_on {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "geometric-collinearity-after-incidence-conflict",
                );
                return self.collinear_chord_intersections(other, policy);
            }
            first_sides = repaired_first;
            second_sides = repaired_second;
            perpendicular_axis_certificate = false;
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "geometric-refinement-after-incidence-conflict",
            );
        }
        if first_sides == all_on || second_sides == all_on {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-blocker",
                "all-on-after-geometric-check",
            );
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        let misses_support = |sides: [crate::classify::LineSide; 2]| {
            matches!(
                sides,
                [
                    crate::classify::LineSide::Left,
                    crate::classify::LineSide::Left
                ] | [
                    crate::classify::LineSide::Right,
                    crate::classify::LineSide::Right
                ]
            )
        };
        if misses_support(first_sides) || misses_support(second_sides) {
            return Ok(Classification::Decided(
                BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
            ));
        }

        let tangent_cross_sign = match second_sides {
            [
                crate::classify::LineSide::Right,
                crate::classify::LineSide::Left,
            ]
            | [
                crate::classify::LineSide::Right,
                crate::classify::LineSide::On,
            ]
            | [
                crate::classify::LineSide::On,
                crate::classify::LineSide::Left,
            ] => RealSign::Positive,
            [
                crate::classify::LineSide::Left,
                crate::classify::LineSide::Right,
            ]
            | [
                crate::classify::LineSide::Left,
                crate::classify::LineSide::On,
            ]
            | [
                crate::classify::LineSide::On,
                crate::classify::LineSide::Right,
            ] => RealSign::Negative,
            _ => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-blocker",
                    "inconsistent-side-orientation",
                );
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        };

        let first_endpoint = first_sides
            .iter()
            .position(|side| *side == crate::classify::LineSide::On);
        let second_endpoint = second_sides
            .iter()
            .position(|side| *side == crate::classify::LineSide::On);
        let Some(point) = first_endpoint
            .map(|endpoint| [self.start(), self.end()][endpoint].clone())
            .or_else(|| {
                second_endpoint.map(|endpoint| [other.start(), other.end()][endpoint].clone())
            })
        else {
            let point = BezierAlgebraicChordPairPoint2::new(
                self.clone(),
                other.clone(),
                first_sides,
                second_sides,
                tangent_cross_sign,
                policy,
            );
            let point = CurvePoint2::from(point);
            let parameter = |chord: &BezierAlgebraicChord2| BezierAlgebraicChordParameter2 {
                data: BezierAlgebraicChordParameterStorage2::Interior(Arc::new(
                    BezierAlgebraicChordParameterData2 {
                        chord: chord.clone(),
                        point: point.clone(),
                        axis: chord.data.parameter_axis,
                        certified_strict_interior: true,
                    },
                )),
            };
            return Ok(Classification::Decided(
                BezierAlgebraicChordPairIntersections2::Contacts(vec![
                    BezierAlgebraicChordPairContact2 {
                        first_parameter: parameter(self),
                        second_parameter: parameter(other),
                        point,
                        tangent_cross_sign,
                    },
                ]),
            ));
        };
        let first_parameter = if let Some(endpoint) = first_endpoint {
            if endpoint == 0 {
                self.start_parameter()
            } else {
                self.end_parameter()
            }
        } else if perpendicular_axis_certificate {
            // The coordinate-side certificate already proved that this
            // unique support intersection lies inside the finite chord.
            self.parameter_at_certified_interior_point(point.clone())
        } else {
            let parameter = self.parameter_at_certified_point(point.clone(), policy)?;
            match parameter {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => {
                    // The support intersection lies outside this finite
                    // chord. The opposite endpoint incidence identifies the
                    // infinite-line solution, not a segment contact.
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let second_parameter = if let Some(endpoint) = second_endpoint {
            if endpoint == 0 {
                other.start_parameter()
            } else {
                other.end_parameter()
            }
        } else if perpendicular_axis_certificate {
            other.parameter_at_certified_interior_point(point.clone())
        } else {
            let parameter = other.parameter_at_certified_point(point.clone(), policy)?;
            match parameter {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        Ok(Classification::Decided(
            BezierAlgebraicChordPairIntersections2::Contacts(vec![
                BezierAlgebraicChordPairContact2 {
                    first_parameter,
                    second_parameter,
                    point,
                    tangent_cross_sign,
                },
            ]),
        ))
    }

    /// Intersects nonparallel retained supports while preserving the exact
    /// order of their common offset-miter point from one endpoint on each.
    ///
    /// The caller's two orders are construction facts from equal signed
    /// offsets of adjacent source tangents. They avoid four unrelated
    /// support-side predicates, while the retained pair still owns the same
    /// complete two-line intersection and can answer arbitrary later queries.
    pub(crate) fn supporting_line_intersection_with_certified_anchor_orders(
        &self,
        other: &Self,
        first_anchor_at_end: bool,
        first_order: std::cmp::Ordering,
        second_anchor_at_end: bool,
        second_order: std::cmp::Ordering,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        if first_order == std::cmp::Ordering::Equal || second_order == std::cmp::Ordering::Equal {
            return Err(CurveError::Topology(
                "a nonparallel offset miter had zero endpoint displacement".into(),
            ));
        }
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        let tangent_cross_sign = match self.tangent_cross_sign(other, policy)? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-support-intersection",
            "certified-offset-anchor-orders",
        );
        Ok(Classification::Decided(Some(CurvePoint2::from(
            BezierAlgebraicChordPairPoint2::new_with_anchor_orders(
                self.clone(),
                other.clone(),
                first_anchor_at_end,
                first_order,
                second_anchor_at_end,
                second_order,
                tangent_cross_sign,
                policy,
            ),
        ))))
    }

    /// Intersects the two infinite retained supporting lines.
    ///
    /// Unlike [`Self::chord_intersections`], this construction deliberately
    /// does not impose either finite endpoint range. Exact offset miters use
    /// the tangent supports and may lie arbitrarily far beyond the short
    /// witness chord used to retain a procedural tangent. The returned point
    /// keeps both support fields separate and refines the same two-line
    /// determinant authority as an ordinary chord contact.
    pub(crate) fn supporting_line_intersection(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        let tangent_cross_sign = match self.tangent_cross_sign(other, policy)? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let first = match BezierAlgebraicChordSupportPredicate2::try_new(self, policy)? {
            Classification::Decided(predicate) => predicate,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match BezierAlgebraicChordSupportPredicate2::try_new(other, policy)? {
            Classification::Decided(predicate) => predicate,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut first_sides = [crate::classify::LineSide::On; 2];
        for (side, point) in first_sides.iter_mut().zip([self.start(), self.end()]) {
            *side = match second.oriented_side(point, policy)? {
                Classification::Decided(side) => side,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        let mut second_sides = [crate::classify::LineSide::On; 2];
        for (side, point) in second_sides.iter_mut().zip([other.start(), other.end()]) {
            *side = match first.oriented_side(point, policy)? {
                Classification::Decided(side) => side,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        Ok(Classification::Decided(Some(CurvePoint2::from(
            BezierAlgebraicChordPairPoint2::new(
                self.clone(),
                other.clone(),
                first_sides,
                second_sides,
                tangent_cross_sign,
                policy,
            ),
        ))))
    }

    /// Classifies a retained point against this chord's oriented supporting
    /// line through the same exact predicate selected by chord incidence.
    pub(crate) fn oriented_support_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        let support = match BezierAlgebraicChordSupportPredicate2::try_new(self, policy)? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        support.oriented_side(point, policy)
    }

    pub(in crate::bezier_offset) fn collinear_chord_intersections(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordPairIntersections2>> {
        #[derive(Clone)]
        struct Boundary {
            point: CurvePoint2,
            first_parameter: Option<BezierAlgebraicChordParameter2>,
            second_parameter: Option<BezierAlgebraicChordParameter2>,
        }

        let certified_axis = self
            .certified_axis_direction()
            .zip(other.certified_axis_direction())
            .filter(|(first, second)| first.axis() == second.axis())
            .map(|(first, _)| first.axis());
        let coordinate_order = |first: &CurvePoint2, second: &CurvePoint2| {
            if let Some(axis) = certified_axis {
                // Cardinal collinearity already identifies the shared
                // affine coordinate. Compare that coordinate directly;
                // a parameter's pair-contact anchor order belongs to its
                // constructing finite chord and is not an ordering proof
                // on a different collinear descendant.
                return Self::point_axis_order(first, second, axis, policy);
            }
            let first = self.parameter_on_retained_support(first.clone());
            let second = self.parameter_on_retained_support(second.clone());
            first
                .cmp_by_refinement(&second, policy)
                .map(|classification| {
                    if self.data.parameter_axis.coordinate_increases {
                        classification
                    } else {
                        classification.map(std::cmp::Ordering::reverse)
                    }
                })
        };
        let first_start = Boundary {
            point: self.start().clone(),
            first_parameter: Some(self.start_parameter()),
            second_parameter: None,
        };
        let first_end = Boundary {
            point: self.end().clone(),
            first_parameter: Some(self.end_parameter()),
            second_parameter: None,
        };
        let (first_low, first_high) = if self.data.parameter_axis.coordinate_increases {
            (first_start, first_end)
        } else {
            (first_end, first_start)
        };
        // Collinearity already proves that the two nondegenerate traversal
        // tangents are parallel.  Their exact dot sign therefore determines
        // the second chord's coordinate orientation without re-adjoining both
        // endpoints to this chord's retained support field.  In particular,
        // exact parallel-offset bands commonly retain the same represented
        // unit-tangent authority even when a cross-field endpoint comparison
        // cannot prove equality under STRICT.
        let tangent_dot_sign = if self.shares_retained_support(other)
            && self.data.parameter_axis.axis == other.data.parameter_axis.axis
        {
            Classification::Decided(
                if self.data.parameter_axis.coordinate_increases
                    == other.data.parameter_axis.coordinate_increases
                {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                },
            )
        } else {
            policy.strict_predicate_pass(|| self.tangent_dot_sign(other, policy))?
        };
        let second_increases = match tangent_dot_sign {
            Classification::Decided(RealSign::Positive) => {
                self.data.parameter_axis.coordinate_increases
            }
            Classification::Decided(RealSign::Negative) => {
                !self.data.parameter_axis.coordinate_increases
            }
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-collinear-range",
            "exact-tangent-orientation",
        );
        let second_order = if second_increases {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        };
        let second_start = Boundary {
            point: other.start().clone(),
            first_parameter: None,
            second_parameter: Some(other.start_parameter()),
        };
        let second_end = Boundary {
            point: other.end().clone(),
            first_parameter: None,
            second_parameter: Some(other.end_parameter()),
        };
        let (second_low, second_high) = if second_order == std::cmp::Ordering::Less {
            (second_start, second_end)
        } else {
            (second_end, second_start)
        };
        let merged = |first: Boundary, second: Boundary| Boundary {
            point: first.point,
            first_parameter: first.first_parameter.or(second.first_parameter),
            second_parameter: first.second_parameter.or(second.second_parameter),
        };
        let mut low = match coordinate_order(&first_low.point, &second_low.point)? {
            Classification::Decided(std::cmp::Ordering::Less) => second_low,
            Classification::Decided(std::cmp::Ordering::Equal) => merged(first_low, second_low),
            Classification::Decided(std::cmp::Ordering::Greater) => first_low,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut high = match coordinate_order(&first_high.point, &second_high.point)? {
            Classification::Decided(std::cmp::Ordering::Less) => first_high,
            Classification::Decided(std::cmp::Ordering::Equal) => merged(first_high, second_high),
            Classification::Decided(std::cmp::Ordering::Greater) => second_high,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let parameter_order = |first: &BezierAlgebraicChordParameter2,
                               second: &BezierAlgebraicChordParameter2,
                               coordinate_increases: bool| {
            first
                .cmp_by_refinement(second, policy)
                .map(|classification| {
                    if coordinate_increases {
                        classification
                    } else {
                        classification.map(std::cmp::Ordering::reverse)
                    }
                })
        };
        let overlap_order = if let (Some(first), Some(second)) =
            (&low.first_parameter, &high.first_parameter)
        {
            parameter_order(first, second, self.data.parameter_axis.coordinate_increases)?
        } else if let (Some(first), Some(second)) = (&low.second_parameter, &high.second_parameter)
        {
            parameter_order(first, second, second_increases)?
        } else {
            coordinate_order(&low.point, &high.point)?
        };
        let overlap_order = match overlap_order {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if overlap_order == std::cmp::Ordering::Greater {
            return Ok(Classification::Decided(
                BezierAlgebraicChordPairIntersections2::Contacts(Vec::new()),
            ));
        }
        if overlap_order == std::cmp::Ordering::Equal {
            // The maximum lower boundary and minimum upper boundary are the
            // same point.  Together they already own an endpoint parameter
            // on each chord; merging those authorities avoids replaying the
            // cross-field point through either finite-range predicate.
            let contact = merged(low, high);
            let first_parameter = if let Some(parameter) = contact.first_parameter {
                parameter
            } else {
                match self.parameter_at_certified_point(contact.point.clone(), policy)? {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let second_parameter = if let Some(parameter) = contact.second_parameter {
                parameter
            } else {
                match other.parameter_at_certified_point(contact.point.clone(), policy)? {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            return Ok(Classification::Decided(
                BezierAlgebraicChordPairIntersections2::Contacts(vec![
                    BezierAlgebraicChordPairContact2 {
                        first_parameter,
                        second_parameter,
                        point: contact.point,
                        tangent_cross_sign: RealSign::Zero,
                    },
                ]),
            ));
        }
        for boundary in [&mut low, &mut high] {
            if boundary.first_parameter.is_none() {
                // A strict, nonempty overlap places a boundary borrowed from
                // the second chord strictly inside the first interval: equal
                // endpoints were merged above and disjoint intervals already
                // returned.  The interval proof is the finite-range
                // certificate; do not repeat it in another algebraic field.
                boundary.first_parameter =
                    Some(self.parameter_at_certified_interior_point(boundary.point.clone()));
            }
            if boundary.second_parameter.is_none() {
                boundary.second_parameter =
                    Some(other.parameter_at_certified_interior_point(boundary.point.clone()));
            }
        }
        let first_low = low
            .first_parameter
            .expect("the first low boundary was mapped");
        let first_high = high
            .first_parameter
            .expect("the first high boundary was mapped");
        let second_low = low
            .second_parameter
            .expect("the second low boundary was mapped");
        let second_high = high
            .second_parameter
            .expect("the second high boundary was mapped");
        let same_direction = self.data.parameter_axis.coordinate_increases
            == (second_order == std::cmp::Ordering::Less);
        Ok(Classification::Decided(
            BezierAlgebraicChordPairIntersections2::Overlaps(vec![
                BezierAlgebraicChordPairOverlap2 {
                    first_range: if self.data.parameter_axis.coordinate_increases {
                        [first_low, first_high]
                    } else {
                        [first_high, first_low]
                    },
                    second_range: if self.data.parameter_axis.coordinate_increases {
                        [second_low, second_high]
                    } else {
                        [second_high, second_low]
                    },
                    orientation: if same_direction {
                        CurveOverlapOrientation2::Same
                    } else {
                        CurveOverlapOrientation2::Reversed
                    },
                },
            ]),
        ))
    }

    pub(crate) fn collinear_rational_intersections(
        &self,
        source: &RationalBezier2,
        range: &CurveParameterRange2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalIntersections2>> {
        self.validate_policy(policy)?;
        if range == &CurveParameterRange2::unit() {
            match source.denominator_sign(&crate::CurveParameterRange2::unit()) {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let axis = self.data.parameter_axis.axis;
        if range != &CurveParameterRange2::unit()
            || !source.has_certified_injective_axis_on(axis, policy)
        {
            return self.collinear_partitioned_rational_intersections(
                source,
                range,
                excluded_source_parameter,
                None,
                policy,
            );
        }

        let source_start_parameter = BezierParameter2::Exact(Real::zero());
        let source_end_parameter = BezierParameter2::Exact(Real::one());
        let source_start_point = CurvePoint2::from(source.start().clone());
        let source_end_point = CurvePoint2::from(source.end().clone());
        let source_order =
            match self.point_parameter_order(&source_start_point, &source_end_point, policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => std::cmp::Ordering::Less,
                Classification::Decided(std::cmp::Ordering::Greater) => std::cmp::Ordering::Greater,
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let (
            source_lower_parameter,
            source_lower_point,
            source_upper_parameter,
            source_upper_point,
        ) = if source_order == std::cmp::Ordering::Less {
            (
                &source_start_parameter,
                &source_start_point,
                &source_end_parameter,
                &source_end_point,
            )
        } else {
            (
                &source_end_parameter,
                &source_end_point,
                &source_start_parameter,
                &source_start_point,
            )
        };
        match self.point_parameter_order(self.start(), source_upper_point, policy)? {
            Classification::Decided(std::cmp::Ordering::Greater) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
                ));
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return self.collinear_point_contact(
                    self.start_parameter(),
                    source_upper_parameter.clone().into(),
                    self.start().clone(),
                    excluded_source_parameter,
                    policy,
                );
            }
            Classification::Decided(std::cmp::Ordering::Less) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match self.point_parameter_order(source_lower_point, self.end(), policy)? {
            Classification::Decided(std::cmp::Ordering::Greater) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
                ));
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return self.collinear_point_contact(
                    self.end_parameter(),
                    source_lower_parameter.clone().into(),
                    self.end().clone(),
                    excluded_source_parameter,
                    policy,
                );
            }
            Classification::Decided(std::cmp::Ordering::Less) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let lower = match self.point_parameter_order(self.start(), source_lower_point, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => self
                .collinear_boundary_from_source_endpoint(
                    source_lower_parameter.clone(),
                    source_lower_point.clone(),
                    policy,
                )?,
            Classification::Decided(std::cmp::Ordering::Equal) => {
                Classification::Decided(BezierAlgebraicChordRationalBoundary2 {
                    chord_parameter: self.start_parameter(),
                    source_parameter: CurveParameter2::from(source_lower_parameter.clone()),
                    point: self.start().clone(),
                })
            }
            Classification::Decided(std::cmp::Ordering::Greater) => self
                .collinear_boundary_from_chord_endpoint(
                    source,
                    self.start_parameter(),
                    self.start().clone(),
                    policy,
                )?,
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        };
        let lower = match lower {
            Classification::Decided(boundary) => boundary,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let upper = match self.point_parameter_order(self.end(), source_upper_point, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => self
                .collinear_boundary_from_chord_endpoint(
                    source,
                    self.end_parameter(),
                    self.end().clone(),
                    policy,
                )?,
            Classification::Decided(std::cmp::Ordering::Equal) => {
                Classification::Decided(BezierAlgebraicChordRationalBoundary2 {
                    chord_parameter: self.end_parameter(),
                    source_parameter: CurveParameter2::from(source_upper_parameter.clone()),
                    point: self.end().clone(),
                })
            }
            Classification::Decided(std::cmp::Ordering::Greater) => self
                .collinear_boundary_from_source_endpoint(
                    source_upper_parameter.clone(),
                    source_upper_point.clone(),
                    policy,
                )?,
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        };
        let upper = match upper {
            Classification::Decided(boundary) => boundary,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let overlap_order = match lower
            .chord_parameter
            .cmp_by_refinement(&upper.chord_parameter, policy)?
        {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match overlap_order {
            std::cmp::Ordering::Greater => Ok(Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
            )),
            std::cmp::Ordering::Equal => {
                if let Some(excluded) = excluded_source_parameter {
                    match lower.source_parameter.cmp_by_refinement(excluded, policy)? {
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            return Ok(Classification::Decided(
                                BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
                            ));
                        }
                        Classification::Decided(_) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Ok(Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::Contacts(vec![
                        BezierAlgebraicChordRationalContact2 {
                            chord_parameter: lower.chord_parameter,
                            other_parameter: lower.source_parameter,
                            point: lower.point,
                            tangent_cross_sign: RealSign::Zero,
                        },
                    ]),
                ))
            }
            std::cmp::Ordering::Less => {
                let source_order = match lower
                    .source_parameter
                    .cmp_by_refinement(&upper.source_parameter, policy)?
                {
                    Classification::Decided(std::cmp::Ordering::Less) => std::cmp::Ordering::Less,
                    Classification::Decided(std::cmp::Ordering::Greater) => {
                        std::cmp::Ordering::Greater
                    }
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        return Ok(Classification::Decided(
                            BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::Overlaps(vec![
                        BezierAlgebraicChordRationalOverlap2 {
                            chord: self.clone(),
                            source: source.clone(),
                            chord_range: [lower.chord_parameter, upper.chord_parameter],
                            source_range: CurveParameterRange2::new_validated(
                                lower.source_parameter,
                                upper.source_parameter,
                            ),
                            orientation: if source_order == std::cmp::Ordering::Less {
                                CurveOverlapOrientation2::Same
                            } else {
                                CurveOverlapOrientation2::Reversed
                            },
                        },
                    ]),
                ))
            }
        }
    }

    /// Clips one certified regular line component through the common
    /// collinear partition engine.
    ///
    /// The component is constructed only after STRICT proves a constant
    /// nonzero tangent direction on its regular source range. Its retained
    /// endpoints, including algebraic roots, therefore form the complete
    /// monotone partition. Opaque global leading coefficients never need to
    /// be normalized merely to rediscover that selected local branch.
    pub(crate) fn collinear_rational_intersections_on_regular_component(
        &self,
        component: &BezierParallelRationalComponent2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalIntersections2>> {
        self.validate_policy(policy)?;
        let strict = policy.strict_counterpart();
        let source = component.curve();
        let range = component.regular_range();
        let Some(support_line) = component.support_line() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        // Suppress the terminal without changing the retained object's policy
        // identity. An APPROXIMATE_512 chord can be replayed by its owning
        // operation while this structural collinearity probe remains STRICT.
        match policy.strict_predicate_pass(|| {
            self.has_non_collinear_support_with_exact_line(support_line, policy)
        })? {
            Classification::Decided(false) => {}
            Classification::Decided(true) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let support_axis_delta = match self.data.parameter_axis.axis {
            Axis2::X => support_line.end().x() - support_line.start().x(),
            Axis2::Y => support_line.end().y() - support_line.start().y(),
        };
        let source_order = match real_sign(&support_axis_delta, &strict) {
            Some(RealSign::Positive) => std::cmp::Ordering::Less,
            Some(RealSign::Negative) => std::cmp::Ordering::Greater,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a certified regular line was constant on its chord parameter axis".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        match polynomial_is_nonzero_on_parameter_range(
            &source.homogeneous_power_basis()?.weight,
            range,
            &strict,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let bounds = match range.start().cmp_by_refinement(range.end(), &strict)? {
            Classification::Decided(std::cmp::Ordering::Less) => {
                [range.start().clone(), range.end().clone()]
            }
            Classification::Decided(std::cmp::Ordering::Greater) => {
                [range.end().clone(), range.start().clone()]
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-collinear-range",
            "certified-regular-line-range",
        );
        policy.strict_predicate_pass(|| {
            self.collinear_partitioned_rational_intersections(
                source,
                &CurveParameterRange2::new_validated(bounds[0].clone(), bounds[1].clone()),
                excluded_source_parameter,
                Some(source_order),
                policy,
            )
        })
    }

    pub(in crate::bezier_offset) fn collinear_partitioned_rational_intersections(
        &self,
        source: &RationalBezier2,
        range: &CurveParameterRange2,
        excluded_source_parameter: Option<&CurveParameter2>,
        certified_monotone: Option<std::cmp::Ordering>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalIntersections2>> {
        let parameter_bounds = match range.ordered_endpoints(policy)? {
            Classification::Decided(bounds) => bounds.map(Clone::clone),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut boundaries = vec![
            BezierAlgebraicChordRationalPartitionBoundary2 {
                source_parameter: parameter_bounds[0].clone(),
                chord_endpoint_at_end: None,
            },
            BezierAlgebraicChordRationalPartitionBoundary2 {
                source_parameter: parameter_bounds[1].clone(),
                chord_endpoint_at_end: None,
            },
        ];
        if certified_monotone.is_none() {
            let source_power = source.homogeneous_power_basis()?;
            let [derivative_x, derivative_y] = rational_parametric_tangent_numerator(source_power);
            let derivative = match self.data.parameter_axis.axis {
                Axis2::X => derivative_x,
                Axis2::Y => derivative_y,
            };
            match polynomial_coefficients_are_identically_zero(&derivative, policy) {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                    ));
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let derivative = match BezierParameterPolynomial::try_new_power_basis_with_policy(
                derivative, policy,
            )? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match CurveParameterDomain2::new(range, None).finite_roots(&derivative, policy)? {
                Classification::Decided(parameters) => {
                    boundaries.extend(parameters.into_iter().map(|source_parameter| {
                        BezierAlgebraicChordRationalPartitionBoundary2 {
                            source_parameter: CurveParameter2::from(source_parameter),
                            chord_endpoint_at_end: None,
                        }
                    }))
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        for (at_end, point) in [(false, self.start()), (true, self.end())] {
            let parameters = if let Some(source_order) = certified_monotone {
                match self.collinear_monotone_source_parameter_at_chord_endpoint(
                    source,
                    point,
                    &parameter_bounds,
                    source_order,
                    policy,
                )? {
                    Classification::Decided(Some(parameter)) => {
                        Classification::Decided(vec![parameter])
                    }
                    Classification::Decided(None) => Classification::Decided(Vec::new()),
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            } else {
                self.collinear_source_parameters_at_chord_endpoint(source, point, range, policy)?
            };
            match parameters {
                Classification::Decided(parameters) => {
                    boundaries.extend(parameters.into_iter().map(|source_parameter| {
                        BezierAlgebraicChordRationalPartitionBoundary2 {
                            source_parameter,
                            chord_endpoint_at_end: Some(at_end),
                        }
                    }))
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        let boundaries = match sort_and_dedup_collinear_partition_boundaries(boundaries, policy)? {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut covered = vec![false; boundaries.len()];
        let mut overlaps = Vec::new();
        for (index, pair) in boundaries.windows(2).enumerate() {
            let sample = match pair[0]
                .source_parameter
                .strict_scalar_between_ordered(&pair[1].source_parameter, policy)?
            {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let sample_point =
                CurvePoint2::from(match source.point_at_affine_classified(&sample, policy) {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                });
            match self.parameter_at_certified_point(sample_point, policy)? {
                Classification::Decided(Some(_)) => {}
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let first =
                match self.collinear_boundary_from_partition_boundary(source, &pair[0], policy)? {
                    Classification::Decided(Some(boundary)) => boundary,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let second =
                match self.collinear_boundary_from_partition_boundary(source, &pair[1], policy)? {
                    Classification::Decided(Some(boundary)) => boundary,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let order = match first
                .chord_parameter
                .cmp_by_refinement(&second.chord_parameter, policy)?
            {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (chord_range, source_range, orientation) = match order {
                std::cmp::Ordering::Less => (
                    [first.chord_parameter, second.chord_parameter],
                    CurveParameterRange2::new_validated(
                        first.source_parameter,
                        second.source_parameter,
                    ),
                    CurveOverlapOrientation2::Same,
                ),
                std::cmp::Ordering::Greater => (
                    [second.chord_parameter, first.chord_parameter],
                    CurveParameterRange2::new_validated(
                        second.source_parameter,
                        first.source_parameter,
                    ),
                    CurveOverlapOrientation2::Reversed,
                ),
                std::cmp::Ordering::Equal => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            };
            covered[index] = true;
            covered[index + 1] = true;
            overlaps.push(BezierAlgebraicChordRationalOverlap2 {
                chord: self.clone(),
                source: source.clone(),
                chord_range,
                source_range,
                orientation,
            });
        }

        let mut contacts = Vec::new();
        for (covered, boundary) in covered.into_iter().zip(boundaries) {
            if covered {
                continue;
            }
            if let Some(excluded) = excluded_source_parameter {
                match boundary
                    .source_parameter
                    .cmp_by_refinement(excluded, policy)?
                {
                    Classification::Decided(std::cmp::Ordering::Equal) => continue,
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let boundary =
                match self.collinear_boundary_from_partition_boundary(source, &boundary, policy)? {
                    Classification::Decided(Some(boundary)) => boundary,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            contacts.push(BezierAlgebraicChordRationalContact2 {
                chord_parameter: boundary.chord_parameter,
                other_parameter: boundary.source_parameter,
                point: boundary.point,
                tangent_cross_sign: RealSign::Zero,
            });
        }
        Ok(Classification::Decided(if overlaps.is_empty() {
            BezierAlgebraicChordRationalIntersections2::Contacts(contacts)
        } else if contacts.is_empty() {
            BezierAlgebraicChordRationalIntersections2::Overlaps(overlaps)
        } else {
            BezierAlgebraicChordRationalIntersections2::ContactsAndOverlaps { contacts, overlaps }
        }))
    }

    pub(in crate::bezier_offset) fn collinear_point_contact(
        &self,
        chord_parameter: BezierAlgebraicChordParameter2,
        source_parameter: CurveParameter2,
        point: CurvePoint2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalIntersections2>> {
        if let Some(excluded) = excluded_source_parameter {
            match source_parameter.cmp_by_refinement(excluded, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
                    ));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(
            BezierAlgebraicChordRationalIntersections2::Contacts(vec![
                BezierAlgebraicChordRationalContact2 {
                    chord_parameter,
                    other_parameter: source_parameter,
                    point,
                    tangent_cross_sign: RealSign::Zero,
                },
            ]),
        ))
    }

    pub(in crate::bezier_offset) fn point_parameter_order(
        &self,
        first: &CurvePoint2,
        second: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let order = algebraic_chord_point_coordinate_order(
            first,
            second,
            self.data.parameter_axis.axis,
            policy,
        )?;
        Ok(if self.data.parameter_axis.coordinate_increases {
            order
        } else {
            order.map(std::cmp::Ordering::reverse)
        })
    }

    pub(in crate::bezier_offset) fn collinear_boundary_from_source_endpoint(
        &self,
        source_parameter: BezierParameter2,
        point: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalBoundary2>> {
        let chord_parameter = match self.parameter_at_certified_point(point.clone(), policy)? {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            BezierAlgebraicChordRationalBoundary2 {
                chord_parameter,
                source_parameter: CurveParameter2::from(source_parameter),
                point,
            },
        ))
    }

    pub(in crate::bezier_offset) fn collinear_boundary_from_partition_boundary(
        &self,
        source: &RationalBezier2,
        boundary: &BezierAlgebraicChordRationalPartitionBoundary2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordRationalBoundary2>>> {
        if let Some(at_end) = boundary.chord_endpoint_at_end {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicChordRationalBoundary2 {
                    chord_parameter: if at_end {
                        self.end_parameter()
                    } else {
                        self.start_parameter()
                    },
                    source_parameter: boundary.source_parameter.clone(),
                    point: if at_end {
                        self.end().clone()
                    } else {
                        self.start().clone()
                    },
                },
            )));
        }
        let source_parameter = &boundary.source_parameter;
        let point =
            match rational_point_evidence_at_region_parameter(source, source_parameter, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        Ok(
            match self.parameter_at_certified_point(point.clone(), policy)? {
                Classification::Decided(Some(chord_parameter)) => {
                    Classification::Decided(Some(BezierAlgebraicChordRationalBoundary2 {
                        chord_parameter,
                        source_parameter: source_parameter.clone(),
                        point,
                    }))
                }
                Classification::Decided(None) => Classification::Decided(None),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    pub(in crate::bezier_offset) fn collinear_boundary_from_chord_endpoint(
        &self,
        source: &RationalBezier2,
        chord_parameter: BezierAlgebraicChordParameter2,
        point: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalBoundary2>> {
        let source_parameters = match self.collinear_source_parameters_at_chord_endpoint(
            source,
            &point,
            &CurveParameterRange2::unit(),
            policy,
        )? {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [source_parameter] = source_parameters.as_slice() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        };
        Ok(Classification::Decided(
            BezierAlgebraicChordRationalBoundary2 {
                chord_parameter,
                source_parameter: source_parameter.clone(),
                point,
            },
        ))
    }

    /// Inverts one chord endpoint on a retained source range whose chosen axis
    /// is already certified injective.
    ///
    /// Strict point/axis comparisons either recover an exact dyadic cut or
    /// retain the unique local root in its authored selected field.  The
    /// injectivity certificate proves the root count, so construction never
    /// depends on a possibly vanishing global leading coefficient.
    pub(in crate::bezier_offset) fn collinear_monotone_source_parameter_at_chord_endpoint(
        &self,
        source: &RationalBezier2,
        point: &CurvePoint2,
        bounds: &[CurveParameter2; 2],
        source_order: std::cmp::Ordering,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        let strict = policy.strict_counterpart();
        let axis = self.data.parameter_axis.axis;
        let point_at = |parameter: &CurveParameter2| {
            rational_point_evidence_at_region_parameter(source, parameter, &strict)
        };
        let lower_point = match point_at(&bounds[0])? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let upper_point = match point_at(&bounds[1])? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        debug_assert_ne!(source_order, std::cmp::Ordering::Equal);
        let lower_order =
            match algebraic_chord_point_coordinate_order(point, &lower_point, axis, &strict)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if lower_order == std::cmp::Ordering::Equal {
            return Ok(Classification::Decided(Some(bounds[0].clone())));
        }
        let upper_order =
            match algebraic_chord_point_coordinate_order(point, &upper_point, axis, &strict)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if upper_order == std::cmp::Ordering::Equal {
            return Ok(Classification::Decided(Some(bounds[1].clone())));
        }
        let target_is_inside = match source_order {
            std::cmp::Ordering::Less => {
                lower_order == std::cmp::Ordering::Greater
                    && upper_order == std::cmp::Ordering::Less
            }
            std::cmp::Ordering::Greater => {
                lower_order == std::cmp::Ordering::Less
                    && upper_order == std::cmp::Ordering::Greater
            }
            std::cmp::Ordering::Equal => unreachable!("the source axis is injective"),
        };
        if !target_is_inside {
            return Ok(Classification::Decided(None));
        }

        if let CurvePoint2(CurvePointData2::Exact(point)) = point {
            let power = source.homogeneous_power_basis()?;
            let (coordinate, numerator) = match axis {
                Axis2::X => (point.x(), &power.x_numerator),
                Axis2::Y => (point.y(), &power.y_numerator),
            };
            let polynomial = match BezierParameterPolynomial::try_new_power_basis_with_policy(
                polynomial_subtract(numerator, &polynomial_scale(&power.weight, coordinate)),
                &strict,
            )? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let range = CurveParameterRange2::new_validated(bounds[0].clone(), bounds[1].clone());
            let roots = match CurveParameterDomain2::new(&range, None)
                .finite_roots(&polynomial, &strict)?
            {
                Classification::Decided(roots) => roots,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let [root] = roots.as_slice() else {
                return Err(CurveError::Topology(
                    "a certified monotone chord inverse did not retain its unique parameter".into(),
                ));
            };
            return Ok(Classification::Decided(Some(root.clone().into())));
        }

        let mut lower = bounds[0].clone();
        let mut upper = bounds[1].clone();
        let mut refinement_steps = 0_usize;
        let (lower_bracket, upper_bracket) = loop {
            if refinement_steps >= 16
                && let (Some(lower), Some(upper)) = (lower.scalar(), upper.scalar())
            {
                break (lower.clone(), upper.clone());
            }
            let midpoint = match lower.strict_scalar_between_ordered(&upper, &strict)? {
                Classification::Decided(midpoint) => midpoint,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let midpoint_parameter = CurveParameter2::from(midpoint);
            let midpoint_point = match point_at(&midpoint_parameter)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let order = match algebraic_chord_point_coordinate_order(
                point,
                &midpoint_point,
                axis,
                &strict,
            )? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if order == std::cmp::Ordering::Equal {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-collinear-endpoint",
                    "exact-monotone-inverse",
                );
                return Ok(Classification::Decided(Some(midpoint_parameter)));
            }
            let root_is_after_midpoint = match source_order {
                std::cmp::Ordering::Less => order == std::cmp::Ordering::Greater,
                std::cmp::Ordering::Greater => order == std::cmp::Ordering::Less,
                std::cmp::Ordering::Equal => unreachable!("the source axis is injective"),
            };
            if root_is_after_midpoint {
                lower = midpoint_parameter;
            } else {
                upper = midpoint_parameter;
            }
            refinement_steps = refinement_steps.checked_add(1).ok_or_else(|| {
                CurveError::Topology("monotone endpoint refinement overflow".into())
            })?;
        };

        let CurvePoint2(CurvePointData2::Algebraic(point)) = point else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let (Some(parameter), Some((point_x, point_y, point_weight))) = (
            point.retained_parameter(),
            point.retained_coordinate_polynomials(),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let source_power = source.homogeneous_power_basis()?;
        let (point_axis, source_axis) = match axis {
            Axis2::X => (point_x, &source_power.x_numerator),
            Axis2::Y => (point_y, &source_power.y_numerator),
        };
        let incidence = bivariate_subtract(
            &bivariate_outer_product(point_weight, source_axis),
            &bivariate_outer_product(point_axis, &source_power.weight),
        );
        let selected =
            BezierAlgebraicSelectedFiberAuthority2::new(incidence, parameter.clone(), &strict)
                .parameter(IsolatedRootInterval {
                    lower: lower_bracket,
                    upper: upper_bracket,
                    exact_root: None,
                    distinct_root_count: 1,
                });
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-collinear-endpoint",
            "retained-monotone-inverse",
        );
        Ok(Classification::Decided(Some(
            CurveParameter2::from_selected_fiber(selected),
        )))
    }

    pub(in crate::bezier_offset) fn collinear_source_parameters_at_chord_endpoint(
        &self,
        source: &RationalBezier2,
        point: &CurvePoint2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveParameter2>>> {
        if !matches!(point, CurvePoint2(CurvePointData2::Exact(_))) {
            match recursive_projective_point_rational_axis_parameters(
                point,
                source,
                self.data.parameter_axis.axis,
                range,
                policy,
            )? {
                Classification::Decided(Some(parameters)) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-collinear-endpoint",
                        "recursive-projective",
                    );
                    return Ok(Classification::Decided(parameters));
                }
                Classification::Decided(None)
                | Classification::Uncertain(UncertaintyReason::Unsupported) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let represented_point = match point {
            CurvePoint2(CurvePointData2::Exact(point)) => Some(point.clone()),
            CurvePoint2(CurvePointData2::Algebraic(point)) => point.exact_point(policy),
            CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
                match point.exact_represented_point(policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        if let Some(point) = represented_point {
            return Ok(
                match source.point_incidence_on_range(&point, range, policy)? {
                    Classification::Decided(crate::RationalBezierPointIncidence2::Parameters(
                        parameters,
                    )) => Classification::Decided(
                        parameters.into_iter().map(CurveParameter2::from).collect(),
                    ),
                    Classification::Decided(crate::RationalBezierPointIncidence2::EntireCurve) => {
                        Classification::Uncertain(UncertaintyReason::Boundary)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    }

    pub(in crate::bezier_offset) fn independent_support_system(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordIndependentSupport2>> {
        self.validate_policy(policy)?;
        // Splitting changes only the finite interval on one retained support.
        // Build the supporting-line equation from that stable root chord so
        // correlated split endpoints remain compact boundary evidence rather
        // than becoming artificial incidence fields.  Preserve local
        // traversal orientation because the tangent-cross sign belongs to the
        // split chord, not merely to its unoriented support.
        let support = self.retained_support();
        let [support_start, support_end] = if self.retained_support_orientation_is_reversed() {
            [support.end(), support.start()]
        } else {
            [support.start(), support.end()]
        };
        let (support_start, support_end) = match (support_start, support_end) {
            (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) if start.shares_carrier(end) && start.at_end != end.at_end => {
                if start.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                (start.source_endpoint(), end.source_endpoint())
            }
            (start, end) => (start, end),
        };
        let [start, end] =
            match algebraic_chord_endpoint_images(support_start, support_end, policy)? {
                Classification::Decided(endpoints) => endpoints,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let start = match start.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match end.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (start_x, start_y, start_denominator) = start.coordinate_polynomials();
        let (end_x, end_y, end_denominator) = end.coordinate_polynomials();
        let start_denominator_sign = start.denominator_sign();
        let end_denominator_sign = end.denominator_sign();
        let line_x = bivariate_subtract(
            &bivariate_outer_product(start_denominator, end_x),
            &bivariate_outer_product(start_x, end_denominator),
        );
        let line_y = bivariate_subtract(
            &bivariate_outer_product(start_denominator, end_y),
            &bivariate_outer_product(start_y, end_denominator),
        );
        Ok(Classification::Decided(
            BezierAlgebraicChordIndependentSupport2 {
                line_x,
                line_y,
                first_parameter: start.retained_parameter().clone(),
                second_parameter: end.retained_parameter().clone(),
                chord_denominator_sign: product_sign(start_denominator_sign, end_denominator_sign),
            },
        ))
    }

    pub(in crate::bezier_offset) fn source_incidence_system(
        &self,
        source: &RationalBezier2,
        source_parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordSourceIncidence2>> {
        self.validate_policy(policy)?;
        let [start, end] = match algebraic_chord_endpoint_images(self.start(), self.end(), policy)?
        {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start_field = match algebraic_chord_image_parameter(&start, policy)? {
            Classification::Decided(parameter) => BezierParameter2::Algebraic(parameter),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_field = match algebraic_chord_image_parameter(&end, policy)? {
            Classification::Decided(parameter) => BezierParameter2::Algebraic(parameter),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_field = BezierParameter2::Algebraic(source_parameter.clone());
        let same_end_field = start_field.same_value(&end_field, policy)?;
        let same_source_parameter = start_field.same_value(&source_field, policy)?;
        if same_end_field != Classification::Decided(true)
            || same_source_parameter != Classification::Decided(true)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-source-incidence",
                "independent-fields",
            );
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let start_predicate = if self.start().coordinates().is_none() {
            Some(match start.predicate_evaluator(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            })
        } else {
            None
        };
        let end_predicate = if self.end().coordinates().is_none() {
            Some(match end.predicate_evaluator(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            })
        } else {
            None
        };
        let (start_x, start_y, start_weight) = if let Some(point) = &start_predicate {
            point.coordinate_polynomials()
        } else {
            let Some(coordinates) = start.retained_coordinate_polynomials() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            coordinates
        };
        let (end_x, end_y, end_weight) = if let Some(point) = &end_predicate {
            point.coordinate_polynomials()
        } else {
            let Some(coordinates) = end.retained_coordinate_polynomials() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            coordinates
        };
        let source = source.homogeneous_power_basis()?;
        let line_x = polynomial_subtract(
            &polynomial_multiply(end_x, start_weight),
            &polynomial_multiply(start_x, end_weight),
        );
        let line_y = polynomial_subtract(
            &polynomial_multiply(end_y, start_weight),
            &polynomial_multiply(start_y, end_weight),
        );
        let point_delta_x = bivariate_subtract(
            &bivariate_outer_product(start_weight, &source.x_numerator),
            &bivariate_outer_product(start_x, &source.weight),
        );
        let point_delta_y = bivariate_subtract(
            &bivariate_outer_product(start_weight, &source.y_numerator),
            &bivariate_outer_product(start_y, &source.weight),
        );
        let incidence = bivariate_subtract(
            &bivariate_multiply_first_parameter(&point_delta_y, &line_x),
            &bivariate_multiply_first_parameter(&point_delta_x, &line_y),
        );
        let start_denominator_sign = start_predicate
            .as_ref()
            .map_or(RealSign::Positive, |point| point.denominator_sign());
        let end_denominator_sign = end_predicate
            .as_ref()
            .map_or(RealSign::Positive, |point| point.denominator_sign());
        Ok(Classification::Decided(
            BezierAlgebraicChordSourceIncidence2 {
                incidence,
                line_x,
                line_y,
                chord_denominator_sign: product_sign(start_denominator_sign, end_denominator_sign),
            },
        ))
    }

    /// Proves that this chord and one represented line have distinct supports.
    /// Adjacent authored segments on distinct supports can share only their
    /// already-seeded loop endpoint, so no additional intersection event is
    /// required.
    pub(crate) fn has_non_collinear_support_with_exact_line(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        let mut uncertainty = None;
        let delta_x = line.end().x() - line.start().x();
        let delta_y = line.end().y() - line.start().y();
        let coefficient_x = -delta_y;
        let coefficient_y = delta_x;
        for endpoint in [
            self.retained_support().start(),
            self.retained_support().end(),
        ] {
            let side = match endpoint {
                CurvePoint2(CurvePointData2::Exact(point)) => {
                    line.classify_point_with_policy(point, policy)
                }
                CurvePoint2(CurvePointData2::Algebraic(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    algebraic_chord_point_linear_order_to_exact(
                        endpoint,
                        line.start(),
                        &coefficient_x,
                        &coefficient_y,
                        policy,
                    )?
                    .map(|order| match order {
                        std::cmp::Ordering::Less => crate::classify::LineSide::Right,
                        std::cmp::Ordering::Equal => crate::classify::LineSide::On,
                        std::cmp::Ordering::Greater => crate::classify::LineSide::Left,
                    })
                }
            };
            match side {
                Classification::Decided(crate::classify::LineSide::On) => {}
                Classification::Decided(
                    crate::classify::LineSide::Left | crate::classify::LineSide::Right,
                ) => return Ok(Classification::Decided(true)),
                Classification::Uncertain(reason) => {
                    uncertainty.get_or_insert(reason);
                }
            }
        }
        Ok(uncertainty.map_or(Classification::Decided(false), Classification::Uncertain))
    }

    /// Proves that the complete chord lies strictly in one open half-plane of
    /// an exact supporting line.  This is a constant-size rejection path for
    /// replayed correlated cuts whose local endpoint fields must not be
    /// materialized merely to reject a distant line carrier.
    pub(crate) fn is_strictly_one_sided_of_exact_line(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        if [self.start(), self.end()].into_iter().any(|endpoint| {
            !matches!(
                endpoint,
                CurvePoint2(CurvePointData2::Exact(_)) | CurvePoint2(CurvePointData2::Algebraic(_))
            )
        }) {
            // This is only a sufficient broad-phase rejection. Recursive
            // composite endpoints can make even a nominal support-side probe
            // materialize their complete joined field, so leave them to the
            // authoritative chord-intersection kernel below the caller.
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        policy.bounded_exact_predicate_pass(|| {
            let exact = match Self::try_new(
                CurvePoint2::from(line.start().clone()),
                CurvePoint2::from(line.end().clone()),
                policy,
            )? {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let predicate = match BezierAlgebraicChordSupportPredicate2::try_new(&exact, policy)? {
                Classification::Decided(predicate) => predicate,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut sides = [crate::classify::LineSide::On; 2];
            for (side, endpoint) in sides.iter_mut().zip([self.start(), self.end()]) {
                *side = match predicate.oriented_side(endpoint, policy)? {
                    Classification::Decided(side) => side,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
            Ok(Classification::Decided(matches!(
                sides,
                [
                    crate::classify::LineSide::Left,
                    crate::classify::LineSide::Left
                ] | [
                    crate::classify::LineSide::Right,
                    crate::classify::LineSide::Right
                ]
            )))
        })
    }

    /// Proves a complete rational Bezier control hull lies in one open
    /// half-plane of this chord's retained support.
    ///
    /// On the unit chart, equal-sign nonzero homogeneous weights make every
    /// curve point a positive affine combination of the authored controls. Classifying each
    /// control through the chord's existing exact support predicate therefore
    /// excludes the complete curve without projecting the chord endpoints or
    /// building a bivariate intersection resultant.
    pub(crate) fn rational_control_hull_is_strictly_one_sided(
        &self,
        curve: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        policy.bounded_exact_predicate_pass(|| {
            // The authored control hull encloses only the unit chart.
            if !matches!(
                CurveParameterDomain2::new(&CurveParameterRange2::unit(), None)
                    .contains_finite_range(range, policy),
                Ok(Classification::Decided(true))
            ) {
                return Ok(Classification::Decided(false));
            }
            let mut common_weight_sign = None;
            for weight in curve.weights() {
                let Some(sign @ (RealSign::Positive | RealSign::Negative)) =
                    real_sign(weight, policy)
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                };
                if common_weight_sign.is_some_and(|common| common != sign) {
                    return Ok(Classification::Decided(false));
                }
                common_weight_sign = Some(sign);
            }
            let support = match BezierAlgebraicChordSupportPredicate2::try_new(self, policy)? {
                Classification::Decided(support) => support,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut common_side = None;
            let Some(controls) = curve.affine_control_points() else {
                return Ok(Classification::Decided(false));
            };
            for point in controls {
                let evidence = CurvePoint2::from(point.clone());
                let side = match support.oriented_side(&evidence, policy)? {
                    Classification::Decided(
                        side @ (crate::classify::LineSide::Left | crate::classify::LineSide::Right),
                    ) => side,
                    Classification::Decided(crate::classify::LineSide::On) => {
                        return Ok(Classification::Decided(false));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if common_side.is_some_and(|common| common != side) {
                    return Ok(Classification::Decided(false));
                }
                common_side = Some(side);
            }
            Ok(Classification::Decided(common_side.is_some()))
        })
    }
}
