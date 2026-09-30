//! Selected algebraic chord carrier: construction, retained parallels, incidence, contacts and parameters.

use super::*;

impl BezierAlgebraicChord2 {
    /// Constructs a nonzero exact chord from retained affine endpoint evidence.
    ///
    /// Algebraic equality is certified from shared source evidence, disjoint
    /// source bounds, or represented coordinate roots. A decision unavailable
    /// under `policy` is returned as uncertainty; no endpoint is rounded.
    pub fn try_new(
        start: CurvePoint2,
        end: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let equality = start.same_point(&end, policy);
        Self::try_new_with_endpoint_equality(start, end, equality, true, policy)
    }

    /// Retains one oriented source unit tangent at a compact analytic
    /// parameter as an exact chord without projecting that parameter into a
    /// degree-multiplied global polynomial.
    ///
    /// The two endpoints differ by exactly one unit tangent.  Signing the
    /// homogeneous tangent components in the retained scalar's own field is
    /// therefore a complete noncoincidence and monotone-axis certificate.
    /// Those signs are construction evidence and are always proved in a
    /// STRICT predicate pass, even when the surrounding object retains
    /// APPROXIMATE_512 replay authority.
    pub(crate) fn from_certified_retained_parallel_unit_tangent(
        parallel: BezierParallel2,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Self::from_certified_retained_parallel_oriented_unit_tangent(
            parallel,
            parameter,
            RealSign::Positive,
            policy,
        )
    }

    /// Oriented counterpart used when traversal is opposite the source frame.
    pub(crate) fn from_certified_retained_parallel_oriented_unit_tangent(
        parallel: BezierParallel2,
        parameter: &CurveParameter2,
        source_direction: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if source_direction == RealSign::Zero {
            return Err(CurveError::Topology(
                "a retained analytic tangent had zero traversal direction".into(),
            ));
        }
        let parameter = if let Some(parameter) = parameter.as_bezier_parameter() {
            BezierAnalyticParallelPointParameter2::Bezier(parameter.clone())
        } else if let Some(parameter) = parameter.as_selected_fiber() {
            parameter.validate_policy(policy)?;
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter.clone())
        } else if let Some(parameter) = parameter.as_recursive_projective() {
            parameter.validate_policy(policy)?;
            BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter.clone())
        } else {
            return Err(CurveError::Topology(
                "an analytic tangent requires a scalar source parameter".into(),
            ));
        };
        let differential = parallel.differential()?;
        let component_sign = |coefficients: &[Real]| match &parameter {
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => parameter
                .predicate_sign(
                    &bivariate_outer_product(&[Real::one()], coefficients),
                    policy,
                ),
            BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                parameter.polynomial_sign(coefficients, policy)
            }
            BezierAnalyticParallelPointParameter2::Bezier(parameter) => {
                signed_coefficients_at_parameter(coefficients, parameter, policy)
            }
        };
        let orient = |sign| {
            if source_direction == RealSign::Negative {
                match sign {
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => RealSign::Zero,
                    RealSign::Positive => RealSign::Negative,
                }
            } else {
                sign
            }
        };
        let structurally_zero = |coefficients: &[Real]| {
            coefficients
                .iter()
                .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        };
        // One certified nonzero component is a complete monotone-axis proof.
        // Do not solve the complementary component merely to discover an
        // axis-alignment optimization: at a recursive contact that predicate
        // may be much harder than the tangent construction itself.
        let x_sign = policy.strict_predicate_pass(|| component_sign(&differential.tangent_x))?;
        let (parameter_axis, certified_axis_aligned) = match x_sign {
            Classification::Decided(x @ (RealSign::Positive | RealSign::Negative)) => {
                let x = orient(x);
                (
                    BezierAlgebraicChordParameterAxis2 {
                        axis: Axis2::X,
                        coordinate_increases: x == RealSign::Positive,
                    },
                    structurally_zero(&differential.tangent_y),
                )
            }
            x => {
                let y_sign =
                    policy.strict_predicate_pass(|| component_sign(&differential.tangent_y))?;
                match y_sign {
                    Classification::Decided(y @ (RealSign::Positive | RealSign::Negative)) => {
                        let y = orient(y);
                        (
                            BezierAlgebraicChordParameterAxis2 {
                                axis: Axis2::Y,
                                coordinate_increases: y == RealSign::Positive,
                            },
                            x == Classification::Decided(RealSign::Zero)
                                || structurally_zero(&differential.tangent_x),
                        )
                    }
                    Classification::Decided(RealSign::Zero) => match x {
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "a retained analytic tangent frame was singular".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                        Classification::Decided(RealSign::Negative | RealSign::Positive) => {
                            unreachable!("handled before the complementary predicate")
                        }
                    },
                    Classification::Uncertain(reason) => match x {
                        Classification::Uncertain(x_reason) => {
                            return Ok(Classification::Uncertain(
                                if structurally_zero(&differential.tangent_x) {
                                    reason
                                } else {
                                    x_reason
                                },
                            ));
                        }
                        Classification::Decided(RealSign::Zero) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                        Classification::Decided(RealSign::Negative | RealSign::Positive) => {
                            unreachable!("handled before the complementary predicate")
                        }
                    },
                }
            }
        };
        let start = CurvePoint2::from(
            BezierAnalyticParallelPoint2::new_with_tangent_distance_parameter(
                parallel.clone(),
                parameter.clone(),
                Real::zero(),
                policy,
            ),
        );
        let end = CurvePoint2::from(
            BezierAnalyticParallelPoint2::new_with_tangent_distance_parameter(
                parallel,
                parameter,
                Real::from(match source_direction {
                    RealSign::Positive => 1_i8,
                    RealSign::Negative => -1_i8,
                    RealSign::Zero => unreachable!("validated above"),
                }),
                policy,
            ),
        );
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start,
                end,
                parameter_axis,
                certified_axis_aligned,
                certified_unit_tangent: None,
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: None,
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        }))
    }

    /// Constructs a retained chord after the caller has already certified
    /// that its endpoints are distinct.
    ///
    /// This is the exact carrier-switch path: a nonzero tangent cross proves
    /// that equal signed normal offsets from the shared source vertex cannot
    /// coincide, so replaying multi-field coordinate equality would duplicate
    /// that proof and can be substantially more expensive than constructing
    /// the chord itself.
    pub(crate) fn try_new_from_certified_distinct_endpoints(
        start: CurvePoint2,
        end: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Self::try_new_with_endpoint_equality(
            start,
            end,
            Classification::Decided(false),
            false,
            policy,
        )
    }

    /// Represents an already-certified unit direction in the shared chord
    /// normal frame. The origin is immaterial: a frame keeps its center
    /// separately. Retain unit length so metric queries need no new norm.
    pub(crate) fn from_unit_direction(
        direction: &crate::direction::UnitDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let (x, y) = direction.components();
        let mut chord = match Self::try_new_from_certified_distinct_endpoints(
            Point2::from_values(0, 0).into(),
            Point2::new(x.clone(), y.clone()).into(),
            policy,
        )? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Arc::get_mut(&mut chord.data)
            .expect("a newly constructed direction chord has unique ownership")
            .certified_unit_tangent = Some(Arc::new([x.clone(), y.clone()]));
        Ok(Classification::Decided(chord))
    }

    pub(super) fn try_new_with_endpoint_equality(
        start: CurvePoint2,
        end: CurvePoint2,
        equality: Classification<bool>,
        probe_generic_axis_alignment: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if equality == Classification::Decided(true) {
            return Err(CurveError::ZeroLengthLine);
        }
        let pair_unit_tangent = if let (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second)),
        ) = (&start, &end)
        {
            first.strict_pair_contact_join_unit_tangent(second, policy)?
        } else {
            None
        };
        let retained_axis = pair_unit_tangent.as_ref().and_then(|(x, y)| {
            [
                (Axis2::X, real_sign(x, &CurveContext::STRICT)),
                (Axis2::Y, real_sign(y, &CurveContext::STRICT)),
            ]
            .into_iter()
            .find_map(|(axis, sign)| match sign {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => {
                    Some(BezierAlgebraicChordParameterAxis2 {
                        axis,
                        coordinate_increases: sign == RealSign::Positive,
                    })
                }
                Some(RealSign::Zero) | None => None,
            })
        });
        let parameter_axis = if let Some(axis) = retained_axis {
            axis
        } else {
            match algebraic_chord_parameter_axis(&start, &end, policy)? {
                Classification::Decided(axis) => axis,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(match equality {
                        Classification::Decided(false) => reason,
                        Classification::Uncertain(equality_reason) => equality_reason,
                        Classification::Decided(true) => unreachable!(),
                    }));
                }
            }
        };
        let parallel_axis_aligned = if let (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second)),
        ) = (&start, &end)
        {
            let constant_axis = match parameter_axis.axis {
                Axis2::X => Axis2::Y,
                Axis2::Y => Axis2::X,
            };
            first.strict_parallel_complementary_mapped_axis_order(second, constant_axis, policy)?
                == Some(std::cmp::Ordering::Equal)
        } else {
            false
        };
        let pair_unit_tangent = pair_unit_tangent.or_else(|| {
            strict_common_retained_line_unit_tangent(&start, &end, parameter_axis, policy)
        });
        let pair_axis_aligned = pair_unit_tangent.as_ref().is_some_and(|(x, y)| {
            x.zero_status() == ZeroKnowledge::Zero || y.zero_status() == ZeroKnowledge::Zero
        });
        // A selected coordinate can be constant without being a rational
        // literal.  Retain that STRICT proof here: later round joins need the
        // cardinal unit tangent structurally, and should not promote an
        // exactly axis-aligned one-field chord into a general chord-normal
        // circle merely because its constant coordinate is algebraic.
        let strict_axis_aligned =
            if parallel_axis_aligned || pair_axis_aligned || !probe_generic_axis_alignment {
                false
            } else {
                let constant_axis = match parameter_axis.axis {
                    Axis2::X => Axis2::Y,
                    Axis2::Y => Axis2::X,
                };
                let expected_parameter_order = if parameter_axis.coordinate_increases {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                };
                policy.strict_predicate_pass(|| {
                matches!(
                    algebraic_chord_point_coordinate_order(
                        &start,
                        &end,
                        parameter_axis.axis,
                        policy,
                    ),
                    Ok(Classification::Decided(order)) if order == expected_parameter_order
                ) && matches!(
                    algebraic_chord_point_coordinate_order(&start, &end, constant_axis, policy,),
                    Ok(Classification::Decided(std::cmp::Ordering::Equal))
                )
            })
            };
        let certified_axis_aligned =
            parallel_axis_aligned || pair_axis_aligned || strict_axis_aligned;
        let certified_unit_tangent = if certified_axis_aligned {
            None
        } else {
            pair_unit_tangent.map(|(x, y)| Arc::new([x, y]))
        };
        let retained_contact_support = if let (
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(second)),
        ) = (&start, &end)
        {
            let first = &first.map_contact().0.data.chord;
            let second = &second.map_contact().0.data.chord;
            first
                .shares_retained_support(second)
                .then(|| first.retained_support().clone())
        } else {
            None
        };
        // A strict coordinate order is itself a complete noncoincidence proof.
        // This matters when endpoints inhabit independent selected fields: a
        // generic two-coordinate equality predicate may be unavailable even
        // though one exact coordinate comparison already separates them.
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start,
                end,
                parameter_axis,
                certified_axis_aligned,
                certified_unit_tangent,
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: retained_contact_support,
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        }))
    }

    /// Builds an axis-aligned chord from a construction certificate.
    ///
    /// The caller must have proved that the endpoints are distinct, have the
    /// supplied direction, and belong to `policy`. This is reserved for exact
    /// translations of an already-certified axis-aligned chord, where replaying
    /// selected-field equality and direction predicates would duplicate proof.
    pub(crate) fn from_certified_axis_aligned_endpoints(
        start: CurvePoint2,
        end: CurvePoint2,
        direction: BezierAlgebraicChordAxisDirection2,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start,
                end,
                parameter_axis: direction.parameter_axis(),
                certified_axis_aligned: true,
                certified_unit_tangent: None,
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: None,
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        }
    }

    /// Builds a non-axis-aligned chord after the caller has certified one
    /// strictly monotone Cartesian component of `end - start`.
    ///
    /// Offset bevel construction derives this proof from separated signs of
    /// the two incident unit-tangent components. It is the same affine
    /// parameter authority selected by the generic constructor, but avoids
    /// reconstructing independently normalized endpoint coordinates merely to
    /// rediscover the supplied sign.
    pub(crate) fn from_certified_monotone_axis_endpoints(
        start: CurvePoint2,
        end: CurvePoint2,
        axis: Axis2,
        coordinate_increases: bool,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start,
                end,
                parameter_axis: BezierAlgebraicChordParameterAxis2 {
                    axis,
                    coordinate_increases,
                },
                certified_axis_aligned: false,
                certified_unit_tangent: None,
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: None,
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        }
    }

    /// Returns a traversal-component sign already certified by this chord's
    /// monotone parameter axis (or its constant coordinate for a cardinal
    /// chord). No endpoint predicate is evaluated.
    pub(crate) fn certified_tangent_axis_sign(&self, axis: Axis2) -> Option<RealSign> {
        if self.data.parameter_axis.axis == axis {
            return Some(if self.data.parameter_axis.coordinate_increases {
                RealSign::Positive
            } else {
                RealSign::Negative
            });
        }
        self.data.certified_axis_aligned.then_some(RealSign::Zero)
    }

    /// Signs one Cartesian tangent component, consuming the stored monotone
    /// axis first and otherwise comparing direction-equivalent endpoints.
    pub(crate) fn tangent_axis_sign(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        if let Some(sign) = self.certified_tangent_axis_sign(axis) {
            return Ok(Classification::Decided(sign));
        }
        if let Some((x, y)) = self.certified_unit_tangent()
            && let Some(sign) = real_sign(
                if axis == Axis2::X { &x } else { &y },
                &CurveContext::STRICT,
            )
        {
            return Ok(Classification::Decided(sign));
        }
        let [start, end] = self.direction_endpoints(policy);
        Ok(
            algebraic_chord_point_coordinate_order(start, end, axis, policy)?.map(|order| {
                match order {
                    std::cmp::Ordering::Less => RealSign::Positive,
                    std::cmp::Ordering::Equal => RealSign::Zero,
                    std::cmp::Ordering::Greater => RealSign::Negative,
                }
            }),
        )
    }

    /// Adapts a represented segment on this chord's already-certified affine
    /// support without asking a generic endpoint constructor to rediscover
    /// the monotone coordinate. The caller supplies a nonzero line oriented
    /// with this traversal; finite-domain ownership remains with `self`.
    pub(super) fn adapt_to_certified_affine_support_line(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        self.validate_policy(policy)?;
        Ok(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start: CurvePoint2::from(line.start().clone()),
                end: CurvePoint2::from(line.end().clone()),
                parameter_axis: self.data.parameter_axis,
                certified_axis_aligned: self.data.certified_axis_aligned,
                certified_unit_tangent: self.data.certified_unit_tangent.clone(),
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: None,
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        })
    }

    /// Coalesces two already-certified forward-collinear chord fragments.
    ///
    /// The caller owns the zero-cross and positive-dot proofs. Reusing the
    /// first fragment's monotone coordinate avoids re-solving an equality
    /// between unrelated selected endpoint fields. A shared retained support
    /// remains attached so later circle/chord and overlap replay can recognize
    /// contacts authored on the unsplit line.
    pub(crate) fn merge_certified_collinear_forward(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Self>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        if self.data.parameter_axis != other.data.parameter_axis {
            return Ok(None);
        }
        let mut contacts = self.parallel_tangent_contacts().to_vec();
        for contact in other.parallel_tangent_contacts() {
            if !contacts.contains(contact) {
                contacts.push(contact.clone());
            }
        }
        Ok(Some(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start: self.start().clone(),
                end: other.end().clone(),
                parameter_axis: self.data.parameter_axis,
                certified_axis_aligned: self.data.certified_axis_aligned
                    && other.data.certified_axis_aligned,
                certified_unit_tangent: self
                    .data
                    .certified_unit_tangent
                    .clone()
                    .or_else(|| other.data.certified_unit_tangent.clone()),
                certified_circle_transverse_endpoints: u8::from(
                    self.certified_circle_transverse_endpoint(false),
                ) | (u8::from(
                    other.certified_circle_transverse_endpoint(true),
                ) << 1),
                parallel_tangent_contacts: (!contacts.is_empty()).then(|| Arc::from(contacts)),
                source: self
                    .shares_retained_support(other)
                    .then(|| self.retained_support().clone()),
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        }))
    }

    /// Returns the endpoint at the start of boundary traversal.
    pub fn start(&self) -> &CurvePoint2 {
        if self.data.reversed {
            &self.data.end
        } else {
            &self.data.start
        }
    }

    /// Returns the endpoint at the end of boundary traversal.
    pub fn end(&self) -> &CurvePoint2 {
        if self.data.reversed {
            &self.data.start
        } else {
            &self.data.end
        }
    }

    /// Returns whether traversal opposes the chord's construction order.
    pub fn is_reversed(&self) -> bool {
        self.data.reversed
    }

    /// Returns the predicate policy under which endpoint inequality was certified.
    pub fn policy(&self) -> CurveContext {
        self.data.policy
    }

    /// Returns traversal-direction endpoints after replacing a zero-distance
    /// finite rational contact by its collinear authored support endpoint.
    /// The replacement changes only positive scale, never direction.
    pub(super) fn direction_endpoints<'a>(&'a self, policy: &CurveContext) -> [&'a CurvePoint2; 2] {
        let start = self.start();
        let end = self.end();
        if let CurvePoint2(CurvePointData2::AnalyticParallel(contact)) = start
            && let Some(authored) = contact.recursive_chord_collinear_support_endpoint(end, policy)
        {
            return [authored, end];
        }
        if let CurvePoint2(CurvePointData2::AnalyticParallel(contact)) = end
            && let Some(authored) =
                contact.recursive_chord_collinear_support_endpoint(start, policy)
        {
            return [start, authored];
        }
        [start, end]
    }

    /// Returns the same exact chord in the opposite traversal direction.
    pub fn reversed(&self) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start: self.data.start.clone(),
                end: self.data.end.clone(),
                parameter_axis: BezierAlgebraicChordParameterAxis2 {
                    axis: self.data.parameter_axis.axis,
                    coordinate_increases: !self.data.parameter_axis.coordinate_increases,
                },
                certified_axis_aligned: self.data.certified_axis_aligned,
                certified_unit_tangent: self
                    .data
                    .certified_unit_tangent
                    .as_ref()
                    .map(|tangent| Arc::new([-tangent[0].clone(), -tangent[1].clone()])),
                certified_circle_transverse_endpoints: ((self
                    .data
                    .certified_circle_transverse_endpoints
                    & 1)
                    << 1)
                    | ((self.data.certified_circle_transverse_endpoints & 2) >> 1),
                parallel_tangent_contacts: self.data.parallel_tangent_contacts.as_deref().map(
                    |contacts| {
                        Arc::from(
                            contacts
                                .iter()
                                .map(crate::bezier::BezierParallelLineTangentContact2::reversed)
                                .collect::<Vec<_>>(),
                        )
                    },
                ),
                // Preserve one stable support identity even when the root
                // chord itself is reversed.  Correlated intersection points
                // use this identity to replay incidence after later splits
                // and reversals without rematerializing endpoint fields.
                source: Some(self.retained_support().clone()),
                reversed: !self.data.reversed,
                policy: self.data.policy,
            }),
        }
    }

    pub(crate) fn retained_support(&self) -> &Self {
        self.data.source.as_ref().unwrap_or(self)
    }

    /// Returns whether this chord is the complete parameter interval of one
    /// procedural parallel carrier and whether its traversal reverses that
    /// carrier's `[0, 1]` parameter.
    pub(super) fn procedural_parallel_parameter_reversed(&self) -> Option<bool> {
        let (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
        ) = (self.start(), self.end())
        else {
            return None;
        };
        let retained = self.retained_support();
        let CurvePoint2(CurvePointData2::AlgebraicChordParallel(retained_start)) = retained.start()
        else {
            return None;
        };
        (Arc::ptr_eq(&start.data, &end.data)
            && Arc::ptr_eq(&start.data, &retained_start.data)
            && start.data.source_point.is_none()
            && start.at_end != end.at_end)
            .then_some(start.at_end)
    }

    /// Returns the oldest retained chord on this exact affine support and
    /// whether the current traversal is reversed relative to it.
    ///
    /// Split, reversal, translation, and affine-image adapters can form more
    /// than one source link.  Incidence equations need the smallest endpoint
    /// field in that chain, while finite clipping deliberately keeps the
    /// nearest descendant.  Every traversed link has the same selected
    /// parameter axis; stop defensively if an unrelated authority ever enters
    /// the chain.
    pub(super) fn smallest_retained_support(&self) -> (&Self, bool) {
        let mut support = self;
        while let Some(source) = support.data.source.as_ref() {
            if source.data.parameter_axis.axis != self.data.parameter_axis.axis {
                break;
            }
            support = source;
        }
        (
            support,
            self.data.parameter_axis.coordinate_increases
                != support.data.parameter_axis.coordinate_increases,
        )
    }

    /// Recovers an older affine support when this chord was reconstructed
    /// between two retained chord-pair contacts on the same carrier.
    ///
    /// Arrangement output may no longer be a finite subrange of that carrier,
    /// so this is intentionally an incidence-only relation and is not stored
    /// in `source`. Both endpoints independently retain the carrier, which is
    /// an exact collinearity certificate; matching monotone axes supplies the
    /// orientation without a coordinate or tangent predicate.
    pub(super) fn smallest_incidence_support(&self) -> (&Self, bool) {
        let (support, _) = self.smallest_retained_support();
        let (
            CurvePoint2(CurvePointData2::AlgebraicChordPair(start)),
            CurvePoint2(CurvePointData2::AlgebraicChordPair(end)),
        ) = (support.start(), support.end())
        else {
            return (
                support,
                self.data.parameter_axis.coordinate_increases
                    != support.data.parameter_axis.coordinate_increases,
            );
        };
        for first in [&start.data.first, &start.data.second] {
            for second in [&end.data.first, &end.data.second] {
                if !first.shares_retained_support(second) {
                    continue;
                }
                let (candidate, _) = first.smallest_retained_support();
                if candidate.data.parameter_axis.axis != self.data.parameter_axis.axis {
                    continue;
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-incidence-support",
                    "shared-chord-pair-carrier",
                );
                return (
                    candidate,
                    self.data.parameter_axis.coordinate_increases
                        != candidate.data.parameter_axis.coordinate_increases,
                );
            }
        }
        (
            support,
            self.data.parameter_axis.coordinate_increases
                != support.data.parameter_axis.coordinate_increases,
        )
    }

    /// Finds the procedural normal-offset support anywhere below a chain of
    /// exact splits, reversals, coalesces, or miter subsegments. Every source
    /// link certifies the same affine line; retaining the nearest link is
    /// useful for local predicates, while this descent recovers the compact
    /// normalized-line system that can solve a later circle contact.
    pub(super) fn retained_normal_offset_ancestor(&self) -> Option<(&Self, bool)> {
        let mut current = self;
        loop {
            if let (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) = (current.start(), current.end())
                && start.shares_carrier(end)
                && start.at_end != end.at_end
                && start.data.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal
            {
                if self.data.parameter_axis.axis != current.data.parameter_axis.axis {
                    return None;
                }
                return Some((
                    current,
                    self.data.parameter_axis.coordinate_increases
                        != current.data.parameter_axis.coordinate_increases,
                ));
            }
            current = current.data.source.as_ref()?;
        }
    }

    /// Recognizes a point authored on this chord's procedural normal-offset
    /// support, including independently replayed endpoints from another
    /// boundary fragment.
    pub(super) fn retains_normal_offset_point_incidence(&self, point: &CurvePoint2) -> bool {
        let Some((support, _)) = self.retained_normal_offset_ancestor() else {
            return false;
        };
        let (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)),
        ) = (support.start(), support.end(), point)
        else {
            return false;
        };
        start.shares_normal_offset_carrier(end) && start.shares_normal_offset_carrier(point)
    }

    pub(super) fn shares_retained_support(&self, other: &Self) -> bool {
        let first = self.retained_support();
        let second = other.retained_support();
        Arc::ptr_eq(&first.data, &second.data)
            || first == second
            || (first.start() == second.end() && first.end() == second.start())
            || matches!(
                (first.start(), first.end(), second.start(), second.end()),
                (
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(first_start)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(first_end)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(second_start)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(second_end)),
                ) if first_start.at_end != first_end.at_end
                    && second_start.at_end != second_end.at_end
                    && first_start.shares_normal_offset_carrier(first_end)
                    && first_start.shares_normal_offset_carrier(second_start)
                    && first_start.shares_normal_offset_carrier(second_end)
            )
    }

    /// Relates the traversal orientation of two structurally identical finite
    /// support segments. This deliberately ignores allocation identity so an
    /// independently replayed native line and its reversal remain one exact
    /// affine authority.
    pub(super) fn retained_support_orientation_to(&self, other: &Self) -> Option<bool> {
        let first = self.retained_support();
        let second = other.retained_support();
        if first.start() == second.start() && first.end() == second.end() {
            Some(false)
        } else if first.start() == second.end() && first.end() == second.start() {
            Some(true)
        } else {
            None
        }
    }

    pub(super) fn support_collinearity(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        if self.shares_retained_support(other) {
            return Ok(Classification::Decided(true));
        }
        if let Some(collinear) = self.retained_parallel_support_collinearity(other, policy)? {
            return Ok(Classification::Decided(collinear));
        }
        if let Some(cross) = self.tangent_cross_sign_with_shared_endpoint(other, policy) {
            match cross? {
                Classification::Decided(RealSign::Zero) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-support-collinearity",
                        "shared-endpoint-zero-cross",
                    );
                    return Ok(Classification::Decided(true));
                }
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    return Ok(Classification::Decided(false));
                }
                Classification::Uncertain(_) => {}
            }
        }

        // Axis collinearity is only a compact precheck. A nonstructural axis
        // proof may require projecting both composite endpoint fields; the
        // complete chord-pair side kernel already decides the same support
        // relation without that optional Cartesian materialization.
        if let (Some(first_direction), Some(second_direction)) = (
            self.certified_axis_direction(),
            other.certified_axis_direction(),
        ) && first_direction.axis() == second_direction.axis()
        {
            let constant_axis = match first_direction.axis() {
                Axis2::X => Axis2::Y,
                Axis2::Y => Axis2::X,
            };
            return Ok(
                Self::point_axis_order(self.start(), other.start(), constant_axis, policy)?
                    .map(|order| order == std::cmp::Ordering::Equal),
            );
        }
        if let Some(line) = self
            .exact_line()
            .or_else(|| self.strict_retained_support_line(policy))
            .or_else(|| self.strict_provenance_support_line(policy))
        {
            return Ok(other
                .has_non_collinear_support_with_exact_line(&line, policy)?
                .map(|non_collinear| !non_collinear));
        }
        if let Some(line) = other
            .exact_line()
            .or_else(|| other.strict_retained_support_line(policy))
            .or_else(|| other.strict_provenance_support_line(policy))
        {
            return Ok(self
                .has_non_collinear_support_with_exact_line(&line, policy)?
                .map(|non_collinear| !non_collinear));
        }
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    }

    /// Decides support equality for chords whose parallel relation is already
    /// owned by their retained construction. This is the single structural
    /// authority used by both collinearity and finite chord intersection;
    /// neither caller needs to rebuild a four-endpoint tangent resultant.
    pub(super) fn retained_parallel_support_collinearity(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<bool>> {
        if let Some(distance) = self
            .retained_normal_offset_distance_to(other)
            .or_else(|| other.retained_normal_offset_distance_to(self))
            && let Some(order) = compare_reals(&distance, &Real::zero(), policy)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-support-collinearity",
                "procedural-offset-to-base",
            );
            return Ok(Some(order == std::cmp::Ordering::Equal));
        }
        // Two procedural displacements of the same retained base support
        // preserve a complete symbolic line relation.  In particular,
        // opposite signed normal offsets are parallel and disjoint even when
        // neither displaced endpoint can be projected into an independent
        // Cartesian field.  Consume that construction certificate before
        // attempting endpoint materialization.
        if let (Some(first), Some(second)) = (
            chord_parallel_support_source(self, policy)?,
            chord_parallel_support_source(other, policy)?,
        ) && first.direction == second.direction
            && first.source.shares_retained_support(&second.source)
            && let Some(reversed) = first.source.shared_tangent_orientation(&second.source)
        {
            let equal = |left: &Real, right: &Real| {
                compare_reals(left, right, policy).map(|order| order == std::cmp::Ordering::Equal)
            };
            let translations_equal = equal(&first.translation_x, &second.translation_x)
                .zip(equal(&first.translation_y, &second.translation_y))
                .map(|(x, y)| x && y);
            if translations_equal == Some(true) {
                let collinear = match first.direction {
                    BezierAlgebraicChordUnitDisplacement2::Tangent => Some(true),
                    BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                        let second_distance = if reversed {
                            -second.distance
                        } else {
                            second.distance
                        };
                        equal(&first.distance, &second_distance)
                    }
                };
                if let Some(collinear) = collinear {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-support-collinearity",
                        "shared-procedural-base",
                    );
                    return Ok(Some(collinear));
                }
            }
        }
        Ok(None)
    }

    pub(super) fn retained_support_orientation_is_reversed(&self) -> bool {
        self.data.parameter_axis.coordinate_increases
            != self
                .retained_support()
                .data
                .parameter_axis
                .coordinate_increases
    }

    /// Relates two procedural normal-offset chords through their shared base
    /// support. The result is true when `other` traverses the common tangent
    /// in the opposite direction from `self`.
    pub(super) fn retained_normal_offset_base_orientation(&self) -> Option<(&Self, bool)> {
        // Exact affine adapters and finite-domain descendants can sit above
        // the procedural offset. Descend to the first actual normal-offset
        // carrier while preserving the accumulated traversal orientation.
        let (support, descendant_reversed) = self.retained_normal_offset_ancestor()?;
        let (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
        ) = (support.start(), support.end())
        else {
            return None;
        };
        if !start.shares_carrier(end)
            || start.at_end == end.at_end
            || start.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
        {
            return None;
        }
        let source = start.data.source.retained_support();
        let reversed = descendant_reversed
            ^ start.at_end
            ^ start.data.source.retained_support_orientation_is_reversed();
        Some((source, reversed))
    }

    pub(crate) fn retained_normal_offset_tangent_reversal_to(&self, other: &Self) -> Option<bool> {
        let (base, self_reversed) = self.retained_normal_offset_base_orientation()?;
        let other_reversed =
            if other.shares_retained_support(base) || other.retained_support() == base {
                other.retained_support_orientation_is_reversed()
            } else {
                let (other_base, reversed) = other.retained_normal_offset_base_orientation()?;
                if !other_base.shares_retained_support(base) && other_base != base {
                    return None;
                }
                reversed
            };
        Some(self_reversed ^ other_reversed)
    }

    /// Relates parallel chords, including independently promoted native
    /// lines. Structural ancestry remains the constant-time path; the exact
    /// tangent kernel is the representation-independent authority.
    pub(super) fn certified_parallel_tangent_reversal_to(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if let Some(reversed) = self.shared_tangent_orientation(other) {
            return Ok(Classification::Decided(reversed));
        }
        match policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))? {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(RealSign::Negative | RealSign::Positive) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        policy
            .strict_predicate_pass(|| self.tangent_dot_sign(other, policy))
            .and_then(|sign| match sign {
                Classification::Decided(RealSign::Positive) => Ok(Classification::Decided(false)),
                Classification::Decided(RealSign::Negative) => Ok(Classification::Decided(true)),
                Classification::Decided(RealSign::Zero) => Err(CurveError::Topology(
                    "parallel nondegenerate chord tangents had zero dot product".into(),
                )),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            })
    }

    /// Returns the signed left-normal displacement from `other` to this
    /// support when both descend from the same procedural normal offset.
    /// Translated descendants deliberately decline this construction proof;
    /// their complete coordinate predicate remains the fallback.
    pub(super) fn retained_normal_offset_distance_to(&self, other: &Self) -> Option<Real> {
        self.retained_normal_offset_distance_with_tangent_reversal(
            self.retained_normal_offset_tangent_reversal_to(other)?,
        )
    }

    /// Recovers the signed procedural displacement after an exact tangent
    /// relation has already identified an independently retained base line.
    pub(super) fn retained_normal_offset_distance_with_tangent_reversal(
        &self,
        target_tangent_reversed: bool,
    ) -> Option<Real> {
        let (support, _) = self.retained_normal_offset_ancestor()?;
        let (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
        ) = (support.start(), support.end())
        else {
            return None;
        };
        if !start.shares_carrier(end)
            || start.at_end == end.at_end
            || start.data.source_point.is_some()
            || start.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
            || start.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || start.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return None;
        }
        let (_, self_reversed) = self.retained_normal_offset_base_orientation()?;
        let other_reversed = self_reversed ^ target_tangent_reversed;
        let source_reversed = start.data.source.retained_support_orientation_is_reversed();
        let reversed = source_reversed != other_reversed;
        Some(if reversed {
            -start.data.distance.clone()
        } else {
            start.data.distance.clone()
        })
    }

    /// Returns the signed separation from `other` to this parallel support in
    /// `other`'s left-normal frame. A certified unit tangent keeps the result
    /// exact and avoids reconstructing either retained contact point.
    pub(super) fn exact_left_normal_support_distance_to(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<Real> {
        let line = self
            .exact_line()
            .or_else(|| self.strict_provenance_support_line(policy))?;
        let other_line = other
            .exact_line()
            .or_else(|| other.strict_provenance_support_line(policy))?;
        let (tangent_x, tangent_y) = other.certified_unit_tangent()?;
        let delta_x = line.start().x() - other_line.start().x();
        let delta_y = line.start().y() - other_line.start().y();
        Some(Real::diff_of_products(
            &tangent_x, &delta_y, &tangent_y, &delta_x,
        ))
    }

    /// Restricts an exact affine line to an arbitrary finite scalar domain.
    /// Its original support supplies the direction and incidence proofs;
    /// selected bounds remain point witnesses on that same support.
    pub(crate) fn from_affine_line_range(
        line: &LineSeg2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let source = match Self::try_new_from_certified_distinct_endpoints(
            line.start().clone().into(),
            line.end().clone().into(),
            policy,
        )? {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let [start, end] = match range.ordered_endpoints(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if start == &CurveParameter2::from(Real::zero())
            && end == &CurveParameter2::from(Real::one())
        {
            return Ok(Classification::Decided(source));
        }
        let curve = RationalBezier2::try_new_with_exact_line_image(
            vec![line.start().clone(), line.end().clone()],
            vec![Real::one(); 2],
            line.clone(),
        )?;
        let point =
            |parameter| rational_point_evidence_at_region_parameter(&curve, parameter, policy);
        let (start, end) = match (point(start)?, point(end)?) {
            (Classification::Decided(start), Classification::Decided(end)) => (start, end),
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // Increasing affine parameters preserve the original line direction,
        // including exterior bounds. No coordinate order needs to be replayed.
        Ok(Classification::Decided(
            Self::from_certified_ordered_parameter_range(
                &source,
                &source.parameter_on_retained_support(start),
                &source.parameter_on_retained_support(end),
                policy,
            )?,
        ))
    }

    pub(crate) fn from_ordered_parameter_range(
        source: &Self,
        start: &BezierAlgebraicChordParameter2,
        end: &BezierAlgebraicChordParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        source.validate_policy(policy)?;
        if !start.chord().shares_retained_support(source)
            || !end.chord().shares_retained_support(source)
        {
            return Err(CurveError::Topology(
                "algebraic chord split parameters do not share the retained support".into(),
            ));
        }
        match start.cmp_by_refinement(end, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => {}
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Err(CurveError::ZeroLengthLine);
            }
            Classification::Decided(std::cmp::Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "algebraic chord split parameters are not increasing".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "algebraic chord split parameter order remained uncertain: {reason:?}"
                )));
            }
        }
        Self::from_certified_ordered_parameter_range(source, start, end, policy)
    }

    /// Builds a subchord after the calling topology operation has already
    /// certified that `start < end` on `source`.
    ///
    /// Fillet/chamfer center solvers can prove a cut's finite-domain placement
    /// on an exact parallel before transporting that cut back to the authored
    /// chord. Parallel translation preserves the monotone support order, so
    /// replaying a Cartesian multi-field comparison here is both redundant
    /// and strictly harder than the construction-owned certificate.
    pub(crate) fn from_certified_ordered_parameter_range(
        source: &Self,
        start: &BezierAlgebraicChordParameter2,
        end: &BezierAlgebraicChordParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        source.validate_policy(policy)?;
        if !start.chord().shares_retained_support(source)
            || !end.chord().shares_retained_support(source)
        {
            return Err(CurveError::Topology(
                "certified algebraic chord split parameters do not share the retained support"
                    .into(),
            ));
        }
        Ok(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start: start.point().clone(),
                end: end.point().clone(),
                parameter_axis: source.data.parameter_axis,
                certified_axis_aligned: source.data.certified_axis_aligned,
                certified_unit_tangent: source.data.certified_unit_tangent.clone(),
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: Some(source.retained_support().clone()),
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        })
    }

    pub(crate) fn start_parameter(&self) -> BezierAlgebraicChordParameter2 {
        BezierAlgebraicChordParameter2 {
            data: BezierAlgebraicChordParameterStorage2::Endpoint {
                chord: self.clone(),
                at_end: false,
            },
        }
    }

    pub(crate) fn end_parameter(&self) -> BezierAlgebraicChordParameter2 {
        BezierAlgebraicChordParameter2 {
            data: BezierAlgebraicChordParameterStorage2::Endpoint {
                chord: self.clone(),
                at_end: true,
            },
        }
    }

    pub(super) fn parameter_on_retained_support_with_interior_certificate(
        &self,
        point: CurvePoint2,
        certified_strict_interior: bool,
    ) -> BezierAlgebraicChordParameter2 {
        BezierAlgebraicChordParameter2 {
            data: BezierAlgebraicChordParameterStorage2::Interior(Arc::new(
                BezierAlgebraicChordParameterData2 {
                    chord: self.clone(),
                    point,
                    axis: self.data.parameter_axis,
                    certified_strict_interior,
                },
            )),
        }
    }

    pub(super) fn parameter_on_retained_support(
        &self,
        point: CurvePoint2,
    ) -> BezierAlgebraicChordParameter2 {
        self.parameter_on_retained_support_with_interior_certificate(point, false)
    }

    pub(crate) fn parameter_at_certified_interior_point(
        &self,
        point: CurvePoint2,
    ) -> BezierAlgebraicChordParameter2 {
        self.parameter_on_retained_support_with_interior_certificate(point, true)
    }

    /// Names a point whose incidence on this chord's affine support was
    /// certified by the caller. Unlike [`Self::parameter_at_certified_point`],
    /// this intentionally does not clamp the parameter to the finite chord;
    /// retained corner extension uses the same monotone support coordinate on
    /// either exterior ray.
    pub(crate) fn parameter_at_certified_support_point(
        &self,
        point: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<BezierAlgebraicChordParameter2> {
        self.validate_policy(policy)?;
        Ok(self.parameter_on_retained_support(point))
    }

    /// Locates a point on this finite chord: `None` when it is off the
    /// support or outside the chord. Support incidence is decided first, so
    /// the finite parameter is never inferred from an unproved projection.
    pub(crate) fn point_parameter(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParameter2>>> {
        match self.oriented_side_by_refinement(point, policy)? {
            Classification::Decided(crate::classify::LineSide::On) => {
                self.parameter_at_certified_point(point.clone(), policy)
            }
            Classification::Decided(_) => Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(crate) fn parameter_at_certified_point(
        &self,
        point: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParameter2>>> {
        self.validate_policy(policy)?;
        let parameter = self.parameter_on_retained_support(point.clone());
        let start = self.start_parameter();
        let end = self.end_parameter();
        let retained_endpoint = |parameter| {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-finite-parameter",
                "strict-retained-endpoint",
            );
            Some(parameter)
        };
        // A support solve can rebuild a Boolean-split endpoint in a different
        // correlated field. If monotone coordinate order cannot rediscover
        // that equality, replay point identity under STRICT and retain the
        // descendant's existing endpoint parameter.
        let retained_endpoint_parameter = || {
            let at_end = policy.strict_predicate_pass(|| {
                [(false, self.start()), (true, self.end())]
                    .into_iter()
                    .find_map(|(at_end, endpoint)| {
                        (point.same_point(endpoint, policy) == Classification::Decided(true)
                            || endpoint.same_point(&point, policy) == Classification::Decided(true))
                        .then_some(at_end)
                    })
            })?;
            retained_endpoint(if at_end { end.clone() } else { start.clone() })
        };
        let compare_orders = || -> CurveResult<_> {
            Ok(if let Some(direction) = self.certified_axis_direction() {
                let axis = direction.axis();
                let order = |endpoint: &CurvePoint2| {
                    if let CurvePoint2(CurvePointData2::AlgebraicChordPair(pair)) = endpoint {
                        return pair
                            .cmp_on_chord_to_evidence(self, &point, policy)
                            .map(|classification| classification.map(std::cmp::Ordering::reverse));
                    }
                    Self::point_axis_order(&point, endpoint, axis, policy).map(|classification| {
                        if self.data.parameter_axis.coordinate_increases {
                            classification
                        } else {
                            classification.map(std::cmp::Ordering::reverse)
                        }
                    })
                };
                (order(self.start())?, order(self.end())?)
            } else {
                (
                    parameter.cmp_by_refinement(&start, policy)?,
                    parameter.cmp_by_refinement(&end, policy)?,
                )
            })
        };
        // Endpoint identity can prove what independent coordinate boxes
        // cannot. Exhaust that certificate before permitting a terminal
        // equality decision to bind this exact contact to a weaker policy.
        let mut orders = policy.strict_predicate_pass(compare_orders)?;
        if matches!(
            orders,
            (Classification::Uncertain(_), _) | (_, Classification::Uncertain(_))
        ) {
            if let Some(parameter) = retained_endpoint_parameter() {
                return Ok(Classification::Decided(Some(parameter)));
            }
            if policy.permits_approximate_512() {
                orders = compare_orders()?;
            }
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            eprintln!(
                "certified point finite orders point={} endpoints=({},{}) orders={orders:?}",
                kind(&point),
                kind(self.start()),
                kind(self.end()),
            );
        }
        let (lower, upper) = match orders {
            (Classification::Decided(lower), Classification::Decided(upper)) => (lower, upper),
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(match (lower, upper) {
            (std::cmp::Ordering::Equal, _) => retained_endpoint(start),
            (_, std::cmp::Ordering::Equal) => retained_endpoint(end),
            (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => {
                Some(self.parameter_at_certified_interior_point(point))
            }
            _ => None,
        }))
    }

    /// Clips a support-certified point through retained endpoint identity and
    /// outward coordinate boxes. Equal source parameters prove endpoint
    /// ownership; strict axis separation proves interior or exterior location.
    pub(super) fn parameter_at_certified_support_point_by_local_evidence(
        &self,
        point: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParameter2>>> {
        self.validate_policy(policy)?;
        for (at_end, endpoint) in [(false, self.start()), (true, self.end())] {
            let same = point.shares_storage(endpoint)
                || point == *endpoint
                || matches!(
                    (&point, endpoint),
                    (
                        CurvePoint2(CurvePointData2::AnalyticParallel(point)),
                        CurvePoint2(CurvePointData2::AnalyticParallel(endpoint)),
                    ) if policy.bounded_exact_predicate_pass(|| {
                        point.shared_parameter_point_equality(endpoint, policy)
                    })? == Classification::Decided(Some(true))
                );
            if same {
                return Ok(Classification::Decided(Some(if at_end {
                    self.end_parameter()
                } else {
                    self.start_parameter()
                })));
            }
        }
        let axis_interval = |bounds: &Aabb2| match self.data.parameter_axis.axis {
            Axis2::X => (bounds.min_x().clone(), bounds.max_x().clone()),
            Axis2::Y => (bounds.min_y().clone(), bounds.max_y().clone()),
        };
        for steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if policy.has_bounded_exact_predicate_budget() && steps > 8 {
                break;
            }
            let (
                Classification::Decided(start),
                Classification::Decided(end),
                Classification::Decided(point_bounds),
            ) = (
                algebraic_chord_endpoint_local_bounds_refined(self.start(), steps, policy),
                algebraic_chord_endpoint_local_bounds_refined(self.end(), steps, policy),
                algebraic_chord_endpoint_local_bounds_refined(&point, steps, policy),
            )
            else {
                continue;
            };
            let (low, high) = if self.data.parameter_axis.coordinate_increases {
                (&start, &end)
            } else {
                (&end, &start)
            };
            let (low_min, low_max) = axis_interval(low);
            let (high_min, high_max) = axis_interval(high);
            let (point_min, point_max) = axis_interval(&point_bounds);
            let after_low = compare_reals(&low_max, &point_min, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less);
            let before_high = compare_reals(&point_max, &high_min, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less);
            if after_low && before_high {
                return Ok(Classification::Decided(Some(
                    self.parameter_at_certified_interior_point(point),
                )));
            }
            let before_low = compare_reals(&point_max, &low_min, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less);
            let after_high = compare_reals(&high_max, &point_min, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less);
            if before_low || after_high {
                return Ok(Classification::Decided(None));
            }
        }
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    }

    /// Returns a represented constant coordinate on the requested axis.
    /// The certified monotone parameter axis is never constant; all other
    /// candidates retain their axis through nested contact-support queries.
    pub(crate) fn constant_axis_coordinate(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Real>> {
        if axis == self.data.parameter_axis.axis {
            return Ok(None);
        }
        let support = self.retained_support();
        let candidate = |endpoint: &CurvePoint2| {
            Ok::<_, CurveError>(match endpoint {
                CurvePoint2(CurvePointData2::Exact(point)) => Some(match axis {
                    Axis2::X => point.x().clone(),
                    Axis2::Y => point.y().clone(),
                }),
                CurvePoint2(CurvePointData2::Algebraic(point)) => {
                    point.exact_coordinate(axis == Axis2::X, policy)
                }
                CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
                    match point.exact_axis_coordinate(axis, policy)? {
                        Classification::Decided(candidate) => candidate,
                        Classification::Uncertain(_) => None,
                    }
                }
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => {
                    match point.exact_axis_coordinate(axis, policy)? {
                        Classification::Decided(candidate) => candidate,
                        Classification::Uncertain(_) => None,
                    }
                }
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => point
                    .map_contact()
                    .0
                    .data
                    .chord
                    .constant_axis_coordinate(axis, policy)?,
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    None
                }
            })
        };
        let [first, second] = [support.start(), support.end()].map(candidate);
        let (first, second) = (first?, second?);
        match (first, second) {
            (Some(first), Some(second))
                if compare_reals(&first, &second, &CurveContext::STRICT)
                    == Some(std::cmp::Ordering::Equal) =>
            {
                Ok(Some(first))
            }
            (Some(candidate), None) | (None, Some(candidate))
                if self.data.certified_axis_aligned =>
            {
                Ok(Some(candidate))
            }
            _ => Ok(None),
        }
    }

    pub(crate) fn representative_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        self.validate_policy(policy)?;
        let strict_axis_direction = self.strict_axis_direction(policy);
        if let Some(line) = self.exact_line()
            && self
                .data
                .source
                .as_ref()
                .is_none_or(|source| source.exact_line().is_some())
        {
            let half = (Real::one() / Real::from(2_i8))?;
            return Ok(Classification::Decided(CurvePoint2::from(
                line.point_at(half),
            )));
        }
        let coordinate = match algebraic_chord_strict_coordinate_between(
            self.start(),
            self.end(),
            self.data.parameter_axis,
            policy,
        )? {
            Classification::Decided(coordinate) => coordinate,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some(line) = self.strict_provenance_support_line(policy) {
            let (delta_x, delta_y) = line.delta();
            let point = match self.data.parameter_axis.axis {
                Axis2::X => {
                    let parameter = ((&coordinate - line.start().x()) / &delta_x)?;
                    Point2::new(coordinate.clone(), line.start().y() + delta_y * parameter)
                }
                Axis2::Y => {
                    let parameter = ((&coordinate - line.start().y()) / &delta_y)?;
                    Point2::new(line.start().x() + delta_x * parameter, coordinate.clone())
                }
            };
            return Ok(Classification::Decided(CurvePoint2::from(point)));
        }
        if strict_axis_direction.is_some()
            && let Some(constant_coordinate) = self.constant_axis_coordinate(
                match self.data.parameter_axis.axis {
                    Axis2::X => Axis2::Y,
                    Axis2::Y => Axis2::X,
                },
                policy,
            )?
        {
            let point = match self.data.parameter_axis.axis {
                Axis2::X => Point2::new(coordinate, constant_coordinate),
                Axis2::Y => Point2::new(constant_coordinate, coordinate),
            };
            return Ok(Classification::Decided(CurvePoint2::from(point)));
        }
        let support = self.data.source.as_ref().unwrap_or(self);
        let [start, end] =
            match algebraic_chord_endpoint_images(support.start(), support.end(), policy)? {
                Classification::Decided(endpoints) => endpoints,
                Classification::Uncertain(_) => {
                    return support
                        .representative_point_at_transverse_coordinate(&coordinate, policy);
                }
            };
        let parameter = match algebraic_chord_image_parameter(&start, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(_) => {
                return support.representative_point_at_transverse_coordinate(&coordinate, policy);
            }
        };
        if strict_axis_direction.is_some() {
            let [start_x, start_y, start_weight] =
                match algebraic_chord_owned_coordinate_polynomials(&start, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(_) => {
                        return support
                            .representative_point_at_transverse_coordinate(&coordinate, policy);
                    }
                };
            let (x_numerator, y_numerator) = match self.data.parameter_axis.axis {
                Axis2::X => (polynomial_scale(&start_weight, &coordinate), start_y),
                Axis2::Y => (start_x, polynomial_scale(&start_weight, &coordinate)),
            };
            let point = RationalBezierAlgebraicPointImage2::from_retained_expression(
                parameter.clone(),
                parameter_representation(&parameter, policy),
                x_numerator,
                y_numerator,
                start_weight,
                "retained an exact axis representative on an algebraic chord support",
            );
            return match point.predicate_evaluator(policy)? {
                Classification::Decided(_) => Ok(Classification::Decided(CurvePoint2::from(point))),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        let end_parameter = match algebraic_chord_image_parameter(&end, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(_) => {
                return support.representative_point_at_transverse_coordinate(&coordinate, policy);
            }
        };
        if BezierParameter2::Algebraic(parameter.clone())
            .same_value(&BezierParameter2::Algebraic(end_parameter), policy)?
            != Classification::Decided(true)
        {
            return support.representative_point_at_transverse_coordinate(&coordinate, policy);
        }
        let [start_x, start_y, start_weight] =
            match algebraic_chord_owned_coordinate_polynomials(&start, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let [end_x, end_y, end_weight] =
            match algebraic_chord_owned_coordinate_polynomials(&end, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let line_x = polynomial_subtract(
            &polynomial_multiply(&end_x, &start_weight),
            &polynomial_multiply(&start_x, &end_weight),
        );
        let line_y = polynomial_subtract(
            &polynomial_multiply(&end_y, &start_weight),
            &polynomial_multiply(&start_y, &end_weight),
        );
        let (x_numerator, y_numerator, denominator) = match self.data.parameter_axis.axis {
            Axis2::X => {
                let denominator = polynomial_multiply(&start_weight, &line_x);
                let offset =
                    polynomial_subtract(&polynomial_scale(&start_weight, &coordinate), &start_x);
                let y_numerator = polynomial_add(
                    &polynomial_multiply(&start_y, &line_x),
                    &polynomial_multiply(&offset, &line_y),
                );
                (
                    polynomial_scale(&denominator, &coordinate),
                    y_numerator,
                    denominator,
                )
            }
            Axis2::Y => {
                let denominator = polynomial_multiply(&start_weight, &line_y);
                let offset =
                    polynomial_subtract(&polynomial_scale(&start_weight, &coordinate), &start_y);
                let x_numerator = polynomial_add(
                    &polynomial_multiply(&start_x, &line_y),
                    &polynomial_multiply(&offset, &line_x),
                );
                (
                    x_numerator,
                    polynomial_scale(&denominator, &coordinate),
                    denominator,
                )
            }
        };
        let point = RationalBezierAlgebraicPointImage2::from_retained_expression(
            parameter.clone(),
            parameter_representation(&parameter, policy),
            x_numerator,
            y_numerator,
            denominator,
            "retained an exact representative on an algebraic chord support",
        );
        match point.predicate_evaluator(policy)? {
            Classification::Decided(_) => Ok(Classification::Decided(CurvePoint2::from(point))),
            Classification::Uncertain(_) => {
                support.representative_point_at_transverse_coordinate(&coordinate, policy)
            }
        }
    }

    pub(super) fn representative_point_at_transverse_coordinate(
        &self,
        coordinate: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        // Independently selected endpoints deliberately do not share a
        // primitive element. Intersect this support with an exact line at the
        // certified interior coordinate instead. The ordinary chord-pair
        // point keeps both endpoint fields separate in one compact carrier.
        let mut last_reason = UncertaintyReason::Ordering;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let bounds = match self.conservative_bounds_refined(refinement_steps, policy)? {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                    continue;
                }
            };
            let one = Real::one();
            let (transverse_start, transverse_end, direction) = match self.data.parameter_axis.axis
            {
                Axis2::X => (
                    Point2::new(coordinate.clone(), bounds.min().y() - &one),
                    Point2::new(coordinate.clone(), bounds.max().y() + &one),
                    BezierAlgebraicChordAxisDirection2::PositiveY,
                ),
                Axis2::Y => (
                    Point2::new(bounds.min().x() - &one, coordinate.clone()),
                    Point2::new(bounds.max().x() + &one, coordinate.clone()),
                    BezierAlgebraicChordAxisDirection2::PositiveX,
                ),
            };
            let transverse = Self::from_certified_axis_aligned_endpoints(
                CurvePoint2::from(transverse_start),
                CurvePoint2::from(transverse_end),
                direction,
                policy,
            );
            // `coordinate` is strictly between the endpoint coordinates on
            // the certified injective axis. The positive cardinal transverse
            // extends strictly beyond a conservative box in the other axis,
            // so the two finite chords have exactly one strict interior
            // contact. Retain those four side signs directly instead of
            // replaying the general chord-intersection kernel.
            let left = crate::classify::LineSide::Left;
            let right = crate::classify::LineSide::Right;
            let (first_sides, tangent_cross_sign) = match (
                self.data.parameter_axis.axis,
                self.data.parameter_axis.coordinate_increases,
            ) {
                (Axis2::X, true) => ([left, right], RealSign::Positive),
                (Axis2::X, false) => ([right, left], RealSign::Negative),
                (Axis2::Y, true) => ([right, left], RealSign::Negative),
                (Axis2::Y, false) => ([left, right], RealSign::Positive),
            };
            let second_sides = match tangent_cross_sign {
                RealSign::Positive => [right, left],
                RealSign::Negative => [left, right],
                RealSign::Zero => unreachable!("a transverse axis has nonzero tangent cross"),
            };
            let point = BezierAlgebraicChordPairPoint2::new(
                self.clone(),
                transverse,
                first_sides,
                second_sides,
                tangent_cross_sign,
                policy,
            );
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-representative",
                "certified-transverse",
            );
            return Ok(Classification::Decided(CurvePoint2::from(point)));
        }
        Ok(Classification::Uncertain(last_reason))
    }

    /// Returns a cardinal direction only from reusable structural evidence.
    /// Approximate equality is never allowed to create an axis certificate.
    pub(crate) fn certified_axis_direction(&self) -> Option<BezierAlgebraicChordAxisDirection2> {
        if !self.data.certified_axis_aligned
            && !self
                .data
                .source
                .as_ref()
                .is_some_and(|source| source.certified_axis_direction().is_some())
        {
            return None;
        }
        Some(
            match (
                self.data.parameter_axis.axis,
                self.data.parameter_axis.coordinate_increases,
            ) {
                (Axis2::X, true) => BezierAlgebraicChordAxisDirection2::PositiveX,
                (Axis2::X, false) => BezierAlgebraicChordAxisDirection2::NegativeX,
                (Axis2::Y, true) => BezierAlgebraicChordAxisDirection2::PositiveY,
                (Axis2::Y, false) => BezierAlgebraicChordAxisDirection2::NegativeY,
            },
        )
    }

    pub(super) fn strict_axis_direction(
        &self,
        policy: &CurveContext,
    ) -> Option<BezierAlgebraicChordAxisDirection2> {
        let certified = self.certified_axis_direction();
        if certified.is_some() || policy.has_bounded_exact_predicate_budget() {
            return certified;
        }
        certified.or_else(|| match self.axis_direction(&policy.strict_counterpart()) {
            Ok(Classification::Decided(Some(direction))) => Some(direction),
            Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => None,
        })
    }

    /// Returns reusable exact unit-tangent evidence for this traversal.
    /// Cardinal chords synthesize it without storage; transformed certified
    /// chords retain one shared two-scalar allocation.
    pub(crate) fn certified_unit_tangent(&self) -> Option<(Real, Real)> {
        if let Some(tangent) = &self.data.certified_unit_tangent {
            return Some((tangent[0].clone(), tangent[1].clone()));
        }
        self.certified_axis_direction()
            .map(|direction| direction.unit_tangent())
    }

    /// Retains the endpoint displaced by a signed Euclidean distance along
    /// this chord's traversal tangent.
    ///
    /// Represented unit tangents keep the existing translated-endpoint fast
    /// path. General independently selected endpoints retain the normalized
    /// tangent expression lazily, without adjoining their fields or rounding.
    pub(crate) fn endpoint_at_signed_tangent_distance(
        &self,
        at_end: bool,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        self.validate_policy(policy)?;
        let endpoint = if at_end { self.end() } else { self.start() };
        if let Some((tangent_x, tangent_y)) = self.certified_unit_tangent() {
            return Self::translated_endpoint(
                endpoint,
                &(tangent_x * &distance),
                &(tangent_y * distance),
                policy,
            );
        }
        Ok(Classification::Decided(CurvePoint2::from(
            BezierAlgebraicChordParallelPoint2::tangent_endpoint(
                self.clone(),
                at_end,
                distance,
                policy,
            ),
        )))
    }

    pub(super) fn axis_oriented_side(
        &self,
        point: &CurvePoint2,
        direction: BezierAlgebraicChordAxisDirection2,
        policy: &CurveContext,
    ) -> Option<Classification<crate::classify::LineSide>> {
        let constant_axis = match direction.axis() {
            Axis2::X => Axis2::Y,
            Axis2::Y => Axis2::X,
        };
        // Keep cheap correlated construction proofs first. If they decline,
        // a retained constant support coordinate needs only one scalar query;
        // complete point-to-point replay can otherwise promote both Cartesian
        // coordinates and the point's local root into independent fields.
        let retained_order = policy.bounded_exact_predicate_pass(|| {
            Self::point_axis_order(point, self.start(), constant_axis, policy)
        });
        let order = match retained_order {
            Ok(decided @ Classification::Decided(_)) => decided,
            Ok(Classification::Uncertain(_)) | Err(_) => {
                if let Ok(Some(coordinate)) = policy.bounded_exact_predicate_pass(|| {
                    self.constant_axis_coordinate(constant_axis, policy)
                }) && let Ok(Classification::Decided(order)) =
                    Self::point_axis_order_to_real(point, constant_axis, &coordinate, policy)
                {
                    return Some(Classification::Decided(
                        direction.line_side_from_perpendicular_order(order),
                    ));
                }
                Self::point_axis_order(point, self.start(), constant_axis, policy).ok()?
            }
        };
        Some(order.map(|order| direction.line_side_from_perpendicular_order(order)))
    }

    /// Classifies a represented point against a procedural displaced support
    /// from the base chord and one speed interval.
    ///
    /// For base direction `D`, source origin `A`, translation `T`, and signed
    /// left-normal distance `d`, the displaced-line incidence is
    ///
    /// `cross(D, P - A - T) - d |D|`.
    ///
    /// This is the complete affine line predicate, but its progressively
    /// refined local intervals avoid constructing either normalized displaced
    /// endpoint. Failure to separate only declines the fast path.
    pub(super) fn normal_offset_point_side_by_local_refinement(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        let Some(support) = chord_parallel_support_source(self, policy)? else {
            return Ok(None);
        };
        let Some(reversed) = support.source.shared_tangent_orientation(self) else {
            return Ok(None);
        };
        let strict = &CurveContext::STRICT;
        let exact = |value: &Real| RealInterval {
            lower: value.clone(),
            upper: value.clone(),
        };
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (
                Classification::Decided(start),
                Classification::Decided(end),
                Classification::Decided(point),
            ) = (
                algebraic_chord_endpoint_local_bounds_refined(
                    support.source.start(),
                    refinement_steps,
                    policy,
                ),
                algebraic_chord_endpoint_local_bounds_refined(
                    support.source.end(),
                    refinement_steps,
                    policy,
                ),
                algebraic_chord_endpoint_local_bounds_refined(point, refinement_steps, policy),
            )
            else {
                continue;
            };
            let start_x = RealInterval::from_axis(&start, Axis2::X);
            let start_y = RealInterval::from_axis(&start, Axis2::Y);
            let direction_x = RealInterval::from_axis(&end, Axis2::X).subtract(&start_x);
            let direction_y = RealInterval::from_axis(&end, Axis2::Y).subtract(&start_y);
            let point_x = RealInterval::from_axis(&point, Axis2::X)
                .subtract(&start_x)
                .subtract(&exact(&support.translation_x));
            let point_y = RealInterval::from_axis(&point, Axis2::Y)
                .subtract(&start_y)
                .subtract(&exact(&support.translation_y));
            let Some(cross) = direction_x.multiply(&point_y).and_then(|first| {
                direction_y
                    .multiply(&point_x)
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            let incidence = match support.direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    let Some(normal) = direction_x
                        .square()
                        .and_then(|x| direction_y.square().map(|y| x.add(&y)))
                        .and_then(|squared| squared.nonnegative_square_root(None))
                        .and_then(|speed| speed.multiply(&exact(&support.distance)))
                    else {
                        continue;
                    };
                    cross.subtract(&normal)
                }
                // A tangent displacement changes only the selected finite
                // endpoints; its complete affine support is unchanged.
                BezierAlgebraicChordUnitDisplacement2::Tangent => cross,
            };
            let sign = if compare_reals(&incidence.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                Some(RealSign::Positive)
            } else if compare_reals(&incidence.upper, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Less)
            {
                Some(RealSign::Negative)
            } else if compare_reals(&incidence.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(&incidence.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Equal)
            {
                Some(RealSign::Zero)
            } else {
                None
            };
            if let Some(mut sign) = sign {
                if reversed {
                    sign = product_sign(sign, RealSign::Negative);
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "procedural-normal-offset-local-interval",
                );
                return Ok(Some(crate::classify::LineSide::from_real_sign(sign)));
            }
        }
        Ok(None)
    }

    /// Signs any retained point against a procedural displaced support
    /// without materializing either displaced endpoint.
    ///
    /// If `D` is a positively normalized projective numerator for the source
    /// direction, `A` is the source origin, `T` is the retained translation,
    /// and `d` is the signed left-normal distance, the oriented line
    /// incidence has the sign of
    ///
    /// `cross(D, P - A - T) - d |D|`.
    ///
    /// All three authored points first enter their least shared projective
    /// tower. Both terms then remain in that field. The only new radical is
    /// the positive source speed, and `affine_positive_root_sign` compares
    /// the two correlated magnitudes without constructing the displaced
    /// Cartesian coordinates or their determinant in a larger tower.
    pub(super) fn normal_offset_recursive_point_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<crate::classify::LineSide>>> {
        let Some(support) = chord_parallel_support_source(self, policy)? else {
            return Ok(Classification::Decided(None));
        };
        let Some(reversed) = support.source.shared_tangent_orientation(self) else {
            return Ok(Classification::Decided(None));
        };
        if policy.has_bounded_exact_predicate_budget() {
            // Constructing the recursive direction frame can already import
            // every endpoint field. The batched caller has shared local-box
            // predicates to try before this complete affine-radical authority.
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        let direction_evidence = support.source.direction_endpoints(policy);
        let evidence = [direction_evidence[0], direction_evidence[1], point];
        let points = match recursive_projective_evidence_points(&evidence, policy)? {
            Classification::Decided(Some(points)) => points,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut normalized = Vec::with_capacity(points.len());
        for (point, evidence) in points.into_iter().zip(evidence) {
            let sign = match recursive_projective_evidence_denominator_sign(evidence, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            normalized.push(orient_recursive_projective_point_positive(point, sign)?);
        }
        let [origin, direction_end, point]: [BezierRecursiveQuadraticProjectivePoint2; 3] =
            normalized
                .try_into()
                .expect("a procedural support predicate retains three points");

        // Compact selected-root witnesses can evaluate the final incidence
        // directly in Hyperreal's factored graph. This consumes no equality
        // decision while building the expression; APPROXIMATE_512 is allowed
        // only at the final scalar sign below.
        let exact = |point: &BezierRecursiveQuadraticProjectivePoint2| {
            Some([
                point.x.exact_real_value_with_retained_witnesses()?,
                point.y.exact_real_value_with_retained_witnesses()?,
                point
                    .denominator
                    .exact_real_value_with_retained_witnesses()?,
            ])
        };
        if let (Some(origin), Some(direction_end), Some(point)) =
            (exact(&origin), exact(&direction_end), exact(&point))
        {
            let direction_x = Real::diff_of_products(
                &direction_end[0],
                &origin[2],
                &origin[0],
                &direction_end[2],
            );
            let direction_y = Real::diff_of_products(
                &direction_end[1],
                &origin[2],
                &origin[1],
                &direction_end[2],
            );
            let point_denominator = &point[2] * &origin[2];
            let point_x = Real::diff_of_products(&point[0], &origin[2], &origin[0], &point[2])
                - &support.translation_x * &point_denominator;
            let point_y = Real::diff_of_products(&point[1], &origin[2], &origin[1], &point[2])
                - &support.translation_y * &point_denominator;
            let cross = Real::diff_of_products(&direction_x, &point_y, &direction_y, &point_x);
            let value = match support.direction {
                BezierAlgebraicChordUnitDisplacement2::Tangent => Some(cross),
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    let speed_squared = Real::signed_product_sum(
                        [true, true],
                        [[&direction_x, &direction_x], [&direction_y, &direction_y]],
                    );
                    speed_squared.sqrt().ok().map(|speed| {
                        let normal_coefficient = &support.distance * &point_denominator;
                        let one = Real::one();
                        Real::diff_of_products(&cross, &one, &normal_coefficient, &speed)
                    })
                }
            };
            if let Some(value) = value {
                let minimum_precision = if policy.has_bounded_exact_predicate_budget() {
                    -128
                } else {
                    -512
                };
                let sign = (value.zero_status() == ZeroKnowledge::Zero)
                    .then_some(RealSign::Zero)
                    .or_else(|| value.immediate_sign())
                    .or_else(|| value.certified_sign_until(minimum_precision).sign())
                    .or_else(|| {
                        (!policy.has_bounded_exact_predicate_budget())
                            .then(|| real_sign(&value, policy))
                            .flatten()
                    });
                if let Some(mut sign) = sign {
                    if reversed {
                        sign = product_sign(sign, RealSign::Negative);
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-side-kernel",
                        "procedural-normal-offset-factored-real",
                    );
                    return Ok(Classification::Decided(Some(
                        crate::classify::LineSide::from_real_sign(sign),
                    )));
                }
            }
        }
        let Some((direction_x, direction_y, _)) = direction_end.difference_numerators(&origin)
        else {
            return Ok(Classification::Decided(None));
        };
        let Some((point_x, point_y, point_denominator)) = point.difference_numerators(&origin)
        else {
            return Ok(Classification::Decided(None));
        };
        let Some((point_x, point_y)) = (|| {
            Some((
                point_x.subtract(&point_denominator.scale(&support.translation_x)?)?,
                point_y.subtract(&point_denominator.scale(&support.translation_y)?)?,
            ))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let Some(cross) = direction_x.multiply(&point_y).and_then(|first| {
            direction_y
                .multiply(&point_x)
                .and_then(|second| first.subtract(&second))
        }) else {
            return Ok(Classification::Decided(None));
        };
        let sign = match support.direction {
            BezierAlgebraicChordUnitDisplacement2::Tangent => cross.sign(policy)?,
            BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                let Some(speed_squared) = direction_x
                    .square()
                    .and_then(|x| direction_y.square().and_then(|y| x.add(&y)))
                else {
                    return Ok(Classification::Decided(None));
                };
                let Some(radical) = point_denominator.scale(&(-support.distance.clone())) else {
                    return Ok(Classification::Decided(None));
                };
                let field = cross.field();
                let retained_speed = field.retained_positive_square_root(&speed_squared);
                let retained_value = retained_speed.as_ref().and_then(|speed| {
                    radical
                        .multiply(speed)
                        .and_then(|normal| cross.add(&normal))
                });
                let bounded_exact_pass = policy.has_bounded_exact_predicate_budget();
                let bounded_steps = if bounded_exact_pass { 128 } else { 512 };
                let retained_sign = retained_value.as_ref().and_then(|value| {
                    if value.is_structurally_zero() {
                        Some(RealSign::Zero)
                    } else {
                        value
                            .exact_real_value_with_retained_witnesses()
                            .and_then(|value| {
                                value.immediate_sign().or_else(|| {
                                    value
                                        .certified_sign_until(
                                            if policy.has_bounded_exact_predicate_budget() {
                                                -128
                                            } else {
                                                -512
                                            },
                                        )
                                        .sign()
                                })
                            })
                            .or_else(|| value.bounded_interval_sign(0..=bounded_steps))
                    }
                });
                if let Some(sign) = retained_sign {
                    Classification::Decided(sign)
                } else if let Some(value) = retained_value
                    && !policy.has_bounded_exact_predicate_budget()
                    && !policy.selects_approximate_512()
                {
                    value.sign(policy)?
                } else {
                    BezierRecursiveQuadraticValue2::affine_positive_root_sign(
                        &cross,
                        &radical,
                        &speed_squared,
                        None,
                        None,
                        policy,
                    )?
                }
            }
        };
        Ok(sign.map(|mut sign| {
            if reversed {
                sign = product_sign(sign, RealSign::Negative);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "procedural-normal-offset-recursive-affine",
            );
            Some(crate::classify::LineSide::from_real_sign(sign))
        }))
    }

    /// Replays exact construction-local side certificates for retained
    /// procedural endpoints. These predicates preserve the shared normal or
    /// tangent field and therefore remain available when this support cannot
    /// be flattened into one algebraic ray. `None` only declines the fast
    /// path; the common endpoint/refinement kernel remains authoritative.
    pub(crate) fn retained_procedural_point_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        if let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point {
            if let Some(side) =
                point.equal_normal_offset_contact_oriented_side_to_chord(self, policy)?
            {
                return Ok(Some(side));
            }
            if self.contains_authored_source_tangent_origin(point, policy) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "authored-source-tangent-incidence",
                );
                return Ok(Some(crate::classify::LineSide::On));
            }
            if let Some(side) = point.oriented_side_to_analytic_tangent_chord(self, policy)? {
                return Ok(Some(side));
            }
            if point.certifies_monotone_chord_incidence(self, policy)? {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "retained-monotone-contact-incidence",
                );
                return Ok(Some(crate::classify::LineSide::On));
            }
            if let Some(side) = point.retained_parameter_oriented_side_to_chord(self, policy)? {
                return Ok(Some(side));
            }
        }
        if self.retains_normal_offset_point_incidence(point) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "procedural-normal-offset-incidence",
            );
            return Ok(Some(crate::classify::LineSide::On));
        }
        if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) = point {
            if let Classification::Decided(Some(side)) =
                point.normal_offset_oriented_side_to_chord(self, policy)?
            {
                return Ok(Some(side));
            }
            if let Classification::Decided(Some(side)) =
                point.oriented_side_to_analytic_tangent_chord(self, policy)?
            {
                return Ok(Some(side));
            }
        }
        if matches!(
            point,
            CurvePoint2(CurvePointData2::Exact(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        ) {
            if let Some(side) = self.normal_offset_point_side_by_local_refinement(point, policy)? {
                return Ok(Some(side));
            }
            if let Classification::Decided(Some(side)) =
                self.normal_offset_recursive_point_side(point, policy)?
            {
                return Ok(Some(side));
            }
        }
        Ok(None)
    }

    /// Classifies a point against this support from a retained traversal
    /// tangent and exact endpoint enclosures. This preserves the compact
    /// transformed-chord certificate without adjoining the endpoint fields.
    pub(crate) fn certified_tangent_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> Classification<crate::classify::LineSide> {
        // A direction inferred under APPROXIMATE_512 may only terminate the
        // equality predicate that requested it; it must not become a
        // construction fact for a later side sign. Recover an axis direction
        // here only through the strict counterpart (or retained construction
        // evidence), then use the selected policy solely for the final
        // coordinate comparison.
        let strict_axis_direction = self.strict_axis_direction(policy);
        let tangent = self
            .certified_unit_tangent()
            .or_else(|| strict_axis_direction.map(|direction| direction.unit_tangent()));
        let Some((tangent_x, tangent_y)) = tangent else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        if [self.start(), self.end()]
            .into_iter()
            .any(|endpoint| point.shares_storage(endpoint))
        {
            return Classification::Decided(crate::classify::LineSide::On);
        }
        if let Some(direction) = strict_axis_direction
            && let Some(Classification::Decided(side)) =
                self.axis_oriented_side(point, direction, policy)
        {
            // A structural cardinal certificate reduces the side predicate
            // to one exact coordinate comparison. In particular, this
            // recognizes incidence when a correlated chord-pair endpoint
            // lies on a represented axis support; interval refinement cannot
            // prove that equality by width separation alone.
            return Classification::Decided(side);
        }
        if policy.has_bounded_exact_predicate_budget() {
            // Beyond structural axis incidence, even the first generic point
            // envelope can materialize a correlated chord-pair tower. Leave
            // that complete tangent-side authority to the full predicate.
            return Classification::Uncertain(UncertaintyReason::Predicate);
        }
        let tangent_x = RealInterval::from_values([tangent_x])
            .expect("one exact tangent coordinate defines an interval");
        let tangent_y = RealInterval::from_values([tangent_y])
            .expect("one exact tangent coordinate defines an interval");
        let zero = Real::zero();
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (Classification::Decided(origin), Classification::Decided(point)) = (
                algebraic_chord_endpoint_bounds_refined(self.start(), refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy),
            ) else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let delta_x = RealInterval::from_axis(&point, Axis2::X)
                .subtract(&RealInterval::from_axis(&origin, Axis2::X));
            let delta_y = RealInterval::from_axis(&point, Axis2::Y)
                .subtract(&RealInterval::from_axis(&origin, Axis2::Y));
            let Some(cross) = tangent_x.multiply(&delta_y).and_then(|first| {
                tangent_y
                    .multiply(&delta_x)
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            if compare_reals(&cross.lower, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Greater)
            {
                return Classification::Decided(crate::classify::LineSide::Left);
            }
            if compare_reals(&cross.upper, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Classification::Decided(crate::classify::LineSide::Right);
            }
        }
        // Equality is deliberately a cold path: interval refinement proves
        // every nonzero oriented area first. A separately retained endpoint
        // can nevertheless be the same exact point as this support endpoint
        // without sharing its allocation (notably where an offset circle
        // collapses). Replay that compact point certificate under STRICT
        // before adjoining all three point fields in the general represented
        // oriented-area authority.
        let retained_endpoint_incidence = || {
            [self.start(), self.end()].into_iter().any(|endpoint| {
                endpoint.same_point(point, policy) == Classification::Decided(true)
                    || point.same_point(endpoint, policy) == Classification::Decided(true)
            })
        };
        if policy.selects_approximate_512() {
            let strict = policy.strict_counterpart();
            if let Ok(Classification::Decided(Some(side))) =
                self.recursive_projective_oriented_side(point, false, &strict)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "certified-tangent-recursive-projective",
                );
                return Classification::Decided(side);
            }
        }
        if !policy.selects_approximate_512()
            && policy.strict_predicate_pass(retained_endpoint_incidence)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "retained-endpoint-incidence",
            );
            return Classification::Decided(crate::classify::LineSide::On);
        }
        if !policy.selects_approximate_512()
            && let Ok(Classification::Decided(side)) =
                policy.strict_predicate_pass(|| self.represented_oriented_side(point, policy))
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "certified-tangent-represented-cold-fallback",
            );
            return Classification::Decided(side);
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "approximate-512-terminal",
            );
            return Classification::Decided(crate::classify::LineSide::On);
        }
        // If the coordinate bounds themselves were unavailable at their
        // terminal, retained endpoint equality can still supply the selected
        // policy's bounded 512-bit decision.
        if policy.permits_approximate_512() && retained_endpoint_incidence() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "retained-endpoint-incidence",
            );
            return Classification::Decided(crate::classify::LineSide::On);
        }
        Classification::Uncertain(UncertaintyReason::Ordering)
    }

    /// Returns the exact unit traversal direction when this retained chord is
    /// axis aligned.
    ///
    /// A diagonal chord returns `None`; uncertainty in either selected-field
    /// comparison remains explicit. The zero/zero case is impossible for a
    /// validated chord and is treated as a topology failure rather than an
    /// offset direction.
    pub(crate) fn axis_direction(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordAxisDirection2>>> {
        let direction_from_signs = |x, y| {
            Ok(match (x, y) {
                (RealSign::Positive, RealSign::Zero) => {
                    Some(BezierAlgebraicChordAxisDirection2::PositiveX)
                }
                (RealSign::Negative, RealSign::Zero) => {
                    Some(BezierAlgebraicChordAxisDirection2::NegativeX)
                }
                (RealSign::Zero, RealSign::Positive) => {
                    Some(BezierAlgebraicChordAxisDirection2::PositiveY)
                }
                (RealSign::Zero, RealSign::Negative) => {
                    Some(BezierAlgebraicChordAxisDirection2::NegativeY)
                }
                (RealSign::Zero, RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "validated algebraic chord has zero direction".into(),
                    ));
                }
                (
                    RealSign::Positive | RealSign::Negative,
                    RealSign::Positive | RealSign::Negative,
                ) => None,
            })
        };
        let source_is_axis_aligned = if self.data.certified_axis_aligned {
            false
        } else if let Some(source) = &self.data.source {
            matches!(
                source.axis_direction(policy)?,
                Classification::Decided(Some(_))
            )
        } else {
            false
        };
        if self.data.certified_axis_aligned || source_is_axis_aligned {
            return Ok(Classification::Decided(Some(
                match (
                    self.data.parameter_axis.axis,
                    self.data.parameter_axis.coordinate_increases,
                ) {
                    (Axis2::X, true) => BezierAlgebraicChordAxisDirection2::PositiveX,
                    (Axis2::X, false) => BezierAlgebraicChordAxisDirection2::NegativeX,
                    (Axis2::Y, true) => BezierAlgebraicChordAxisDirection2::PositiveY,
                    (Axis2::Y, false) => BezierAlgebraicChordAxisDirection2::NegativeY,
                },
            )));
        }
        if let Some(tangent) = &self.data.certified_unit_tangent
            && let (Some(x), Some(y)) = (
                real_sign(&tangent[0], &CurveContext::STRICT),
                real_sign(&tangent[1], &CurveContext::STRICT),
            )
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-axis-direction",
                "certified-unit-tangent",
            );
            return Ok(Classification::Decided(direction_from_signs(x, y)?));
        }
        let x = self.tangent_axis_sign(Axis2::X, policy)?;
        let y = self.tangent_axis_sign(Axis2::Y, policy)?;
        let x = match x {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let y = match y {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let direction = direction_from_signs(x, y)?;
        Ok(Classification::Decided(direction))
    }

    /// Compares one coordinate of two retained affine point carriers.
    pub(crate) fn point_axis_order(
        first: &CurvePoint2,
        second: &CurvePoint2,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        algebraic_chord_point_coordinate_order(first, second, axis, policy)
    }

    /// Compares one retained point coordinate with a represented scalar.
    pub(crate) fn point_axis_order_to_real(
        point: &CurvePoint2,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let represented = CurvePoint2::from(match axis {
            Axis2::X => Point2::new(value.clone(), Real::zero()),
            Axis2::Y => Point2::new(Real::zero(), value.clone()),
        });
        algebraic_chord_point_coordinate_order(point, &represented, axis, policy)
    }

    /// Certifies that both finite endpoints lie strictly on one side of an
    /// axis coordinate without ordering the endpoints against each other.
    ///
    /// Correlated endpoint pairs can make a conventional AABB unavailable
    /// even though each endpoint interval is immediately separated from a
    /// distant represented coordinate. This is the exact exterior-probe path.
    pub(crate) fn endpoints_strict_axis_order_to_real(
        &self,
        axis: Axis2,
        value: &Real,
        expected: std::cmp::Ordering,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        if !matches!(
            expected,
            std::cmp::Ordering::Less | std::cmp::Ordering::Greater
        ) {
            return Err(CurveError::Topology(
                "strict chord endpoint axis order requires a nonzero direction".into(),
            ));
        }
        let mut last_reason = UncertaintyReason::Ordering;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let mut certified = true;
            for endpoint in [self.start(), self.end()] {
                let bounds = match algebraic_chord_endpoint_local_bounds_refined(
                    endpoint,
                    refinement_steps,
                    policy,
                ) {
                    Classification::Decided(bounds) => bounds,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        certified = false;
                        break;
                    }
                };
                let coordinate = match (axis, expected) {
                    (Axis2::X, std::cmp::Ordering::Greater) => bounds.min().x(),
                    (Axis2::X, std::cmp::Ordering::Less) => bounds.max().x(),
                    (Axis2::Y, std::cmp::Ordering::Greater) => bounds.min().y(),
                    (Axis2::Y, std::cmp::Ordering::Less) => bounds.max().y(),
                    (_, std::cmp::Ordering::Equal) => unreachable!(),
                };
                let order = compare_reals(coordinate, value, policy);
                if order != Some(expected) {
                    if order.is_some() {
                        return Ok(Classification::Decided(false));
                    }
                    last_reason = UncertaintyReason::RealSign;
                    certified = false;
                    break;
                }
            }
            if certified {
                return Ok(Classification::Decided(true));
            }
        }
        Ok(Classification::Uncertain(last_reason))
    }

    /// Proves finite-chord separation from the exact axis-aligned bounds of a
    /// retained circle.  This remains useful when correlated endpoint evidence
    /// makes construction of a conventional chord AABB unnecessarily hard.
    pub(crate) fn certifiably_disjoint_from_circle_bounds(
        &self,
        center: &Point2,
        radius_squared: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        match real_sign(radius_squared, policy) {
            Some(RealSign::Positive | RealSign::Zero) => {}
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "retained circle had negative radius squared".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radius = radius_squared.clone().sqrt()?;
        let mut uncertainty = None;
        for (axis, center_coordinate) in [(Axis2::X, center.x()), (Axis2::Y, center.y())] {
            let lower = center_coordinate - &radius;
            let upper = center_coordinate + &radius;
            let orders = |value: &Real| {
                let start = Self::point_axis_order_to_real(self.start(), axis, value, policy)?;
                let end = Self::point_axis_order_to_real(self.end(), axis, value, policy)?;
                Ok::<_, CurveError>((start, end))
            };
            let (start_lower, end_lower) = orders(&lower)?;
            if matches!(
                (start_lower, end_lower),
                (
                    Classification::Decided(std::cmp::Ordering::Less),
                    Classification::Decided(std::cmp::Ordering::Less)
                )
            ) {
                return Ok(Classification::Decided(true));
            }
            if let Classification::Uncertain(reason) = start_lower {
                uncertainty.get_or_insert(reason);
            }
            if let Classification::Uncertain(reason) = end_lower {
                uncertainty.get_or_insert(reason);
            }
            let (start_upper, end_upper) = orders(&upper)?;
            if matches!(
                (start_upper, end_upper),
                (
                    Classification::Decided(std::cmp::Ordering::Greater),
                    Classification::Decided(std::cmp::Ordering::Greater)
                )
            ) {
                return Ok(Classification::Decided(true));
            }
            if let Classification::Uncertain(reason) = start_upper {
                uncertainty.get_or_insert(reason);
            }
            if let Classification::Uncertain(reason) = end_upper {
                uncertainty.get_or_insert(reason);
            }
        }
        Ok(uncertainty.map_or(Classification::Decided(false), Classification::Uncertain))
    }

    /// Retains an oriented finite chord between two certified points on this
    /// infinite supporting line.
    ///
    /// Neither point is required to lie inside this witness chord. Exact miter intersections
    /// commonly lie beyond an endpoint. Their correlated chord-pair carrier
    /// can order itself against a support endpoint from the stored side signs,
    /// avoiding a fresh multi-field coordinate comparison.
    pub(crate) fn chord_between_certified_support_points(
        &self,
        start: CurvePoint2,
        end: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        self.validate_policy(policy)?;
        let start_parameter = self.parameter_on_retained_support(start);
        let end_parameter = self.parameter_on_retained_support(end);
        let order = match start_parameter.cmp_by_refinement(&end_parameter, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if order == std::cmp::Ordering::Equal {
            return Ok(Classification::Decided(None));
        }
        let forward = order == std::cmp::Ordering::Less;
        let mut parameter_axis = self.data.parameter_axis;
        if !forward {
            parameter_axis.coordinate_increases = !parameter_axis.coordinate_increases;
        }
        let certified_unit_tangent = self.data.certified_unit_tangent.as_ref().map(|tangent| {
            if forward {
                tangent.clone()
            } else {
                Arc::new([-tangent[0].clone(), -tangent[1].clone()])
            }
        });
        let chord = Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start: start_parameter.point().clone(),
                end: end_parameter.point().clone(),
                parameter_axis,
                certified_axis_aligned: self.data.certified_axis_aligned,
                certified_unit_tangent,
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                // These points are certified on this infinite support but may
                // lie outside its finite witness. Keep the immediate support:
                // after an offset translation it can carry a stronger exact
                // affine line than an older ancestral chord, and it is the
                // authored tangent identity needed by adjacent circle joins.
                source: Some(self.clone()),
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        };
        Ok(Classification::Decided(Some(chord)))
    }

    /// Retains an already ordered pair of certified points on this support.
    ///
    /// The caller owns point incidence, distinctness, and agreement with this
    /// chord's traversal order. Exact offset-line construction has all three
    /// facts before endpoint evidence is wrapped, so rebuilding two selected
    /// parameters merely to rediscover their order would discard that proof.
    pub(crate) fn chord_between_certified_ordered_support_points(
        &self,
        start: CurvePoint2,
        end: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        self.validate_policy(policy)?;
        Ok(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start,
                end,
                parameter_axis: self.data.parameter_axis,
                certified_axis_aligned: self.data.certified_axis_aligned,
                certified_unit_tangent: self.data.certified_unit_tangent.clone(),
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: Some(self.clone()),
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        })
    }

    /// Translates retained endpoint evidence by a represented exact vector.
    ///
    /// A one-field algebraic image remains a rational expression in the same
    /// selected root: `N + delta*D` over its original denominator. Correlated
    /// chord-pair intersections need a distinct translated-support carrier and
    /// therefore stay explicit instead of being flattened.
    pub(crate) fn translated(
        &self,
        delta_x: &Real,
        delta_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        let start = match Self::translated_endpoint(&self.data.start, delta_x, delta_y, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match Self::translated_endpoint(&self.data.end, delta_x, delta_y, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source = if self.data.source.is_some() {
            match self
                .retained_support()
                .translated(delta_x, delta_y, policy)?
            {
                Classification::Decided(source) => Some(source),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            None
        };
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start,
                end,
                parameter_axis: self.data.parameter_axis,
                certified_axis_aligned: self.data.certified_axis_aligned,
                certified_unit_tangent: self.data.certified_unit_tangent.clone(),
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source,
                reversed: self.data.reversed,
                policy: policy.retained_object_policy(),
            }),
        }))
    }

    pub(crate) fn translated_endpoint(
        endpoint: &CurvePoint2,
        delta_x: &Real,
        delta_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        match endpoint {
            CurvePoint2(CurvePointData2::Endpoint(_)) => {
                let transform = Similarity2::try_from_real_affine(
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    delta_x.clone(),
                    delta_y.clone(),
                )?;
                Ok(Classification::Decided(CurvePoint2::from(
                    BezierSimilarityPoint2::new(endpoint.clone(), transform, policy),
                )))
            }
            CurvePoint2(CurvePointData2::Exact(point)) => Ok(Classification::Decided(
                CurvePoint2::from(point.translated(delta_x.clone(), delta_y.clone())),
            )),
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                let parameter = match algebraic_chord_image_parameter(point, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let [x, y, denominator] =
                    match algebraic_chord_owned_coordinate_polynomials(point, policy)? {
                        Classification::Decided(coordinates) => coordinates,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let translated = RationalBezierAlgebraicPointImage2::from_retained_expression(
                    parameter.clone(),
                    parameter_representation(&parameter, policy),
                    polynomial_add(&x, &polynomial_scale(&denominator, delta_x)),
                    polynomial_add(&y, &polynomial_scale(&denominator, delta_y)),
                    denominator,
                    "retained an exact translated algebraic chord endpoint",
                );
                Ok(Classification::Decided(CurvePoint2::from(translated)))
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => Ok(point
                .translated(delta_x, delta_y, policy)?
                .map(CurvePoint2::from)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => Ok(Classification::Decided(
                CurvePoint2::from(point.translated(delta_x, delta_y)),
            )),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => Ok(
                Classification::Decided(CurvePoint2::from(point.translated(delta_x, delta_y))),
            ),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => {
                Ok(Classification::Decided(CurvePoint2::from(
                    point.translated(delta_x, delta_y, policy)?,
                )))
            }
            CurvePoint2(CurvePointData2::AnalyticParallel(point)) => Ok(Classification::Decided(
                CurvePoint2::from(point.translated(delta_x, delta_y, policy)?),
            )),
            CurvePoint2(CurvePointData2::Similarity(point)) => Ok(Classification::Decided(
                CurvePoint2::from(point.translated(delta_x, delta_y, policy)?),
            )),
        }
    }

    pub(crate) fn scaled_about_point_endpoint(
        endpoint: &CurvePoint2,
        origin: &Point2,
        scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        match endpoint {
            CurvePoint2(CurvePointData2::Exact(point)) => {
                let radial = point.delta_from(origin);
                Ok(Classification::Decided(CurvePoint2::from(
                    origin.translated(&radial.0 * scale, &radial.1 * scale),
                )))
            }
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                let parameter = match algebraic_chord_image_parameter(point, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let [x, y, denominator] =
                    match algebraic_chord_owned_coordinate_polynomials(point, policy)? {
                        Classification::Decided(coordinates) => coordinates,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let complement = Real::one() - scale;
                let scale_coordinate = |coordinate: Vec<Real>, origin: &Real| {
                    polynomial_add(
                        &polynomial_scale(&coordinate, scale),
                        &polynomial_scale(&denominator, &(origin * &complement)),
                    )
                };
                Ok(Classification::Decided(CurvePoint2::from(
                    RationalBezierAlgebraicPointImage2::from_retained_expression(
                        parameter.clone(),
                        parameter_representation(&parameter, policy),
                        scale_coordinate(x, origin.x()),
                        scale_coordinate(y, origin.y()),
                        denominator,
                        "retained an exact radial image of an algebraic endpoint",
                    ),
                )))
            }
            endpoint @ (CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(
                CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_),
            )) => {
                let complement = Real::one() - scale;
                let transform = Similarity2::try_from_real_affine(
                    scale.clone(),
                    Real::zero(),
                    Real::zero(),
                    scale.clone(),
                    origin.x() * &complement,
                    origin.y() * complement,
                )?;
                Ok(Classification::Decided(CurvePoint2::from(
                    BezierSimilarityPoint2::new(endpoint.clone(), transform, policy),
                )))
            }
        }
    }

    pub(super) fn affine_transformed_endpoint(
        endpoint: &CurvePoint2,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        similarity: Option<&Similarity2>,
        similarity_cache: Option<&mut BezierAlgebraicCuspSemicircleSimilarityCache2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        match endpoint {
            CurvePoint2(CurvePointData2::Exact(point)) => {
                Ok(Classification::Decided(CurvePoint2::from(Point2::new(
                    m00 * point.x() + m01 * point.y() + tx,
                    m10 * point.x() + m11 * point.y() + ty,
                ))))
            }
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                let parameter = match algebraic_chord_image_parameter(point, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let [x, y, denominator] =
                    match algebraic_chord_owned_coordinate_polynomials(point, policy)? {
                        Classification::Decided(coordinates) => coordinates,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let coordinate = |first: &Real, second: &Real, offset: &Real| {
                    polynomial_add(
                        &polynomial_add(
                            &polynomial_scale(&x, first),
                            &polynomial_scale(&y, second),
                        ),
                        &polynomial_scale(&denominator, offset),
                    )
                };
                Ok(Classification::Decided(CurvePoint2::from(
                    RationalBezierAlgebraicPointImage2::from_retained_expression(
                        parameter.clone(),
                        parameter_representation(&parameter, policy),
                        coordinate(m00, m01, tx),
                        coordinate(m10, m11, ty),
                        denominator,
                        "retained an exact affine-transformed algebraic chord endpoint",
                    ),
                )))
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => Ok(point
                .transform_affine(m00, m01, m10, m11, tx, ty, policy)?
                .map(CurvePoint2::from)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => {
                let (Some(similarity), Some(similarity_cache)) = (similarity, similarity_cache)
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                Ok(point
                    .transform_similarity_cached(similarity, policy, similarity_cache)?
                    .map(CurvePoint2::from))
            }
            endpoint @ (CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(
                CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_),
            )) => {
                let Some(similarity) = similarity else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                Ok(Classification::Decided(CurvePoint2::from(
                    BezierSimilarityPoint2::new(endpoint.clone(), similarity.clone(), policy),
                )))
            }
        }
    }

    pub(super) fn affine_transformed_axis_direction(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
    ) -> Option<BezierAlgebraicChordAxisDirection2> {
        if !self.data.certified_axis_aligned {
            return None;
        }
        let sign = Real::from(if self.data.parameter_axis.coordinate_increases {
            1_i8
        } else {
            -1_i8
        });
        let (x, y) = match self.data.parameter_axis.axis {
            Axis2::X => (m00 * &sign, m10 * sign),
            Axis2::Y => (m01 * &sign, m11 * sign),
        };
        // A cardinal direction is reusable construction evidence, not a
        // terminal query result. Never promote an APPROXIMATE_512 zero to a
        // structural axis certificate.
        match (
            real_sign(&x, &CurveContext::STRICT),
            real_sign(&y, &CurveContext::STRICT),
        ) {
            (Some(RealSign::Positive), Some(RealSign::Zero)) => {
                Some(BezierAlgebraicChordAxisDirection2::PositiveX)
            }
            (Some(RealSign::Negative), Some(RealSign::Zero)) => {
                Some(BezierAlgebraicChordAxisDirection2::NegativeX)
            }
            (Some(RealSign::Zero), Some(RealSign::Positive)) => {
                Some(BezierAlgebraicChordAxisDirection2::PositiveY)
            }
            (Some(RealSign::Zero), Some(RealSign::Negative)) => {
                Some(BezierAlgebraicChordAxisDirection2::NegativeY)
            }
            _ => None,
        }
    }

    pub(super) fn affine_transformed_unit_tangent(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
    ) -> CurveResult<Option<Arc<[Real; 2]>>> {
        let Some((x, y)) = self.certified_unit_tangent() else {
            return Ok(None);
        };
        let transformed_x = m00 * &x + m01 * &y;
        let transformed_y = m10 * x + m11 * y;
        let length = (&transformed_x * &transformed_x + &transformed_y * &transformed_y).sqrt()?;
        Ok(Some(Arc::new([
            (transformed_x / &length)?,
            (transformed_y / length)?,
        ])))
    }

    pub(super) fn transform_affine_root(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        similarity: Option<&Similarity2>,
        mut similarity_cache: Option<&mut BezierAlgebraicCuspSemicircleSimilarityCache2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        debug_assert!(self.data.source.is_none());
        let start = match Self::affine_transformed_endpoint(
            &self.data.start,
            m00,
            m01,
            m10,
            m11,
            tx,
            ty,
            similarity,
            similarity_cache.as_deref_mut(),
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match Self::affine_transformed_endpoint(
            &self.data.end,
            m00,
            m01,
            m10,
            m11,
            tx,
            ty,
            similarity,
            similarity_cache,
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let certified_direction = self.affine_transformed_axis_direction(m00, m01, m10, m11);
        let certified_unit_tangent = if certified_direction.is_some() {
            None
        } else {
            self.affine_transformed_unit_tangent(m00, m01, m10, m11)?
        };
        let parameter_axis = match certified_direction {
            Some(direction) => direction.parameter_axis(),
            None => match algebraic_chord_parameter_axis(&start, &end, policy)? {
                Classification::Decided(axis) => axis,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start,
                end,
                parameter_axis,
                certified_axis_aligned: certified_direction.is_some(),
                certified_unit_tangent,
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: None,
                reversed: self.data.reversed,
                policy: policy.retained_object_policy(),
            }),
        }))
    }

    /// Applies a certified nonsingular affine transform while retaining every
    /// independently selected endpoint field and any root support identity.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn transform_affine(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.transform_affine_with_similarity(m00, m01, m10, m11, tx, ty, None, None, policy)
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn transform_affine_with_similarity(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        similarity: Option<&Similarity2>,
        mut similarity_cache: Option<&mut BezierAlgebraicCuspSemicircleSimilarityCache2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        if self.data.source.is_none() {
            return self.transform_affine_root(
                m00,
                m01,
                m10,
                m11,
                tx,
                ty,
                similarity,
                similarity_cache,
                policy,
            );
        }
        let transformed_support = match if let (Some(similarity), Some(cache)) =
            (similarity, similarity_cache.as_deref_mut())
        {
            cache.chord(self.retained_support(), similarity, policy)
        } else {
            self.retained_support()
                .transform_affine_root(m00, m01, m10, m11, tx, ty, None, None, policy)
        }? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start = match Self::affine_transformed_endpoint(
            &self.data.start,
            m00,
            m01,
            m10,
            m11,
            tx,
            ty,
            similarity,
            similarity_cache.as_deref_mut(),
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match Self::affine_transformed_endpoint(
            &self.data.end,
            m00,
            m01,
            m10,
            m11,
            tx,
            ty,
            similarity,
            similarity_cache,
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let orientation_reversed = self.data.parameter_axis.coordinate_increases
            != self
                .retained_support()
                .data
                .parameter_axis
                .coordinate_increases;
        let mut parameter_axis = transformed_support.data.parameter_axis;
        parameter_axis.coordinate_increases ^= orientation_reversed;
        let certified_unit_tangent = if transformed_support.data.certified_axis_aligned {
            None
        } else {
            self.affine_transformed_unit_tangent(m00, m01, m10, m11)?
        };
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start,
                end,
                parameter_axis,
                certified_axis_aligned: transformed_support.data.certified_axis_aligned,
                certified_unit_tangent,
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: Some(transformed_support),
                reversed: self.data.reversed,
                policy: policy.retained_object_policy(),
            }),
        }))
    }

    pub(super) fn transform_similarity_cached(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
        cache: &mut BezierAlgebraicCuspSemicircleSimilarityCache2,
    ) -> CurveResult<Classification<Self>> {
        let (m00, m01, m10, m11, tx, ty) = transform.affine_components();
        self.transform_affine_with_similarity(
            m00,
            m01,
            m10,
            m11,
            tx,
            ty,
            Some(transform),
            Some(cache),
            policy,
        )
    }

    pub(crate) fn validate_policy(&self, policy: &CurveContext) -> CurveResult<()> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "algebraic chord was replayed under a different predicate policy".into(),
            ));
        }
        Ok(())
    }

    /// Retains construction-time transversality for a newly authored bevel.
    /// The flags refer to traversal start/end and survive only transformations
    /// that preserve the same endpoint tangent relationship.
    pub(crate) fn with_certified_circle_transverse_endpoints(
        mut self,
        endpoints: [bool; 2],
    ) -> Self {
        Arc::make_mut(&mut self.data).certified_circle_transverse_endpoints =
            u8::from(endpoints[0]) | (u8::from(endpoints[1]) << 1);
        self
    }

    pub(crate) fn with_parallel_tangent_contacts(
        mut self,
        contacts: Vec<crate::bezier::BezierParallelLineTangentContact2>,
    ) -> Self {
        Arc::make_mut(&mut self.data).parallel_tangent_contacts =
            (!contacts.is_empty()).then(|| Arc::from(contacts));
        self
    }

    pub(crate) fn parallel_tangent_contacts(
        &self,
    ) -> &[crate::bezier::BezierParallelLineTangentContact2] {
        self.data
            .parallel_tangent_contacts
            .as_deref()
            .unwrap_or_default()
    }

    pub(super) fn certified_circle_transverse_endpoint(&self, at_end: bool) -> bool {
        self.data.certified_circle_transverse_endpoints & if at_end { 2 } else { 1 } != 0
    }

    /// Materializes the native fast path when both endpoints are represented.
    pub fn exact_line(&self) -> Option<LineSeg2> {
        let (CurvePoint2(CurvePointData2::Exact(start)), CurvePoint2(CurvePointData2::Exact(end))) =
            (self.start(), self.end())
        else {
            return None;
        };
        LineSeg2::try_new(start.clone(), end.clone()).ok()
    }

    /// Materializes any STRICT-certified affine support without pretending
    /// that the finite algebraic endpoints themselves are represented.
    ///
    /// This lets line/circle and line-side kernels use their compact exact
    /// support path while finite containment remains owned by this chord's
    /// retained endpoint parameters.
    pub(crate) fn strict_retained_support_line(&self, policy: &CurveContext) -> Option<LineSeg2> {
        self.validate_policy(policy).ok()?;
        let [a, b, c] = strict_common_retained_line_coefficients(self.start(), self.end())?;
        let strict = &CurveContext::STRICT;
        let Classification::Decided(bounds) = self.conservative_bounds_refined(0, policy).ok()?
        else {
            return None;
        };
        let mut line = match self.data.parameter_axis.axis {
            Axis2::X => {
                if !matches!(
                    real_sign(&b, strict)?,
                    RealSign::Positive | RealSign::Negative
                ) {
                    return None;
                }
                let point = |x: Real| {
                    let y = ((-(&a * &x) - &c) / &b).ok()?;
                    Some(Point2::new(x, y))
                };
                LineSeg2::try_new(
                    point(bounds.min().x().clone())?,
                    point(bounds.max().x().clone())?,
                )
                .ok()?
            }
            Axis2::Y => {
                if !matches!(
                    real_sign(&a, strict)?,
                    RealSign::Positive | RealSign::Negative
                ) {
                    return None;
                }
                let point = |y: Real| {
                    let x = ((-(&b * &y) - &c) / &a).ok()?;
                    Some(Point2::new(x, y))
                };
                LineSeg2::try_new(
                    point(bounds.min().y().clone())?,
                    point(bounds.max().y().clone())?,
                )
                .ok()?
            }
        };
        let (delta_x, delta_y) = line.delta();
        let component = match self.data.parameter_axis.axis {
            Axis2::X => &delta_x,
            Axis2::Y => &delta_y,
        };
        let line_increases = match real_sign(component, strict)? {
            RealSign::Positive => true,
            RealSign::Negative => false,
            RealSign::Zero => return None,
        };
        if line_increases != self.data.parameter_axis.coordinate_increases {
            line = line.reversed();
        }
        Some(line)
    }

    /// Materializes a construction-certified axis support with a canonical
    /// unit tangent.
    ///
    /// Retained provenance can represent the same line with an arbitrary
    /// nonzero algebraic endpoint separation.  That scale is irrelevant to
    /// support predicates but expensive when it is carried into a resultant
    /// or quadratic formula, so axis-aware consumers use this normalized
    /// representation first.
    pub(super) fn strict_canonical_axis_support_line(
        &self,
        policy: &CurveContext,
    ) -> Option<LineSeg2> {
        self.validate_policy(policy).ok()?;
        let direction = self.certified_axis_direction()?;
        let constant = self
            .constant_axis_coordinate(
                match direction.axis() {
                    Axis2::X => Axis2::Y,
                    Axis2::Y => Axis2::X,
                },
                policy,
            )
            .ok()??;
        let (tangent_x, tangent_y) = direction.unit_tangent();
        let anchor = match direction.axis() {
            Axis2::X => Point2::new(Real::zero(), constant),
            Axis2::Y => Point2::new(constant, Real::zero()),
        };
        LineSeg2::try_new(anchor.clone(), anchor.translated(tangent_x, tangent_y)).ok()
    }

    /// Extends the STRICT support adapter with authored axis and retained
    /// source provenance for algorithms whose finite-domain checks remain on
    /// this chord. Existing Boolean paths deliberately keep the narrower
    /// endpoint-coefficient authority above.
    pub(crate) fn strict_provenance_support_line(&self, policy: &CurveContext) -> Option<LineSeg2> {
        if let Some(line) = self.strict_canonical_axis_support_line(policy) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-support-line",
                "canonical-axis",
            );
            return Some(line);
        }
        if let Some(line) = self.strict_retained_support_line(policy) {
            return Some(line);
        }
        self.validate_policy(policy).ok()?;
        let strict = &CurveContext::STRICT;
        let exact_endpoint = |point: &CurvePoint2| match point {
            CurvePoint2(CurvePointData2::Exact(point)) => Some(point.clone()),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => {
                point.strict_exact_point(policy)
            }
            CurvePoint2(CurvePointData2::Algebraic(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        if let (Some(start), Some(end)) = (exact_endpoint(self.start()), exact_endpoint(self.end()))
            && let Ok(line) = LineSeg2::try_new(start, end)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-support-line",
                "procedural-endpoints-canonicalized",
            );
            return Some(line);
        }
        // A procedural parallel can still have an ordinary canonical-Real
        // support even when one of its finite source endpoints is composite.
        // One exact source point and the retained unit tangent completely
        // determine that support; rebuilding both displaced endpoints would
        // introduce an unnecessary normalization field and can raise the
        // analytic-parallel projection degree by an order of magnitude.
        if let Some(line) = (|| {
            let structural = chord_parallel_support_source(self, policy).ok()??;
            let (source_tangent_x, source_tangent_y) =
                structural.source.certified_unit_tangent()?;
            let source_anchor = structural
                .source
                .start()
                .coordinates()
                .or_else(|| structural.source.end().coordinates())?;
            let (unit_x, unit_y) = match structural.direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    (-source_tangent_y.clone(), source_tangent_x.clone())
                }
                BezierAlgebraicChordUnitDisplacement2::Tangent => {
                    (source_tangent_x.clone(), source_tangent_y.clone())
                }
            };
            let anchor = source_anchor.translated(
                &unit_x * &structural.distance + &structural.translation_x,
                &unit_y * &structural.distance + &structural.translation_y,
            );
            let tangent = self.certified_unit_tangent().or_else(|| {
                let reversed = matches!(
                    self.start(),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(point))
                        if point.at_end
                );
                Some(if reversed {
                    (-source_tangent_x.clone(), -source_tangent_y.clone())
                } else {
                    (source_tangent_x.clone(), source_tangent_y.clone())
                })
            })?;
            LineSeg2::try_new(anchor.clone(), anchor.translated(tangent.0, tangent.1)).ok()
        })() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-support-line",
                "procedural-parallel-exact-anchor",
            );
            return Some(line);
        }
        let inherited = self
            .data
            .source
            .as_ref()
            .and_then(|source| {
                source
                    .exact_line()
                    .or_else(|| source.strict_provenance_support_line(policy))
                    .map(|line| (line, Some(source.data.parameter_axis)))
            })
            .or_else(|| {
                let (
                    CurvePoint2(CurvePointData2::Similarity(start)),
                    CurvePoint2(CurvePointData2::Similarity(end)),
                ) = (self.start(), self.end())
                else {
                    return None;
                };
                if start.data.transform != end.data.transform
                    || !policy.accepts_retained_policy(start.data.policy)
                    || !policy.accepts_retained_policy(end.data.policy)
                {
                    return None;
                }
                let Classification::Decided(source) =
                    Self::try_new_from_certified_distinct_endpoints(
                        start.data.source.clone(),
                        end.data.source.clone(),
                        policy,
                    )
                    .ok()?
                else {
                    return None;
                };
                let source_line = source
                    .exact_line()
                    .or_else(|| source.strict_provenance_support_line(policy));
                let source_line = source_line?;
                LineSeg2::try_new(
                    start.data.transform.transform_point(source_line.start()),
                    start.data.transform.transform_point(source_line.end()),
                )
                .ok()
                .map(|line| (line, None))
            });
        let (mut line, inherited_parameter_axis) = if let Some((line, axis)) = inherited {
            (line, axis)
        } else if let (Some((tangent_x, tangent_y)), Some(anchor)) = (
            self.certified_unit_tangent(),
            self.start()
                .coordinates()
                .or_else(|| self.end().coordinates()),
        ) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-support-line",
                "exact-anchor-certified-tangent",
            );
            return LineSeg2::try_new(anchor.clone(), anchor.translated(tangent_x, tangent_y)).ok();
        } else if self.data.certified_axis_aligned {
            let constant = self
                .constant_axis_coordinate(
                    match self.data.parameter_axis.axis {
                        Axis2::X => Axis2::Y,
                        Axis2::Y => Axis2::X,
                    },
                    policy,
                )
                .ok()??;
            let Classification::Decided(bounds) =
                self.conservative_bounds_refined(0, policy).ok()?
            else {
                return None;
            };
            let line = match self.data.parameter_axis.axis {
                Axis2::X => LineSeg2::try_new(
                    Point2::new(bounds.min().x().clone(), constant.clone()),
                    Point2::new(bounds.max().x().clone(), constant),
                )
                .ok()?,
                Axis2::Y => LineSeg2::try_new(
                    Point2::new(constant.clone(), bounds.min().y().clone()),
                    Point2::new(constant, bounds.max().y().clone()),
                )
                .ok()?,
            };
            (line, None)
        } else {
            return None;
        };
        if let Some(inherited) = inherited_parameter_axis
            && inherited.axis == self.data.parameter_axis.axis
        {
            if inherited.coordinate_increases != self.data.parameter_axis.coordinate_increases {
                line = line.reversed();
            }
            return Some(line);
        }
        let (delta_x, delta_y) = line.delta();
        let component = match self.data.parameter_axis.axis {
            Axis2::X => &delta_x,
            Axis2::Y => &delta_y,
        };
        let line_increases = match real_sign(component, strict)? {
            RealSign::Positive => true,
            RealSign::Negative => false,
            RealSign::Zero => return None,
        };
        if line_increases != self.data.parameter_axis.coordinate_increases {
            line = line.reversed();
        }
        Some(line)
    }

    /// Returns a conservative exact box containing the complete chord.
    pub fn conservative_bounds(&self, policy: &CurveContext) -> CurveResult<Classification<Aabb2>> {
        self.conservative_bounds_refined(0, policy)
    }

    pub(crate) fn conservative_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Aabb2>> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, false)
    }

    pub(crate) fn conservative_local_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Aabb2>> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, true)
    }

    pub(super) fn conservative_bounds_refined_impl(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
        local_only: bool,
    ) -> CurveResult<Classification<Aabb2>> {
        self.validate_policy(policy)?;
        let endpoint_bounds = |endpoint| {
            if local_only {
                algebraic_chord_endpoint_local_bounds_refined(endpoint, refinement_steps, policy)
            } else {
                algebraic_chord_endpoint_bounds_refined(endpoint, refinement_steps, policy)
            }
        };
        let start = match endpoint_bounds(self.start()) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match endpoint_bounds(self.end()) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // Endpoint bounds already certify the geometry. Their union needs
        // conservative enclosures, without ordering unrelated exact scalar
        // expressions tightly. Honor refinement so close chords can separate.
        let start = start
            .certified_rational_outer_envelope(refinement_steps)
            .unwrap_or(start);
        let end = end
            .certified_rational_outer_envelope(refinement_steps)
            .unwrap_or(end);
        Ok(start.union(&end))
    }

    /// Prepares exact independent-field endpoint predicates for this chord.
    pub(crate) fn algebraic_ray_evaluator(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordAlgebraicRay2>> {
        self.validate_policy(policy)?;
        let [start, end] = match algebraic_chord_endpoint_images(self.start(), self.end(), policy)?
        {
            Classification::Decided(images) => images,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(BezierAlgebraicChordAlgebraicRay2 {
            start,
            end,
        }))
    }

    /// Returns the two endpoint signs relative to a ray through an algebraic
    /// query. Ordinary algebraic endpoints retain the direct selected-field
    /// fast path; composite endpoints refine their existing evidence without
    /// constructing a primitive element.
    pub(crate) fn algebraic_ray_endpoint_side_signs(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        side_x: &Real,
        side_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[RealSign; 2]>> {
        if let Classification::Decided(ray) = self.algebraic_ray_evaluator(policy)? {
            return ray.endpoint_side_signs(point, side_x, side_y, policy);
        }
        let mut signs = [RealSign::Zero; 2];
        for (sign, endpoint) in signs.iter_mut().zip([self.start(), self.end()]) {
            *sign = match retained_point_linear_difference_to_algebraic_sign(
                endpoint, point, side_x, side_y, policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        Ok(Classification::Decided(signs))
    }

    /// Classifies exact incidence of an algebraic point with this finite
    /// chord, including composite endpoint carriers.
    pub(crate) fn contains_algebraic_point(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if let Classification::Decided(ray) = self.algebraic_ray_evaluator(policy)? {
            return ray.contains_point(point, policy);
        }
        let evidence = CurvePoint2::from(point.point_image().clone());
        let support = match BezierAlgebraicChordSupportPredicate2::try_new(self, policy)? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match support.oriented_side(&evidence, policy)? {
            Classification::Decided(crate::classify::LineSide::On) => {}
            Classification::Decided(
                crate::classify::LineSide::Left | crate::classify::LineSide::Right,
            ) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(self
            .parameter_at_certified_point(evidence, policy)?
            .map(|parameter| parameter.is_some()))
    }

    /// Returns this chord's half-open winding contribution to a ray whose
    /// origin is an exact algebraic point. Composite endpoints use the same
    /// support predicate as chord/chord Boolean intersection.
    pub(crate) fn algebraic_forward_ray_winding_delta(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        if let Classification::Decided(ray) = self.algebraic_ray_evaluator(policy)? {
            return ray.forward_ray_winding_delta(point, direction_x, direction_y, policy);
        }
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let [start_side, end_side] =
            match self.algebraic_ray_endpoint_side_signs(point, &side_x, &side_y, policy)? {
                Classification::Decided(signs) => signs.map(|sign| match sign {
                    RealSign::Negative => std::cmp::Ordering::Less,
                    RealSign::Zero => std::cmp::Ordering::Equal,
                    RealSign::Positive => std::cmp::Ordering::Greater,
                }),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if matches!(
            (start_side, end_side),
            (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
        ) {
            return Ok(Classification::Decided(0));
        }
        let evidence = CurvePoint2::from(point.point_image().clone());
        let support = match BezierAlgebraicChordSupportPredicate2::try_new(self, policy)? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let side = match support.oriented_side(&evidence, policy)? {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if side == crate::classify::LineSide::On {
            return match self.parameter_at_certified_point(evidence, policy)? {
                Classification::Decided(Some(_)) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                }
                Classification::Decided(None) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Predicate))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        Ok(Classification::Decided(
            if start_side != std::cmp::Ordering::Greater
                && end_side == std::cmp::Ordering::Greater
                && side == crate::classify::LineSide::Left
            {
                1
            } else if start_side == std::cmp::Ordering::Greater
                && end_side != std::cmp::Ordering::Greater
                && side == crate::classify::LineSide::Right
            {
                -1
            } else {
                0
            },
        ))
    }

    /// Omits this chord when an algebraic boundary-side ray starts on its
    /// finite image.
    pub(crate) fn algebraic_forward_ray_winding_delta_skipping_incident_origin(
        &self,
        origin: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<i32>>> {
        self.validate_policy(policy)?;
        let incidence = self.contains_algebraic_point(origin, policy)?;
        self.forward_ray_winding_delta_skipping_incident_origin_from_incidence(
            incidence,
            direction_x,
            direction_y,
            policy,
        )
    }

    pub(super) fn has_composite_endpoint(&self) -> bool {
        [self.start(), self.end()].into_iter().any(|point| {
            matches!(
                point,
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                    | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                    | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                    | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_))
            )
        })
    }

    /// Classifies represented and retained points on this finite chord.
    /// Exact-line, local-bound and native algebraic predicates are strict
    /// accelerators; unresolved queries retain the common support-side and
    /// monotone-parameter proof used by chord/chord intersection.
    pub(crate) fn contains_point(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        if let Some(represented) = point.coordinates() {
            if let Some(line) = self.exact_line() {
                if let decided @ Classification::Decided(_) =
                    policy.strict_predicate_pass(|| line.contains_point(represented, policy))
                {
                    return Ok(decided);
                }
            } else if self.has_composite_endpoint() {
                if let Classification::Decided(bounds) =
                    self.conservative_local_bounds_refined(0, policy)?
                    && bounds.contains_point(represented, &CurveContext::STRICT)
                        == Classification::Decided(false)
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-point-incidence",
                        "strict-local-box-rejection",
                    );
                    return Ok(Classification::Decided(false));
                }
                if self.certified_unit_tangent().is_some() {
                    match policy
                        .strict_predicate_pass(|| self.certified_tangent_side(point, policy))
                    {
                        Classification::Decided(crate::classify::LineSide::On) => {
                            if let Classification::Decided(parameter) = policy
                                .strict_predicate_pass(|| {
                                    self.parameter_at_certified_point(point.clone(), policy)
                                })?
                            {
                                return Ok(Classification::Decided(parameter.is_some()));
                            }
                        }
                        Classification::Decided(
                            crate::classify::LineSide::Left | crate::classify::LineSide::Right,
                        ) => return Ok(Classification::Decided(false)),
                        Classification::Uncertain(_) => {}
                    }
                }
            } else if let Classification::Decided(evaluator) =
                self.algebraic_ray_evaluator(policy)?
                && let decided @ Classification::Decided(_) = policy
                    .strict_predicate_pass(|| evaluator.contains_exact_point(represented, policy))?
            {
                return Ok(decided);
            }
        }
        let support = match BezierAlgebraicChordSupportPredicate2::try_new(self, policy)? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match support.oriented_side(point, policy)? {
            Classification::Decided(crate::classify::LineSide::On) => self
                .parameter_at_certified_point(point.clone(), policy)
                .map(|parameter| parameter.map(|parameter| parameter.is_some())),
            Classification::Decided(
                crate::classify::LineSide::Left | crate::classify::LineSide::Right,
            ) => Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Returns this chord's half-open winding contribution to a forward ray.
    pub(crate) fn forward_ray_winding_delta(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        if let Some(line) = self.exact_line() {
            let side_x = -direction_y.clone();
            let side_y = direction_x.clone();
            let zero = Real::zero();
            let endpoint_order = |point: &Point2| {
                let delta_x = point.x() - origin.x();
                let delta_y = point.y() - origin.y();
                let coordinate = Real::dot2_refs([&delta_x, &delta_y], [&side_x, &side_y]);
                compare_reals(&coordinate, &zero, policy)
            };
            let Some(start_side) = endpoint_order(line.start()) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            };
            let Some(end_side) = endpoint_order(line.end()) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            };
            if matches!(
                (start_side, end_side),
                (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                    | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
            ) {
                return Ok(Classification::Decided(0));
            }
            let origin_side = match classify_oriented_line(line.start(), line.end(), origin, policy)
            {
                Classification::Decided(side) => side,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if origin_side == crate::classify::LineSide::On {
                // A ray collinear with any retained boundary chord is not a
                // generic winding ray, even when its origin lies outside the
                // finite chord. An adjacent curved fragment can meet the ray
                // at either chord endpoint; deciding zero here would let the
                // curve's half-open parameter rule retain only one side of a
                // tangential vertex pair. Let the region classifier select a
                // different exact ray direction instead.
                return Ok(match line.contains_point(origin, policy) {
                    Classification::Decided(true) => {
                        Classification::Uncertain(UncertaintyReason::Boundary)
                    }
                    Classification::Decided(false) => {
                        Classification::Uncertain(UncertaintyReason::Predicate)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                });
            }
            return Ok(Classification::Decided(
                if start_side != std::cmp::Ordering::Greater
                    && end_side == std::cmp::Ordering::Greater
                    && origin_side == crate::classify::LineSide::Left
                {
                    1
                } else if start_side == std::cmp::Ordering::Greater
                    && end_side != std::cmp::Ordering::Greater
                    && origin_side == crate::classify::LineSide::Right
                {
                    -1
                } else {
                    0
                },
            ));
        }
        if real_sign(direction_y, policy) == Some(RealSign::Zero) {
            let direction_x_sign = match real_sign(direction_x, policy) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "algebraic-chord winding ray has zero direction".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            // Axis winding is only a shortcut. Asking a general composite
            // chord to rediscover a zero tangent coordinate can be harder
            // than the complete support predicate (and, for selected endpoint
            // fields, may construct a full tensor resultant merely to prove
            // that this shortcut does not apply). Enter it only from a
            // retained construction certificate; all other chords use the
            // authoritative general winding path below.
            let Some(axis_direction) = self.certified_axis_direction() else {
                return self.forward_ray_winding_delta_general(
                    origin,
                    direction_x,
                    direction_y,
                    policy,
                );
            };
            let coordinate_order =
                |point, axis, value| Self::point_axis_order_to_real(point, axis, value, policy);
            match axis_direction {
                BezierAlgebraicChordAxisDirection2::PositiveX
                | BezierAlgebraicChordAxisDirection2::NegativeX => {
                    let support_order = match coordinate_order(
                        self.retained_support().start(),
                        Axis2::Y,
                        origin.y(),
                    )? {
                        Classification::Decided(order) => order,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    if support_order != std::cmp::Ordering::Equal {
                        return Ok(Classification::Decided(0));
                    }
                    let start_order = match coordinate_order(self.start(), Axis2::X, origin.x())? {
                        Classification::Decided(order) => order,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let end_order = match coordinate_order(self.end(), Axis2::X, origin.x())? {
                        Classification::Decided(order) => order,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    return Ok(
                        if matches!(
                            (start_order, end_order),
                            (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                                | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
                        ) {
                            Classification::Uncertain(UncertaintyReason::Predicate)
                        } else {
                            Classification::Uncertain(UncertaintyReason::Boundary)
                        },
                    );
                }
                BezierAlgebraicChordAxisDirection2::PositiveY
                | BezierAlgebraicChordAxisDirection2::NegativeY => {
                    let start_order = match coordinate_order(self.start(), Axis2::Y, origin.y())? {
                        Classification::Decided(order) => order,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let end_order = match coordinate_order(self.end(), Axis2::Y, origin.y())? {
                        Classification::Decided(order) => order,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let ray_side_order = |order: std::cmp::Ordering| {
                        if direction_x_sign == RealSign::Negative {
                            order.reverse()
                        } else {
                            order
                        }
                    };
                    let start_side = ray_side_order(start_order);
                    let end_side = ray_side_order(end_order);
                    if matches!(
                        (start_side, end_side),
                        (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                            | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
                    ) {
                        return Ok(Classification::Decided(0));
                    }
                    let support_order = match coordinate_order(
                        self.retained_support().start(),
                        Axis2::X,
                        origin.x(),
                    )? {
                        Classification::Decided(order) => order,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    if support_order == std::cmp::Ordering::Equal {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    let origin_side = match (axis_direction, support_order) {
                        (
                            BezierAlgebraicChordAxisDirection2::PositiveY,
                            std::cmp::Ordering::Greater,
                        )
                        | (
                            BezierAlgebraicChordAxisDirection2::NegativeY,
                            std::cmp::Ordering::Less,
                        ) => crate::classify::LineSide::Left,
                        (
                            BezierAlgebraicChordAxisDirection2::PositiveY,
                            std::cmp::Ordering::Less,
                        )
                        | (
                            BezierAlgebraicChordAxisDirection2::NegativeY,
                            std::cmp::Ordering::Greater,
                        ) => crate::classify::LineSide::Right,
                        _ => unreachable!(),
                    };
                    return Ok(Classification::Decided(
                        if start_side != std::cmp::Ordering::Greater
                            && end_side == std::cmp::Ordering::Greater
                            && origin_side == crate::classify::LineSide::Left
                        {
                            1
                        } else if start_side == std::cmp::Ordering::Greater
                            && end_side != std::cmp::Ordering::Greater
                            && origin_side == crate::classify::LineSide::Right
                        {
                            -1
                        } else {
                            0
                        },
                    ));
                }
            }
        }
        self.forward_ray_winding_delta_general(origin, direction_x, direction_y, policy)
    }

    /// Returns this chord's winding contribution while omitting its certified
    /// transverse contact at the ray origin.
    ///
    /// A straight support and a nonparallel ray have exactly one contact, so
    /// incidence plus the certified crossing orientation proves that the
    /// residual contribution is zero without materializing either endpoint.
    pub(crate) fn forward_ray_winding_delta_skipping_origin(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        parameter: &BezierAlgebraicChordParameter2,
        crossing_direction: BezierLineCrossingDirection,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        self.validate_policy(policy)?;
        let origin_evidence = CurvePoint2::from(origin.clone());
        if parameter.point().same_point(&origin_evidence, policy) != Classification::Decided(true) {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        if !parameter.is_certified_strict_interior_of(self) {
            match self.parameter_at_certified_point(origin_evidence, policy)? {
                Classification::Decided(Some(_)) => {}
                Classification::Decided(None) => {
                    return Err(CurveError::Topology(
                        "algebraic-chord origin certificate was outside its source fragment".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let cross = match self
            .tangent_cross_vector_sign(&(direction_x.clone(), direction_y.clone()), policy)?
        {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let certified_direction = match cross {
            RealSign::Positive => BezierLineCrossingDirection::PositiveToNegative,
            RealSign::Negative => BezierLineCrossingDirection::NegativeToPositive,
            RealSign::Zero => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        };
        if certified_direction != crossing_direction {
            return Err(CurveError::Topology(
                "algebraic-chord origin crossing disagreed with its boundary tangent".into(),
            ));
        }
        Ok(Classification::Decided(0))
    }

    /// Omits this chord when a represented boundary-side ray starts on its
    /// finite image.
    ///
    /// A side query is the winding of the open forward ray immediately after
    /// its origin. Every transverse boundary image through that origin must
    /// therefore be omitted, including coincident images from other loops. A
    /// straight chord has no residual contact after its certified origin; a
    /// parallel ray remains a nongeneric boundary direction.
    pub(crate) fn forward_ray_winding_delta_skipping_incident_origin(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<i32>>> {
        self.validate_policy(policy)?;
        let incidence = self.contains_point(&CurvePoint2::from(origin.clone()), policy)?;
        self.forward_ray_winding_delta_skipping_incident_origin_from_incidence(
            incidence,
            direction_x,
            direction_y,
            policy,
        )
    }

    pub(super) fn forward_ray_winding_delta_skipping_incident_origin_from_incidence(
        &self,
        incidence: Classification<bool>,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<i32>>> {
        match incidence {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match self.tangent_cross_vector_sign(&(direction_x.clone(), direction_y.clone()), policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                Ok(Classification::Decided(Some(0)))
            }
            Classification::Decided(RealSign::Zero) => {
                Ok(Classification::Uncertain(UncertaintyReason::Boundary))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(super) fn forward_ray_winding_delta_general(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        if self.has_composite_endpoint() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-ray-winding",
                "composite-endpoint",
            );
            let side_x = -direction_y.clone();
            let side_y = direction_x.clone();
            let start_side = match algebraic_chord_point_linear_order_to_exact(
                self.start(),
                origin,
                &side_x,
                &side_y,
                policy,
            )? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end_side = match algebraic_chord_point_linear_order_to_exact(
                self.end(),
                origin,
                &side_x,
                &side_y,
                policy,
            )? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if matches!(
                (start_side, end_side),
                (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                    | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
            ) {
                return Ok(Classification::Decided(0));
            }
            let support = match BezierAlgebraicChordSupportPredicate2::try_new(self, policy)? {
                Classification::Decided(support) => support,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let side = match support.oriented_side(&CurvePoint2::from(origin.clone()), policy)? {
                Classification::Decided(side) => side,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if side == crate::classify::LineSide::On {
                return match self
                    .parameter_at_certified_point(CurvePoint2::from(origin.clone()), policy)?
                {
                    Classification::Decided(Some(_)) => {
                        Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                    }
                    Classification::Decided(None) => {
                        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
            return Ok(Classification::Decided(
                if start_side != std::cmp::Ordering::Greater
                    && end_side == std::cmp::Ordering::Greater
                    && side == crate::classify::LineSide::Left
                {
                    1
                } else if start_side == std::cmp::Ordering::Greater
                    && end_side != std::cmp::Ordering::Greater
                    && side == crate::classify::LineSide::Right
                {
                    -1
                } else {
                    0
                },
            ));
        }
        let evaluator = match self.algebraic_ray_evaluator(policy)? {
            Classification::Decided(evaluator) => evaluator,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        evaluator.forward_ray_winding_delta_from_exact(origin, direction_x, direction_y, policy)
    }

    /// Replays contacts with the source curve that supplied one selected chord
    /// endpoint.
    ///
    /// The authored endpoint is removed as an exact diagonal fiber factor.
    /// Remaining source parameters are projected and replayed in that selected
    /// field, then represented on the chord by their exact point evidence. A
    /// nonadjacent caller may retain the authored endpoint as well; adjacent
    /// topology already owns it and requests residual contacts only.
    pub(crate) fn source_related_intersections(
        &self,
        source: &RationalBezier2,
        source_parameter: &BezierAlgebraicParameter2,
        include_authored_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalIntersections2>> {
        let system = match self.source_incidence_system(source, source_parameter, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let retained_root = parameter_representation(source_parameter, policy);
        let deflated = deflate_bivariate_fiber_diagonal_root_at_algebraic_parameter(
            &system.incidence,
            CurveResultantParameter::First,
            &retained_root,
            policy.predicate_policy(),
        );
        if deflated.certainty == PredicateCertainty::Approximate {
            policy.observe_approximate_512();
        }
        let residual = match deflated.status {
            AlgebraicFiberDiagonalDeflationStatus::Deflated if deflated.multiplicity > 0 => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-source-incidence",
                    "diagonal-deflated",
                );
                deflated
                    .reduced_polynomial
                    .expect("a deflated local fiber retains its quotient")
            }
            AlgebraicFiberDiagonalDeflationStatus::IdenticallyZeroFiber => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                ));
            }
            AlgebraicFiberDiagonalDeflationStatus::NotARoot => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::NotSourceRelated,
                ));
            }
            AlgebraicFiberDiagonalDeflationStatus::InvalidEvidence => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            AlgebraicFiberDiagonalDeflationStatus::UnsupportedCoefficient => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-source-incidence",
                    "unsupported-coefficient",
                );
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            AlgebraicFiberDiagonalDeflationStatus::Undecided => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            AlgebraicFiberDiagonalDeflationStatus::Deflated => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        };

        let count = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
            &residual,
            CurveResultantParameter::First,
            &retained_root,
            &Real::zero(),
            &Real::one(),
            policy.predicate_policy(),
        );
        if count.certainty == PredicateCertainty::Approximate {
            policy.observe_approximate_512();
        }
        let has_residual_contacts = match count.status {
            AlgebraicFiberRootCountStatus::Counted if count.distinct_root_count == Some(0) => false,
            AlgebraicFiberRootCountStatus::Counted
            | AlgebraicFiberRootCountStatus::EndpointRoot => true,
            AlgebraicFiberRootCountStatus::IdenticallyZeroFiber => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                ));
            }
            AlgebraicFiberRootCountStatus::InvalidEvidence
            | AlgebraicFiberRootCountStatus::InvalidInterval => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            AlgebraicFiberRootCountStatus::Undecided => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        };

        // Diagonal deflation returns a fiber polynomial already reduced
        // modulo the retained root's defining polynomial. Project it directly
        // in that quotient ring; the reduced projector preserves the general
        // resultant as its complete fallback for a degenerate or undecided
        // construction.
        let candidates = if has_residual_contacts {
            match algebraic_selected_reduced_fiber_parameters(
                &residual,
                source_parameter,
                &crate::CurveParameterRange2::unit(),
                policy,
            )? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    candidates,
                )) => candidates,
                Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero)
                | Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            Vec::new()
        };
        let retained_parameter = BezierParameter2::Algebraic(source_parameter.clone());
        // Most authored chord/source pairs have no residual root. Build the
        // tangent-cross polynomial only after the root count proves that a
        // contact needs orientation replay.
        let source_power = source.homogeneous_power_basis()?;
        let [source_tangent_x, source_tangent_y] =
            rational_parametric_tangent_numerator(source_power);
        let tangent_cross = bivariate_subtract(
            &bivariate_outer_product(&system.line_x, &source_tangent_y),
            &bivariate_outer_product(&system.line_y, &source_tangent_x),
        );
        let mut contacts = Vec::with_capacity(
            candidates
                .len()
                .saturating_add(usize::from(include_authored_endpoint)),
        );
        if include_authored_endpoint {
            let point =
                match rational_point_evidence_at_parameter(source, &retained_parameter, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let chord_parameter = match self.parameter_at_certified_point(point.clone(), policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordRationalIntersections2::NotSourceRelated,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let tangent_cross_sign = match algebraic_selected_correlated_predicate_sign(
                &system.incidence,
                &tangent_cross,
                &retained_parameter,
                &retained_parameter,
                policy,
            )? {
                Classification::Decided(sign) => product_sign(sign, system.chord_denominator_sign),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicChordRationalContact2 {
                chord_parameter,
                other_parameter: CurveParameter2::from(retained_parameter.clone()),
                point,
                tangent_cross_sign,
            });
        }
        for candidate in candidates {
            let point = match rational_point_evidence_at_parameter(source, &candidate, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let chord_parameter = match self.parameter_at_certified_point(point.clone(), policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let tangent_cross_sign = match algebraic_selected_correlated_predicate_sign(
                &residual,
                &tangent_cross,
                &retained_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => product_sign(sign, system.chord_denominator_sign),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicChordRationalContact2 {
                chord_parameter,
                other_parameter: CurveParameter2::from(candidate),
                point,
                tangent_cross_sign,
            });
        }
        Ok(Classification::Decided(
            BezierAlgebraicChordRationalIntersections2::Contacts(contacts),
        ))
    }

    /// Returns one algebraic endpoint field that can seed source-related
    /// incidence replay. Transformed images retain their root representation
    /// even when no deferred coordinate expression remains cached.
    pub(crate) fn algebraic_endpoint_parameter(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicParameter2>>> {
        for point in [self.start(), self.end()] {
            let CurvePoint2(CurvePointData2::Algebraic(point)) = point else {
                continue;
            };
            return algebraic_chord_image_parameter(point, policy)
                .map(|parameter| parameter.map(Some));
        }
        Ok(Classification::Decided(None))
    }

    /// Certifies that the oriented support incidence is strictly monotone on
    /// one retained analytic-parallel range.
    ///
    /// A boundary chord adjacent to that range already owns one common
    /// endpoint.  If `cross(chord_direction, parallel_tangent)` has one
    /// nonzero sign everywhere, the signed support incidence has at most one
    /// zero, so the authored endpoint is the complete finite contact set.
    /// Endpoint and tangent boxes are only sufficient certificates: an
    /// unresolved enclosure falls through to the general incidence kernel.
    pub(crate) fn parallel_tangent_cross_sign_on_region_range(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        policy.strict_predicate_pass(|| {
            let strict = policy;
            // Splits, reversals, and exact normal/tangent displacements preserve
            // one authored direction up to sign.  Refine that least-field
            // authority instead of the displaced finite endpoints: the latter
            // add normalization radicals which cancel completely from this
            // tangent-cross predicate.
            let (direction_authority, direction_reversed) = self.tangent_authority();
            #[cfg(feature = "dispatch-trace")]
            if !Arc::ptr_eq(&self.data, &direction_authority.data) {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotonicity",
                    "retained-tangent-authority",
                );
            }
            let range_order = match range.start().cmp_by_refinement(range.end(), strict)? {
                Classification::Decided(std::cmp::Ordering::Less) => std::cmp::Ordering::Less,
                Classification::Decided(std::cmp::Ordering::Greater) => std::cmp::Ordering::Greater,
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Err(CurveError::InvalidBezierRange);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let certified_tangent_endpoint =
                |parameter: &CurveParameter2| -> CurveResult<bool> {
                    for contact in self
                        .parallel_tangent_contacts()
                        .iter()
                        .chain(direction_authority.parallel_tangent_contacts())
                    {
                        if contact.parallel() != parallel {
                            continue;
                        }
                        let retained = CurveParameter2::from(BezierParameter2::Exact(
                            contact.parameter().clone(),
                        ));
                        if parameter.cmp_by_refinement(&retained, strict)?
                            == Classification::Decided(std::cmp::Ordering::Equal)
                        {
                            return Ok(true);
                        }
                    }
                    Ok(false)
                };
            let retained_identity_tangent_sign =
                |parameter: &CurveParameter2| -> CurveResult<Option<RealSign>> {
                    let Some(parameter) = parameter.as_recursive_projective() else {
                        return Ok(None);
                    };
                    let BezierParallelSource2::Rational(source) = parallel.source() else {
                        return Ok(None);
                    };
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        match parameter.data.identity.as_deref() {
                            None => eprintln!("retained tangent identity: none"),
                            Some(BezierRecursiveProjectiveParameterIdentity2::Line(_)) => {
                                eprintln!("retained tangent identity: line")
                            }
                            Some(
                                BezierRecursiveProjectiveParameterIdentity2::ChordRationalTangent(
                                    identity,
                                ),
                            ) => {
                                let kind = |point: &CurvePoint2| {
                                    match point {
                                        CurvePoint2(CurvePointData2::Exact(_)) => {
                                            "exact"
                                        }
                                        CurvePoint2(CurvePointData2::Algebraic(_)) => {
                                            "algebraic"
                                        }
                                        CurvePoint2(CurvePointData2::AlgebraicChordPair(
                                            _,
                                        )) => "pair",
                                        CurvePoint2(CurvePointData2::AlgebraicCuspChord(
                                            _,
                                        )) => "cusp",
                                        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(
                                            _,
                                        )) => "derived",
                                        CurvePoint2(CurvePointData2::AlgebraicChordParallel(
                                            _,
                                        )) => "parallel",
                                        CurvePoint2(CurvePointData2::AnalyticParallel(
                                            _,
                                        )) => "analytic",
                                        CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                                            "similarity"
                                        }
                                    }
                                };
                                let support = direction_authority.retained_support();
                                let procedural = chord_parallel_support_source(
                                    direction_authority,
                                    strict,
                                )
                                .ok()
                                .flatten();
                                eprintln!(
                                    "retained tangent identity: chord source_match={} orientation={:?} current=({},{}) support=({},{}) procedural={:?} procedural_source=({},{}) source_orientation={:?} identity=({},{})",
                                    source == &identity.source,
                                    identity.chord.shared_tangent_orientation(direction_authority),
                                    kind(direction_authority.start()),
                                    kind(direction_authority.end()),
                                    kind(support.start()),
                                    kind(support.end()),
                                    procedural.as_ref().map(|value| value.direction),
                                    procedural
                                        .as_ref()
                                        .map_or("n/a", |value| kind(value.source.start())),
                                    procedural
                                        .as_ref()
                                        .map_or("n/a", |value| kind(value.source.end())),
                                    procedural.as_ref().and_then(|value| identity
                                        .chord
                                        .shared_tangent_orientation(&value.source)),
                                    kind(identity.chord.start()),
                                    kind(identity.chord.end()),
                                );
                                if let (
                                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
                                ) = (direction_authority.start(), direction_authority.end())
                                {
                                    eprintln!(
                                        "retained tangent parallel endpoints carrier={} at_end=[{},{}] direction=[{:?},{:?}] source-points=[{},{}] sources-identity=[{},{}] source-orientation={:?} distance-equal={} translation-equal={} point-storage={:?} point-equality={:?}",
                                        start.shares_carrier(end),
                                        start.at_end,
                                        end.at_end,
                                        start.data.direction,
                                        end.data.direction,
                                        kind(start.source_endpoint()),
                                        kind(end.source_endpoint()),
                                        start.data.source.shares_retained_support(&identity.chord),
                                        end.data.source.shares_retained_support(&identity.chord),
                                        start.data.source.shared_tangent_orientation(&end.data.source),
                                        start.data.distance == end.data.distance,
                                        start.data.translation_x == end.data.translation_x
                                            && start.data.translation_y == end.data.translation_y,
                                        [identity.chord.start(), identity.chord.end()].map(|endpoint| {
                                            [start.source_endpoint(), end.source_endpoint()]
                                                .map(|point| point.shares_storage(endpoint))
                                        }),
                                        [identity.chord.start(), identity.chord.end()].map(|endpoint| {
                                            [start.source_endpoint(), end.source_endpoint()]
                                                .map(|point| point == endpoint)
                                        }),
                                    );
                                }
                            }
                        }
                    }
                    let Some(sign) = parameter.chord_rational_tangent_cross_sign(
                        direction_authority,
                        source,
                        RealSign::Positive,
                        strict,
                    ) else {
                        return Ok(None);
                    };
                    Ok(match sign? {
                        Classification::Decided(sign) => Some(sign),
                        Classification::Uncertain(_) => None,
                    })
                };
            let certified_source_sign = |parameter: &CurveParameter2,
                                         tangent: bool|
             -> CurveResult<Option<RealSign>> {
                let identity = retained_identity_tangent_sign(parameter)?;
                if tangent && identity.is_some_and(|sign| sign != RealSign::Zero) {
                    return Err(CurveError::Topology(
                        "retained tangent and transverse endpoint certificates conflicted".into(),
                    ));
                }
                Ok(identity.or(tangent.then_some(RealSign::Zero)))
            };
            let singularities = match parallel.singularity_analysis(range, strict)? {
                Classification::Decided(analysis) => analysis,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for parameter in singularities
                .source_singularities()
                .iter()
                .chain(singularities.parallel_cusps())
            {
                let parameter = CurveParameter2::from(parameter.clone());
                let start_order = match parameter.cmp_by_refinement(range.start(), strict)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let end_order = match parameter.cmp_by_refinement(range.end(), strict)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let lies_strictly_inside = match range_order {
                    std::cmp::Ordering::Less => {
                        start_order == std::cmp::Ordering::Greater
                            && end_order == std::cmp::Ordering::Less
                    }
                    std::cmp::Ordering::Greater => {
                        start_order == std::cmp::Ordering::Less
                            && end_order == std::cmp::Ordering::Greater
                    }
                    std::cmp::Ordering::Equal => unreachable!("validated above"),
                };
                if lies_strictly_inside {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-parallel-monotonicity",
                        "interior-singularity",
                    );
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            }
            let interior = match range.strict_interior_scalar(strict)? {
                Classification::Decided(interior) => interior,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let derivative_scale = match parallel
                .parallel_derivative_scale_sign_at_exact(&interior, strict)?
            {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let regularized = match parallel
                .source_oriented_regularized_tangent_field_at_interior(&interior, strict)?
            {
                Classification::Decided(field) => field,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let authored_tangent_at_range_start = direction_authority
                .is_authored_source_tangent_at_region_parameter(
                    parallel,
                    range.start(),
                    regularized.as_deref(),
                    strict,
                );
            let authored_tangent_at_range_end = direction_authority
                .is_authored_source_tangent_at_region_parameter(
                    parallel,
                    range.end(),
                    regularized.as_deref(),
                    strict,
                );
            #[cfg(feature = "dispatch-trace")]
            if authored_tangent_at_range_start || authored_tangent_at_range_end {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotonicity",
                    "authored-source-tangent-endpoint",
                );
            }
            let certified_tangent_at_range_start = certified_tangent_endpoint(range.start())?
                || authored_tangent_at_range_start;
            let certified_tangent_at_range_end = certified_tangent_endpoint(range.end())?
                || authored_tangent_at_range_end;
            // A join chord whose endpoints are independently displaced from
            // authored source points has a direction involving two distinct
            // unit-normal fields.  Endpoint-box refinement is only an
            // optional monotonicity certificate here; merging those complete
            // point fields can dwarf the authoritative chord/parallel
            // incidence kernel that follows this declined fast path.
            let endpoint_specific_displacement_direction = matches!(
                (direction_authority.start(), direction_authority.end()),
                (
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
                ) if start.data.source_point.is_some() || end.data.source_point.is_some()
            );
            if direction_authority.certified_unit_tangent().is_none()
                && endpoint_specific_displacement_direction
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotonicity",
                    "defer-composite-join-direction",
                );
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let source_sign_at_range_start =
                certified_source_sign(range.start(), certified_tangent_at_range_start)?;
            let source_sign_at_range_end =
                certified_source_sign(range.end(), certified_tangent_at_range_end)?;
            let (source_sign_at_lower, source_sign_at_upper) = match range_order {
                std::cmp::Ordering::Less => {
                    (source_sign_at_range_start, source_sign_at_range_end)
                }
                std::cmp::Ordering::Greater => {
                    (source_sign_at_range_end, source_sign_at_range_start)
                }
                std::cmp::Ordering::Equal => unreachable!("validated above"),
            };
            let differential;
            let (tangent_x_coefficients, tangent_y_coefficients) = match regularized.as_deref() {
                Some(field) => (&field.x[..], &field.y[..]),
                None => {
                    differential = parallel.differential()?;
                    (&differential.tangent_x[..], &differential.tangent_y[..])
                }
            };
            let certified_direction_values = direction_authority.certified_unit_tangent();
            let certified_direction = certified_direction_values.as_ref().map(|(x, y)| {
                (
                    RealInterval::from_values(
                        [x.clone()],
                    )
                    .expect("one certified tangent coordinate defines an exact interval"),
                    RealInterval::from_values(
                        [y.clone()],
                    )
                    .expect("one certified tangent coordinate defines an exact interval"),
                )
            });
            for (steps, precision) in [
                (0, -32),
                (2, -64),
                (4, -96),
                (8, -128),
                (16, -192),
                (32, -256),
                (64, -384),
                (128, -512),
                (256, -768),
                (512, -1024),
            ] {
                let endpoint_direction = if certified_direction.is_none() {
                    let (
                        Classification::Decided(start_bounds),
                        Classification::Decided(end_bounds),
                    ) = (
                        algebraic_chord_endpoint_bounds_refined(
                            direction_authority.start(),
                            steps,
                            policy,
                        ),
                        algebraic_chord_endpoint_bounds_refined(
                            direction_authority.end(),
                            steps,
                            policy,
                        ),
                    )
                    else {
                        continue;
                    };
                    Some((
                        RealInterval::from_axis(&end_bounds, Axis2::X)
                            .subtract(&RealInterval::from_axis(
                                &start_bounds,
                                Axis2::X,
                            )),
                        RealInterval::from_axis(&end_bounds, Axis2::Y)
                            .subtract(&RealInterval::from_axis(
                                &start_bounds,
                                Axis2::Y,
                            )),
                    ))
                } else {
                    None
                };
                let refined_start =
                    match range.start().refined_for_finite_envelope(steps, strict)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => continue,
                    };
                let refined_end = match range.end().refined_for_finite_envelope(steps, strict)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(_) => continue,
                };
                let (Some(start_bounds), Some(end_bounds)) = (
                    refined_start.finite_envelope_bounds(),
                    refined_end.finite_envelope_bounds(),
                ) else {
                    continue;
                };
                let (parameter_lower, parameter_upper) = match range_order {
                    std::cmp::Ordering::Less => (start_bounds.0, end_bounds.1),
                    std::cmp::Ordering::Greater => (end_bounds.0, start_bounds.1),
                    std::cmp::Ordering::Equal => unreachable!("validated above"),
                };
                let Some([tangent_x_lower, tangent_x_upper]) =
                    coefficients_value_interval_on_real_interval(
                        tangent_x_coefficients,
                        parameter_lower,
                        parameter_upper,
                        precision,
                    )?
                else {
                    continue;
                };
                let Some([tangent_y_lower, tangent_y_upper]) =
                    coefficients_value_interval_on_real_interval(
                        tangent_y_coefficients,
                        parameter_lower,
                        parameter_upper,
                        precision,
                    )?
                else {
                    continue;
                };
                let (direction_x, direction_y) = certified_direction
                    .as_ref()
                    .or(endpoint_direction.as_ref())
                    .expect("a retained chord supplies either a certified or endpoint direction");
                let tangent_x = RealInterval {
                    lower: Real::new(tangent_x_lower),
                    upper: Real::new(tangent_x_upper),
                };
                let tangent_y = RealInterval {
                    lower: Real::new(tangent_y_lower),
                    upper: Real::new(tangent_y_upper),
                };
                let Some(cross) = direction_x.multiply(&tangent_y).and_then(|first| {
                    direction_y
                        .multiply(&tangent_x)
                        .map(|second| first.subtract(&second))
                }) else {
                    continue;
                };
                let zero = Real::zero();
                let source_sign = if compare_reals(&cross.lower, &zero, strict)
                    == Some(std::cmp::Ordering::Greater)
                {
                    Some(RealSign::Positive)
                } else if compare_reals(&cross.upper, &zero, strict)
                    == Some(std::cmp::Ordering::Less)
                {
                    Some(RealSign::Negative)
                } else {
                    None
                };
                if let Some(source_sign) = source_sign {
                    let sign = product_sign(source_sign, derivative_scale);
                    return Ok(Classification::Decided(if direction_reversed {
                        product_sign(sign, RealSign::Negative)
                    } else {
                        sign
                    }));
                }

                // Direct range evaluation loses the endpoint correlation and
                // can contain zero forever when the tangent cross vanishes at
                // a closed endpoint.  Restrict the (at most quadratic in the
                // hot path) cross polynomial to this exact parameter box and
                // inspect its Bernstein controls instead.  Nonnegative
                // controls with one certified positive control (or the
                // negative counterpart) prove a strict sign throughout the
                // open retained range without isolating any recursive-field
                // roots.
                let degree = tangent_x_coefficients
                    .len()
                    .max(tangent_y_coefficients.len())
                    .saturating_sub(1);
                if degree <= 2 {
                    let exact_interval = |value: Real| RealInterval {
                        lower: value.clone(),
                        upper: value,
                    };
                    let cross_coefficient = |power: usize| {
                        let tangent_x = tangent_x_coefficients
                            .get(power)
                            .cloned()
                            .unwrap_or_else(Real::zero);
                        let tangent_y = tangent_y_coefficients
                            .get(power)
                            .cloned()
                            .unwrap_or_else(Real::zero);
                        direction_x
                            .multiply(&exact_interval(tangent_y))
                            .and_then(|first| {
                                direction_y
                                    .multiply(&exact_interval(tangent_x))
                                    .map(|second| first.subtract(&second))
                            })
                    };
                    let coefficients = [
                        cross_coefficient(0),
                        cross_coefficient(1),
                        cross_coefficient(2),
                    ];
                    if let [Some(c0), Some(c1), Some(c2)] = coefficients {
                        let (start_lower, start_upper, end_lower, end_upper) = match range_order {
                            std::cmp::Ordering::Less => {
                                (start_bounds.0, start_bounds.1, end_bounds.0, end_bounds.1)
                            }
                            std::cmp::Ordering::Greater => {
                                (end_bounds.0, end_bounds.1, start_bounds.0, start_bounds.1)
                            }
                            std::cmp::Ordering::Equal => unreachable!("validated above"),
                        };
                        let start = RealInterval {
                            lower: start_lower.clone(),
                            upper: start_upper.clone(),
                        };
                        let end = RealInterval {
                            lower: end_lower.clone(),
                            upper: end_upper.clone(),
                        };
                        let delta = end.subtract(&start);
                        let evaluate = |parameter: &RealInterval| {
                            c2.multiply(parameter)
                                .map(|value| value.add(&c1))?
                                .multiply(parameter)
                                .map(|value| value.add(&c0))
                        };
                        let controls = (|| {
                            let q0 = evaluate(&start)?;
                            let derivative = c2
                                .multiply(&start)?
                                .multiply(&exact_interval(Real::from(2_i8)))?
                                .add(&c1);
                            let q1 = delta.multiply(&derivative)?;
                            let first = q0.clone();
                            let middle = q0.add(&q1.multiply(
                                &exact_interval(
                                    (Real::one() / Real::from(2_i8)).expect("two is nonzero"),
                                ),
                            )?);
                            // Endpoint evaluation avoids the dependency loss
                            // from expanding (end - start) a second time.
                            let last = evaluate(&end)?;
                            Some([first, middle, last])
                        })();
                        if let Some(controls) = controls {
                            let zero = Real::zero();
                            let mut control_signs = controls.each_ref().map(|control| {
                                (
                                    compare_reals(&control.lower, &zero, strict),
                                    compare_reals(&control.upper, &zero, strict),
                                )
                            });
                            let sign_orders = |sign| {
                                let order = match sign {
                                    RealSign::Negative => std::cmp::Ordering::Less,
                                    RealSign::Zero => std::cmp::Ordering::Equal,
                                    RealSign::Positive => std::cmp::Ordering::Greater,
                                };
                                (Some(order), Some(order))
                            };
                            if let Some(sign) = source_sign_at_lower {
                                control_signs[0] = sign_orders(sign);
                            }
                            if let Some(sign) = source_sign_at_upper {
                                control_signs[2] = sign_orders(sign);
                            }
                            let strict_endpoint_sign = |(lower, upper)| {
                                if lower == Some(std::cmp::Ordering::Greater) {
                                    Some(RealSign::Positive)
                                } else if upper == Some(std::cmp::Ordering::Less) {
                                    Some(RealSign::Negative)
                                } else {
                                    None
                                }
                            };
                            if strict_signs_are_opposite(
                                strict_endpoint_sign(control_signs[0]),
                                strict_endpoint_sign(control_signs[2]),
                            ) {
                                // These controls enclose the actual endpoint
                                // values. Opposite signs prove an interior zero,
                                // so complete point-field projection cannot
                                // make this optional monotonicity proof succeed.
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-chord-parallel-monotonicity",
                                    "opposed-endpoint-signs",
                                );
                                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                            }
                            #[cfg(test)]
                            if steps == 512
                                && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
                            {
                                eprintln!(
                                    "retained range Bernstein controls={control_signs:?} endpoints=[{source_sign_at_lower:?},{source_sign_at_upper:?}]",
                                );
                            }
                            let nonnegative = control_signs.iter().all(|(lower, _)| {
                                matches!(
                                    lower,
                                    Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater)
                                )
                            });
                            let nonpositive = control_signs.iter().all(|(_, upper)| {
                                matches!(
                                    upper,
                                    Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                                )
                            });
                            let positive = control_signs.iter().any(|(lower, _)| {
                                *lower == Some(std::cmp::Ordering::Greater)
                            });
                            let negative = control_signs.iter().any(|(_, upper)| {
                                *upper == Some(std::cmp::Ordering::Less)
                            });
                            let source_sign = if nonnegative && positive {
                                Some(RealSign::Positive)
                            } else if nonpositive && negative {
                                Some(RealSign::Negative)
                            } else {
                                None
                            };
                            if let Some(source_sign) = source_sign {
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-chord-parallel-monotonicity",
                                    "retained-range-bernstein-interval",
                                );
                                let sign = product_sign(source_sign, derivative_scale);
                                return Ok(Classification::Decided(if direction_reversed {
                                    product_sign(sign, RealSign::Negative)
                                } else {
                                    sign
                                }));
                            }
                        }
                    }
                }
            }

            // A closed range can legitimately include a stationary common
            // endpoint.  Its interval hull contains zero forever even when the
            // tangent cross is strict everywhere in the open span.  Bernstein
            // positivity is the exact finite certificate for that case: zero
            // endpoint controls are harmless because every basis function is
            // positive on `(0, 1)`. These unit-chart certificates apply only
            // when that chart covers the entire retained range.
            let unit = CurveParameterRange2::unit();
            let unit_covers_range = CurveParameterDomain2::new(&unit, None)
                .contains_finite_range(range, strict)?
                == Classification::Decided(true);
            let represented_source_sign =
                certified_direction_values
                    .as_ref()
                    .filter(|_| unit_covers_range)
                    .and_then(|(direction_x, direction_y)| {
                        let degree = tangent_x_coefficients
                            .len()
                            .max(tangent_y_coefficients.len())
                            .checked_sub(1)?;
                        let coefficients = (0..=degree)
                            .map(|power| {
                                direction_x
                                    * tangent_y_coefficients
                                        .get(power)
                                        .cloned()
                                        .unwrap_or_else(Real::zero)
                                    - direction_y
                                        * tangent_x_coefficients
                                            .get(power)
                                            .cloned()
                                            .unwrap_or_else(Real::zero)
                            })
                            .collect::<Vec<_>>();
                        let controls =
                            power_to_bernstein_coefficients(&coefficients, degree).ok()?;
                        let mut retained = None;
                        for control in controls {
                            match (retained, real_sign(&control, strict)?) {
                                (None, sign @ (RealSign::Positive | RealSign::Negative)) => {
                                    retained = Some(sign)
                                }
                                (
                                    Some(expected),
                                    actual @ (RealSign::Positive | RealSign::Negative),
                                ) if expected != actual => return None,
                                (_, RealSign::Zero) | (Some(_), _) => {}
                            }
                        }
                        retained
                    });
            let mut recursive_cross = None;
            let mut source_sign = represented_source_sign;
            if source_sign.is_none()
                && let Classification::Decided(Some(frame)) =
                    direction_authority.recursive_projective_endpoints_with_direction(strict)?
            {
                let [start, end] = frame.direction_endpoints;
                let Some((direction_x, direction_y, _)) = end.difference_numerators(&start) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let field = direction_x.field();
                let Some(tangent_x) =
                    recursive_quadratic_real_polynomial(&field, tangent_x_coefficients)
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some(tangent_y) =
                    recursive_quadratic_real_polynomial(&field, tangent_y_coefficients)
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some(cross) = recursive_quadratic_polynomial_scale(&tangent_y, &direction_x)
                    .and_then(|first| {
                        recursive_quadratic_polynomial_scale(&tangent_x, &direction_y).and_then(
                            |second| recursive_quadratic_polynomial_combine(&first, &second, true),
                        )
                    })
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                if unit_covers_range {
                    source_sign = recursive_quadratic_open_unit_bernstein_sign(&cross);
                }
                recursive_cross = Some((field, cross));
            }

            // Mixed Bernstein controls do not imply a root on this retained
            // subrange.  Tangent-cross degree is normally at most quadratic, so
            // isolate its recursive-field roots directly (without a dense norm)
            // and accept endpoint roots while rejecting any strict interior root.
            if source_sign.is_none()
                && let Some((field, cross)) = recursive_cross.as_ref()
                && cross.len() <= 3
            {
                let (roots, mut root_classification_complete) =
                    match recursive_projective_polynomial_parameters(
                        field,
                        cross.clone(), SelectedThirdAxisDomain2::Finite(range),
                        strict,
                    )? {
                        Classification::Decided(roots) => (roots, true),
                        Classification::Uncertain(_) => (Vec::new(), false),
                    };
                let mut has_interior_root = false;
                for root in roots {
                    let first = root.cmp_by_refinement(range.start(), strict)?;
                    let second = root.cmp_by_refinement(range.end(), strict)?;
                    let (Classification::Decided(first), Classification::Decided(second)) =
                        (first, second)
                    else {
                        root_classification_complete = false;
                        break;
                    };
                    has_interior_root |= match range_order {
                        std::cmp::Ordering::Less => {
                            first == std::cmp::Ordering::Greater
                                && second == std::cmp::Ordering::Less
                        }
                        std::cmp::Ordering::Greater => {
                            first == std::cmp::Ordering::Less
                                && second == std::cmp::Ordering::Greater
                        }
                        std::cmp::Ordering::Equal => unreachable!("validated above"),
                    };
                }
                if root_classification_complete && !has_interior_root {
                    let mut sample = field.constant(Real::zero()).ok_or_else(|| {
                        CurveError::Topology("a recursive tangent cross lost its zero".into())
                    })?;
                    for coefficient in cross.iter().rev() {
                        sample = sample
                            .scale(&interior)
                            .and_then(|value| value.add(coefficient))
                            .ok_or_else(|| {
                                CurveError::Topology(
                                    "a recursive tangent-cross sample exceeded its field budget"
                                        .into(),
                                )
                            })?;
                    }
                    source_sign = match sample.sign(strict)? {
                        Classification::Decided(
                            sign @ (RealSign::Positive | RealSign::Negative),
                        ) => Some(sign),
                        Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => {
                            None
                        }
                    };
                }
            }
            if let Some(source_sign) = source_sign {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotonicity",
                    "retained-range-sign",
                );
                let sign = product_sign(source_sign, derivative_scale);
                return Ok(Classification::Decided(if direction_reversed {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }));
            }
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        })
    }

    /// Publishes the unique contact proved by a strictly monotone support
    /// incidence without forming the recursive coefficient tower's global
    /// norm.  Opposite endpoint sides provide existence, the supplied
    /// nonzero tangent-cross sign provides uniqueness, and exact scalar
    /// bisection retains an authored-sheet bracket for every later predicate.
    pub(crate) fn retained_monotone_parallel_contact_on_region_range(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        endpoint_sides: [crate::classify::LineSide; 2],
        tangent_cross_sign: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordRetainedParallelContact2>>> {
        self.validate_policy(policy)?;
        if tangent_cross_sign == RealSign::Zero {
            return Err(CurveError::Topology(
                "a monotone chord/parallel contact requires a nonzero tangent cross".into(),
            ));
        }
        let prepared = policy.strict_predicate_pass(|| {
            let strict = policy;
            let range_order = match range.start().cmp_by_refinement(range.end(), strict)? {
                Classification::Decided(
                    order @ (std::cmp::Ordering::Less | std::cmp::Ordering::Greater),
                ) => order,
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Err(CurveError::InvalidBezierRange);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let interior = match range.strict_interior_scalar(strict)? {
                Classification::Decided(interior) => interior,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let frame = match parallel
                .source_oriented_regularized_tangent_field_at_interior(&interior, strict)?
            {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if let Classification::Uncertain(reason) = parallel.certify_source_frame_in_domain(
                SelectedThirdAxisDomain2::Finite(range),
                frame.as_deref(),
                strict,
            )? {
                return Ok(Classification::Uncertain(reason));
            }
            // Splitting and Boolean walking can replace one finite endpoint
            // with a correlated chord-pair contact while retaining the exact
            // authored supporting line.  Incidence depends only on that line;
            // importing the clipped endpoint would unnecessarily join every
            // field owned by both defining supports.  Keep `self` as the
            // finite-domain authority below, but construct the root equation
            // from its smallest retained affine support.
            let (support, support_reversed) = self.smallest_incidence_support();
            #[cfg(feature = "dispatch-trace")]
            if !Arc::ptr_eq(&self.data, &support.data) {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotone",
                    "retained-support-incidence",
                );
            }
            let system = match support.recursive_projective_parallel_system_with_frame(
                parallel,
                frame.as_deref(),
                false,
                strict,
            )? {
                Classification::Decided(Some(system)) => Arc::new(system),
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let weight_sign = match strict.strict_predicate_pass(|| {
                system.polynomial_sign_at_real(&system.source_weight, &interior, strict)
            })? {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let side_on_support = |side| {
                if !support_reversed {
                    return side;
                }
                match side {
                    crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                    crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                    crate::classify::LineSide::On => crate::classify::LineSide::On,
                }
            };
            let incidence_sign = |side| match side {
                crate::classify::LineSide::Left => Some(weight_sign),
                crate::classify::LineSide::Right => {
                    Some(product_sign(weight_sign, RealSign::Negative))
                }
                crate::classify::LineSide::On => None,
            };
            let [Some(first_sign), Some(second_sign)] =
                endpoint_sides.map(side_on_support).map(incidence_sign)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            };
            if !strict_signs_are_opposite(Some(first_sign), Some(second_sign)) {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            let (mut lower_parameter, mut upper_parameter, lower_sign, upper_sign) =
                match range_order {
                    std::cmp::Ordering::Less => (
                        range.start().clone(),
                        range.end().clone(),
                        first_sign,
                        second_sign,
                    ),
                    std::cmp::Ordering::Greater => (
                        range.end().clone(),
                        range.start().clone(),
                        second_sign,
                        first_sign,
                    ),
                    std::cmp::Ordering::Equal => unreachable!("validated above"),
                };
            let mut lower_real = lower_parameter.scalar().cloned();
            let mut upper_real = upper_parameter.scalar().cloned();
            // A retained monotone bracket may not be authored from an
            // approximate midpoint sign. When the defining chord is
            // replayable under STRICT, finish that pure polynomial sign with
            // exact authority instead of treating the caller's bounded pass
            // as the completeness boundary. Retained point and range
            // topology below must keep the caller's policy identity: those
            // inputs can legitimately have been authored by APPROXIMATE_512.
            let exact_completion_policy = policy.strict_counterpart();
            let midpoint_sign_policy = if policy.has_bounded_exact_predicate_budget()
                && chord_parallel_support_source(self, policy)?.is_none()
                && self.validate_policy(&exact_completion_policy).is_ok()
            {
                &exact_completion_policy
            } else {
                strict
            };
            // Each step keeps an exact bracket, but only a scalar endpoint can
            // be replaced. When the retained algebraic endpoint itself is the
            // incidence, every midpoint keeps the scalar side's sign and the
            // bracket closes on that endpoint without ever replacing it. A
            // strict opposite-side certificate excludes that case, so a stall
            // means the side evidence and this incidence disagree: report it
            // instead of refining without end.
            const MAX_SCALAR_BRACKET_STEPS: usize = 128;
            let mut scalar_bracket_steps = 0;
            let parameter = loop {
                if let (Some(lower), Some(upper)) = (&lower_real, &upper_real) {
                    let parameter = match BezierRecursiveProjectiveParameter2::new_monotone(
                        Arc::clone(&system),
                        self.clone(),
                        parallel.clone(),
                        lower.clone(),
                        upper.clone(),
                        lower_sign,
                        upper_sign,
                        policy,
                    )? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    break CurveParameter2::from_recursive_projective(parameter);
                }
                scalar_bracket_steps += 1;
                if scalar_bracket_steps > MAX_SCALAR_BRACKET_STEPS {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                let midpoint = match CurveParameterRange2::new_validated(
                    lower_parameter.clone(),
                    upper_parameter.clone(),
                )
                .strict_interior_scalar(strict)?
                {
                    Classification::Decided(midpoint) => midpoint,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let midpoint_sign =
                    match system.incidence_sign_at_real(&midpoint, midpoint_sign_policy)? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let midpoint_parameter =
                    CurveParameter2::from(BezierParameter2::Exact(midpoint.clone()));
                match midpoint_sign {
                    RealSign::Zero => break midpoint_parameter,
                    sign if sign == lower_sign => {
                        lower_parameter = midpoint_parameter;
                        lower_real = Some(midpoint);
                    }
                    sign if sign == upper_sign => {
                        upper_parameter = midpoint_parameter;
                        upper_real = Some(midpoint);
                    }
                    _ => {
                        return Err(CurveError::Topology(
                            "a monotone chord/parallel sample left its endpoint sign partition"
                                .into(),
                        ));
                    }
                }
            };
            Ok(Classification::Decided((frame, weight_sign, parameter)))
        })?;
        let (frame, weight_sign, mut parameter) = match prepared {
            Classification::Decided(prepared) => prepared,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let point_at = |parameter: &CurveParameter2| {
            BezierAnalyticParallelPoint2::new_with_region_parameter_and_frame_tangent(
                parallel.clone(),
                parameter,
                frame.clone(),
                Real::zero(),
                policy,
            )
            .map(CurvePoint2::from)
            .ok_or_else(|| {
                CurveError::Topology("a retained monotone contact lost its source parameter".into())
            })
        };
        let mut point = point_at(&parameter)?;
        // Consult construction-owned endpoint order before asking this
        // analytic root for Cartesian bounds. A correlated pair endpoint can
        // separate itself from the root by one retained miter-anchor order;
        // refining the root first would materialize its recursive field tower.
        // A generic endpoint-order terminal may only guess that an unresolved
        // coordinate is equal.  This kernel retains the incidence equation
        // needed for exact finite clipping below, so exhaust endpoint identity
        // with terminal approximation suppressed before consuming that
        // correlated authority.
        let exact = policy
            .strict_predicate_pass(|| self.parameter_at_certified_point(point.clone(), policy))?;
        let mut chord_parameter = match exact {
            Classification::Decided(Some(parameter)) => Some(parameter),
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(_) => None,
        };

        if chord_parameter.is_none() {
            // Strictly separated outward boxes are the next-smallest
            // finite-domain authority. Endpoint overlap falls through to the
            // selected-coordinate elimination below.
            match self
                .parameter_at_certified_support_point_by_local_evidence(point.clone(), policy)?
            {
                Classification::Decided(Some(parameter)) => {
                    chord_parameter = Some(parameter);
                }
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(_) => {}
            }
        }

        if chord_parameter.is_none() {
            // Endpoint boxes overlap this root, so construct the exact
            // selected-axis expressions on demand.  At the known incidence
            // root `A + B sqrt(V) = 0`, the sign of a coordinate difference
            // `D + E sqrt(V)` is the sign of `(D B - E A) / B`; this removes
            // the shared radical without forming the global field norm.
            let clipping_system = match self.recursive_projective_parallel_system_with_frame(
                parallel,
                frame.as_deref(),
                true,
                policy,
            )? {
                Classification::Decided(Some(system)) => system,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let coordinate_differences = clipping_system
                .coordinate_differences
                .as_ref()
                .expect("the clipping system retains endpoint coordinates");
            let eliminated_coordinate_numerators = (|| {
                let eliminate = |difference: &BezierRecursiveQuadraticParallelExpression2| {
                    let rational_radical = recursive_quadratic_polynomial_multiply(
                        &difference.rational,
                        &clipping_system.incidence.radical,
                    )?;
                    let radical_rational = recursive_quadratic_polynomial_multiply(
                        &difference.radical,
                        &clipping_system.incidence.rational,
                    )?;
                    recursive_quadratic_polynomial_combine(
                        &rational_radical,
                        &radical_rational,
                        true,
                    )
                };
                Some([
                    eliminate(&coordinate_differences[0])?,
                    eliminate(&coordinate_differences[1])?,
                ])
            })();
            let mut finite_location = None;
            for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64] {
                if refinement_steps != 0 {
                    parameter =
                        match parameter.refined_for_finite_envelope(refinement_steps, policy)? {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(_) => break,
                        };
                }
                let Some((lower, upper)) = parameter.finite_envelope_bounds() else {
                    break;
                };
                let target = RealInterval {
                    lower: lower.clone(),
                    upper: upper.clone(),
                };
                let polynomial_sign = |polynomial: &[BezierRecursiveQuadraticValue2]| {
                    [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512]
                        .into_iter()
                        .find_map(|source_steps| {
                            let coefficient_bits =
                                source_steps.saturating_add(64).min(i32::MAX as usize) as i32;
                            recursive_quadratic_polynomial_interval(
                                polynomial,
                                &target,
                                source_steps,
                                -coefficient_bits,
                            )
                            .as_ref()
                            .and_then(dense_strict_interval_sign)
                        })
                };
                let mut coordinate_signs = [None; 2];
                if let (Some(numerators), Some(radical_sign)) = (
                    eliminated_coordinate_numerators.as_ref(),
                    polynomial_sign(&clipping_system.incidence.radical)
                        .filter(|sign| *sign != RealSign::Zero),
                ) {
                    for (sign, numerator) in coordinate_signs.iter_mut().zip(numerators) {
                        *sign = polynomial_sign(numerator).map(|numerator_sign| {
                            product_sign(product_sign(numerator_sign, radical_sign), weight_sign)
                        });
                    }
                }
                for (sign, difference) in coordinate_signs.iter_mut().zip(coordinate_differences) {
                    if sign.is_some() {
                        continue;
                    }
                    for source_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
                        let coefficient_bits =
                            source_steps.saturating_add(64).min(i32::MAX as usize) as i32;
                        *sign = recursive_quadratic_parallel_expression_interval(
                            difference,
                            false,
                            &target,
                            source_steps,
                            -coefficient_bits,
                        )
                        .as_ref()
                        .and_then(dense_strict_interval_sign)
                        .map(|sign| product_sign(sign, weight_sign));
                        if sign.is_some() {
                            break;
                        }
                    }
                }
                if !self.data.parameter_axis.coordinate_increases {
                    coordinate_signs = coordinate_signs
                        .map(|sign| sign.map(|sign| product_sign(sign, RealSign::Negative)));
                }
                finite_location = match coordinate_signs {
                    [Some(RealSign::Negative), _] | [_, Some(RealSign::Positive)] => Some(false),
                    [Some(RealSign::Positive), Some(RealSign::Negative)] => Some(true),
                    _ => None,
                };
                if finite_location.is_some() {
                    break;
                }
            }
            point = point_at(&parameter)?;
            chord_parameter = match finite_location {
                Some(true) => Some(self.parameter_at_certified_interior_point(point.clone())),
                Some(false) => return Ok(Classification::Decided(None)),
                None => match self.parameter_at_certified_point(point.clone(), policy)? {
                    Classification::Decided(Some(parameter)) => Some(parameter),
                    Classification::Decided(None) => return Ok(Classification::Decided(None)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
        }
        let chord_parameter = chord_parameter.expect("the finite root was classified above");
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-parallel-kernel",
            "retained-monotone-root",
        );
        Ok(Classification::Decided(Some(
            BezierAlgebraicChordRetainedParallelContact2 {
                chord_parameter,
                parallel_parameter: parameter,
                point,
                tangent_cross_sign,
            },
        )))
    }

    /// Builds the same procedural affine support from the oldest collinear
    /// source chord before importing its endpoints into a recursive field.
    ///
    /// Boolean clipping can replace one source endpoint with a correlated
    /// contact while retaining the authored line in `source`.  Applying the
    /// common unit-normal displacement to any two points on that line yields
    /// the same displaced support, so support-only incidence should use the
    /// smallest exact endpoint fields.  Finite clipping deliberately keeps
    /// using the descendant endpoints through the general frame below.
    pub(super) fn compact_recursive_projective_parallel_support_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParallelRecursiveFrame2>>> {
        let Some(structural) = chord_parallel_support_source(self, policy)? else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("compact procedural support: no structural source");
            }
            return Ok(Classification::Decided(None));
        };
        if structural.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("compact procedural support: non-normal displacement");
            }
            return Ok(Classification::Decided(None));
        }
        let (base, source_reversed) = structural.source.smallest_retained_support();
        let Some(self_reversed) = self.shared_tangent_orientation(&structural.source) else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("compact procedural support: unknown source orientation");
            }
            return Ok(Classification::Decided(None));
        };
        let distance = if source_reversed {
            -structural.distance
        } else {
            structural.distance
        };
        let (point, _) = BezierAlgebraicChordParallelPoint2::new_pair(
            base.clone(),
            distance,
            structural.translation_x,
            structural.translation_y,
            policy,
        );
        let mut frame = match point.recursive_projective_frame(policy)? {
            Classification::Decided(Some(frame)) => frame,
            Classification::Decided(None) => {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                    eprintln!("compact procedural support: frame unsupported");
                }
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                    eprintln!("compact procedural support: frame uncertain {reason:?}");
                }
                return Ok(Classification::Uncertain(reason));
            }
        };
        if self_reversed ^ source_reversed {
            frame.displaced.swap(0, 1);
            frame.direction_endpoints.swap(0, 1);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-projective-chord-parallel-frame",
            "canonical-procedural-support",
        );
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!(
                "compact procedural support: retained source-reversed={source_reversed} self-reversed={self_reversed}"
            );
        }
        Ok(Classification::Decided(Some(frame)))
    }

    #[track_caller]
    pub(super) fn recursive_projective_parallel_system_with_frame(
        &self,
        parallel: &BezierParallel2,
        frame_tangent: Option<&BezierAnalyticParallelTangentField2>,
        build_coordinate_differences: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveProjectiveChordParallelSystem2>>> {
        let compact_frame = if !build_coordinate_differences {
            match self.compact_recursive_projective_parallel_support_frame(policy)? {
                Classification::Decided(frame) => frame,
                // This is a representation fast path.  A field-capacity or
                // predicate miss must not reduce the complete generic path.
                Classification::Uncertain(_) => None,
            }
        } else {
            None
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            && compact_frame.is_none()
        {
            let caller = std::panic::Location::caller();
            eprintln!(
                "recursive parallel system generic caller={}:{} coordinates={build_coordinate_differences}",
                caller.file(),
                caller.line(),
            );
        }
        let frame = match compact_frame {
            Some(frame) => frame,
            None => match self.recursive_projective_endpoints_with_direction(policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        let [start, end] = frame.displaced;
        let [direction_start, direction_end] = frame.direction_endpoints;
        let field = start.denominator.field();
        let source = parallel.source_power_basis()?;
        let differential = parallel.differential()?;
        let (tangent_x_coefficients, tangent_y_coefficients) = frame_tangent
            .map(|frame| (&frame.x[..], &frame.y[..]))
            .unwrap_or((&differential.tangent_x, &differential.tangent_y));
        // The formal source equation is independent of a consumed parameter
        // domain. Whole-domain consumers certify their actual range or ray;
        // point predicates replay weight and speed at their retained parameter.
        let unit = [Real::one()];
        let weight_coefficients = source.weight.unwrap_or(&unit);
        let Some((incidence, source_weight, tangent_cross, tangent_dot, coordinate_differences)) =
            (|| {
                let real = |coefficients: &[Real]| {
                    recursive_quadratic_real_polynomial(&field, coefficients)
                };
                let add = |first: &[BezierRecursiveQuadraticValue2],
                           second: &[BezierRecursiveQuadraticValue2]| {
                    recursive_quadratic_polynomial_combine(first, second, false)
                };
                let subtract =
                    |first: &[BezierRecursiveQuadraticValue2],
                     second: &[BezierRecursiveQuadraticValue2]| {
                        recursive_quadratic_polynomial_combine(first, second, true)
                    };
                let multiply =
                    |first: &[BezierRecursiveQuadraticValue2],
                     second: &[BezierRecursiveQuadraticValue2]| {
                        recursive_quadratic_polynomial_multiply(first, second)
                    };
                let scale = |polynomial: &[BezierRecursiveQuadraticValue2],
                             value: &BezierRecursiveQuadraticValue2| {
                    recursive_quadratic_polynomial_scale(polynomial, value)
                };
                let source_x = real(source.x_numerator)?;
                let source_y = real(source.y_numerator)?;
                let source_weight = real(weight_coefficients)?;
                let tangent_x = real(tangent_x_coefficients)?;
                let tangent_y = real(tangent_y_coefficients)?;
                let (direction_x, direction_y, _) =
                    direction_end.difference_numerators(&direction_start)?;
                let point_delta_x = subtract(
                    &scale(&source_x, &start.denominator)?,
                    &scale(&source_weight, &start.x)?,
                )?;
                let point_delta_y = subtract(
                    &scale(&source_y, &start.denominator)?,
                    &scale(&source_weight, &start.y)?,
                )?;
                let source_incidence = subtract(
                    &scale(&point_delta_y, &direction_x)?,
                    &scale(&point_delta_x, &direction_y)?,
                )?;
                let tangent_dot = add(
                    &scale(&tangent_x, &direction_x)?,
                    &scale(&tangent_y, &direction_y)?,
                )?;
                let normal_incidence = recursive_quadratic_polynomial_scale_real(
                    &scale(&multiply(&tangent_dot, &source_weight)?, &start.denominator)?,
                    parallel.distance(),
                )?;
                let zero_distance = parallel.distance().zero_status() == ZeroKnowledge::Zero;
                let speed_squared: Arc<[_]> = if zero_distance {
                    real(&[Real::one()])?
                } else {
                    add(
                        &multiply(&tangent_x, &tangent_x)?,
                        &multiply(&tangent_y, &tangent_y)?,
                    )?
                }
                .into();
                let tangent_cross = subtract(
                    &scale(&tangent_y, &direction_x)?,
                    &scale(&tangent_x, &direction_y)?,
                )?;
                let (source_coordinate, start_coordinate, end_coordinate, normal_coordinate) =
                    match self.data.parameter_axis.axis {
                        Axis2::X => (
                            &source_x,
                            &start.x,
                            &end.x,
                            recursive_quadratic_polynomial_scale_real(
                                &tangent_y,
                                &Real::from(-1_i8),
                            )?,
                        ),
                        Axis2::Y => (&source_y, &start.y, &end.y, tangent_x.clone()),
                    };
                let coordinate_difference =
                    |endpoint_coordinate: &BezierRecursiveQuadraticValue2,
                     endpoint_denominator: &BezierRecursiveQuadraticValue2| {
                        let radical = subtract(
                            &scale(source_coordinate, endpoint_denominator)?,
                            &scale(&source_weight, endpoint_coordinate)?,
                        )?;
                        let rational = recursive_quadratic_polynomial_scale_real(
                            &scale(
                                &multiply(&normal_coordinate, &source_weight)?,
                                endpoint_denominator,
                            )?,
                            parallel.distance(),
                        )?;
                        Some(if zero_distance {
                            BezierRecursiveQuadraticParallelExpression2::new(
                                radical,
                                real(&[Real::zero()])?,
                                speed_squared.clone(),
                            )
                        } else {
                            BezierRecursiveQuadraticParallelExpression2::new(
                                rational,
                                radical,
                                speed_squared.clone(),
                            )
                        })
                    };
                let coordinate_differences = build_coordinate_differences
                    .then(|| {
                        Some([
                            coordinate_difference(start_coordinate, &start.denominator)?,
                            coordinate_difference(end_coordinate, &end.denominator)?,
                        ])
                    })
                    .flatten();
                Some((
                    if zero_distance {
                        BezierRecursiveQuadraticParallelExpression2::new(
                            source_incidence,
                            real(&[Real::zero()])?,
                            speed_squared.clone(),
                        )
                    } else {
                        BezierRecursiveQuadraticParallelExpression2::new(
                            normal_incidence,
                            source_incidence,
                            speed_squared.clone(),
                        )
                    },
                    source_weight,
                    tangent_cross,
                    tangent_dot,
                    coordinate_differences,
                ))
            })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let system = BezierRecursiveProjectiveChordParallelSystem2 {
            base: field.base_and_extension_path().0,
            field,
            projection: OnceLock::new(),
            incidence,
            source_weight,
            tangent_cross,
            tangent_dot,
            coordinate_differences,
        };
        Ok(Classification::Decided(Some(system)))
    }

    /// Replays one target domain against the authored positive target-speed
    /// sheet of the recursive projective chord equation. The squared norm is
    /// candidate enumeration only: every retained candidate must satisfy the
    /// unsquared incidence strictly before it may become topology evidence.
    pub(super) fn recursive_projective_parallel_intersections_in_domain(
        &self,
        parallel: &BezierParallel2,
        system: &BezierRecursiveProjectiveChordParallelSystem2,
        frame_tangent: Option<&Arc<BezierAnalyticParallelTangentField2>>,
        derivative_scale_sign: Option<RealSign>,
        domain: SelectedThirdAxisDomain2<'_>,
        component_sample: Option<&Real>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        if let Classification::Uncertain(reason) = parallel.certify_source_frame_in_domain(
            domain,
            frame_tangent.map(Arc::as_ref),
            policy,
        )? {
            return Ok(Classification::Uncertain(reason));
        }
        let endpoint_roots = [self.start(), self.end()].map(|point| {
            let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point else {
                return None;
            };
            (point.data.parallel == *parallel
                && point.data.frame_tangent.as_ref() == frame_tangent
                && policy.accepts_retained_policy(point.data.policy)
                && [
                    &point.data.tangent_distance,
                    &point.data.translation_x,
                    &point.data.translation_y,
                ]
                .into_iter()
                .all(|value| value.zero_status() == ZeroKnowledge::Zero))
            .then(|| point.data.parameter.curve_parameter())
        });
        if let Some(candidates) = policy
            .strict_predicate_pass(|| system.local_parameters(domain, endpoint_roots, policy))?
            && let Classification::Decided(contacts) = policy.strict_predicate_pass(|| {
                self.recursive_projective_parallel_contacts(
                    parallel,
                    system,
                    candidates,
                    BezierRecursiveParallelCandidateEvidence2::SelectedNorm,
                    frame_tangent,
                    derivative_scale_sign,
                    clip_to_finite_chord,
                    policy,
                )
            })?
        {
            return Ok(Classification::Decided(contacts));
        }
        self.recursive_projective_parallel_intersections_from_projection(
            parallel,
            system,
            frame_tangent,
            derivative_scale_sign,
            domain,
            component_sample,
            clip_to_finite_chord,
            policy,
        )
    }

    pub(super) fn recursive_projective_parallel_intersections_from_projection(
        &self,
        parallel: &BezierParallel2,
        system: &BezierRecursiveProjectiveChordParallelSystem2,
        frame_tangent: Option<&Arc<BezierAnalyticParallelTangentField2>>,
        derivative_scale_sign: Option<RealSign>,
        domain: SelectedThirdAxisDomain2<'_>,
        component_sample: Option<&Real>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        let (candidates, original_projection_is_discrete) = match system
            .parameters(domain, policy)?
        {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                (candidates, true)
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                let sample = match component_sample.cloned().map(Classification::Decided) {
                    Some(sample) => sample,
                    None => domain.strict_sample(policy)?,
                };
                let sample = match sample {
                    Classification::Decided(sample) => sample,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match system.norm_component_sheet_at_real(&sample, policy)? {
                    Classification::Decided(Some(true)) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "recursive-chord-parallel-degenerate",
                            "coincident-support",
                        );
                        return Ok(Classification::Decided(
                            BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent { sample },
                        ));
                    }
                    Classification::Decided(Some(false)) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "recursive-chord-parallel-degenerate",
                            "opposite-speed-sheet",
                        );
                    }
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(
                            BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }

                // A target-wide squared norm can belong wholly to the
                // opposite positive-speed sheet. Authored-sheet contacts are
                // then exactly the common zeros of the two unsquared terms.
                // Enumerate either nonzero term and replay the complete
                // expression below; no parallel-specific component solver is
                // introduced.
                let mut residual = None;
                for coefficients in [&system.incidence.radical, &system.incidence.rational] {
                    let Some(projection) = system.projected_polynomial(coefficients) else {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    };
                    match system.projected_parameters(&projection, domain, policy)? {
                        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                            candidates,
                        )) => {
                            residual = Some(candidates);
                            break;
                        }
                        Classification::Decided(
                            BezierAlgebraicFiberProjection2::IdenticallyZero,
                        ) => {}
                        Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                            return Ok(Classification::Decided(
                                BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let Some(candidates) = residual else {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                    ));
                };
                (candidates, false)
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.recursive_projective_parallel_contacts(
            parallel,
            system,
            candidates.into_iter().map(CurveParameter2::from).collect(),
            if original_projection_is_discrete {
                BezierRecursiveParallelCandidateEvidence2::Projected(
                    system
                        .projection
                        .get()
                        .expect("the global chord/parallel enumerator retains its projection"),
                )
            } else {
                BezierRecursiveParallelCandidateEvidence2::Replay
            },
            frame_tangent,
            derivative_scale_sign,
            clip_to_finite_chord,
            policy,
        )
    }

    pub(super) fn recursive_projective_parallel_contacts(
        &self,
        parallel: &BezierParallel2,
        system: &BezierRecursiveProjectiveChordParallelSystem2,
        candidates: Vec<CurveParameter2>,
        evidence: BezierRecursiveParallelCandidateEvidence2<'_>,
        frame_tangent: Option<&Arc<BezierAnalyticParallelTangentField2>>,
        derivative_scale_sign: Option<RealSign>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        let projection = match evidence {
            BezierRecursiveParallelCandidateEvidence2::Projected(projection) => Some(projection),
            _ => None,
        };
        let mut contacts = Vec::with_capacity(candidates.len());
        for mut candidate in candidates {
            // A local incidence root can rediscover a scalar already owned
            // by a chord endpoint. Reuse that authority before branch and
            // tangent replay rebuild a high-degree remainder over the joined
            // endpoint field. Only exact scalar equality permits reuse;
            // overlapping isolators or point equality alone do not.
            if candidate.as_recursive_projective().is_some() {
                for point in [self.start(), self.end()] {
                    let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point else {
                        continue;
                    };
                    if point.data.parallel != *parallel
                        || !policy.accepts_retained_policy(point.data.policy)
                    {
                        continue;
                    }
                    let retained = point.data.parameter.curve_parameter();
                    if policy
                        .bounded_exact_predicate_pass(|| candidate.same_value(&retained, policy))?
                        == Classification::Decided(true)
                    {
                        candidate = retained;
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-parallel-kernel",
                            "owned-endpoint-parameter",
                        );
                        break;
                    }
                }
            }
            let projected_incidence = projection.zip(candidate.as_bezier_parameter()).and_then(
                |(projection, parameter)| {
                    projected_selected_dense_candidate_box_incidence(
                        projection,
                        &system.base.sources,
                        parameter,
                        64,
                        64,
                    )
                },
            );
            let projected_certificate = match projected_incidence {
                Some(BezierDenseCandidateBoxIncidence2::Root(certificate)) => Some(certificate),
                Some(BezierDenseCandidateBoxIncidence2::Disjoint(_)) => continue,
                None => None,
            };
            let evaluation = if let Some(parameter) = candidate.as_bezier_parameter() {
                match system.candidate_evaluation(parameter, policy)? {
                    Classification::Decided(Some(evaluation)) => Some(evaluation),
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                None
            };
            let polynomial_sign = |polynomial: &[BezierRecursiveQuadraticValue2]| {
                if let Some(evaluation) = &evaluation {
                    system.polynomial_sign(polynomial, evaluation, policy)
                } else {
                    recursive_projective_polynomial_sign_at_parameter(
                        &system.field,
                        polynomial,
                        &candidate,
                        policy,
                    )
                }
            };
            let weight_sign = match policy
                .strict_predicate_pass(|| polynomial_sign(&system.source_weight))?
            {
                Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let expression_sign = |expression: &BezierRecursiveQuadraticParallelExpression2| {
                if let Some(evaluation) = &evaluation {
                    system.expression_sign(expression, evaluation, policy)
                } else {
                    expression.sign_with_positive_speed(policy, polynomial_sign)
                }
            };
            // This is the terminal authored-sheet predicate, not persistent
            // object construction. The selected policy must therefore remain
            // able to consume APPROXIMATE_512 after every exact interval and
            // projected-zero certificate has declined.
            let replay = if matches!(
                evidence,
                BezierRecursiveParallelCandidateEvidence2::SelectedNorm
            ) {
                // Deflation changes the defining polynomial, not this norm
                // certificate. Reuse it without reconstructing a remainder or
                // refining intervals around an already-proven exact zero.
                if system
                    .incidence
                    .radical
                    .iter()
                    .all(BezierRecursiveQuadraticValue2::is_structurally_zero)
                {
                    Classification::Decided(RealSign::Zero)
                } else {
                    positive_root_sum_sign_from_components(
                        polynomial_sign(&system.incidence.rational)?,
                        polynomial_sign(&system.incidence.radical)?,
                        Classification::Decided(RealSign::Zero),
                    )
                }
            } else if let (Some(certificate), Some(evaluation)) =
                (&projected_certificate, &evaluation)
            {
                system.certified_expression_replay_sign(
                    &system.incidence,
                    evaluation,
                    certificate,
                    policy,
                )?
            } else {
                expression_sign(&system.incidence)?
            };
            match replay {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Negative | RealSign::Positive) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let contact_parameter = projected_certificate
                .as_ref()
                .and_then(|certificate| certificate.candidate.exact_point_witness())
                .cloned()
                .map(CurveParameter2::from)
                .unwrap_or_else(|| candidate.clone());
            let Some(point) =
                BezierAnalyticParallelPoint2::new_with_region_parameter_and_frame_tangent(
                    parallel.clone(),
                    &contact_parameter,
                    frame_tangent.cloned(),
                    Real::zero(),
                    policy,
                )
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let point = CurvePoint2::from(point);
            let chord_parameter = if clip_to_finite_chord {
                let mut coordinate_signs = [RealSign::Zero; 2];
                for (sign, difference) in coordinate_signs.iter_mut().zip(
                    system
                        .coordinate_differences
                        .as_ref()
                        .expect("the all-roots system retains finite-chord coordinates")
                        .iter(),
                ) {
                    *sign = match policy.strict_predicate_pass(|| expression_sign(difference))? {
                        Classification::Decided(sign) => product_sign(sign, weight_sign),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                }
                if !self.data.parameter_axis.coordinate_increases {
                    coordinate_signs =
                        coordinate_signs.map(|sign| product_sign(sign, RealSign::Negative));
                }
                match coordinate_signs {
                    [RealSign::Zero, RealSign::Zero] => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    [RealSign::Zero, _] => self.start_parameter(),
                    [_, RealSign::Zero] => self.end_parameter(),
                    [RealSign::Positive, RealSign::Negative] => {
                        self.parameter_at_certified_interior_point(point.clone())
                    }
                    [RealSign::Negative, _] | [_, RealSign::Positive] => continue,
                }
            } else {
                self.parameter_at_certified_support_point(point.clone(), policy)?
            };
            let source_cross =
                match policy.strict_predicate_pass(|| polynomial_sign(&system.tangent_cross))? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let source_dot =
                match policy.strict_predicate_pass(|| polynomial_sign(&system.tangent_dot))? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let derivative_scale = match if let Some(sign) = derivative_scale_sign {
                Classification::Decided(sign)
            } else {
                parallel.parallel_derivative_scale_sign(&contact_parameter, policy)?
            } {
                Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicChordParallelContact2 {
                point: chord_parameter.point().clone(),
                chord_parameter,
                parallel_parameter: contact_parameter,
                tangent_cross_sign: product_sign(source_cross, derivative_scale),
                tangent_dot_sign: product_sign(source_dot, derivative_scale),
            });
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-parallel-kernel",
            "recursive-projective",
        );
        Ok(Classification::Decided(
            BezierAlgebraicChordParallelIntersections2::Contacts(contacts),
        ))
    }

    /// Two distinct points on a strictly convex closed curve exhaust its
    /// intersections with their secant. Retain those endpoint authorities
    /// instead of rediscovering their roots over a joined coefficient field.
    /// Unit seam aliases remain separate parameter contacts; finite clipping
    /// consumes the caller's original range, including reversed ranges.
    pub(super) fn closed_parallel_endpoint_contacts(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Vec<BezierAlgebraicChordParallelContact2>>> {
        policy.strict_predicate_pass(|| {
            let [
                CurvePoint2(CurvePointData2::AnalyticParallel(start)),
                CurvePoint2(CurvePointData2::AnalyticParallel(end)),
            ] = [self.start(), self.end()]
            else {
                return Ok(None);
            };
            let unit = CurveParameterRange2::unit();
            let unit_domain = CurveParameterDomain2::new(&unit, None);
            let endpoints = [start, end];
            for point in endpoints {
                if point.data.parallel != *parallel
                    || point.data.frame_tangent.is_some()
                    || !policy.accepts_retained_policy(point.data.policy)
                    || [
                        &point.data.tangent_distance,
                        &point.data.translation_x,
                        &point.data.translation_y,
                    ]
                    .into_iter()
                    .any(|value| value.zero_status() != ZeroKnowledge::Zero)
                    || unit_domain.contains_finite_parameter(
                        &point.data.parameter.curve_parameter(),
                        policy,
                    )? != Classification::Decided(true)
                {
                    return Ok(None);
                }
            }
            if unit_domain.contains_finite_range(range, policy)? != Classification::Decided(true)
                || !parallel.certifies_simple_closed_parallel(policy)?
            {
                return Ok(None);
            }
            let differential = parallel.differential()?;
            let constant =
                |polynomial: &[Real]| polynomial.first().cloned().unwrap_or_else(Real::zero);
            let turn = Real::diff_of_products(
                &constant(&differential.tangent_x),
                &constant(&differential.tangent_derivative_y),
                &constant(&differential.tangent_y),
                &constant(&differential.tangent_derivative_x),
            );
            let Some(turn @ (RealSign::Positive | RealSign::Negative)) = real_sign(&turn, policy)
            else {
                return Ok(None);
            };
            let mut contacts: Vec<BezierAlgebraicChordParallelContact2> = Vec::new();
            for (index, point) in endpoints.into_iter().enumerate() {
                let parameter = point.data.parameter.curve_parameter();
                // Convexity has already proved the complete contact set.
                // Finish its tangent evidence in the retained endpoint fields;
                // a speculative field-join budget must not force fresh root
                // discovery merely to recover these same owned contacts.
                let Classification::Decided(dot) = self
                    .tangent_cross_dot_parallel_linear_combination_sign(
                        parallel,
                        &parameter,
                        &Real::zero(),
                        &Real::one(),
                        policy,
                    )?
                else {
                    return Ok(None);
                };
                let mut parameters = vec![parameter.clone()];
                for (seam, alias) in [(Real::zero(), Real::one()), (Real::one(), Real::zero())] {
                    match parameter.same_value(&seam.into(), policy)? {
                        Classification::Decided(true) => parameters.push(alias.into()),
                        Classification::Decided(false) => {}
                        Classification::Uncertain(_) => return Ok(None),
                    }
                }
                let chord_parameter = if index == 0 {
                    self.start_parameter()
                } else {
                    self.end_parameter()
                };
                for parameter in parameters {
                    match CurveParameterDomain2::new(range, None)
                        .contains_finite_parameter(&parameter, policy)?
                    {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => continue,
                        Classification::Uncertain(_) => return Ok(None),
                    }
                    let mut position = contacts.len();
                    for (index, contact) in contacts.iter().enumerate() {
                        match parameter.cmp_by_refinement(&contact.parallel_parameter, policy)? {
                            Classification::Decided(std::cmp::Ordering::Less) => {
                                position = index;
                                break;
                            }
                            Classification::Decided(std::cmp::Ordering::Greater) => {}
                            Classification::Decided(std::cmp::Ordering::Equal)
                            | Classification::Uncertain(_) => return Ok(None),
                        }
                    }
                    contacts.insert(
                        position,
                        BezierAlgebraicChordParallelContact2 {
                            point: chord_parameter.point().clone(),
                            chord_parameter: chord_parameter.clone(),
                            parallel_parameter: parameter,
                            tangent_cross_sign: if index == 0 {
                                product_sign(turn, RealSign::Negative)
                            } else {
                                turn
                            },
                            tangent_dot_sign: dot,
                        },
                    );
                }
            }
            Ok(Some(contacts))
        })
    }

    pub(super) fn recursive_projective_parallel_intersections_with_frame(
        &self,
        parallel: &BezierParallel2,
        frame_tangent: Option<Arc<BezierAnalyticParallelTangentField2>>,
        derivative_scale_sign: Option<RealSign>,
        domain: SelectedThirdAxisDomain2<'_>,
        component_sample: Option<&Real>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParallelIntersections2>>> {
        if clip_to_finite_chord
            && frame_tangent.is_none()
            && let SelectedThirdAxisDomain2::Finite(range) = domain
            && let Some(contacts) =
                self.closed_parallel_endpoint_contacts(parallel, range, policy)?
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-parallel-kernel",
                "closed-convex-endpoints",
            );
            return Ok(Classification::Decided(Some(
                BezierAlgebraicChordParallelIntersections2::Contacts(contacts),
            )));
        }
        if clip_to_finite_chord {
            // Enumerate the infinite-support contacts before importing the two
            // finite descendant endpoints. Procedural chords can then use their
            // oldest exact support frame; only actual contacts need a finite
            // axis-order test. If a contact cannot be clipped from local boxes,
            // the complete joined endpoint system below remains authoritative.
            let support_system = match self.recursive_projective_parallel_system_with_frame(
                parallel,
                frame_tangent.as_deref(),
                false,
                policy,
            )? {
                Classification::Decided(Some(system)) => Some(system),
                Classification::Decided(None) | Classification::Uncertain(_) => None,
            };
            if let Some(support_system) = support_system
                && let Classification::Decided(intersections) = self
                    .recursive_projective_parallel_intersections_in_domain(
                        parallel,
                        &support_system,
                        frame_tangent.as_ref(),
                        derivative_scale_sign,
                        domain,
                        component_sample,
                        false,
                        policy,
                    )?
            {
                match intersections {
                    BezierAlgebraicChordParallelIntersections2::Contacts(contacts) => {
                        let mut clipped = Vec::with_capacity(contacts.len());
                        let mut complete = true;
                        for mut contact in contacts {
                            match policy.bounded_exact_predicate_pass(|| {
                                self.parameter_at_certified_support_point_by_local_evidence(
                                    contact.point.clone(),
                                    policy,
                                )
                            })? {
                                Classification::Decided(Some(parameter)) => {
                                    contact.point = parameter.point().clone();
                                    contact.chord_parameter = parameter;
                                    clipped.push(contact);
                                }
                                Classification::Decided(None) => {}
                                Classification::Uncertain(_) => {
                                    complete = false;
                                    break;
                                }
                            }
                        }
                        if complete {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-parallel-kernel",
                                "support-first-finite-clipping",
                            );
                            return Ok(Classification::Decided(Some(
                                BezierAlgebraicChordParallelIntersections2::Contacts(clipped),
                            )));
                        }
                    }
                    intersection => {
                        return Ok(Classification::Decided(Some(intersection)));
                    }
                }
            }
        }
        let system = match self.recursive_projective_parallel_system_with_frame(
            parallel,
            frame_tangent.as_deref(),
            true,
            policy,
        )? {
            Classification::Decided(Some(system)) => system,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(self
            .recursive_projective_parallel_intersections_in_domain(
                parallel,
                &system,
                frame_tangent.as_ref(),
                derivative_scale_sign,
                domain,
                component_sample,
                clip_to_finite_chord,
                policy,
            )?
            .map(Some))
    }

    /// Replays every finite contact between this retained chord and a
    /// genuinely analytic parallel.
    ///
    /// Both authored endpoints enter their least shared recursive projective
    /// tower. Complete local root isolation keeps simple contacts in that
    /// coefficient field; unresolved cases demand the global norm projection.
    /// Both enumerators replay the authored positive-speed sheet before
    /// finite chord containment and tangent orientation are published. No
    /// approximate value selects a carrier representation.
    #[track_caller]
    pub(crate) fn parallel_intersections(
        &self,
        parallel: &BezierParallel2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        self.validate_policy(policy)?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let caller = std::panic::Location::caller();
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            eprintln!(
                "chord parallel intersections caller={}:{} endpoints=({},{}) support=({},{})",
                caller.file(),
                caller.line(),
                kind(self.start()),
                kind(self.end()),
                kind(self.retained_support().start()),
                kind(self.retained_support().end()),
            );
        }
        Ok(
            match self.recursive_projective_parallel_intersections_with_frame(
                parallel,
                None,
                None,
                SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit()),
                None,
                true,
                policy,
            )? {
                Classification::Decided(Some(intersections)) => {
                    Classification::Decided(intersections)
                }
                Classification::Decided(None) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Runs the authoritative chord/parallel incidence on one regular side
    /// of a source cusp.  The exact common hodograph factor is cancelled only
    /// for the selected unit-normal frame; source coordinates and the shared
    /// algebraic chord solver are otherwise unchanged.
    pub(crate) fn parallel_intersections_on_regular_range(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        self.validate_policy(policy)?;
        let component_sample =
            match policy.strict_predicate_pass(|| range.strict_interior_scalar(policy))? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let frame = match policy.strict_predicate_pass(|| {
            parallel
                .source_oriented_regularized_tangent_field_at_interior(&component_sample, policy)
        })? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let derivative_scale_sign = if frame.is_some() {
            match parallel.parallel_derivative_scale_sign_at_exact(&component_sample, policy)? {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                    Some(sign)
                }
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            parallel.certified_derivative_scale_sign_on_range(range, policy)?
        };
        Ok(
            match self.recursive_projective_parallel_intersections_with_frame(
                parallel,
                frame,
                derivative_scale_sign,
                SelectedThirdAxisDomain2::Finite(range),
                Some(&component_sample),
                true,
                policy,
            )? {
                Classification::Decided(Some(intersections)) => {
                    Classification::Decided(intersections)
                }
                Classification::Decided(None) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Replays this finite chord and its complete affine support against the
    /// authored analytic span plus one regular incident projective ray.
    ///
    /// This is the `TrimOrExtend` domain for a chord/parallel corner. The
    /// recursive incidence system is built once; both the authored target
    /// span and its exterior ray replay against the chord's affine support.
    /// Only final-axis isolation differs. The ray stops before its first source
    /// pole or tangent-speed zero.
    pub(crate) fn parallel_intersections_with_incident_ray(
        &self,
        parallel: &BezierParallel2,
        incident: &BezierParallelIncidentDomain2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        self.validate_policy(policy)?;
        let system = match self
            .recursive_projective_parallel_system_with_frame(parallel, None, true, policy)?
        {
            Classification::Decided(Some(system)) => system,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let finite = match self.recursive_projective_parallel_intersections_in_domain(
            parallel,
            &system,
            None,
            None,
            SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit()),
            None,
            false,
            policy,
        )? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let exterior = match self.recursive_projective_parallel_intersections_in_domain(
            parallel,
            &system,
            None,
            None,
            SelectedThirdAxisDomain2::IncidentRay {
                anchor: incident.anchor(),
                direction: incident.direction(),
                barrier: incident.barrier(),
            },
            None,
            false,
            policy,
        )? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(match (finite, exterior) {
            (
                BezierAlgebraicChordParallelIntersections2::Contacts(mut finite),
                BezierAlgebraicChordParallelIntersections2::Contacts(exterior),
            ) => {
                finite.extend(exterior);
                BezierAlgebraicChordParallelIntersections2::Contacts(finite)
            }
            (
                component
                @ BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                    ..
                },
                _,
            )
            | (
                _,
                component
                @ BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                    ..
                },
            ) => component,
            _ => BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
        }))
    }

    /// Replays every finite contact between this retained chord and an
    /// arbitrary rational Bezier without adjoining its finite-boundary fields.
    ///
    /// The stable support's first endpoint field is eliminated into a
    /// bivariate polynomial by Hypersolve, its second endpoint field is
    /// eliminated into source-parameter candidates, and every candidate is
    /// replayed against the selected root triple before the current finite
    /// boundary admits it as topology evidence. One optional source parameter
    /// may be omitted when authored adjacency already owns it.
    pub(super) fn exact_line_retained_circle_intersections(
        &self,
        line: &LineSeg2,
        source: &RationalBezier2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<BezierAlgebraicChordRationalIntersections2>>> {
        if source.retained_circular_conic().is_none() {
            return Ok(None);
        }
        let arc = match crate::arc_bezier::rational_bezier_circular_arc(source, policy)? {
            Classification::Decided(Some(arc)) => arc,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        // A provenance line may be only a short directed witness for the
        // infinite support.  Classify that support first and let the retained
        // parameterization and this finite chord perform the two authoritative domain
        // filters; `intersect_arc` would incorrectly clip to the witness.
        let hits = match line.supporting_line_circle_relation(&arc, policy)? {
            LineCircleRelation::Disjoint => Vec::new(),
            LineCircleRelation::Tangent { point, .. } => vec![point],
            LineCircleRelation::Secant {
                first_point,
                second_point,
                ..
            } => vec![first_point, second_point],
            LineCircleRelation::Uncertain { reason } => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let (chord_dx, chord_dy) = line.delta();
        let mut contacts = Vec::with_capacity(hits.len());
        for hit in hits {
            // The inverse retained-circle parameter map is itself the exact
            // finite-span admission certificate.  Reclassifying the same hit
            // through reconstructed Cartesian sweep sides can needlessly
            // exhaust the predicate budget for algebraic line/circle roots.
            let source_parameters = match policy
                .strict_predicate_pass(|| source.retained_circle_point_parameters(&hit, policy))?
            {
                Classification::Decided(parameters) => parameters,
                Classification::Uncertain(_) => {
                    // The retained-circle inverse is an accelerator.  A hit
                    // at the quadratic chart's omitted projective point can
                    // make that inverse undecidable.  The retained arc sweep
                    // can still reject that exact circle point before the
                    // ordinary Bernstein line-contact fallback is needed.
                    if arc.contains_sweep_point(&hit, policy) == Classification::Decided(false) {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-retained-circle",
                            "omitted-projective-point-outside-sweep",
                        );
                        continue;
                    }
                    return Ok(None);
                }
            };
            if source_parameters.is_empty() {
                continue;
            }
            let point = CurvePoint2::from(hit.clone());
            let chord_parameter = match self.parameter_at_certified_point(point.clone(), policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let (radial_x, radial_y) = hit.delta_from(arc.center());
            let (source_dx, source_dy) = if arc.is_clockwise() {
                (radial_y, -radial_x)
            } else {
                (-radial_y, radial_x)
            };
            let tangent_cross =
                Real::diff_of_products(&chord_dx, &source_dy, &chord_dy, &source_dx);
            let Some(tangent_cross_sign) = real_sign(&tangent_cross, policy) else {
                return Ok(Some(Classification::Uncertain(UncertaintyReason::RealSign)));
            };
            for source_parameter in source_parameters {
                let source_parameter = CurveParameter2::from(source_parameter);
                if let Some(excluded) = excluded_source_parameter {
                    match source_parameter.cmp_by_refinement(excluded, policy)? {
                        Classification::Decided(std::cmp::Ordering::Equal) => continue,
                        Classification::Decided(_) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    }
                }
                contacts.push(BezierAlgebraicChordRationalContact2 {
                    chord_parameter: chord_parameter.clone(),
                    other_parameter: source_parameter,
                    point: point.clone(),
                    tangent_cross_sign,
                });
            }
        }
        Ok(Some(Classification::Decided(
            BezierAlgebraicChordRationalIntersections2::Contacts(contacts),
        )))
    }

    /// Reuses the selected circle/chord map's own recursive frame when this
    /// support was authored directly between that circle's center and the
    /// retained contact.  The structural center match is only a sufficient
    /// provenance certificate: unrelated but geometrically equal points fall
    /// through to the complete projective importer below.
    pub(super) fn recursive_cusp_contact_frame_endpoints(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<[BezierRecursiveQuadraticProjectivePoint2; 2]>> {
        for (contact_endpoint, center_endpoint, contact_at_start) in [
            (self.start(), self.end(), true),
            (self.end(), self.start(), false),
        ] {
            let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = contact_endpoint else {
                continue;
            };
            let (map, contact) = point.map_contact();
            let center = match map.data.semicircle.center_point_evidence(policy)? {
                Classification::Decided(center) => center,
                Classification::Uncertain(_) => continue,
            };
            let same_center = if center_endpoint == &center {
                true
            } else if let (
                CurvePoint2(CurvePointData2::Algebraic(first)),
                CurvePoint2(CurvePointData2::Algebraic(second)),
            ) = (center_endpoint, &center)
            {
                match policy
                    .strict_predicate_pass(|| first.same_retained_rational_point(second, policy))?
                {
                    Some(Classification::Decided(equal)) => equal,
                    Some(Classification::Uncertain(_)) | None => matches!(
                        policy
                            .strict_predicate_pass(|| center_endpoint.same_point(&center, policy)),
                        Classification::Decided(true)
                    ),
                }
            } else {
                false
            };
            if !same_center {
                continue;
            }
            let frame = match map.recursive_contact_frame(contact, policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) | Classification::Uncertain(_) => continue,
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-chord-endpoints",
                "cusp-contact-center-frame",
            );
            return Ok(Some(if contact_at_start {
                [frame.point, frame.center]
            } else {
                [frame.center, frame.point]
            }));
        }
        Ok(None)
    }

    /// Imports both finite endpoints together with the least-field direction
    /// witnesses for their affine support.  A procedural displacement shares
    /// one normalized translation between its endpoints, so subtracting the
    /// displaced points would only introduce and then cancel that radical.
    /// Retaining the source direction keeps support incidence small while the
    /// displaced points remain authoritative for finite-domain clipping.
    #[track_caller]
    pub(super) fn recursive_projective_endpoints_with_direction(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParallelRecursiveFrame2>>> {
        if let (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
        ) = (self.start(), self.end())
            && start.shares_carrier(end)
            && start.at_end != end.at_end
            && start.data.source_point.is_none()
        {
            let frame = match start.recursive_projective_frame(policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let indices = [usize::from(start.at_end), usize::from(end.at_end)];
            return Ok(Classification::Decided(Some(
                BezierAlgebraicChordParallelRecursiveFrame2 {
                    displaced: indices.map(|index| frame.displaced[index].clone()),
                    direction_endpoints: indices
                        .map(|index| frame.direction_endpoints[index].clone()),
                },
            )));
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            && let (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) = (self.start(), self.end())
        {
            let caller = std::panic::Location::caller();
            eprintln!(
                "recursive parallel frame fallback caller={}:{} carrier={} normal-carrier={} ends=({},{}) origins=({},{}) sources={} directions=({:?},{:?}) distances={:?} translations={} policies={}",
                caller.file(),
                caller.line(),
                start.shares_carrier(end),
                start.shares_normal_offset_carrier(end),
                start.at_end,
                end.at_end,
                start.data.source_point.is_some(),
                end.data.source_point.is_some(),
                start.data.source == end.data.source,
                start.data.direction,
                end.data.direction,
                compare_reals(
                    &start.data.distance,
                    &end.data.distance,
                    &CurveContext::STRICT
                ),
                start.data.translation_x == end.data.translation_x
                    && start.data.translation_y == end.data.translation_y,
                start.data.policy == end.data.policy,
            );
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            let start_direction = start.source_direction_endpoints(policy);
            let end_direction = end.source_direction_endpoints(policy);
            eprintln!(
                "recursive parallel frame fallback origins-kind=({},{}) origins-storage={} origins-equal={} direction-kinds=(({},{}),({},{})) source-axes=({:?},{:?})",
                kind(start.source_endpoint()),
                kind(end.source_endpoint()),
                start
                    .source_endpoint()
                    .shares_storage(end.source_endpoint()),
                start.source_endpoint() == end.source_endpoint(),
                kind(start_direction[0]),
                kind(start_direction[1]),
                kind(end_direction[0]),
                kind(end_direction[1]),
                start.data.source.data.parameter_axis,
                end.data.source.data.parameter_axis,
            );
            if start
                .source_endpoint()
                .shares_storage(end.source_endpoint())
                && matches!(
                    start.source_endpoint(),
                    CurvePoint2(CurvePointData2::Exact(_))
                )
            {
                let status =
                    |point: &CurvePoint2| match recursive_projective_point_source(point, policy) {
                        Ok(Classification::Decided(Some(_))) => "decided",
                        Ok(Classification::Decided(None)) => "none",
                        Ok(Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                            "unsupported"
                        }
                        Ok(Classification::Uncertain(_)) => "uncertain",
                        Err(_) => "error",
                    };
                eprintln!(
                    "recursive parallel frame direction-status=(({},{}),({},{}))",
                    status(start_direction[0]),
                    status(start_direction[1]),
                    status(end_direction[0]),
                    status(end_direction[1]),
                );
            }
        }
        Ok(self.recursive_projective_endpoints(policy)?.map(|points| {
            points.map(|displaced| BezierAlgebraicChordParallelRecursiveFrame2 {
                direction_endpoints: displaced.clone(),
                displaced,
            })
        }))
    }

    /// Imports both authored endpoints into one positively normalized
    /// recursive projective frame. Rational and analytic-parallel targets
    /// share this exact construction boundary; target-specific kernels add
    /// only their own parameter axis and procedural radicals afterward.
    #[track_caller]
    pub(super) fn recursive_projective_endpoints(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<[BezierRecursiveQuadraticProjectivePoint2; 2]>>> {
        let ([start, end], [start_sign, end_sign]) = if let Some(points) =
            self.recursive_cusp_contact_frame_endpoints(policy)?
        {
            (points, [RealSign::Positive; 2])
        } else {
            let evidence = [self.start(), self.end()];
            let mut denominator_signs = Vec::with_capacity(2);
            for point in evidence {
                match recursive_projective_evidence_denominator_sign(point, policy)? {
                    Classification::Decided(sign) => denominator_signs.push(sign),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                let caller = std::panic::Location::caller();
                let kind = |point: &CurvePoint2| match point {
                    CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                    CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                    CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                    CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                    CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                        "similarity"
                    }
                };
                eprintln!(
                    "recursive chord endpoints caller={}:{} kinds=({},{}) selects-approximate={} permits-approximate={}",
                    caller.file(),
                    caller.line(),
                    kind(evidence[0]),
                    kind(evidence[1]),
                    policy.selects_approximate_512(),
                    policy.permits_approximate_512(),
                );
            }
            let recursive = if policy.selects_approximate_512() {
                // Exact coordinate-field selection must not consume the
                // terminal policy.  This also lets the bounded recursive
                // importer decline a divergent primitive compositum.
                policy.strict_predicate_pass(|| {
                    recursive_projective_evidence_points(&evidence, policy)
                })
            } else {
                recursive_projective_evidence_points(&evidence, policy)
            }?;
            let recursive = if policy.permits_approximate_512()
                && !matches!(&recursive, Classification::Decided(Some(_)))
            {
                // The endpoint tuple is still exact: import its selected
                // coordinate witnesses into a flat dense base.  Run this
                // bridge with approximation suppressed so no terminal
                // equality can select a persistent representation.
                policy.strict_predicate_pass(|| {
                    represented_projective_evidence_points(&evidence, policy)
                })?
            } else {
                recursive
            };
            let points = match recursive {
                Classification::Decided(Some(points)) => points,
                Classification::Decided(None) => {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("recursive chord endpoints stage=evidence-unavailable");
                    }
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("recursive chord endpoints stage=evidence reason={reason:?}");
                    }
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (
                points
                    .try_into()
                    .expect("a recursive chord support retains two authored endpoints"),
                denominator_signs
                    .try_into()
                    .expect("a recursive chord support retains two denominator signs"),
            )
        };
        let start = orient_recursive_projective_point_positive(start, start_sign)?;
        let end = orient_recursive_projective_point_positive(end, end_sign)?;
        if !start
            .denominator
            .field()
            .same_field(&end.denominator.field())
        {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("recursive chord endpoints stage=field-mismatch");
            }
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Ok(Classification::Decided(Some([start, end])))
    }

    pub(super) fn recursive_projective_rational_system(
        &self,
        source: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveProjectiveChordRationalSystem2>>> {
        // Boolean clipping can replace one finite endpoint with a correlated
        // contact while preserving the exact authored supporting line in the
        // chord's source chain. Incidence depends only on that line, so import
        // its oldest (and therefore smallest-field) endpoints first. The
        // current endpoints remain authoritative for finite clipping through
        // `parameter_at_certified_point`; they need not share an eager
        // primitive field merely to reject the usual interior candidate.
        let (support, support_reversed) = self.smallest_incidence_support();
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "chord-pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp-chord",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "cusp-derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "chord-parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic-parallel",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            eprintln!(
                "recursive chord/rational support finite=[{},{}] support=[{},{}] has-source={}",
                kind(self.start()),
                kind(self.end()),
                kind(support.start()),
                kind(support.end()),
                self.data.source.is_some(),
            );
        }
        let retained_tangent_line = if Arc::ptr_eq(&self.data, &support.data) {
            None
        } else {
            match (support.start(), support.end()) {
                (
                    CurvePoint2(CurvePointData2::AnalyticParallel(start)),
                    CurvePoint2(CurvePointData2::AnalyticParallel(end)),
                ) => match start.recursive_tangent_line_to(end, policy) {
                    Ok(Classification::Decided(Some(line))) => Some(line),
                    Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => {
                        None
                    }
                },
                _ => None,
            }
        };
        let retained_frame = if Arc::ptr_eq(&self.data, &support.data)
            || retained_tangent_line.is_some()
        {
            None
        } else {
            match support.recursive_projective_endpoints_with_direction(policy) {
                Ok(Classification::Decided(Some(frame))) => Some(frame),
                Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => None,
            }
        };
        let (
            support_line,
            support_anchor,
            direction_endpoints,
            start,
            end,
            _uses_retained_support_incidence,
        ) = if let Some(line) = retained_tangent_line {
            let line = if support_reversed {
                let negative = Real::from(-1_i8);
                BezierRecursiveQuadraticProjectivePoint2 {
                    x: line.x.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a retained analytic support line exceeded its field budget".into(),
                        )
                    })?,
                    y: line.y.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a retained analytic support line exceeded its field budget".into(),
                        )
                    })?,
                    denominator: line.denominator.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a retained analytic support line exceeded its field budget".into(),
                        )
                    })?,
                }
            } else {
                line
            };
            (Some(line), None, None, None, None, true)
        } else if let Some(frame) = retained_frame {
            let support_anchor = frame.displaced[usize::from(support_reversed)].clone();
            let direction_endpoints = if support_reversed {
                [
                    frame.direction_endpoints[1].clone(),
                    frame.direction_endpoints[0].clone(),
                ]
            } else {
                frame.direction_endpoints
            };
            (
                None,
                Some(support_anchor),
                Some(direction_endpoints),
                None,
                None,
                true,
            )
        } else {
            let [start, end] = match self.recursive_projective_endpoints(policy)? {
                Classification::Decided(Some(points)) => points,
                Classification::Decided(None) => {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("recursive chord/rational system stage=endpoints-unavailable");
                    }
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!(
                            "recursive chord/rational system stage=endpoints reason={reason:?}"
                        );
                    }
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (
                None,
                Some(start.clone()),
                Some([start.clone(), end.clone()]),
                Some(start),
                Some(end),
                false,
            )
        };
        if _uses_retained_support_incidence {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "retained-support-incidence",
            );
        }
        let field = support_line
            .as_ref()
            .map(|line| line.denominator.field())
            .or_else(|| {
                support_anchor
                    .as_ref()
                    .map(|anchor| anchor.denominator.field())
            })
            .expect("a recursive chord support retains a line or anchor");
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!(
                "recursive chord/rational system stage=endpoints-complete field-depth={} finite-depth={:?} retained-support-incidence={_uses_retained_support_incidence}",
                field.base_and_extension_path().1.len(),
                start.as_ref().map(|point| point
                    .denominator
                    .field()
                    .base_and_extension_path()
                    .1
                    .len()),
            );
        }
        let source_power = source.homogeneous_power_basis()?;
        let source_weight_sign = match source.denominator_sign(&crate::CurveParameterRange2::unit())
        {
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(_) => None,
        };
        let [tangent_power_x, tangent_power_y] =
            rational_parametric_tangent_numerator(source_power);
        let constant_coordinate = self.constant_axis_coordinate(
            match self.data.parameter_axis.axis {
                Axis2::X => Axis2::Y,
                Axis2::Y => Axis2::X,
            },
            policy,
        )?;
        let exact_support_line = self.strict_provenance_support_line(policy);
        let chord_axis_direction_sign = if self.data.parameter_axis.coordinate_increases {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let chord_cross_perpendicular_factor_sign = match self.data.parameter_axis.axis {
            Axis2::X => chord_axis_direction_sign,
            Axis2::Y => product_sign(chord_axis_direction_sign, RealSign::Negative),
        };
        let exact_line_cross_perpendicular_factor_sign = exact_support_line
            .as_ref()
            .and_then(|line| {
                let (line_x, line_y) = line.delta();
                real_sign(
                    match self.data.parameter_axis.axis {
                        Axis2::X => &line_x,
                        Axis2::Y => &line_y,
                    },
                    &policy.strict_counterpart(),
                )
            })
            .filter(|sign| *sign != RealSign::Zero)
            .map(|sign| match self.data.parameter_axis.axis {
                Axis2::X => sign,
                Axis2::Y => product_sign(sign, RealSign::Negative),
            });
        let uses_direct_axis_incidence =
            exact_support_line.is_none() && constant_coordinate.is_some();
        let affine_preimage_incidence_factor_sign = if exact_support_line.is_some() {
            exact_line_cross_perpendicular_factor_sign
        } else if uses_direct_axis_incidence {
            Some(RealSign::Positive)
        } else {
            Some(chord_cross_perpendicular_factor_sign)
        };
        let tangent_from_incidence_derivative_sign = if uses_direct_axis_incidence {
            Some(chord_cross_perpendicular_factor_sign)
        } else {
            Some(RealSign::Positive)
        };
        let Some((source_x, source_y, source_weight, incidence, tangent_cross)) = (|| {
            let real =
                |coefficients: &[Real]| recursive_quadratic_real_polynomial(&field, coefficients);
            let canonical_real = |value: Real| {
                value
                    .exact_rational_normal_form()
                    .map(Real::new)
                    .unwrap_or(value)
            };
            let exact_line_incidence = |line: &LineSeg2,
                                        x: &[Real],
                                        y: &[Real],
                                        weight: &[Real]| {
                let (line_x, line_y) = line.delta();
                let constant =
                    Real::diff_of_products(&line_y, line.start().x(), &line_x, line.start().y());
                let zero = Real::zero();
                (0..x.len().max(y.len()).max(weight.len()))
                    .map(|index| {
                        canonical_real(Real::signed_product_sum(
                            [true, false, true],
                            [
                                [&line_x, y.get(index).unwrap_or(&zero)],
                                [&line_y, x.get(index).unwrap_or(&zero)],
                                [&constant, weight.get(index).unwrap_or(&zero)],
                            ],
                        ))
                    })
                    .collect::<Vec<_>>()
            };
            let exact_line_tangent_cross =
                |line: &LineSeg2, tangent_x: &[Real], tangent_y: &[Real]| {
                    let (line_x, line_y) = line.delta();
                    let zero = Real::zero();
                    (0..tangent_x.len().max(tangent_y.len()))
                        .map(|index| {
                            canonical_real(Real::diff_of_products(
                                &line_x,
                                tangent_y.get(index).unwrap_or(&zero),
                                &line_y,
                                tangent_x.get(index).unwrap_or(&zero),
                            ))
                        })
                        .collect::<Vec<_>>()
                };
            let subtract = |first: &[BezierRecursiveQuadraticValue2],
                            second: &[BezierRecursiveQuadraticValue2]| {
                recursive_quadratic_polynomial_combine(first, second, true)
            };
            let add = |first: &[BezierRecursiveQuadraticValue2],
                       second: &[BezierRecursiveQuadraticValue2]| {
                recursive_quadratic_polynomial_combine(first, second, false)
            };
            let scale = |polynomial: &[BezierRecursiveQuadraticValue2],
                         value: &BezierRecursiveQuadraticValue2| {
                recursive_quadratic_polynomial_scale(polynomial, value)
            };
            let source_x = real(&source_power.x_numerator)?;
            let source_y = real(&source_power.y_numerator)?;
            let source_weight = real(&source_power.weight)?;
            let tangent_x = real(&tangent_power_x)?;
            let tangent_y = real(&tangent_power_y)?;
            let direction = direction_endpoints
                .as_ref()
                .and_then(|endpoints| endpoints[1].difference_numerators(&endpoints[0]));
            let incidence = if let Some(line) = exact_support_line.as_ref() {
                real(&exact_line_incidence(
                    line,
                    &source_power.x_numerator,
                    &source_power.y_numerator,
                    &source_power.weight,
                ))?
            } else if let Some(line) = support_line.as_ref() {
                add(
                    &add(&scale(&source_x, &line.x)?, &scale(&source_y, &line.y)?)?,
                    &scale(&source_weight, &line.denominator)?,
                )?
            } else {
                let (direction_x, direction_y, _) = direction.as_ref()?;
                let support_start = support_anchor.as_ref()?;
                let point_delta_x = subtract(
                    &scale(&source_x, &support_start.denominator)?,
                    &scale(&source_weight, &support_start.x)?,
                )?;
                let point_delta_y = subtract(
                    &scale(&source_y, &support_start.denominator)?,
                    &scale(&source_weight, &support_start.y)?,
                )?;
                match (self.data.parameter_axis.axis, constant_coordinate.as_ref()) {
                    (Axis2::X, Some(constant_y)) => subtract(
                        &source_y,
                        &recursive_quadratic_polynomial_scale_real(&source_weight, constant_y)?,
                    )?,
                    (Axis2::Y, Some(constant_x)) => subtract(
                        &source_x,
                        &recursive_quadratic_polynomial_scale_real(&source_weight, constant_x)?,
                    )?,
                    _ => subtract(
                        &scale(&point_delta_y, direction_x)?,
                        &scale(&point_delta_x, direction_y)?,
                    )?,
                }
            };
            let tangent_cross = if let Some(line) = exact_support_line.as_ref() {
                real(&exact_line_tangent_cross(
                    line,
                    &tangent_power_x,
                    &tangent_power_y,
                ))?
            } else if let Some(line) = support_line.as_ref() {
                add(&scale(&tangent_x, &line.x)?, &scale(&tangent_y, &line.y)?)?
            } else {
                let (direction_x, direction_y, _) = direction.as_ref()?;
                subtract(
                    &scale(&tangent_y, direction_x)?,
                    &scale(&tangent_x, direction_y)?,
                )?
            };
            Some((source_x, source_y, source_weight, incidence, tangent_cross))
        })() else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("recursive chord/rational system stage=polynomial-assembly");
            }
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(Classification::Decided(Some(
            BezierRecursiveProjectiveChordRationalSystem2 {
                field,
                start,
                end,
                source_x,
                source_y,
                source_weight,
                source_weight_sign,
                incidence,
                tangent_cross,
                affine_preimage_incidence_factor_sign,
                tangent_from_incidence_derivative_sign,
            },
        )))
    }

    pub(super) fn recursive_projective_rational_intersections(
        &self,
        source: &RationalBezier2,
        range: &CurveParameterRange2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordRationalIntersections2>>> {
        macro_rules! recursive_rational_uncertain {
            ($stage:literal, $reason:expr) => {{
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "algebraic chord/rational blocker stage={} reason={:?} selects-approximate={} permits-approximate={}",
                        $stage,
                        $reason,
                        policy.selects_approximate_512(),
                        policy.permits_approximate_512(),
                    );
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-rational-blocker",
                    $stage,
                );
                return Ok(Classification::Uncertain($reason));
            }};
        }
        let system = match self.recursive_projective_rational_system(source, policy)? {
            Classification::Decided(Some(system)) => system,
            Classification::Decided(None) => {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "algebraic chord/rational blocker stage=system-unavailable selects-approximate={} permits-approximate={}",
                        policy.selects_approximate_512(),
                        policy.permits_approximate_512(),
                    );
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-rational-blocker",
                    "system-unavailable",
                );
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                recursive_rational_uncertain!("system", reason);
            }
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!(
                "algebraic chord/rational stage=system-complete incidence-degree={} field-depth={}",
                system.incidence.len().saturating_sub(1),
                system.field.base_and_extension_path().1.len(),
            );
        }
        let unit_domain = range == &CurveParameterRange2::unit();
        let strict_unit_crossing = unit_domain
            .then(|| {
                recursive_quadratic_polynomial_strict_unit_crossing(
                    &system.field,
                    &system.incidence,
                )
            })
            .flatten();
        let mut certified_endpoint_roots = [false; 2];
        let (strict_unit_crossing, geometric_no_roots) = if !unit_domain
            || strict_unit_crossing.is_some()
        {
            (strict_unit_crossing, false)
        } else {
            let geometric_crossing = policy.bounded_exact_predicate_pass(|| -> CurveResult<
                (Option<BezierRecursiveQuadraticUnitCrossing2>, bool),
            > {
                let (predicate_chord, predicate_reversed) = self.smallest_incidence_support();
                let predicate = match BezierAlgebraicChordSupportPredicate2::try_new(
                    predicate_chord,
                    policy,
                )? {
                    Classification::Decided(predicate) => predicate,
                    Classification::Uncertain(_reason) => {
                        #[cfg(test)]
                        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                            eprintln!(
                                "algebraic chord/rational geometric crossing stage=predicate reason={_reason:?}"
                            );
                        }
                        return Ok((None, false));
                    }
                };
                let point_side = |point: Point2| -> CurveResult<Option<RealSign>> {
                    let point = CurvePoint2::from(point);
                    Ok(match predicate.oriented_side(&point, policy)? {
                        Classification::Decided(crate::classify::LineSide::Left) => {
                            Some(RealSign::Positive)
                        }
                        Classification::Decided(crate::classify::LineSide::Right) => {
                            Some(RealSign::Negative)
                        }
                        Classification::Decided(crate::classify::LineSide::On) => {
                            Some(RealSign::Zero)
                        }
                        Classification::Uncertain(_) => None,
                    })
                };
                let start_side = point_side(source.start().clone())?;
                let end_side = point_side(source.end().clone())?;
                // A finite endpoint on the supporting line is an incidence
                // root regardless of the other endpoint's sign or the signs
                // of the homogeneous controls. Mixed-weight major conics
                // retain this proof even when a convex-hull test is invalid.
                certified_endpoint_roots = [
                    start_side == Some(RealSign::Zero),
                    end_side == Some(RealSign::Zero),
                ];
                let (Some(weight_sign), Some(side_to_incidence_sign)) = (
                    system.source_weight_sign,
                    system.tangent_from_incidence_derivative_sign,
                ) else {
                    return Ok((None, false));
                };
                let incidence_factor = product_sign(
                    product_sign(weight_sign, side_to_incidence_sign),
                    if predicate_reversed {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    },
                );
                let point_sign = |point: Point2| -> CurveResult<Option<RealSign>> {
                    Ok(point_side(point)?.map(|side| product_sign(incidence_factor, side)))
                };
                let start_sign = start_side.map(|side| product_sign(incidence_factor, side));
                let end_sign = end_side.map(|side| product_sign(incidence_factor, side));
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "algebraic chord/rational geometric crossing stage=endpoints start={start_sign:?} end={end_sign:?} weight={weight_sign:?} side-factor={side_to_incidence_sign:?}"
                    );
                }
                let (Some(start_sign), Some(end_sign)) = (start_sign, end_sign) else {
                    return Ok((None, false));
                };
                // Opposite endpoint signs prove one unit root only when the
                // incidence has degree at most two. Higher-degree curves can
                // cross three or more times and need complete root isolation.
                if system.incidence.len() <= 3
                    && strict_signs_are_opposite(Some(start_sign), Some(end_sign))
                {
                    let mut lower = Real::zero();
                    let mut upper = Real::one();
                    let mut lower_sign = start_sign;
                    for _ in 0..12 {
                        let midpoint = ((&lower + &upper) / Real::from(2_i8))?;
                        let Ok(point) = source.point_at(&midpoint, policy) else {
                            break;
                        };
                        let Some(midpoint_sign) = point_sign(point)? else {
                            break;
                        };
                        match midpoint_sign {
                            RealSign::Zero => {
                                lower = midpoint.clone();
                                upper = midpoint;
                                break;
                            }
                            sign if sign == lower_sign => {
                                lower = midpoint;
                                lower_sign = sign;
                            }
                            _ => upper = midpoint,
                        }
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-rational-crossing",
                        "geometric-endpoint-sides",
                    );
                    return Ok((
                        Some(BezierRecursiveQuadraticUnitCrossing2 {
                            start_sign,
                            end_sign,
                            leading_sign: None,
                            lower,
                            upper,
                        }),
                        false,
                    ));
                }
                let uniform_control_sign = match (start_sign, end_sign) {
                    (RealSign::Positive, RealSign::Positive) => RealSign::Positive,
                    (RealSign::Negative, RealSign::Negative) => RealSign::Negative,
                    _ => return Ok((None, false)),
                };
                let mut control_hull_is_disjoint = true;
                let Some(controls) = source.affine_control_points() else { return Ok((None, false)); };
                let interior_control_count = controls.len().saturating_sub(2);
                for (_index, control) in controls.iter()
                    .enumerate()
                    .skip(1)
                    .take(interior_control_count)
                {
                    let sign = point_sign(control.clone())?;
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!(
                            "algebraic chord/rational geometric crossing stage=control index={_index} sign={sign:?}"
                        );
                    }
                    match sign {
                        Some(sign) if sign == uniform_control_sign || sign == RealSign::Zero => {}
                        _ => {
                            control_hull_is_disjoint = false;
                            break;
                        }
                    }
                }
                // Every interior Bernstein basis function is nonnegative on the
                // closed unit interval, while the endpoint basis functions are
                // strictly positive in its interior. Strict same-sign endpoint
                // coefficients and same-sign-or-zero interior coefficients
                // therefore certify that the incidence has no unit root.
                if control_hull_is_disjoint {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-rational-crossing",
                        "geometric-control-hull-disjoint",
                    );
                    return Ok((None, true));
                }
                Ok((None, false))
            })?;
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=geometric-crossing result={} ",
                    geometric_crossing.0.is_some(),
                );
            }
            geometric_crossing
        };
        if geometric_no_roots {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "geometric-control-hull-disjoint",
            );
            return Ok(Classification::Decided(Some(
                BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
            )));
        }
        // Opposite certified endpoint signs are already an exact witness that
        // this incidence is not the zero polynomial. Reuse that witness rather
        // than signing every coefficient in a potentially deep retained field.
        let identically_zero = if strict_unit_crossing.is_some() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-collinearity",
                "strict-unit-crossing-nonzero",
            );
            Classification::Decided(false)
        } else {
            recursive_quadratic_polynomial_is_identically_zero(&system.incidence, policy)?
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!(
                "algebraic chord/rational stage=collinearity-complete result={identically_zero:?}"
            );
        }
        if matches!(identically_zero, Classification::Decided(true)) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "collinear-before-projection",
            );
            return Ok(self
                .collinear_rational_intersections(source, range, excluded_source_parameter, policy)?
                .map(Some));
        }
        let (candidates, strict_unit_crossing) = match system.parameters(
            range,
            strict_unit_crossing,
            certified_endpoint_roots,
            excluded_source_parameter,
            policy,
        )? {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                recursive_rational_uncertain!("parameter-isolation", reason);
            }
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!(
                "algebraic chord/rational stage=parameters-complete candidates={} strict-unit-crossing={}",
                candidates.len(),
                strict_unit_crossing.is_some(),
            );
        }
        let mut contacts = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("algebraic chord/rational stage=candidate-begin");
            }
            if let Some(excluded) = excluded_source_parameter {
                match candidate.cmp_by_refinement(excluded, policy)? {
                    Classification::Decided(std::cmp::Ordering::Equal) => continue,
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        recursive_rational_uncertain!("excluded-parameter-order", reason);
                    }
                }
            }
            let parameter_order = |endpoint: &BezierRecursiveQuadraticProjectivePoint2| ->
             CurveResult<Option<std::cmp::Ordering>> {
                if let Some(crossing) = strict_unit_crossing.as_ref()
                    && let Some(order) = system.affine_parameter_axis_order(
                        endpoint,
                        self.data.parameter_axis.axis,
                        crossing,
                    )
                {
                    return Ok(Some(order));
                }
                if unit_domain && let Some(order) = system.parameter_hull_axis_order(
                    source,
                    &candidate,
                    endpoint,
                    self.data.parameter_axis.axis,
                )? {
                    return Ok(Some(order));
                }
                Ok(None)
            };
            let start_parameter_order = match system.start.as_ref() {
                Some(start) => parameter_order(start)?,
                None => None,
            };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=start-parameter-order order={start_parameter_order:?}"
                );
            }
            let end_parameter_order = match system.end.as_ref() {
                Some(end) => parameter_order(end)?,
                None => None,
            };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=end-parameter-order order={end_parameter_order:?}"
                );
            }
            let chord_location = match (start_parameter_order, end_parameter_order) {
                (Some(start_order), Some(end_order)) => {
                    let (start_order, end_order) = if self.data.parameter_axis.coordinate_increases
                    {
                        (start_order, end_order)
                    } else {
                        (start_order.reverse(), end_order.reverse())
                    };
                    match (start_order, end_order) {
                        (std::cmp::Ordering::Equal, _) => {
                            BezierRecursiveChordContactLocation2::Start
                        }
                        (_, std::cmp::Ordering::Equal) => BezierRecursiveChordContactLocation2::End,
                        (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => {
                            BezierRecursiveChordContactLocation2::Interior
                        }
                        _ => continue,
                    }
                }
                _ => {
                    // The native parameter hull is the hot finite-domain
                    // authority.  Only an overlapping endpoint falls back to
                    // the chord's canonical retained-point classifier; this
                    // avoids adjoining the selected root merely to create a
                    // transient Cartesian target point.
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("algebraic chord/rational stage=finite-point-begin");
                    }
                    let point = match rational_point_evidence_at_region_parameter(
                        source, &candidate, policy,
                    )? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            recursive_rational_uncertain!("finite-point-evidence", reason);
                        }
                    };
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("algebraic chord/rational stage=finite-parameter-begin");
                    }
                    // Strict outward axis separation is the cheapest complete
                    // finite-domain certificate. Overlapping boxes next
                    // consume construction-owned endpoint identity in a
                    // bounded pass; only both declines reach the complete
                    // retained-point comparison.
                    let bounds = policy.bounded_exact_predicate_pass(|| {
                        self.parameter_at_certified_support_point_by_local_evidence(
                            point.clone(),
                            policy,
                        )
                    })?;
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!(
                            "algebraic chord/rational stage=finite-parameter-bounds result={bounds:?}"
                        );
                    }
                    let parameter = match bounds {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => {
                            let bounded = policy.bounded_exact_predicate_pass(|| {
                                self.parameter_at_certified_point(point.clone(), policy)
                            })?;
                            match bounded {
                                Classification::Decided(parameter) => parameter,
                                Classification::Uncertain(_) => {
                                    match self.parameter_at_certified_point(point, policy)? {
                                        Classification::Decided(parameter) => parameter,
                                        Classification::Uncertain(reason) => {
                                            recursive_rational_uncertain!(
                                                "finite-chord-parameter",
                                                reason
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    };
                    let Some(parameter) = parameter else {
                        continue;
                    };
                    match parameter.data {
                        BezierAlgebraicChordParameterStorage2::Endpoint {
                            at_end: false, ..
                        } => BezierRecursiveChordContactLocation2::Start,
                        BezierAlgebraicChordParameterStorage2::Endpoint {
                            at_end: true, ..
                        } => BezierRecursiveChordContactLocation2::End,
                        BezierAlgebraicChordParameterStorage2::Interior(_) => {
                            BezierRecursiveChordContactLocation2::Interior
                        }
                    }
                }
            };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=location-complete location={chord_location:?}"
                );
            }
            let tangent_cross_sign =
                if let (Some(crossing), Some(weight_sign), Some(incidence_factor_sign)) = (
                    strict_unit_crossing.as_ref(),
                    system.source_weight_sign,
                    system.tangent_from_incidence_derivative_sign,
                ) {
                    // At a simple line-incidence root, differentiating the
                    // homogeneous incidence gives the rational tangent cross
                    // multiplied only by the target weight and positive chord
                    // projective factors. A unique strict unit crossing has
                    // derivative sign equal to its right-end sign.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-rational-tangent",
                        "unit-crossing-derivative",
                    );
                    product_sign(
                        product_sign(crossing.end_sign, weight_sign),
                        incidence_factor_sign,
                    )
                } else {
                    match policy.strict_predicate_pass(|| {
                        recursive_projective_polynomial_sign_at_parameter(
                            &system.field,
                            &system.tangent_cross,
                            &candidate,
                            policy,
                        )
                    })? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            recursive_rational_uncertain!("tangent-cross", reason);
                        }
                    }
                };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=tangent-complete sign={tangent_cross_sign:?}"
                );
            }
            let other_parameter = candidate.with_chord_rational_tangent_identity(
                self.clone(),
                source.clone(),
                tangent_cross_sign,
                chord_location,
            );
            let point_evidence = match rational_point_evidence_at_region_parameter(
                source,
                &other_parameter,
                policy,
            )? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    recursive_rational_uncertain!("point-evidence", reason);
                }
            };
            let chord_parameter = match chord_location {
                BezierRecursiveChordContactLocation2::Start => self.start_parameter(),
                BezierRecursiveChordContactLocation2::End => self.end_parameter(),
                BezierRecursiveChordContactLocation2::Interior => {
                    self.parameter_at_certified_interior_point(point_evidence.clone())
                }
            };
            contacts.push(BezierAlgebraicChordRationalContact2 {
                chord_parameter,
                other_parameter,
                point: point_evidence,
                tangent_cross_sign,
            });
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("algebraic chord/rational stage=candidate-complete");
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-rational-kernel",
            "recursive-projective",
        );
        Ok(Classification::Decided(Some(
            BezierAlgebraicChordRationalIntersections2::Contacts(contacts),
        )))
    }

    /// Discharges a caller-owned endpoint contact on a pole-free conic span.
    /// The line incidence has a numerator of degree at most two. Its strict
    /// sign immediately inside the owned endpoint and at the opposite endpoint
    /// agrees only if there is no second root. A further simple root would
    /// reverse that sign; a further even root would require degree at least
    /// three. A double root at the owned endpoint itself is permitted.
    ///
    /// The caller must first certify that the whole finite range is pole-free.
    /// This sufficient proof uses local point bounds and the existing tangent
    /// certificate; any failed optional construction leaves full isolation to
    /// the caller.
    pub(super) fn rational_endpoint_contact_is_complete(
        &self,
        source: &RationalBezier2,
        range: &CurveParameterRange2,
        owned: &CurveParameter2,
        policy: &CurveContext,
    ) -> bool {
        policy
            .bounded_exact_predicate_pass(|| -> CurveResult<bool> {
                let power = source.homogeneous_power_basis()?;
                if [&power.x_numerator, &power.y_numerator, &power.weight]
                    .into_iter()
                    .any(|coefficients| {
                        coefficients
                            .iter()
                            .skip(3)
                            .any(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
                    })
                {
                    return Ok(false);
                }
                let other = if owned.cmp_by_refinement(range.start(), policy)?
                    == Classification::Decided(std::cmp::Ordering::Equal)
                {
                    range.end()
                } else if owned.cmp_by_refinement(range.end(), policy)?
                    == Classification::Decided(std::cmp::Ordering::Equal)
                {
                    range.start()
                } else {
                    return Ok(false);
                };
                // Retain the refined authorities themselves: a bounded order
                // query intentionally declines a stored isolator that still
                // touches the other endpoint. Reusing the tighter envelopes
                // also certifies the subsequent interior sample's order.
                let (Classification::Decided(owned), Classification::Decided(other)) = (
                    owned.refined_for_finite_envelope(4, policy)?,
                    other.refined_for_finite_envelope(4, policy)?,
                ) else {
                    return Ok(false);
                };
                let direction = match other.cmp_by_refinement(&owned, policy)? {
                    Classification::Decided(std::cmp::Ordering::Less) => RealSign::Negative,
                    Classification::Decided(std::cmp::Ordering::Greater) => RealSign::Positive,
                    _ => return Ok(false),
                };
                let Classification::Decided(point) =
                    rational_point_evidence_at_region_parameter(source, &other, policy)?
                else {
                    return Ok(false);
                };
                let side = match self
                    .strict_oriented_side_by_local_interval_refinement(&point, policy)?
                {
                    Classification::Decided(crate::classify::LineSide::Left) => RealSign::Positive,
                    Classification::Decided(crate::classify::LineSide::Right) => RealSign::Negative,
                    _ => return Ok(false),
                };
                let parallel = source.parallel_left(Real::zero())?;
                let refined_range = CurveParameterRange2::new_validated(other, owned.clone());
                let Classification::Decided(interior) =
                    refined_range.strict_interior_scalar(policy)?
                else {
                    return Ok(false);
                };
                let terminal = CurveParameterRange2::new_validated(
                    BezierParameter2::Exact(interior).into(),
                    owned,
                );
                let Classification::Decided(tangent) =
                    self.parallel_tangent_cross_sign_on_region_range(&parallel, &terminal, policy)?
                else {
                    return Ok(false);
                };
                Ok(tangent != RealSign::Zero && side == product_sign(tangent, direction))
            })
            .unwrap_or(false)
    }

    /// Discovers exact incidence evidence covering the finite source range.
    /// Unit-domain fast paths may retain wider certified components; consumers
    /// clip their contacts and correspondences to the active operand domains.
    /// An excluded parameter names an already-certified contact owned by the caller.
    pub(crate) fn rational_intersections(
        &self,
        source: &RationalBezier2,
        range: &CurveParameterRange2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalIntersections2>> {
        self.validate_policy(policy)?;
        let unit = CurveParameterRange2::unit();
        let unit_covers_range = CurveParameterDomain2::new(&unit, None)
            .contains_finite_range(range, &policy.strict_counterpart())?;
        let finite_unit_source = unit_covers_range == Classification::Decided(true)
            && matches!(
                source.denominator_sign(&crate::CurveParameterRange2::unit()),
                Classification::Decided(RealSign::Positive | RealSign::Negative)
            );
        if !finite_unit_source {
            match polynomial_is_nonzero_on_parameter_range(
                &source.homogeneous_power_basis()?.weight,
                range,
                &policy.strict_counterpart(),
            )? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        if let Some(owned) = excluded_source_parameter
            && self.rational_endpoint_contact_is_complete(source, range, owned, policy)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "conic-owned-endpoint-complete",
            );
            return Ok(Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
            ));
        }
        if !finite_unit_source {
            return Ok(
                match self.recursive_projective_rational_intersections(
                    source,
                    range,
                    excluded_source_parameter,
                    policy,
                )? {
                    Classification::Decided(Some(intersections)) => {
                        Classification::Decided(intersections)
                    }
                    Classification::Decided(None) => {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
        if excluded_source_parameter.is_none()
            && let Some(intersections) = self.exact_linear_rational_intersections(source, policy)?
        {
            return Ok(Classification::Decided(intersections));
        }
        // A represented parameter can still contain arbitrary exact values.
        // Reuse its certified incidence as a polynomial factor before circle
        // reconstruction asks a freshly solved point to prove endpoint equality.
        if excluded_source_parameter
            .and_then(CurveParameter2::scalar)
            .is_some()
            && let Classification::Decided(Some(intersections)) =
                policy.strict_predicate_pass(|| {
                    self.recursive_projective_rational_intersections(
                        source,
                        range,
                        excluded_source_parameter,
                        policy,
                    )
                })?
        {
            return Ok(Classification::Decided(intersections));
        }
        // Diagonal fiber deflation needs a univariate algebraic root. Other
        // retained locations still participate in general contact ownership.
        let selected =
            match excluded_source_parameter.and_then(CurveParameter2::as_bezier_parameter) {
                Some(BezierParameter2::Algebraic(parameter)) => Some(parameter.clone()),
                None if excluded_source_parameter.is_none() => {
                    match self.algebraic_endpoint_parameter(policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => None,
                    }
                }
                Some(BezierParameter2::Exact(_)) | None => None,
            };
        if let Some(parameter) = selected {
            match self.source_related_intersections(
                source,
                &parameter,
                excluded_source_parameter.is_none(),
                policy,
            )? {
                Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::NotSourceRelated
                    | BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                )
                | Classification::Uncertain(UncertaintyReason::Unsupported) => {}
                intersections => return Ok(intersections),
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record("hypercurve", "algebraic-chord-pair", "general-rational");
        if let Some(line) = self
            .exact_line()
            .or_else(|| self.strict_provenance_support_line(policy))
        {
            if let Some(intersections) = self.exact_line_retained_circle_intersections(
                &line,
                source,
                excluded_source_parameter,
                policy,
            )? {
                return Ok(intersections);
            }
            let line_relation = source.relation_to_line_with_contacts(&line, policy);
            let line_contacts = match line_relation {
                Classification::Decided(
                    BezierLineContactRelation::ControlHullDisjoint { .. }
                    | BezierLineContactRelation::NoContact,
                ) => Vec::new(),
                Classification::Decided(BezierLineContactRelation::OnSupportingLine) => {
                    return self.collinear_rational_intersections(
                        source,
                        &unit,
                        excluded_source_parameter,
                        policy,
                    );
                }
                Classification::Decided(BezierLineContactRelation::Contacts { contacts }) => {
                    contacts
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut contacts = Vec::with_capacity(line_contacts.len());
            for contact in line_contacts {
                let source_parameter = contact.parameter().clone();
                if let Some(excluded) = excluded_source_parameter {
                    match excluded.cmp_by_refinement(&source_parameter.clone().into(), policy)? {
                        Classification::Decided(std::cmp::Ordering::Equal) => continue,
                        Classification::Decided(_) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let point = match rational_point_evidence_at_parameter(
                    source,
                    &source_parameter,
                    policy,
                )? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let chord_parameter =
                    match self.parameter_at_certified_point(point.clone(), policy)? {
                        Classification::Decided(Some(parameter)) => parameter,
                        Classification::Decided(None) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let tangent_cross_sign = match (contact.kind(), contact.crossing_direction()) {
                    (
                        BezierLineContactKind::Crossing,
                        Some(BezierLineCrossingDirection::NegativeToPositive),
                    ) => RealSign::Positive,
                    (
                        BezierLineContactKind::Crossing,
                        Some(BezierLineCrossingDirection::PositiveToNegative),
                    ) => RealSign::Negative,
                    (BezierLineContactKind::Tangent, None) => RealSign::Zero,
                    (BezierLineContactKind::Crossing, None)
                    | (BezierLineContactKind::Tangent, Some(_)) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                };
                contacts.push(BezierAlgebraicChordRationalContact2 {
                    chord_parameter,
                    other_parameter: CurveParameter2::from(source_parameter),
                    point,
                    tangent_cross_sign,
                });
            }
            return Ok(Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::Contacts(contacts),
            ));
        }
        match self.recursive_projective_rational_intersections(
            source,
            &CurveParameterRange2::unit(),
            excluded_source_parameter,
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                Ok(Classification::Decided(intersections))
            }
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Reuses finite chord replay when the rational source has the identical
    /// normalized affine parameter. All endpoint contacts remain present;
    /// adjacency belongs to the region consumer.
    pub(crate) fn exact_linear_rational_intersections(
        &self,
        source: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierAlgebraicChordRationalIntersections2>> {
        let Some(line) = source.exact_linear_parameterization_line() else {
            return Ok(None);
        };
        let chord = match Self::try_new(
            line.start().clone().into(),
            line.end().clone().into(),
            policy,
        )? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(_) => return Ok(None),
        };
        if policy
            .strict_predicate_pass(|| self.is_strictly_one_sided_of_exact_line(&line, policy))?
            == Classification::Decided(true)
        {
            return Ok(Some(BezierAlgebraicChordRationalIntersections2::Contacts(
                Vec::new(),
            )));
        }
        let intersections = match self.chord_intersections(&chord, policy)? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(_) => return Ok(None),
        };
        Ok(Some(match intersections {
            BezierAlgebraicChordPairIntersections2::Contacts(contacts) => {
                let mut retained = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    let parameter = match contact
                        .second_parameter
                        .exact_line_curve_parameter(policy)?
                    {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                    let parameter = if let Some(parameter) = parameter.as_recursive_projective() {
                        // This contact owns the same tangent and finite-domain
                        // evidence as general rational replay. Keep it with the
                        // selected scalar so later corner operations can reuse
                        // the proof. A pre-existing specialized identity needs
                        // its complete kernel; never overwrite that evidence.
                        if parameter.data.identity.is_some() {
                            return Ok(None);
                        }
                        let location = if contact.first_parameter.is_endpoint_of(self, true) {
                            BezierRecursiveChordContactLocation2::Start
                        } else if contact.first_parameter.is_endpoint_of(self, false) {
                            BezierRecursiveChordContactLocation2::End
                        } else {
                            BezierRecursiveChordContactLocation2::Interior
                        };
                        CurveParameter2::from_recursive_projective(
                            parameter.clone().with_chord_rational_tangent_identity(
                                self.clone(),
                                source.clone(),
                                contact.tangent_cross_sign,
                                location,
                            ),
                        )
                    } else {
                        parameter
                    };
                    retained.push(BezierAlgebraicChordRationalContact2 {
                        chord_parameter: contact.first_parameter,
                        other_parameter: parameter,
                        point: contact.point,
                        tangent_cross_sign: contact.tangent_cross_sign,
                    });
                }
                BezierAlgebraicChordRationalIntersections2::Contacts(retained)
            }
            BezierAlgebraicChordPairIntersections2::Overlaps(overlaps) => {
                let mut retained = Vec::with_capacity(overlaps.len());
                for overlap in overlaps {
                    let mut parameters = [None, None];
                    for (index, parameter) in overlap.second_range.iter().enumerate() {
                        match parameter.exact_line_curve_parameter(policy)? {
                            Classification::Decided(parameter) => {
                                parameters[index] = Some(parameter)
                            }
                            Classification::Uncertain(_) => return Ok(None),
                        }
                    }
                    let [start, end] = parameters.map(|p| p.expect("two overlap boundaries"));
                    retained.push(BezierAlgebraicChordRationalOverlap2 {
                        chord: self.clone(),
                        source: source.clone(),
                        chord_range: overlap.first_range,
                        source_range: CurveParameterRange2::new_validated(start, end),
                        orientation: overlap.orientation,
                    });
                }
                BezierAlgebraicChordRationalIntersections2::Overlaps(retained)
            }
        }))
    }

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

    pub(super) fn chord_intersections_once(
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
                if first.overlaps(&second, &CurveContext::STRICT) == Classification::Decided(false)
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

    pub(super) fn collinear_chord_intersections(
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

    pub(super) fn collinear_partitioned_rational_intersections(
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
            let derivative =
                match BezierParameterPolynomial::try_new_power_basis(derivative, policy)? {
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

    pub(super) fn collinear_point_contact(
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

    pub(super) fn point_parameter_order(
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

    pub(super) fn collinear_boundary_from_source_endpoint(
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

    pub(super) fn collinear_boundary_from_partition_boundary(
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

    pub(super) fn collinear_boundary_from_chord_endpoint(
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
    pub(super) fn collinear_monotone_source_parameter_at_chord_endpoint(
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
            let polynomial = match BezierParameterPolynomial::try_new_power_basis(
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

    pub(super) fn collinear_source_parameters_at_chord_endpoint(
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

    pub(super) fn independent_support_system(
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

    pub(super) fn source_incidence_system(
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
                CurvePoint2(CurvePointData2::Exact(point)) => line.classify_point(point, policy),
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

impl BezierAlgebraicChord2 {
    /// Returns the original direction authority and whether this traversal is
    /// reversed relative to it. Exact parallel construction preserves this
    /// relation structurally, even when its normalized vector is not a pair of
    /// represented `Real`s.
    pub(super) fn tangent_authority(&self) -> (&Self, bool) {
        let mut current = self;
        let mut reversed = false;
        loop {
            reversed ^= current.retained_support_orientation_is_reversed();
            let support = current.retained_support();
            let (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) = (support.start(), support.end())
            else {
                return (support, reversed);
            };
            if !start.shares_carrier(end) || start.at_end == end.at_end {
                return (support, reversed);
            }
            // Displacement preserves the source tangent in endpoint order.
            // The retained support may itself be reversed even when the
            // current finite chord was constructed in traversal order.
            reversed ^= start.at_end;
            current = &start.data.source;
        }
    }

    /// Returns the direction of this support relative to one source-unit
    /// tangent at `parameter`. The two analytic endpoints share every term
    /// except tangent distance, so their difference is exactly the signed
    /// tangent displacement; no endpoint coordinates or selected fields need
    /// to be reconstructed.
    pub(super) fn authored_source_tangent_displacement_sign(
        &self,
        parameter: &CurveParameter2,
        frame_tangent: Option<&BezierAnalyticParallelTangentField2>,
        policy: &CurveContext,
        source_matches: impl Fn(&BezierParallelSource2) -> bool,
    ) -> Option<RealSign> {
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (self.start(), self.end())
        else {
            return None;
        };
        let source_matches = source_matches(start.data.parallel.source());
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!(
                "authored tangent identity parallel={} source={} parameter={} region={} frame={} translation={} policy={}",
                start.data.parallel == end.data.parallel,
                source_matches,
                start.data.parameter == end.data.parameter,
                start.data.parameter.matches_region_parameter(parameter),
                start.data.frame_tangent == end.data.frame_tangent
                    && start.data.frame_tangent.as_deref() == frame_tangent,
                start.data.translation_x == end.data.translation_x
                    && start.data.translation_y == end.data.translation_y,
                policy.accepts_retained_policy(start.data.policy)
                    && policy.accepts_retained_policy(end.data.policy),
            );
        }
        if !(start.data.parallel == end.data.parallel
            && source_matches
            && start.data.parameter == end.data.parameter
            && start.data.parameter.matches_region_parameter(parameter)
            && start.data.frame_tangent == end.data.frame_tangent
            && start.data.frame_tangent.as_deref() == frame_tangent
            && start.data.translation_x == end.data.translation_x
            && start.data.translation_y == end.data.translation_y
            && policy.accepts_retained_policy(start.data.policy)
            && policy.accepts_retained_policy(end.data.policy))
        {
            return None;
        }
        match real_sign(
            &(&end.data.tangent_distance - &start.data.tangent_distance),
            &CurveContext::STRICT,
        ) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => Some(sign),
            Some(RealSign::Zero) | None => None,
        }
    }

    /// Returns whether this support was authored as one nonzero source-unit
    /// tangent at `parameter`.
    pub(super) fn is_authored_source_tangent_at_region_parameter(
        &self,
        parallel: &BezierParallel2,
        parameter: &CurveParameter2,
        frame_tangent: Option<&BezierAnalyticParallelTangentField2>,
        policy: &CurveContext,
    ) -> bool {
        self.authored_source_tangent_displacement_sign(parameter, frame_tangent, policy, |source| {
            source == parallel.source()
        })
        .is_some()
    }

    /// Returns whether `point` is the zero-tangent-distance origin of this
    /// authored analytic tangent line.  Unlike the direction-only predicate
    /// above, incidence also requires the same parallel distance, frame, and
    /// translation so the two constructions share the exact affine origin.
    pub(super) fn contains_authored_source_tangent_origin(
        &self,
        point: &BezierAnalyticParallelPoint2,
        policy: &CurveContext,
    ) -> bool {
        if !policy.accepts_retained_policy(point.data.policy)
            || point.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
        {
            return false;
        }
        let (authority, _) = self.tangent_authority();
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (authority.start(), authority.end())
        else {
            return false;
        };
        start.data.parallel == end.data.parallel
            && start.data.parallel == point.data.parallel
            && start.data.parameter == end.data.parameter
            && start.data.parameter == point.data.parameter
            && start.data.frame_tangent == end.data.frame_tangent
            && start.data.frame_tangent == point.data.frame_tangent
            && start.data.translation_x == end.data.translation_x
            && start.data.translation_x == point.data.translation_x
            && start.data.translation_y == end.data.translation_y
            && start.data.translation_y == point.data.translation_y
            && policy.accepts_retained_policy(start.data.policy)
            && policy.accepts_retained_policy(end.data.policy)
            && matches!(
                real_sign(
                    &(&end.data.tangent_distance - &start.data.tangent_distance),
                    &CurveContext::STRICT,
                ),
                Some(RealSign::Negative | RealSign::Positive),
            )
    }

    pub(super) fn shared_tangent_orientation(&self, other: &Self) -> Option<bool> {
        let (first, first_reversed) = self.tangent_authority();
        let (second, second_reversed) = other.tangent_authority();
        if let Some(support_reversed) = first
            .retained_support_orientation_to(second)
            .or_else(|| first.shares_retained_support(second).then_some(false))
        {
            return Some(first_reversed ^ support_reversed ^ second_reversed);
        }
        // Offset subsegments can retain different finite support descendants
        // while still descending from the same procedural normal translation.
        // That construction owns the same exact tangent relation without any
        // endpoint-field comparison.
        self.retained_normal_offset_tangent_reversal_to(other)
    }

    /// Recognizes two finite subchords of the same retained radial line even
    /// when clipping replaced both endpoints. Each authority has direction
    /// `(a_end-a_start)(P-C)`; comparing the two exact scalar signs is enough
    /// to recover their relative traversal without endpoint incidence.
    pub(super) fn retained_radial_tangent_reversal_to(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<bool>> {
        let (first, first_reversed) = self.tangent_authority();
        let (second, second_reversed) = other.tangent_authority();
        #[cfg(test)]
        let evidence_kind = |point: &CurvePoint2| match point {
            CurvePoint2(CurvePointData2::Exact(_)) => "exact",
            CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
            CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
            CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                "similarity"
            }
        };
        let (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first_start)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first_end)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second_start)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second_end)),
        ) = (first.start(), first.end(), second.start(), second.end())
        else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                eprintln!(
                    "radial tangent ancestry kinds=({},{})/({},{}) reversals={first_reversed}/{second_reversed}",
                    evidence_kind(first.start()),
                    evidence_kind(first.end()),
                    evidence_kind(second.start()),
                    evidence_kind(second.end()),
                );
            }
            return Ok(None);
        };
        let shared_source = first_start
            .data
            .source
            .shares_exact_evidence(&second_start.data.source);
        let shared_circle =
            first_start.data.source.semicircle() == second_start.data.source.semicircle();
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some()
            && (!shared_source || !shared_circle)
        {
            let source = |point: &BezierAlgebraicCuspChordDerivedPoint2| match &point.data.source {
                BezierAlgebraicCuspDerivedPointSource2::Chord(point) => {
                    ("chord", Arc::as_ptr(&point.data) as usize)
                }
                BezierAlgebraicCuspDerivedPointSource2::Mapped { parameter, .. } => {
                    ("mapped", Arc::as_ptr(parameter) as usize)
                }
            };
            eprintln!(
                "radial tangent ancestry shared-source={shared_source} shared-circle={shared_circle} first={:?}/{:?} second={:?}/{:?}",
                source(first_start),
                source(first_end),
                source(second_start),
                source(second_end),
            );
        }
        if !shared_source || !shared_circle {
            return Ok(None);
        }
        let first_difference =
            first_start.common_untranslated_radial_difference_sign(first_end, policy)?;
        let second_difference =
            second_start.common_untranslated_radial_difference_sign(second_end, policy)?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some()
            && (first_difference.is_none() || second_difference.is_none())
        {
            eprintln!(
                "radial tangent ancestry differences={first_difference:?}/{second_difference:?}"
            );
        }
        let first_sign = match first_difference {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) | None => return Ok(None),
        };
        let second_sign = match second_difference {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) | None => return Ok(None),
        };
        Ok(Some(
            first_reversed ^ second_reversed ^ (first_sign != second_sign),
        ))
    }

    pub(super) fn tangent_relation_sign_by_refinement(
        &self,
        other: &Self,
        cross: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let zero = Real::zero();
        let one = Real::one();
        self.tangent_cross_dot_linear_combination_sign(
            other,
            if cross { &one } else { &zero },
            if cross { &zero } else { &one },
            policy,
        )
    }

    /// Cancels the anchors and positive speed denominators of two retained
    /// tangent displacements before signing their polynomial directions.
    /// Selected parameters remain authoritative; equality can identify one
    /// shared scalar even when the two witnesses use different root carriers.
    pub(super) fn analytic_tangent_pair_linear_combination_sign(
        &self,
        other: &Self,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(first)),
            CurvePoint2(CurvePointData2::AnalyticParallel(first_end)),
            CurvePoint2(CurvePointData2::AnalyticParallel(second)),
            CurvePoint2(CurvePointData2::AnalyticParallel(second_end)),
        ) = (self.start(), self.end(), other.start(), other.end())
        else {
            return Ok(None);
        };
        policy.bounded_exact_predicate_pass(|| {
            let mut orientation = RealSign::Positive;
            for (start, end) in [(first, first_end), (second, second_end)] {
                let Some(Classification::Decided(sign)) =
                    start.shared_tangent_displacement_sign(end, policy)
                else {
                    return Ok(None);
                };
                orientation = product_sign(orientation, sign);
            }
            let (first_x, first_y) = first.frame_tangent_power_basis()?;
            let (second_x, second_y) = second.frame_tangent_power_basis()?;
            let first_parameter = first.data.parameter.curve_parameter();
            let second_parameter = second.data.parameter.curve_parameter();
            let strict = policy.strict_counterpart();
            let sign = if matches!(
                first_parameter.same_value(&second_parameter, &strict)?,
                Classification::Decided(true)
            ) {
                let cross = polynomial_subtract(
                    &polynomial_multiply(first_x, second_y),
                    &polynomial_multiply(first_y, second_x),
                );
                let dot = polynomial_add(
                    &polynomial_multiply(first_x, second_x),
                    &polynomial_multiply(first_y, second_y),
                );
                first.parameter_polynomial_sign(
                    &polynomial_add(
                        &polynomial_scale(&cross, cross_scale),
                        &polynomial_scale(&dot, dot_scale),
                    ),
                    &strict,
                )?
            } else if let (Some(first_parameter), Some(second_parameter)) = (
                first_parameter.as_bezier_parameter(),
                second_parameter.as_bezier_parameter(),
            ) {
                let cross = bivariate_subtract(
                    &bivariate_outer_product(first_x, second_y),
                    &bivariate_outer_product(first_y, second_x),
                );
                let dot = bivariate_add(
                    &bivariate_outer_product(first_x, second_x),
                    &bivariate_outer_product(first_y, second_y),
                );
                signed_bivariate_at_parameter_pair(
                    &bivariate_add(
                        &bivariate_scale(cross, cross_scale),
                        &bivariate_scale(dot, dot_scale),
                    ),
                    first_parameter,
                    second_parameter,
                    &strict,
                )?
            } else {
                return Ok(None);
            };
            Ok(match sign {
                Classification::Decided(sign) => Some(product_sign(orientation, sign)),
                Classification::Uncertain(_) => None,
            })
        })
    }

    pub(super) fn certified_axis_tangent_relation_sign(
        &self,
        other: &Self,
        cross: bool,
    ) -> Option<RealSign> {
        let (first, second) = self
            .certified_axis_direction()
            .zip(other.certified_axis_direction())?;
        let ((first_x, first_y), (second_x, second_y)) =
            (first.cardinal_components(), second.cardinal_components());
        let value = if cross {
            first_x * second_y - first_y * second_x
        } else {
            first_x * second_x + first_y * second_y
        };
        Some(match value.cmp(&0) {
            std::cmp::Ordering::Less => RealSign::Negative,
            std::cmp::Ordering::Equal => RealSign::Zero,
            std::cmp::Ordering::Greater => RealSign::Positive,
        })
    }

    /// Returns either `chord x self` or `chord dot self` when this chord is
    /// the unit-tangent witness authored by a recursive rational contact.
    pub(super) fn retained_rational_tangent_relation_sign_to(
        &self,
        chord: &Self,
        cross: bool,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let (authority, reversed) = self.tangent_authority();
        let (chord, chord_reversed) = chord.tangent_authority();
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (authority.start(), authority.end())
        else {
            return None;
        };
        start
            .shared_rational_tangent_relation_sign_to_chord(end, chord, cross, policy)
            .map(|result| {
                result.map(|classification| {
                    classification.map(|sign| {
                        if reversed ^ chord_reversed {
                            product_sign(sign, RealSign::Negative)
                        } else {
                            sign
                        }
                    })
                })
            })
    }

    pub(super) fn retained_rational_tangent_cross_sign_to(
        &self,
        chord: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        self.retained_rational_tangent_relation_sign_to(chord, true, policy)
    }

    pub(super) fn retained_rational_tangent_dot_sign_to(
        &self,
        chord: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        self.retained_rational_tangent_relation_sign_to(chord, false, policy)
    }

    /// Returns `self x tangent` when `self` descends from two unrotated
    /// concentric radial images of one retained circle contact and `tangent`
    /// is the source-tangent chord authored at that same contact.
    pub(super) fn retained_radial_tangent_cross_sign_to(
        &self,
        tangent: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let (radial, radial_reversed) = self.tangent_authority();
        let (tangent, tangent_reversed) = tangent.tangent_authority();
        let (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(start)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(end)),
        ) = (radial.start(), radial.end())
        else {
            return None;
        };
        let radial_sign = match start.common_untranslated_radial_difference_sign(end, policy) {
            Ok(Some(sign)) => sign,
            Ok(None) => return None,
            Err(error) => return Some(Err(error)),
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!("retained radial tangent relation scale={radial_sign:?}");
        }
        let sign = start
            .data
            .source
            .radial_cross_authored_tangent_sign(tangent, policy)?;
        Some(sign.map(|classification| {
            classification.map(|sign| {
                let sign = product_sign(sign, radial_sign);
                if radial_reversed ^ tangent_reversed {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }
            })
        }))
    }

    /// Returns only construction-owned or compact represented evidence for
    /// `self x other`. This deliberately excludes endpoint-box refinement and
    /// recursive norms so callers can exhaust provenance before selecting a
    /// cold general authority.
    pub(super) fn retained_tangent_cross_sign(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        if let Some(sign) = other.retained_rational_tangent_cross_sign_to(self, policy) {
            return Some(sign);
        }
        if let Some(sign) = self.retained_rational_tangent_cross_sign_to(other, policy) {
            return Some(sign.map(|classification| {
                classification.map(|sign| product_sign(sign, RealSign::Negative))
            }));
        }
        if let Some(sign) = self.retained_radial_tangent_cross_sign_to(other, policy) {
            return Some(sign);
        }
        if let Some(sign) = other.retained_radial_tangent_cross_sign_to(self, policy) {
            return Some(sign.map(|classification| {
                classification.map(|sign| product_sign(sign, RealSign::Negative))
            }));
        }
        if self.shared_tangent_orientation(other).is_some() {
            return Some(Ok(Classification::Decided(RealSign::Zero)));
        }
        if let Some(sign) = self.certified_axis_tangent_relation_sign(other, true) {
            return Some(Ok(Classification::Decided(sign)));
        }
        if let (Some(first), Some(second)) = (
            self.data.certified_unit_tangent.as_ref(),
            other.data.certified_unit_tangent.as_ref(),
        ) && Arc::ptr_eq(first, second)
        {
            return Some(Ok(Classification::Decided(RealSign::Zero)));
        }
        let (Some(first), Some(second)) = (
            self.certified_unit_tangent(),
            other.certified_unit_tangent(),
        ) else {
            return None;
        };
        let cross = Real::diff_of_products(&first.0, &second.1, &first.1, &second.0);
        cross
            .refine_sign_until(-512)
            .map(|sign| Ok(Classification::Decided(sign)))
    }

    /// Signs `cross_scale * (self x other) + dot_scale * (self dot other)`.
    /// Both products share the same positive normalization factor, so the
    /// unnormalized endpoint differences are authoritative. Independent
    /// selected endpoint fields are refined in place and only an
    /// APPROXIMATE_512 query may terminate an unresolved equality.
    pub(crate) fn tangent_cross_dot_linear_combination_sign(
        &self,
        other: &Self,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        // Boolean clipping and offset trimming replace finite endpoints but
        // retain an exact direction authority. Evaluate every tangent
        // relation on those smallest authored supports before inspecting the
        // descendant endpoints: both cross and dot acquire the same sign
        // under one reversal, so the requested linear combination needs only
        // one final orientation correction. This also prevents a chord-pair
        // endpoint from being reconstructed merely to rediscover its
        // ancestor's tangent.
        let (first_authority, first_reversed) = self.tangent_authority();
        let (second_authority, second_reversed) = other.tangent_authority();
        if !Arc::ptr_eq(&first_authority.data, &self.data)
            || !Arc::ptr_eq(&second_authority.data, &other.data)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "retained-direction-authority",
            );
            return first_authority
                .tangent_cross_dot_linear_combination_sign(
                    second_authority,
                    cross_scale,
                    dot_scale,
                    policy,
                )
                .map(|classification| {
                    classification.map(|sign| {
                        if first_reversed ^ second_reversed {
                            product_sign(sign, RealSign::Negative)
                        } else {
                            sign
                        }
                    })
                });
        }
        if let Some(sign) = self.analytic_tangent_pair_linear_combination_sign(
            other,
            cross_scale,
            dot_scale,
            policy,
        )? {
            return Ok(Classification::Decided(sign));
        }
        if dot_scale.zero_status() == ZeroKnowledge::Zero
            && let Some(cross_scale_sign @ (RealSign::Negative | RealSign::Positive)) =
                real_sign(cross_scale, &CurveContext::STRICT)
            && let Some(sign) = self.retained_tangent_cross_sign(other, policy)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "retained-construction-cross",
            );
            return sign.map(|classification| {
                classification.map(|sign| product_sign(sign, cross_scale_sign))
            });
        }
        if let Some(reversed) = self.shared_tangent_orientation(other) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                if reversed {
                    "shared-support-reversed"
                } else {
                    "shared-support-forward"
                },
            );
            let value = if reversed {
                -dot_scale.clone()
            } else {
                dot_scale.clone()
            };
            return Ok(real_sign(&value, &CurveContext::STRICT).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        if let (Some(first), Some(second)) = (
            self.data.certified_unit_tangent.as_ref(),
            other.data.certified_unit_tangent.as_ref(),
        ) && Arc::ptr_eq(first, second)
        {
            // Exact parallel translations clone this shared construction
            // authority.  Its vector is certified unit length, so the cross
            // is exactly zero and the dot exactly one; asking the scalar DAG
            // to rediscover x*y-y*x would lose the commutative identity.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "shared-unit-tangent-authority",
            );
            return Ok(real_sign(dot_scale, &CurveContext::STRICT).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        // This is a structural fast path, not a prerequisite for the complete
        // tangent predicate below.  Do not invoke `axis_direction` here to
        // rediscover an unretained cardinal fact from composite endpoints:
        // that can build the same large recursive field the general
        // endpoint-difference refinement deliberately avoids.
        if let (Some(first), Some(second)) = (
            self.certified_unit_tangent(),
            other.certified_unit_tangent(),
        ) {
            let structural_orientation = if first.0 == second.0 && first.1 == second.1 {
                Some(false)
            } else if first.0 == -second.0.clone() && first.1 == -second.1.clone() {
                Some(true)
            } else {
                None
            };
            if let Some(reversed) = structural_orientation {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-tangent-relation",
                    if reversed {
                        "equal-represented-unit-tangents-reversed"
                    } else {
                        "equal-represented-unit-tangents-forward"
                    },
                );
                let value = if reversed {
                    -dot_scale.clone()
                } else {
                    dot_scale.clone()
                };
                return Ok(real_sign(&value, &CurveContext::STRICT).map_or(
                    Classification::Uncertain(UncertaintyReason::RealSign),
                    Classification::Decided,
                ));
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "represented-unit-tangents",
            );
            let cross = Real::diff_of_products(&first.0, &second.1, &first.1, &second.0);
            let dot = &first.0 * &second.0 + &first.1 * &second.1;
            let value = cross_scale * cross + dot_scale * dot;
            return Ok(real_sign(&value, &CurveContext::STRICT).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        match (
            self.certified_unit_tangent(),
            other.certified_unit_tangent(),
        ) {
            (Some(first), None) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-tangent-relation",
                    "represented-retained-linear-form",
                );
                return other.tangent_cross_dot_vector_linear_combination_sign(
                    &first,
                    cross_scale,
                    dot_scale,
                    policy,
                );
            }
            (None, Some(second)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-tangent-relation",
                    "retained-represented-linear-form",
                );
                return self.tangent_cross_dot_vector_linear_combination_sign(
                    &second,
                    &(-cross_scale.clone()),
                    dot_scale,
                    policy,
                );
            }
            (Some(_), Some(_)) => {
                unreachable!("the represented pair was handled above")
            }
            (None, None) => {}
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-tangent-relation",
            "refinement",
        );
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let bounds = |chord: &Self| {
                let start = algebraic_chord_endpoint_local_bounds_refined(
                    chord.start(),
                    refinement_steps,
                    policy,
                );
                let end = algebraic_chord_endpoint_local_bounds_refined(
                    chord.end(),
                    refinement_steps,
                    policy,
                );
                match (start, end) {
                    (Classification::Decided(start), Classification::Decided(end)) => {
                        Some((start, end))
                    }
                    _ => None,
                }
            };
            let (Some((first_start, first_end)), Some((second_start, second_end))) =
                (bounds(self), bounds(other))
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let delta = |start: &Aabb2, end: &Aabb2, axis| {
                RealInterval::from_axis(end, axis).subtract(&RealInterval::from_axis(start, axis))
            };
            let first_x = delta(&first_start, &first_end, Axis2::X);
            let first_y = delta(&first_start, &first_end, Axis2::Y);
            let second_x = delta(&second_start, &second_end, Axis2::X);
            let second_y = delta(&second_start, &second_end, Axis2::Y);
            let strict = &CurveContext::STRICT;
            let cross = first_x.multiply(&second_y).and_then(|first| {
                first_y
                    .multiply(&second_x)
                    .map(|second| first.subtract(&second))
            });
            let dot = first_x
                .multiply(&second_x)
                .and_then(|first| first_y.multiply(&second_y).map(|second| first.add(&second)));
            let scale = |value: RealInterval, scale: &Real| {
                value.multiply(&RealInterval {
                    lower: scale.clone(),
                    upper: scale.clone(),
                })
            };
            let value = cross
                .and_then(|cross| scale(cross, cross_scale))
                .and_then(|cross| {
                    dot.and_then(|dot| scale(dot, dot_scale))
                        .map(|dot| cross.add(&dot))
                });
            let Some(value) = value else {
                continue;
            };
            if compare_reals(&value.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Classification::Decided(RealSign::Positive));
            }
            if compare_reals(&value.upper, &Real::zero(), strict) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(RealSign::Negative));
            }
            if compare_reals(&value.lower, &Real::zero(), strict) == Some(std::cmp::Ordering::Equal)
                && compare_reals(&value.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Ok(Classification::Decided(RealSign::Zero));
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(RealSign::Zero))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    }

    pub(crate) fn tangent_cross_sign(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(sign) = self.certified_axis_tangent_relation_sign(other, true) {
            return Ok(Classification::Decided(sign));
        }
        self.tangent_relation_sign_by_refinement(other, true, policy)
    }

    /// Cancels one structurally shared vertex before signing two adjacent
    /// chord tangents. Offset corner construction is the consumer: its source
    /// loop has already certified endpoint connectivity, while the general
    /// chord-pair kernel deliberately keeps its established four-endpoint
    /// dispatch for unrelated Boolean pairs.
    pub(crate) fn tangent_cross_sign_with_shared_endpoint(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let first_points = [self.start(), self.end()];
        let second_points = [other.start(), other.end()];
        let (first_index, second_index) =
            first_points
                .iter()
                .enumerate()
                .find_map(|(first_index, first)| {
                    second_points
                        .iter()
                        .enumerate()
                        .find_map(|(second_index, second)| {
                            (first.shares_storage(second) || *first == *second)
                                .then_some((first_index, second_index))
                        })
                })?;
        // Split, reversed, coalesced, and parallel-translated descendants can
        // retain one exact tangent authority even when their independently
        // selected endpoint fields make the oriented-area replay expensive.
        // A shared tangent authority is already a complete zero-cross
        // certificate; the orientation only affects the dot product.
        if self.shared_tangent_orientation(other).is_some() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "shared-endpoint-shared-tangent-authority",
            );
            return Some(Ok(Classification::Decided(RealSign::Zero)));
        }
        Some((|| {
            let shared_point = first_points[first_index];
            let first_other = first_points[1 - first_index];
            let second_other = second_points[1 - second_index];
            let reverse_side = |side| match side {
                crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                crate::classify::LineSide::On => crate::classify::LineSide::On,
                crate::classify::LineSide::Right => crate::classify::LineSide::Left,
            };
            let recursive_contact_side = match (first_other, second_other) {
                (CurvePoint2(CurvePointData2::AnalyticParallel(contact)), endpoint) => contact
                    .recursive_chord_contact_to_endpoint_oriented_side(
                        endpoint,
                        shared_point,
                        policy,
                    ),
                (endpoint, CurvePoint2(CurvePointData2::AnalyticParallel(contact))) => contact
                    .recursive_chord_contact_to_endpoint_oriented_side(
                        endpoint,
                        shared_point,
                        policy,
                    )
                    .map(|result| result.map(|classification| classification.map(reverse_side))),
                _ => None,
            };
            if let Some(side) = recursive_contact_side
                && let Classification::Decided(side) = side?
            {
                let mut sign = match side {
                    crate::classify::LineSide::Left => RealSign::Positive,
                    crate::classify::LineSide::On => RealSign::Zero,
                    crate::classify::LineSide::Right => RealSign::Negative,
                };
                if first_index != second_index {
                    sign = product_sign(sign, RealSign::Negative);
                }
                return Ok(Classification::Decided(sign));
            }
            if let (
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first)),
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second)),
            ) = (first_other, second_other)
                && let Some(side) = first.common_untranslated_radial_line_oriented_side(
                    second,
                    shared_point,
                    policy,
                )?
            {
                let mut sign = match side {
                    crate::classify::LineSide::Left => RealSign::Positive,
                    crate::classify::LineSide::On => RealSign::Zero,
                    crate::classify::LineSide::Right => RealSign::Negative,
                };
                if first_index != second_index {
                    sign = product_sign(sign, RealSign::Negative);
                }
                return Ok(Classification::Decided(sign));
            }
            match Self::try_new(first_other.clone(), second_other.clone(), policy) {
                Err(CurveError::ZeroLengthLine) => {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Ok(Classification::Decided(diagonal)) => {
                    if let Classification::Decided(side) =
                        diagonal.oriented_support_side(shared_point, policy)?
                    {
                        let mut sign = match side {
                            crate::classify::LineSide::Left => RealSign::Positive,
                            crate::classify::LineSide::On => RealSign::Zero,
                            crate::classify::LineSide::Right => RealSign::Negative,
                        };
                        if first_index != second_index {
                            sign = product_sign(sign, RealSign::Negative);
                        }
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-tangent-relation",
                            "shared-endpoint-diagonal",
                        );
                        return Ok(Classification::Decided(sign));
                    }
                }
                Ok(Classification::Uncertain(_)) | Err(_) => {}
            }
            let other_point = second_points[1 - second_index];
            let side = match self.represented_oriented_side(other_point, policy)? {
                Classification::Decided(side) => side,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut sign = match side {
                crate::classify::LineSide::Left => RealSign::Positive,
                crate::classify::LineSide::On => RealSign::Zero,
                crate::classify::LineSide::Right => RealSign::Negative,
            };
            if second_index == 1 {
                sign = product_sign(sign, RealSign::Negative);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "shared-endpoint-oriented-area",
            );
            Ok(Classification::Decided(sign))
        })())
    }

    pub(crate) fn tangent_dot_sign(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(sign) = self.certified_axis_tangent_relation_sign(other, false) {
            return Ok(Classification::Decided(sign));
        }
        self.tangent_relation_sign_by_refinement(other, false, policy)
    }

    /// Signs one exact linear form of this chord's traversal tangent.
    ///
    /// Ordinary algebraic endpoint images use their retained polynomial
    /// predicates first, including exact zero. Unresolved forms reuse the
    /// oriented support's recursive field before interval refinement reaches
    /// the APPROXIMATE_512 terminal equality policy.
    pub(super) fn tangent_linear_form_sign(
        &self,
        coefficient_x: &Real,
        coefficient_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        if let Some(tangent) = self.certified_unit_tangent() {
            let value = &tangent.0 * coefficient_x + &tangent.1 * coefficient_y;
            return Ok(real_sign(&value, policy).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        // Splits retain the same oriented straight support. Its original
        // endpoint field owns the tangent; later contact endpoints only bound
        // the finite segment and need no refinement for this query.
        let (support, reversed) = self.smallest_incidence_support();
        support.validate_policy(policy)?;
        let mut endpoints = support.direction_endpoints(policy);
        if reversed {
            endpoints.swap(0, 1);
        }
        let [start_point, end_point] = endpoints;
        if let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (start_point, end_point)
            && let Some(sign) = start.shared_tangent_displacement_linear_form_sign(
                end,
                coefficient_x,
                coefficient_y,
                policy,
            )
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-linear-form",
                "retained-unit-displacement",
            );
            return sign;
        }
        let exact_endpoint_sign = policy.strict_predicate_pass(|| -> CurveResult<_> {
            Ok(match (start_point, end_point) {
                (
                    CurvePoint2(CurvePointData2::Exact(start)),
                    CurvePoint2(CurvePointData2::Exact(end)),
                ) => real_sign(
                    &(coefficient_x * (end.x() - start.x())
                        + coefficient_y * (end.y() - start.y())),
                    policy,
                )
                .map(Classification::Decided),
                (
                    CurvePoint2(CurvePointData2::Algebraic(start)),
                    CurvePoint2(CurvePointData2::Algebraic(end)),
                ) => {
                    let start = match start.predicate_evaluator(policy)? {
                        Classification::Decided(start) => start,
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    };
                    let end = match end.predicate_evaluator(policy)? {
                        Classification::Decided(end) => end,
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    };
                    Some(signed_algebraic_point_linear_difference(
                        &end,
                        &start,
                        coefficient_x,
                        coefficient_y,
                        policy,
                    )?)
                }
                (
                    CurvePoint2(CurvePointData2::Exact(start)),
                    CurvePoint2(CurvePointData2::Algebraic(end)),
                ) => {
                    let end = match end.predicate_evaluator(policy)? {
                        Classification::Decided(end) => end,
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    };
                    Some(
                        end.homogeneous_linear_difference_sign(
                            start.x(),
                            start.y(),
                            &Real::one(),
                            coefficient_x,
                            coefficient_y,
                            RealSign::Positive,
                            policy,
                        )?
                        .map(|sign| product_sign(sign, RealSign::Negative)),
                    )
                }
                (
                    CurvePoint2(CurvePointData2::Algebraic(start)),
                    CurvePoint2(CurvePointData2::Exact(end)),
                ) => {
                    let start = match start.predicate_evaluator(policy)? {
                        Classification::Decided(start) => start,
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    };
                    Some(start.homogeneous_linear_difference_sign(
                        end.x(),
                        end.y(),
                        &Real::one(),
                        coefficient_x,
                        coefficient_y,
                        RealSign::Positive,
                        policy,
                    )?)
                }
                _ => None,
            })
        })?;
        if let Some(Classification::Decided(sign)) = exact_endpoint_sign {
            return Ok(Classification::Decided(sign));
        }
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (Classification::Decided(start), Classification::Decided(end)) = (
                algebraic_chord_endpoint_bounds_refined(start_point, refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(end_point, refinement_steps, policy),
            ) else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let delta_x = RealInterval::from_axis(&end, Axis2::X)
                .subtract(&RealInterval::from_axis(&start, Axis2::X));
            let delta_y = RealInterval::from_axis(&end, Axis2::Y)
                .subtract(&RealInterval::from_axis(&start, Axis2::Y));
            let coefficient_x = RealInterval {
                lower: coefficient_x.clone(),
                upper: coefficient_x.clone(),
            };
            let coefficient_y = RealInterval {
                lower: coefficient_y.clone(),
                upper: coefficient_y.clone(),
            };
            let strict = &CurveContext::STRICT;
            let value = delta_x.multiply(&coefficient_x).and_then(|first| {
                delta_y
                    .multiply(&coefficient_y)
                    .map(|second| first.add(&second))
            });
            let Some(value) = value else {
                continue;
            };
            if compare_reals(&value.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Classification::Decided(RealSign::Positive));
            }
            if compare_reals(&value.upper, &Real::zero(), strict) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(RealSign::Negative));
            }
            if compare_reals(&value.lower, &Real::zero(), strict) == Some(std::cmp::Ordering::Equal)
                && compare_reals(&value.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Ok(Classification::Decided(RealSign::Zero));
            }
        }
        // Preserve every native interval decision before adjoining fields.
        // Unresolved forms, including correlated zeros with arbitrary exact
        // coefficients, reuse the existing projective direction authority
        // before an approximate terminal. Procedural offsets contribute their
        // original direction, without a cancelled normal.
        if let Classification::Decided(Some(frame)) = policy.strict_predicate_pass(|| {
            support.recursive_projective_endpoints_with_direction(policy)
        })? {
            let [start, end] = frame.direction_endpoints;
            let value = (|| {
                let (x, y, _) = end.difference_numerators(&start)?;
                x.scale(coefficient_x)?.add(&y.scale(coefficient_y)?)
            })();
            if let Some(value) = value
                && let Classification::Decided(sign) =
                    policy.strict_predicate_pass(|| value.sign(policy))?
            {
                return Ok(Classification::Decided(if reversed {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }));
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(RealSign::Zero))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    }

    pub(super) fn tangent_relation_to_vector_sign(
        &self,
        vector: &(Real, Real),
        cross: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let (coefficient_x, coefficient_y) = if cross {
            (vector.1.clone(), -vector.0.clone())
        } else {
            (vector.0.clone(), vector.1.clone())
        };
        self.tangent_linear_form_sign(&coefficient_x, &coefficient_y, policy)
    }

    /// Signs `cross_scale * (anchor x tangent) +
    /// dot_scale * (anchor dot tangent)` without normalizing this chord.
    pub(crate) fn tangent_cross_dot_vector_linear_combination_sign(
        &self,
        anchor: &(Real, Real),
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let coefficient_x = dot_scale * &anchor.0 - cross_scale * &anchor.1;
        let coefficient_y = cross_scale * &anchor.0 + dot_scale * &anchor.1;
        self.tangent_linear_form_sign(&coefficient_x, &coefficient_y, policy)
    }

    /// Signs `cross_scale * (T_chord x T_parallel) + dot_scale *
    /// (T_chord dot T_parallel)` at one retained analytic parameter.
    ///
    /// The two chord endpoint fields and analytic source parameter stay on
    /// separate tensor axes. Projective denominator and parallel derivative
    /// scale signs are restored only after the exact trivariate linear form is
    /// signed, so the result is valid for ordinary and retained-offset chord
    /// carriers under either terminal policy.
    pub(crate) fn tangent_cross_dot_parallel_linear_combination_sign(
        &self,
        parallel: &BezierParallel2,
        parameter: &CurveParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let source_sign = match self.tangent_cross_dot_parallel_source_linear_combination_sign(
            parallel,
            parameter,
            cross_scale,
            dot_scale,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let scale = match parallel.parallel_derivative_scale_sign(parameter, policy)? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(product_sign(source_sign, scale)))
    }

    /// Signs the same chord relation against the parallel's source tangent.
    /// Circle normal frames use this direction even where the parallel's
    /// derivative reverses. The traversal predicate above applies that scale
    /// only when the actual parallel tangent is requested.
    pub(super) fn tangent_cross_dot_parallel_source_linear_combination_sign(
        &self,
        parallel: &BezierParallel2,
        parameter: &CurveParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        // A retained oriented unit tangent is already the complete chord-side
        // authority for this predicate. Keep the parallel parameter as the
        // only algebraic axis instead of rebuilding two endpoint fields and a
        // trivariate support system merely to recover the same direction.
        if let Some((tangent_x, tangent_y)) = self.certified_unit_tangent() {
            let source_sign = match parallel
                .vector_source_tangent_cross_dot_linear_combination_sign(
                    parameter,
                    &tangent_x,
                    &tangent_y,
                    cross_scale,
                    dot_scale,
                    policy,
                )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Ok(Classification::Decided(source_sign));
        }
        // Retained endpoint fields also support native and exact target
        // parameters. A local root keeps its defining coefficient relations;
        // a native target joins this field only for the requested sign. The
        // oriented support line is (-dy, dx, c) up to positive scale.
        if let Some(line) = self.recursive_projective_support_line(policy)? {
            let differential = parallel.differential()?;
            let coefficients = (|| {
                let x_scale = line.x.scale(cross_scale)?.add(&line.y.scale(dot_scale)?)?;
                let y_scale = line
                    .y
                    .scale(cross_scale)?
                    .subtract(&line.x.scale(dot_scale)?)?;
                recursive_quadratic_polynomial_combine(
                    &differential
                        .tangent_x
                        .iter()
                        .map(|value| x_scale.scale(value))
                        .collect::<Option<Vec<_>>>()?,
                    &differential
                        .tangent_y
                        .iter()
                        .map(|value| y_scale.scale(value))
                        .collect::<Option<Vec<_>>>()?,
                    false,
                )
            })();
            if let Some(coefficients) = coefficients {
                let sign = if let Some(root) = parameter.as_recursive_projective() {
                    root.recursive_polynomial_sign_joined(&coefficients, policy)?
                } else {
                    recursive_projective_polynomial_sign_at_parameter(
                        &line.x.field(),
                        &coefficients,
                        parameter,
                        policy,
                    )?
                };
                if let Classification::Decided(sign) = sign {
                    return Ok(Classification::Decided(sign));
                }
            }
        }
        // The independent trivariate fallback needs an ordinary root axis;
        // promotion is demand-driven and does not replace the retained root.
        let parameter = match promote_curve_region_bezier_parameter(parameter, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let support = match self.independent_support_system(policy)? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let differential = parallel.differential()?;
        let Some(line_x) = trivariate_from_axis_bivariate_coefficients(
            std::slice::from_ref(&support.line_x),
            2,
            [0, 1],
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(line_y) = trivariate_from_axis_bivariate_coefficients(
            std::slice::from_ref(&support.line_y),
            2,
            [0, 1],
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(tangent_x) =
            TrivariatePolynomial2::from_axis_polynomial_or_zero(&differential.tangent_x, 2)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(tangent_y) =
            TrivariatePolynomial2::from_axis_polynomial_or_zero(&differential.tangent_y, 2)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(cross) = TrivariatePolynomial2::sum_products(&[
            (&line_x, &tangent_y, false),
            (&line_y, &tangent_x, true),
        ]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(dot) = TrivariatePolynomial2::sum_products(&[
            (&line_x, &tangent_x, false),
            (&line_y, &tangent_y, false),
        ]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(relation) = cross
            .scale(cross_scale)
            .and_then(|cross| dot.scale(dot_scale).and_then(|dot| cross.add(&dot)))
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let relation = trivariate_reduce_parameter_pair_relations(
            &relation,
            &support.first_parameter,
            &support.second_parameter,
        )
        .unwrap_or(relation);
        let source_sign = match trivariate_parameter_triple_sign_by_refinement(
            &relation,
            &support.first_parameter,
            &support.second_parameter,
            &parameter,
            policy,
        )? {
            Classification::Decided(sign) => product_sign(sign, support.chord_denominator_sign),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(source_sign))
    }

    pub(crate) fn tangent_cross_vector_sign(
        &self,
        vector: &(Real, Real),
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.tangent_relation_to_vector_sign(vector, true, policy)
    }

    pub(crate) fn tangent_dot_vector_sign(
        &self,
        vector: &(Real, Real),
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.tangent_relation_to_vector_sign(vector, false, policy)
    }

    /// Publishes this oriented affine support as homogeneous line
    /// coefficients `(a, b, c)`. Analytic tangent supports use their direct
    /// differential identity, while other procedural supports retain one
    /// anchor and an undisplaced direction so normalized translations are
    /// never introduced merely to cancel them again.
    pub(super) fn recursive_projective_support_line(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRecursiveQuadraticProjectivePoint2>> {
        if let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (self.start(), self.end())
        {
            match start.recursive_tangent_line_to(end, policy)? {
                Classification::Decided(Some(line)) => return Ok(Some(line)),
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }
        let frame = match self.recursive_projective_endpoints_with_direction(policy)? {
            Classification::Decided(Some(frame)) => frame,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        let [anchor, _] = frame.displaced;
        let [direction_start, direction_end] = frame.direction_endpoints;
        let Some((direction_x, direction_y, _)) =
            direction_end.difference_numerators(&direction_start)
        else {
            return Ok(None);
        };
        Ok((|| {
            Some(BezierRecursiveQuadraticProjectivePoint2 {
                x: direction_y
                    .multiply(&anchor.denominator)?
                    .scale(&Real::from(-1_i8))?,
                y: direction_x.multiply(&anchor.denominator)?,
                denominator: direction_y
                    .multiply(&anchor.x)?
                    .subtract(&direction_x.multiply(&anchor.y)?)?,
            })
        })())
    }

    /// Returns the exact traversal cross sign from the two compact projective
    /// support lines when bounded local arithmetic can decide it. For line
    /// coefficients `(a,b,c)=(-dy,dx,c)`, `a1*b2-b1*a2` is
    /// `cross(tangent1,tangent2)` up to the positive projective scales.
    pub(super) fn recursive_support_tangent_cross_sign(
        &self,
        other: &Self,
        permit_recursive_norm: bool,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        let first_support = self.retained_support();
        let second_support = other.retained_support();
        let (Some(first), Some(second)) = (
            first_support.recursive_projective_support_line(policy)?,
            second_support.recursive_projective_support_line(policy)?,
        ) else {
            return Ok(None);
        };
        let orient = |mut sign| {
            if self.retained_support_orientation_is_reversed()
                ^ other.retained_support_orientation_is_reversed()
            {
                sign = product_sign(sign, RealSign::Negative);
            }
            sign
        };
        let exact = |line: &BezierRecursiveQuadraticProjectivePoint2| {
            Some([
                line.x.exact_real_value_with_retained_witnesses()?,
                line.y.exact_real_value_with_retained_witnesses()?,
            ])
        };
        if let (Some(first), Some(second)) = (exact(&first), exact(&second)) {
            let cross = Real::diff_of_products(&first[0], &second[1], &first[1], &second[0]);
            if let Some(sign) = cross.refine_sign_until(-512) {
                return Ok(Some(orient(sign)));
            }
        }
        let schedule: &[usize] = if policy.has_bounded_exact_predicate_budget() {
            &[0, 8]
        } else {
            &[0, 8, 128, 512]
        };
        let interval_cross = |first: &BezierRecursiveQuadraticProjectivePoint2,
                              second: &BezierRecursiveQuadraticProjectivePoint2,
                              refinement_steps| {
            let first_x = first.x.interval(refinement_steps)?;
            let first_y = first.y.interval(refinement_steps)?;
            let second_x = second.x.interval(refinement_steps)?;
            let second_y = second.y.interval(refinement_steps)?;
            first_x.multiply(&second_y).and_then(|positive| {
                first_y
                    .multiply(&second_x)
                    .map(|negative| positive.subtract(&negative))
            })
        };
        let mut terminal_refined = false;
        for &refinement_steps in schedule {
            if let Some(cross) = interval_cross(&first, &second, refinement_steps) {
                terminal_refined |= refinement_steps == 512;
                if let Some(sign) = dense_strict_interval_sign(&cross) {
                    return Ok(Some(orient(sign)));
                }
            }
        }
        if policy.has_bounded_exact_predicate_budget() {
            return Ok(None);
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Some(RealSign::Zero));
        }
        let direct_join = first.joined_pair(&second, policy)?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            let (first_base, first_extensions) =
                first.denominator.field().base_and_extension_path();
            let (second_base, second_extensions) =
                second.denominator.field().base_and_extension_path();
            let joined = match &direct_join {
                Classification::Decided(Some(_)) => "joined",
                Classification::Decided(None) => "none",
                Classification::Uncertain(_) => "uncertain",
            };
            eprintln!(
                "support tangent cross direct-join={joined} fields=({}+{},{}+{})",
                first_base.sources.len(),
                first_extensions.len(),
                second_base.sources.len(),
                second_extensions.len(),
            );
        }
        let (first, second) = match direct_join {
            Classification::Decided(Some((_, first, second))) => (first, second),
            Classification::Decided(None) | Classification::Uncertain(_) => {
                let field = first.denominator.field();
                let merged = recursive_merge_projective_point_fields(
                    &field,
                    std::slice::from_ref(&first),
                    &second,
                    policy,
                )?;
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                    eprintln!(
                        "support tangent cross source-union={}",
                        match &merged {
                            Classification::Decided(Some(_)) => "joined",
                            Classification::Decided(None) => "none",
                            Classification::Uncertain(_) => "uncertain",
                        }
                    );
                }
                match merged {
                    Classification::Decided(Some((_, mut first, second))) => {
                        let first = first
                            .pop()
                            .expect("one support line enters a two-line field merge");
                        (first, second)
                    }
                    Classification::Decided(None) | Classification::Uncertain(_) => {
                        return Ok(None);
                    }
                }
            }
        };
        let Some(cross) = first.x.multiply(&second.y).and_then(|positive| {
            first
                .y
                .multiply(&second.x)
                .and_then(|negative| positive.subtract(&negative))
        }) else {
            return Ok(None);
        };
        if cross.is_structurally_zero() {
            return Ok(Some(RealSign::Zero));
        }
        let refinement_schedule: &[usize] = if policy.selects_approximate_512() {
            &[0, 8, 128, 512]
        } else {
            // A transverse mixed-field determinant can be far smaller than
            // 2^-512 while still separating cheaply in its retained tower.
            // Exhaust a few exact local enclosures before constructing the
            // much larger global tensor image reserved for equality.
            &[0, 8, 128, 512, 1024, 1664, 2560, 4096]
        };
        for &refinement_steps in refinement_schedule {
            if let Some(interval) = cross.interval(refinement_steps)
                && let Some(sign) = dense_strict_interval_sign(&interval)
            {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                    eprintln!(
                        "support tangent cross merged interval={refinement_steps} sign={sign:?}"
                    );
                }
                return Ok(Some(orient(sign)));
            }
        }
        if let Some(value) = cross.exact_real_value_with_retained_witnesses()
            && let Some(sign) = value.refine_sign_until(-512)
        {
            return Ok(Some(orient(sign)));
        }
        if !permit_recursive_norm {
            return Ok(None);
        }
        let sign = cross.sign(policy)?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!("support tangent cross compact-sign={sign:?}");
        }
        match sign {
            Classification::Decided(sign) => Ok(Some(orient(sign))),
            Classification::Uncertain(_) => Ok(None),
        }
    }

    /// Signs one retained point against the compact support line before the
    /// generic three-point kernel imports two independently materialized
    /// support endpoints. This is especially important for analytic tangent
    /// supports, whose shared speed radical otherwise appears twice.
    pub(super) fn recursive_support_line_oriented_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        if matches!(point, CurvePoint2(CurvePointData2::AlgebraicChordPair(_))) {
            return Ok(None);
        }
        let support = self.retained_support();
        let Some(line) = support.recursive_projective_support_line(policy)? else {
            return Ok(None);
        };
        let point = match recursive_projective_evidence_points(&[point], policy)? {
            Classification::Decided(Some(mut points)) => points
                .pop()
                .expect("one recursive point query retains one projective point"),
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        let reverse = self.retained_support_orientation_is_reversed();
        let side = |mut sign| {
            if reverse {
                sign = product_sign(sign, RealSign::Negative);
            }
            crate::classify::LineSide::from_real_sign(sign)
        };
        let exact = |value: &BezierRecursiveQuadraticProjectivePoint2| {
            Some([
                value.x.exact_real_value_with_retained_witnesses()?,
                value.y.exact_real_value_with_retained_witnesses()?,
                value
                    .denominator
                    .exact_real_value_with_retained_witnesses()?,
            ])
        };
        if let (Some(line), Some(point)) = (exact(&line), exact(&point)) {
            let incidence = Real::signed_product_sum(
                [true, true, true],
                [
                    [&line[0], &point[0]],
                    [&line[1], &point[1]],
                    [&line[2], &point[2]],
                ],
            );
            let minimum_precision = if policy.has_bounded_exact_predicate_budget() {
                -8
            } else {
                -512
            };
            if let Some(sign) = incidence.refine_sign_until(minimum_precision) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "compact-real-support-line-incidence",
                );
                return Ok(Some(side(sign)));
            }
        }
        let schedule: &[usize] = if policy.has_bounded_exact_predicate_budget() {
            &[0, 8]
        } else if policy.permits_approximate_512() {
            &[0, 8, 128, 512]
        } else {
            &[0, 8, 128, 256, 512]
        };
        for &refinement_steps in schedule {
            let interval = |value: &BezierRecursiveQuadraticProjectivePoint2| {
                Some([
                    value.x.interval(refinement_steps)?,
                    value.y.interval(refinement_steps)?,
                    value.denominator.interval(refinement_steps)?,
                ])
            };
            let (Some(line), Some(point)) = (interval(&line), interval(&point)) else {
                continue;
            };
            let incidence = line[0]
                .multiply(&point[0])
                .and_then(|value| line[1].multiply(&point[1]).map(|term| value.add(&term)))
                .and_then(|value| line[2].multiply(&point[2]).map(|term| value.add(&term)));
            if let Some(sign) = incidence.as_ref().and_then(dense_strict_interval_sign) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "recursive-support-line-incidence-interval",
                );
                return Ok(Some(side(sign)));
            }
        }
        if !policy.has_bounded_exact_predicate_budget()
            && let Classification::Decided(Some((_, line, point))) =
                line.joined_pair(&point, policy)?
            && let Some(incidence) = line
                .x
                .multiply(&point.x)
                .and_then(|value| line.y.multiply(&point.y).and_then(|term| value.add(&term)))
                .and_then(|value| {
                    line.denominator
                        .multiply(&point.denominator)
                        .and_then(|term| value.add(&term))
                })
        {
            let compact_real = incidence.exact_real_value_with_retained_witnesses();
            let compact_real_zero = compact_real
                .as_ref()
                .is_some_and(|value| value.zero_status() == ZeroKnowledge::Zero);
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!(
                    "support-line incidence structural-zero={} compact-real-zero={}",
                    incidence.is_structurally_zero(),
                    compact_real_zero,
                );
            }
            if incidence.is_structurally_zero() || compact_real_zero {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "recursive-support-line-structural-incidence",
                );
                return Ok(Some(crate::classify::LineSide::On));
            }
            for refinement_steps in [0, 8, 128, 512] {
                if let Some(interval) = incidence.interval(refinement_steps)
                    && let Some(sign) = dense_strict_interval_sign(&interval)
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-side-kernel",
                        "joined-recursive-support-line-incidence-interval",
                    );
                    return Ok(Some(side(sign)));
                }
            }
            if let Some(value) = compact_real
                && let Some(sign) = value.refine_sign_until(-512)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "bounded-real-joined-support-line-incidence",
                );
                return Ok(Some(side(sign)));
            }
        }
        Ok(None)
    }

    /// Reuses recursive projective point fields for an exact oriented-area
    /// predicate. This is the native authority for nested selected-radial
    /// contacts, whose compact quadratic tower should not be flattened into
    /// independent high-degree coordinate representations.
    pub(super) fn recursive_projective_oriented_side(
        &self,
        point: &CurvePoint2,
        certified_nonzero: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<crate::classify::LineSide>>> {
        if let Some(side) = self.recursive_support_line_oriented_side(point, policy)? {
            return Ok(Classification::Decided(Some(side)));
        }
        recursive_projective_point_evidence_oriented_side(
            self.start(),
            self.end(),
            point,
            certified_nonzero,
            policy,
        )
    }

    /// Cold exact side predicate for retained points whose interval boxes keep
    /// sharing a boundary after full refinement.  All coordinate witnesses
    /// enter one tensor authority, so the oriented area is signed as one
    /// correlated algebraic value instead of comparing independently rounded
    /// endpoint boxes.
    pub(super) fn represented_oriented_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        match self.recursive_projective_oriented_side(point, false, policy)? {
            Classification::Decided(Some(side)) => {
                return Ok(Classification::Decided(side));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        represented_point_evidence_oriented_side(self.start(), self.end(), point, policy)
    }

    /// Classifies a retained point against this oriented support without
    /// adjoining its endpoint fields. The cross product is evaluated over
    /// progressively refined exact boxes. Only APPROXIMATE_512 may turn an
    /// unresolved terminal overlap into equality.
    pub(super) fn oriented_side_by_refinement_with_limit(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
        maximum_refinement_steps: usize,
        recursive_prepass: bool,
        local_bounds_only: bool,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        self.validate_policy(policy)?;
        if [self.start(), self.end()]
            .into_iter()
            .any(|endpoint| endpoint.shares_storage(point))
        {
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        let procedural_point = matches!(
            point,
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        );
        let procedural_support = [self.start(), self.end()].into_iter().all(|endpoint| {
            matches!(
                endpoint,
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                    | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            )
        });
        if recursive_prepass
            && !policy.has_bounded_exact_predicate_budget()
            && procedural_point
            && procedural_support
        {
            match self.recursive_projective_oriented_side(point, false, policy)? {
                Classification::Decided(Some(side)) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-side-kernel",
                        "recursive-projective-prepass",
                    );
                    return Ok(Classification::Decided(side));
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if refinement_steps > maximum_refinement_steps {
                break;
            }
            let bounds = |point| {
                if local_bounds_only {
                    algebraic_chord_endpoint_local_bounds_refined(point, refinement_steps, policy)
                } else {
                    algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
                }
            };
            let (
                Classification::Decided(start),
                Classification::Decided(end),
                Classification::Decided(point),
            ) = (bounds(self.start()), bounds(self.end()), bounds(point))
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let strict = &CurveContext::STRICT;
            let start_x = RealInterval::from_axis(&start, Axis2::X);
            let start_y = RealInterval::from_axis(&start, Axis2::Y);
            let delta_x = RealInterval::from_axis(&end, Axis2::X).subtract(&start_x);
            let delta_y = RealInterval::from_axis(&end, Axis2::Y).subtract(&start_y);
            let point_x = RealInterval::from_axis(&point, Axis2::X).subtract(&start_x);
            let point_y = RealInterval::from_axis(&point, Axis2::Y).subtract(&start_y);
            let Some(cross) = delta_x.multiply(&point_y).and_then(|first| {
                delta_y
                    .multiply(&point_x)
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            if compare_reals(&cross.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::Left));
            }
            if compare_reals(&cross.upper, &Real::zero(), strict) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::Right));
            }
            if compare_reals(&cross.lower, &Real::zero(), strict) == Some(std::cmp::Ordering::Equal)
                && compare_reals(&cross.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::On));
            }
        }
        // Independently rebuilt descendants can still be an exact support
        // endpoint without sharing its allocation. This compact equality
        // certificate is cheaper and more specific than joining all three
        // point fields, and remains valid under an approximate object's
        // retained policy identity.
        if terminal_refined && policy.selects_approximate_512() {
            for endpoint in [self.start(), self.end()] {
                let same = policy.strict_predicate_pass(|| {
                    endpoint.same_point(point, policy) == Classification::Decided(true)
                        || point.same_point(endpoint, policy) == Classification::Decided(true)
                });
                if same {
                    return Ok(Classification::Decided(crate::classify::LineSide::On));
                }
            }
        }
        if recursive_prepass
            && !policy.has_bounded_exact_predicate_budget()
            && maximum_refinement_steps >= 512
        {
            match self.represented_oriented_side(point, policy)? {
                Classification::Decided(side) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-side-kernel",
                        "represented-cold-fallback",
                    );
                    return Ok(Classification::Decided(side));
                }
                Classification::Uncertain(_) => {}
            }
            // Equality is a cold residual case. Running two independent
            // endpoint comparisons before the correlated cross predicate
            // makes every ordinary interior query pay for unrelated selected
            // fields. Retain the exact fallback here for endpoint evidence
            // that cannot enter the represented oriented-area kernel.
            for endpoint in [self.start(), self.end()] {
                if policy.strict_predicate_pass(|| endpoint.same_point(point, policy))
                    == Classification::Decided(true)
                {
                    return Ok(Classification::Decided(crate::classify::LineSide::On));
                }
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    }

    pub(super) fn oriented_side_by_refinement(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        if policy.has_bounded_exact_predicate_budget() {
            return self.oriented_side_by_refinement_with_limit(point, policy, 8, true, true);
        }
        let bounded = policy.bounded_exact_predicate_pass(|| {
            self.oriented_side_by_refinement_with_limit(point, policy, 8, true, true)
        })?;
        if matches!(bounded, Classification::Decided(_)) {
            return Ok(bounded);
        }
        self.oriented_side_by_refinement_with_limit(point, policy, 512, true, false)
    }

    pub(crate) fn strict_oriented_side_by_fast_refinement(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        self.oriented_side_by_refinement_with_limit(point, policy, 8, true, false)
    }

    /// Applies the same exact interval-side test using construction-local
    /// endpoint boxes only. A composite point that lacks such a box declines
    /// immediately instead of materializing its recursive field tower for a
    /// speculative Boolean fast path.
    pub(crate) fn strict_oriented_side_by_local_interval_refinement(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        self.oriented_side_by_refinement_with_limit(point, policy, 512, false, true)
    }

    /// Classifies all four endpoint/support relations for two general retained
    /// chords in one refinement pass.
    ///
    /// Calling the scalar fallback four times repeats both endpoint equality
    /// and the same support enclosures. Unary regularization is dominated by
    /// these all-pairs tests, so share each exact enclosure while preserving
    /// identical STRICT and APPROXIMATE_512 terminal behavior.
    pub(super) fn pair_sides_by_refinement(
        &self,
        other: &Self,
        retain_structural_incidence: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordPairSides2>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        if !policy.has_bounded_exact_predicate_budget() {
            let bounded = policy.bounded_exact_predicate_pass(|| {
                self.pair_sides_by_refinement(other, retain_structural_incidence, policy)
            })?;
            if matches!(bounded, Classification::Decided(_)) {
                return Ok(bounded);
            }
        }
        let first_points = [self.start(), self.end()];
        let second_points = [other.start(), other.end()];
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            && first_points
                .iter()
                .chain(second_points.iter())
                .any(|point| matches!(point, CurvePoint2(CurvePointData2::AlgebraicChordPair(_))))
        {
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            let describe = |chord: &BezierAlgebraicChord2| {
                let support = chord.retained_support();
                format!(
                    "({},{})/support=({},{})/same={}/rev={}/axis={:?}/unit={}",
                    kind(chord.start()),
                    kind(chord.end()),
                    kind(support.start()),
                    kind(support.end()),
                    Arc::ptr_eq(&chord.data, &support.data),
                    chord.retained_support_orientation_is_reversed(),
                    chord.certified_axis_direction(),
                    chord.certified_unit_tangent().is_some(),
                )
            };
            let relation = |first: &BezierAlgebraicChord2, second: &BezierAlgebraicChord2| {
                let parallel = chord_parallel_support_source(first, policy)
                    .ok()
                    .flatten()
                    .zip(chord_parallel_support_source(second, policy).ok().flatten())
                    .map(|(first, second)| {
                        format!(
                            "parallel(dir={},source={},orientation={:?},distance={:?},tx={:?},ty={:?})",
                            first.direction == second.direction,
                            first.source.shares_retained_support(&second.source),
                            first.source.shared_tangent_orientation(&second.source),
                            compare_reals(&first.distance, &second.distance, &CurveContext::STRICT),
                            compare_reals(
                                &first.translation_x,
                                &second.translation_x,
                                &CurveContext::STRICT,
                            ),
                            compare_reals(
                                &first.translation_y,
                                &second.translation_y,
                                &CurveContext::STRICT,
                            ),
                        )
                    })
                    .unwrap_or_else(|| "parallel(n/a)".into());
                format!(
                    "{parallel}/tangent={:?}/normal={:?}",
                    first.shared_tangent_orientation(second),
                    first.retained_normal_offset_distance_to(second),
                )
            };
            eprintln!(
                "pair sides retain={retain_structural_incidence} first={} second={}",
                describe(self),
                describe(other),
            );
            for (owner, points, query) in [
                ("first", first_points, other),
                ("second", second_points, self),
            ] {
                for (index, point) in points.into_iter().enumerate() {
                    let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                        continue;
                    };
                    eprintln!(
                        "pair endpoint owner={owner} index={index} a={} b={} query-a={} query-b={} rel-a={} rel-b={} location={:?}",
                        describe(&point.data.first),
                        describe(&point.data.second),
                        query.shares_retained_support(&point.data.first),
                        query.shares_retained_support(&point.data.second),
                        relation(query, &point.data.first),
                        relation(query, &point.data.second),
                        point.data.location,
                    );
                    for (endpoint_index, endpoint) in [
                        query.retained_support().start(),
                        query.retained_support().end(),
                    ]
                    .into_iter()
                    .enumerate()
                    {
                        let CurvePoint2(CurvePointData2::AlgebraicChordParallel(endpoint)) =
                            endpoint
                        else {
                            continue;
                        };
                        let source_point = endpoint.data.source_point.as_deref();
                        let anchors = match point.data.location {
                            BezierAlgebraicChordPairPointLocation2::AnchorOrders {
                                first_at_end,
                                second_at_end,
                                ..
                            } => [
                                if first_at_end {
                                    point.data.first.end()
                                } else {
                                    point.data.first.start()
                                },
                                if second_at_end {
                                    point.data.second.end()
                                } else {
                                    point.data.second.start()
                                },
                            ],
                            BezierAlgebraicChordPairPointLocation2::EndpointSides { .. } => {
                                [point.data.first.start(), point.data.second.start()]
                            }
                        };
                        let endpoint_evidence = CurvePoint2::from(endpoint.clone());
                        eprintln!(
                            "pair query endpoint={endpoint_index} parallel origin={} origin-pair={} dir={:?} distance={:?} tx-zero={:?} ty-zero={:?} source-a={} source-b={} source={} anchor-storage={:?} anchor-eq={:?} source-anchor-storage={:?} source-anchor-eq={:?}",
                            source_point.map(kind).unwrap_or("endpoint"),
                            matches!(
                                source_point,
                                Some(CurvePoint2(CurvePointData2::AlgebraicChordPair(
                                    origin,
                                ))) if point == origin
                            ),
                            endpoint.data.direction,
                            real_sign(&endpoint.data.distance, &CurveContext::STRICT),
                            endpoint.data.translation_x.zero_status(),
                            endpoint.data.translation_y.zero_status(),
                            endpoint
                                .data
                                .source
                                .shares_retained_support(&point.data.first),
                            endpoint
                                .data
                                .source
                                .shares_retained_support(&point.data.second),
                            describe(&endpoint.data.source),
                            anchors.map(|anchor| endpoint_evidence.shares_storage(anchor)),
                            anchors.map(|anchor| endpoint_evidence == *anchor),
                            anchors.map(|anchor| endpoint.source_endpoint().shares_storage(anchor)),
                            anchors.map(|anchor| endpoint.source_endpoint() == anchor),
                        );
                    }
                }
            }
        }
        let mut equal = [[false; 2]; 2];
        for (first_index, first) in first_points.iter().enumerate() {
            for (second_index, second) in second_points.iter().enumerate() {
                equal[first_index][second_index] = first.shares_storage(second);
            }
        }

        let mut first_sides = [None; 2];
        let mut second_sides = [None; 2];
        for index in 0..2 {
            if equal[index][0] || equal[index][1] {
                first_sides[index] = Some(crate::classify::LineSide::On);
            }
            if equal[0][index] || equal[1][index] {
                second_sides[index] = Some(crate::classify::LineSide::On);
            }
        }
        // A retained miter endpoint is the authored intersection of two
        // support lines. Preserve that incidence in the batched endpoint
        // kernel instead of asking interval boxes to rediscover an exact
        // zero. The remaining endpoint sides then decide whether the unique
        // support intersection lies inside both finite chords.
        if retain_structural_incidence {
            for (index, point) in first_points.iter().enumerate() {
                if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point
                    && (other.shares_retained_support(&point.data.first)
                        || other.shares_retained_support(&point.data.second))
                {
                    first_sides[index] = Some(crate::classify::LineSide::On);
                }
            }
            for (index, point) in second_points.iter().enumerate() {
                if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point
                    && (self.shares_retained_support(&point.data.first)
                        || self.shares_retained_support(&point.data.second))
                {
                    second_sides[index] = Some(crate::classify::LineSide::On);
                }
            }
        }

        // The scalar support-predicate path owns exact correlated authorities
        // for analytic-tangent and procedural normal-offset endpoints. Seed
        // those same facts before the batched interval pass; choosing the
        // batch because one support lacks an algebraic ray must not bypass
        // construction evidence carried by the individual endpoints.
        for index in 0..2 {
            if first_sides[index].is_none()
                && self.certified_axis_direction().is_none()
                && other.certified_axis_direction().is_none()
                && let Some(side) =
                    other.retained_procedural_point_side(first_points[index], policy)?
            {
                first_sides[index] = Some(side);
            }
            if second_sides[index].is_none()
                && self.certified_axis_direction().is_none()
                && other.certified_axis_direction().is_none()
                && let Some(side) =
                    self.retained_procedural_point_side(second_points[index], policy)?
            {
                second_sides[index] = Some(side);
            }
        }

        let decided_sides =
            |first_sides: &[Option<crate::classify::LineSide>; 2],
             second_sides: &[Option<crate::classify::LineSide>; 2]| {
                let on = crate::classify::LineSide::On;
                let strictly_one_sided = |sides: &[Option<crate::classify::LineSide>; 2]| {
                    matches!(
                        sides,
                        [
                            Some(crate::classify::LineSide::Left),
                            Some(crate::classify::LineSide::Left)
                        ] | [
                            Some(crate::classify::LineSide::Right),
                            Some(crate::classify::LineSide::Right)
                        ]
                    )
                };
                if strictly_one_sided(first_sides) || strictly_one_sided(second_sides) {
                    return Some(BezierAlgebraicChordPairSides2::Disjoint);
                }
                if first_sides.iter().all(|side| *side == Some(on))
                    || second_sides.iter().all(|side| *side == Some(on))
                {
                    // Either nonzero chord contributes two distinct points
                    // to the other affine support, which proves that the two
                    // supports are identical. Do not ask a fourth selected
                    // endpoint predicate to rediscover the same line equality.
                    return Some(BezierAlgebraicChordPairSides2::Complete([on; 2], [on; 2]));
                }
                (first_sides.iter().all(Option::is_some)
                    && second_sides.iter().all(Option::is_some))
                .then(|| {
                    BezierAlgebraicChordPairSides2::Complete(
                        first_sides.map(Option::unwrap),
                        second_sides.map(Option::unwrap),
                    )
                })
            };

        // A nonincident retained pair endpoint can still classify itself
        // against the opposite support from its certified offset-anchor
        // orders. Consume that compact affine certificate before the shared
        // interval loop asks for the pair's Cartesian intersection box.
        let merge_sides = |target: &mut [Option<crate::classify::LineSide>; 2],
                           source: [crate::classify::LineSide; 2]|
         -> CurveResult<()> {
            for (target, source) in target.iter_mut().zip(source) {
                if target.is_some_and(|target| target != source) {
                    return Err(CurveError::Topology(
                        "retained chord-pair endpoint incidences conflicted".into(),
                    ));
                }
                *target = Some(source);
            }
            Ok(())
        };
        let owner_incidence_sides = |owner: &BezierAlgebraicChord2,
                                     point: &BezierAlgebraicChordPairPoint2,
                                     defining_sides: [[crate::classify::LineSide; 2]; 2]|
         -> Option<[crate::classify::LineSide; 2]> {
            for (support, sides) in [
                (&point.data.first, defining_sides[0]),
                (&point.data.second, defining_sides[1]),
            ] {
                if !owner.shares_retained_support(support) {
                    continue;
                }
                let reversed = support.shared_tangent_orientation(owner)?;
                return Some(if reversed {
                    sides.map(|side| match side {
                        crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                        crate::classify::LineSide::On => crate::classify::LineSide::On,
                        crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                    })
                } else {
                    sides
                });
            }
            None
        };
        if !policy.has_bounded_exact_predicate_budget() {
            for (index, point) in first_points.iter().enumerate() {
                let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                    continue;
                };
                if first_sides[index].is_some() {
                    continue;
                }
                if other.certified_axis_direction().is_some() {
                    continue;
                }
                if let Some((side, defining_sides)) =
                    point.endpoint_incidence_oriented_side_to_chord(other, policy)?
                {
                    if let Some(sides) = owner_incidence_sides(self, point, defining_sides) {
                        merge_sides(&mut second_sides, sides)?;
                    }
                    if let Some(side) = side {
                        if first_sides[index].is_some_and(|existing| existing != side) {
                            return Err(CurveError::Topology(
                                "retained chord-pair point-side certificates conflicted".into(),
                            ));
                        }
                        first_sides[index] = Some(side);
                    }
                }
            }
            for (index, point) in second_points.iter().enumerate() {
                let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                    continue;
                };
                if second_sides[index].is_some() {
                    continue;
                }
                if self.certified_axis_direction().is_some() {
                    continue;
                }
                if let Some((side, defining_sides)) =
                    point.endpoint_incidence_oriented_side_to_chord(self, policy)?
                {
                    if let Some(sides) = owner_incidence_sides(other, point, defining_sides) {
                        merge_sides(&mut first_sides, sides)?;
                    }
                    if let Some(side) = side {
                        if second_sides[index].is_some_and(|existing| existing != side) {
                            return Err(CurveError::Topology(
                                "retained chord-pair point-side certificates conflicted".into(),
                            ));
                        }
                        second_sides[index] = Some(side);
                    }
                }
            }
        }

        let complete_shared_endpoint_sides = |first_sides: &mut [Option<crate::classify::LineSide>;
                                                       2],
                                              second_sides: &mut [Option<crate::classify::LineSide>;
                                                       2],
                                              equal: &[[bool; 2]; 2]|
         -> CurveResult<bool> {
            let Some((first_index, second_index)) =
                equal.iter().enumerate().find_map(|(first_index, row)| {
                    row.iter()
                        .position(|is_equal| *is_equal)
                        .map(|second_index| (first_index, second_index))
                })
            else {
                return Ok(false);
            };
            if first_sides[1 - first_index].is_some() && second_sides[1 - second_index].is_some() {
                return Ok(false);
            }
            let Classification::Decided(tangent_cross) =
                policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))?
            else {
                return Ok(false);
            };
            let oriented_side = |reverse: bool| {
                crate::classify::LineSide::from_real_sign(if reverse {
                    product_sign(tangent_cross, RealSign::Negative)
                } else {
                    tangent_cross
                })
            };
            first_sides[1 - first_index] = Some(oriented_side(first_index == 0));
            second_sides[1 - second_index] = Some(oriented_side(second_index == 1));
            Ok(true)
        };
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            return Ok(Classification::Decided(sides));
        }

        // A retained chord-pair endpoint is the exact intersection of the
        // current support and the support carried by its incident chord.  If
        // the two current tangents are transverse, that endpoint is therefore
        // the unique support intersection.  Order it against the opposite
        // finite endpoints in the already-retained chord parameter instead of
        // refining the correlated Cartesian point together with three
        // unrelated endpoint fields.
        for (second_index, point) in second_points.iter().enumerate() {
            let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                continue;
            };
            if !self.shares_retained_support(&point.data.first)
                && !self.shares_retained_support(&point.data.second)
            {
                continue;
            }
            let Classification::Decided(cross @ (RealSign::Positive | RealSign::Negative)) =
                policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))?
            else {
                continue;
            };
            second_sides[second_index] = Some(crate::classify::LineSide::On);
            second_sides[1 - second_index] = Some(crate::classify::LineSide::from_real_sign(
                if second_index == 0 {
                    cross
                } else {
                    product_sign(cross, RealSign::Negative)
                },
            ));
            for (first_index, endpoint) in first_points.iter().enumerate() {
                let order = match point.cmp_on_chord_to_evidence(self, endpoint, policy)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(_) => continue,
                };
                first_sides[first_index] = Some(match order {
                    std::cmp::Ordering::Equal => crate::classify::LineSide::On,
                    std::cmp::Ordering::Less => crate::classify::LineSide::from_real_sign(
                        product_sign(cross, RealSign::Negative),
                    ),
                    std::cmp::Ordering::Greater => crate::classify::LineSide::from_real_sign(cross),
                });
            }
            if let Some(sides) = decided_sides(&first_sides, &second_sides) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "retained-intersection-endpoint-order",
                );
                return Ok(Classification::Decided(sides));
            }
        }
        for (first_index, point) in first_points.iter().enumerate() {
            let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                continue;
            };
            if !other.shares_retained_support(&point.data.first)
                && !other.shares_retained_support(&point.data.second)
            {
                continue;
            }
            let Classification::Decided(cross @ (RealSign::Positive | RealSign::Negative)) =
                policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))?
            else {
                continue;
            };
            first_sides[first_index] = Some(crate::classify::LineSide::On);
            first_sides[1 - first_index] = Some(crate::classify::LineSide::from_real_sign(
                if first_index == 0 {
                    product_sign(cross, RealSign::Negative)
                } else {
                    cross
                },
            ));
            for (second_index, endpoint) in second_points.iter().enumerate() {
                let order = match point.cmp_on_chord_to_evidence(other, endpoint, policy)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(_) => continue,
                };
                second_sides[second_index] = Some(match order {
                    std::cmp::Ordering::Equal => crate::classify::LineSide::On,
                    std::cmp::Ordering::Less => crate::classify::LineSide::from_real_sign(cross),
                    std::cmp::Ordering::Greater => crate::classify::LineSide::from_real_sign(
                        product_sign(cross, RealSign::Negative),
                    ),
                });
            }
            if let Some(sides) = decided_sides(&first_sides, &second_sides) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "retained-intersection-endpoint-order",
                );
                return Ok(Classification::Decided(sides));
            }
        }

        // When the two finite chords retain the same endpoint allocation,
        // their remaining endpoint sides are exactly the sign of the two
        // traversal tangents (with the appropriate endpoint orientation).
        // Reuse that two-vector predicate before adjoining the shared point
        // and both opposite endpoints into a generic three-point field.
        if complete_shared_endpoint_sides(&mut first_sides, &mut second_sides, &equal)?
            && let Some(sides) = decided_sides(&first_sides, &second_sides)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "shared-endpoint-tangent-cross",
            );
            return Ok(Classification::Decided(sides));
        }

        // A pair endpoint is natively the intersection of two retained
        // supports. Sign that three-line determinant before the batched
        // Cartesian loop asks four independently selected endpoint towers to
        // coexist. The bounded preliminary pass deliberately skips this cold
        // exact authority.
        if !policy.has_bounded_exact_predicate_budget() {
            for index in 0..2 {
                if first_sides[index].is_none()
                    && let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) =
                        first_points[index]
                {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        eprintln!("pair precedence first={index} begin");
                    }
                    if let Classification::Decided(side) =
                        point.oriented_side_to_chord(other, policy)?
                    {
                        first_sides[index] = Some(side);
                    }
                }
                if second_sides[index].is_none()
                    && let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) =
                        second_points[index]
                {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        eprintln!("pair precedence second={index} begin");
                    }
                    if let Classification::Decided(side) =
                        point.oriented_side_to_chord(self, policy)?
                    {
                        second_sides[index] = Some(side);
                    }
                }
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "retained-pair-precedence",
            );
            return Ok(Classification::Decided(sides));
        }

        // Publish each remaining ordinary endpoint directly into the other
        // support's compact projective line before requesting four Cartesian
        // endpoint boxes. A retained pair point already consumed its native
        // three-line determinant above; every other point needs only one
        // line-point dot product, preserving its correlated quadratic tower.
        for index in 0..2 {
            if first_sides[index].is_none()
                && !matches!(
                    first_points[index],
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                )
                && let Some(side) =
                    other.recursive_support_line_oriented_side(first_points[index], policy)?
            {
                first_sides[index] = Some(side);
            }
            if second_sides[index].is_none()
                && !matches!(
                    second_points[index],
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                )
                && let Some(side) =
                    self.recursive_support_line_oriented_side(second_points[index], policy)?
            {
                second_sides[index] = Some(side);
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "compact-support-line-endpoint-seeding",
            );
            return Ok(Classification::Decided(sides));
        }

        // Incidence with an affine support changes linearly along the other
        // chord. If a known endpoint side agrees with that exact derivative,
        // the unknown endpoint cannot cross zero; an `On` endpoint takes the
        // derivative side immediately. This completes many retained offset
        // pairs without comparing two nearly cancelling endpoint values.
        let mut recursive_support_cross =
            self.recursive_support_tangent_cross_sign(other, false, policy)?;
        if recursive_support_cross.is_none()
            && !policy.has_bounded_exact_predicate_budget()
            && let Some(sign) = self.retained_tangent_cross_sign(other, policy)
        {
            recursive_support_cross = match sign? {
                Classification::Decided(sign) => Some(sign),
                Classification::Uncertain(_) => None,
            };
        }
        if recursive_support_cross.is_none() && !policy.has_bounded_exact_predicate_budget() {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                eprintln!("pair tangent cross entering recursive norm");
            }
            recursive_support_cross =
                self.recursive_support_tangent_cross_sign(other, true, policy)?;
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            let first_support = self.retained_support();
            let second_support = other.retained_support();
            let (first_authority, first_authority_reversed) = self.tangent_authority();
            let (second_authority, second_authority_reversed) = other.tangent_authority();
            eprintln!(
                "pair affine propagation first=({},{})/support=({},{})/rev={}/authority=({},{})/rev={} second=({},{})/support=({},{})/rev={}/authority=({},{})/rev={} sides={first_sides:?}/{second_sides:?} cross={recursive_support_cross:?}",
                kind(self.start()),
                kind(self.end()),
                kind(first_support.start()),
                kind(first_support.end()),
                self.retained_support_orientation_is_reversed(),
                kind(first_authority.start()),
                kind(first_authority.end()),
                first_authority_reversed,
                kind(other.start()),
                kind(other.end()),
                kind(second_support.start()),
                kind(second_support.end()),
                other.retained_support_orientation_is_reversed(),
                kind(second_authority.start()),
                kind(second_authority.end()),
                second_authority_reversed,
            );
        }
        if let Some(cross @ (RealSign::Positive | RealSign::Negative)) = recursive_support_cross {
            let cross_side = crate::classify::LineSide::from_real_sign(cross);
            let opposite = |side| match side {
                crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                crate::classify::LineSide::On => crate::classify::LineSide::On,
                crate::classify::LineSide::Right => crate::classify::LineSide::Left,
            };
            let propagate = |sides: &mut [Option<crate::classify::LineSide>; 2], delta_side| {
                if sides[1].is_none()
                    && let Some(start) = sides[0]
                    && (start == crate::classify::LineSide::On || start == delta_side)
                {
                    sides[1] = Some(if start == crate::classify::LineSide::On {
                        delta_side
                    } else {
                        start
                    });
                }
                let backward_side = opposite(delta_side);
                if sides[0].is_none()
                    && let Some(end) = sides[1]
                    && (end == crate::classify::LineSide::On || end == backward_side)
                {
                    sides[0] = Some(if end == crate::classify::LineSide::On {
                        backward_side
                    } else {
                        end
                    });
                }
            };
            // `side(other, self(t))` has derivative `-cross(self, other)`;
            // `side(self, other(t))` has derivative `cross(self, other)`.
            propagate(&mut first_sides, opposite(cross_side));
            propagate(&mut second_sides, cross_side);
            if let Some(sides) = decided_sides(&first_sides, &second_sides) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "affine-side-monotonicity",
                );
                return Ok(Classification::Decided(sides));
            }
        }

        if policy.has_bounded_exact_predicate_budget()
            && first_points
                .iter()
                .zip(first_sides)
                .chain(second_points.iter().zip(second_sides))
                .any(|(point, side)| {
                    side.is_none()
                        && matches!(point, CurvePoint2(CurvePointData2::AlgebraicChordPair(_)))
                })
        {
            // A pair point's bounded native determinant and all ordinary
            // line-point incidences have already run above. Cartesian boxes
            // for that support intersection require the very field promotion
            // this speculative pass is meant to defer, so yield directly to
            // the complete STRICT or APPROXIMATE_512 terminal pass.
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }

        let strict = &CurveContext::STRICT;
        let zero = Real::zero();
        let check_endpoint_equalities = |first_sides: &mut [Option<crate::classify::LineSide>;
                                                  2],
                                         second_sides: &mut [Option<crate::classify::LineSide>;
                                                  2],
                                         equal: &mut [[bool; 2]; 2]|
         -> CurveResult<()> {
            for (first_index, first) in first_points.iter().enumerate() {
                for (second_index, second) in second_points.iter().enumerate() {
                    if first_sides[first_index].is_some() && second_sides[second_index].is_some() {
                        continue;
                    }
                    // Either strict side already proves that this particular
                    // endpoint pair is distinct. Do not let an APPROXIMATE_512
                    // equality terminal contradict stronger exact evidence.
                    if first_sides[first_index]
                        .is_some_and(|side| side != crate::classify::LineSide::On)
                        || second_sides[second_index]
                            .is_some_and(|side| side != crate::classify::LineSide::On)
                    {
                        continue;
                    }
                    let is_equal = match first.same_point(second, policy) {
                        Classification::Decided(is_equal) => is_equal,
                        Classification::Uncertain(_) => {
                            second.same_point(first, policy) == Classification::Decided(true)
                        }
                    };
                    if !is_equal {
                        continue;
                    }
                    // Only certified equality may become the shared-endpoint
                    // premise for the tangent theorem below. An
                    // APPROXIMATE_512 terminal may finish this side predicate
                    // as `On`, but must not become reusable construction
                    // evidence for the opposite endpoints.
                    if !policy.permits_approximate_512() {
                        equal[first_index][second_index] = true;
                    }
                    for side in [
                        &mut first_sides[first_index],
                        &mut second_sides[second_index],
                    ] {
                        if side.is_some_and(|side| side != crate::classify::LineSide::On) {
                            return Err(CurveError::Topology(
                                "exact chord side and endpoint-equality certificates conflict"
                                    .into(),
                            ));
                        }
                        *side = Some(crate::classify::LineSide::On);
                    }
                }
            }
            Ok(())
        };
        let interval_side = |start: &Aabb2, end: &Aabb2, point: &Aabb2| {
            let start_x = RealInterval::from_axis(start, Axis2::X);
            let start_y = RealInterval::from_axis(start, Axis2::Y);
            let delta_x = RealInterval::from_axis(end, Axis2::X).subtract(&start_x);
            let delta_y = RealInterval::from_axis(end, Axis2::Y).subtract(&start_y);
            let point_x = RealInterval::from_axis(point, Axis2::X).subtract(&start_x);
            let point_y = RealInterval::from_axis(point, Axis2::Y).subtract(&start_y);
            let cross = delta_x.multiply(&point_y).and_then(|first| {
                delta_y
                    .multiply(&point_x)
                    .map(|second| first.subtract(&second))
            })?;
            if compare_reals(&cross.lower, &zero, strict) == Some(std::cmp::Ordering::Greater) {
                return Some(crate::classify::LineSide::Left);
            }
            if compare_reals(&cross.upper, &zero, strict) == Some(std::cmp::Ordering::Less) {
                return Some(crate::classify::LineSide::Right);
            }
            (compare_reals(&cross.lower, &zero, strict) == Some(std::cmp::Ordering::Equal)
                && compare_reals(&cross.upper, &zero, strict) == Some(std::cmp::Ordering::Equal))
            .then_some(crate::classify::LineSide::On)
        };

        let mut terminal_refined = false;
        let endpoint_bounds = |point, refinement_steps| {
            if policy.has_bounded_exact_predicate_budget() {
                // A correlated support intersection may need four divergent
                // endpoint fields to publish Cartesian coordinates.  The
                // batched APPROXIMATE_512 pass needs only construction-local
                // boxes here; unresolved pair endpoints are signed below by
                // the native flat three-support determinant.  STRICT retains
                // the complete nonlocal recursive fallback.
                algebraic_chord_endpoint_local_bounds_refined(point, refinement_steps, policy)
            } else {
                algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
            }
        };
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
                break;
            }
            let (
                Classification::Decided(first_start),
                Classification::Decided(first_end),
                Classification::Decided(second_start),
                Classification::Decided(second_end),
            ) = (
                endpoint_bounds(self.start(), refinement_steps),
                endpoint_bounds(self.end(), refinement_steps),
                endpoint_bounds(other.start(), refinement_steps),
                endpoint_bounds(other.end(), refinement_steps),
            )
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            if let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) = (
                first_start.union(&first_end),
                second_start.union(&second_end),
            ) && first_bounds.overlaps(&second_bounds, &CurveContext::STRICT)
                == Classification::Decided(false)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "refined-chord-box-disjointness",
                );
                return Ok(Classification::Decided(
                    BezierAlgebraicChordPairSides2::Disjoint,
                ));
            }
            let first_bounds = [&first_start, &first_end];
            let second_bounds = [&second_start, &second_end];
            for index in 0..2 {
                if first_sides[index].is_none() {
                    first_sides[index] =
                        interval_side(&second_start, &second_end, first_bounds[index]);
                }
                if second_sides[index].is_none() {
                    second_sides[index] =
                        interval_side(&first_start, &first_end, second_bounds[index]);
                }
            }
            if let Some(sides) = decided_sides(&first_sides, &second_sides) {
                return Ok(Classification::Decided(sides));
            }
        }
        if complete_shared_endpoint_sides(&mut first_sides, &mut second_sides, &equal)?
            && let Some(sides) = decided_sides(&first_sides, &second_sides)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "shared-endpoint-tangent-cross",
            );
            return Ok(Classification::Decided(sides));
        }
        // Pair intersections have already consumed their flat three-line
        // authority above. Remaining endpoints may now use the exact
        // one-coordinate predicate of a retained cardinal support without
        // constructing a general oriented-area compositum.
        for index in 0..2 {
            if first_sides[index].is_none()
                && !matches!(
                    first_points[index],
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                )
                && let Some(direction) = other.certified_axis_direction()
                && let Some(Classification::Decided(side)) =
                    other.axis_oriented_side(first_points[index], direction, policy)
            {
                first_sides[index] = Some(side);
            }
            if second_sides[index].is_none()
                && !matches!(
                    second_points[index],
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                )
                && let Some(direction) = self.certified_axis_direction()
                && let Some(Classification::Decided(side)) =
                    self.axis_oriented_side(second_points[index], direction, policy)
            {
                second_sides[index] = Some(side);
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "cardinal-coordinate-terminal",
            );
            return Ok(Classification::Decided(sides));
        }
        // The batched box pass requires all four endpoints to expose boxes in
        // the same iteration. A single retained pair point can therefore hide
        // an otherwise complete 512-bit scalar refinement for a procedural
        // endpoint. Finish each remaining oriented area independently before
        // considering a multi-field represented compositum.
        if !policy.has_bounded_exact_predicate_budget() {
            for index in 0..2 {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some()
                    && (first_sides[index].is_none() || second_sides[index].is_none())
                {
                    eprintln!(
                        "pair scalar refinement index={index} before={first_sides:?}/{second_sides:?}"
                    );
                }
                if first_sides[index].is_none()
                    && let Classification::Decided(side) =
                        other.oriented_side_by_refinement(first_points[index], policy)?
                {
                    first_sides[index] = Some(side);
                }
                if second_sides[index].is_none()
                    && let Classification::Decided(side) =
                        self.oriented_side_by_refinement(second_points[index], policy)?
                {
                    second_sides[index] = Some(side);
                }
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!(
                    "pair sides scalar terminal first={first_sides:?} second={second_sides:?}"
                );
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "scalar-refinement-terminal",
            );
            return Ok(Classification::Decided(sides));
        }
        if !policy.has_bounded_exact_predicate_budget() && !policy.selects_approximate_512() {
            check_endpoint_equalities(&mut first_sides, &mut second_sides, &mut equal)?;
            if complete_shared_endpoint_sides(&mut first_sides, &mut second_sides, &equal)?
                && let Some(sides) = decided_sides(&first_sides, &second_sides)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "exact-endpoint-equality-terminal",
                );
                return Ok(Classification::Decided(sides));
            }
        }
        // Interval boxes cannot prove every nonzero side when a retained
        // endpoint is an affine image of a selected-circle contact.  At the
        // terminal predicate stage, sign each remaining oriented area in one
        // correlated represented tensor. STRICT keeps its complete exact
        // authority; the full APPROXIMATE_512 replay reaches the same scalar
        // only after the operation-wide strict pass has declined, and may
        // therefore consume its 512-bit equality terminal even when no
        // Cartesian endpoint box could be constructed.
        if !policy.has_bounded_exact_predicate_budget()
            && (!policy.selects_approximate_512() || policy.permits_approximate_512())
        {
            for index in 0..2 {
                if first_sides[index].is_none()
                    && let Classification::Decided(side) =
                        other.represented_oriented_side(first_points[index], policy)?
                {
                    first_sides[index] = Some(side);
                }
                if second_sides[index].is_none()
                    && let Classification::Decided(side) =
                        self.represented_oriented_side(second_points[index], policy)?
                {
                    second_sides[index] = Some(side);
                }
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "represented-cold-fallback",
            );
            return Ok(Classification::Decided(sides));
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!(
                "pair sides terminal first={first_sides:?} second={second_sides:?} equal={equal:?} refined={terminal_refined} selected={} permits={}",
                policy.selects_approximate_512(),
                policy.permits_approximate_512(),
            );
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(
                BezierAlgebraicChordPairSides2::Complete(
                    first_sides.map(|side| side.unwrap_or(crate::classify::LineSide::On)),
                    second_sides.map(|side| side.unwrap_or(crate::classify::LineSide::On)),
                ),
            ))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    }
}

impl BezierAlgebraicChord2 {
    /// Certifies that `point` is `center` displaced along this chord's unit
    /// left normal and returns the displacement's authored signed distance.
    ///
    /// A retained chord-parallel endpoint already owns that construction
    /// fact. Reusing its distance is essential when boundary orientation makes
    /// the signed parallel distance differ from a caller's unsigned radius.
    /// Other exact point carriers are checked by reconstructing the supplied
    /// fallback displacement. If traversal reversal changed the chord's left
    /// normal, the opposite sign is checked as well and the exactly matching
    /// signed distance is returned.
    pub(super) fn certified_normal_displacement_distance(
        &self,
        center: &CurvePoint2,
        point: &CurvePoint2,
        fallback_distance: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        self.validate_policy(policy)?;
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "chord-normal-displacement-point-kind",
            match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "chord-pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp-chord",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "cusp-chord-derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "chord-parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic-parallel",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            },
        );
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "chord-normal-displacement-center-kind",
            match center {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "chord-pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp-chord",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "cusp-chord-derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "chord-parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic-parallel",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            },
        );
        if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(displaced)) = point
            && displaced.accepts_policy(policy)
            && displaced.data.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal
            && displaced.data.translation_x.zero_status() == ZeroKnowledge::Zero
            && displaced.data.translation_y.zero_status() == ZeroKnowledge::Zero
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "chord-normal-displacement-certificate",
                "retained-displacement",
            );
            let source = &displaced.data.source;
            let shares_support = source.shares_retained_support(self)
                || source.retained_support() == self.retained_support();
            let shares_center = displaced.source_endpoint().shares_storage(center)
                || displaced.source_endpoint().same_point(center, policy)
                    == Classification::Decided(true);
            let mut reversed = shares_support.then(|| {
                source.retained_support_orientation_is_reversed()
                    != self.retained_support_orientation_is_reversed()
            });
            if reversed.is_none()
                && shares_center
                && source.support_collinearity(self, policy)? == Classification::Decided(true)
            {
                reversed = match source.tangent_dot_sign(self, policy)? {
                    Classification::Decided(RealSign::Positive) => Some(false),
                    Classification::Decided(RealSign::Negative) => Some(true),
                    Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => None,
                };
            }
            if let (true, Some(reversed)) = (shares_center, reversed) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "chord-normal-displacement-certificate",
                    if reversed {
                        "retained-reversed"
                    } else {
                        "retained-forward"
                    },
                );
                return Ok(Classification::Decided(if reversed {
                    -displaced.data.distance.clone()
                } else {
                    displaced.data.distance.clone()
                }));
            }
        }
        let expected = self.normal_displaced_point_evidence(
            center.clone(),
            fallback_distance.clone(),
            policy,
        )?;
        let forward = expected.same_point(point, policy);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "chord-normal-displacement-forward",
            match forward {
                Classification::Decided(true) => "equal",
                Classification::Decided(false) => "different",
                Classification::Uncertain(UncertaintyReason::Unsupported) => "unsupported",
                Classification::Uncertain(UncertaintyReason::Predicate) => "predicate",
                Classification::Uncertain(UncertaintyReason::Ordering) => "ordering",
                Classification::Uncertain(UncertaintyReason::RealSign) => "real-sign",
                Classification::Uncertain(UncertaintyReason::Boundary) => "boundary",
            },
        );
        if forward == Classification::Decided(true) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "chord-normal-displacement-certificate",
                "reconstructed-forward",
            );
            return Ok(Classification::Decided(fallback_distance.clone()));
        }
        if fallback_distance.zero_status() == ZeroKnowledge::Zero {
            return Ok(match forward {
                Classification::Decided(false) => {
                    Classification::Uncertain(UncertaintyReason::Predicate)
                }
                Classification::Decided(true) => unreachable!("handled above"),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        let reversed_distance = -fallback_distance.clone();
        let expected = self.normal_displaced_point_evidence(
            center.clone(),
            reversed_distance.clone(),
            policy,
        )?;
        Ok(match expected.same_point(point, policy) {
            Classification::Decided(true) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "chord-normal-displacement-certificate",
                    "reconstructed-reversed",
                );
                Classification::Decided(reversed_distance)
            }
            Classification::Decided(false) => match forward {
                Classification::Decided(false) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "chord-normal-displacement-certificate",
                        "reconstruction-mismatch",
                    );
                    Classification::Uncertain(UncertaintyReason::Predicate)
                }
                Classification::Decided(true) => unreachable!("handled above"),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
            Classification::Uncertain(reason) => match forward {
                Classification::Decided(false) => Classification::Uncertain(reason),
                Classification::Decided(true) => unreachable!("handled above"),
                Classification::Uncertain(forward_reason) => {
                    Classification::Uncertain(forward_reason)
                }
            },
        })
    }

    /// Displaces one retained point along this chord's exact unit left normal.
    ///
    /// The point need not be a chord endpoint. Fillet reconstruction uses the
    /// solved offset center as the origin and the negated offset distance to
    /// recover the source tangency point without adjoining the two endpoint
    /// fields, their direction norm, and the center field. Chords with a
    /// represented certified unit tangent normally use the smaller exact
    /// translated-point authority. An analytic-parallel source point retains
    /// this chord-normal construction even in that fast case because later
    /// fillet reconstruction must replay the correlated displacement without
    /// flattening the selected center into independent coordinates.
    pub(crate) fn normal_displaced_point_evidence(
        &self,
        point: CurvePoint2,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<CurvePoint2> {
        self.validate_policy(policy)?;
        if distance.zero_status() == ZeroKnowledge::Zero {
            return Ok(point);
        }
        if let Some((tangent_x, tangent_y)) = self.certified_unit_tangent()
            && !matches!(&point, CurvePoint2(CurvePointData2::AnalyticParallel(_)))
        {
            return match Self::translated_endpoint(
                &point,
                &(-tangent_y * &distance),
                &(tangent_x * distance),
                policy,
            )? {
                Classification::Decided(point) => Ok(point),
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "a certified chord normal displacement became uncertain: {reason:?}"
                ))),
            };
        }
        Ok(CurvePoint2::from(
            BezierAlgebraicChordParallelPoint2::normal_displaced_point(
                self.clone(),
                point,
                distance,
                policy,
            ),
        ))
    }

    /// Displaces retained point evidence along this chord's exact unit
    /// tangent. Together with [`Self::normal_displaced_point_evidence`] this
    /// is the complete local orthonormal chart used by a chord-framed circle;
    /// neither displacement merges the source point and endpoint fields.
    pub(crate) fn tangent_displaced_point_evidence(
        &self,
        point: CurvePoint2,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<CurvePoint2> {
        self.validate_policy(policy)?;
        if distance.zero_status() == ZeroKnowledge::Zero {
            return Ok(point);
        }
        if let Some((tangent_x, tangent_y)) = self.certified_unit_tangent() {
            return match Self::translated_endpoint(
                &point,
                &(tangent_x * &distance),
                &(tangent_y * distance),
                policy,
            )? {
                Classification::Decided(point) => Ok(point),
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "a certified chord tangent displacement became uncertain: {reason:?}"
                ))),
            };
        }
        Ok(CurvePoint2::from(
            BezierAlgebraicChordParallelPoint2::tangent_displaced_point(
                self.clone(),
                point,
                distance,
                policy,
            ),
        ))
    }

    /// Constructs the exact left parallel without materializing a multi-field
    /// unit tangent.  The represented tangent path remains the hot fast path;
    /// this carrier is used only when normalization spans selected fields.
    pub(crate) fn parallel_left_retained(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        self.validate_policy(policy)?;
        if distance.zero_status() == ZeroKnowledge::Zero {
            return Ok(self.clone());
        }
        if self.is_reversed() {
            // `left(reverse(C), d) = reverse(left(C, -d))`. Re-enter the
            // construction order before authoring procedural endpoints, but
            // preserve this finite interval: a clipped or extended descendant
            // need not have the same endpoints as its retained support.
            return self
                .reversed()
                .parallel_left_retained(-distance, policy)
                .map(|parallel| parallel.reversed());
        }
        let (source, distance, translation_x, translation_y) = match (self.start(), self.end()) {
            (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) if start.shares_carrier(end)
                && start.data.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal
                && start.at_end != end.at_end =>
            {
                if start.at_end {
                    (
                        start.data.source.reversed(),
                        &distance - &start.data.distance,
                        start.data.translation_x.clone(),
                        start.data.translation_y.clone(),
                    )
                } else {
                    (
                        start.data.source.clone(),
                        &start.data.distance + &distance,
                        start.data.translation_x.clone(),
                        start.data.translation_y.clone(),
                    )
                }
            }
            _ => (self.clone(), distance, Real::zero(), Real::zero()),
        };
        let (start, end) = BezierAlgebraicChordParallelPoint2::new_pair(
            source,
            distance,
            translation_x,
            translation_y,
            policy,
        );
        Ok(Self {
            data: Arc::new(BezierAlgebraicChordData2 {
                start: CurvePoint2::from(start),
                end: CurvePoint2::from(end),
                parameter_axis: self.data.parameter_axis,
                certified_axis_aligned: self.data.certified_axis_aligned,
                certified_unit_tangent: self.data.certified_unit_tangent.clone(),
                certified_circle_transverse_endpoints: 0,
                parallel_tangent_contacts: None,
                source: None,
                reversed: false,
                policy: policy.retained_object_policy(),
            }),
        })
    }
}

impl BezierAlgebraicChordPairPoint2 {
    pub(super) fn accepts_policy(&self, policy: &CurveContext) -> bool {
        policy.accepts_retained_policy(self.data.policy)
    }

    /// Extends a defining cardinal support from outside `bounds` to this
    /// retained intersection while avoiding the boundary support being
    /// classified. The resulting finite probe preserves its canonical affine
    /// line instead of rebuilding that line from an exact point and a
    /// correlated pair endpoint.
    pub(crate) fn exterior_axis_probe_avoiding(
        &self,
        boundary: &BezierAlgebraicChord2,
        bounds: &Aabb2,
        policy: &CurveContext,
    ) -> CurveResult<Option<(BezierAlgebraicChord2, bool)>> {
        if !self.accepts_policy(policy) {
            return Ok(None);
        }
        let supports = [&self.data.first, &self.data.second];
        let boundary_support = supports
            .iter()
            .position(|support| support.shares_retained_support(boundary));
        let Some(boundary_support) = boundary_support else {
            return Ok(None);
        };
        for (index, support) in supports.into_iter().enumerate() {
            if index == boundary_support {
                continue;
            }
            let Some(direction) = support.certified_axis_direction() else {
                continue;
            };
            let Some(constant) = support.constant_axis_coordinate(
                match direction.axis() {
                    Axis2::X => Axis2::Y,
                    Axis2::Y => Axis2::X,
                },
                policy,
            )?
            else {
                continue;
            };
            let one = Real::one();
            let outside = match direction {
                BezierAlgebraicChordAxisDirection2::PositiveX => {
                    Point2::new(bounds.min().x() - &one, constant)
                }
                BezierAlgebraicChordAxisDirection2::NegativeX => {
                    Point2::new(bounds.max().x() + &one, constant)
                }
                BezierAlgebraicChordAxisDirection2::PositiveY => {
                    Point2::new(constant, bounds.min().y() - &one)
                }
                BezierAlgebraicChordAxisDirection2::NegativeY => {
                    Point2::new(constant, bounds.max().y() + &one)
                }
            };
            let point = CurvePoint2::from(self.clone());
            let probe = support.chord_between_certified_ordered_support_points(
                CurvePoint2::from(outside),
                point,
                policy,
            )?;
            let source_cross_is_positive = match probe.tangent_cross_sign(boundary, policy)? {
                Classification::Decided(RealSign::Positive) => true,
                Classification::Decided(RealSign::Negative) => false,
                Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => {
                    continue;
                }
            };
            return Ok(Some((probe, source_cross_is_positive)));
        }
        Ok(None)
    }

    /// Orders this retained support intersection against another point when
    /// their construction-owned orders lie on opposite sides of the same
    /// miter anchor. Same-side orders deliberately decline because their
    /// magnitudes still require the complete support predicate.
    pub(super) fn anchor_separation_order_on_owner(
        owner: &BezierAlgebraicChord2,
        anchor_at_end: bool,
        pair_to_anchor: std::cmp::Ordering,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> Option<std::cmp::Ordering> {
        let anchor = if anchor_at_end {
            owner.end()
        } else {
            owner.start()
        };
        let other_to_anchor = if other.shares_storage(anchor) || other == anchor {
            Some(std::cmp::Ordering::Equal)
        } else {
            [0_usize, 2, 4, 8, 16].into_iter().find_map(|steps| {
                let (
                    Classification::Decided(other_bounds),
                    Classification::Decided(anchor_bounds),
                ) = (
                    algebraic_chord_endpoint_local_bounds_refined(other, steps, policy),
                    algebraic_chord_endpoint_local_bounds_refined(anchor, steps, policy),
                )
                else {
                    return None;
                };
                algebraic_chord_bounds_axis_order(
                    &other_bounds,
                    &anchor_bounds,
                    owner.data.parameter_axis.axis,
                )
                .map(|order| {
                    if owner.data.parameter_axis.coordinate_increases {
                        order
                    } else {
                        order.reverse()
                    }
                })
            })
        };
        match (pair_to_anchor, other_to_anchor) {
            (
                std::cmp::Ordering::Less,
                Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater),
            ) => Some(std::cmp::Ordering::Less),
            (
                std::cmp::Ordering::Greater,
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            ) => Some(std::cmp::Ordering::Greater),
            _ => None,
        }
    }

    pub(super) fn new(
        first: BezierAlgebraicChord2,
        second: BezierAlgebraicChord2,
        first_sides: [crate::classify::LineSide; 2],
        second_sides: [crate::classify::LineSide; 2],
        tangent_cross_sign: RealSign,
        policy: &CurveContext,
    ) -> Self {
        Self::new_with_location(
            first,
            second,
            BezierAlgebraicChordPairPointLocation2::EndpointSides {
                first: first_sides,
                second: second_sides,
                tangent_cross_sign,
            },
            policy,
        )
    }

    pub(super) fn new_with_anchor_orders(
        first: BezierAlgebraicChord2,
        second: BezierAlgebraicChord2,
        first_at_end: bool,
        first_order: std::cmp::Ordering,
        second_at_end: bool,
        second_order: std::cmp::Ordering,
        tangent_cross_sign: RealSign,
        policy: &CurveContext,
    ) -> Self {
        Self::new_with_location(
            first,
            second,
            BezierAlgebraicChordPairPointLocation2::AnchorOrders {
                first_at_end,
                first: first_order,
                second_at_end,
                second: second_order,
                tangent_cross_sign,
            },
            policy,
        )
    }

    pub(super) fn new_with_location(
        first: BezierAlgebraicChord2,
        second: BezierAlgebraicChord2,
        location: BezierAlgebraicChordPairPointLocation2,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicChordPairPointData2 {
                first,
                second,
                location,
                policy: policy.retained_object_policy(),
                recursive_support_lines: OnceLock::new(),
                recursive_point: OnceLock::new(),
            }),
        }
    }

    /// Classifies this exact supporting-line intersection against one finite
    /// analytic parallel without flattening either selected endpoint field.
    ///
    /// The point belongs to both retained supporting lines, including when it
    /// lies beyond either finite witness chord. Intersecting either complete
    /// support with the finite parallel therefore gives a complete membership
    /// test. If one projection is positive-dimensional, the other retained
    /// line is nonparallel by construction and remains an independent exact
    /// authority.
    pub(super) fn visit_incidence_on_parallel(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        regular_domain: bool,
        policy: &CurveContext,
        visitor: &mut impl FnMut(Option<&CurveParameter2>) -> ControlFlow<()>,
    ) -> CurveResult<Classification<ControlFlow<()>>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "correlated chord point entered parallel incidence under a different policy".into(),
            ));
        }
        if let Classification::Decided(Some(point)) = self.exact_represented_point(policy)? {
            return parallel.visit_point_incidence_evidence(
                &point.into(),
                range,
                incident,
                regular_domain,
                policy,
                visitor,
            );
        }

        let expanded = match incident
            .map(|incident| incident.expanded_range(range, policy))
            .transpose()?
        {
            Some(Classification::Decided(range)) => Some(range),
            Some(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            None => None,
        };
        let finite = expanded.as_ref().unwrap_or(range);
        let frame = if regular_domain {
            match parallel.source_tangent_field_in_regular_domain(
                CurveParameterDomain2::new(
                    finite,
                    incident.map(BezierParallelIncidentDomain2::parameter_ray),
                ),
                policy,
            )? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        } else {
            None
        };
        let query = CurvePoint2::from(self.clone());
        // An affine image can reuse the point's retained field and inverse
        // parameter directly. The line is an infinite support witness; only
        // the actual finite range and regular incident continuation clip it.
        if let Classification::Decided(Some(component)) =
            parallel.exact_rational_parallel_component_on_regular_range(finite, policy)?
            && let Some(line) = component.curve().exact_linear_parameterization_line()
        {
            let support = BezierAlgebraicChord2::try_new(
                line.start().clone().into(),
                line.end().clone().into(),
                policy,
            )?;
            if let Classification::Decided(support) = support {
                match support.oriented_support_side(&query, policy)? {
                    Classification::Decided(
                        crate::classify::LineSide::Left | crate::classify::LineSide::Right,
                    ) => return Ok(Classification::Decided(ControlFlow::Continue(()))),
                    Classification::Decided(crate::classify::LineSide::On) => {
                        if let Classification::Decided(parameter) =
                            affine_line_parameter_at_incident_point(&line, &query, policy)?
                        {
                            let contains =
                                policy.strict_predicate_pass(
                                    || match CurveParameterDomain2::new(finite, None)
                                        .contains_finite_parameter(&parameter, policy)?
                                    {
                                        Classification::Decided(false) => match incident {
                                            Some(incident) => incident
                                                .contains_extension_parameter(&parameter, policy),
                                            None => Ok(Classification::Decided(false)),
                                        },
                                        result => Ok(result),
                                    },
                                )?;
                            if let Classification::Decided(contains) = contains {
                                return Ok(Classification::Decided(if contains {
                                    visitor(Some(&parameter))
                                } else {
                                    ControlFlow::Continue(())
                                }));
                            }
                        }
                    }
                    Classification::Uncertain(_) => {}
                }
            }
        }

        let mut uncertainty = None;
        for chord in [&self.data.first, &self.data.second] {
            let system = match chord.recursive_projective_parallel_system_with_frame(
                parallel,
                frame.as_deref(),
                false,
                policy,
            )? {
                Classification::Decided(Some(system)) => system,
                Classification::Decided(None) => {
                    uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                    continue;
                }
                Classification::Uncertain(reason) => {
                    uncertainty.get_or_insert(reason);
                    continue;
                }
            };
            let domains = std::iter::once(SelectedThirdAxisDomain2::Finite(finite)).chain(
                incident.map(|incident| SelectedThirdAxisDomain2::IncidentRay {
                    anchor: incident.anchor(),
                    direction: incident.direction(),
                    barrier: incident.barrier(),
                }),
            );
            let mut contact_uncertainty = None;
            for domain in domains {
                let intersections = match chord
                    .recursive_projective_parallel_intersections_in_domain(
                        parallel,
                        &system,
                        frame.as_ref(),
                        None,
                        domain,
                        None,
                        false,
                        policy,
                    )? {
                    Classification::Decided(intersections) => intersections,
                    Classification::Uncertain(reason) => {
                        contact_uncertainty.get_or_insert(reason);
                        continue;
                    }
                };
                let BezierAlgebraicChordParallelIntersections2::Contacts(contacts) = intersections
                else {
                    contact_uncertainty.get_or_insert(UncertaintyReason::Boundary);
                    continue;
                };
                for contact in contacts {
                    match query.same_point(contact.point(), policy) {
                        Classification::Decided(true) => {
                            if let stop @ ControlFlow::Break(()) =
                                visitor(Some(contact.parallel_parameter()))
                            {
                                return Ok(Classification::Decided(stop));
                            }
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            contact_uncertainty.get_or_insert(reason);
                        }
                    }
                }
            }
            if contact_uncertainty.is_none() {
                return Ok(Classification::Decided(ControlFlow::Continue(())));
            }
            uncertainty = uncertainty.or(contact_uncertainty);
        }
        Ok(Classification::Uncertain(
            uncertainty.unwrap_or(UncertaintyReason::Predicate),
        ))
    }

    pub(super) fn translated(
        &self,
        delta_x: &Real,
        delta_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "correlated chord point was translated under a different predicate policy".into(),
            ));
        }
        let first = match self.data.first.translated(delta_x, delta_y, policy)? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match self.data.second.translated(delta_x, delta_y, policy)? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Self::new_with_location(
            first,
            second,
            self.data.location,
            policy,
        )))
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn transform_affine(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "correlated chord point was transformed under a different predicate policy".into(),
            ));
        }
        let determinant = Real::diff_of_products(m00, m11, m01, m10);
        let reverses_orientation = match real_sign(&determinant, &CurveContext::STRICT) {
            Some(RealSign::Positive) => false,
            Some(RealSign::Negative) => true,
            Some(RealSign::Zero) => return Err(CurveError::InvalidAffineTransform),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let first = match self
            .data
            .first
            .transform_affine(m00, m01, m10, m11, tx, ty, policy)?
        {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match self
            .data
            .second
            .transform_affine(m00, m01, m10, m11, tx, ty, policy)?
        {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let transform_location = |location| match location {
            BezierAlgebraicChordPairPointLocation2::EndpointSides {
                first,
                second,
                tangent_cross_sign,
            } => {
                let transform_sides = |sides: [crate::classify::LineSide; 2]| {
                    if !reverses_orientation {
                        return sides;
                    }
                    sides.map(|side| match side {
                        crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                        crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                        crate::classify::LineSide::On => crate::classify::LineSide::On,
                    })
                };
                BezierAlgebraicChordPairPointLocation2::EndpointSides {
                    first: transform_sides(first),
                    second: transform_sides(second),
                    tangent_cross_sign: if reverses_orientation {
                        product_sign(tangent_cross_sign, RealSign::Negative)
                    } else {
                        tangent_cross_sign
                    },
                }
            }
            BezierAlgebraicChordPairPointLocation2::AnchorOrders {
                first_at_end,
                first,
                second_at_end,
                second,
                tangent_cross_sign,
            } => BezierAlgebraicChordPairPointLocation2::AnchorOrders {
                first_at_end,
                first,
                second_at_end,
                second,
                tangent_cross_sign: if reverses_orientation {
                    product_sign(tangent_cross_sign, RealSign::Negative)
                } else {
                    tangent_cross_sign
                },
            },
        };
        Ok(Classification::Decided(Self::new_with_location(
            first,
            second,
            transform_location(self.data.location),
            policy,
        )))
    }

    pub(super) fn constant_axis_support_point(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<&CurvePoint2>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "correlated chord point was replayed under a different predicate policy".into(),
            ));
        }
        let mut uncertainty = None;
        for chord in [&self.data.first, &self.data.second] {
            let constant_coordinate = match chord.certified_axis_direction() {
                Some(
                    BezierAlgebraicChordAxisDirection2::PositiveX
                    | BezierAlgebraicChordAxisDirection2::NegativeX,
                ) => axis == Axis2::Y,
                Some(
                    BezierAlgebraicChordAxisDirection2::PositiveY
                    | BezierAlgebraicChordAxisDirection2::NegativeY,
                ) => axis == Axis2::X,
                None => false,
            };
            if !constant_coordinate {
                continue;
            }
            // Either endpoint of a certified cardinal support carries the
            // same constant coordinate.  Prefer a non-correlated endpoint:
            // Boolean splitting commonly replaces only one endpoint with a
            // chord-pair contact, while the other remains the cheapest exact
            // authority for this coordinate.
            let support = chord.retained_support();
            for support_point in [support.start(), support.end()] {
                if matches!(
                    support_point,
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                ) {
                    uncertainty.get_or_insert(UncertaintyReason::Ordering);
                    continue;
                }
                return Ok(Classification::Decided(support_point));
            }
        }
        Ok(Classification::Uncertain(
            uncertainty.unwrap_or(UncertaintyReason::Ordering),
        ))
    }

    pub(super) fn exact_axis_coordinate(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let support_point = match self.constant_axis_support_point(axis, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let coordinate = match support_point {
            CurvePoint2(CurvePointData2::Exact(point)) => Some(match axis {
                Axis2::X => point.x().clone(),
                Axis2::Y => point.y().clone(),
            }),
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                point.exact_coordinate(axis == Axis2::X, policy)
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        Ok(Classification::Decided(coordinate))
    }

    /// Materializes a correlated chord contact only when both coordinates
    /// already reduce to the canonical `Real` scalar.
    ///
    /// Cardinal supports expose their constant coordinates independently, so
    /// `(alpha, 0)` need not collapse as a complete point merely to recover
    /// its represented ordinate. General supports retain the native
    /// supporting-line fast path when all four source endpoints are represented.
    pub(crate) fn exact_represented_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Point2>>> {
        if !self.accepts_policy(policy) {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let x = self.exact_axis_coordinate(Axis2::X, policy)?;
        let y = self.exact_axis_coordinate(Axis2::Y, policy)?;
        if let (Classification::Decided(Some(x)), Classification::Decided(Some(y))) = (&x, &y) {
            return Ok(Classification::Decided(Some(Point2::new(
                x.clone(),
                y.clone(),
            ))));
        }

        let first = self.data.first.retained_support().exact_line();
        let second = self.data.second.retained_support().exact_line();
        if let (Some(first), Some(second)) = (first, second) {
            return match crate::offset::line_support_intersection(&first, &second, policy)? {
                Classification::Decided(point) => Ok(Classification::Decided(point)),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }

        Ok(match (x, y) {
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                Classification::Uncertain(reason)
            }
            _ => Classification::Decided(None),
        })
    }

    /// Reconstructs this two-support intersection inside the recursive tower
    /// already shared by its four authored endpoints. Homogeneous line and
    /// point cross products preserve all endpoint correlations and add no new
    /// algebraic generator.
    pub(super) fn recursive_projective_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "a correlated chord point entered a recursive field under a different predicate policy"
                    .into(),
            ));
        }
        if let Some(point) = self.data.recursive_point.get() {
            return Ok(Classification::Decided(Some(point.clone())));
        }
        let first = self.data.first.retained_support();
        let second = self.data.second.retained_support();
        let endpoints = [first.start(), first.end(), second.start(), second.end()];
        if endpoints.iter().any(|endpoint| {
            matches!(
                endpoint,
                CurvePoint2(CurvePointData2::AlgebraicChordPair(point))
                    if Arc::ptr_eq(&point.data, &self.data)
            )
        }) {
            return Ok(Classification::Decided(None));
        }
        let points = match if policy.selects_approximate_512() {
            // Cartesian materialization is only a cold predicate accelerator
            // for this already-authoritative symbolic support intersection.
            // Reuse a common or ancestral exact field when one exists, but
            // keep divergent descendants out of both APPROXIMATE_512 passes;
            // native support determinants below own the terminal decision.
            policy
                .strict_predicate_pass(|| recursive_projective_evidence_points(&endpoints, policy))
        } else {
            recursive_projective_evidence_points(&endpoints, policy)
        }? {
            Classification::Decided(Some(points)) => points,
            Classification::Decided(None) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [first_start, first_end, second_start, second_end]: [
            BezierRecursiveQuadraticProjectivePoint2;
            4
        ] = points
            .try_into()
            .expect("a recursive chord pair retains four support endpoints");
        let line = |start: &BezierRecursiveQuadraticProjectivePoint2,
                    end: &BezierRecursiveQuadraticProjectivePoint2| {
            Some([
                start
                    .y
                    .multiply(&end.denominator)?
                    .subtract(&start.denominator.multiply(&end.y)?)?,
                start
                    .denominator
                    .multiply(&end.x)?
                    .subtract(&start.x.multiply(&end.denominator)?)?,
                start
                    .x
                    .multiply(&end.y)?
                    .subtract(&start.y.multiply(&end.x)?)?,
            ])
        };
        let (Some(first), Some(second)) = (
            line(&first_start, &first_end),
            line(&second_start, &second_end),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(point) = (|| {
            Some(BezierRecursiveQuadraticProjectivePoint2 {
                x: first[1]
                    .multiply(&second[2])?
                    .subtract(&first[2].multiply(&second[1])?)?,
                y: first[2]
                    .multiply(&second[0])?
                    .subtract(&first[0].multiply(&second[2])?)?,
                denominator: first[0]
                    .multiply(&second[1])?
                    .subtract(&first[1].multiply(&second[0])?)?,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        // With positively normalized endpoint denominators, each retained
        // support has homogeneous coefficients `(-dy, dx, ...)`. The point
        // cross product's denominator is therefore exactly
        // `cross(first_tangent, second_tangent)`, whose sign was already
        // certified when this nonparallel pair was constructed. Reuse that
        // certificate instead of re-signing the much larger recursive
        // determinant.
        let tangent_cross_sign = match self.data.location {
            BezierAlgebraicChordPairPointLocation2::EndpointSides {
                tangent_cross_sign, ..
            }
            | BezierAlgebraicChordPairPointLocation2::AnchorOrders {
                tangent_cross_sign, ..
            } => tangent_cross_sign,
        };
        let point = match tangent_cross_sign {
            RealSign::Positive => point,
            RealSign::Negative => {
                let negative = Real::from(-1_i8);
                BezierRecursiveQuadraticProjectivePoint2 {
                    x: point.x.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive chord-pair point normalization exceeded its field budget"
                                .into(),
                        )
                    })?,
                    y: point.y.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive chord-pair point normalization exceeded its field budget"
                                .into(),
                        )
                    })?,
                    denominator: point.denominator.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive chord-pair point normalization exceeded its field budget"
                                .into(),
                        )
                    })?,
                }
            }
            RealSign::Zero => {
                return Err(CurveError::Topology(
                    "a nonparallel chord pair retained a zero tangent cross sign".into(),
                ));
            }
        };
        let _ = self.data.recursive_point.set(point);
        Ok(Classification::Decided(Some(
            self.data
                .recursive_point
                .get()
                .expect("a recursive chord-pair point was initialized above")
                .clone(),
        )))
    }

    /// Materializes the unique projective intersection as two standalone
    /// algebraic coordinates.
    ///
    /// This is the cold, rank-independent fallback for consumers which need
    /// a Cartesian frame (notably general circle incidence).  Ordinary side,
    /// order, and support predicates continue to use the retained two-line
    /// authority above.  All endpoint roots remain in one dense tensor until
    /// each final quotient is selected under STRICT, so no rounded primitive
    /// element or independently chosen conjugate can enter the construction.
    pub(super) fn represented_coordinates(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "correlated chord point was represented under a different predicate policy".into(),
            ));
        }
        let first = self.data.first.retained_support();
        let second = self.data.second.retained_support();
        let represented_first = represented_parallel_chord_support(first, policy)?;
        let represented_second = represented_parallel_chord_support(second, policy)?;
        match (represented_first, represented_second) {
            (Classification::Decided(Some(first)), Classification::Decided(Some(second))) => {
                let BezierRepresentedParallelChordSupport2 {
                    coordinates: first_coordinates,
                    distance: first_distance,
                    translation_x: first_translation_x,
                    translation_y: first_translation_y,
                    direction: first_direction,
                } = first;
                let BezierRepresentedParallelChordSupport2 {
                    coordinates: second_coordinates,
                    distance: second_distance,
                    translation_x: second_translation_x,
                    translation_y: second_translation_y,
                    direction: second_direction,
                } = second;
                let coordinates = first_coordinates
                    .into_iter()
                    .chain(second_coordinates)
                    .collect::<Vec<_>>();
                let Some((sources, coordinates)) = represented_affine_tensor_basis(&coordinates)
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let [
                    first_x,
                    first_y,
                    first_dx,
                    first_dy,
                    first_speed,
                    second_x,
                    second_y,
                    second_dx,
                    second_dy,
                    second_speed,
                ]: [DenseTensorPolynomial; 10] = coordinates
                    .try_into()
                    .expect("a represented parallel pair retains both support frames");
                let line =
                    |x: DenseTensorPolynomial,
                     y: DenseTensorPolynomial,
                     dx: DenseTensorPolynomial,
                     dy: DenseTensorPolynomial,
                     speed: DenseTensorPolynomial,
                     distance: &Real,
                     translation_x: &Real,
                     translation_y: &Real,
                     direction: BezierAlgebraicChordUnitDisplacement2| {
                        let a = dy.scale(&Real::from(-1_i8))?;
                        let b = dx.clone();
                        let mut c = dy
                            .multiply(&x)?
                            .subtract(&dx.multiply(&y)?)?
                            .add(&dy.scale(translation_x)?)?
                            .subtract(&dx.scale(translation_y)?)?;
                        if direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal {
                            c = c.subtract(&speed.scale(distance)?)?;
                        }
                        Some([a, b, c])
                    };
                let (Some(first), Some(second)) = (
                    line(
                        first_x,
                        first_y,
                        first_dx,
                        first_dy,
                        first_speed,
                        &first_distance,
                        &first_translation_x,
                        &first_translation_y,
                        first_direction,
                    ),
                    line(
                        second_x,
                        second_y,
                        second_dx,
                        second_dy,
                        second_speed,
                        &second_distance,
                        &second_translation_x,
                        &second_translation_y,
                        second_direction,
                    ),
                ) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                return Ok(represented_projective_line_intersection(
                    first, second, &sources,
                ));
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            _ => {}
        }
        let endpoint = |point: &CurvePoint2| {
            if matches!(
                point,
                CurvePoint2(CurvePointData2::AlgebraicChordPair(pair))
                    if Arc::ptr_eq(&pair.data, &self.data)
            ) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            represented_point_evidence_coordinates(point, policy)
        };
        let mut coordinates = Vec::with_capacity(8);
        for point in [first.start(), first.end(), second.start(), second.end()] {
            match endpoint(point)? {
                Classification::Decided(point) => coordinates.extend(point),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let Some((sources, coordinates)) = represented_affine_tensor_basis(&coordinates) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [ax, ay, bx, by, cx, cy, dx, dy]: [DenseTensorPolynomial; 8] = coordinates
            .try_into()
            .expect("a represented chord pair retains all endpoint coordinates");
        let Some((first, second)) = (|| {
            let first_dx = bx.subtract(&ax)?;
            let first_dy = by.subtract(&ay)?;
            let second_dx = dx.subtract(&cx)?;
            let second_dy = dy.subtract(&cy)?;
            let first_c = first_dy.multiply(&ax)?.subtract(&first_dx.multiply(&ay)?)?;
            let second_c = second_dy
                .multiply(&cx)?
                .subtract(&second_dx.multiply(&cy)?)?;
            Some((
                [first_dy.scale(&Real::from(-1_i8))?, first_dx, first_c],
                [second_dy.scale(&Real::from(-1_i8))?, second_dx, second_c],
            ))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(represented_projective_line_intersection(
            first, second, &sources,
        ))
    }

    pub(crate) fn same_point(&self, other: &Self, policy: &CurveContext) -> Classification<bool> {
        if !self.accepts_policy(policy) || !other.accepts_policy(policy) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        if self == other {
            return Classification::Decided(true);
        }
        let mut incidence_uncertainty = None;
        let mut incident_with_both = true;
        for support in [&other.data.first, &other.data.second] {
            match self.oriented_side_to_chord(support, policy) {
                Ok(Classification::Decided(crate::classify::LineSide::On)) => {}
                Ok(Classification::Decided(
                    crate::classify::LineSide::Left | crate::classify::LineSide::Right,
                )) => return Classification::Decided(false),
                Ok(Classification::Uncertain(reason)) => {
                    incidence_uncertainty.get_or_insert(reason);
                    incident_with_both = false;
                }
                Err(_) => {
                    incidence_uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                    incident_with_both = false;
                }
            }
        }
        if incident_with_both {
            return Classification::Decided(true);
        }
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if let (Classification::Decided(first), Classification::Decided(second)) = (
                self.conservative_bounds_refined(refinement_steps, policy),
                other.conservative_bounds_refined(refinement_steps, policy),
            ) {
                terminal_refined |= refinement_steps == 512;
                if first.overlaps(&second, policy) == Classification::Decided(false) {
                    return Classification::Decided(false);
                }
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Classification::Decided(true);
        }
        Classification::Uncertain(incidence_uncertainty.unwrap_or(UncertaintyReason::Predicate))
    }

    /// Signs this support intersection against a third support in one flat
    /// recursive field.  Building the pair point first and then importing it
    /// into the query field nests two composita and expands the same line
    /// determinants twice. Each support first publishes one local line from
    /// its compact anchor/direction frame; only those three lines enter the
    /// shared field, and the dual determinant never materializes Cartesian
    /// coordinates.
    pub(super) fn flat_recursive_oriented_side_to_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!("pair flat determinant stage=begin");
        }
        if policy.has_bounded_exact_predicate_budget() {
            // Publishing even one local projective line can multiply a
            // descendant endpoint's complete recursive tower. This authority
            // is therefore wholly outside speculative bounded dispatch; the
            // full STRICT pass (or APPROXIMATE_512 terminal pass) re-enters it.
            return Ok(None);
        }
        let first = self.data.first.retained_support();
        let second = self.data.second.retained_support();
        let query = chord.retained_support();
        let defining_lines = if let Some(lines) = self.data.recursive_support_lines.get() {
            lines.clone()
        } else {
            let (Some(first), Some(second)) = (
                first.recursive_projective_support_line(policy)?,
                second.recursive_projective_support_line(policy)?,
            ) else {
                return Ok(None);
            };
            let lines = Arc::new([first, second]);
            let _ = self.data.recursive_support_lines.set(lines.clone());
            self.data
                .recursive_support_lines
                .get()
                .cloned()
                .unwrap_or(lines)
        };
        let [first, second] = defining_lines.as_ref().clone();
        let Some(query) = query.recursive_projective_support_line(policy)? else {
            return Ok(None);
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!("pair flat determinant stage=lines");
        }
        let exact_line = |line: &BezierRecursiveQuadraticProjectivePoint2| {
            Some([
                line.x.exact_real_value_with_retained_witnesses()?,
                line.y.exact_real_value_with_retained_witnesses()?,
                line.denominator
                    .exact_real_value_with_retained_witnesses()?,
            ])
        };
        if let (Some(first_real), Some(second_real), Some(query_real)) =
            (exact_line(&first), exact_line(&second), exact_line(&query))
        {
            let x = Real::diff_of_products(
                &first_real[1],
                &second_real[2],
                &first_real[2],
                &second_real[1],
            );
            let y = Real::diff_of_products(
                &first_real[2],
                &second_real[0],
                &first_real[0],
                &second_real[2],
            );
            let denominator = Real::diff_of_products(
                &first_real[0],
                &second_real[1],
                &first_real[1],
                &second_real[0],
            );
            let incidence = Real::signed_product_sum(
                [true, true, true],
                [
                    [&query_real[0], &x],
                    [&query_real[1], &y],
                    [&query_real[2], &denominator],
                ],
            );
            if let (
                Some(mut incidence_sign),
                Some(denominator_sign @ (RealSign::Positive | RealSign::Negative)),
            ) = (
                real_sign(&incidence, policy),
                real_sign(&denominator, policy),
            ) {
                incidence_sign = product_sign(incidence_sign, denominator_sign);
                if chord.retained_support_orientation_is_reversed() {
                    incidence_sign = product_sign(incidence_sign, RealSign::Negative);
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "compact-real-support-determinant",
                );
                return Ok(Some(crate::classify::LineSide::from_real_sign(
                    incidence_sign,
                )));
            }
        }
        // Most third supports are transverse to the retained miter point.
        // Bound each locally compact line independently before constructing
        // a compositum: interval arithmetic over the scalar determinant is
        // exact whenever both the affine denominator and incidence exclude
        // zero, and it preserves every correlation within each source field.
        let refinement_schedule: &[usize] = if policy.permits_approximate_512() {
            &[0, 8, 128, 512]
        } else {
            &[0, 8, 128, 256, 512]
        };
        let mut terminal_refined = false;
        for &refinement_steps in refinement_schedule {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("pair flat determinant stage=interval-{refinement_steps}-begin");
            }
            let interval_line = |line: &BezierRecursiveQuadraticProjectivePoint2| {
                Some([
                    line.x.interval(refinement_steps)?,
                    line.y.interval(refinement_steps)?,
                    line.denominator.interval(refinement_steps)?,
                ])
            };
            let (Some(first_interval), Some(second_interval), Some(query_interval)) = (
                interval_line(&first),
                interval_line(&second),
                interval_line(&query),
            ) else {
                continue;
            };
            let product_difference = |first: &RealInterval,
                                      second: &RealInterval,
                                      third: &RealInterval,
                                      fourth: &RealInterval| {
                first
                    .multiply(second)
                    .and_then(|first| third.multiply(fourth).map(|second| first.subtract(&second)))
            };
            let (Some(x), Some(y), Some(denominator)) = (
                product_difference(
                    &first_interval[1],
                    &second_interval[2],
                    &first_interval[2],
                    &second_interval[1],
                ),
                product_difference(
                    &first_interval[2],
                    &second_interval[0],
                    &first_interval[0],
                    &second_interval[2],
                ),
                product_difference(
                    &first_interval[0],
                    &second_interval[1],
                    &first_interval[1],
                    &second_interval[0],
                ),
            ) else {
                continue;
            };
            let incidence = query_interval[0]
                .multiply(&x)
                .and_then(|value| query_interval[1].multiply(&y).map(|y| value.add(&y)))
                .and_then(|value| {
                    query_interval[2]
                        .multiply(&denominator)
                        .map(|constant| value.add(&constant))
                });
            let Some(incidence) = incidence else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!(
                    "pair flat determinant stage=interval-{refinement_steps}-values incidence=({:?},{:?}) denominator=({:?},{:?})",
                    incidence.lower.to_f64_lossy(),
                    incidence.upper.to_f64_lossy(),
                    denominator.lower.to_f64_lossy(),
                    denominator.upper.to_f64_lossy(),
                );
            }
            let (Some(mut incidence_sign), Some(denominator_sign)) = (
                dense_strict_interval_sign(&incidence),
                dense_strict_interval_sign(&denominator),
            ) else {
                continue;
            };
            if denominator_sign == RealSign::Zero {
                continue;
            }
            incidence_sign = product_sign(incidence_sign, denominator_sign);
            if chord.retained_support_orientation_is_reversed() {
                incidence_sign = product_sign(incidence_sign, RealSign::Negative);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "local-support-determinant-interval",
            );
            return Ok(Some(crate::classify::LineSide::from_real_sign(
                incidence_sign,
            )));
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("pair flat determinant approximate-512 terminal=On");
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "local-support-determinant-approximate-512",
            );
            return Ok(Some(crate::classify::LineSide::On));
        }
        if policy.has_bounded_exact_predicate_budget() {
            // The caller continues with independently refinable point/support
            // boxes through the 512-bit equality terminal. Joining all three
            // recursive support fields here would be an unbounded exact
            // promotion inside both the preliminary and terminal
            // APPROXIMATE_512 passes; retain that complete fallback for
            // STRICT only.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "bounded-before-flat-field-merge",
            );
            return Ok(None);
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!("pair flat determinant stage=merge");
        }
        let mut field = first.denominator.field();
        let mut lines = vec![first];
        for line in [second, query] {
            if let Some(line) = line.lifted_to(&field) {
                lines.push(line);
                continue;
            }
            let line_field = line.denominator.field();
            if let Some(lifted) = lines
                .iter()
                .map(|line| line.lifted_to(&line_field))
                .collect::<Option<Vec<_>>>()
            {
                field = line_field;
                lines = lifted;
                lines.push(line);
                continue;
            }
            match recursive_merge_projective_point_fields(&field, &lines, &line, policy)? {
                Classification::Decided(Some((joined, mut lifted, line))) => {
                    field = joined;
                    lifted.push(line);
                    lines = lifted;
                }
                Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
            }
        }
        let [first, second, query]: [BezierRecursiveQuadraticProjectivePoint2; 3] = lines
            .try_into()
            .expect("a flat support determinant retains three projective lines");
        let first = [first.x, first.y, first.denominator];
        let second = [second.x, second.y, second.denominator];
        let query = [query.x, query.y, query.denominator];
        let Some((x, y, denominator)) = (|| {
            Some((
                first[1]
                    .multiply(&second[2])?
                    .subtract(&first[2].multiply(&second[1])?)?,
                first[2]
                    .multiply(&second[0])?
                    .subtract(&first[0].multiply(&second[2])?)?,
                first[0]
                    .multiply(&second[1])?
                    .subtract(&first[1].multiply(&second[0])?)?,
            ))
        })() else {
            return Ok(None);
        };
        let Some(incidence) = query[0]
            .multiply(&x)
            .and_then(|value| query[1].multiply(&y).and_then(|y| value.add(&y)))
            .and_then(|value| {
                query[2]
                    .multiply(&denominator)
                    .and_then(|constant| value.add(&constant))
            })
        else {
            return Ok(None);
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let (base, extensions) = incidence.field().base_and_extension_path();
            eprintln!(
                "pair flat determinant merged field sources={} extensions={} incidence-zero={} denominator-zero={}",
                base.sources.len(),
                extensions.len(),
                incidence.is_structurally_zero(),
                denominator.is_structurally_zero(),
            );
            for (index, source) in base.sources.iter().enumerate() {
                eprintln!(
                    "pair flat source index={index} constraint={} symbol={:?} ordinal={} degree={} interval=({:?},{:?}) exact={} compact-degree={:?} real-witness={}",
                    source.constraint_index,
                    source.symbol,
                    source.interval_index,
                    source.polynomial_coefficients.len().saturating_sub(1),
                    source.interval.lower.to_f64_lossy(),
                    source.interval.upper.to_f64_lossy(),
                    source.exact_point_witness().is_some(),
                    hypersolve::compact_algebraic_root_low_degree_witness(source)
                        .map(|root| root.polynomial_coefficients.len().saturating_sub(1)),
                    hypersolve::compact_algebraic_root_low_degree_witness(source)
                        .is_some_and(|root| root.exact_point_witness().is_some()),
                );
            }
        }
        let strict = policy.strict_counterpart();
        let sign_policy = if policy.permits_approximate_512() {
            policy
        } else {
            &strict
        };
        let sign = |value: &BezierRecursiveQuadraticValue2| {
            value.sign(sign_policy).map(|sign| match sign {
                Classification::Decided(sign) => Some(sign),
                Classification::Uncertain(_) => None,
            })
        };
        let (Some(mut incidence_sign), Some(denominator_sign)) =
            (sign(&incidence)?, sign(&denominator)?)
        else {
            return Ok(None);
        };
        if denominator_sign == RealSign::Zero {
            return Ok(None);
        }
        incidence_sign = product_sign(incidence_sign, denominator_sign);
        if chord.retained_support_orientation_is_reversed() {
            incidence_sign = product_sign(incidence_sign, RealSign::Negative);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair-side-kernel",
            "flat-recursive-support-determinant",
        );
        Ok(Some(crate::classify::LineSide::from_real_sign(
            incidence_sign,
        )))
    }

    /// Signs a third support from its two authored endpoints without joining
    /// the three support fields.  For retained lines `A`, `B` meeting at this
    /// point and an oriented query chord `R -> S`, the Grassmann-Pluecker
    /// identity gives
    ///
    /// `side(RS, A∩B) = sign((A(R)B(S) - A(S)B(R)) * cross(A, B))`.
    ///
    /// Exact endpoint-side predicates often contain a structural zero (an
    /// offset/bevel endpoint lies on one defining support), or make the two
    /// products opposite-signed.  Those cases need no magnitude comparison,
    /// Cartesian point, primitive element, or recursive field compositum.
    pub(super) fn endpoint_incidence_oriented_side_to_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<
        Option<(
            Option<crate::classify::LineSide>,
            [[crate::classify::LineSide; 2]; 2],
        )>,
    > {
        let BezierAlgebraicChordPairPointLocation2::AnchorOrders {
            tangent_cross_sign, ..
        } = self.data.location
        else {
            return Ok(None);
        };
        let endpoints = [chord.start(), chord.end()];
        let side_sign = |support: &BezierAlgebraicChord2,
                         point: &CurvePoint2|
         -> CurveResult<Option<RealSign>> {
            if matches!(
                point,
                CurvePoint2(CurvePointData2::AlgebraicChordPair(point))
                    if point == self
            ) {
                return Ok(Some(RealSign::Zero));
            }
            if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point
                && point != self
            {
                let retained = support.retained_support();
                if [retained.start(), retained.end()]
                    .into_iter()
                    .all(|endpoint| {
                        !matches!(
                            endpoint,
                            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                        )
                    })
                    && let Some((Some(side), _)) =
                        point.endpoint_incidence_oriented_side_to_chord(retained, policy)?
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair-side-kernel",
                        "nested-endpoint-incidence-pluecker-sign",
                    );
                    return Ok(Some(match side {
                        crate::classify::LineSide::Left => RealSign::Positive,
                        crate::classify::LineSide::On => RealSign::Zero,
                        crate::classify::LineSide::Right => RealSign::Negative,
                    }));
                }
            }
            let classify = || -> CurveResult<Classification<crate::classify::LineSide>> {
                if [support.start(), support.end()]
                    .into_iter()
                    .any(|endpoint| point.shares_storage(endpoint) || point == endpoint)
                    || support.retains_normal_offset_point_incidence(point)
                {
                    return Ok(Classification::Decided(crate::classify::LineSide::On));
                }
                if let Some(direction) = support.certified_axis_direction()
                    && let Some(side @ Classification::Decided(_)) =
                        support.axis_oriented_side(point, direction, policy)
                {
                    return Ok(side);
                }
                let bounded = support
                    .oriented_side_by_refinement_with_limit(point, policy, 8, false, true)?;
                if matches!(bounded, Classification::Decided(_)) {
                    return Ok(bounded);
                }
                if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) = point {
                    if let Classification::Decided(Some(side)) =
                        point.normal_offset_oriented_side_to_chord(support, policy)?
                    {
                        return Ok(Classification::Decided(side));
                    }
                    if let Classification::Decided(Some(side)) =
                        point.oriented_side_to_analytic_tangent_chord(support, policy)?
                    {
                        return Ok(Classification::Decided(side));
                    }
                }
                Ok(bounded)
            };
            Ok(match policy.strict_predicate_pass(classify)? {
                Classification::Decided(crate::classify::LineSide::Left) => {
                    Some(RealSign::Positive)
                }
                Classification::Decided(crate::classify::LineSide::On) => Some(RealSign::Zero),
                Classification::Decided(crate::classify::LineSide::Right) => {
                    Some(RealSign::Negative)
                }
                Classification::Uncertain(_) => None,
            })
        };
        let (Some(first_start), Some(first_end), Some(second_start), Some(second_end)) = (
            side_sign(&self.data.first, endpoints[0])?,
            side_sign(&self.data.first, endpoints[1])?,
            side_sign(&self.data.second, endpoints[0])?,
            side_sign(&self.data.second, endpoints[1])?,
        ) else {
            return Ok(None);
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!(
                "pair endpoint incidence signs first=[{first_start:?},{first_end:?}] second=[{second_start:?},{second_end:?}] cross={tangent_cross_sign:?}",
            );
        }
        let positive = product_sign(first_start, second_end);
        let negative = product_sign(first_end, second_start);
        let determinant_sign = match (positive, negative) {
            (RealSign::Zero, RealSign::Zero) => Some(RealSign::Zero),
            (sign, RealSign::Zero) => Some(sign),
            (RealSign::Zero, RealSign::Positive) => Some(RealSign::Negative),
            (RealSign::Zero, RealSign::Negative) => Some(RealSign::Positive),
            (RealSign::Positive, RealSign::Negative) => Some(RealSign::Positive),
            (RealSign::Negative, RealSign::Positive) => Some(RealSign::Negative),
            (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
                None
            }
        };
        #[cfg(feature = "dispatch-trace")]
        if determinant_sign.is_some() {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "endpoint-incidence-pluecker-sign",
            );
        }
        Ok(Some((
            determinant_sign.map(|determinant_sign| {
                crate::classify::LineSide::from_real_sign(product_sign(
                    determinant_sign,
                    tangent_cross_sign,
                ))
            }),
            [
                [first_start, first_end].map(crate::classify::LineSide::from_real_sign),
                [second_start, second_end].map(crate::classify::LineSide::from_real_sign),
            ],
        )))
    }

    pub(super) fn oriented_side_to_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "correlated chord point was replayed under a different predicate policy".into(),
            ));
        }
        if !policy.has_bounded_exact_predicate_budget() {
            let bounded = policy
                .bounded_exact_predicate_pass(|| self.oriented_side_to_chord(chord, policy))?;
            if matches!(bounded, Classification::Decided(_)) {
                return Ok(bounded);
            }
        }
        if chord.shares_retained_support(&self.data.first)
            || chord.shares_retained_support(&self.data.second)
        {
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        // Offset construction may rebuild a represented span independently
        // from the tangent witness used to author this support intersection.
        // Pointer identity is then absent even though both exact supports are
        // the same affine line. Replay that collinearity certificate before
        // falling back to coordinate refinement.
        for support in [&self.data.first, &self.data.second] {
            let collinearity = chord.support_collinearity(support, policy)?;
            if collinearity == Classification::Decided(true) {
                return Ok(Classification::Decided(crate::classify::LineSide::On));
            }
        }

        // Try the Grassmann-Pluecker sign identity before constructing the
        // intersection point or joining three recursive support fields. Four
        // endpoint incidences decide every structural-zero and
        // opposite-product case exactly; only equal-signed products require
        // the magnitude-bearing determinant below.
        let retained = chord.retained_support();
        if !Arc::ptr_eq(&chord.data, &retained.data)
            && let Some((Some(mut side), _)) =
                self.endpoint_incidence_oriented_side_to_chord(retained, policy)?
        {
            if chord.retained_support_orientation_is_reversed() {
                side = match side {
                    crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                    crate::classify::LineSide::On => crate::classify::LineSide::On,
                    crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                };
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "retained-support-endpoint-incidence",
            );
            return Ok(Classification::Decided(side));
        }
        if let Some((Some(side), _)) =
            self.endpoint_incidence_oriented_side_to_chord(chord, policy)?
        {
            return Ok(Classification::Decided(side));
        }

        // A certified cardinal query is only a one-coordinate order.  Keep
        // the correlated support intersection intact and compare that
        // coordinate directly; forming the general three-line determinant
        // would otherwise join every radical carried by both defining
        // supports merely to classify against a represented horizontal or
        // vertical line.
        if let Some(direction) = chord.certified_axis_direction() {
            let constant_axis = match direction.axis() {
                Axis2::X => Axis2::Y,
                Axis2::Y => Axis2::X,
            };
            if matches!(
                self.constant_axis_support_point(constant_axis, policy)?,
                Classification::Decided(_)
            ) && let point = CurvePoint2::from(self.clone())
                && let Some(Classification::Decided(side)) =
                    policy.bounded_exact_predicate_pass(|| {
                        chord.axis_oriented_side(&point, direction, policy)
                    })
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "certified-axis-coordinate-order",
                );
                return Ok(Classification::Decided(side));
            }
        }

        // A tangent/normal displacement can use this exact miter point as
        // its arbitrary origin.  The displaced endpoint Q lies on `chord`,
        // so the requested side is the sign of
        // `cross(chord_tangent, P-Q)`. With no extra translation this is one
        // retained tangent cross (tangent displacement) or dot (left-normal
        // displacement), scaled by the signed distance. No point coordinate
        // or support determinant is involved.
        for endpoint in [
            chord.retained_support().start(),
            chord.retained_support().end(),
        ] {
            let CurvePoint2(CurvePointData2::AlgebraicChordParallel(endpoint)) = endpoint else {
                continue;
            };
            let Some(CurvePoint2(CurvePointData2::AlgebraicChordPair(origin))) =
                endpoint.data.source_point.as_deref()
            else {
                continue;
            };
            if self != origin
                || endpoint.data.translation_x.zero_status() != ZeroKnowledge::Zero
                || endpoint.data.translation_y.zero_status() != ZeroKnowledge::Zero
            {
                continue;
            }
            let distance_sign = match real_sign(&endpoint.data.distance, &CurveContext::STRICT) {
                Some(sign) => sign,
                None => continue,
            };
            if distance_sign == RealSign::Zero {
                return Ok(Classification::Decided(crate::classify::LineSide::On));
            }
            let (cross_scale, dot_scale) = match endpoint.data.direction {
                BezierAlgebraicChordUnitDisplacement2::Tangent => (Real::one(), Real::zero()),
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => (Real::zero(), Real::one()),
            };
            let relation = match chord.tangent_cross_dot_linear_combination_sign(
                &endpoint.data.source,
                &cross_scale,
                &dot_scale,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(_) => continue,
            };
            let side = crate::classify::LineSide::from_real_sign(product_sign(
                product_sign(relation, distance_sign),
                RealSign::Negative,
            ));
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "displacement-origin-incidence",
            );
            return Ok(Classification::Decided(side));
        }

        if chord.certified_axis_direction().is_some()
            && !policy.has_bounded_exact_predicate_budget()
            && let Some(side) = policy.strict_predicate_pass(|| {
                self.flat_recursive_oriented_side_to_chord(chord, policy)
            })?
        {
            return Ok(Classification::Decided(side));
        }

        // An offset miter retains the sign of its affine displacement from
        // one endpoint on each defining support.  Along such a support the
        // query-line incidence is affine:
        //
        //   L(P) = L(anchor) + lambda * cross(query, support).
        //
        // The anchor order owns `sign(lambda)`.  When the two exact terms
        // have one sign (or either is zero), their sum has that sign without
        // reconstructing `P` in the compositum of all three support fields.
        if !policy.has_bounded_exact_predicate_budget()
            && let BezierAlgebraicChordPairPointLocation2::AnchorOrders {
                first_at_end,
                first,
                second_at_end,
                second,
                ..
            } = self.data.location
        {
            for (support, at_end, order) in [
                (&self.data.first, first_at_end, first),
                (&self.data.second, second_at_end, second),
            ] {
                let anchor = if at_end {
                    support.end()
                } else {
                    support.start()
                };
                if matches!(anchor, CurvePoint2(CurvePointData2::AlgebraicChordPair(_))) {
                    continue;
                }
                // This is a sufficient accelerator, not the complete side
                // predicate. Keep it on bounded boxes and leave an unresolved
                // anchor to the exact determinant fallback below.
                let anchor_side = match chord
                    .oriented_side_by_refinement_with_limit(anchor, policy, 8, false, true)?
                {
                    Classification::Decided(side) => side,
                    Classification::Uncertain(_) => continue,
                };
                let cross = match policy
                    .strict_predicate_pass(|| chord.tangent_cross_sign(support, policy))?
                {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(_) => continue,
                };
                let displacement_sign = match order {
                    std::cmp::Ordering::Less => RealSign::Negative,
                    std::cmp::Ordering::Greater => RealSign::Positive,
                    std::cmp::Ordering::Equal => {
                        return Err(CurveError::Topology(
                            "a retained offset-miter anchor lost its strict order".into(),
                        ));
                    }
                };
                let displacement_side = crate::classify::LineSide::from_real_sign(product_sign(
                    cross,
                    displacement_sign,
                ));
                let side = match (anchor_side, displacement_side) {
                    (crate::classify::LineSide::On, side)
                    | (side, crate::classify::LineSide::On) => Some(side),
                    (first, second) if first == second => Some(first),
                    _ => None,
                };
                if let Some(side) = side {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair-side-kernel",
                        "certified-offset-anchor-affine-sign",
                    );
                    return Ok(Classification::Decided(side));
                }
            }
        }

        // The pair is already represented by two exact projective supports.
        // Sign their intersection against the query before expanding four
        // query-endpoint incidences.  In particular, a cardinal query remains
        // one compact line coefficient instead of adjoining both endpoint
        // coordinates independently.
        if chord.certified_axis_direction().is_none()
            && !policy.has_bounded_exact_predicate_budget()
            && let Some(side) = policy.strict_predicate_pass(|| {
                self.flat_recursive_oriented_side_to_chord(chord, policy)
            })?
        {
            return Ok(Classification::Decided(side));
        }

        for endpoint in [chord.start(), chord.end()] {
            match endpoint {
                CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) if self == point => {
                    return Ok(Classification::Decided(crate::classify::LineSide::On));
                }
                CurvePoint2(CurvePointData2::Exact(_))
                | CurvePoint2(CurvePointData2::Algebraic(_))
                    if self.same_point_evidence(endpoint, policy)
                        == Classification::Decided(true) =>
                {
                    return Ok(Classification::Decided(crate::classify::LineSide::On));
                }
                CurvePoint2(CurvePointData2::Exact(_))
                | CurvePoint2(CurvePointData2::Algebraic(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {}
            }
        }

        let support = chord.retained_support();
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
                break;
            }
            let Classification::Decided(point) =
                self.conservative_bounds_refined(refinement_steps, policy)
            else {
                continue;
            };
            let Classification::Decided(start) =
                algebraic_chord_endpoint_bounds_refined(support.start(), refinement_steps, policy)
            else {
                continue;
            };
            let Classification::Decided(end) =
                algebraic_chord_endpoint_bounds_refined(support.end(), refinement_steps, policy)
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let coordinate = |bounds: &Aabb2, axis| RealInterval::from_axis(bounds, axis);
            let start_x = coordinate(&start, Axis2::X);
            let start_y = coordinate(&start, Axis2::Y);
            let delta_x = coordinate(&end, Axis2::X).subtract(&start_x);
            let delta_y = coordinate(&end, Axis2::Y).subtract(&start_y);
            let point_x = coordinate(&point, Axis2::X).subtract(&start_x);
            let point_y = coordinate(&point, Axis2::Y).subtract(&start_y);
            let Some(cross) = delta_x.multiply(&point_y).and_then(|first| {
                delta_y
                    .multiply(&point_x)
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            let zero = Real::zero();
            let raw_side = if compare_reals(&cross.lower, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Greater)
            {
                Some(crate::classify::LineSide::Left)
            } else if compare_reals(&cross.upper, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                Some(crate::classify::LineSide::Right)
            } else if compare_reals(&cross.lower, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(&cross.upper, &zero, &CurveContext::STRICT)
                    == Some(std::cmp::Ordering::Equal)
            {
                Some(crate::classify::LineSide::On)
            } else {
                None
            };
            if let Some(side) = raw_side {
                return Ok(Classification::Decided(
                    if chord.retained_support_orientation_is_reversed() {
                        match side {
                            crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                            crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                            crate::classify::LineSide::On => crate::classify::LineSide::On,
                        }
                    } else {
                        side
                    },
                ));
            }
        }
        if policy.has_bounded_exact_predicate_budget() {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        let flat_side = if policy.permits_approximate_512() {
            self.flat_recursive_oriented_side_to_chord(chord, policy)?
        } else {
            None
        };
        if let Some(side) = flat_side {
            return Ok(Classification::Decided(side));
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    }

    pub(crate) fn conservative_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        if !self.accepts_policy(policy) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        // A conservative enclosure is construction evidence even when the
        // caller permits a terminal approximate topology decision. Preserve
        // its retained policy identity while requiring exact interval signs.
        policy
            .strict_predicate_pass(|| self.intersection_bounds_refined(refinement_steps, policy))
            .map_or(
                Classification::Uncertain(UncertaintyReason::Ordering),
                Classification::Decided,
            )
    }

    pub(super) fn intersection_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Option<Aabb2> {
        let first = self.data.first.retained_support();
        let second = self.data.second.retained_support();
        let endpoint_bounds = |point: &CurvePoint2| {
            let Classification::Decided(bounds) =
                algebraic_chord_endpoint_local_bounds_refined(point, refinement_steps, policy)
            else {
                return None;
            };
            // Only an outer enclosure is needed by the support determinant.
            // Rational endpoints avoid replaying unrelated scalar expressions
            // during interval arithmetic and continue tightening on demand.
            Some(
                bounds
                    .certified_rational_outer_envelope(refinement_steps)
                    .unwrap_or(bounds),
            )
        };
        if let (Some(first_direction), Some(second_direction)) = (
            first.certified_axis_direction(),
            second.certified_axis_direction(),
        ) && first_direction.axis() != second_direction.axis()
        {
            let constant_coordinate = |chord: &BezierAlgebraicChord2, axis: Axis2| {
                let bounds = endpoint_bounds(chord.start())?;
                Some(RealInterval::from_axis(&bounds, axis))
            };
            let vertical = if first_direction.axis() == Axis2::Y {
                first
            } else {
                second
            };
            let horizontal = if first_direction.axis() == Axis2::X {
                first
            } else {
                second
            };
            let x = constant_coordinate(vertical, Axis2::X)?;
            let y = constant_coordinate(horizontal, Axis2::Y)?;
            return Some(Aabb2::new_unchecked(
                Point2::new(x.lower, y.lower),
                Point2::new(x.upper, y.upper),
            ));
        }
        let first_start = endpoint_bounds(first.start())?;
        let first_end = endpoint_bounds(first.end())?;
        let second_start = endpoint_bounds(second.start())?;
        let second_end = endpoint_bounds(second.end())?;
        let coordinate = |bounds: &Aabb2, axis| RealInterval::from_axis(bounds, axis);
        let first_start_x = coordinate(&first_start, Axis2::X);
        let first_start_y = coordinate(&first_start, Axis2::Y);
        let first_delta_x = coordinate(&first_end, Axis2::X).subtract(&first_start_x);
        let first_delta_y = coordinate(&first_end, Axis2::Y).subtract(&first_start_y);
        let second_start_x = coordinate(&second_start, Axis2::X);
        let second_start_y = coordinate(&second_start, Axis2::Y);
        let second_delta_x = coordinate(&second_end, Axis2::X).subtract(&second_start_x);
        let second_delta_y = coordinate(&second_end, Axis2::Y).subtract(&second_start_y);
        let cross = |first_x: &RealInterval,
                     first_y: &RealInterval,
                     second_x: &RealInterval,
                     second_y: &RealInterval| {
            Some(
                first_x
                    .multiply(second_y)?
                    .subtract(&first_y.multiply(second_x)?),
            )
        };
        let denominator = cross(
            &first_delta_x,
            &first_delta_y,
            &second_delta_x,
            &second_delta_y,
        )?;
        let origin_delta_x = second_start_x.subtract(&first_start_x);
        let origin_delta_y = second_start_y.subtract(&first_start_y);
        let numerator = cross(
            &origin_delta_x,
            &origin_delta_y,
            &second_delta_x,
            &second_delta_y,
        )?;
        let parameter = numerator.divide(&denominator)?;
        let x = first_start_x.add(&parameter.multiply(&first_delta_x)?);
        let y = first_start_y.add(&parameter.multiply(&first_delta_y)?);
        if compare_reals(&x.lower, &x.upper, &CurveContext::STRICT)? == std::cmp::Ordering::Greater
            || compare_reals(&y.lower, &y.upper, &CurveContext::STRICT)?
                == std::cmp::Ordering::Greater
        {
            return None;
        }
        Some(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }

    pub(crate) fn same_point_evidence(
        &self,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if !self.accepts_policy(policy) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        // Contact reconciliation compares every newly discovered point with
        // earlier vertices.  Most pairs are unrelated, and their retained
        // construction boxes already prove that before either defining chord
        // needs a complete support-side predicate.  Only strict disjointness
        // is evidence here; overlapping boxes continue to exact incidence.
        let pair = CurvePoint2::from(self.clone());
        for refinement_steps in [0, 2, 4, 8] {
            let (Classification::Decided(pair_bounds), Classification::Decided(other_bounds)) = (
                algebraic_chord_endpoint_local_bounds_refined(&pair, refinement_steps, policy),
                algebraic_chord_endpoint_local_bounds_refined(other, refinement_steps, policy),
            ) else {
                continue;
            };
            if pair_bounds.overlaps(&other_bounds, &CurveContext::STRICT)
                == Classification::Decided(false)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "retained-point-equality",
                    "strict-box-disjointness-precedence",
                );
                return Classification::Decided(false);
            }
        }
        if let BezierAlgebraicChordPairPointLocation2::AnchorOrders {
            first_at_end,
            first,
            second_at_end,
            second,
            ..
        } = self.data.location
        {
            for (owner, anchor_at_end, pair_to_anchor) in [
                (&self.data.first, first_at_end, first),
                (&self.data.second, second_at_end, second),
            ] {
                if Self::anchor_separation_order_on_owner(
                    owner,
                    anchor_at_end,
                    pair_to_anchor,
                    other,
                    policy,
                )
                .is_some()
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "retained-point-equality",
                        "certified-offset-anchor-separation",
                    );
                    return Classification::Decided(false);
                }
            }
        }
        if policy.has_bounded_exact_predicate_budget() {
            // A speculative broad-phase pass may use the point's independently
            // refinable exact boxes, but it must not materialize both defining
            // support fields merely to decide optional endpoint incidence.
            // The complete caller retains represented-coordinate and
            // support-side authorities after this bounded pass declines.
            return Classification::Uncertain(UncertaintyReason::Predicate);
        }
        let classify = |chord: &BezierAlgebraicChord2| {
            if let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = other
                && let Ok(Some(side)) = point.oriented_side_to_analytic_tangent_chord(chord, policy)
            {
                return Classification::Decided(side);
            }
            let predicate = match BezierAlgebraicChordSupportPredicate2::try_new(chord, policy) {
                Ok(Classification::Decided(predicate)) => predicate,
                Ok(Classification::Uncertain(reason)) => {
                    return Classification::Uncertain(reason);
                }
                Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
            };
            match predicate.oriented_side(other, policy) {
                Ok(classification) => classification,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        };
        let incidence = match (classify(&self.data.first), classify(&self.data.second)) {
            (
                Classification::Decided(crate::classify::LineSide::On),
                Classification::Decided(crate::classify::LineSide::On),
            ) => Classification::Decided(true),
            (Classification::Decided(crate::classify::LineSide::Left), _)
            | (Classification::Decided(crate::classify::LineSide::Right), _)
            | (_, Classification::Decided(crate::classify::LineSide::Left))
            | (_, Classification::Decided(crate::classify::LineSide::Right)) => {
                Classification::Decided(false)
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                Classification::Uncertain(reason)
            }
        };
        if matches!(incidence, Classification::Decided(_)) {
            return incidence;
        }
        // A foreign retained point representation need not implement either
        // support-side predicate. Conservative exact boxes still prove
        // inequality without flattening either point into coordinates.
        let point = CurvePoint2::from(self.clone());
        match retained_point_evidence_equality_by_refinement(&point, other, policy) {
            decided @ Classification::Decided(_) => decided,
            Classification::Uncertain(_) => incidence,
        }
    }

    pub(super) fn cmp_on_chord_to_evidence(
        &self,
        chord: &BezierAlgebraicChord2,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "correlated chord point was replayed under a different predicate policy".into(),
            ));
        }
        if matches!(
            other,
            CurvePoint2(CurvePointData2::AlgebraicChordPair(point))
                if self == point
        ) {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        let correlated_other = match other {
            CurvePoint2(CurvePointData2::AlgebraicChordPair(other)) => Some(other),
            CurvePoint2(CurvePointData2::Exact(_))
            | CurvePoint2(CurvePointData2::Algebraic(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        let shares_first_support = chord.shares_retained_support(&self.data.first)
            || chord.support_collinearity(&self.data.first, policy)?
                == Classification::Decided(true);
        let shares_second_support = chord.shares_retained_support(&self.data.second)
            || chord.support_collinearity(&self.data.second, policy)?
                == Classification::Decided(true);
        let (first_sides, second_sides, first_anchor, second_anchor, retained_pair_cross) =
            match self.data.location {
                BezierAlgebraicChordPairPointLocation2::EndpointSides {
                    first,
                    second,
                    tangent_cross_sign,
                } => (
                    Some(first),
                    Some(second),
                    None,
                    None,
                    Some(tangent_cross_sign),
                ),
                BezierAlgebraicChordPairPointLocation2::AnchorOrders {
                    first_at_end,
                    first,
                    second_at_end,
                    second,
                    tangent_cross_sign,
                } => (
                    None,
                    None,
                    Some((first_at_end, first)),
                    Some((second_at_end, second)),
                    Some(tangent_cross_sign),
                ),
            };
        let (owner, opposite_chord, owner_sides, owner_anchor) = if shares_first_support {
            (
                &self.data.first,
                &self.data.second,
                first_sides,
                first_anchor,
            )
        } else if shares_second_support {
            (
                &self.data.second,
                &self.data.first,
                second_sides,
                second_anchor,
            )
        } else {
            let point = CurvePoint2::from(self.clone());
            let order = algebraic_chord_point_coordinate_order(
                &point,
                other,
                chord.data.parameter_axis.axis,
                policy,
            )?;
            return Ok(if chord.data.parameter_axis.coordinate_increases {
                order
            } else {
                order.map(std::cmp::Ordering::reverse)
            });
        };
        if let Some((at_end, order)) = owner_anchor
            && let Some(pair_to_other) =
                Self::anchor_separation_order_on_owner(owner, at_end, order, other, policy)
        {
            let reversed = owner.shared_tangent_orientation(chord).unwrap_or(
                owner.data.parameter_axis.coordinate_increases
                    != chord.data.parameter_axis.coordinate_increases,
            );
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-order",
                "certified-offset-anchor-separation",
            );
            return Ok(Classification::Decided(if reversed {
                pair_to_other.reverse()
            } else {
                pair_to_other
            }));
        }
        // Event sorting needs only the order on this chord's certified
        // injective axis.  Most unrelated contacts separate in their native
        // construction boxes, which is a complete strict order certificate
        // and avoids asking the opposite support for a much larger incidence
        // field.  Overlap (including equality) continues below.
        // Retained procedural points are the converse case: their exact
        // support-side certificate is constant-size, while refining one
        // analytic coordinate can evaluate the complete recursive tower.
        let retained_procedural_side = if matches!(
            other,
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        ) {
            opposite_chord.retained_procedural_point_side(other, policy)?
        } else {
            None
        };
        let pair = CurvePoint2::from(self.clone());
        if retained_procedural_side.is_none() {
            for refinement_steps in [0, 2, 4, 8, 16] {
                let (Classification::Decided(pair_bounds), Classification::Decided(other_bounds)) = (
                    algebraic_chord_endpoint_local_bounds_refined(
                        &pair,
                        refinement_steps,
                        &policy.strict_counterpart(),
                    ),
                    algebraic_chord_endpoint_local_bounds_refined(
                        other,
                        refinement_steps,
                        &policy.strict_counterpart(),
                    ),
                ) else {
                    continue;
                };
                if let Some(order) = algebraic_chord_bounds_axis_order(
                    &pair_bounds,
                    &other_bounds,
                    chord.data.parameter_axis.axis,
                ) && order != std::cmp::Ordering::Equal
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair-order",
                        "strict-axis-box-precedence",
                    );
                    return Ok(Classification::Decided(
                        if chord.data.parameter_axis.coordinate_increases {
                            order
                        } else {
                            order.reverse()
                        },
                    ));
                }
            }
        }
        // Support-side incidence is the cheaper authority and proves equality.
        // When a foreign retained representation cannot enter that predicate,
        // the carrier's certified injective axis still orders exact boxes.
        let compare_coordinates = || {
            let point = CurvePoint2::from(self.clone());
            let order = algebraic_chord_point_coordinate_order(
                &point,
                other,
                chord.data.parameter_axis.axis,
                policy,
            )?;
            Ok(if chord.data.parameter_axis.coordinate_increases {
                order
            } else {
                order.map(std::cmp::Ordering::reverse)
            })
        };
        if policy.has_bounded_exact_predicate_budget()
            && let Some(other) = correlated_other
        {
            // Both points already expose independently refinable exact boxes
            // in the common chord's injective coordinate. Keep the
            // speculative exact pass bounded there: rebuilding an
            // opposite-support incidence system can promote the four
            // retained endpoint fields into an unbounded compositum before
            // the owning complete predicate is allowed to run.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-order",
                "bounded-correlated-pair",
            );
            return self.cmp_on_common_chord(chord, other, policy);
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            && let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = other
        {
            let analytic = |endpoint: &CurvePoint2| {
                let CurvePoint2(CurvePointData2::AnalyticParallel(endpoint)) = endpoint else {
                    return None;
                };
                Some((
                    endpoint.data.parallel == point.data.parallel,
                    endpoint.data.parameter == point.data.parameter,
                    endpoint.data.frame_tangent == point.data.frame_tangent,
                    endpoint.data.translation_x == point.data.translation_x
                        && endpoint.data.translation_y == point.data.translation_y,
                    real_sign(&endpoint.data.tangent_distance, &CurveContext::STRICT),
                ))
            };
            eprintln!(
                "pair order opposite analytic endpoints={:?} same-endpoint-parameter={} same-parallel={} same-frame={} same-translation={}",
                [opposite_chord.start(), opposite_chord.end()].map(analytic),
                match (opposite_chord.start(), opposite_chord.end()) {
                    (
                        CurvePoint2(CurvePointData2::AnalyticParallel(first)),
                        CurvePoint2(CurvePointData2::AnalyticParallel(second)),
                    ) => first.data.parameter == second.data.parameter,
                    _ => false,
                },
                match (opposite_chord.start(), opposite_chord.end()) {
                    (
                        CurvePoint2(CurvePointData2::AnalyticParallel(first)),
                        CurvePoint2(CurvePointData2::AnalyticParallel(second)),
                    ) => first.data.parallel == second.data.parallel,
                    _ => false,
                },
                match (opposite_chord.start(), opposite_chord.end()) {
                    (
                        CurvePoint2(CurvePointData2::AnalyticParallel(first)),
                        CurvePoint2(CurvePointData2::AnalyticParallel(second)),
                    ) => first.data.frame_tangent == second.data.frame_tangent,
                    _ => false,
                },
                match (opposite_chord.start(), opposite_chord.end()) {
                    (
                        CurvePoint2(CurvePointData2::AnalyticParallel(first)),
                        CurvePoint2(CurvePointData2::AnalyticParallel(second)),
                    ) =>
                        first.data.translation_x == second.data.translation_x
                            && first.data.translation_y == second.data.translation_y,
                    _ => false,
                },
            );
        }
        // Construction-local incidence can classify retained analytic tangent
        // and normal-offset supports without representing their endpoints in
        // one tensor field.  Consult it before eagerly building the generic
        // support predicate; the latter remains the complete fallback for
        // every representation the local kernel does not recognize.
        let side = if let Some(side) = retained_procedural_side {
            side
        } else if let Some(side) = opposite_chord.retained_procedural_point_side(other, policy)? {
            side
        } else {
            let opposite =
                match BezierAlgebraicChordSupportPredicate2::try_new(opposite_chord, policy)? {
                    Classification::Decided(predicate) => predicate,
                    Classification::Uncertain(reason) => {
                        return match compare_coordinates()? {
                            decided @ Classification::Decided(_) => Ok(decided),
                            Classification::Uncertain(_) => Ok(Classification::Uncertain(reason)),
                        };
                    }
                };
            match opposite.oriented_side(other, policy)? {
                Classification::Decided(side) => side,
                Classification::Uncertain(reason) => {
                    if let Some(other) = correlated_other {
                        return self.cmp_on_common_chord(chord, other, policy);
                    }
                    return match compare_coordinates()? {
                        decided @ Classification::Decided(_) => Ok(decided),
                        Classification::Uncertain(_) => Ok(Classification::Uncertain(reason)),
                    };
                }
            }
        };
        if side == crate::classify::LineSide::On {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if owner_sides.is_none_or(|sides| sides[0] == sides[1]) {
            // A supporting-line intersection may lie beyond both endpoints of
            // its finite witness. Equal endpoint sides then prove only that
            // the witness is one-sided; treating index zero as "before" would
            // reverse every miter leg extended past that witness. Along the
            // requested chord traversal, a positive chord/opposite cross
            // enters the right half-plane after the intersection, while a
            // negative cross enters the left half-plane.
            let retained_cross = retained_pair_cross.map(|mut sign| {
                if !shares_first_support {
                    sign = product_sign(sign, RealSign::Negative);
                }
                let owner_reversed =
                    if owner.data.parameter_axis.axis == chord.data.parameter_axis.axis {
                        owner.data.parameter_axis.coordinate_increases
                            != chord.data.parameter_axis.coordinate_increases
                    } else {
                        owner.shared_tangent_orientation(chord).unwrap_or(false)
                    };
                if owner_reversed {
                    sign = product_sign(sign, RealSign::Negative);
                }
                sign
            });
            let cross = match retained_cross.map_or_else(
                || chord.tangent_cross_sign(opposite_chord, policy),
                |sign| {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair-order",
                        "retained-offset-tangent-cross",
                    );
                    Ok(Classification::Decided(sign))
                },
            )? {
                Classification::Decided(RealSign::Positive) => crate::classify::LineSide::Right,
                Classification::Decided(RealSign::Negative) => crate::classify::LineSide::Left,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Ok(Classification::Decided(if side == cross {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }));
        }
        let owner_sides = owner_sides.expect("distinct endpoint sides were retained");
        let order = if side == owner_sides[0] {
            std::cmp::Ordering::Greater
        } else if side == owner_sides[1] {
            std::cmp::Ordering::Less
        } else {
            if let Some(other) = correlated_other {
                return self.cmp_on_common_chord(chord, other, policy);
            }
            return compare_coordinates();
        };
        Ok(Classification::Decided(
            if owner.data.parameter_axis.coordinate_increases
                == chord.data.parameter_axis.coordinate_increases
            {
                order
            } else {
                order.reverse()
            },
        ))
    }

    pub(super) fn cmp_on_common_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if !self.accepts_policy(policy) || !other.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "correlated chord points were compared under a different predicate policy".into(),
            ));
        }
        if self == other {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        let axis = chord.data.parameter_axis;
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (Classification::Decided(first), Classification::Decided(second)) = (
                self.conservative_bounds_refined(refinement_steps, policy),
                other.conservative_bounds_refined(refinement_steps, policy),
            ) else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let (first_lower, first_upper, second_lower, second_upper) = match axis.axis {
                Axis2::X => (
                    first.min().x(),
                    first.max().x(),
                    second.min().x(),
                    second.max().x(),
                ),
                Axis2::Y => (
                    first.min().y(),
                    first.max().y(),
                    second.min().y(),
                    second.max().y(),
                ),
            };
            let coordinate_order = if compare_reals(first_upper, second_lower, policy)
                == Some(std::cmp::Ordering::Less)
            {
                Some(std::cmp::Ordering::Less)
            } else if compare_reals(first_lower, second_upper, policy)
                == Some(std::cmp::Ordering::Greater)
            {
                Some(std::cmp::Ordering::Greater)
            } else {
                None
            };
            if let Some(order) = coordinate_order {
                return Ok(Classification::Decided(if axis.coordinate_increases {
                    order
                } else {
                    order.reverse()
                }));
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(std::cmp::Ordering::Equal))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Ordering))
        }
    }
}

impl BezierAlgebraicChordParallelPoint2 {
    pub(super) fn accepts_policy(&self, policy: &CurveContext) -> bool {
        policy.accepts_retained_policy(self.data.policy)
    }

    pub(crate) fn new_pair(
        source: BezierAlgebraicChord2,
        distance: Real,
        translation_x: Real,
        translation_y: Real,
        policy: &CurveContext,
    ) -> (Self, Self) {
        let data = Arc::new(BezierAlgebraicChordParallelData2 {
            source,
            source_point: None,
            distance,
            translation_x,
            translation_y,
            direction: BezierAlgebraicChordUnitDisplacement2::LeftNormal,
            policy: policy.retained_object_policy(),
            recursive_points: OnceLock::new(),
        });
        (
            Self {
                data: data.clone(),
                at_end: false,
            },
            Self { data, at_end: true },
        )
    }

    pub(super) fn tangent_endpoint(
        source: BezierAlgebraicChord2,
        at_end: bool,
        distance: Real,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicChordParallelData2 {
                source,
                source_point: None,
                distance,
                translation_x: Real::zero(),
                translation_y: Real::zero(),
                direction: BezierAlgebraicChordUnitDisplacement2::Tangent,
                policy: policy.retained_object_policy(),
                recursive_points: OnceLock::new(),
            }),
            at_end,
        }
    }

    pub(super) fn normal_displaced_point(
        source: BezierAlgebraicChord2,
        source_point: CurvePoint2,
        distance: Real,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicChordParallelData2 {
                source,
                source_point: Some(Arc::new(source_point)),
                distance,
                translation_x: Real::zero(),
                translation_y: Real::zero(),
                direction: BezierAlgebraicChordUnitDisplacement2::LeftNormal,
                policy: policy.retained_object_policy(),
                recursive_points: OnceLock::new(),
            }),
            at_end: false,
        }
    }

    pub(super) fn tangent_displaced_point(
        source: BezierAlgebraicChord2,
        source_point: CurvePoint2,
        distance: Real,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicChordParallelData2 {
                source,
                source_point: Some(Arc::new(source_point)),
                distance,
                translation_x: Real::zero(),
                translation_y: Real::zero(),
                direction: BezierAlgebraicChordUnitDisplacement2::Tangent,
                policy: policy.retained_object_policy(),
                recursive_points: OnceLock::new(),
            }),
            at_end: false,
        }
    }

    pub(super) fn source_endpoint(&self) -> &CurvePoint2 {
        if let Some(point) = self.data.source_point.as_deref() {
            return point;
        }
        if self.at_end {
            self.data.source.end()
        } else {
            self.data.source.start()
        }
    }

    /// Returns a direction-equivalent endpoint pair after collapsing a
    /// Boolean-published finite rational contact back to the authored chord
    /// endpoint on the same ray. The physical displacement origin remains
    /// `source_endpoint`; only the normalized direction uses this smaller
    /// retained carrier.
    pub(super) fn source_direction_endpoints<'a>(
        &'a self,
        policy: &CurveContext,
    ) -> [&'a CurvePoint2; 2] {
        self.data.source.direction_endpoints(policy)
    }

    /// Materializes this procedural point when its source already supplies
    /// both a represented endpoint and an exact retained unit tangent.
    /// General normalized displacements stay lazy; this adapter only removes
    /// a redundant normalization layer whose exact components are already
    /// present as canonical `Real`s.
    pub(super) fn strict_exact_point(&self, policy: &CurveContext) -> Option<Point2> {
        if !self.accepts_policy(policy) {
            return None;
        }
        let source = self.source_endpoint().coordinates()?;
        let (tangent_x, tangent_y) = self.data.source.certified_unit_tangent()?;
        let (unit_x, unit_y) = match self.data.direction {
            BezierAlgebraicChordUnitDisplacement2::LeftNormal => (-tangent_y, tangent_x),
            BezierAlgebraicChordUnitDisplacement2::Tangent => (tangent_x, tangent_y),
        };
        Some(source.translated(
            &unit_x * &self.data.distance + &self.data.translation_x,
            &unit_y * &self.data.distance + &self.data.translation_y,
        ))
    }

    pub(super) fn recursive_projective_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParallelRecursiveFrame2>>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "an algebraic chord displacement entered a recursive field under a different predicate policy"
                    .into(),
            ));
        }
        let [source_start, source_end] = self.source_direction_endpoints(policy);
        let points = [self.source_endpoint(), source_start, source_end];
        let points = match recursive_projective_evidence_points(&points, policy)? {
            Classification::Decided(Some(points)) => points,
            Classification::Decided(None) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [origin, start, end]: [BezierRecursiveQuadraticProjectivePoint2; 3] = points
            .try_into()
            .expect("a recursive chord displacement retains three source points");
        let origin = match positive_recursive_projective_point(origin)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start = match positive_recursive_projective_point(start)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match positive_recursive_projective_point(end)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some((dx, dy, _)) = end.difference_numerators(&start) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(speed_squared) = dx.square().and_then(|x| x.add(&dy.square()?)) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        // `source_direction_endpoints` belongs to an already-validated
        // nondegenerate chord. Its squared speed is therefore certified
        // nonzero independently of this recursive representation; interval
        // separation is complete and avoids constructing a dense norm merely
        // to re-prove source validity.
        match speed_squared.sign_with_nonzero_certificate()? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "an algebraic chord displacement retained a zero source direction".into(),
                ));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "an algebraic chord displacement retained a negative squared speed".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let parent = origin.denominator.field();
        let (field, speed) =
            if let Some(speed) = parent.retained_positive_square_root(&speed_squared) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-chord-displacement-speed",
                    "retained-generator",
                );
                (parent.clone(), speed)
            } else if let Some(speed) = speed_squared
                .exact_real_value_with_retained_witnesses()
                .and_then(|speed_squared| speed_squared.sqrt().ok())
                .and_then(|speed| parent.constant(speed))
                .filter(|speed| {
                    speed
                        .square()
                        .and_then(|square| square.subtract(&speed_squared))
                        .is_some_and(|difference| difference.is_structurally_zero())
                })
            {
                // A scalar speed may replace a generator only when its
                // square still replays against the retained radicand.
                // Otherwise keep the defining relation over selected source
                // axes; the generator also retains its scalar witness for
                // cheap evaluation without discarding that relation.
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-chord-displacement-speed",
                    "exact-real-witness",
                );
                (parent.clone(), speed)
            } else {
                let Some(field) = parent.extension(speed_squared) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some(speed) = field.element(
                    parent.constant(Real::zero()).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive chord displacement lost its zero coefficient".into(),
                        )
                    })?,
                    parent.constant(Real::one()).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive chord displacement lost its unit coefficient".into(),
                        )
                    })?,
                ) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-chord-displacement-speed",
                    "new-generator",
                );
                (field, speed)
            };
        let Some((origin, start, end, dx, dy, speed)) = (|| {
            Some((
                origin.lifted_to(&field)?,
                start.lifted_to(&field)?,
                end.lifted_to(&field)?,
                field.lift(&dx)?,
                field.lift(&dy)?,
                field.lift(&speed)?,
            ))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let (unit_x, unit_y) = match self.data.direction {
            BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                let Some(unit_x) = dy.scale(&Real::from(-1_i8)) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                (unit_x, dx)
            }
            BezierAlgebraicChordUnitDisplacement2::Tangent => (dx, dy),
        };
        let displaced = |origin: &BezierRecursiveQuadraticProjectivePoint2| {
            let denominator = origin.denominator.multiply(&speed)?;
            let coordinate = |coordinate_origin: &BezierRecursiveQuadraticValue2,
                              unit: &BezierRecursiveQuadraticValue2,
                              translation: &Real| {
                coordinate_origin
                    .multiply(&speed)?
                    .add(
                        &origin
                            .denominator
                            .multiply(unit)?
                            .scale(&self.data.distance)?,
                    )?
                    .add(&denominator.scale(translation)?)
            };
            Some(BezierRecursiveQuadraticProjectivePoint2 {
                x: coordinate(&origin.x, &unit_x, &self.data.translation_x)?,
                y: coordinate(&origin.y, &unit_y, &self.data.translation_y)?,
                denominator,
            })
        };
        let direction_endpoints = [start.clone(), end.clone()];
        let origins = if self.data.source_point.is_some() {
            [origin.clone(), origin]
        } else {
            [start, end]
        };
        let [Some(first), Some(second)] = origins.each_ref().map(displaced) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let points = [first, second];
        let _ = self.data.recursive_points.set(points.clone());
        Ok(Classification::Decided(Some(
            BezierAlgebraicChordParallelRecursiveFrame2 {
                displaced: points,
                direction_endpoints,
            },
        )))
    }

    /// Evaluates this unit-tangent or unit-normal displacement in the
    /// recursive field already owned by its source geometry. The source chord
    /// contributes one positive speed generator shared by both Cartesian
    /// coordinates; flattening either case into independent coordinates would
    /// lose that correlation and multiply the algebraic degree.
    pub(super) fn recursive_projective_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "an algebraic chord displacement entered a recursive field under a different predicate policy"
                    .into(),
            ));
        }
        let index = usize::from(self.data.source_point.is_none() && self.at_end);
        if let Some(points) = self.data.recursive_points.get() {
            return Ok(Classification::Decided(Some(points[index].clone())));
        }
        Ok(self
            .recursive_projective_frame(policy)?
            .map(|frame| frame.map(|frame| frame.displaced[index].clone())))
    }

    pub(super) fn represented_coordinates(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "an algebraic chord displacement was represented under a different predicate policy"
                    .into(),
            ));
        }
        if let Some(cardinal) = self.strict_cardinal_point_evidence(policy)? {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-chord-parallel",
                "cardinal-displacement-canonicalized",
            );
            return represented_point_evidence_coordinates(&cardinal, policy);
        }
        let origin = match represented_point_evidence_coordinates(self.source_endpoint(), policy)? {
            Classification::Decided(origin) => origin,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                {
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "represented-chord-parallel-blocker",
                        "origin",
                    );
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "represented-chord-parallel-origin-kind",
                        match self.source_endpoint() {
                            CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                            CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                            CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "chord-pair",
                            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp-chord",
                            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => {
                                "cusp-chord-derived"
                            }
                            CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {
                                "chord-parallel"
                            }
                            CurvePoint2(CurvePointData2::AnalyticParallel(_)) => {
                                "analytic-parallel"
                            }
                            CurvePoint2(
                                CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_),
                            ) => "similarity",
                        },
                    );
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "represented-chord-parallel-origin-mode",
                        match (
                            self.data.source_point.is_some(),
                            self.data.direction,
                            self.at_end,
                        ) {
                            (false, BezierAlgebraicChordUnitDisplacement2::LeftNormal, false) => {
                                "endpoint-start-normal"
                            }
                            (false, BezierAlgebraicChordUnitDisplacement2::LeftNormal, true) => {
                                "endpoint-end-normal"
                            }
                            (false, BezierAlgebraicChordUnitDisplacement2::Tangent, false) => {
                                "endpoint-start-tangent"
                            }
                            (false, BezierAlgebraicChordUnitDisplacement2::Tangent, true) => {
                                "endpoint-end-tangent"
                            }
                            (true, BezierAlgebraicChordUnitDisplacement2::LeftNormal, _) => {
                                "interior-normal"
                            }
                            (true, BezierAlgebraicChordUnitDisplacement2::Tangent, _) => {
                                "interior-tangent"
                            }
                        },
                    );
                }
                return Ok(Classification::Uncertain(reason));
            }
        };
        let unit =
            match represented_chord_unit_direction(&self.data.source, self.data.direction, policy)?
            {
                Classification::Decided(unit) => unit,
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "represented-chord-parallel-blocker",
                        "unit-direction",
                    );
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_DISABLE_RECURSIVE_REPRESENTED_POINT")
                        .is_some()
                    {
                        return Ok(Classification::Uncertain(reason));
                    }
                    // The standalone direction adapter can fail when it
                    // independently selects `dx`, `dy`, and
                    // `sqrt(dx^2 + dy^2)`: doing so forgets that all three
                    // values belong to one selected recursive field.  The
                    // displacement authority already retains that correlated
                    // projective point.  Publish its exact Cartesian roots at
                    // this cold interoperability boundary instead.  Suppress
                    // APPROXIMATE_512 throughout the conversion so an
                    // approximate terminal can never select persistent point
                    // coordinates.
                    let recursive = policy.strict_predicate_pass(|| {
                        match self.recursive_projective_point(policy)? {
                            Classification::Decided(Some(point)) => {
                                point.represented_coordinates(policy)
                            }
                            Classification::Decided(None) => Ok(Classification::Uncertain(reason)),
                            Classification::Uncertain(recursive_reason) => {
                                Ok(Classification::Uncertain(recursive_reason))
                            }
                        }
                    })?;
                    return Ok(recursive);
                }
            };
        let x = represented_affine_coordinate(
            &[(&origin[0], &Real::one()), (&unit[0], &self.data.distance)],
            &self.data.translation_x,
        );
        let y = represented_affine_coordinate(
            &[(&origin[1], &Real::one()), (&unit[1], &self.data.distance)],
            &self.data.translation_y,
        );
        Ok(match (x, y) {
            (Classification::Decided(x), Classification::Decided(y)) => {
                Classification::Decided([x, y].map(|coordinate| {
                    hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                        .unwrap_or(coordinate)
                }))
            }
            (Classification::Uncertain(UncertaintyReason::Unsupported), _)
            | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-parallel-blocker",
                    "affine-coordinate",
                );
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            _ => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-parallel-blocker",
                    "affine-predicate",
                );
                Classification::Uncertain(UncertaintyReason::Predicate)
            }
        })
    }

    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        self.at_end == other.at_end && Arc::ptr_eq(&self.data, &other.data)
    }

    pub(super) fn shares_carrier(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.source == other.data.source
                && self.data.source_point == other.data.source_point
                && self.data.distance == other.data.distance
                && self.data.translation_x == other.data.translation_x
                && self.data.translation_y == other.data.translation_y
                && self.data.direction == other.data.direction
                && self.data.policy == other.data.policy)
    }

    /// Recognizes two independently replayed instances of the same procedural
    /// normal-offset support. Reversing the base tangent reverses the signed
    /// left-normal distance, but leaves the physical affine line unchanged.
    pub(super) fn shares_normal_offset_carrier(&self, other: &Self) -> bool {
        if self.data.source_point.is_some()
            || other.data.source_point.is_some()
            || self.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
            || other.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
            || self.data.translation_x != other.data.translation_x
            || self.data.translation_y != other.data.translation_y
            || self.data.policy != other.data.policy
        {
            return false;
        }
        self.data
            .source
            .retained_support_orientation_to(&other.data.source)
            .is_some_and(|reversed| {
                if reversed {
                    self.data.distance == -&other.data.distance
                } else {
                    self.data.distance == other.data.distance
                }
            })
    }

    /// Signs this procedural normal-offset endpoint against another
    /// procedural normal-offset support in one shared recursive tower.
    ///
    /// Expanding three displaced Cartesian points independently introduces
    /// duplicate speed radicals.  If `u` is this point's source direction,
    /// `v` is the support direction, `D` joins their source origins, and
    /// `su, sv` are their positive speeds, the required oriented area has the
    /// same sign as
    ///
    /// `cross(v,D)*su + point_distance*dot(v,u)
    ///                   - support_distance*sv*su`.
    ///
    /// Homogeneous source denominators are included below as positive scale
    /// factors.  This retains both normalization sheets exactly while adding
    /// only the two speed generators actually used by the scalar predicate.
    pub(super) fn normal_offset_oriented_side_to_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<crate::classify::LineSide>>> {
        if !self.accepts_policy(policy)
            || self.data.source_point.is_some()
            || self.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
        {
            return Ok(Classification::Decided(None));
        }
        let support_chord = chord.retained_support();
        let Some(support) = chord_parallel_support_source(support_chord, policy)? else {
            return Ok(Classification::Decided(None));
        };
        if support.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal {
            return Ok(Classification::Decided(None));
        }
        let point_source = self.data.source.retained_support();
        let support_source = support.source.retained_support();
        let points = [
            point_source.start(),
            point_source.end(),
            support_source.start(),
            support_source.end(),
        ];
        let points = match recursive_projective_evidence_points(&points, policy)? {
            Classification::Decided(Some(points)) => points,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [point_start, point_end, support_start, support_end]: [
            BezierRecursiveQuadraticProjectivePoint2;
            4
        ] = points
            .try_into()
            .expect("two procedural supports retain four source endpoints");
        let point_origin = if self.at_end {
            &point_end
        } else {
            &point_start
        };
        let (Some((u_x, u_y, _)), Some((v_x, v_y, _)), Some((mut d_x, mut d_y, d_denominator))) = (
            point_end.difference_numerators(&point_start),
            support_end.difference_numerators(&support_start),
            point_origin.difference_numerators(&support_start),
        ) else {
            return Ok(Classification::Decided(None));
        };
        let translation_x = &self.data.translation_x - &support.translation_x;
        let translation_y = &self.data.translation_y - &support.translation_y;
        let (Some(translated_x), Some(translated_y)) = (
            d_x.add(&d_denominator.scale(&translation_x).ok_or_else(|| {
                CurveError::Topology(
                    "a procedural support translation exceeded its recursive field budget".into(),
                )
            })?),
            d_y.add(&d_denominator.scale(&translation_y).ok_or_else(|| {
                CurveError::Topology(
                    "a procedural support translation exceeded its recursive field budget".into(),
                )
            })?),
        ) else {
            return Ok(Classification::Decided(None));
        };
        d_x = translated_x;
        d_y = translated_y;
        let Some(u_squared) = u_x.square().and_then(|x| x.add(&u_y.square()?)) else {
            return Ok(Classification::Decided(None));
        };
        let Some(v_squared) = v_x.square().and_then(|x| x.add(&v_y.square()?)) else {
            return Ok(Classification::Decided(None));
        };
        for squared in [&u_squared, &v_squared] {
            // Both vectors are directions of validated retained supports, so
            // their squared speeds have independent nonzero certificates.
            // Exact interval separation is complete on that domain.
            match squared.sign_with_nonzero_certificate()? {
                Classification::Decided(RealSign::Positive) => {}
                Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "a procedural normal-offset support retained a nonpositive speed".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let base = u_x.field();
        let Some(u_field) = base.extension(u_squared) else {
            return Ok(Classification::Decided(None));
        };
        let Some(u_speed) = u_field.element(
            base.constant(Real::zero()).ok_or_else(|| {
                CurveError::Topology("a procedural speed lacked a zero coefficient".into())
            })?,
            base.constant(Real::one()).ok_or_else(|| {
                CurveError::Topology("a procedural speed lacked a unit coefficient".into())
            })?,
        ) else {
            return Ok(Classification::Decided(None));
        };
        let Some(v_squared) = u_field.lift(&v_squared) else {
            return Ok(Classification::Decided(None));
        };
        let Some(field) = u_field.extension(v_squared) else {
            return Ok(Classification::Decided(None));
        };
        let Some(v_speed) = field.element(
            u_field.constant(Real::zero()).ok_or_else(|| {
                CurveError::Topology("a procedural speed lacked a zero coefficient".into())
            })?,
            u_field.constant(Real::one()).ok_or_else(|| {
                CurveError::Topology("a procedural speed lacked a unit coefficient".into())
            })?,
        ) else {
            return Ok(Classification::Decided(None));
        };
        let lift = |value: &BezierRecursiveQuadraticValue2| field.lift(value);
        let (
            Some(u_x),
            Some(u_y),
            Some(v_x),
            Some(v_y),
            Some(d_x),
            Some(d_y),
            Some(d_denominator),
            Some(u_speed),
        ) = (
            lift(&u_x),
            lift(&u_y),
            lift(&v_x),
            lift(&v_y),
            lift(&d_x),
            lift(&d_y),
            lift(&d_denominator),
            lift(&u_speed),
        )
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(value) = (|| {
            let cross = v_x.multiply(&d_y)?.subtract(&v_y.multiply(&d_x)?)?;
            let dot = v_x.multiply(&u_x)?.add(&v_y.multiply(&u_y)?)?;
            let point_normal = dot.multiply(&d_denominator)?.scale(&self.data.distance)?;
            let support_normal = v_speed
                .multiply(&d_denominator)?
                .multiply(&u_speed)?
                .scale(&support.distance)?;
            cross
                .multiply(&u_speed)?
                .add(&point_normal)?
                .subtract(&support_normal)
        })() else {
            return Ok(Classification::Decided(None));
        };
        let sign = match value.sign(policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut side = match sign {
            RealSign::Positive => crate::classify::LineSide::Left,
            RealSign::Negative => crate::classify::LineSide::Right,
            RealSign::Zero => crate::classify::LineSide::On,
        };
        let Some(reversed) = support_source.shared_tangent_orientation(chord) else {
            return Ok(Classification::Decided(None));
        };
        if reversed {
            side = match side {
                crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                crate::classify::LineSide::On => crate::classify::LineSide::On,
            };
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-side-kernel",
            "procedural-normal-offset-recursive-scalar",
        );
        Ok(Classification::Decided(Some(side)))
    }

    /// Separates the mixed chord-normal/analytic-tangent determinant without
    /// embedding the analytic hodograph through the complete recursive chord
    /// tower. Every interval is an exact enclosure. STRICT retains the exact
    /// recursive fallback when they overlap; APPROXIMATE_512 may interpret
    /// only the completed 512-bit scalar enclosure as equality.
    pub(super) fn analytic_tangent_side_interval_sign(
        &self,
        source: &BezierAnalyticParallelPoint2,
        support: &BezierAnalyticParallelPoint2,
        parameter: &BezierRecursiveProjectiveParameter2,
        anchor_index: usize,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        if self.data.source_point.is_some() {
            return Ok(None);
        }
        let point_index = usize::from(self.at_end);
        let (tangent_x_coefficients, tangent_y_coefficients) =
            support.frame_tangent_power_basis()?;
        let strict = &CurveContext::STRICT;
        let exact = |value: &Real| RealInterval {
            lower: value.clone(),
            upper: value.clone(),
        };
        let scale = |value: &RealInterval, scalar: &Real| value.multiply(&exact(scalar));
        let mut parameter_lower = parameter.data.lower.clone();
        let mut parameter_upper = parameter.data.upper.clone();
        let source_parameter_interval = |refinement_steps| -> CurveResult<Option<RealInterval>> {
            Ok(Some(match &source.data.parameter {
                BezierAnalyticParallelPointParameter2::Bezier(parameter) => {
                    let parameter = parameter
                        .clone()
                        .refined_isolating_interval(refinement_steps, policy);
                    RealInterval::from_parameter(&parameter)
                }
                BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => {
                    let parameter = match parameter.refined(refinement_steps, policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                    RealInterval {
                        lower: parameter.root().lower.clone(),
                        upper: parameter.root().upper.clone(),
                    }
                }
                BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                    let parameter = match parameter.refined(refinement_steps, policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                    let (lower, upper) = parameter.isolating_bounds();
                    RealInterval {
                        lower: lower.clone(),
                        upper: upper.clone(),
                    }
                }
            }))
        };
        let bounded_exact_pass = policy.has_bounded_exact_predicate_budget();
        let refinement_schedule: &[usize] = if bounded_exact_pass {
            &[0, 2, 4, 8, 16, 32, 64, 128]
        } else {
            &[0, 2, 4, 8, 16, 32, 64, 128, 256, 512]
        };
        let mut terminal_refined = false;
        for &refinement_steps in refinement_schedule {
            let refined_parameter = match parameter.refined(refinement_steps, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(_) => {
                    continue;
                }
            };
            if compare_reals(
                &refined_parameter.data.lower,
                &parameter_lower,
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Greater)
            {
                parameter_lower = refined_parameter.data.lower.clone();
            }
            if compare_reals(
                &refined_parameter.data.upper,
                &parameter_upper,
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Less)
            {
                parameter_upper = refined_parameter.data.upper.clone();
            }
            let parameter = RealInterval {
                lower: parameter_lower.clone(),
                upper: parameter_upper.clone(),
            };
            let Some(source_parameter) = source_parameter_interval(refinement_steps)? else {
                continue;
            };
            let [direction_start, direction_end] =
                self.source_direction_endpoints(policy).map(|point| {
                    algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
                });
            let (Classification::Decided(direction_start), Classification::Decided(direction_end)) =
                (direction_start, direction_end)
            else {
                continue;
            };
            let direction_x = RealInterval::from_axis(&direction_end, Axis2::X)
                .subtract(&RealInterval::from_axis(&direction_start, Axis2::X));
            let direction_y = RealInterval::from_axis(&direction_end, Axis2::Y)
                .subtract(&RealInterval::from_axis(&direction_start, Axis2::Y));
            let (Some(v_x), Some(v_y)) = (
                RealInterval::evaluate_power_basis(tangent_x_coefficients, &parameter),
                RealInterval::evaluate_power_basis(tangent_y_coefficients, &parameter),
            ) else {
                continue;
            };
            let physical_cross = if point_index == anchor_index {
                exact(&Real::zero())
            } else {
                let source_bounds = analytic_parallel_point_bounds_over_interval_with_tangent(
                    &source.data.parallel,
                    &source_parameter,
                    source
                        .data
                        .frame_tangent
                        .as_ref()
                        .map(|tangent| (&tangent.x[..], &tangent.y[..])),
                    &source.data.tangent_distance,
                    &source.data.translation_x,
                    &source.data.translation_y,
                );
                let other_bounds = algebraic_chord_endpoint_bounds_refined(
                    if anchor_index == 0 {
                        self.data.source.end()
                    } else {
                        self.data.source.start()
                    },
                    refinement_steps,
                    policy,
                );
                let (start, end) = if anchor_index == 0 {
                    (source_bounds, other_bounds)
                } else {
                    (other_bounds, source_bounds)
                };
                let (Classification::Decided(start), Classification::Decided(end)) = (start, end)
                else {
                    continue;
                };
                let physical_x = RealInterval::from_axis(&end, Axis2::X)
                    .subtract(&RealInterval::from_axis(&start, Axis2::X));
                let physical_y = RealInterval::from_axis(&end, Axis2::Y)
                    .subtract(&RealInterval::from_axis(&start, Axis2::Y));
                let Some(physical_cross) = physical_x.multiply(&v_y).and_then(|first| {
                    physical_y
                        .multiply(&v_x)
                        .map(|second| first.subtract(&second))
                }) else {
                    continue;
                };
                physical_cross
            };
            let Some(u_speed) = direction_x
                .square()
                .and_then(|x| direction_y.square().map(|y| x.add(&y)))
                .and_then(|squared| squared.nonnegative_square_root(None))
            else {
                continue;
            };
            let Some(v_speed) = v_x
                .square()
                .and_then(|x| v_y.square().map(|y| x.add(&y)))
                .and_then(|squared| squared.nonnegative_square_root(None))
            else {
                continue;
            };
            let Some(direction_cross) = direction_x.multiply(&v_y).and_then(|first| {
                direction_y
                    .multiply(&v_x)
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            let Some(dot) = direction_x
                .multiply(&v_x)
                .and_then(|first| direction_y.multiply(&v_y).map(|second| first.add(&second)))
            else {
                continue;
            };
            let mut value = match (point_index, anchor_index) {
                (0, 1) => physical_cross.clone(),
                (1, 0) => RealInterval {
                    lower: -physical_cross.upper.clone(),
                    upper: -physical_cross.lower.clone(),
                },
                (0, 0) | (1, 1) => exact(&Real::zero()),
                _ => return Ok(None),
            };
            let displacement_numerator = match self.data.direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => dot.clone(),
                BezierAlgebraicChordUnitDisplacement2::Tangent => RealInterval {
                    lower: -direction_cross.upper.clone(),
                    upper: -direction_cross.lower.clone(),
                },
            };
            let Some(displacement) = scale(&displacement_numerator, &self.data.distance) else {
                continue;
            };
            let normal_delta = source.data.parallel.distance() - support.data.parallel.distance();
            let Some(normal) = scale(&v_speed, &normal_delta) else {
                continue;
            };
            value = value.add(&normal);
            let translation_x =
                &self.data.translation_x + &source.data.translation_x - &support.data.translation_x;
            let translation_y =
                &self.data.translation_y + &source.data.translation_y - &support.data.translation_y;
            let Some(translation) = v_x.multiply(&exact(&translation_y)).and_then(|first| {
                v_y.multiply(&exact(&translation_x))
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            value = value.add(&translation);
            let Some(scaled_value) = value.multiply(&u_speed) else {
                continue;
            };
            value = scaled_value.add(&displacement);
            terminal_refined |= refinement_steps == 512;
            if compare_reals(&value.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Some(RealSign::Positive));
            }
            if compare_reals(&value.upper, &Real::zero(), strict) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Some(RealSign::Negative));
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("analytic tangent side approximate-512 terminal=Zero");
            }
            return Ok(Some(RealSign::Zero));
        }
        Ok(None)
    }

    /// Signs this procedural point against an unrelated retained analytic
    /// tangent without materializing either displaced point.
    ///
    /// Let `L=(a,b,c)` be the oriented analytic line, `Q` this point's source
    /// endpoint, and `V` its unnormalized source-chord direction.  For either
    /// a tangent or left-normal displacement, substitution has the form
    ///
    /// `L(P) = I / W + distance * K / sqrt(V dot V)`.
    ///
    /// The projective denominator `W` and the speed root are strictly
    /// positive, so the requested side is exactly the sign of
    /// `I * sqrt(V dot V) + W * distance * K`.  Keeping that final root
    /// symbolic avoids joining the procedural point's normalized Cartesian
    /// extension to the analytic line's own speed extension.
    pub(super) fn recursive_general_oriented_side_to_analytic_tangent_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        start: &BezierAnalyticParallelPoint2,
        end: &BezierAnalyticParallelPoint2,
        parameter: &BezierRecursiveProjectiveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<crate::classify::LineSide>>> {
        if parameter.projective_scalar().is_none() {
            return Ok(Classification::Decided(None));
        }
        let tangent_displacement = &end.data.tangent_distance - &start.data.tangent_distance;
        let tangent_displacement_sign =
            match real_sign(&tangent_displacement, &CurveContext::STRICT) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "an analytic tangent support retained zero displacement".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
        let source_weight_sign = match start.data.parallel.source() {
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                RealSign::Positive
            }
            BezierParallelSource2::Rational(source) => {
                match parameter.polynomial_sign(
                    &source.homogeneous_power_basis()?.weight,
                    &policy.strict_counterpart(),
                )? {
                    Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                        sign
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "an analytic tangent support retained a zero source denominator".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        let orientation = Real::from(
            match product_sign(tangent_displacement_sign, source_weight_sign) {
                RealSign::Positive => 1_i8,
                RealSign::Negative => -1_i8,
                RealSign::Zero => {
                    unreachable!("both analytic tangent orientation factors are nonzero")
                }
            },
        );
        let source = start.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let translated_x = polynomial_add(
            source.x_numerator,
            &polynomial_scale(weight, &start.data.translation_x),
        );
        let translated_y = polynomial_add(
            source.y_numerator,
            &polynomial_scale(weight, &start.data.translation_y),
        );
        let (tangent_x, tangent_y) = start.frame_tangent_power_basis()?;
        let source_degree = [translated_x.len(), translated_y.len(), weight.len()]
            .into_iter()
            .max()
            .unwrap_or(1)
            .saturating_sub(1);
        let tangent_degree = tangent_x.len().max(tangent_y.len()).saturating_sub(1);
        let Some(line_degree) = source_degree.checked_add(tangent_degree) else {
            return Ok(Classification::Decided(None));
        };
        let Some(speed_degree) = tangent_degree.checked_mul(2) else {
            return Ok(Classification::Decided(None));
        };
        let line_a = polynomial_scale(&polynomial_multiply(tangent_y, weight), &Real::from(-1_i8));
        let line_b = polynomial_multiply(tangent_x, weight);
        let line_c = polynomial_subtract(
            &polynomial_multiply(tangent_y, &translated_x),
            &polynomial_multiply(tangent_x, &translated_y),
        );
        let analytic_speed_squared = polynomial_add(
            &polynomial_multiply(tangent_x, tangent_x),
            &polynomial_multiply(tangent_y, tangent_y),
        );
        let (Some(line_a), Some(line_b), Some(line_c), Some(weight), Some(analytic_speed_squared)) = (
            parameter.homogeneous_polynomial_value(&line_a, line_degree),
            parameter.homogeneous_polynomial_value(&line_b, line_degree),
            parameter.homogeneous_polynomial_value(&line_c, line_degree),
            parameter.homogeneous_polynomial_value(weight, source_degree),
            parameter.homogeneous_polynomial_value(&analytic_speed_squared, speed_degree),
        ) else {
            return Ok(Classification::Decided(None));
        };
        let Some((line_a, line_b, line_c, analytic_speed_coefficient)) = (|| {
            Some((
                line_a.scale(&orientation)?,
                line_b.scale(&orientation)?,
                line_c.scale(&orientation)?,
                weight
                    .scale(&(-start.data.parallel.distance().clone()))?
                    .scale(&orientation)?,
            ))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let direction_evidence = self.source_direction_endpoints(policy);
        let evidence = [
            self.source_endpoint(),
            direction_evidence[0],
            direction_evidence[1],
        ];
        let points = match recursive_projective_evidence_points(&evidence, policy)? {
            Classification::Decided(Some(points)) => points,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut normalized = Vec::with_capacity(points.len());
        for (point, evidence) in points.into_iter().zip(evidence) {
            let point = match recursive_projective_evidence_denominator_sign(evidence, policy)? {
                Classification::Decided(RealSign::Positive) => point,
                Classification::Decided(RealSign::Negative) => {
                    let negative = Real::from(-1_i8);
                    BezierRecursiveQuadraticProjectivePoint2 {
                        x: point.x.scale(&negative).ok_or_else(|| {
                            CurveError::Topology(
                                "a procedural tangent-side point exceeded its field budget".into(),
                            )
                        })?,
                        y: point.y.scale(&negative).ok_or_else(|| {
                            CurveError::Topology(
                                "a procedural tangent-side point exceeded its field budget".into(),
                            )
                        })?,
                        denominator: point.denominator.scale(&negative).ok_or_else(|| {
                            CurveError::Topology(
                                "a procedural tangent-side point exceeded its field budget".into(),
                            )
                        })?,
                    }
                }
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a procedural tangent-side point retained zero weight".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            normalized.push(point);
        }
        let [origin, direction_start, direction_end]: [BezierRecursiveQuadraticProjectivePoint2;
            3] = normalized
            .try_into()
            .expect("a procedural tangent-side frame retains three points");
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let (line_base, line_extensions) = line_a.field().base_and_extension_path();
            let (point_base, point_extensions) =
                origin.denominator.field().base_and_extension_path();
            eprintln!(
                "general analytic side fields line={}+{} point={}+{} shared-base={} line-lifts={} point-lifts={}",
                line_base.sources.len(),
                line_extensions.len(),
                point_base.sources.len(),
                point_extensions.len(),
                Arc::ptr_eq(&line_base, &point_base),
                origin.denominator.field().lift(&line_a).is_some(),
                origin.lifted_to(&line_a.field()).is_some(),
            );
        }
        let field = line_a.field();
        let (Some(origin), Some(direction_start), Some(direction_end)) = (
            origin.lifted_to(&field),
            direction_start.lifted_to(&field),
            direction_end.lifted_to(&field),
        ) else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("general analytic side decline=direction-field");
            }
            return Ok(Classification::Decided(None));
        };
        let Some((direction_x, direction_y, _)) =
            direction_end.difference_numerators(&direction_start)
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(speed_squared) = direction_x
            .square()
            .and_then(|x| direction_y.square().and_then(|y| x.add(&y)))
        else {
            return Ok(Classification::Decided(None));
        };
        let Some((retained, radical, constant)) = (|| {
            let translated_x = origin
                .x
                .add(&origin.denominator.scale(&self.data.translation_x)?)?;
            let translated_y = origin
                .y
                .add(&origin.denominator.scale(&self.data.translation_y)?)?;
            let retained = line_a
                .multiply(&translated_x)?
                .add(&line_b.multiply(&translated_y)?)?
                .add(&line_c.multiply(&origin.denominator)?)?;
            let radical = analytic_speed_coefficient.multiply(&origin.denominator)?;
            let constant = match self.data.direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => line_b
                    .multiply(&direction_x)?
                    .subtract(&line_a.multiply(&direction_y)?)?,
                BezierAlgebraicChordUnitDisplacement2::Tangent => line_a
                    .multiply(&direction_x)?
                    .add(&line_b.multiply(&direction_y)?)?,
            }
            .multiply(&origin.denominator)?
            .scale(&self.data.distance)?;
            Some((retained, radical, constant))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let retained_sign = retained.bounded_or_exact_real_witness_sign();
        let radical_sign = real_sign(start.data.parallel.distance(), &CurveContext::STRICT)
            .map(|sign| {
                product_sign(
                    product_sign(sign, tangent_displacement_sign),
                    RealSign::Negative,
                )
            })
            .or_else(|| radical.bounded_or_exact_real_witness_sign());
        let tangent_dot_sign = match chord
            .retained_support()
            .retained_rational_tangent_dot_sign_to(&self.data.source, policy)
        {
            Some(result) => match result? {
                Classification::Decided(sign) => Some(sign),
                Classification::Uncertain(_) => None,
            },
            None => None,
        };
        let constant_sign = tangent_dot_sign
            .zip(real_sign(&self.data.distance, &CurveContext::STRICT))
            .map(|(tangent, distance)| product_sign(tangent, distance))
            .or_else(|| constant.bounded_or_exact_real_witness_sign());
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!(
                "general analytic side terms retained={retained_sign:?}/zero={} radical={radical_sign:?}/zero={} constant={constant_sign:?}/zero={} normal={:?} displacement={:?}",
                retained.is_structurally_zero(),
                radical.is_structurally_zero(),
                constant.is_structurally_zero(),
                real_sign(start.data.parallel.distance(), &CurveContext::STRICT),
                real_sign(&self.data.distance, &CurveContext::STRICT),
            );
        }
        let sign = BezierRecursiveQuadraticValue2::nested_positive_root_affine_sign(
            &retained,
            &radical,
            &analytic_speed_squared,
            &constant,
            &speed_squared,
            retained_sign,
            radical_sign,
            constant_sign,
            policy,
        )?;
        let reversed = chord.retained_support_orientation_is_reversed();
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!("general analytic side result={sign:?} reversed={reversed}");
        }
        Ok(sign.map(|sign| {
            let sign = if reversed {
                product_sign(sign, RealSign::Negative)
            } else {
                sign
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "analytic-tangent-procedural-affine-root",
            );
            Some(crate::classify::LineSide::from_real_sign(sign))
        }))
    }

    /// Signs this procedural chord-normal endpoint against a retained
    /// analytic tangent support without constructing the support's second
    /// displaced endpoint.
    ///
    /// One analytic point supplies the line origin and owns the positive
    /// speed sheet. Its unnormalized tangent is evaluated in the same
    /// recursive parameter field. Joining that frame with this point's
    /// projective carrier leaves a single homogeneous cross-product scalar.
    pub(super) fn oriented_side_to_analytic_tangent_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<crate::classify::LineSide>>> {
        if !self.accepts_policy(policy) {
            return Ok(Classification::Decided(None));
        }
        let support = chord.retained_support();
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (support.start(), support.end())
        else {
            return Ok(Classification::Decided(None));
        };
        if start.data.parallel != end.data.parallel
            || start.data.parameter != end.data.parameter
            || start.data.frame_tangent != end.data.frame_tangent
            || start.data.translation_x != end.data.translation_x
            || start.data.translation_y != end.data.translation_y
            || !policy.accepts_retained_policy(start.data.policy)
            || !policy.accepts_retained_policy(end.data.policy)
        {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("mixed analytic tangent decline=frame");
            }
            return Ok(Classification::Decided(None));
        }
        let tangent_displacement = &end.data.tangent_distance - &start.data.tangent_distance;
        let tangent_displacement_sign =
            match real_sign(&tangent_displacement, &CurveContext::STRICT) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "an analytic tangent support retained zero displacement".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
        let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &start.data.parameter
        else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("mixed analytic tangent decline=parameter");
            }
            return Ok(Classification::Decided(None));
        };
        let physical_endpoints = [self.data.source.start(), self.data.source.end()];
        let related_source = physical_endpoints
            .iter()
            .enumerate()
            .find_map(|(index, source)| {
                let CurvePoint2(CurvePointData2::AnalyticParallel(source)) = source else {
                    return None;
                };
                (source.data.parallel.source() == start.data.parallel.source()
                    && source.data.frame_tangent == start.data.frame_tangent
                    && policy.accepts_retained_policy(source.data.policy))
                .then_some((index, source))
            });
        let Some((anchor_index, source)) = related_source else {
            match self.recursive_general_oriented_side_to_analytic_tangent_chord(
                chord, start, end, parameter, policy,
            )? {
                Classification::Decided(Some(side)) => {
                    return Ok(Classification::Decided(Some(side)));
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                let relations = physical_endpoints.map(|source| {
                    let CurvePoint2(CurvePointData2::AnalyticParallel(source)) = source else {
                        return None;
                    };
                    Some((
                        source.data.parallel.source() == start.data.parallel.source(),
                        source.data.parallel == start.data.parallel,
                        source.data.parameter == start.data.parameter,
                        source.data.frame_tangent == start.data.frame_tangent,
                        policy.accepts_retained_policy(source.data.policy),
                    ))
                });
                let parameter_relation = physical_endpoints.iter().find_map(|source| {
                    let CurvePoint2(CurvePointData2::AnalyticParallel(source)) = source else {
                        return None;
                    };
                    let (
                        BezierAnalyticParallelPointParameter2::RecursiveProjective(first),
                        BezierAnalyticParallelPointParameter2::RecursiveProjective(second),
                    ) = (&source.data.parameter, &start.data.parameter)
                    else {
                        return Some(("non-recursive", false, false, false, false, false));
                    };
                    let authority_kind =
                        |parameter: &BezierRecursiveProjectiveParameter2| match &parameter
                            .data
                            .authority
                        {
                            BezierRecursiveProjectiveParameterAuthority2::Projective(_) => {
                                "projective"
                            }
                            BezierRecursiveProjectiveParameterAuthority2::Monotone(_) => "monotone",
                            BezierRecursiveProjectiveParameterAuthority2::Polynomial { .. } => {
                                "polynomial"
                            }
                        };
                    let identities = match (
                        first.data.identity.as_deref(),
                        second.data.identity.as_deref(),
                    ) {
                        (
                            Some(
                                BezierRecursiveProjectiveParameterIdentity2::ChordRationalTangent(
                                    first,
                                ),
                            ),
                            Some(
                                BezierRecursiveProjectiveParameterIdentity2::ChordRationalTangent(
                                    second,
                                ),
                            ),
                        ) => (
                            first.source == second.source,
                            first.chord.shares_retained_support(&second.chord),
                            first
                                .chord
                                .shared_tangent_orientation(&second.chord)
                                .is_some(),
                            first.chord_location == second.chord_location,
                            first.tangent_cross_sign == second.tangent_cross_sign,
                        ),
                        _ => (false, false, false, false, false),
                    };
                    Some((
                        authority_kind(first),
                        first.shares_polynomial_root(second),
                        identities.0,
                        identities.1,
                        identities.2,
                        identities.3 && identities.4,
                    ))
                });
                eprintln!(
                    "mixed analytic tangent decline=related-source relations={relations:?} parameter={parameter_relation:?}"
                );
            }
            return Ok(Classification::Decided(None));
        };
        let reverse = (tangent_displacement_sign == RealSign::Negative)
            ^ chord.retained_support_orientation_is_reversed();
        if let Some(sign) = self.analytic_tangent_side_interval_sign(
            source,
            start,
            parameter,
            anchor_index,
            policy,
        )? {
            let side = match (sign, reverse) {
                (RealSign::Positive, false) | (RealSign::Negative, true) => {
                    crate::classify::LineSide::Left
                }
                (RealSign::Negative, false) | (RealSign::Positive, true) => {
                    crate::classify::LineSide::Right
                }
                (RealSign::Zero, _) => crate::classify::LineSide::On,
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "analytic-tangent-correlated-interval",
            );
            return Ok(Classification::Decided(Some(side)));
        }
        let (tangent_x_coefficients, tangent_y_coefficients) = start.frame_tangent_power_basis()?;
        let tangent_degree = tangent_x_coefficients
            .len()
            .max(tangent_y_coefficients.len())
            .saturating_sub(1);
        let Some(speed_degree) = tangent_degree.checked_mul(2) else {
            return Ok(Classification::Decided(None));
        };
        let speed_squared_coefficients = polynomial_add(
            &polynomial_multiply(tangent_x_coefficients, tangent_x_coefficients),
            &polynomial_multiply(tangent_y_coefficients, tangent_y_coefficients),
        );
        let (Some(tangent_x), Some(tangent_y), Some(speed_squared)) = (
            parameter.homogeneous_polynomial_value(tangent_x_coefficients, tangent_degree),
            parameter.homogeneous_polynomial_value(tangent_y_coefficients, tangent_degree),
            parameter.homogeneous_polynomial_value(&speed_squared_coefficients, speed_degree),
        ) else {
            return Ok(Classification::Decided(None));
        };
        // The physical tangent-line origin is the correlated analytic
        // contact, not the authored endpoint substituted only to normalize
        // the finite chord direction. Put that contact first so its retained
        // parameter tower is the authority into which the chord points are
        // embedded. This avoids joining two divergent descendants after the
        // chord-speed radical has already been adjoined.
        let [direction_start_evidence, direction_end_evidence] =
            self.source_direction_endpoints(policy);
        let points = match recursive_projective_evidence_points(
            &[
                physical_endpoints[anchor_index],
                self.source_endpoint(),
                direction_start_evidence,
                direction_end_evidence,
            ],
            policy,
        )? {
            Classification::Decided(Some(points)) => points,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [source_origin, query_origin, direction_start, direction_end]: [
            BezierRecursiveQuadraticProjectivePoint2;
            4
        ] = points
            .try_into()
            .expect("a correlated tangent side frame retains four projective points");
        let normalize = |point, evidence: &CurvePoint2| {
            if matches!(
                evidence,
                CurvePoint2(CurvePointData2::Exact(_))
                    | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            ) {
                // Exact points have unit denominator. Analytic recursive
                // points normalize from their common source-weight sign, and
                // derived radial points preserve the positive denominator of
                // their already-normalized contact frame. Field embedding
                // changes none of those signs.
                Ok(Classification::Decided(point))
            } else {
                positive_recursive_projective_point(point)
            }
        };
        let source_origin = match normalize(source_origin, physical_endpoints[anchor_index])? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let query_origin = match normalize(query_origin, self.source_endpoint())? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let direction_start = match normalize(direction_start, direction_start_evidence)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let direction_end = match normalize(direction_end, direction_end_evidence)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let field = query_origin.denominator.field();
        let (Some(tangent_x), Some(tangent_y), Some(tangent_speed_squared)) = (
            field.lift(&tangent_x),
            field.lift(&tangent_y),
            field.lift(&speed_squared),
        ) else {
            return Ok(Classification::Decided(None));
        };
        let Some((direction_x, direction_y, _)) =
            direction_end.difference_numerators(&direction_start)
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(direction_speed_squared) = direction_x
            .square()
            .and_then(|x| x.add(&direction_y.square()?))
        else {
            return Ok(Classification::Decided(None));
        };
        // Keeping both positive norms symbolic collapses the determinant to
        //
        //   sqrt(|W|^2) * (A + B sqrt(|H|^2)) + C,
        //
        // where W is the canonical chord direction and H is the analytic
        // hodograph. All A/B/C coefficients stay in the already-shared
        // parameter field. Two exact squared-magnitude comparisons therefore
        // replace two new quadratic extensions and their exponentially larger
        // recursive norm.
        let normal_delta = source.data.parallel.distance() - start.data.parallel.distance();
        let translation_x = &source.data.translation_x - &start.data.translation_x;
        let translation_y = &source.data.translation_y - &start.data.translation_y;
        let Some((retained, radical, constant)) = (|| {
            let denominator = query_origin
                .denominator
                .multiply(&source_origin.denominator)?;
            let delta_x = query_origin
                .x
                .multiply(&source_origin.denominator)?
                .subtract(&source_origin.x.multiply(&query_origin.denominator)?)?
                .add(&denominator.scale(&self.data.translation_x)?)?;
            let delta_y = query_origin
                .y
                .multiply(&source_origin.denominator)?
                .subtract(&source_origin.y.multiply(&query_origin.denominator)?)?
                .add(&denominator.scale(&self.data.translation_y)?)?;
            let point_cross = tangent_x
                .multiply(&delta_y)?
                .subtract(&tangent_y.multiply(&delta_x)?)?;
            let support_translation = tangent_x
                .scale(&translation_y)?
                .subtract(&tangent_y.scale(&translation_x)?)?;
            let retained = point_cross.add(&denominator.multiply(&support_translation)?)?;
            let radical = denominator.scale(&normal_delta)?;
            let displacement_cross = match self.data.direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => tangent_x
                    .multiply(&direction_x)?
                    .add(&tangent_y.multiply(&direction_y)?)?,
                BezierAlgebraicChordUnitDisplacement2::Tangent => tangent_x
                    .multiply(&direction_y)?
                    .subtract(&tangent_y.multiply(&direction_x)?)?,
            };
            let constant = denominator
                .multiply(&displacement_cross)?
                .scale(&self.data.distance)?;
            Some((retained, radical, constant))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let radical_sign = real_sign(&normal_delta, &CurveContext::STRICT);
        let retained_sign = if self.data.source_point.is_none()
            && source.data.parallel.distance().zero_status() == ZeroKnowledge::Zero
            && source.data.tangent_distance.zero_status() == ZeroKnowledge::Zero
            && source.data.translation_x.zero_status() == ZeroKnowledge::Zero
            && source.data.translation_y.zero_status() == ZeroKnowledge::Zero
            && self.data.translation_x.zero_status() == ZeroKnowledge::Zero
            && self.data.translation_y.zero_status() == ZeroKnowledge::Zero
            && start.data.translation_x.zero_status() == ZeroKnowledge::Zero
            && start.data.translation_y.zero_status() == ZeroKnowledge::Zero
            && source.data.frame_tangent.is_none()
        {
            let point_index = usize::from(self.at_end);
            if point_index == anchor_index {
                Some(RealSign::Zero)
            } else if let BezierParallelSource2::Rational(rational) = start.data.parallel.source() {
                match parameter.chord_rational_tangent_cross_sign(
                    &self.data.source,
                    rational,
                    RealSign::Positive,
                    policy,
                ) {
                    Some(result) => match result? {
                        Classification::Decided(sign) => {
                            Some(if (point_index, anchor_index) == (1, 0) {
                                product_sign(sign, RealSign::Negative)
                            } else {
                                sign
                            })
                        }
                        Classification::Uncertain(_) => None,
                    },
                    None => None,
                }
            } else {
                None
            }
        } else {
            None
        };
        let sign = match BezierRecursiveQuadraticValue2::nested_positive_root_affine_sign(
            &retained,
            &radical,
            &tangent_speed_squared,
            &constant,
            &direction_speed_squared,
            retained_sign,
            radical_sign,
            None,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let side = match (sign, reverse) {
            (RealSign::Positive, false) | (RealSign::Negative, true) => {
                crate::classify::LineSide::Left
            }
            (RealSign::Negative, false) | (RealSign::Positive, true) => {
                crate::classify::LineSide::Right
            }
            (RealSign::Zero, _) => crate::classify::LineSide::On,
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-side-kernel",
            "analytic-tangent-recursive-scalar",
        );
        Ok(Classification::Decided(Some(side)))
    }

    pub(super) fn translated(
        &self,
        delta_x: &Real,
        delta_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "algebraic chord parallel point was translated under a different predicate policy"
                    .into(),
            ));
        }
        Ok(Self {
            data: Arc::new(BezierAlgebraicChordParallelData2 {
                source: self.data.source.clone(),
                source_point: self.data.source_point.clone(),
                distance: self.data.distance.clone(),
                translation_x: &self.data.translation_x + delta_x,
                translation_y: &self.data.translation_y + delta_y,
                direction: self.data.direction,
                policy: policy.retained_object_policy(),
                recursive_points: OnceLock::new(),
            }),
            at_end: self.at_end,
        })
    }

    pub(super) fn transform_similarity_cached(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
        cache: &mut BezierAlgebraicCuspSemicircleSimilarityCache2,
    ) -> CurveResult<Classification<Self>> {
        if !self.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "algebraic chord parallel point was transformed under a different predicate policy"
                    .into(),
            ));
        }
        let source = match cache.chord(&self.data.source, transform, policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut distance = &self.data.distance * transform.scale();
        if self.data.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal
            && transform.reverses_orientation()
        {
            distance = -distance;
        }
        let (translation_x, translation_y) = transform
            .transform_vector_coordinates(&self.data.translation_x, &self.data.translation_y);
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicChordParallelData2 {
                source,
                source_point: self.data.source_point.as_ref().map(|point| {
                    Arc::new(CurvePoint2::from(BezierSimilarityPoint2::new(
                        point.as_ref().clone(),
                        transform.clone(),
                        policy,
                    )))
                }),
                distance,
                translation_x,
                translation_y,
                direction: self.data.direction,
                policy: policy.retained_object_policy(),
                recursive_points: OnceLock::new(),
            }),
            at_end: self.at_end,
        }))
    }

    pub(crate) fn conservative_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, false)
    }

    pub(super) fn conservative_local_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, true)
    }

    pub(super) fn conservative_bounds_refined_impl(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
        local_only: bool,
    ) -> Classification<Aabb2> {
        if !self.accepts_policy(policy) || self.data.source.validate_policy(policy).is_err() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        let [source_start, source_end] = self.source_direction_endpoints(policy);
        let endpoint_bounds = |point| {
            if local_only {
                algebraic_chord_endpoint_local_bounds_refined(point, refinement_steps, policy)
            } else {
                algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
            }
        };
        let (start, end) = match (endpoint_bounds(source_start), endpoint_bounds(source_end)) {
            (Classification::Decided(start), Classification::Decided(end)) => (start, end),
            _ => {
                if local_only {
                    return Classification::Uncertain(UncertaintyReason::Ordering);
                }
                let points =
                    match recursive_projective_evidence_points(&[source_start, source_end], policy)
                    {
                        Ok(Classification::Decided(Some(points))) => points,
                        Ok(Classification::Decided(None) | Classification::Uncertain(_))
                        | Err(_) => {
                            return Classification::Uncertain(UncertaintyReason::Ordering);
                        }
                    };
                let [start, end]: [BezierRecursiveQuadraticProjectivePoint2; 2] = points
                    .try_into()
                    .expect("a chord direction retains two projective endpoints");
                let (Classification::Decided(start), Classification::Decided(end)) = (
                    start.bounds_refined(refinement_steps),
                    end.bounds_refined(refinement_steps),
                ) else {
                    return Classification::Uncertain(UncertaintyReason::Ordering);
                };
                (start, end)
            }
        };
        let delta_x = RealInterval::from_axis(&end, Axis2::X)
            .subtract(&RealInterval::from_axis(&start, Axis2::X));
        let delta_y = RealInterval::from_axis(&end, Axis2::Y)
            .subtract(&RealInterval::from_axis(&start, Axis2::Y));
        let Some(speed) = delta_x
            .square()
            .and_then(|x| delta_y.square().map(|y| x.add(&y)))
            .and_then(|speed_squared| speed_squared.nonnegative_square_root(None))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let direction_x = match self.data.direction {
            BezierAlgebraicChordUnitDisplacement2::LeftNormal => RealInterval {
                lower: -delta_y.upper.clone(),
                upper: -delta_y.lower.clone(),
            },
            BezierAlgebraicChordUnitDisplacement2::Tangent => delta_x.clone(),
        };
        let direction_y = match self.data.direction {
            BezierAlgebraicChordUnitDisplacement2::LeftNormal => delta_x,
            BezierAlgebraicChordUnitDisplacement2::Tangent => delta_y,
        };
        let Some(unit_x) = direction_x.divide(&speed) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(unit_y) = direction_y.divide(&speed) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let distance = RealInterval {
            lower: self.data.distance.clone(),
            upper: self.data.distance.clone(),
        };
        let (Some(offset_x), Some(offset_y)) =
            (unit_x.multiply(&distance), unit_y.multiply(&distance))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let origin = if let Some(point) = self.data.source_point.as_deref() {
            match endpoint_bounds(point) {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => {
                    return Classification::Uncertain(reason);
                }
            }
        } else if self.at_end {
            if std::ptr::eq(source_end, self.data.source.end()) {
                end
            } else {
                match endpoint_bounds(self.data.source.end()) {
                    Classification::Decided(bounds) => bounds,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
        } else if std::ptr::eq(source_start, self.data.source.start()) {
            start
        } else {
            match endpoint_bounds(self.data.source.start()) {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => {
                    return Classification::Uncertain(reason);
                }
            }
        };
        let x = RealInterval::from_axis(&origin, Axis2::X)
            .add(&offset_x)
            .add(&RealInterval {
                lower: self.data.translation_x.clone(),
                upper: self.data.translation_x.clone(),
            });
        let y = RealInterval::from_axis(&origin, Axis2::Y)
            .add(&offset_y)
            .add(&RealInterval {
                lower: self.data.translation_y.clone(),
                upper: self.data.translation_y.clone(),
            });
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }

    /// Signs this parallel endpoint against a concentric circle when its
    /// source endpoint already retains exact circle incidence.
    ///
    /// With `P-C = R`, `|R| = r`, unit left normal `N`, and parallel distance
    /// `d`, the endpoint is `P' = P + dN` and
    /// `|P'-C|^2-r_t^2 = r^2+d^2-r_t^2+2d R·N`. Cauchy bounds the final
    /// term by `2|d|r`. At a zero bound, equality would require `R` parallel
    /// to `N`, equivalently `R·D = 0` for the source chord direction `D`.
    /// Strict refinement rules that equality out without flattening fields.
    pub(super) fn concentric_circle_incidence_sign(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RealSign>>> {
        // A point authored directly in a chord's orthonormal frame around the
        // selected circle center has squared radius `distance^2`, independent
        // of the chord's selected endpoint fields.  Repeated collinear
        // translations are folded into `distance` by `translated`, so this
        // certificate survives subsequent exact offsets without projection.
        if self.accepts_policy(policy)
            && self.data.translation_x.zero_status() == ZeroKnowledge::Zero
            && self.data.translation_y.zero_status() == ZeroKnowledge::Zero
            && let Some(source_point) = self.data.source_point.as_deref()
        {
            let center = match semicircle.center_point_evidence(policy)? {
                Classification::Decided(center) => center,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            if source_point.shares_storage(&center)
                || source_point.same_point(&center, policy) == Classification::Decided(true)
            {
                let residual = &self.data.distance * &self.data.distance
                    - semicircle.radial_distance() * semicircle.radial_distance();
                return Ok(Some(match real_sign(&residual, policy) {
                    Some(sign) => Classification::Decided(sign),
                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                }));
            }
        }
        if !self.accepts_policy(policy)
            || self.data.source_point.is_some()
            || self.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return Ok(None);
        }
        let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(source)) =
            self.source_endpoint()
        else {
            return Ok(None);
        };
        source.data.source.validate_policy(policy)?;
        if source.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || source.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return Ok(None);
        }
        let source_semicircle = source.data.source.semicircle();
        if source_semicircle.data.frame != semicircle.data.frame {
            return Ok(None);
        }

        let source_scale = (&source.data.radial_scale * &source.data.radial_scale
            + &source.data.perpendicular_scale * &source.data.perpendicular_scale)
            .sqrt()?;
        let source_radius = (source_scale * source_semicircle.radial_distance()).abs();
        let target_radius = semicircle.radial_distance().abs();
        let distance = self.data.distance.abs();
        let constant = &source_radius * &source_radius + &distance * &distance
            - &target_radius * &target_radius;
        let reach = Real::from(2_i8) * &distance * &source_radius;
        let lower = &constant - &reach;
        let upper = &constant + &reach;
        let strict = &CurveContext::STRICT;
        let lower_sign = real_sign(&lower, strict);
        let upper_sign = real_sign(&upper, strict);
        if upper_sign == Some(RealSign::Negative) {
            return Ok(Some(Classification::Decided(RealSign::Negative)));
        }
        if lower_sign == Some(RealSign::Positive) {
            return Ok(Some(Classification::Decided(RealSign::Positive)));
        }
        if lower_sign == Some(RealSign::Zero) && upper_sign == Some(RealSign::Zero) {
            return Ok(Some(Classification::Decided(RealSign::Zero)));
        }
        let boundary_sign = match (lower_sign, upper_sign) {
            (Some(RealSign::Negative), Some(RealSign::Zero)) => RealSign::Negative,
            (Some(RealSign::Zero), Some(RealSign::Positive)) => RealSign::Positive,
            _ => return Ok(None),
        };
        if self
            .data
            .source
            .certified_circle_transverse_endpoint(self.at_end)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-parallel-circle-incidence",
                "certified-transverse-cauchy-boundary",
            );
            return Ok(Some(Classification::Decided(boundary_sign)));
        }

        let point = CurvePoint2::from(source.clone());
        let center = CurvePoint2::from(source_semicircle.center_point_image(policy)?);
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (
                Classification::Decided(point),
                Classification::Decided(center),
                Classification::Decided(chord_start),
                Classification::Decided(chord_end),
            ) = (
                algebraic_chord_endpoint_bounds_refined(&point, refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(&center, refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(
                    self.data.source.start(),
                    refinement_steps,
                    policy,
                ),
                algebraic_chord_endpoint_bounds_refined(
                    self.data.source.end(),
                    refinement_steps,
                    policy,
                ),
            )
            else {
                continue;
            };
            let radial_x = RealInterval::from_axis(&point, Axis2::X)
                .subtract(&RealInterval::from_axis(&center, Axis2::X));
            let radial_y = RealInterval::from_axis(&point, Axis2::Y)
                .subtract(&RealInterval::from_axis(&center, Axis2::Y));
            let chord_x = RealInterval::from_axis(&chord_end, Axis2::X)
                .subtract(&RealInterval::from_axis(&chord_start, Axis2::X));
            let chord_y = RealInterval::from_axis(&chord_end, Axis2::Y)
                .subtract(&RealInterval::from_axis(&chord_start, Axis2::Y));
            let Some(projection) = radial_x
                .multiply(&chord_x)
                .and_then(|x| radial_y.multiply(&chord_y).map(|y| x.add(&y)))
            else {
                continue;
            };
            if real_sign(&projection.lower, strict) == Some(RealSign::Positive)
                || real_sign(&projection.upper, strict) == Some(RealSign::Negative)
            {
                return Ok(Some(Classification::Decided(boundary_sign)));
            }
        }
        Ok(None)
    }

    pub(super) fn strict_cardinal_axis_shift(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Option<Real> {
        // A cardinal source makes each normalized tangent/normal component
        // exactly -1, 0, or 1. Determine that component under STRICT so an
        // APPROXIMATE_512 equality never becomes a reusable construction
        // fact for a later coordinate sign.
        if !policy.accepts_retained_policy(self.data.policy) {
            return None;
        }
        let direction = self.data.source.certified_axis_direction()?;
        let (tangent_x, tangent_y) = direction.cardinal_components();
        let component = match (self.data.direction, axis) {
            (BezierAlgebraicChordUnitDisplacement2::Tangent, Axis2::X) => tangent_x,
            (BezierAlgebraicChordUnitDisplacement2::Tangent, Axis2::Y) => tangent_y,
            (BezierAlgebraicChordUnitDisplacement2::LeftNormal, Axis2::X) => -tangent_y,
            (BezierAlgebraicChordUnitDisplacement2::LeftNormal, Axis2::Y) => tangent_x,
        };
        let mut shift = match axis {
            Axis2::X => self.data.translation_x.clone(),
            Axis2::Y => self.data.translation_y.clone(),
        };
        match component {
            1 => shift += &self.data.distance,
            -1 => shift -= &self.data.distance,
            _ => {}
        }
        Some(shift)
    }

    pub(super) fn strict_cardinal_shifts(&self, policy: &CurveContext) -> Option<[Real; 2]> {
        Some([
            self.strict_cardinal_axis_shift(Axis2::X, policy)?,
            self.strict_cardinal_axis_shift(Axis2::Y, policy)?,
        ])
    }

    /// Materializes a cardinal unit displacement in the source endpoint's
    /// existing exact field.
    ///
    /// This is intentionally a predicate-boundary adapter rather than the
    /// stored offset representation. General offsets remain one-word
    /// procedural points; circle/line incidence can nevertheless remove a
    /// known `sqrt(1)` sheet before constructing its polynomial system.
    pub(super) fn strict_cardinal_point_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<CurvePoint2>> {
        let Some([shift_x, shift_y]) = self.strict_cardinal_shifts(policy) else {
            return Ok(None);
        };
        Ok(
            match BezierAlgebraicChord2::translated_endpoint(
                self.source_endpoint(),
                &shift_x,
                &shift_y,
                policy,
            )? {
                Classification::Decided(point) => Some(point),
                Classification::Uncertain(_) => None,
            },
        )
    }

    pub(super) fn strict_preserved_axis_source<'a>(
        &'a self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Option<&'a CurvePoint2> {
        let shift = self.strict_cardinal_axis_shift(axis, policy)?;
        (real_sign(&shift, &CurveContext::STRICT) == Some(RealSign::Zero))
            .then(|| self.source_endpoint())
    }

    /// Compares two exact cardinal-frame displacements after cancelling the
    /// first frame shift from both coordinates.
    ///
    /// Normal offsets of independently replayed boundary fragments commonly
    /// name the same affine support without sharing allocation identity. The
    /// coordinate identity is `S1+a ? S2+b`, equivalently
    /// `S1 ? S2+(b-a)`. Descending to the retained source endpoints preserves
    /// their contact correlation and removes one normalization layer instead
    /// of asking two independently expanded `Real` coordinates to prove the
    /// same equality.
    pub(super) fn cardinal_axis_order_to_parallel(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<std::cmp::Ordering>>> {
        if !self.accepts_policy(policy) || !other.accepts_policy(policy) {
            return Some(Err(CurveError::Topology(
                "cardinal chord displacements crossed retained predicate policies".into(),
            )));
        }
        let first_shift = self.strict_cardinal_axis_shift(axis, policy)?;
        let second_shift = other.strict_cardinal_axis_shift(axis, policy)?;
        let delta = second_shift - first_shift;
        if real_sign(&delta, &CurveContext::STRICT) == Some(RealSign::Zero) {
            return Some(BezierAlgebraicChord2::point_axis_order(
                self.source_endpoint(),
                other.source_endpoint(),
                axis,
                policy,
            ));
        }
        let (delta_x, delta_y) = match axis {
            Axis2::X => (delta, Real::zero()),
            Axis2::Y => (Real::zero(), delta),
        };
        Some((|| {
            let translated = match BezierAlgebraicChord2::translated_endpoint(
                other.source_endpoint(),
                &delta_x,
                &delta_y,
                policy,
            )? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            BezierAlgebraicChord2::point_axis_order(
                self.source_endpoint(),
                &translated,
                axis,
                policy,
            )
        })())
    }

    pub(super) fn exact_axis_coordinate(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let Some(shift) = self.strict_cardinal_axis_shift(axis, policy) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        };
        let source = match self.source_endpoint() {
            CurvePoint2(CurvePointData2::Exact(point)) => Some(match axis {
                Axis2::X => point.x().clone(),
                Axis2::Y => point.y().clone(),
            }),
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                point.exact_coordinate(axis == Axis2::X, policy)
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
                match point.exact_axis_coordinate(axis, policy)? {
                    Classification::Decided(coordinate) => coordinate,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => {
                match point.exact_axis_coordinate(axis, policy)? {
                    Classification::Decided(coordinate) => coordinate,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        Ok(Classification::Decided(
            source.map(|coordinate| coordinate + shift),
        ))
    }

    pub(super) fn axis_coordinate_order_to_real(
        &self,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> Classification<std::cmp::Ordering> {
        // Project cardinal displacement back to the source endpoint before
        // interval refinement. Besides avoiding a radical, this proves
        // identities such as "a normal displacement of a vertical chord
        // preserves y", which finite-width boxes cannot establish in STRICT.
        if let Some(shift) = self.strict_cardinal_axis_shift(axis, policy) {
            let source_target = value - shift;
            if let Ok(Classification::Decided(order)) =
                BezierAlgebraicChord2::point_axis_order_to_real(
                    self.source_endpoint(),
                    axis,
                    &source_target,
                    &policy.strict_counterpart(),
                )
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-axis-order",
                    "cardinal-source-projection",
                );
                return Classification::Decided(order);
            }
        }
        // Preserve the inexpensive native separation path before constructing
        // a shared field. Suppress the approximate terminal here so unresolved
        // comparisons still reach the complete retained predicate below.
        let bounded = policy.strict_predicate_pass(|| {
            retained_bounds_axis_order_to_real(
                |steps| self.conservative_bounds_refined(steps, policy),
                axis,
                value,
                policy,
            )
        });
        if bounded.is_decided() {
            return bounded;
        }
        // Oblique displacements also retain their exact source and positive
        // normal sheet. Independent bounds cannot prove a shared endpoint;
        // reuse the common coordinate predicate and its projective replay.
        let target = match axis {
            Axis2::X => Point2::new(value.clone(), Real::zero()),
            Axis2::Y => Point2::new(Real::zero(), value.clone()),
        };
        algebraic_chord_point_coordinate_order_fallback(
            &CurvePoint2::from(self.clone()),
            &CurvePoint2::from(target),
            axis,
            policy,
        )
    }

    pub(crate) fn same_point_evidence(
        &self,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(other)) = other
            && self.shares_carrier(other)
        {
            return Classification::Decided(self.at_end == other.at_end);
        }
        if self.data.distance.zero_status() == ZeroKnowledge::Zero
            && self.data.translation_x.zero_status() == ZeroKnowledge::Zero
            && self.data.translation_y.zero_status() == ZeroKnowledge::Zero
        {
            let same = self.source_endpoint().same_point(other, policy);
            if matches!(same, Classification::Decided(_)) {
                return same;
            }
        }
        retained_point_evidence_equality_by_refinement(
            &CurvePoint2::from(self.clone()),
            other,
            policy,
        )
    }
}

impl BezierAlgebraicCuspChordDerivedPoint2 {
    pub(super) fn from_mapped_source(
        parameter: Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
        point: Option<CurvePoint2>,
        radial_scale: Real,
    ) -> Self {
        Self::from_mapped_source_with_rotation(parameter, point, radial_scale, Real::zero())
    }

    pub(super) fn from_mapped_source_with_rotation(
        parameter: Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
        point: Option<CurvePoint2>,
        radial_scale: Real,
        perpendicular_scale: Real,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicCuspChordDerivedPointData2 {
                source: BezierAlgebraicCuspDerivedPointSource2::Mapped { parameter, point },
                radial_scale,
                perpendicular_scale,
                translation_x: Real::zero(),
                translation_y: Real::zero(),
            }),
        }
    }

    pub(super) fn rotated_from_mapped_source(
        parameter: Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
        point: CurvePoint2,
        radial_scale: Real,
        perpendicular_scale: Real,
    ) -> Self {
        Self::from_mapped_source_with_rotation(
            parameter,
            Some(point),
            radial_scale,
            perpendicular_scale,
        )
    }

    pub(super) fn translated(&self, translation_x: &Real, translation_y: &Real) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicCuspChordDerivedPointData2 {
                source: self.data.source.clone(),
                radial_scale: self.data.radial_scale.clone(),
                perpendicular_scale: self.data.perpendicular_scale.clone(),
                translation_x: &self.data.translation_x + translation_x,
                translation_y: &self.data.translation_y + translation_y,
            }),
        }
    }

    /// Returns the retained source point when this affine derivation is the
    /// identity. Keeping that provenance visible lets coordinate predicates
    /// reuse the source carrier's exact constant-coordinate and shared-field
    /// proofs instead of comparing two reconstructed interval boxes. Selected
    /// fibers reuse their source evaluation without promoting the parameter
    /// or reconstructing either Cartesian coordinate.
    pub(crate) fn identity_source_point(&self, policy: &CurveContext) -> Option<CurvePoint2> {
        self.data.source.validate_policy(policy).ok()?;
        if self.data.radial_scale != Real::one()
            || self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return None;
        }
        match &self.data.source {
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                point: Some(point), ..
            } => Some(point.clone()),
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter,
                point: None,
            } => parameter
                .selected_fiber_analytic_point(policy)
                .ok()
                .flatten()
                .map(CurvePoint2::from),
            BezierAlgebraicCuspDerivedPointSource2::Chord(_) => None,
        }
    }

    /// Orders equal concentric radial images through their retained source
    /// points. For `Q=C+a(P-C)+T`, two images with the same `C`, `a`, and `T`
    /// satisfy `Q2-Q1=a(P2-P1)`, so the selected center field cancels before
    /// any coordinate representation is constructed.
    pub(super) fn common_radial_source_axis_order(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<std::cmp::Ordering>> {
        self.data.source.validate_policy(policy)?;
        other.data.source.validate_policy(policy)?;
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || other.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || self.data.radial_scale != other.data.radial_scale
            || self.data.translation_x != other.data.translation_x
            || self.data.translation_y != other.data.translation_y
            || self.data.source.semicircle() != other.data.source.semicircle()
        {
            return Ok(None);
        }
        let scale_sign = match real_sign(&self.data.radial_scale, &CurveContext::STRICT) {
            Some(sign) => sign,
            None => return Ok(None),
        };
        if scale_sign == RealSign::Zero {
            return Ok(Some(std::cmp::Ordering::Equal));
        }
        let order = if let Some(order) =
            self.data
                .source
                .common_rational_constant_axis(&other.data.source, axis, policy)
        {
            order
        } else {
            let (
                BezierAlgebraicCuspDerivedPointSource2::Mapped {
                    point: Some(first), ..
                },
                BezierAlgebraicCuspDerivedPointSource2::Mapped {
                    point: Some(second),
                    ..
                },
            ) = (&self.data.source, &other.data.source)
            else {
                return Ok(None);
            };
            let source_order = algebraic_chord_point_coordinate_order(
                first,
                second,
                axis,
                &policy.strict_counterpart(),
            );
            let Ok(Classification::Decided(order)) = source_order else {
                return Ok(None);
            };
            order
        };
        Ok(Some(if scale_sign == RealSign::Negative {
            order.reverse()
        } else {
            order
        }))
    }

    /// Returns the exact direction scale between two untranslated radial
    /// images of the same retained contact. For
    /// `Q(a) = C + a(P-C)`, `Q(a2)-Q(a1) = (a2-a1)(P-C)`.
    pub(super) fn common_untranslated_radial_difference_sign(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || other.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
            || other.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || other.data.translation_y.zero_status() != ZeroKnowledge::Zero
            || !self.data.source.shares_exact_evidence(&other.data.source)
            || self.data.source.semicircle() != other.data.source.semicircle()
        {
            return Ok(None);
        }
        self.data.source.validate_policy(policy)?;
        other.data.source.validate_policy(policy)?;
        Ok(real_sign(
            &(&other.data.radial_scale - &self.data.radial_scale),
            &CurveContext::STRICT,
        ))
    }

    /// Classifies a point against the line through two unrotated radial
    /// images of the same retained contact without constructing either image.
    ///
    /// For `Q(a) = C + a(P-C)`, the oriented area is
    ///
    /// `cross(Q(a2)-Q(a1), E-Q(a1)) = (a2-a1) cross(P-C, E-C)`.
    ///
    /// The selected source and support-center fields therefore enter one
    /// smaller recursive predicate exactly once. Rotated or translated
    /// images deliberately decline this identity and retain the complete
    /// represented fallback.
    pub(super) fn common_untranslated_radial_line_oriented_side(
        &self,
        other: &Self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        let radial_sign = match self.common_untranslated_radial_difference_sign(other, policy)? {
            Some(sign) => sign,
            None => return Ok(None),
        };
        if radial_sign == RealSign::Zero {
            return Ok(Some(crate::classify::LineSide::On));
        }
        let orient_to_scale = |side| {
            if radial_sign == RealSign::Negative {
                match side {
                    crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                    crate::classify::LineSide::On => crate::classify::LineSide::On,
                    crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                }
            } else {
                side
            }
        };
        if let CurvePoint2(CurvePointData2::Exact(point)) = point
            && let Some((map, contact)) = self.data.source.chord_map_contact()
            && let Some(system) = map.recursive_quadratic_line_system()
        {
            let native = system.radial_oriented_side_to_exact_point(contact, point, policy)?;
            if let Classification::Decided(side) = native {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-tangent-relation",
                    "shared-radial-recursive-line",
                );
                return Ok(Some(orient_to_scale(side)));
            }
        }

        let synthesized_source = match &self.data.source {
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter,
                point: None,
            } => parameter
                .selected_fiber_analytic_point(policy)?
                .map(CurvePoint2::from),
            BezierAlgebraicCuspDerivedPointSource2::Chord(_)
            | BezierAlgebraicCuspDerivedPointSource2::Mapped { point: Some(_), .. } => None,
        };
        let source = match &self.data.source {
            BezierAlgebraicCuspDerivedPointSource2::Chord(source) => {
                CurvePoint2::from(source.clone())
            }
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                point: Some(source),
                ..
            } => source.clone(),
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter,
                point: None,
            } => match parameter
                .retained_point_evidence()
                .cloned()
                .or(synthesized_source)
            {
                Some(source) => source,
                None => return Ok(None),
            },
        };
        let center = match self
            .data
            .source
            .semicircle()
            .center_point_evidence(policy)?
        {
            Classification::Decided(center) => center,
            Classification::Uncertain(_) => return Ok(None),
        };
        let side = match recursive_projective_point_evidence_oriented_side(
            &center, &source, point, false, policy,
        )? {
            Classification::Decided(Some(side)) => side,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-tangent-relation",
            "shared-radial-source",
        );
        Ok(Some(orient_to_scale(side)))
    }

    pub(super) fn recursive_projective_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        self.data.source.validate_policy(policy)?;
        if let Some((map, contact)) = self.data.source.chord_map_contact() {
            return map.recursive_derived_projective_point(
                contact,
                &self.data.radial_scale,
                &self.data.perpendicular_scale,
                &self.data.translation_x,
                &self.data.translation_y,
                policy,
            );
        }
        let synthesized_point = match &self.data.source {
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter,
                point: None,
            } => parameter
                .selected_fiber_analytic_point(policy)?
                .map(CurvePoint2::from),
            BezierAlgebraicCuspDerivedPointSource2::Mapped { point: Some(_), .. }
            | BezierAlgebraicCuspDerivedPointSource2::Chord(_) => None,
        };
        let explicit_point = match &self.data.source {
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                point: Some(point), ..
            } => Some(point),
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter,
                point: None,
            } => parameter
                .retained_point_evidence()
                .or(synthesized_point.as_ref()),
            BezierAlgebraicCuspDerivedPointSource2::Chord(_) => None,
        };
        if let Some(source) = explicit_point
            && !matches!(
                source,
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point))
                    if Arc::ptr_eq(&point.data, &self.data)
            )
        {
            // Q = C + a(P-C) + b J(P-C) + T has no dependence on C
            // when a=1 and b=0. Import only the points that survive that
            // cancellation, preserving P's field and positive denominator.
            let center = if self.data.radial_scale == Real::one()
                && self.data.perpendicular_scale.zero_status() == ZeroKnowledge::Zero
            {
                None
            } else {
                match self
                    .data
                    .source
                    .semicircle()
                    .center_point_evidence(policy)?
                {
                    Classification::Decided(center) => Some(center),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let pair;
            let inputs = match &center {
                Some(center) => {
                    pair = [source, center];
                    pair.as_slice()
                }
                None => std::slice::from_ref(&source),
            };
            let points = match recursive_projective_evidence_points(inputs, policy)? {
                Classification::Decided(Some(points)) => points,
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut points = points.into_iter();
            let source = points.next().expect("a derived point retains its source");
            let source = match positive_recursive_projective_point(source)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let transformed = if let Some(center) = points.next() {
                let center = match positive_recursive_projective_point(center)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                source.rotated_radial_image(
                    &center,
                    &self.data.radial_scale,
                    &self.data.perpendicular_scale,
                    &self.data.translation_x,
                    &self.data.translation_y,
                )
            } else {
                source.transformed_affine(
                    &Real::one(),
                    &Real::zero(),
                    &Real::zero(),
                    &Real::one(),
                    &self.data.translation_x,
                    &self.data.translation_y,
                )
            };
            return Ok(match transformed {
                Some(point) => Classification::Decided(Some(point)),
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            });
        }
        let Some((map, contact, first)) = self.data.source.coincident_pair_map_contact() else {
            return Ok(Classification::Decided(None));
        };
        let support = if first {
            &map.data.first_semicircle
        } else {
            &map.data.second_semicircle
        };
        let parameter = Arc::new(BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
            map: map.clone(),
            contact: contact.clone(),
            first,
        });
        // Reuse the selected-radial frame constructor to expose the pair
        // contact and its support center in the pair map's canonical compact
        // field. The unit-radius carrier is only a frame view; no curve or
        // coordinate is materialized from it.
        let frame_view = match BezierAlgebraicCuspSemicircle2::from_selected_circle_radial(
            support,
            BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter),
            support.radial_distance().clone(),
            Real::one(),
            false,
            policy,
        )? {
            Classification::Decided(Some(frame)) => frame,
            Classification::Decided(None) => {
                return Err(CurveError::Topology(
                    "a direct pair contact produced a zero-radius recursive frame".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (source, center) = match frame_view.recursive_selected_pair_frame_authority(policy)? {
            Classification::Decided(Some(authority)) => {
                (authority.center, authority.support_center)
            }
            Classification::Decided(None) => {
                match map.recursive_represented_contact_frame(contact, first, policy)? {
                    Classification::Decided(Some(frame)) => (frame.point, frame.center),
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(
            match source.rotated_radial_image(
                &center,
                &self.data.radial_scale,
                &self.data.perpendicular_scale,
                &self.data.translation_x,
                &self.data.translation_y,
            ) {
                Some(point) => Classification::Decided(Some(point)),
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
        )
    }

    /// Compares affine images whose selected contacts live in one recursive
    /// quadratic tower. A later line/circle contact extends the earlier
    /// point's field by one positive discriminant root, so lifting the older
    /// projective point with zero radical coefficients preserves their exact
    /// correlation and avoids interval equality at every nesting depth.
    pub(super) fn recursive_projective_axis_order(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<std::cmp::Ordering>>> {
        let first = match self.recursive_projective_point(policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let second = match other.recursive_projective_point(policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        first.axis_order(&second, axis, policy)
    }

    pub(super) fn recursive_projective_axis_order_to_chord(
        &self,
        other: &BezierAlgebraicCuspChordPoint2,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<std::cmp::Ordering>>> {
        let first = match self.recursive_projective_point(policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let second = match other.recursive_projective_point(policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let order = first.axis_order(&second, axis, policy)?;
        Ok(order)
    }

    /// Recognizes the covariant image of the same retained radial derivation.
    ///
    /// Similarity-transformed selected-radial circles keep their local radial
    /// coefficient, negate only the perpendicular coefficient under
    /// reflection, and linearly transform any extra displacement.  Matching
    /// that compact provenance proves endpoint equality without refining the
    /// original independent circle-pair fields to 512 bits.
    pub(super) fn is_similarity_image_of(
        &self,
        source: &Self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> bool {
        let (
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter: transformed_parameter,
                point: Some(transformed_source_point),
            },
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter: source_parameter,
                point: source_point,
            },
        ) = (&self.data.source, &source.data.source)
        else {
            return false;
        };
        let BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
            source: transported_parameter,
            point: transported_point,
            policy: transport_policy,
            ..
        } = transformed_parameter.as_ref()
        else {
            return false;
        };
        let CurvePoint2(CurvePointData2::Similarity(transported_source_point)) = transported_point
        else {
            return false;
        };
        let transported_base_matches = source_point
            .as_ref()
            .is_some_and(|source_point| transported_source_point.data.source == *source_point)
            || matches!(
                &transported_source_point.data.source,
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point))
                    if point.data.radial_scale == Real::one()
                        && point.data.perpendicular_scale.zero_status() == ZeroKnowledge::Zero
                        && point.data.translation_x.zero_status() == ZeroKnowledge::Zero
                        && point.data.translation_y.zero_status() == ZeroKnowledge::Zero
                        && matches!(
                            &point.data.source,
                            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                                parameter,
                                ..
                            } if BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter.clone())
                                .shares_exact_evidence(
                                    &BezierAlgebraicCuspSemicircleParameter2::Mapped(
                                        source_parameter.clone(),
                                    ),
                                )
                        )
            );
        if !policy.accepts_retained_policy(*transport_policy)
            || !policy.accepts_retained_policy(transported_source_point.data.policy)
            || transported_source_point.data.transform != *transform
            || !transported_base_matches
            || transformed_source_point != transported_point
            || !transported_parameter.shares_exact_evidence(
                &BezierAlgebraicCuspSemicircleParameter2::Mapped(source_parameter.clone()),
            )
            || self.data.radial_scale != source.data.radial_scale
        {
            return false;
        }
        let expected_perpendicular = if transform.reverses_orientation() {
            -source.data.perpendicular_scale.clone()
        } else {
            source.data.perpendicular_scale.clone()
        };
        if self.data.perpendicular_scale != expected_perpendicular {
            return false;
        }
        let (translation_x, translation_y) = transform
            .transform_vector_coordinates(&source.data.translation_x, &source.data.translation_y);
        self.data.translation_x == translation_x && self.data.translation_y == translation_y
    }

    /// Returns an exact represented circle known to contain this affine image.
    ///
    /// If `P` lies on the source circle centered at `C`, every retained image
    /// has the form `C + a(P-C) + b J(P-C) + T`.  It therefore lies on the
    /// circle centered at `C+T` with squared radius `(a^2+b^2)r^2`.  This
    /// structural invariant can reject equality with another retained point
    /// using one circle predicate, without refining two unrelated coordinate
    /// fields into overlapping boxes.
    pub(super) fn exact_supporting_circle(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<(Point2, Real)>> {
        self.data.source.validate_policy(policy)?;
        let center = match self
            .data
            .source
            .semicircle()
            .center_point_evidence(policy)?
        {
            Classification::Decided(CurvePoint2(CurvePointData2::Exact(center))) => Some(center),
            Classification::Decided(CurvePoint2(CurvePointData2::Algebraic(center))) => {
                center.exact_point(&CurveContext::STRICT)
            }
            Classification::Decided(CurvePoint2(CurvePointData2::AnalyticParallel(center))) => {
                match center.represented_point(policy)? {
                    Classification::Decided(center) => center,
                    Classification::Uncertain(_) => None,
                }
            }
            Classification::Decided(
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)),
            )
            | Classification::Uncertain(_) => None,
        };
        let Some(center) = center else {
            return Ok(None);
        };
        let scale_squared = &self.data.radial_scale * &self.data.radial_scale
            + &self.data.perpendicular_scale * &self.data.perpendicular_scale;
        let radius_squared = scale_squared
            * self.data.source.semicircle().radial_distance()
            * self.data.source.semicircle().radial_distance();
        Ok(Some((
            center.translated(
                self.data.translation_x.clone(),
                self.data.translation_y.clone(),
            ),
            radius_squared,
        )))
    }

    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }

    /// Returns exact standalone coordinate representations for a derived
    /// point backed by a represented chord or selected-radial pair map. The
    /// source map owns correlation and STRICT sheet selection; callers receive
    /// ordinary algebraic numbers only after that proof is complete.
    pub(super) fn represented_coordinates(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<[AlgebraicRootRepresentation; 2]>>> {
        self.data.source.validate_policy(policy)?;
        if let Some((map, contact)) = self.data.source.chord_map_contact()
            && map.represented_oblique_system().is_some()
        {
            return Ok(map
                .represented_oblique_derived_coordinates(
                    contact,
                    &self.data.radial_scale,
                    &self.data.perpendicular_scale,
                    &self.data.translation_x,
                    &self.data.translation_y,
                )?
                .map(Some));
        }
        if let Some((map, contact)) = self.data.source.chord_map_contact()
            && map.oblique_system().is_some()
        {
            return Ok(map
                .oblique_represented_derived_coordinates(
                    contact,
                    &self.data.radial_scale,
                    &self.data.perpendicular_scale,
                    &self.data.translation_x,
                    &self.data.translation_y,
                )?
                .map(Some));
        }
        if let Some((map, contact, first)) = self.data.source.coincident_pair_map_contact() {
            return Ok(map
                .represented_selected_radial_derived_point(
                    contact,
                    first,
                    &self.data.radial_scale,
                    &self.data.perpendicular_scale,
                    &self.data.translation_x,
                    &self.data.translation_y,
                    policy,
                )?
                .map(Some));
        }
        let source = match &self.data.source {
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                point: Some(source),
                ..
            } => represented_point_evidence_coordinates(source, policy)?,
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter,
                point: None,
            } => {
                if let Some(source) = parameter.selected_fiber_analytic_point(policy)? {
                    source.represented_coordinates(policy)?
                } else {
                    let Some(source) = parameter.mapped_point_source(policy)? else {
                        return Ok(Classification::Decided(None));
                    };
                    source.represented_coordinates(policy)?
                }
            }
            BezierAlgebraicCuspDerivedPointSource2::Chord(_) => {
                return Ok(Classification::Decided(None));
            }
        };
        let source = match source {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match self
            .data
            .source
            .semicircle()
            .center_point_evidence(policy)?
        {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match represented_point_evidence_coordinates(&center, policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radial = self.data.radial_scale.clone();
        let perpendicular = self.data.perpendicular_scale.clone();
        let negative_perpendicular = -perpendicular.clone();
        let one_minus_radial = Real::one() - &radial;
        let x = represented_affine_coordinate(
            &[
                (&source[0], &radial),
                (&source[1], &negative_perpendicular),
                (&center[0], &one_minus_radial),
                (&center[1], &perpendicular),
            ],
            &self.data.translation_x,
        );
        let y = represented_affine_coordinate(
            &[
                (&source[0], &perpendicular),
                (&source[1], &radial),
                (&center[0], &negative_perpendicular),
                (&center[1], &one_minus_radial),
            ],
            &self.data.translation_y,
        );
        Ok(match (x, y) {
            (Classification::Decided(x), Classification::Decided(y)) => {
                Classification::Decided(Some([x, y].map(|coordinate| {
                    hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                        .unwrap_or(coordinate)
                })))
            }
            (Classification::Uncertain(UncertaintyReason::Unsupported), _)
            | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            _ => Classification::Uncertain(UncertaintyReason::Predicate),
        })
    }

    pub(super) fn affine_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.affine_bounds_refined_impl(refinement_steps, policy, false)
    }

    pub(super) fn affine_bounds_refined_impl(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
        local_only: bool,
    ) -> Classification<Aabb2> {
        if self.data.source.validate_policy(policy).is_err() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        let point = match self.data.source.conservative_bounds_refined_impl(
            refinement_steps,
            policy,
            local_only,
        ) {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Classification::Uncertain(reason);
            }
        };
        let center = match self.data.source.semicircle().center_point_evidence(policy) {
            Ok(Classification::Decided(center)) => {
                let bounds = if local_only {
                    algebraic_chord_endpoint_local_bounds_refined(&center, refinement_steps, policy)
                } else {
                    algebraic_chord_endpoint_bounds_refined(&center, refinement_steps, policy)
                };
                match bounds {
                    Classification::Decided(center) => center,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let point_x = RealInterval::from_axis(&point, Axis2::X);
        let point_y = RealInterval::from_axis(&point, Axis2::Y);
        let center_x = RealInterval::from_axis(&center, Axis2::X);
        let center_y = RealInterval::from_axis(&center, Axis2::Y);
        let multiply = |value: &RealInterval, coefficient: Real| {
            value.multiply(&RealInterval {
                lower: coefficient.clone(),
                upper: coefficient,
            })
        };
        let a = self.data.radial_scale.clone();
        let b = self.data.perpendicular_scale.clone();
        let one_minus_a = Real::one() - &a;
        let Some(x) = multiply(&point_x, a.clone())
            .and_then(|value| multiply(&point_y, -b.clone()).map(|term| value.add(&term)))
            .and_then(|value| multiply(&center_x, one_minus_a.clone()).map(|term| value.add(&term)))
            .and_then(|value| multiply(&center_y, b.clone()).map(|term| value.add(&term)))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(y) = multiply(&point_x, b.clone())
            .and_then(|value| multiply(&point_y, a).map(|term| value.add(&term)))
            .and_then(|value| multiply(&center_x, -b).map(|term| value.add(&term)))
            .and_then(|value| multiply(&center_y, one_minus_a).map(|term| value.add(&term)))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let x = x.add(&RealInterval {
            lower: self.data.translation_x.clone(),
            upper: self.data.translation_x.clone(),
        });
        let y = y.add(&RealInterval {
            lower: self.data.translation_y.clone(),
            upper: self.data.translation_y.clone(),
        });
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }

    pub(super) fn refined_linear_order_to_real(
        &self,
        x_factor: &Real,
        y_factor: &Real,
        value: &Real,
        policy: &CurveContext,
    ) -> Classification<std::cmp::Ordering> {
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
                break;
            }
            let Classification::Decided(bounds) =
                self.affine_bounds_refined(refinement_steps, policy)
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let x = RealInterval::from_axis(&bounds, Axis2::X);
            let y = RealInterval::from_axis(&bounds, Axis2::Y);
            let x_factor = RealInterval {
                lower: x_factor.clone(),
                upper: x_factor.clone(),
            };
            let y_factor = RealInterval {
                lower: y_factor.clone(),
                upper: y_factor.clone(),
            };
            let Some(linear) = x
                .multiply(&x_factor)
                .and_then(|x| y.multiply(&y_factor).map(|y| x.add(&y)))
            else {
                continue;
            };
            if compare_reals(&linear.upper, value, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Classification::Decided(std::cmp::Ordering::Less);
            }
            if compare_reals(&linear.lower, value, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Greater)
            {
                return Classification::Decided(std::cmp::Ordering::Greater);
            }
            if compare_reals(&linear.lower, value, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(&linear.upper, value, &CurveContext::STRICT)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Classification::Decided(std::cmp::Ordering::Equal);
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Classification::Decided(std::cmp::Ordering::Equal)
        } else {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
    }

    pub(crate) fn conservative_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, false)
    }

    pub(super) fn conservative_local_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, true)
    }

    pub(super) fn conservative_bounds_refined_impl(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
        local_only: bool,
    ) -> Classification<Aabb2> {
        if self.data.radial_scale == Real::one()
            && self.data.perpendicular_scale.zero_status() == ZeroKnowledge::Zero
            && self.data.translation_x.zero_status() == ZeroKnowledge::Zero
            && self.data.translation_y.zero_status() == ZeroKnowledge::Zero
        {
            return self.data.source.conservative_bounds_refined_impl(
                refinement_steps,
                policy,
                local_only,
            );
        }
        if self.data.perpendicular_scale.zero_status() == ZeroKnowledge::Zero
            && let Some((map, contact)) = self.data.source.chord_map_contact()
        {
            if map.validate_policy(policy).is_err() {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
            return map.derived_contact_bounds_refined(
                contact,
                &self.data.radial_scale,
                &self.data.translation_x,
                &self.data.translation_y,
                refinement_steps,
            );
        }
        self.affine_bounds_refined_impl(refinement_steps, policy, local_only)
    }

    pub(super) fn axis_coordinate_order_to_real(
        &self,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let Some((map, contact, first)) = self.data.source.direct_pair_map_contact()
            && map.represented_contact_data(contact).is_some()
        {
            if !policy.accepts_retained_policy(map.data.policy) {
                return Err(CurveError::Topology(
                    "a represented pair point crossed predicate policies".into(),
                ));
            }
            let coordinates = match map.represented_selected_radial_derived_point(
                contact,
                first,
                &self.data.radial_scale,
                &self.data.perpendicular_scale,
                &self.data.translation_x,
                &self.data.translation_y,
                policy,
            )? {
                Classification::Decided(coordinates) => coordinates,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let coordinate = match axis {
                Axis2::X => &coordinates[0],
                Axis2::Y => &coordinates[1],
            };
            return Ok(represented_order_to_real(coordinate, value, policy));
        }
        let (x_factor, y_factor) = match axis {
            Axis2::X => (Real::one(), Real::zero()),
            Axis2::Y => (Real::zero(), Real::one()),
        };
        if self.data.perpendicular_scale.zero_status() == ZeroKnowledge::Zero
            && self.data.source.chord_map_contact().is_some()
        {
            return self.linear_order_to_real(&x_factor, &y_factor, value, policy);
        }
        let order = policy.strict_predicate_pass(|| {
            self.linear_order_to_real(&x_factor, &y_factor, value, policy)
        })?;
        if matches!(order, Classification::Decided(_)) {
            return Ok(order);
        }
        if !policy.has_bounded_exact_predicate_budget()
            && let Classification::Decided(Some(coordinates)) =
                self.represented_coordinates(policy)?
        {
            let coordinate = match axis {
                Axis2::X => &coordinates[0],
                Axis2::Y => &coordinates[1],
            };
            let represented = represented_order_to_real(coordinate, value, policy);
            if matches!(represented, Classification::Decided(_)) {
                return Ok(represented);
            }
        }
        if policy.permits_approximate_512() {
            self.linear_order_to_real(&x_factor, &y_factor, value, policy)
        } else {
            Ok(order)
        }
    }

    /// Recognizes the equal tangent-frame coordinate of complementary points
    /// on one cardinally framed selected semicircle. This is structural
    /// equality, not the APPROXIMATE_512 interval terminal.
    pub(super) fn complementary_mapped_axis_order(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<std::cmp::Ordering>> {
        self.complementary_mapped_axis_order_with_authority(other, axis, false, policy)
    }

    /// Recognizes the same equality only when two analytic-parallel maps
    /// supply the STRICT complement proof. This result may be retained as a
    /// construction fact rather than used only for the current query.
    pub(super) fn strict_parallel_complementary_mapped_axis_order(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<std::cmp::Ordering>> {
        self.complementary_mapped_axis_order_with_authority(other, axis, true, policy)
    }

    /// Derives the exact bevel direction between equal radial images of one
    /// retained circle-circle contact on its two original carriers.
    ///
    /// For `Q_i=C_i+s(P-C_i)+T`, equal `s` and `T` give
    /// `Q_2-Q_1=(1-s)(C_2-C_1)`: the multi-field contact `P` cancels. When
    /// both selected centers materialize as exact rational points, normalize
    /// that remaining vector once and retain it on the chord. All signs and
    /// the square root are exact under STRICT; no approximate terminal may
    /// create a construction direction.
    pub(super) fn strict_pair_contact_join_unit_tangent(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<(Real, Real)>> {
        self.data.source.validate_policy(policy)?;
        other.data.source.validate_policy(policy)?;
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || other.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || self.data.radial_scale != other.data.radial_scale
            || self.data.perpendicular_scale != other.data.perpendicular_scale
            || self.data.translation_x != other.data.translation_x
            || self.data.translation_y != other.data.translation_y
        {
            return Ok(None);
        }
        let (
            Some((first_map, first_contact, first_side)),
            Some((second_map, second_contact, second_side)),
        ) = (
            self.data.source.coincident_pair_map_contact(),
            other.data.source.coincident_pair_map_contact(),
        )
        else {
            return Ok(None);
        };
        if !Arc::ptr_eq(&first_map.data, &second_map.data)
            || first_contact != second_contact
            || first_side == second_side
        {
            return Ok(None);
        }
        let center = |first: bool| -> CurveResult<Option<Point2>> {
            let semicircle = if first {
                &first_map.data.first_semicircle
            } else {
                &first_map.data.second_semicircle
            };
            Ok(semicircle
                .center_point_image(policy)?
                .exact_point(&CurveContext::STRICT))
        };
        let (Some(first_center), Some(second_center)) = (center(first_side)?, center(second_side)?)
        else {
            return Ok(None);
        };
        let factor = Real::one() - &self.data.radial_scale;
        let delta_x = factor.clone() * (second_center.x() - first_center.x());
        let delta_y = factor * (second_center.y() - first_center.y());
        let length_squared = &delta_x * &delta_x + &delta_y * &delta_y;
        match real_sign(&length_squared, &CurveContext::STRICT) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => return Ok(None),
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "pair-contact bevel direction had negative squared length".into(),
                ));
            }
            None => return Ok(None),
        }
        let length = length_squared.sqrt()?;
        Ok(Some(((delta_x / &length)?, (delta_y / length)?)))
    }

    pub(super) fn complementary_mapped_axis_order_with_authority(
        &self,
        other: &Self,
        axis: Axis2,
        strict_parallel_only: bool,
        policy: &CurveContext,
    ) -> CurveResult<Option<std::cmp::Ordering>> {
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || other.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || self.data.radial_scale != other.data.radial_scale
            || self.data.perpendicular_scale != other.data.perpendicular_scale
            || self.data.translation_x != other.data.translation_x
            || self.data.translation_y != other.data.translation_y
            || self.data.source.semicircle() != other.data.source.semicircle()
        {
            return Ok(None);
        }
        if self.data.radial_scale.zero_status() == ZeroKnowledge::Zero && !strict_parallel_only {
            return Ok(Some(std::cmp::Ordering::Equal));
        }
        let (
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter: first, ..
            },
            BezierAlgebraicCuspDerivedPointSource2::Mapped {
                parameter: second, ..
            },
        ) = (&self.data.source, &other.data.source)
        else {
            return Ok(None);
        };
        let first = first.coincident_base_data();
        let second = second.coincident_base_data();
        if first.semicircle_carrier() != second.semicircle_carrier() {
            return Ok(None);
        }
        let tangent_axis = match first
            .semicircle_carrier()
            .data
            .frame
            .certified_cardinal_normal()?
        {
            Some((1 | -1, 0)) => Axis2::Y,
            Some((0, 1 | -1)) => Axis2::X,
            None => return Ok(None),
            Some(_) => unreachable!("a retained cardinal normal is validated at construction"),
        };
        if axis != tangent_axis {
            return Ok(None);
        }
        let complementary = if strict_parallel_only {
            first.parallel_complementary_to(second, policy)?
        } else {
            Some(first.is_complementary_to(second, policy)?)
        };
        Ok(match complementary {
            Some(Classification::Decided(true)) => Some(std::cmp::Ordering::Equal),
            None | Some(Classification::Decided(false) | Classification::Uncertain(_)) => None,
        })
    }

    pub(super) fn linear_order_to_real(
        &self,
        x_factor: &Real,
        y_factor: &Real,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let Some((map, contact)) = self.data.source.chord_map_contact() {
            let px = &self.data.radial_scale * x_factor + &self.data.perpendicular_scale * y_factor;
            let py = &self.data.radial_scale * y_factor - &self.data.perpendicular_scale * x_factor;
            let cx = x_factor - &px;
            let cy = y_factor - &py;
            let offset =
                &self.data.translation_x * x_factor + &self.data.translation_y * y_factor - value;
            return map.affine_order(contact, [&px, &py], [&cx, &cy], &offset, policy);
        }
        Ok(self.refined_linear_order_to_real(x_factor, y_factor, value, policy))
    }

    pub(super) fn oriented_side_to_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        let evidence = CurvePoint2::from(self.clone());
        if [chord.start(), chord.end()]
            .into_iter()
            .any(|endpoint| evidence.shares_storage(endpoint))
        {
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        let Some(line) = chord.exact_line() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let delta_x = line.end().x() - line.start().x();
        let delta_y = line.end().y() - line.start().y();
        let x_factor = -delta_y;
        let y_factor = delta_x;
        let value = &x_factor * line.start().x() + &y_factor * line.start().y();
        Ok(self
            .linear_order_to_real(&x_factor, &y_factor, &value, policy)?
            .map(|order| {
                crate::classify::LineSide::from_real_sign(match order {
                    std::cmp::Ordering::Less => RealSign::Negative,
                    std::cmp::Ordering::Equal => RealSign::Zero,
                    std::cmp::Ordering::Greater => RealSign::Positive,
                })
            }))
    }

    pub(super) fn same_source_transform(
        &self,
        other_radial_scale: &Real,
        other_perpendicular_scale: &Real,
        other_translation_x: &Real,
        other_translation_y: &Real,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if self.data.source.validate_policy(policy).is_err() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        if self.data.radial_scale == *other_radial_scale
            && self.data.perpendicular_scale == *other_perpendicular_scale
            && self.data.translation_x == *other_translation_x
            && self.data.translation_y == *other_translation_y
        {
            return Classification::Decided(true);
        }
        let Some((map, contact)) = self.data.source.chord_map_contact() else {
            return Classification::Uncertain(UncertaintyReason::Predicate);
        };
        let radial = &self.data.radial_scale - other_radial_scale;
        let perpendicular = &self.data.perpendicular_scale - other_perpendicular_scale;
        let negative_radial = -&radial;
        let negative_perpendicular = -&perpendicular;
        let tx = &self.data.translation_x - other_translation_x;
        let ty = &self.data.translation_y - other_translation_y;
        let mut uncertainty = None;
        for (point, center, offset) in [
            (
                [&radial, &negative_perpendicular],
                [&negative_radial, &perpendicular],
                &tx,
            ),
            (
                [&perpendicular, &radial],
                [&negative_perpendicular, &negative_radial],
                &ty,
            ),
        ] {
            match map.affine_order(contact, point, center, offset, policy) {
                Ok(Classification::Decided(std::cmp::Ordering::Equal)) => {}
                Ok(Classification::Decided(
                    std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                )) => {
                    return Classification::Decided(false);
                }
                Ok(Classification::Uncertain(reason)) => {
                    uncertainty.get_or_insert(reason);
                }
                Err(_) => {
                    uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                }
            }
        }
        uncertainty
            .map(Classification::Uncertain)
            .unwrap_or(Classification::Decided(true))
    }

    pub(super) fn center_relative_coordinate_expression(
        &self,
        axis: Axis2,
    ) -> Option<BezierAlgebraicCuspTwoTermExpression2> {
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero {
            return None;
        }
        let (map, _) = self.data.source.chord_map_contact()?;
        let system = map.axis_system()?;
        let mut coordinate = map.derived_coordinate_expression(
            axis,
            &self.data.radial_scale,
            match axis {
                Axis2::X => &self.data.translation_x,
                Axis2::Y => &self.data.translation_y,
            },
        );
        coordinate.rational = bivariate_subtract(
            &coordinate.rational,
            match axis {
                Axis2::X => &system.center_x,
                Axis2::Y => &system.center_y,
            },
        );
        Some(coordinate)
    }

    pub(super) fn unshifted_concentric_circle_incidence_residual(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
    ) -> Option<Real> {
        let source_semicircle = self.data.source.semicircle();
        if source_semicircle.data.frame != semicircle.data.frame
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return None;
        }
        let source_radius_squared =
            source_semicircle.radial_distance() * source_semicircle.radial_distance();
        let target_radius_squared = semicircle.radial_distance() * semicircle.radial_distance();
        let scale_squared = &self.data.radial_scale * &self.data.radial_scale
            + &self.data.perpendicular_scale * &self.data.perpendicular_scale;
        Some(scale_squared * source_radius_squared - target_radius_squared)
    }

    /// Signs an equal-radius pair-contact image against the opposite circle
    /// after both circles receive the same radial scale.
    ///
    /// With `P` on both source circles and
    /// `Q_j=C_j+s(P-C_j)`, the residual against the scaled opposite circle is
    /// `(1-s)[s(r_i^2-r_j^2)+|C_j-C_i|^2]`. Equal source radii cancel the
    /// first term. A retained transverse/tangent pair map already proved the
    /// center distance strictly positive, so only the exact sign of `1-s`
    /// remains. This is the common bevel-adjacency case and requires no new
    /// bivariate storage.
    pub(super) fn equal_radius_pair_other_circle_incidence_sign(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
    ) -> CurveResult<Option<RealSign>> {
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x != Real::zero()
            || self.data.translation_y != Real::zero()
        {
            return Ok(None);
        }
        let Some((map, _, point_first)) = self.data.source.direct_pair_map_contact() else {
            return Ok(None);
        };
        let point_source = if point_first {
            &map.data.first_semicircle
        } else {
            &map.data.second_semicircle
        };
        let target_source = if point_first {
            &map.data.second_semicircle
        } else {
            &map.data.first_semicircle
        };
        if target_source.data.frame != semicircle.data.frame
            || point_source.radial_distance() * point_source.radial_distance()
                != target_source.radial_distance() * target_source.radial_distance()
            || semicircle.radial_distance() * semicircle.radial_distance()
                != &self.data.radial_scale
                    * &self.data.radial_scale
                    * target_source.radial_distance()
                    * target_source.radial_distance()
        {
            return Ok(None);
        }
        Ok(real_sign(
            &(Real::one() - &self.data.radial_scale),
            &CurveContext::STRICT,
        ))
    }

    /// Signs this derived point's squared-distance residual against a circle
    /// with the same retained center frame.
    pub(super) fn concentric_circle_incidence_sign(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        self.data.source.validate_policy(policy)?;
        if let Some(sign) = self.equal_radius_pair_other_circle_incidence_sign(semicircle)? {
            return Ok(Classification::Decided(Some(sign)));
        }
        let source_semicircle = self.data.source.semicircle();
        if source_semicircle.data.frame != semicircle.data.frame {
            return Ok(Classification::Decided(None));
        }
        if let Some(residual) = self.unshifted_concentric_circle_incidence_residual(semicircle) {
            return Ok(real_sign(&residual, policy).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                |sign| Classification::Decided(Some(sign)),
            ));
        }
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero {
            return Ok(Classification::Decided(None));
        }
        let Some((map, contact)) = self.data.source.chord_map_contact() else {
            return Ok(Classification::Decided(None));
        };
        if let Some(system) = map.selected_radial_system() {
            // Reuse |P-C|^2=r0^2 exactly, as in the trivariate carrier below,
            // but retain both authored radicals:
            //   D^2(s^2 r0^2+|T|^2-r^2)+2sD T.(P-C).
            let Some(center_x) = BezierSelectedRadialCircleChordNestedExpression2::from_retained(
                system.center_x.clone(),
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(center_y) = BezierSelectedRadialCircleChordNestedExpression2::from_retained(
                system.center_y.clone(),
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let (Some(delta_x), Some(delta_y)) = (
                system.point_x.subtract(&center_x),
                system.point_y.subtract(&center_y),
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(dot) =
                BezierSelectedRadialCircleChordNestedExpression2::linear_combination(&[
                    (&delta_x, &self.data.translation_x),
                    (&delta_y, &self.data.translation_y),
                ])
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let source_radius_squared =
                map.data.semicircle.radial_distance() * map.data.semicircle.radial_distance();
            let target_radius_squared = semicircle.radial_distance() * semicircle.radial_distance();
            let scale_squared = &self.data.radial_scale * &self.data.radial_scale;
            let translation_squared = &self.data.translation_x * &self.data.translation_x
                + &self.data.translation_y * &self.data.translation_y;
            let constant =
                scale_squared * source_radius_squared + translation_squared - target_radius_squared;
            let Some(denominator_squared) = system
                .common_denominator
                .multiply(&system.common_denominator)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(constant) = BezierSelectedRadialCircleChordNestedExpression2::from_rational(
                denominator_squared.scale(&constant).ok_or_else(|| {
                    CurveError::Topology(
                        "a pair-radial incidence exceeded its tensor budget".into(),
                    )
                })?,
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let twice_scale = Real::from(2_i8) * &self.data.radial_scale;
            let Some(cross) = dot
                .multiply_rational(&system.common_denominator)
                .and_then(|cross| cross.scale(&twice_scale))
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(incidence) =
                BezierSelectedRadialCircleChordNestedExpression2::linear_combination(&[
                    (&constant, &Real::one()),
                    (&cross, &Real::one()),
                ])
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return Ok(map
                .selected_radial_nested_sign(&incidence, contact.branch, policy)?
                .map(Some));
        }
        if let Some(system) = map.oblique_system() {
            // The source contact P already satisfies |P-C|^2 = r0^2.
            // For Q = C + s(P-C) + T, reuse that exact selected-root
            // incidence instead of expanding two squared radical coordinates:
            //
            //   |Q-C|^2-r^2 = s^2 r0^2 + 2s T·(P-C) + |T|^2-r^2.
            //
            // With the retained positive homogeneous denominator D, only the
            // middle term carries the original square root. This is an exact
            // quotient-ring substitution at the same three selected roots.
            let negative_translation_x = -self.data.translation_x.clone();
            let negative_translation_y = -self.data.translation_y.clone();
            let Some(rational_dot) = TrivariatePolynomial2::linear_combination(&[
                (&system.point_x.rational, &self.data.translation_x),
                (&system.center_x, &negative_translation_x),
                (&system.point_y.rational, &self.data.translation_y),
                (&system.center_y, &negative_translation_y),
            ]) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(radical_dot) = TrivariatePolynomial2::linear_combination(&[
                (&system.point_x.radical, &self.data.translation_x),
                (&system.point_y.radical, &self.data.translation_y),
            ]) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let source_radius_squared =
                map.data.semicircle.radial_distance() * map.data.semicircle.radial_distance();
            let target_radius_squared = semicircle.radial_distance() * semicircle.radial_distance();
            let scale_squared = &self.data.radial_scale * &self.data.radial_scale;
            let translation_squared = &self.data.translation_x * &self.data.translation_x
                + &self.data.translation_y * &self.data.translation_y;
            let constant =
                scale_squared * source_radius_squared + translation_squared - target_radius_squared;
            let twice_scale = Real::from(2_i8) * &self.data.radial_scale;
            let Some(denominator_squared) = system
                .common_denominator
                .multiply(&system.common_denominator)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(rational_cross) = system
                .common_denominator
                .multiply(&rational_dot)
                .and_then(|cross| cross.scale(&twice_scale))
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(rational) = denominator_squared
                .scale(&constant)
                .and_then(|constant| constant.add(&rational_cross))
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(radical) = system
                .common_denominator
                .multiply(&radical_dot)
                .and_then(|cross| cross.scale(&twice_scale))
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return Ok(map
                .trivariate_radical_components_sign(&rational, &radical, contact.branch, policy)?
                .map(Some));
        }
        let Some(system) = map.axis_system() else {
            return Ok(Classification::Decided(None));
        };
        let square = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
            BezierAlgebraicCuspTwoTermExpression2 {
                rational: bivariate_add(
                    &bivariate_multiply(&expression.rational, &expression.rational),
                    &bivariate_multiply(
                        &bivariate_multiply(&expression.radical, &expression.radical),
                        &system.discriminant,
                    ),
                ),
                radical: bivariate_scale(
                    bivariate_multiply(&expression.rational, &expression.radical),
                    &Real::from(2_i8),
                ),
            }
        };
        let x_squared = square(
            &self
                .center_relative_coordinate_expression(Axis2::X)
                .expect("the selected axis map has an x expression"),
        );
        let y_squared = square(
            &self
                .center_relative_coordinate_expression(Axis2::Y)
                .expect("the selected axis map has a y expression"),
        );
        let incidence = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_subtract(
                &bivariate_add(&x_squared.rational, &y_squared.rational),
                &bivariate_scale(
                    bivariate_multiply(&system.common_denominator, &system.common_denominator),
                    &(semicircle.radial_distance() * semicircle.radial_distance()),
                ),
            ),
            radical: bivariate_add(&x_squared.radical, &y_squared.radical),
        };
        Ok(map
            .radical_sign(&incidence, contact.branch, policy)?
            .map(Some))
    }

    /// Orders one contact of `semicircle` with the certified axis support
    /// containing this point against the point itself.
    ///
    /// Both circles have the same retained center frame, so the derived point
    /// `Q = C + s(P - C) + T` remains in the original circle/chord radical
    /// system.  Evaluating `|Q-C|^2-r^2` there says whether `Q` is inside,
    /// on, or outside the current circle. Together with the exact sign of its
    /// directed axial coordinate, that completely orders either support-line
    /// contact without adjoining the current square root to the source field.
    pub(super) fn concentric_axis_contact_minus_point_sign(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
        direction: BezierAlgebraicChordAxisDirection2,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero {
            return Ok(Classification::Decided(None));
        }
        let Some((map, contact)) = self.data.source.chord_map_contact() else {
            return Ok(Classification::Decided(None));
        };
        map.validate_policy(policy)?;
        if map.data.semicircle.data.frame != semicircle.data.frame
            || map
                .axis_direction()
                .is_none_or(|map_direction| map_direction.axis() != direction.axis())
        {
            return Ok(Classification::Decided(None));
        }

        let directed_axis = {
            let coordinate = self
                .center_relative_coordinate_expression(direction.axis())
                .expect("an axis contact has an axis coordinate expression");
            if matches!(
                direction,
                BezierAlgebraicChordAxisDirection2::NegativeX
                    | BezierAlgebraicChordAxisDirection2::NegativeY
            ) {
                BezierAlgebraicCuspTwoTermExpression2 {
                    rational: bivariate_scale(coordinate.rational, &Real::from(-1_i8)),
                    radical: bivariate_scale(coordinate.radical, &Real::from(-1_i8)),
                }
            } else {
                coordinate
            }
        };
        let point_side = match map.radical_sign(&directed_axis, contact.branch, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if branch == 0 {
            return Ok(Classification::Decided(Some(match point_side {
                RealSign::Negative => RealSign::Positive,
                RealSign::Zero => RealSign::Zero,
                RealSign::Positive => RealSign::Negative,
            })));
        }
        let branch_side = if branch < 0 {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        if point_side == RealSign::Zero || point_side != branch_side {
            return Ok(Classification::Decided(Some(branch_side)));
        }

        let incidence = match self.concentric_circle_incidence_sign(semicircle, policy)? {
            Classification::Decided(Some(sign)) => sign,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Some(match incidence {
            RealSign::Negative => branch_side,
            RealSign::Zero => RealSign::Zero,
            RealSign::Positive => match branch_side {
                RealSign::Negative => RealSign::Positive,
                RealSign::Positive => RealSign::Negative,
                RealSign::Zero => unreachable!("a nonzero contact branch has a side"),
            },
        })))
    }

    pub(crate) fn same_point(&self, other: &Self, policy: &CurveContext) -> Classification<bool> {
        if self == other {
            return Classification::Decided(true);
        }
        if self.data.source.shares_exact_evidence(&other.data.source)
            && self.data.radial_scale == other.data.radial_scale
            && self.data.perpendicular_scale == other.data.perpendicular_scale
            && self.data.translation_x == other.data.translation_x
            && self.data.translation_y == other.data.translation_y
        {
            return Classification::Decided(true);
        }
        let identity_transform = |point: &Self| {
            point.data.radial_scale == Real::one()
                && point.data.perpendicular_scale == Real::zero()
                && point.data.translation_x == Real::zero()
                && point.data.translation_y == Real::zero()
        };
        if identity_transform(self)
            && identity_transform(other)
            && let (
                BezierAlgebraicCuspDerivedPointSource2::Mapped {
                    parameter: first, ..
                },
                BezierAlgebraicCuspDerivedPointSource2::Mapped {
                    parameter: second, ..
                },
            ) = (&self.data.source, &other.data.source)
        {
            if self.data.source.validate_policy(policy).is_err()
                || other.data.source.validate_policy(policy).is_err()
            {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
            let first_parameter = BezierAlgebraicCuspSemicircleParameter2::Mapped(first.clone());
            let second_parameter = BezierAlgebraicCuspSemicircleParameter2::Mapped(second.clone());
            if first_parameter.shares_coincident_point_evidence(&second_parameter)
                || first.shares_coincident_pair_point(second)
            {
                return Classification::Decided(true);
            }
        }
        if self.data.source == other.data.source {
            let same_source = self.same_source_transform(
                &other.data.radial_scale,
                &other.data.perpendicular_scale,
                &other.data.translation_x,
                &other.data.translation_y,
                policy,
            );
            if matches!(same_source, Classification::Decided(_)) {
                return same_source;
            }
        }
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (Classification::Decided(first), Classification::Decided(second)) = (
                self.conservative_bounds_refined(refinement_steps, policy),
                other.conservative_bounds_refined(refinement_steps, policy),
            ) else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            if first.overlaps(&second, policy) == Classification::Decided(false) {
                return Classification::Decided(false);
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Classification::Decided(true)
        } else {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
    }

    /// Materializes an affine derived point only when every nonrepresented
    /// quantity already belongs to one retained selected-parameter field.
    ///
    /// This is the exact bridge used by retained fillet reconstruction when a
    /// selected-circle/rational overlap supplies `P` and the circle center
    /// reduces under STRICT to represented coordinates. The general
    /// multi-field carrier deliberately remains procedural.
    pub(super) fn exact_one_field_point_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<RationalBezierAlgebraicPointImage2>> {
        self.data.source.validate_policy(policy)?;
        let BezierAlgebraicCuspDerivedPointSource2::Mapped {
            point: Some(CurvePoint2(CurvePointData2::Algebraic(point))),
            ..
        } = &self.data.source
        else {
            return Ok(None);
        };
        let Some(parameter) = point.retained_parameter().cloned() else {
            return Ok(None);
        };
        let Some(point) = point.resolved(policy) else {
            return Ok(None);
        };
        let Some((point_x, point_y, denominator)) = point.retained_coordinate_polynomials() else {
            return Ok(None);
        };
        if !self.data.source.semicircle().has_rational_frame() {
            return Ok(None);
        }
        let Some(center) = self
            .data
            .source
            .semicircle()
            .center_point_image(policy)?
            .exact_point(&CurveContext::STRICT)
        else {
            return Ok(None);
        };
        let radial = &self.data.radial_scale;
        let perpendicular = &self.data.perpendicular_scale;
        let one_minus_radial = Real::one() - radial;
        let x_constant =
            &one_minus_radial * center.x() + perpendicular * center.y() + &self.data.translation_x;
        let y_constant =
            &one_minus_radial * center.y() - perpendicular * center.x() + &self.data.translation_y;
        let x_numerator = polynomial_add(
            &polynomial_subtract(
                &polynomial_scale(point_x, radial),
                &polynomial_scale(point_y, perpendicular),
            ),
            &polynomial_scale(denominator, &x_constant),
        );
        let y_numerator = polynomial_add(
            &polynomial_add(
                &polynomial_scale(point_x, perpendicular),
                &polynomial_scale(point_y, radial),
            ),
            &polynomial_scale(denominator, &y_constant),
        );
        Ok(Some(
            RationalBezierAlgebraicPointImage2::from_retained_expression(
                parameter,
                point.parameter().clone(),
                x_numerator,
                y_numerator,
                denominator.to_vec(),
                "retained mapped contact affine image",
            ),
        ))
    }

    /// Compares a concentric image of a certified chord-normal contact with
    /// the same support's accumulated procedural normal offset.
    ///
    /// If `P = C + dN` is the authored contact, a concentric scale `s`
    /// produces `C + sdN`.  Retained chord parallels fold repeated offsets
    /// into exactly that signed distance on their base support.  Replaying
    /// this construction identity avoids adjoining the selected circle-center
    /// field to the chord endpoint fields merely to prove their shared point.
    pub(super) fn selected_chord_normal_parallel_distance_delta(
        &self,
        other: &BezierAlgebraicChordParallelPoint2,
        policy: &CurveContext,
    ) -> Option<(Real, BezierAlgebraicChord2)> {
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
            || !other.accepts_policy(policy)
            || other.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
            || other.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || other.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return None;
        }
        let BezierAlgebraicCuspDerivedPointSource2::Mapped { parameter, .. } = &self.data.source
        else {
            return None;
        };
        let (semicircle, chord, radial_product_sign, retained_policy) =
            match parameter.as_ref() {
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                    semicircle,
                    chord,
                    radial_product_sign,
                    policy,
                    ..
                }
                | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                    semicircle,
                    chord,
                    radial_product_sign,
                    policy,
                    ..
                } => (semicircle, chord, *radial_product_sign, *policy),
                _ => return None,
            };
        if !policy.accepts_retained_policy(retained_policy)
            || (!other.data.source.shares_retained_support(chord)
                && other.data.source.retained_support() != chord.retained_support())
        {
            return None;
        }
        let center = match semicircle.center_point_evidence(policy) {
            Ok(Classification::Decided(center)) => center,
            Ok(Classification::Uncertain(_)) | Err(_) => return None,
        };
        let origin = other.source_endpoint();
        if !origin.shares_storage(&center)
            && origin.same_point(&center, policy) != Classification::Decided(true)
        {
            return None;
        }
        let contact_distance = match radial_product_sign {
            RealSign::Positive => semicircle.radial_distance().clone(),
            RealSign::Negative => -semicircle.radial_distance().clone(),
            RealSign::Zero => return None,
        };
        let expected_distance = &self.data.radial_scale * contact_distance;
        let support_reversed = other.data.source.retained_support_orientation_is_reversed()
            != chord.retained_support_orientation_is_reversed();
        let retained_distance = if support_reversed {
            -other.data.distance.clone()
        } else {
            other.data.distance.clone()
        };
        Some((expected_distance - retained_distance, chord.clone()))
    }

    pub(super) fn same_selected_chord_normal_parallel_point(
        &self,
        other: &BezierAlgebraicChordParallelPoint2,
        policy: &CurveContext,
    ) -> Option<Classification<bool>> {
        let (distance_delta, _) =
            self.selected_chord_normal_parallel_distance_delta(other, policy)?;
        let equality = match real_sign(&distance_delta, policy) {
            Some(RealSign::Zero) => Classification::Decided(true),
            Some(RealSign::Positive | RealSign::Negative) => Classification::Decided(false),
            None => Classification::Uncertain(UncertaintyReason::RealSign),
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "contact-point-equality",
            match equality {
                Classification::Decided(true) => "concentric-chord-normal-offset-equal",
                Classification::Decided(false) => "concentric-chord-normal-offset-different",
                Classification::Uncertain(_) => "concentric-chord-normal-offset-uncertain",
            },
        );
        Some(equality)
    }

    /// Orders a concentric image of a transverse circle/chord cut against the
    /// same cut displaced along a cardinal chord normal.
    ///
    /// Let `P` be the retained cut, `R = P - C`, and `U` the oriented cardinal
    /// chord tangent.  A chord-normal displacement has no component along
    /// `U`, while the concentric image is `P + (s - 1)R`.  The rational contact
    /// map already retains `cross(T_circle, U) = -turn * dot(R, U)`, so the
    /// requested cardinal-coordinate order follows from three scalar signs.
    /// This consumes the original intersection correlation without rebuilding
    /// either selected coordinate field.
    pub(super) fn transverse_cardinal_chord_parallel_axis_order(
        &self,
        other: &BezierAlgebraicChordParallelPoint2,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<std::cmp::Ordering>>> {
        if self.data.perpendicular_scale.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
            || !other.accepts_policy(policy)
            || other.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
            || other.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || other.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return None;
        }
        let BezierAlgebraicCuspDerivedPointSource2::Mapped {
            parameter,
            point: Some(point),
        } = &self.data.source
        else {
            return None;
        };
        let (semicircle, chord, mut circle_cross_chord, retained_policy, circle_reversed) =
            parameter.coincident_chord_tangent_source()?;
        if circle_cross_chord == RealSign::Zero
            || !policy.accepts_retained_policy(retained_policy)
            || !other.data.source.shares_retained_support(chord)
        {
            return None;
        }
        let direction = chord.certified_axis_direction()?;
        if direction.axis() != axis {
            return None;
        }
        let origin = other.source_endpoint();
        if !origin.shares_storage(point)
            && origin.same_point(point, policy) != Classification::Decided(true)
        {
            return None;
        }
        if circle_reversed {
            circle_cross_chord = product_sign(circle_cross_chord, RealSign::Negative);
        }
        let scale_sign = match real_sign(&(&self.data.radial_scale - Real::one()), policy) {
            Some(sign) => sign,
            None => {
                return Some(Ok(Classification::Uncertain(UncertaintyReason::RealSign)));
            }
        };
        if scale_sign == RealSign::Zero {
            return Some(Ok(Classification::Decided(std::cmp::Ordering::Equal)));
        }
        let minus_turn = if semicircle.is_clockwise() {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let chord_axis_sign = match direction {
            BezierAlgebraicChordAxisDirection2::PositiveX
            | BezierAlgebraicChordAxisDirection2::PositiveY => RealSign::Positive,
            BezierAlgebraicChordAxisDirection2::NegativeX
            | BezierAlgebraicChordAxisDirection2::NegativeY => RealSign::Negative,
        };
        let coordinate_sign = product_sign(
            scale_sign,
            product_sign(
                product_sign(minus_turn, circle_cross_chord),
                chord_axis_sign,
            ),
        );
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-point-axis-order",
            "transverse-cardinal-circle-chord-offset",
        );
        Some(Ok(Classification::Decided(match coordinate_sign {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Positive => std::cmp::Ordering::Greater,
        })))
    }

    pub(super) fn selected_chord_normal_parallel_axis_order(
        &self,
        other: &BezierAlgebraicChordParallelPoint2,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<std::cmp::Ordering>>> {
        let Some((distance_delta, chord)) =
            self.selected_chord_normal_parallel_distance_delta(other, policy)
        else {
            return self.transverse_cardinal_chord_parallel_axis_order(other, axis, policy);
        };
        let distance_sign = match real_sign(&distance_delta, policy) {
            Some(sign) => sign,
            None => {
                return Some(Ok(Classification::Uncertain(UncertaintyReason::RealSign)));
            }
        };
        if distance_sign == RealSign::Zero {
            return Some(Ok(Classification::Decided(std::cmp::Ordering::Equal)));
        }
        let (coefficient_x, coefficient_y) = match axis {
            // The chord's left normal is `(-tangent_y, tangent_x)`.
            Axis2::X => (Real::zero(), -Real::one()),
            Axis2::Y => (Real::one(), Real::zero()),
        };
        let normal_component_sign =
            match chord.tangent_linear_form_sign(&coefficient_x, &coefficient_y, policy) {
                Ok(Classification::Decided(sign)) => sign,
                Ok(Classification::Uncertain(reason)) => {
                    return Some(Ok(Classification::Uncertain(reason)));
                }
                Err(error) => return Some(Err(error)),
            };
        Some(Ok(Classification::Decided(
            match product_sign(distance_sign, normal_component_sign) {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            },
        )))
    }

    pub(crate) fn same_point_evidence(
        &self,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if self.data.radial_scale == Real::one()
            && self.data.perpendicular_scale == Real::zero()
            && self.data.translation_x == Real::zero()
            && self.data.translation_y == Real::zero()
        {
            let original = self.data.source.same_original_point(other, policy);
            if matches!(original, Classification::Decided(_)) {
                return original;
            }
        }
        match other {
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(other)) => {
                self.same_point(other, policy)
            }
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(other))
                if matches!(
                    &self.data.source,
                    BezierAlgebraicCuspDerivedPointSource2::Chord(source) if source == other
                ) =>
            {
                self.same_source_transform(
                    &Real::one(),
                    &Real::zero(),
                    &Real::zero(),
                    &Real::zero(),
                    policy,
                )
            }
            CurvePoint2(CurvePointData2::Exact(point)) => {
                let x = self.axis_coordinate_order_to_real(Axis2::X, point.x(), policy);
                let y = self.axis_coordinate_order_to_real(Axis2::Y, point.y(), policy);
                match (x, y) {
                    (
                        Ok(Classification::Decided(std::cmp::Ordering::Equal)),
                        Ok(Classification::Decided(std::cmp::Ordering::Equal)),
                    ) => Classification::Decided(true),
                    (
                        Ok(Classification::Decided(
                            std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                        )),
                        _,
                    )
                    | (
                        _,
                        Ok(Classification::Decided(
                            std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                        )),
                    ) => Classification::Decided(false),
                    (Ok(Classification::Uncertain(reason)), _)
                    | (_, Ok(Classification::Uncertain(reason))) => {
                        Classification::Uncertain(reason)
                    }
                    _ => Classification::Uncertain(UncertaintyReason::Unsupported),
                }
            }
            CurvePoint2(CurvePointData2::AnalyticParallel(other)) => {
                let support = self.exact_supporting_circle(policy);
                if let Ok(Some((center, radius_squared))) = support {
                    let residual =
                        other.circle_residual_sign_to_exact(&center, &radius_squared, policy);
                    match residual {
                        Ok(Classification::Decided(RealSign::Positive | RealSign::Negative)) => {
                            return Classification::Decided(false);
                        }
                        Ok(
                            Classification::Decided(RealSign::Zero) | Classification::Uncertain(_),
                        )
                        | Err(_) => {}
                    }
                }
                retained_point_evidence_equality_by_refinement(
                    &CurvePoint2::from(self.clone()),
                    &CurvePoint2::from(other.clone()),
                    policy,
                )
            }
            CurvePoint2(CurvePointData2::Similarity(similarity))
                if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(source)) =
                    &similarity.data.source
                    && self.is_similarity_image_of(source, &similarity.data.transform, policy) =>
            {
                Classification::Decided(true)
            }
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(other)) => {
                if let Some(classification) =
                    self.same_selected_chord_normal_parallel_point(other, policy)
                {
                    return classification;
                }
                retained_point_evidence_equality_by_refinement(
                    &CurvePoint2::from(self.clone()),
                    &CurvePoint2::from(other.clone()),
                    policy,
                )
            }
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                retained_point_evidence_equality_by_refinement(
                    &CurvePoint2::from(self.clone()),
                    other,
                    policy,
                )
            }
            CurvePoint2(CurvePointData2::Algebraic(other)) => {
                if let Ok(Some(point)) = self.exact_one_field_point_image(policy)
                    && let Ok(Some(classification)) =
                        point.same_retained_rational_point(other, policy)
                {
                    return classification;
                }
                let other_evidence = CurvePoint2::from(other.clone());
                retained_point_evidence_equality_by_refinement(
                    &CurvePoint2::from(self.clone()),
                    &other_evidence,
                    policy,
                )
            }
        }
    }
}
