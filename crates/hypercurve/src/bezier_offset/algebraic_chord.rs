//! Selected algebraic chord carrier: construction, retained parallels, incidence, contacts and parameters.

use super::*;

mod chord_kernel;
mod parallel_kernel;
mod points;
mod rational_kernel;
mod tangent;

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

    pub(crate) fn shares_retained_support(&self, other: &Self) -> bool {
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
        let reversal = self.retained_normal_offset_tangent_reversal_to(other)?;
        let own = self.retained_normal_offset_distance_with_tangent_reversal(reversal)?;
        let (base, _) = self.retained_normal_offset_base_orientation()?;
        if other.shares_retained_support(base) || other.retained_support() == base {
            return Some(own);
        }
        // `other` is itself a normal offset of the same base. Both signed
        // displacements are measured from that base in `other`'s left-normal
        // frame, so the separation is their difference, not `self`'s
        // displacement alone.
        let other_own = other.retained_normal_offset_distance_with_tangent_reversal(false)?;
        Some(own - other_own)
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
            let start_x = real_interval_from_axis(&start, Axis2::X);
            let start_y = real_interval_from_axis(&start, Axis2::Y);
            let direction_x = real_interval_from_axis(&end, Axis2::X).subtract(&start_x);
            let direction_y = real_interval_from_axis(&end, Axis2::Y).subtract(&start_y);
            let point_x = real_interval_from_axis(&point, Axis2::X)
                .subtract(&start_x)
                .subtract(&exact(&support.translation_x));
            let point_y = real_interval_from_axis(&point, Axis2::Y)
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
                    RecursiveQuadraticValue::affine_positive_root_sign(
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
            let delta_x = real_interval_from_axis(&point, Axis2::X)
                .subtract(&real_interval_from_axis(&origin, Axis2::X));
            let delta_y = real_interval_from_axis(&point, Axis2::Y)
                .subtract(&real_interval_from_axis(&origin, Axis2::Y));
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
                // An axis-aligned map keeps a coordinate image's represented
                // roots, so equality against other carriers' contacts stays
                // decidable without the retained-expression fallback below.
                let structurally_zero =
                    |value: &Real| value.zero_status() == hyperreal::ZeroKnowledge::Zero;
                if structurally_zero(m01)
                    && structurally_zero(m10)
                    && let Some(image) = point.axis_affine_image(m00, tx, m11, ty)
                {
                    return Ok(Classification::Decided(CurvePoint2::from(image)));
                }
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
                .transform_affine(
                    m00,
                    m01,
                    m10,
                    m11,
                    tx,
                    ty,
                    similarity,
                    similarity_cache,
                    policy,
                )?
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
                if let decided @ Classification::Decided(_) = policy
                    .strict_predicate_pass(|| line.contains_point_with_policy(represented, policy))
                {
                    return Ok(decided);
                }
            } else if self.has_composite_endpoint() {
                if let Classification::Decided(bounds) =
                    self.conservative_local_bounds_refined(0, policy)?
                    && bounds.contains_point_with_policy(represented, &CurveContext::STRICT)
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
                return Ok(match line.contains_point_with_policy(origin, policy) {
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
