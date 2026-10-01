//! Cusp/chord contact points and derived point sources.

use super::*;

impl BezierAlgebraicCuspChordPoint2 {
    pub(in crate::bezier_offset) fn map_contact(
        &self,
    ) -> (
        &BezierAlgebraicCuspSemicircleChordParameterMap2,
        &BezierAlgebraicCuspSemicircleChordContact2,
    ) {
        let BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { map, contact } =
            self.data.as_ref()
        else {
            unreachable!("cusp/chord point must retain a cusp/chord parameter map")
        };
        (map, contact)
    }

    /// Proves another point distinct from this circle/chord contact whenever
    /// it lies strictly off the retained contact support. The contact itself
    /// is on that support by construction, so no point equality or coordinate
    /// compositum is required.
    pub(in crate::bezier_offset) fn contact_support_separates_point(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let (map, _) = self.map_contact();
        map.validate_policy(policy)?;
        let support = match BezierAlgebraicChordSupportPredicate2::try_new(&map.data.chord, policy)?
        {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(support
            .oriented_side(point, policy)?
            .map(|side| side != crate::classify::LineSide::On))
    }

    pub(in crate::bezier_offset) fn recursive_projective_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        let (map, contact) = self.map_contact();
        Ok(map
            .recursive_contact_frame(contact, policy)?
            .map(|frame| frame.map(|frame| frame.point)))
    }

    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }

    /// Returns the affine-line parameter retained by the authoritative
    /// recursive quadratic contact map. A native line promoted to a chord
    /// cell can reuse this same scalar on its source support, so final region
    /// trimming neither reconstructs the contact nor projects a global norm.
    pub(crate) fn recursive_quadratic_line_parameter(
        &self,
    ) -> Option<BezierRecursiveProjectiveParameter2> {
        let (map, contact) = self.map_contact();
        map.recursive_quadratic_line_system()
            .and_then(|system| system.contact(contact.branch).ok())
            .map(|contact| contact.parameter.clone())
    }

    pub(in crate::bezier_offset) fn translated(
        &self,
        translation_x: &Real,
        translation_y: &Real,
    ) -> BezierAlgebraicCuspChordDerivedPoint2 {
        BezierAlgebraicCuspChordDerivedPoint2 {
            data: Arc::new(BezierAlgebraicCuspChordDerivedPointData2 {
                source: BezierAlgebraicCuspDerivedPointSource2::Chord(self.clone()),
                radial_scale: Real::one(),
                perpendicular_scale: Real::zero(),
                translation_x: translation_x.clone(),
                translation_y: translation_y.clone(),
            }),
        }
    }

    pub(in crate::bezier_offset) fn radial_scaled(
        &self,
        radial_scale: Real,
    ) -> BezierAlgebraicCuspChordDerivedPoint2 {
        BezierAlgebraicCuspChordDerivedPoint2 {
            data: Arc::new(BezierAlgebraicCuspChordDerivedPointData2 {
                source: BezierAlgebraicCuspDerivedPointSource2::Chord(self.clone()),
                radial_scale,
                perpendicular_scale: Real::zero(),
                translation_x: Real::zero(),
                translation_y: Real::zero(),
            }),
        }
    }

    pub(crate) fn conservative_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        let (map, contact) = self.map_contact();
        if map.validate_policy(policy).is_err() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        map.contact_bounds_refined(contact, refinement_steps)
    }

    pub(in crate::bezier_offset) fn represented_coordinates(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let (map, contact) = self.map_contact();
        map.validate_policy(policy)?;
        if map.represented_oblique_system().is_some() {
            Ok(Classification::Decided(
                map.represented_oblique_contact(contact)?.point.clone(),
            ))
        } else if map.oblique_system().is_some() {
            map.oblique_represented_contact_coordinates(contact)
        } else if map.axis_system().is_some() {
            map.axis_represented_coordinates(contact)
        } else if let Some(system) = map.recursive_quadratic_line_system() {
            // The recursive line map already owns the authoritative selected
            // contact point. Publish its two standalone coordinates only for
            // this cold consumer instead of rebuilding an obsolete dense
            // circle/chord system.
            system
                .contact(contact.branch)?
                .point
                .represented_coordinates(policy)
        } else if map.chord_normal_projective_system().is_some() {
            map.chord_normal_dense_represented_coordinates(contact)
        } else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-cusp-chord-blocker",
                match &map.data.system {
                    BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Axis(_) => "axis",
                    BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Oblique(_) => "oblique",
                    BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(_) => {
                        "represented-oblique"
                    }
                    BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RetainedOffset(_) => {
                        "retained-offset"
                    }
                    BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(
                        _,
                    ) => "recursive-quadratic-line",
                    BezierAlgebraicCuspSemicircleChordParameterMapSystem2::SelectedRadial(_) => {
                        "selected-radial"
                    }
                    BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(
                        _,
                    ) => "chord-normal-projective",
                },
            );
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
    }

    pub(in crate::bezier_offset) fn axis_coordinate_order_to_real(
        &self,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let (x_factor, y_factor) = match axis {
            Axis2::X => (Real::one(), Real::zero()),
            Axis2::Y => (Real::zero(), Real::one()),
        };
        self.linear_order_to_real(&x_factor, &y_factor, value, policy)
    }

    pub(in crate::bezier_offset) fn linear_order_to_real(
        &self,
        x_factor: &Real,
        y_factor: &Real,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let (map, contact) = self.map_contact();
        let zero = Real::zero();
        map.affine_order(
            contact,
            [x_factor, y_factor],
            [&zero, &zero],
            &(-value),
            policy,
        )
    }

    pub(in crate::bezier_offset) fn same_point(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if self == other {
            return Classification::Decided(true);
        }
        let (map, _) = self.map_contact();
        if let Ok(Some(order)) = self.collinear_semicircle_contact_order(
            other,
            BezierAlgebraicChordParameterAxis2 {
                axis: map.data.chord.data.parameter_axis.axis,
                coordinate_increases: true,
            },
            policy,
        ) {
            return Classification::Decided(order == std::cmp::Ordering::Equal);
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

    pub(in crate::bezier_offset) fn collinear_semicircle_contact_order(
        &self,
        other: &Self,
        target_axis: BezierAlgebraicChordParameterAxis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<std::cmp::Ordering>> {
        let (first_map, first_contact) = self.map_contact();
        let (second_map, second_contact) = other.map_contact();
        first_map.validate_policy(policy)?;
        second_map.validate_policy(policy)?;
        if first_map.data.semicircle != second_map.data.semicircle
            || first_map.data.chord.data.parameter_axis.axis != target_axis.axis
            || second_map.data.chord.data.parameter_axis.axis != target_axis.axis
        {
            return Ok(None);
        }
        let collinear = if first_map
            .data
            .chord
            .shares_retained_support(&second_map.data.chord)
        {
            Classification::Decided(true)
        } else {
            first_map
                .data
                .chord
                .support_collinearity(&second_map.data.chord, policy)?
        };
        if collinear != Classification::Decided(true) {
            return Ok(None);
        }
        let first_represented = first_map.represented_oblique_system().is_some();
        let second_represented = second_map.represented_oblique_system().is_some();
        if first_represented || second_represented {
            if !first_represented
                || !second_represented
                || !Arc::ptr_eq(&first_map.data, &second_map.data)
            {
                return Ok(None);
            }
            let first = &first_map
                .represented_oblique_contact(first_contact)?
                .chord_parameter;
            let second = &second_map
                .represented_oblique_contact(second_contact)?
                .chord_parameter;
            let difference = match represented_affine_coordinate(
                &[(first, &Real::one()), (second, &Real::from(-1_i8))],
                &Real::zero(),
            ) {
                Classification::Decided(difference) => difference,
                Classification::Uncertain(_) => return Ok(None),
            };
            let order = match represented_policy_sign(&difference, policy) {
                Classification::Decided(RealSign::Negative) => std::cmp::Ordering::Less,
                Classification::Decided(RealSign::Zero) => std::cmp::Ordering::Equal,
                Classification::Decided(RealSign::Positive) => std::cmp::Ordering::Greater,
                Classification::Uncertain(_) => return Ok(None),
            };
            return Ok(Some(
                if first_map
                    .data
                    .chord
                    .data
                    .parameter_axis
                    .coordinate_increases
                    == target_axis.coordinate_increases
                {
                    order
                } else {
                    order.reverse()
                },
            ));
        }
        let first_projective = first_map.has_chord_normal_projective_system();
        let second_projective = second_map.has_chord_normal_projective_system();
        if first_projective || second_projective {
            if !first_projective
                || !second_projective
                || !Arc::ptr_eq(&first_map.data, &second_map.data)
            {
                return Ok(None);
            }
            let order = match first_map
                .chord_normal_projective_parameter(first_contact)?
                .cmp_by_refinement(
                    second_map.chord_normal_projective_parameter(second_contact)?,
                    policy,
                )? {
                Classification::Decided(order) => order,
                Classification::Uncertain(_) => return Ok(None),
            };
            return Ok(Some(
                if first_map
                    .data
                    .chord
                    .data
                    .parameter_axis
                    .coordinate_increases
                    == target_axis.coordinate_increases
                {
                    order
                } else {
                    order.reverse()
                },
            ));
        }
        let orient = |map: &BezierAlgebraicCuspSemicircleChordParameterMap2, branch: i8| {
            if map.data.chord.data.parameter_axis.coordinate_increases
                == target_axis.coordinate_increases
            {
                branch
            } else {
                -branch
            }
        };
        Ok(Some(
            orient(first_map, first_contact.branch).cmp(&orient(second_map, second_contact.branch)),
        ))
    }

    pub(in crate::bezier_offset) fn cmp_on_axis_by_refinement(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Classification<std::cmp::Ordering> {
        if self == other {
            return Classification::Decided(std::cmp::Ordering::Equal);
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
            let (first_min, first_max, second_min, second_max) = match axis {
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
            if compare_reals(first_max, second_min, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Classification::Decided(std::cmp::Ordering::Less);
            }
            if compare_reals(second_max, first_min, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Classification::Decided(std::cmp::Ordering::Greater);
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Classification::Decided(std::cmp::Ordering::Equal)
        } else {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
    }

    pub(crate) fn same_point_evidence(
        &self,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if let Some(point) = self.data.retained_point_evidence() {
            let (map, _) = self.map_contact();
            if map.validate_policy(policy).is_err() {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
            return point.same_point(other, policy);
        }
        match other {
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(other)) => {
                self.same_point(other, policy)
            }
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(other)) => {
                other.same_point_evidence(&CurvePoint2::from(self.clone()), policy)
            }
            CurvePoint2(CurvePointData2::Exact(point)) => {
                let (map, contact) = self.map_contact();
                if let Some(direction) = map.axis_direction() {
                    let exact = CurvePoint2::from(point.clone());
                    if let Ok(Classification::Decided(contact_delta)) =
                        map.data.semicircle.axis_chord_contact_minus_point_sign(
                            &exact,
                            direction,
                            contact.branch,
                            policy,
                        )
                    {
                        if contact_delta != RealSign::Zero {
                            return Classification::Decided(false);
                        }
                        let constant_axis = match direction.axis() {
                            Axis2::X => Axis2::Y,
                            Axis2::Y => Axis2::X,
                        };
                        if let Ok(Classification::Decided(support_order)) = self
                            .axis_coordinate_order_to_real(
                                constant_axis,
                                match constant_axis {
                                    Axis2::X => point.x(),
                                    Axis2::Y => point.y(),
                                },
                                policy,
                            )
                        {
                            return Classification::Decided(
                                support_order == std::cmp::Ordering::Equal,
                            );
                        }
                    }
                }
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
            CurvePoint2(CurvePointData2::Algebraic(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                retained_point_evidence_equality_by_refinement(
                    &CurvePoint2::from(self.clone()),
                    other,
                    policy,
                )
            }
        }
    }

    pub(in crate::bezier_offset) fn cmp_on_chord_to_parameter(
        &self,
        chord: &BezierAlgebraicChord2,
        other: &BezierAlgebraicChordParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let (map, contact) = self.map_contact();
        map.validate_policy(policy)?;
        let shared_domain = map.data.chord.shares_retained_support(chord)
            && map.data.chord.shares_retained_support(other.chord());
        if shared_domain {
            if map.data.finite_chord_domain
                && let BezierAlgebraicChordParameterStorage2::Endpoint { at_end, .. } = &other.data
            {
                return Ok(Classification::Decided(if *at_end {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                }));
            }
            if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(other_point)) = other.point() {
                if self == other_point {
                    return Ok(Classification::Decided(std::cmp::Ordering::Equal));
                }
                if let Some(order) = self.collinear_semicircle_contact_order(
                    other_point,
                    chord.data.parameter_axis,
                    policy,
                )? {
                    return Ok(Classification::Decided(order));
                }
                let axis = chord.data.parameter_axis;
                let order = self.cmp_on_axis_by_refinement(other_point, axis.axis, policy);
                if let Classification::Decided(order) = order {
                    return Ok(Classification::Decided(if axis.coordinate_increases {
                        order
                    } else {
                        order.reverse()
                    }));
                }
            }
            if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(other_point)) =
                other.point()
                && let BezierAlgebraicCuspDerivedPointSource2::Chord(source) =
                    &other_point.data.source
                && source == self
                && other_point.data.radial_scale == Real::one()
                && other_point.data.perpendicular_scale == Real::zero()
            {
                let axis = chord.data.parameter_axis;
                let (delta, perpendicular) = match axis.axis {
                    Axis2::X => (
                        &other_point.data.translation_x,
                        &other_point.data.translation_y,
                    ),
                    Axis2::Y => (
                        &other_point.data.translation_y,
                        &other_point.data.translation_x,
                    ),
                };
                if perpendicular.zero_status() == ZeroKnowledge::Zero {
                    let sign = match real_sign(delta, policy) {
                        Some(sign) => sign,
                        None => {
                            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                        }
                    };
                    let coordinate_order = match sign {
                        RealSign::Negative => std::cmp::Ordering::Greater,
                        RealSign::Zero => std::cmp::Ordering::Equal,
                        RealSign::Positive => std::cmp::Ordering::Less,
                    };
                    return Ok(Classification::Decided(if axis.coordinate_increases {
                        coordinate_order
                    } else {
                        coordinate_order.reverse()
                    }));
                }
            }
            if let Some(direction) = map.axis_direction()
                && let Classification::Decided(sign) =
                    map.data.semicircle.axis_chord_contact_minus_point_sign(
                        other.point(),
                        direction,
                        contact.branch,
                        policy,
                    )?
            {
                let order = match sign {
                    RealSign::Negative => std::cmp::Ordering::Less,
                    RealSign::Zero => std::cmp::Ordering::Equal,
                    RealSign::Positive => std::cmp::Ordering::Greater,
                };
                return Ok(Classification::Decided(
                    if direction.parameter_axis().coordinate_increases
                        == chord.data.parameter_axis.coordinate_increases
                    {
                        order
                    } else {
                        order.reverse()
                    },
                ));
            }
            // The source contact's correlated axis predicate is a fast path,
            // not the only authority for the shared support. A derived
            // fillet/chamfer point can retain a different exact field and
            // still be ordered by the monotone coordinate below.
        }
        if chord.shares_retained_support(other.chord()) {
            let axis = chord.data.parameter_axis;
            let point = CurvePoint2::from(self.clone());
            let order =
                algebraic_chord_point_coordinate_order(&point, other.point(), axis.axis, policy)?;
            return Ok(if axis.coordinate_increases {
                order
            } else {
                order.map(std::cmp::Ordering::reverse)
            });
        }
        Err(CurveError::Topology(
            "cusp/chord point was compared on an unrelated chord".into(),
        ))
    }

    pub(in crate::bezier_offset) fn oriented_side_to_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        let (map, _) = self.map_contact();
        map.validate_policy(policy)?;
        if map.data.chord.shares_retained_support(chord) {
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
}

impl BezierAlgebraicCuspDerivedPointSource2 {
    /// Returns whether two affine derivations start from the same exact mapped
    /// point.  The optional point field is only an alternate evaluation cache;
    /// mapped-parameter evidence remains the geometric authority.
    pub(in crate::bezier_offset) fn shares_exact_evidence(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Chord(first), Self::Chord(second)) => first == second,
            (
                Self::Mapped {
                    parameter: first, ..
                },
                Self::Mapped {
                    parameter: second, ..
                },
            ) => BezierAlgebraicCuspSemicircleParameter2::Mapped(first.clone())
                .shares_exact_evidence(&BezierAlgebraicCuspSemicircleParameter2::Mapped(
                    second.clone(),
                )),
            (Self::Chord(_), Self::Mapped { .. }) | (Self::Mapped { .. }, Self::Chord(_)) => false,
        }
    }

    pub(in crate::bezier_offset) fn chord_map_contact(
        &self,
    ) -> Option<(
        &BezierAlgebraicCuspSemicircleChordParameterMap2,
        &BezierAlgebraicCuspSemicircleChordContact2,
    )> {
        match self {
            Self::Chord(point) => Some(point.map_contact()),
            Self::Mapped { parameter, .. } => parameter
                .coincident_chord_source()
                .map(|(map, contact, _)| (map, contact)),
        }
    }

    pub(in crate::bezier_offset) fn direct_pair_map_contact(
        &self,
    ) -> Option<(
        &BezierAlgebraicCuspSemicirclePairParameterMap2,
        &BezierAlgebraicCuspSemicirclePairContact2,
        bool,
    )> {
        let Self::Mapped { parameter, .. } = self else {
            return None;
        };
        let BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
            map,
            contact,
            first,
        } = parameter.as_ref()
        else {
            return None;
        };
        Some((map, contact, *first))
    }

    /// Recovers the pair contact below exact coincident-circle transports.
    /// Those wrappers change only the selected chart, so the represented
    /// Cartesian point remains the pair map's existing contact point.
    pub(in crate::bezier_offset) fn coincident_pair_map_contact(
        &self,
    ) -> Option<(
        &BezierAlgebraicCuspSemicirclePairParameterMap2,
        &BezierAlgebraicCuspSemicirclePairContact2,
        bool,
    )> {
        let Self::Mapped { parameter, .. } = self else {
            return None;
        };
        let (map, contact, first, _) = parameter.coincident_pair_source()?;
        Some((map, contact, first))
    }

    pub(in crate::bezier_offset) fn semicircle(&self) -> &BezierAlgebraicCuspSemicircle2 {
        match self {
            Self::Chord(point) => &point.map_contact().0.data.semicircle,
            Self::Mapped { parameter, .. } => parameter.semicircle_carrier(),
        }
    }

    pub(in crate::bezier_offset) fn validate_policy(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<()> {
        match self {
            Self::Chord(point) => point.map_contact().0.validate_policy(policy),
            Self::Mapped { parameter, .. } => {
                BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter.clone())
                    .validate_policy(policy)
            }
        }
    }

    pub(in crate::bezier_offset) fn rational_contact_source(
        &self,
    ) -> Option<(&RationalBezier2, CurveContext)> {
        let Self::Mapped { parameter, .. } = self else {
            return None;
        };
        match parameter.coincident_base_data() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, .. } => {
                Some((&map.data.curve, map.data.policy))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                ..
            } => Some((&map.data.curve, map.data.policy)),
            _ => None,
        }
    }

    /// Returns `cross(P-C, T)` when this source is a mapped circle contact and
    /// `T` is an authored source-tangent chord at that same contact.
    ///
    /// The contact map already owns `dot(T_circle, T_source)`, while
    /// `T_circle = turn * J(P-C)` up to a positive chart scale. Hence
    /// `cross(P-C, T_source) = turn * dot(T_circle, T_source)`. Multiplying by
    /// the analytic chord's retained tangent-displacement sign recovers its
    /// actual traversal without constructing either endpoint field.
    pub(in crate::bezier_offset) fn radial_cross_authored_tangent_sign(
        &self,
        tangent: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let Self::Mapped { parameter, .. } = self else {
            return None;
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            let kind = match parameter.as_ref() {
                BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { .. } => "rational",
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                    ..
                } => "selected-rational",
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                    ..
                } => "selected-parallel",
                BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { .. } => "parallel",
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                    ..
                } => "selected-contact",
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                    ..
                } => "selected-circular",
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact { .. } => {
                    "selected-pair"
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                    ..
                } => "selected-chord-normal",
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                    ..
                } => "selected-chord-parallel-normal",
                BezierAlgebraicCuspSemicircleMappedParameterData2::Pair { .. } => "pair",
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. } => "chord",
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap { .. } => {
                    "pair-overlap"
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap { .. } => {
                    "pair-overlap-map"
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                    ..
                } => "similarity",
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer { .. } => "chamfer",
            };
            eprintln!("radial/source tangent source kind={kind}");
        }
        let displacement_sign = match parameter.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, contact } => {
                tangent.authored_source_tangent_displacement_sign(
                    &contact.other_parameter,
                    None,
                    policy,
                    |source| {
                        matches!(
                            source,
                            BezierParallelSource2::Rational(curve) if curve == &map.data.curve
                        )
                    },
                )
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } => tangent.authored_source_tangent_displacement_sign(
                &CurveParameter2::from_selected_fiber(other_parameter.clone()),
                None,
                policy,
                |source| {
                    matches!(
                        source,
                        BezierParallelSource2::Rational(curve) if curve == &map.data.curve
                    )
                },
            ),
            BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact } => {
                tangent.authored_source_tangent_displacement_sign(
                    &CurveParameter2::from(contact.parallel_parameter.clone()),
                    None,
                    policy,
                    |source| source == map.data.parallel.source(),
                )
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map,
                other_parameter,
                ..
            } => tangent.authored_source_tangent_displacement_sign(
                &CurveParameter2::from_selected_fiber(other_parameter.clone()),
                None,
                policy,
                |source| source == map.data.parallel.source(),
            ),
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::Pair { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer { .. } => None,
        }?;
        let tangent_dot = match parameter
            .ordinary_carrier_tangent_cross_dot_linear_combination_sign(
                &Real::zero(),
                &Real::one(),
                policy,
            ) {
            Ok(Some(sign)) => sign,
            Ok(None) => return None,
            Err(error) => return Some(Err(error)),
        };
        let turn_sign = if parameter.semicircle_carrier().is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        Some(Ok(tangent_dot.map(|sign| {
            product_sign(sign, product_sign(turn_sign, displacement_sign))
        })))
    }

    /// Recognizes a coordinate that is constant on the same retained
    /// rational contact carrier. The two selected-fiber parameters may be
    /// unrelated algebraic roots; the rational Bernstein control net proves
    /// the coordinate identity without comparing either root.
    /// Returns the retained exact source point evidence, if any.
    pub(in crate::bezier_offset) fn retained_point(&self) -> Option<CurvePoint2> {
        match self {
            Self::Chord(point) => Some(CurvePoint2::from(point.clone())),
            Self::Mapped {
                point: Some(point), ..
            } => Some(point.clone()),
            Self::Mapped { parameter, .. } => parameter.retained_point_evidence().cloned(),
        }
    }

    pub(in crate::bezier_offset) fn common_rational_constant_axis(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Option<std::cmp::Ordering> {
        let ((first, first_policy), (second, second_policy)) = (
            self.rational_contact_source()?,
            other.rational_contact_source()?,
        );
        if !policy.accepts_retained_policy(first_policy)
            || !policy.accepts_retained_policy(second_policy)
            || first != second
        {
            return None;
        }
        let value = match axis {
            Axis2::X => first.start().x(),
            Axis2::Y => first.start().y(),
        };
        first
            .homogeneous_controls()
            .iter()
            .all(|control| {
                let coordinate = match axis {
                    Axis2::X => control.x(),
                    Axis2::Y => control.y(),
                };
                compare_reals(
                    coordinate,
                    &(value * control.weight()),
                    &CurveContext::STRICT,
                ) == Some(std::cmp::Ordering::Equal)
            })
            .then_some(std::cmp::Ordering::Equal)
    }

    pub(in crate::bezier_offset) fn conservative_bounds_refined_impl(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
        local_only: bool,
    ) -> Classification<Aabb2> {
        match self {
            Self::Chord(point) => point.conservative_bounds_refined(refinement_steps, policy),
            Self::Mapped {
                point: Some(point), ..
            } => {
                if local_only {
                    algebraic_chord_endpoint_local_bounds_refined(point, refinement_steps, policy)
                } else {
                    algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
                }
            }
            Self::Mapped {
                parameter,
                point: None,
            } => parameter
                .selected_fiber_point_bounds_refined(refinement_steps, policy)
                .or_else(|| {
                    parameter.coincident_rational_point_bounds_refined(refinement_steps, policy)
                })
                .or_else(|| {
                    parameter.coincident_parallel_point_bounds_refined(refinement_steps, policy)
                })
                .or_else(|| {
                    parameter.coincident_chord_point_bounds_refined(refinement_steps, policy)
                })
                .or_else(|| {
                    parameter.coincident_pair_point_bounds_refined(refinement_steps, policy)
                })
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
        }
    }

    pub(in crate::bezier_offset) fn same_original_point(
        &self,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        match self {
            Self::Chord(point) => point.same_point_evidence(other, policy),
            Self::Mapped {
                point: Some(point), ..
            } => point.same_point(other, policy),
            Self::Mapped {
                parameter,
                point: None,
            } => {
                if let (
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                        map,
                        other_parameter,
                        ..
                    },
                    CurvePoint2(CurvePointData2::Exact(point)),
                ) = (parameter.as_ref(), other)
                {
                    let x = map.point_coordinate_order_to_real(
                        other_parameter,
                        Axis2::X,
                        point.x(),
                        policy,
                    );
                    let y = map.point_coordinate_order_to_real(
                        other_parameter,
                        Axis2::Y,
                        point.y(),
                        policy,
                    );
                    return match (x, y) {
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
                    };
                }
                Classification::Uncertain(UncertaintyReason::Predicate)
            }
        }
    }
}
