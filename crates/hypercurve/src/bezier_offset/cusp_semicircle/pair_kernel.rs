//! Coincident, translated and represented circle-pair intersections.

use super::*;

impl BezierAlgebraicCuspSemicircle2 {
    /// Publishes the single coincident-support topology classification from
    /// exact start-radial signs. The identical-frame fast path and general
    /// represented frames both enter here.
    pub(in crate::bezier_offset) fn coincident_pair_intersections_from_radial_signs(
        &self,
        other: &Self,
        radial_cross: RealSign,
        radial_dot: Option<RealSign>,
        general_parameter_map: Option<BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicirclePairIntersections2>> {
        let orientation = if self.is_clockwise() == other.is_clockwise() {
            CurveOverlapOrientation2::Same
        } else {
            CurveOverlapOrientation2::Reversed
        };
        let (first_boundaries, second_boundaries) = if radial_cross == RealSign::Zero {
            let radial_same = match radial_dot.ok_or_else(|| {
                CurveError::Topology("a coincident circle map lost its radial dot sign".into())
            })? {
                RealSign::Positive => true,
                RealSign::Negative => false,
                RealSign::Zero => {
                    return Err(CurveError::Topology(
                        "coincident nonzero cusp-circle radii had zero dot and cross".into(),
                    ));
                }
            };
            let traversal_same = orientation == CurveOverlapOrientation2::Same;
            if radial_same != traversal_same {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(
                        Self::coincident_endpoint_contacts(radial_same),
                    ),
                ));
            }
            (
                [
                    BezierAlgebraicCuspSemicirclePairEndpoint2::FirstStart,
                    BezierAlgebraicCuspSemicirclePairEndpoint2::FirstEnd,
                ],
                [
                    BezierAlgebraicCuspSemicirclePairEndpoint2::SecondStart,
                    BezierAlgebraicCuspSemicirclePairEndpoint2::SecondEnd,
                ],
            )
        } else {
            let first_turn = if self.is_clockwise() {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            let second_turn = if other.is_clockwise() {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            let second_inside_first =
                if product_sign(first_turn, radial_cross) == RealSign::Positive {
                    BezierAlgebraicCuspSemicirclePairEndpoint2::SecondStart
                } else {
                    BezierAlgebraicCuspSemicirclePairEndpoint2::SecondEnd
                };
            let first_inside_second =
                if product_sign(product_sign(RealSign::Negative, second_turn), radial_cross)
                    == RealSign::Positive
                {
                    BezierAlgebraicCuspSemicirclePairEndpoint2::FirstStart
                } else {
                    BezierAlgebraicCuspSemicirclePairEndpoint2::FirstEnd
                };
            let first_boundaries =
                if first_inside_second == BezierAlgebraicCuspSemicirclePairEndpoint2::FirstStart {
                    [first_inside_second, second_inside_first]
                } else {
                    [second_inside_first, first_inside_second]
                };
            let second_boundaries =
                if second_inside_first == BezierAlgebraicCuspSemicirclePairEndpoint2::SecondStart {
                    [second_inside_first, first_inside_second]
                } else {
                    [first_inside_second, second_inside_first]
                };
            (first_boundaries, second_boundaries)
        };
        let parameter_map = if radial_cross == RealSign::Zero {
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::ExactEndpoints {
                first_semicircle: self.clone(),
                second_semicircle: other.clone(),
            }
        } else {
            general_parameter_map.ok_or_else(|| {
                CurveError::Topology("a partial coincident-circle overlap lost its map".into())
            })?
        };
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(
                BezierAlgebraicCuspSemicirclePairOverlap2 {
                    data: Arc::new(BezierAlgebraicCuspSemicirclePairOverlapData2 {
                        parameter_map,
                        first_boundaries,
                        second_boundaries,
                        orientation,
                        policy: policy.retained_object_policy_with_dependencies(
                            self.data
                                .frame
                                .evidence_policy()
                                .into_iter()
                                .chain(other.data.frame.evidence_policy()),
                        ),
                    }),
                },
            ),
        ))
    }

    pub(in crate::bezier_offset) fn optional_similarity_components(
        similarity: Option<&Similarity2>,
    ) -> [Real; 6] {
        match similarity {
            Some(similarity) => {
                let (a, b, d, e, x, y) = similarity.affine_components();
                [
                    a.clone(),
                    b.clone(),
                    d.clone(),
                    e.clone(),
                    x.clone(),
                    y.clone(),
                ]
            }
            None => [
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::zero(),
                Real::zero(),
            ],
        }
    }

    pub(in crate::bezier_offset) fn retained_similarity_source<'a>(
        mut point: &'a CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<(&'a CurvePoint2, Option<Similarity2>)> {
        let mut transform: Option<Similarity2> = None;
        while let CurvePoint2(CurvePointData2::Similarity(similarity)) = point {
            if !policy.accepts_retained_policy(similarity.data.policy) {
                return Err(CurveError::Topology(
                    "a structural circle translation crossed predicate policies".into(),
                ));
            }
            transform = Some(match transform {
                Some(outer) => similarity.data.transform.then(&outer),
                None => similarity.data.transform.clone(),
            });
            point = &similarity.data.source;
        }
        Ok((point, transform))
    }

    /// Recovers a rational translation from two exact similarity views of one
    /// retained point. Equal linear parts cancel the selected source
    /// coordinates, so no algebraic-root affine relation has to rediscover
    /// construction provenance after materialization.
    pub(in crate::bezier_offset) fn retained_point_structural_translation(
        first: &CurvePoint2,
        second: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Option<[Real; 2]>> {
        let (first_source, first_transform) = Self::retained_similarity_source(first, policy)?;
        let (second_source, second_transform) = Self::retained_similarity_source(second, policy)?;
        if !first_source.shares_storage(second_source) && first_source != second_source {
            return Ok(None);
        }
        let first = Self::optional_similarity_components(first_transform.as_ref());
        let second = Self::optional_similarity_components(second_transform.as_ref());
        if (0..4).any(|index| {
            compare_reals(&first[index], &second[index], &CurveContext::STRICT)
                != Some(std::cmp::Ordering::Equal)
        }) {
            return Ok(None);
        }
        let translation = [&second[4] - &first[4], &second[5] - &first[5]];
        if translation
            .iter()
            .any(|coordinate| coordinate.exact_rational_ref().is_none())
        {
            return Ok(None);
        }
        Ok(Some(translation))
    }

    pub(in crate::bezier_offset) fn represented_structural_translation(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<[Real; 2]>> {
        if let (Some(first), Some(second)) = (
            self.data.frame.chord_normal(),
            other.data.frame.chord_normal(),
        ) && let Some(translation) =
            Self::retained_point_structural_translation(&first.center, &second.center, policy)?
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-circle-pair-translation",
                "retained-similarity-point",
            );
            return Ok(Some(translation));
        }
        if !self.uses_selected_radial_frame() || !other.uses_selected_radial_frame() {
            return Ok(None);
        }
        let first = match self.selected_radial_frame_source(policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(_) => return Ok(None),
        };
        let second = match other.selected_radial_frame_source(policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(_) => return Ok(None),
        };
        if !Arc::ptr_eq(&first.pair_map.data, &second.pair_map.data)
            || first.pair_contact != second.pair_contact
            || first.support_first != second.support_first
        {
            return Ok(None);
        }
        let first = Self::optional_similarity_components(first.similarity.as_ref());
        let second = Self::optional_similarity_components(second.similarity.as_ref());
        if (0..4).any(|index| {
            compare_reals(&first[index], &second[index], &CurveContext::STRICT)
                != Some(std::cmp::Ordering::Equal)
        }) {
            return Ok(None);
        }
        Ok(Some([&second[4] - &first[4], &second[5] - &first[5]]))
    }

    /// Reuses one authored center point when two recursive frames are exact
    /// similarity images of independently encoded copies of that point.
    ///
    /// The materialized Cartesian coordinates remain valid standalone
    /// algebraic numbers, but treating both transformed copies as four
    /// unrelated tensor axes needlessly forms every conjugate cross-product.
    /// After STRICT affine-root replay proves that the two untransformed
    /// centers agree, both world centers are instead retained as affine
    /// expressions of the same two selected coordinates.
    pub(in crate::bezier_offset) fn represented_common_source_center_tensor(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRepresentedCommonSourceCenter2>> {
        if !self.uses_selected_radial_frame() || !other.uses_selected_radial_frame() {
            return Ok(None);
        }
        let first = match self.selected_radial_frame_source(policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(_) => return Ok(None),
        };
        let second = match other.selected_radial_frame_source(policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(_) => return Ok(None),
        };
        let first_center = match first
            .pair_map
            .represented_selected_radial_contact_point(first.pair_contact)?
        {
            Classification::Decided(center) => center,
            Classification::Uncertain(_) => return Ok(None),
        }
        .map(|coordinate| {
            hypersolve::compact_algebraic_root_low_degree_witness(&coordinate).unwrap_or(coordinate)
        });
        let second_center = match second
            .pair_map
            .represented_selected_radial_contact_point(second.pair_contact)?
        {
            Classification::Decided(center) => center,
            Classification::Uncertain(_) => return Ok(None),
        }
        .map(|coordinate| {
            hypersolve::compact_algebraic_root_low_degree_witness(&coordinate).unwrap_or(coordinate)
        });
        for axis in 0..2 {
            let relation = if first_center[axis] == second_center[axis] {
                hypersolve::AlgebraicRootAffineRelation {
                    scale: Real::one(),
                    offset: Real::zero(),
                }
            } else if let Some(relation) =
                algebraic_root_affine_relation(&first_center[axis], &second_center[axis])
            {
                relation
            } else if represented_roots_strictly_equal(&first_center[axis], &second_center[axis]) {
                hypersolve::AlgebraicRootAffineRelation {
                    scale: Real::one(),
                    offset: Real::zero(),
                }
            } else {
                return Ok(None);
            };
            if compare_reals(&relation.scale, &Real::one(), &CurveContext::STRICT)
                != Some(std::cmp::Ordering::Equal)
                || compare_reals(&relation.offset, &Real::zero(), &CurveContext::STRICT)
                    != Some(std::cmp::Ordering::Equal)
            {
                return Ok(None);
            }
        }

        let first_transform = Self::optional_similarity_components(first.similarity.as_ref());
        let second_transform = Self::optional_similarity_components(second.similarity.as_ref());
        Ok(Some(BezierRepresentedCommonSourceCenter2 {
            sources: first_center.to_vec(),
            transforms: [first_transform, second_transform],
        }))
    }

    /// Recovers a rational translation between independently materialized
    /// frames. Each coordinate relation is accepted only after Hypersolve has
    /// replayed the affine image and selected-root equality under STRICT.
    pub(in crate::bezier_offset) fn represented_certified_frame_translation(
        first: &BezierRepresentedSelectedRadialCircleFrame2,
        second: &BezierRepresentedSelectedRadialCircleFrame2,
    ) -> Option<[Real; 2]> {
        let mut translation = [Real::zero(), Real::zero()];
        for (axis, translated) in translation.iter_mut().enumerate() {
            let relation = if first.center[axis] == second.center[axis] {
                hypersolve::AlgebraicRootAffineRelation {
                    scale: Real::one(),
                    offset: Real::zero(),
                }
            } else if let Some(relation) =
                algebraic_root_affine_relation(&first.center[axis], &second.center[axis])
            {
                relation
            } else if represented_roots_strictly_equal(&first.center[axis], &second.center[axis]) {
                hypersolve::AlgebraicRootAffineRelation {
                    scale: Real::one(),
                    offset: Real::zero(),
                }
            } else {
                return None;
            };
            if compare_reals(&relation.scale, &Real::one(), &CurveContext::STRICT)
                != Some(std::cmp::Ordering::Equal)
                || relation.offset.exact_rational_ref().is_none()
            {
                return None;
            }
            *translated = relation.offset;
        }
        Some(translation)
    }

    /// Recovers a circle-pair center distance already certified by recursive
    /// authorship. A selected-radial center is an exact retained contact on
    /// both circles in its source pair, so its squared distance to either
    /// source center is that support's squared radius. Reusing this fact keeps
    /// a tangent discriminant out of an unnecessary independent-coordinate
    /// norm while retaining the same represented pair kernel below.
    pub(in crate::bezier_offset) fn represented_authored_center_relation(
        candidate: &Self,
        other: &Self,
        other_frame: Option<&BezierRepresentedSelectedRadialCircleFrame2>,
        candidate_first: bool,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRepresentedAuthoredCenterRelation2>> {
        if !candidate.uses_selected_radial_frame() {
            return Ok(None);
        }
        let source = match candidate.selected_radial_frame_source(policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(_) => return Ok(None),
        };
        for (support_index, original_support) in [
            &source.pair_map.data.first_semicircle,
            &source.pair_map.data.second_semicircle,
        ]
        .into_iter()
        .enumerate()
        {
            let transformed_support = match source.similarity.as_ref() {
                Some(similarity) => original_support.transform_similarity(similarity)?,
                None => original_support.clone(),
            };
            let structurally_equal = transformed_support.data.frame == other.data.frame;
            let equivalent_frame = if structurally_equal {
                None
            } else if other_frame.is_some() {
                match transformed_support.represented_circle_frame(policy)? {
                    Classification::Decided(frame) => Some(frame),
                    Classification::Uncertain(_) => continue,
                }
            } else {
                continue;
            };
            let (same_center, same_unit_radial) =
                equivalent_frame
                    .as_ref()
                    .map_or((true, true), |support_frame| {
                        let other_frame = other_frame.expect("an equivalent frame has its target");
                        let equal =
                            |left: &[AlgebraicRootRepresentation; 2],
                             right: &[AlgebraicRootRepresentation; 2]| {
                                left.iter().zip(right).all(|(left, right)| {
                                    left == right
                                        || (!(compare_reals(
                                            &left.interval.upper,
                                            &right.interval.lower,
                                            &CurveContext::STRICT,
                                        ) == Some(std::cmp::Ordering::Less)
                                            || compare_reals(
                                                &right.interval.upper,
                                                &left.interval.lower,
                                                &CurveContext::STRICT,
                                            ) == Some(std::cmp::Ordering::Less))
                                            && represented_roots_strictly_equal(left, right))
                                })
                            };
                        let same_center = equal(&support_frame.center, &other_frame.center);
                        let same_unit_radial = same_center
                            && equal(&support_frame.unit_radial, &other_frame.unit_radial);
                        (same_center, same_unit_radial)
                    });
            if same_center {
                return Ok(Some(BezierRepresentedAuthoredCenterRelation2 {
                    distance_squared: transformed_support.radial_distance()
                        * transformed_support.radial_distance(),
                    pair_map: source.pair_map.clone(),
                    pair_contact: source.pair_contact.clone(),
                    candidate_first,
                    radial_support_first: source.support_first,
                    center_support_first: support_index == 0,
                    normal_denominator: source.frame.normal_denominator.clone(),
                    source_similarity: source.similarity.clone(),
                    direct_parameter_evidence: same_unit_radial,
                }));
            }
        }
        Ok(None)
    }

    pub(in crate::bezier_offset) fn represented_structural_center_relation(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRepresentedAuthoredCenterRelation2>> {
        if let Some(relation) =
            Self::represented_authored_center_relation(self, other, None, true, policy)?
        {
            return Ok(Some(relation));
        }
        Self::represented_authored_center_relation(other, self, None, false, policy)
    }

    pub(in crate::bezier_offset) fn represented_equivalent_center_relation(
        &self,
        other: &Self,
        first_frame: &BezierRepresentedSelectedRadialCircleFrame2,
        second_frame: &BezierRepresentedSelectedRadialCircleFrame2,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRepresentedAuthoredCenterRelation2>> {
        if let Some(relation) = Self::represented_authored_center_relation(
            self,
            other,
            Some(second_frame),
            true,
            policy,
        )? {
            return Ok(Some(relation));
        }
        Self::represented_authored_center_relation(other, self, Some(first_frame), false, policy)
    }

    /// Reuses angular evidence from the represented contact that authored one
    /// of the two centers. At a structural tangency the parent-side direction
    /// is the retained source contact itself, while the child-side angle is a
    /// scaled dot/cross image of the two retained source radials. Keeping those
    /// values in their original authority avoids eliminating the same repeated
    /// zero-radical parameter through a larger independent tensor.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn represented_authored_tangent_parameters(
        &self,
        other: &Self,
        relation: &BezierRepresentedAuthoredCenterRelation2,
        first_frame: &BezierRepresentedSelectedRadialCircleFrame2,
        second_frame: &BezierRepresentedSelectedRadialCircleFrame2,
        center_fraction: &Real,
    ) -> CurveResult<Option<BezierRepresentedAuthoredTangent2>> {
        if !relation.direct_parameter_evidence {
            return Ok(None);
        }
        let Some((_, source_data)) = relation
            .pair_map
            .represented_contact_data(&relation.pair_contact)
        else {
            return Ok(None);
        };
        let (candidate, candidate_frame, parent, parent_frame) = if relation.candidate_first {
            (self, first_frame, other, second_frame)
        } else {
            (other, second_frame, self, first_frame)
        };
        let source_parent = if relation.center_support_first {
            &relation.pair_map.data.first_semicircle
        } else {
            &relation.pair_map.data.second_semicircle
        };
        let parent_radial_scale = if relation.candidate_first {
            Real::one() - center_fraction
        } else {
            center_fraction.clone()
        };
        // A similarity preserves the local half-circle parameter, but a
        // reflection negates the signed parameter-zero radial in addition to
        // scaling it. Replay that same signed-radius transport before deciding
        // whether the authored parent parameter is direct or complementary.
        let source_parent_radial_distance = relation.source_similarity.as_ref().map_or_else(
            || source_parent.radial_distance().clone(),
            |similarity| {
                let radial_distance = source_parent.radial_distance() * similarity.scale();
                if similarity.reverses_orientation() {
                    -radial_distance
                } else {
                    radial_distance
                }
            },
        );
        let parent_direction =
            (&parent_radial_scale * source_parent_radial_distance) / &parent_frame.signed_radius;
        let parent_direction_sign = match real_sign(&parent_direction?, &CurveContext::STRICT) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "an authored tangent retained a zero parent radial scale".into(),
                ));
            }
            None => return Ok(None),
        };
        let source_parent_location = if relation.center_support_first {
            relation.pair_contact.first_location
        } else {
            relation.pair_contact.second_location
        };
        let parent_evidence =
            if source_parent_location == BezierAlgebraicCuspSemicircleContactLocation2::Interior {
                let source_parent_clockwise = source_parent.is_clockwise()
                    != relation
                        .source_similarity
                        .as_ref()
                        .is_some_and(Similarity2::reverses_orientation);
                let parent_turn_same = parent.is_clockwise() == source_parent_clockwise;
                let unit_complement = match (parent_direction_sign, parent_turn_same) {
                    (RealSign::Positive, true) => false,
                    (RealSign::Negative, false) => true,
                    (RealSign::Positive | RealSign::Negative, _) => {
                        return Ok(Some(BezierRepresentedAuthoredTangent2::OutsideRetainedHalf));
                    }
                    (RealSign::Zero, _) => unreachable!("the zero direction was rejected"),
                };
                let mut parameter = if relation.center_support_first {
                    relation
                        .pair_map
                        .first_contact_parameter(&relation.pair_contact)
                } else {
                    relation
                        .pair_map
                        .second_contact_parameter(&relation.pair_contact)
                };
                if let Some(similarity) = relation.source_similarity.as_ref() {
                    parameter = BezierAlgebraicCuspSemicircleSimilarityCache2::default()
                        .parameter(&parameter, source_parent, similarity)?;
                }
                BezierRepresentedCircleContactParameterEvidence2 {
                    location: source_parent_location,
                    parameter: BezierRepresentedCircleContactParameterData2::Retained {
                        parameter,
                        unit_complement,
                    },
                }
            } else {
                let location = if parent_direction_sign == RealSign::Positive {
                    source_parent_location
                } else if source_parent_location
                    == BezierAlgebraicCuspSemicircleContactLocation2::Start
                {
                    BezierAlgebraicCuspSemicircleContactLocation2::End
                } else {
                    BezierAlgebraicCuspSemicircleContactLocation2::Start
                };
                BezierRepresentedCircleContactParameterEvidence2 {
                    location,
                    parameter: BezierRepresentedCircleContactParameterData2::Materialized(
                        BezierParameter2::Exact(
                            if location == BezierAlgebraicCuspSemicircleContactLocation2::Start {
                                Real::zero()
                            } else {
                                Real::one()
                            },
                        ),
                    ),
                }
            };

        let candidate_radial_scale = if relation.candidate_first {
            -center_fraction.clone()
        } else {
            center_fraction - Real::one()
        };
        let candidate_scale = (&candidate_frame.signed_radius * candidate_radial_scale)
            / &relation.normal_denominator;
        let candidate_scale = candidate_scale?;
        let radius_squared = candidate.radial_distance() * candidate.radial_distance();
        let candidate_evidence = if relation.radial_support_first == relation.center_support_first {
            let location = match real_sign(
                &(&candidate_scale * &relation.distance_squared),
                &CurveContext::STRICT,
            ) {
                Some(RealSign::Positive) => BezierAlgebraicCuspSemicircleContactLocation2::Start,
                Some(RealSign::Negative) => BezierAlgebraicCuspSemicircleContactLocation2::End,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "an authored nonzero tangent had zero diameter coordinates".into(),
                    ));
                }
                None => return Ok(None),
            };
            BezierRepresentedCircleContactParameterEvidence2 {
                location,
                parameter: BezierRepresentedCircleContactParameterData2::Materialized(
                    BezierParameter2::Exact(
                        if location == BezierAlgebraicCuspSemicircleContactLocation2::Start {
                            Real::zero()
                        } else {
                            Real::one()
                        },
                    ),
                ),
            }
        } else if let Classification::Decided(Some(parameter)) = candidate
            .certified_selected_pair_contact_parameter(parent, &relation.pair_map.data.policy)?
        {
            // The tangent lies on the companion support that authored this
            // selected circle's other endpoint. Retain that endpoint's
            // existing pair authority directly: a concentric offset changes
            // its point but not its local half-circle parameter.
            BezierRepresentedCircleContactParameterEvidence2 {
                location: BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                parameter: BezierRepresentedCircleContactParameterData2::Retained {
                    parameter,
                    unit_complement: false,
                },
            }
        } else {
            let turn_product = relation.pair_map.data.first_semicircle.turn_sign()
                * relation.pair_map.data.second_semicircle.turn_sign();
            let two_turn_product = Real::from(2_i8) * turn_product;
            let source_metric_scale = relation
                .source_similarity
                .as_ref()
                .map_or_else(Real::one, |similarity| {
                    similarity.scale() * similarity.scale()
                });
            let dot_scale = (&candidate_scale * &source_metric_scale / &two_turn_product)?;
            let radial_cross_orientation = if relation.radial_support_first {
                1_i8
            } else {
                -1_i8
            };
            let oriented_cross_scale = (&candidate_scale
                * candidate.turn_sign()
                * Real::from(radial_cross_orientation)
                * &source_metric_scale
                / two_turn_product)?;
            let dot = match Classification::from(represented_affine_coordinate(
                &[(&source_data.tangent_dot, &dot_scale)],
                &Real::zero(),
            )) {
                Classification::Decided(dot) => dot,
                Classification::Uncertain(_) => return Ok(None),
            };
            let oriented_cross = match Classification::from(represented_affine_coordinate(
                &[(&source_data.tangent_cross, &oriented_cross_scale)],
                &Real::zero(),
            )) {
                Classification::Decided(cross) => cross,
                Classification::Uncertain(_) => return Ok(None),
            };
            let location = match represented_strict_sign(&oriented_cross) {
                Some(RealSign::Positive) => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                Some(RealSign::Negative) => {
                    return Ok(Some(BezierRepresentedAuthoredTangent2::OutsideRetainedHalf));
                }
                Some(RealSign::Zero) => match represented_strict_sign(&dot) {
                    Some(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Some(RealSign::Negative) => BezierAlgebraicCuspSemicircleContactLocation2::End,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "an authored nonzero tangent had zero diameter coordinates".into(),
                        ));
                    }
                    None => return Ok(None),
                },
                None => return Ok(None),
            };
            BezierRepresentedCircleContactParameterEvidence2 {
                location,
                parameter: BezierRepresentedCircleContactParameterData2::AuthoredPairAngular {
                    map: relation.pair_map.clone(),
                    contact: relation.pair_contact.clone(),
                    dot_scale,
                    oriented_cross_scale,
                    radius_squared,
                },
            }
        };
        let parent_parameter = if relation.center_support_first {
            relation
                .pair_map
                .first_contact_parameter(&relation.pair_contact)
        } else {
            relation
                .pair_map
                .second_contact_parameter(&relation.pair_contact)
        };
        let point = match parent_parameter {
            BezierAlgebraicCuspSemicircleParameter2::Exact(_) => None,
            BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter) => {
                let point =
                    CurvePoint2::from(BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                        parameter,
                        None,
                        parent_radial_scale,
                    ));
                Some(match relation.source_similarity.as_ref() {
                    Some(similarity) => CurvePoint2::from(BezierSimilarityPoint2::new(
                        point,
                        similarity.clone(),
                        &relation.pair_map.data.policy,
                    )),
                    None => point,
                })
            }
        };
        Ok(Some(BezierRepresentedAuthoredTangent2::Contact {
            parameters: if relation.candidate_first {
                [candidate_evidence, parent_evidence]
            } else {
                [parent_evidence, candidate_evidence]
            },
            point,
        }))
    }

    pub(in crate::bezier_offset) fn represented_coincident_pair_intersections(
        &self,
        other: &Self,
        first_frame: &BezierRepresentedSelectedRadialCircleFrame2,
        second_frame: &BezierRepresentedSelectedRadialCircleFrame2,
        first_radius_squared: &Real,
        second_radius_squared: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicirclePairIntersections2>> {
        match real_sign(
            &(first_radius_squared - second_radius_squared),
            &CurveContext::STRICT,
        ) {
            Some(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
            Some(RealSign::Zero) => {}
        }
        let radial_scale_product = &first_frame.signed_radius * &second_frame.signed_radius;
        let [radial_dot, radial_cross] = match represented_scaled_unit_radial_dot_cross(
            &first_frame.unit_radial,
            &second_frame.unit_radial,
            &radial_scale_product,
        ) {
            Classification::Decided(values) => values,
            Classification::Uncertain(reason) => {
                if let Some(radial_dot) = self.coincident_endpoint_radial_dot(other, policy)? {
                    return self.coincident_pair_intersections_from_radial_signs(
                        other,
                        RealSign::Zero,
                        Some(radial_dot),
                        None,
                        policy,
                    );
                }
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radial_cross_sign =
            match Classification::from(represented_policy_sign(&radial_cross, policy)) {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    if let Some(radial_dot) = self.coincident_endpoint_radial_dot(other, policy)? {
                        return self.coincident_pair_intersections_from_radial_signs(
                            other,
                            RealSign::Zero,
                            Some(radial_dot),
                            None,
                            policy,
                        );
                    }
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let radial_dot_sign = if radial_cross_sign == RealSign::Zero {
            Some(
                match Classification::from(represented_policy_sign(&radial_dot, policy)) {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        if let Some(radial_dot) =
                            self.coincident_endpoint_radial_dot(other, policy)?
                        {
                            return self.coincident_pair_intersections_from_radial_signs(
                                other,
                                RealSign::Zero,
                                Some(radial_dot),
                                None,
                                policy,
                            );
                        }
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            )
        } else {
            None
        };
        let parameter_map = (radial_cross_sign != RealSign::Zero).then(|| {
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::Represented {
                first_semicircle: self.clone(),
                second_semicircle: other.clone(),
                radial_dot,
                radial_cross,
                first_radius_squared: first_radius_squared.clone(),
                second_radius_squared: second_radius_squared.clone(),
                first_clockwise: self.is_clockwise(),
            }
        });
        self.coincident_pair_intersections_from_radial_signs(
            other,
            radial_cross_sign,
            radial_dot_sign,
            parameter_map,
            policy,
        )
    }

    /// Recovers the aligned or antipodal radial relation from exact endpoint
    /// identity after the coordinate product loses its shared circle sheet.
    /// Equal centers and radii are already certified by the caller, so one
    /// coincident diameter endpoint uniquely fixes the physical start radial.
    pub(in crate::bezier_offset) fn coincident_endpoint_radial_dot(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        let first_start = match self.start_point_evidence(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(_) => return Ok(None),
        };
        let second_start = match other.start_point_evidence(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(_) => return Ok(None),
        };
        let second_end = match other.end_point_evidence(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(_) => return Ok(None),
        };
        let same = |first: &CurvePoint2, second: &CurvePoint2| {
            first.same_point(second, policy) == Classification::Decided(true)
                || second.same_point(first, policy) == Classification::Decided(true)
        };
        Ok(if same(&first_start, &second_start) {
            Some(RealSign::Positive)
        } else if same(&first_start, &second_end) {
            Some(RealSign::Negative)
        } else {
            None
        })
    }

    /// Exact translated-frame circle-pair lane. Structural provenance is the
    /// allocation-free fast path; independently encoded frames enter only
    /// after exact affine-image replay proves both rational center deltas. The
    /// support discriminant remains a canonical `Real`, and each accepted
    /// contact keeps its exact radial for lazy angular predicates.
    pub(in crate::bezier_offset) fn represented_translation_pair_intersections(
        &self,
        other: &Self,
        has_authored_center_relation: bool,
        structural_translation: Option<&[Real; 2]>,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<BezierAlgebraicCuspSemicirclePairIntersections2>>> {
        let mut materialized_frames = None;
        let [dx, dy] = if let Some(translation) = structural_translation {
            translation.clone()
        } else {
            // A selected-radial center already retains its exact squared
            // distance to either parent support. Reconstructing both world
            // centers merely to search for an independently materialized
            // rational translation discards that correlation and can form a
            // large Cartesian field after similarity transport. Let the
            // general pair lane consume the authored distance directly.
            if has_authored_center_relation {
                return Ok(None);
            }
            let first_frame = match self.represented_circle_frame(policy)? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let second_frame = match other.represented_circle_frame(policy)? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let Some(translation) =
                Self::represented_certified_frame_translation(&first_frame, &second_frame)
            else {
                return Ok(None);
            };
            materialized_frames = Some((first_frame, second_frame));
            translation
        };
        let (first_frame, second_frame) = if let Some(frames) = materialized_frames {
            frames
        } else {
            let first_frame = match self.represented_circle_frame(policy)? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let second_frame = match other.represented_circle_frame(policy)? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            (first_frame, second_frame)
        };
        let first_radius_squared = self.radial_distance() * self.radial_distance();
        let second_radius_squared = other.radial_distance() * other.radial_distance();
        let q = &dx * &dx + &dy * &dy;
        match real_sign(&q, &CurveContext::STRICT) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Some(self.represented_coincident_pair_intersections(
                    other,
                    &first_frame,
                    &second_frame,
                    &first_radius_squared,
                    &second_radius_squared,
                    policy,
                )?));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a translated circle-pair center distance squared was negative".into(),
                ));
            }
            None => {
                return Ok(Some(Classification::Uncertain(
                    UncertaintyReason::Predicate,
                )));
            }
        }
        let line = &q + &first_radius_squared - &second_radius_squared;
        let discriminant = Real::from(4_i8) * &q * &first_radius_squared - &line * &line;
        let branches: &[i8] = match real_sign(&discriminant, &CurveContext::STRICT) {
            Some(RealSign::Negative) => {
                return Ok(Some(Classification::Decided(
                    BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
                )));
            }
            Some(RealSign::Zero) => &[0],
            Some(RealSign::Positive) => &[-1, 1],
            None => {
                return Ok(Some(Classification::Uncertain(
                    UncertaintyReason::Predicate,
                )));
            }
        };
        let radical = if branches.len() == 1 {
            Real::zero()
        } else {
            discriminant.sqrt()?
        };
        let denominator = Real::from(2_i8) * &q;
        let turn_product = self.turn_sign() * other.turn_sign();
        let radial_dot = &first_radius_squared + &second_radius_squared - &q;
        let mut contacts = Vec::with_capacity(branches.len());
        let mut represented_contacts = Vec::with_capacity(branches.len());
        for &branch in branches {
            let signed_radical = Real::from(branch) * &radical;
            let first_contact_radial = [
                ((&line * &dx - &signed_radical * &dy) / &denominator)?,
                ((&line * &dy + &signed_radical * &dx) / &denominator)?,
            ];
            let second_contact_radial = [
                &first_contact_radial[0] - &dx,
                &first_contact_radial[1] - &dy,
            ];
            let point = [
                match Classification::from(represented_affine_coordinate(
                    &[(&first_frame.center[0], &Real::one())],
                    &first_contact_radial[0],
                )) {
                    Classification::Decided(value) => value,
                    Classification::Uncertain(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                },
                match Classification::from(represented_affine_coordinate(
                    &[(&first_frame.center[1], &Real::one())],
                    &first_contact_radial[1],
                )) {
                    Classification::Decided(value) => value,
                    Classification::Uncertain(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                },
            ];
            let first_location = match represented_circle_contact_location_from_exact_radial(
                &first_frame,
                &first_contact_radial,
                &first_radius_squared,
                &self.turn_sign(),
            )? {
                Classification::Decided(Some(value)) => value,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let second_location = match represented_circle_contact_location_from_exact_radial(
                &second_frame,
                &second_contact_radial,
                &second_radius_squared,
                &other.turn_sign(),
            )? {
                Classification::Decided(Some(value)) => value,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let tangent_cross =
                AlgebraicRootRepresentation::from_exact_value(&(&turn_product * &signed_radical));
            let tangent_dot =
                AlgebraicRootRepresentation::from_exact_value(&(&turn_product * &radial_dot));
            let Some(tangent_cross_sign) = represented_strict_sign(&tangent_cross) else {
                return Ok(Some(Classification::Uncertain(
                    UncertaintyReason::Predicate,
                )));
            };
            contacts.push(BezierAlgebraicCuspSemicirclePairContact2 {
                branch,
                first_location,
                second_location,
                tangent_cross_sign,
            });
            represented_contacts.push(BezierRepresentedCirclePairContactData2 {
                branch,
                point,
                first_parameter: BezierRepresentedCircleContactParameterData2::ExactContactRadial(
                    first_contact_radial,
                ),
                second_parameter: BezierRepresentedCircleContactParameterData2::ExactContactRadial(
                    second_contact_radial,
                ),
                tangent_cross,
                tangent_dot,
                recursive_contact_frame: OnceLock::new(),
            });
        }
        if contacts.is_empty() {
            return Ok(Some(Classification::Decided(
                BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
            )));
        }
        let parameter_map = BezierAlgebraicCuspSemicirclePairParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicirclePairParameterMapData2 {
                first_semicircle: self.clone(),
                second_semicircle: other.clone(),
                system: BezierCirclePairParameterMapSystem2::Represented(
                    BezierRepresentedCirclePairParameterMapSystem2 {
                        first_center: first_frame.center,
                        second_center: second_frame.center,
                        contacts: represented_contacts,
                    },
                ),
                recursive_field: OnceLock::new(),
                policy: policy.retained_object_policy_with_dependencies(
                    self.data
                        .frame
                        .evidence_policy()
                        .into_iter()
                        .chain(other.data.frame.evidence_policy()),
                ),
            }),
        };
        Ok(Some(Classification::Decided(
            BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        )))
    }

    /// Rank-independent exact circle-pair fallback. Compact fixed-field
    /// systems remain the first choice; this path materializes only the two
    /// circle frames and the retained outputs needed by later topology.
    pub(in crate::bezier_offset) fn represented_pair_intersections(
        &self,
        other: &Self,
        structural_center_relation: Option<&BezierRepresentedAuthoredCenterRelation2>,
        structural_translation: Option<&[Real; 2]>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicirclePairIntersections2>> {
        if let Some(result) = self.represented_translation_pair_intersections(
            other,
            structural_center_relation.is_some(),
            structural_translation,
            policy,
        )? {
            return Ok(result);
        }
        let common_source = self.represented_common_source_center_tensor(other, policy)?;
        let first_frame = match self.represented_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second_frame = match other.represented_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let first_radius_squared = self.radial_distance() * self.radial_distance();
        let second_radius_squared = other.radial_distance() * other.radial_distance();
        let (sources, center_coordinates, unit_radial_coordinates) = if let Some(common_source) =
            common_source
        {
            let mut sources = common_source.sources;
            sources.extend(first_frame.unit_radial.iter().cloned());
            sources.extend(second_frame.unit_radial.iter().cloned());
            let rank = sources.len() + 1;
            let axis = |axis| {
                DenseTensorPolynomial::from_axis_polynomial(
                    rank,
                    axis,
                    &[Real::zero(), Real::one()],
                )
            };
            let constant = |value: &Real| {
                DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(value))
            };
            let base_x = axis(0).expect("a common-source tensor has its base x axis");
            let base_y = axis(1).expect("a common-source tensor has its base y axis");
            let affine = |components: &[Real; 6], x_axis: bool| {
                let (x_scale, y_scale, translation) = if x_axis {
                    (&components[0], &components[1], &components[4])
                } else {
                    (&components[2], &components[3], &components[5])
                };
                base_x
                    .scale(x_scale)?
                    .add(&base_y.scale(y_scale)?)?
                    .add(&constant(translation)?)
            };
            let Some(center_coordinates) = (|| {
                Some([
                    affine(&common_source.transforms[0], true)?,
                    affine(&common_source.transforms[0], false)?,
                    affine(&common_source.transforms[1], true)?,
                    affine(&common_source.transforms[1], false)?,
                ])
            })() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let unit_radial_coordinates = [
                axis(2).expect("a common-source tensor has its first radial x axis"),
                axis(3).expect("a common-source tensor has its first radial y axis"),
                axis(4).expect("a common-source tensor has its second radial x axis"),
                axis(5).expect("a common-source tensor has its second radial y axis"),
            ];
            (sources, center_coordinates, unit_radial_coordinates)
        } else {
            // Keep both local frames and the circle-pair construction in one
            // tensor authority. Materializing a contact first and then
            // subtracting independently eliminated center coordinates loses
            // the useful expression correlation and can multiply the same
            // selected fields a second time merely to recover angular
            // parameters.
            let coordinates = [
                first_frame.center[0].clone(),
                first_frame.center[1].clone(),
                second_frame.center[0].clone(),
                second_frame.center[1].clone(),
                first_frame.unit_radial[0].clone(),
                first_frame.unit_radial[1].clone(),
                second_frame.unit_radial[0].clone(),
                second_frame.unit_radial[1].clone(),
            ];
            let Some((sources, coordinates)) = represented_affine_tensor_basis(&coordinates) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let coordinates: [DenseTensorPolynomial; 8] = coordinates
                .try_into()
                .expect("the represented frame basis retains all eight coordinates");
            let [
                first_center_x,
                first_center_y,
                second_center_x,
                second_center_y,
                first_radial_x,
                first_radial_y,
                second_radial_x,
                second_radial_y,
            ] = coordinates;
            (
                sources,
                [
                    first_center_x,
                    first_center_y,
                    second_center_x,
                    second_center_y,
                ],
                [
                    first_radial_x,
                    first_radial_y,
                    second_radial_x,
                    second_radial_y,
                ],
            )
        };
        let equivalent_center_relation = if structural_center_relation.is_none()
            && sources.len() >= 3
        {
            self.represented_equivalent_center_relation(other, &first_frame, &second_frame, policy)?
        } else {
            None
        };
        let structural_center_relation =
            structural_center_relation.or(equivalent_center_relation.as_ref());
        let rank = sources.len() + 1;
        let constant = |value: &Real| {
            DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(value))
        };
        let Some((
            first_center_x,
            first_center_y,
            dx,
            dy,
            q,
            line,
            discriminant,
            radial_dot,
            twice_q,
        )) = (|| {
            let [first_x, first_y, second_x, second_y] = center_coordinates;
            let dx = second_x.subtract(&first_x)?;
            let dy = second_y.subtract(&first_y)?;
            let q = match structural_center_relation {
                Some(relation) => constant(&relation.distance_squared)?,
                None => dx.multiply(&dx)?.add(&dy.multiply(&dy)?)?,
            };
            let line = q.add(&constant(
                &(&first_radius_squared - &second_radius_squared),
            )?)?;
            let discriminant = q
                .scale(&(Real::from(4_i8) * &first_radius_squared))?
                .subtract(&line.multiply(&line)?)?;
            let radial_dot =
                constant(&(&first_radius_squared + &second_radius_squared))?.subtract(&q)?;
            let twice_q = q.scale(&Real::from(2_i8))?;
            Some((
                first_x,
                first_y,
                dx,
                dy,
                q,
                line,
                discriminant,
                radial_dot,
                twice_q,
            ))
        })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let q_value = match Classification::from(represented_dense_value_refined(&q, &sources)) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let q_sign = represented_strict_sign(&q_value);
        match q_sign {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return self.represented_coincident_pair_intersections(
                    other,
                    &first_frame,
                    &second_frame,
                    &first_radius_squared,
                    &second_radius_squared,
                    policy,
                );
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a represented circle-pair center distance squared was negative".into(),
                ));
            }
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        }
        let discriminant_value =
            match Classification::from(represented_dense_value_refined(&discriminant, &sources)) {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let discriminant_sign = represented_strict_sign(&discriminant_value);
        let branches: &[i8] = match discriminant_sign {
            Some(RealSign::Negative) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
                ));
            }
            Some(RealSign::Zero) => &[0],
            Some(RealSign::Positive) => &[-1, 1],
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        };
        let radial_dot_value =
            match Classification::from(represented_dense_value_refined(&radial_dot, &sources)) {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let turn_product = self.turn_sign() * other.turn_sign();
        let zero = constant(&Real::zero())
            .expect("a represented circle-pair tensor has its zero polynomial");
        let authored_tangent = if branches == [0] {
            if let Some(relation) = structural_center_relation {
                let center_line =
                    &relation.distance_squared + &first_radius_squared - &second_radius_squared;
                let center_fraction =
                    (center_line / (Real::from(2_i8) * &relation.distance_squared))?;
                self.represented_authored_tangent_parameters(
                    other,
                    relation,
                    &first_frame,
                    &second_frame,
                    &center_fraction,
                )?
            } else {
                None
            }
        } else {
            None
        };
        let (authored_tangent_parameters, authored_tangent_point) = match authored_tangent {
            Some(BezierRepresentedAuthoredTangent2::Contact { parameters, point }) => {
                (Some(parameters), point)
            }
            Some(BezierRepresentedAuthoredTangent2::OutsideRetainedHalf) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
                ));
            }
            None => (None, None),
        };
        let mut contacts = Vec::with_capacity(branches.len());
        let mut represented_contacts = Vec::with_capacity(branches.len());
        for &branch in branches {
            let radical = square_root_algebraic_root_representation(&discriminant_value, branch);
            let signed_radical = match radical.status {
                AlgebraicRootSquareRootStatus::Transformed => radical
                    .representation
                    .expect("a transformed square root retains its representation"),
                AlgebraicRootSquareRootStatus::UndecidedSign => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                AlgebraicRootSquareRootStatus::InvalidEvidence
                | AlgebraicRootSquareRootStatus::InvalidBranch
                | AlgebraicRootSquareRootStatus::NegativeRadicand
                | AlgebraicRootSquareRootStatus::NonzeroZeroBranch
                | AlgebraicRootSquareRootStatus::InvalidTransformedEvidence => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            };
            let radial_system = (|| {
                let radial_retained_x = line.multiply(&dx)?;
                let radial_retained_y = line.multiply(&dy)?;
                let radial_candidate_x = dy.scale(&Real::from(-1_i8))?;
                let radial_candidate_y = dx.clone();
                let point_retained_x =
                    twice_q.multiply(&first_center_x)?.add(&radial_retained_x)?;
                let point_retained_y =
                    twice_q.multiply(&first_center_y)?.add(&radial_retained_y)?;
                Some((
                    [radial_retained_x, radial_retained_y],
                    [radial_candidate_x, radial_candidate_y],
                    [point_retained_x, point_retained_y],
                ))
            })();
            let Some((first_radial_retained, first_radial_candidate, point_retained)) =
                radial_system
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let point = if branch == 0
                && let Some(point) = authored_tangent_point.as_ref()
            {
                match represented_point_evidence_coordinates(point, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                let x = represented_tensor_nested_ratio(
                    &point_retained[0],
                    &first_radial_candidate[0],
                    &twice_q,
                    &zero,
                    &discriminant,
                    &sources,
                    &signed_radical,
                );
                let y = represented_tensor_nested_ratio(
                    &point_retained[1],
                    &first_radial_candidate[1],
                    &twice_q,
                    &zero,
                    &discriminant,
                    &sources,
                    &signed_radical,
                );
                match (x, y) {
                    (Classification::Decided(x), Classification::Decided(y)) => [x, y],
                    (Classification::Uncertain(UncertaintyReason::Unsupported), _)
                    | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    _ => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                }
            }
            .map(|coordinate| {
                hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                    .unwrap_or(coordinate)
            });
            let [first, second] = if let Some(parameters) =
                authored_tangent_parameters.as_ref().filter(|_| branch == 0)
            {
                [
                    BezierRepresentedCircleContactParameterEvidence2 {
                        location: parameters[0].location,
                        parameter: parameters[0].parameter.clone(),
                    },
                    BezierRepresentedCircleContactParameterEvidence2 {
                        location: parameters[1].location,
                        parameter: parameters[1].parameter.clone(),
                    },
                ]
            } else {
                let Some(second_radial_retained) = (|| {
                    Some([
                        first_radial_retained[0].subtract(&twice_q.multiply(&dx)?)?,
                        first_radial_retained[1].subtract(&twice_q.multiply(&dy)?)?,
                    ])
                })() else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let first_unit = [
                    unit_radial_coordinates[0].clone(),
                    unit_radial_coordinates[1].clone(),
                ];
                let second_unit = [
                    unit_radial_coordinates[2].clone(),
                    unit_radial_coordinates[3].clone(),
                ];
                let first = match represented_tensor_circle_contact_location_parameter(
                    &first_unit,
                    &first_radial_retained,
                    &first_radial_candidate,
                    &twice_q,
                    &discriminant,
                    &sources,
                    &signed_radical,
                    &first_frame.signed_radius,
                    &self.turn_sign(),
                    &first_radius_squared,
                )? {
                    Classification::Decided(Some(value)) => value,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let second = match represented_tensor_circle_contact_location_parameter(
                    &second_unit,
                    &second_radial_retained,
                    &first_radial_candidate,
                    &twice_q,
                    &discriminant,
                    &sources,
                    &signed_radical,
                    &second_frame.signed_radius,
                    &other.turn_sign(),
                    &second_radius_squared,
                )? {
                    Classification::Decided(Some(value)) => value,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                [
                    BezierRepresentedCircleContactParameterEvidence2 {
                        location: first.0,
                        parameter: BezierRepresentedCircleContactParameterData2::Materialized(
                            first.1,
                        ),
                    },
                    BezierRepresentedCircleContactParameterEvidence2 {
                        location: second.0,
                        parameter: BezierRepresentedCircleContactParameterData2::Materialized(
                            second.1,
                        ),
                    },
                ]
            };
            let authored_relation = if branch == 0 && authored_tangent_parameters.is_some() {
                structural_center_relation
            } else {
                None
            };
            let (tangent_cross, tangent_dot) = if let Some(relation) = authored_relation {
                (
                    AlgebraicRootRepresentation::from_exact_value(&Real::zero()),
                    AlgebraicRootRepresentation::from_exact_value(
                        &(&turn_product
                            * (&first_radius_squared + &second_radius_squared
                                - &relation.distance_squared)),
                    ),
                )
            } else {
                let tangent_cross = match Classification::from(represented_affine_coordinate(
                    &[(&signed_radical, &turn_product)],
                    &Real::zero(),
                )) {
                    Classification::Decided(value) => value,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let tangent_dot = match Classification::from(represented_affine_coordinate(
                    &[(&radial_dot_value, &turn_product)],
                    &Real::zero(),
                )) {
                    Classification::Decided(value) => value,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (tangent_cross, tangent_dot)
            };
            let Some(tangent_cross_sign) = represented_strict_sign(&tangent_cross) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            };
            contacts.push(BezierAlgebraicCuspSemicirclePairContact2 {
                branch,
                first_location: first.location,
                second_location: second.location,
                tangent_cross_sign,
            });
            represented_contacts.push(BezierRepresentedCirclePairContactData2 {
                branch,
                point,
                first_parameter: first.parameter,
                second_parameter: second.parameter,
                tangent_cross,
                tangent_dot,
                recursive_contact_frame: OnceLock::new(),
            });
        }
        if contacts.is_empty() {
            return Ok(Classification::Decided(
                BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
            ));
        }
        let parameter_map = BezierAlgebraicCuspSemicirclePairParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicirclePairParameterMapData2 {
                first_semicircle: self.clone(),
                second_semicircle: other.clone(),
                system: BezierCirclePairParameterMapSystem2::Represented(
                    BezierRepresentedCirclePairParameterMapSystem2 {
                        first_center: first_frame.center,
                        second_center: second_frame.center,
                        contacts: represented_contacts,
                    },
                ),
                recursive_field: OnceLock::new(),
                policy: policy.retained_object_policy_with_dependencies(
                    self.data
                        .frame
                        .evidence_policy()
                        .into_iter()
                        .chain(other.data.frame.evidence_policy()),
                ),
            }),
        };
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        ))
    }
}

impl BezierAlgebraicCuspSemicircle2 {
    pub(in crate::bezier_offset) fn recursive_pair_frame_authorities(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<[BezierRecursiveCircleFrame2; 2]>>> {
        let first = match self.recursive_circle_frame_authority(policy)? {
            Classification::Decided(Some(authority)) => authority,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match other.recursive_circle_frame_authority(policy)? {
            Classification::Decided(Some(authority)) => authority,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if self.data.frame.rational().is_some()
            && let Classification::Decided(Some(first)) =
                self.recursive_rational_circle_frame_authority_in_field(&second.field, policy)?
        {
            return Ok(Classification::Decided(Some([first, second])));
        }
        if other.data.frame.rational().is_some()
            && let Classification::Decided(Some(second)) =
                other.recursive_rational_circle_frame_authority_in_field(&first.field, policy)?
        {
            return Ok(Classification::Decided(Some([first, second])));
        }
        if let Some(second) = second.lifted_to(&first.field) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-pair-frame-join",
                "second-to-first-ancestor",
            );
            return Ok(Classification::Decided(Some([first, second])));
        }
        if let Some(first) = first.lifted_to(&second.field) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-pair-frame-join",
                "first-to-second-ancestor",
            );
            return Ok(Classification::Decided(Some([first, second])));
        }
        let joined = match first.field.joined_with(&second.field, policy)? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some((field, embeddings)) = joined {
            let Some((first, second)) = (|| {
                Some((
                    first.lifted_to(&field)?,
                    BezierRecursiveCircleFrame2 {
                        center: second.center.embedded_to(&field, &embeddings)?,
                        support_center: second.support_center.embedded_to(&field, &embeddings)?,
                        field: field.clone(),
                        normal_denominator: second.normal_denominator,
                    },
                ))
            })() else {
                return Ok(Classification::Decided(None));
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-pair-frame-join",
                "retained-field-join",
            );
            return Ok(Classification::Decided(Some([first, second])));
        }
        let represented = [first.center.clone(), first.support_center.clone()];
        let (field, mut represented, second_center) = match recursive_merge_projective_point_fields(
            &first.field,
            &represented,
            &second.center,
            policy,
        )? {
            Classification::Decided(Some(joined)) => joined,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        represented.push(second_center);
        let (field, represented, second_support) =
            if let Some(second_support) = second.support_center.lifted_to(&field) {
                (field, represented, second_support)
            } else {
                match recursive_merge_projective_point_fields(
                    &field,
                    &represented,
                    &second.support_center,
                    policy,
                )? {
                    Classification::Decided(Some(joined)) => joined,
                    Classification::Decided(None) => return Ok(Classification::Decided(None)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
        let [first_center, first_support, second_center]: [BezierRecursiveQuadraticProjectivePoint2;
            3] = represented
            .try_into()
            .expect("a joined recursive circle pair retains three imported points");
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-circle-pair-frame-join",
            "projective-field-merge",
        );
        Ok(Classification::Decided(Some([
            BezierRecursiveCircleFrame2 {
                field: field.clone(),
                center: first_center,
                support_center: first_support,
                normal_denominator: first.normal_denominator,
            },
            BezierRecursiveCircleFrame2 {
                field,
                center: second_center,
                support_center: second_support,
                normal_denominator: second.normal_denominator,
            },
        ])))
    }

    pub(in crate::bezier_offset) fn recursive_pair_contact_side(
        &self,
        center: &BezierRecursiveQuadraticProjectivePoint2,
        support_center: &BezierRecursiveQuadraticProjectivePoint2,
        normal_denominator: &Real,
        radial: &[BezierRecursiveQuadraticValue2; 2],
        radial_denominator: &BezierRecursiveQuadraticValue2,
        certified_location: Option<BezierAlgebraicCuspSemicircleContactLocation2>,
    ) -> CurveResult<Classification<Option<BezierRecursiveCirclePairContactSide2>>> {
        let Some((anchor_x, anchor_y, anchor_denominator)) =
            center.difference_numerators(support_center)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [radial_x, radial_y] = radial;
        let Some((selected_half_plane, diameter, radius_squared_denominator)) = (|| {
            let radial_scale = self.radial_distance() * normal_denominator;
            let selected_half_plane = anchor_x
                .multiply(radial_y)?
                .subtract(&anchor_y.multiply(radial_x)?)?
                .scale(&(self.turn_sign() * &radial_scale))?;
            let diameter = anchor_x
                .multiply(radial_x)?
                .add(&anchor_y.multiply(radial_y)?)?
                .scale(&radial_scale)?;
            let radius_squared_denominator =
                anchor_denominator.multiply(radial_denominator)?.scale(
                    &(self.radial_distance()
                        * self.radial_distance()
                        * normal_denominator
                        * normal_denominator),
                )?;
            Some((selected_half_plane, diameter, radius_squared_denominator))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let location = if let Some(location) = certified_location {
            location
        } else {
            match selected_half_plane.sign(&CurveContext::STRICT)? {
                Classification::Decided(RealSign::Negative) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Decided(RealSign::Positive) => {
                    BezierAlgebraicCuspSemicircleContactLocation2::Interior
                }
                Classification::Decided(RealSign::Zero) => {
                    match diameter.sign(&CurveContext::STRICT)? {
                        Classification::Decided(RealSign::Positive) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::Start
                        }
                        Classification::Decided(RealSign::Negative) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::End
                        }
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "a nonzero recursive circle-pair contact had zero local diameter"
                                    .into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        Ok(Classification::Decided(Some(
            BezierRecursiveCirclePairContactSide2 {
                location,
                angular: BezierRecursiveCirclePairAngularData2 {
                    diameter,
                    radius_squared_denominator,
                },
            },
        )))
    }

    /// Finds a retained diameter endpoint shared by two tangent selected
    /// semicircles.  Once the full-circle discriminant is exactly zero and
    /// the centers are distinct, such a point is necessarily the unique
    /// support contact.  This certificate avoids asking a deep recursive
    /// field to rediscover that its selected-half cross product is zero.
    pub(in crate::bezier_offset) fn recursive_pair_shared_tangent_endpoint_locations(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<[BezierAlgebraicCuspSemicircleContactLocation2; 2]>>>
    {
        let endpoints = |circle: &Self| -> CurveResult<Option<[CurvePoint2; 2]>> {
            let start = match circle.start_point_evidence(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(_) => return Ok(None),
            };
            let end = match circle.end_point_evidence(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(_) => return Ok(None),
            };
            Ok(Some([start, end]))
        };
        let (Some(first), Some(second)) = (endpoints(self)?, endpoints(other)?) else {
            return Ok(Classification::Decided(None));
        };
        let locations = [
            BezierAlgebraicCuspSemicircleContactLocation2::Start,
            BezierAlgebraicCuspSemicircleContactLocation2::End,
        ];
        for first_index in 0..2 {
            for second_index in 0..2 {
                if first[first_index].same_point(&second[second_index], policy)
                    == Classification::Decided(true)
                {
                    return Ok(Classification::Decided(Some([
                        locations[first_index],
                        locations[second_index],
                    ])));
                }
            }
        }
        Ok(Classification::Decided(None))
    }

    pub(in crate::bezier_offset) fn recursive_pair_support_relation_from_centers(
        &self,
        other: &Self,
        first: &BezierRecursiveQuadraticProjectivePoint2,
        second: &BezierRecursiveQuadraticProjectivePoint2,
        center_equality: Option<bool>,
    ) -> CurveResult<Classification<Option<BezierRecursiveCirclePairSupportRelation2>>> {
        if !first
            .denominator
            .field()
            .same_field(&second.denominator.field())
        {
            return Err(CurveError::Topology(
                "a recursive circle-pair relation crossed retained coefficient fields".into(),
            ));
        }
        if center_equality == Some(true) {
            return Ok(Classification::Decided(Some(
                BezierRecursiveCirclePairSupportRelation2::Concentric,
            )));
        }
        let normalize = |point: &BezierRecursiveQuadraticProjectivePoint2| {
            let mut coordinates = [point.x.clone(), point.y.clone(), point.denominator.clone()];
            BezierRecursiveQuadraticValue2::normalize_positive_scale(&mut coordinates);
            let [x, y, denominator] = coordinates;
            BezierRecursiveQuadraticProjectivePoint2 { x, y, denominator }
        };
        let first = normalize(first);
        let second = normalize(second);
        let Some((common_denominator, q)) = (|| {
            let common_denominator = first.denominator.multiply(&second.denominator)?;
            let dx = second
                .x
                .multiply(&first.denominator)?
                .subtract(&first.x.multiply(&second.denominator)?)?;
            let dy = second
                .y
                .multiply(&first.denominator)?
                .subtract(&first.y.multiply(&second.denominator)?)?;
            Some((common_denominator, dx.square()?.add(&dy.square()?)?))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let mut q_sign = match center_equality {
            Some(true) => Classification::Decided(RealSign::Zero),
            // `q` is a sum of two squares after cross-multiplying nonzero
            // projective denominators. Exact center inequality proves it
            // strictly positive without a second algebraic sign solve.
            Some(false) => Classification::Decided(RealSign::Positive),
            None => q.sign(&CurveContext::STRICT)?,
        };
        if matches!(q_sign, Classification::Uncertain(_)) {
            for refinement_steps in [128_usize, 256, 512] {
                let coefficient_bits = refinement_steps.min(i32::MAX as usize) as i32;
                let Some(interval) = q
                    .interval_with_coefficient_precision(refinement_steps, Some(-coefficient_bits))
                else {
                    continue;
                };
                if let Some(sign) = dense_strict_interval_sign(&interval) {
                    q_sign = Classification::Decided(sign);
                    break;
                }
            }
        }
        match q_sign {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided(Some(
                    BezierRecursiveCirclePairSupportRelation2::Concentric,
                )));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a recursive circle-pair center distance squared was negative".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let first_radius_squared = self.radial_distance() * self.radial_distance();
        let second_radius_squared = other.radial_distance() * other.radial_distance();
        let Some(discriminant) = (|| {
            let denominator_squared = common_denominator.square()?;
            let line = q.add(
                &denominator_squared.scale(&(&first_radius_squared - &second_radius_squared))?,
            )?;
            q.multiply(&denominator_squared)?
                .scale(&(Real::from(4_i8) * &first_radius_squared))?
                .subtract(&line.square()?)
        })() else {
            return Ok(Classification::Decided(None));
        };
        let mut discriminant_sign = discriminant.sign(&CurveContext::STRICT)?;
        if matches!(discriminant_sign, Classification::Uncertain(_)) {
            for refinement_steps in [128_usize, 256, 512] {
                let coefficient_bits = refinement_steps.min(i32::MAX as usize) as i32;
                let Some(interval) = discriminant
                    .interval_with_coefficient_precision(refinement_steps, Some(-coefficient_bits))
                else {
                    continue;
                };
                if let Some(sign) = dense_strict_interval_sign(&interval) {
                    discriminant_sign = Classification::Decided(sign);
                    break;
                }
            }
        }
        Ok(discriminant_sign.map(|sign| {
            Some(BezierRecursiveCirclePairSupportRelation2::Discriminant(
                sign,
            ))
        }))
    }

    /// Classifies the two full supporting circles in the least shared
    /// recursive field of their authored centers.  This is the compact
    /// authority for later selected-radial generations whose line-contact
    /// centers do not have useful standalone Cartesian primitive elements.
    pub(in crate::bezier_offset) fn recursive_pair_support_relation(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveCirclePairSupportRelation2>>> {
        let selected_center_distance_squared =
            |candidate: &Self, parent: &Self| -> CurveResult<Option<Real>> {
                let Some(frame) = candidate.data.frame.selected_radial() else {
                    return Ok(None);
                };
                if !policy.accepts_retained_policy(frame.policy) {
                    return Err(CurveError::Topology(
                        "a recursive circle-pair relation crossed selected-radial policies".into(),
                    ));
                }
                let support = frame.center_parameter.semicircle_carrier();
                Ok((support.data.frame == parent.data.frame)
                    .then(|| support.radial_distance() * support.radial_distance()))
            };
        let structural_distance_squared =
            if let Some(distance) = selected_center_distance_squared(self, other)? {
                Some(distance)
            } else {
                selected_center_distance_squared(other, self)?
            };
        if let Some(q) = structural_distance_squared {
            match real_sign(&q, &CurveContext::STRICT) {
                Some(RealSign::Positive) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Decided(Some(
                        BezierRecursiveCirclePairSupportRelation2::Concentric,
                    )));
                }
                Some(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "an authored circle-pair center distance squared was negative".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let first_radius_squared = self.radial_distance() * self.radial_distance();
            let second_radius_squared = other.radial_distance() * other.radial_distance();
            let line = &q + &first_radius_squared - &second_radius_squared;
            let discriminant = Real::from(4_i8) * &q * &first_radius_squared - &line * &line;
            return Ok(match real_sign(&discriminant, &CurveContext::STRICT) {
                Some(sign) => Classification::Decided(Some(
                    BezierRecursiveCirclePairSupportRelation2::Discriminant(sign),
                )),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            });
        }
        let first_center_evidence = self.center_point_evidence(policy)?;
        let second_center_evidence = other.center_point_evidence(policy)?;
        let frame_center_equality = match (&first_center_evidence, &second_center_evidence) {
            (Classification::Decided(first), Classification::Decided(second)) => {
                match policy.strict_predicate_pass(|| {
                    retained_point_evidence_equality_by_refinement(first, second, policy)
                }) {
                    Classification::Decided(equal) => Some(equal),
                    Classification::Uncertain(_) => None,
                }
            }
            (Classification::Decided(_), Classification::Uncertain(_))
            | (Classification::Uncertain(_), Classification::Decided(_))
            | (Classification::Uncertain(_), Classification::Uncertain(_)) => None,
        };
        match self.recursive_pair_frame_authorities(other, policy)? {
            Classification::Decided(Some([first, second])) => {
                return self.recursive_pair_support_relation_from_centers(
                    other,
                    &first.center,
                    &second.center,
                    frame_center_equality,
                );
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }
        let first_center = match first_center_evidence {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second_center = match second_center_evidence {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let centers =
            match recursive_projective_evidence_points(&[&first_center, &second_center], policy)? {
                Classification::Decided(Some(centers)) => centers,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let [first, second]: [BezierRecursiveQuadraticProjectivePoint2; 2] = centers
            .try_into()
            .expect("a recursive circle-pair relation retains two centers");
        let first = match positive_recursive_projective_point(first)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match positive_recursive_projective_point(second)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center_equality = match policy.strict_predicate_pass(|| {
            retained_point_evidence_equality_by_refinement(&first_center, &second_center, policy)
        }) {
            Classification::Decided(equal) => Some(equal),
            Classification::Uncertain(_) => None,
        };
        self.recursive_pair_support_relation_from_centers(other, &first, &second, center_equality)
    }

    /// Publishes tangent or transverse full-circle contacts directly in the
    /// least shared recursive field, then applies both selected-half
    /// predicates before retaining topology.  The point and both angular
    /// parameter predicates share one quadratic extension of the center
    /// field; no Cartesian primitive element is formed.
    pub(in crate::bezier_offset) fn recursive_pair_contact_intersections(
        &self,
        other: &Self,
        discriminant_sign: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<BezierAlgebraicCuspSemicirclePairIntersections2>>> {
        let branches: &[i8] = match discriminant_sign {
            RealSign::Zero => &[0],
            RealSign::Positive => &[-1, 1],
            RealSign::Negative => return Ok(None),
        };
        let [first_frame, second_frame] =
            match self.recursive_pair_frame_authorities(other, policy)? {
                Classification::Decided(Some(frames)) => frames,
                Classification::Decided(None) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "recursive-circle-pair-contact-blocker",
                        "missing-frame-authority",
                    );
                    return Ok(None);
                }
                Classification::Uncertain(_) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "recursive-circle-pair-contact-blocker",
                        "uncertain-frame-authority",
                    );
                    return Ok(None);
                }
            };
        let tangent_endpoint_locations = if discriminant_sign == RealSign::Zero {
            match self.recursive_pair_shared_tangent_endpoint_locations(other, policy)? {
                Classification::Decided(locations) => locations,
                Classification::Uncertain(_) => None,
            }
        } else {
            None
        };
        let first_center = first_frame.center;
        let first_support = first_frame.support_center;
        let first_normal_denominator = first_frame.normal_denominator;
        let second_center = second_frame.center;
        let second_support = second_frame.support_center;
        let second_normal_denominator = second_frame.normal_denominator;
        let parent = first_center.denominator.field();
        if [&second_center, &first_support, &second_support]
            .iter()
            .any(|point| !parent.same_field(&point.denominator.field()))
        {
            return Err(CurveError::Topology(
                "a recursive circle-pair contact crossed retained coefficient fields".into(),
            ));
        }
        let first_radius_squared = self.radial_distance() * self.radial_distance();
        let second_radius_squared = other.radial_distance() * other.radial_distance();
        let Some((common_denominator, dx, dy, q, line, discriminant)) = (|| {
            let common_denominator = first_center
                .denominator
                .multiply(&second_center.denominator)?;
            let dx = second_center
                .x
                .multiply(&first_center.denominator)?
                .subtract(&first_center.x.multiply(&second_center.denominator)?)?;
            let dy = second_center
                .y
                .multiply(&first_center.denominator)?
                .subtract(&first_center.y.multiply(&second_center.denominator)?)?;
            let q = dx.square()?.add(&dy.square()?)?;
            let denominator_squared = common_denominator.square()?;
            let line = q.add(
                &denominator_squared.scale(&(&first_radius_squared - &second_radius_squared))?,
            )?;
            let discriminant = q
                .multiply(&denominator_squared)?
                .scale(&(Real::from(4_i8) * &first_radius_squared))?
                .subtract(&line.square()?)?;
            Some((common_denominator, dx, dy, q, line, discriminant))
        })() else {
            return Ok(None);
        };
        let field = if discriminant_sign == RealSign::Positive {
            let Some(field) = parent.extension(discriminant.clone()) else {
                return Ok(None);
            };
            field
        } else {
            parent.clone()
        };
        let Some((first_center, second_center, first_support, second_support)) = (|| {
            Some((
                first_center.lifted_to(&field)?,
                second_center.lifted_to(&field)?,
                first_support.lifted_to(&field)?,
                second_support.lifted_to(&field)?,
            ))
        })() else {
            return Ok(None);
        };
        let Some((common_denominator, dx, dy, q, line)) = (|| {
            Some((
                field.lift(&common_denominator)?,
                field.lift(&dx)?,
                field.lift(&dy)?,
                field.lift(&q)?,
                field.lift(&line)?,
            ))
        })() else {
            return Ok(None);
        };
        let turn_product = self.turn_sign() * other.turn_sign();
        let tangent_cross_sign = |branch: i8| {
            if branch == 0 {
                RealSign::Zero
            } else {
                let branch_sign = if branch < 0 {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                let turn_sign = if self.is_clockwise() == other.is_clockwise() {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                };
                product_sign(branch_sign, turn_sign)
            }
        };
        let turn_product_sign = if self.is_clockwise() == other.is_clockwise() {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let Some(second_line) = q
            .scale(&Real::from(2_i8))
            .and_then(|twice_q| line.subtract(&twice_q))
        else {
            return Ok(None);
        };
        let tangent_dot_sign = if discriminant_sign == RealSign::Zero {
            let line_sign = match line.sign(&CurveContext::STRICT)? {
                Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a tangent circle pair retained a zero first radial line factor".into(),
                    ));
                }
                Classification::Uncertain(_) => return Ok(None),
            };
            let second_line_sign = match second_line.sign(&CurveContext::STRICT)? {
                Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a tangent circle pair retained a zero second radial line factor".into(),
                    ));
                }
                Classification::Uncertain(_) => return Ok(None),
            };
            Some(product_sign(
                product_sign(line_sign, second_line_sign),
                turn_product_sign,
            ))
        } else {
            None
        };
        let mut contacts = Vec::with_capacity(branches.len());
        let mut retained_contacts = Vec::with_capacity(branches.len());
        for &branch in branches {
            let signed_root = if branch == 0 {
                let Some(zero) = field.constant(Real::zero()) else {
                    return Ok(None);
                };
                zero
            } else {
                let Some(root) = field.element(
                    parent.constant(Real::zero()).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive pair discriminant lost its zero coefficient".into(),
                        )
                    })?,
                    parent.constant(Real::from(branch)).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive pair discriminant lost its branch coefficient".into(),
                        )
                    })?,
                ) else {
                    return Ok(None);
                };
                root
            };
            let Some((point, first_radial, second_radial, radial_denominator)) = (|| {
                // Subtracting either center from the full projective point
                // introduces a large term that cancels identically.  Retain
                // the analytic radial numerators directly instead.  The
                // omitted factor is that center's certified-positive
                // denominator, so signs are unchanged and pairing each
                // numerator with `2 Q common_denominator` preserves the exact
                // angular ratio used by the parameter map.
                let first_radial = [
                    line.multiply(&dx)?.subtract(&signed_root.multiply(&dy)?)?,
                    line.multiply(&dy)?.add(&signed_root.multiply(&dx)?)?,
                ];
                let second_line = line.subtract(&q.scale(&Real::from(2_i8))?)?;
                let second_radial = [
                    second_line
                        .multiply(&dx)?
                        .subtract(&signed_root.multiply(&dy)?)?,
                    second_line
                        .multiply(&dy)?
                        .add(&signed_root.multiply(&dx)?)?,
                ];
                let first_x = q
                    .multiply(&second_center.denominator)?
                    .multiply(&first_center.x)?
                    .scale(&Real::from(2_i8))?;
                let first_y = q
                    .multiply(&second_center.denominator)?
                    .multiply(&first_center.y)?
                    .scale(&Real::from(2_i8))?;
                let radial_denominator =
                    q.multiply(&common_denominator)?.scale(&Real::from(2_i8))?;
                Some((
                    BezierRecursiveQuadraticProjectivePoint2 {
                        x: first_x.add(&first_radial[0])?,
                        y: first_y.add(&first_radial[1])?,
                        denominator: radial_denominator.clone(),
                    },
                    first_radial,
                    second_radial,
                    radial_denominator,
                ))
            })() else {
                return Ok(None);
            };
            let first = match self.recursive_pair_contact_side(
                &first_center,
                &first_support,
                &first_normal_denominator,
                &first_radial,
                &radial_denominator,
                tangent_endpoint_locations.map(|locations| locations[0]),
            )? {
                Classification::Decided(Some(side)) => side,
                Classification::Decided(None) => continue,
                Classification::Uncertain(_) => return Ok(None),
            };
            let second = match other.recursive_pair_contact_side(
                &second_center,
                &second_support,
                &second_normal_denominator,
                &second_radial,
                &radial_denominator,
                tangent_endpoint_locations.map(|locations| locations[1]),
            )? {
                Classification::Decided(Some(side)) => side,
                Classification::Decided(None) => continue,
                Classification::Uncertain(_) => return Ok(None),
            };
            let Some((tangent_cross, tangent_dot)) = (|| {
                // With `D=C2-C1`, `q=D·D`, `a=line`, `b=line-2q`, and
                // `s=branch*sqrt(discriminant)`, the two radial numerators
                // above are `aD+sJ(D)` and `bD+sJ(D)`.  Keep their cross and
                // dot in that factored frame:
                //
                //   cross = 2 s q²
                //   dot   = q (a b + s²)
                //
                // Expanding the Cartesian components first asks a deep
                // recursive coefficient tower to rediscover both exact
                // cancellations.  In particular, a tangent (`s=0`) could be
                // simplified to a false zero dot even though both radii are
                // certified nonzero.
                let tangent_cross = if branch == 0 {
                    field.constant(Real::zero())?
                } else {
                    signed_root
                        .multiply(&q.square()?)?
                        .scale(&(Real::from(2_i8) * &turn_product))?
                };
                let tangent_dot = line
                    .multiply(&second_line)?
                    .add(&signed_root.square()?)?
                    .multiply(&q)?
                    .scale(&turn_product)?;
                Some((tangent_cross, tangent_dot))
            })() else {
                return Ok(None);
            };
            let cross_sign = tangent_cross_sign(branch);
            contacts.push(BezierAlgebraicCuspSemicirclePairContact2 {
                branch,
                first_location: first.location,
                second_location: second.location,
                tangent_cross_sign: cross_sign,
            });
            retained_contacts.push(BezierRecursiveCirclePairContactData2 {
                branch,
                frame: BezierRecursiveQuadraticPairContactFrame2 {
                    field: field.clone(),
                    point,
                    centers: [first_center.clone(), second_center.clone()],
                },
                angular: [first.angular, second.angular],
                tangent_cross,
                tangent_dot,
                tangent_dot_sign,
            });
        }
        if contacts.is_empty() {
            return Ok(Some(Classification::Decided(
                BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
            )));
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-pair-kernel",
            "recursive-contact-publisher",
        );
        let parameter_map = BezierAlgebraicCuspSemicirclePairParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicirclePairParameterMapData2 {
                first_semicircle: self.clone(),
                second_semicircle: other.clone(),
                system: BezierCirclePairParameterMapSystem2::Recursive(
                    BezierRecursiveCirclePairParameterMapSystem2 {
                        contacts: retained_contacts,
                    },
                ),
                recursive_field: OnceLock::new(),
                policy: policy.retained_object_policy_with_dependencies(
                    self.data
                        .frame
                        .evidence_policy()
                        .into_iter()
                        .chain(other.data.frame.evidence_policy()),
                ),
            }),
        };
        Ok(Some(Classification::Decided(
            BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                contacts,
                parameter_map,
            },
        )))
    }

    /// Resolves a circle pair directly from its recursive support relation.
    /// A negative full-circle discriminant (or concentric unequal radii)
    /// proves disjointness; tangent and transverse supports continue through
    /// the recursive contact publisher when their center fields can be joined.
    pub(in crate::bezier_offset) fn recursive_pair_intersections(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<BezierAlgebraicCuspSemicirclePairIntersections2>>> {
        let relation = match self.recursive_pair_support_relation(other, policy)? {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(_) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-circle-pair-contact-blocker",
                    "uncertain-support-relation",
                );
                return Ok(None);
            }
        };
        match relation {
            Some(BezierRecursiveCirclePairSupportRelation2::Discriminant(RealSign::Negative)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-pair-kernel",
                    "recursive-full-circle-disjoint",
                );
                Ok(Some(Classification::Decided(
                    BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
                )))
            }
            Some(BezierRecursiveCirclePairSupportRelation2::Concentric) => {
                let radius_squared_difference = self.radial_distance() * self.radial_distance()
                    - other.radial_distance() * other.radial_distance();
                match real_sign(&radius_squared_difference, &CurveContext::STRICT) {
                    Some(RealSign::Positive | RealSign::Negative) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-pair-kernel",
                            "recursive-concentric-distinct",
                        );
                        Ok(Some(Classification::Decided(
                            BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
                        )))
                    }
                    Some(RealSign::Zero) => {
                        let first_frame = match self.represented_circle_frame(policy)? {
                            Classification::Decided(frame) => frame,
                            Classification::Uncertain(_) => return Ok(None),
                        };
                        let second_frame = match other.represented_circle_frame(policy)? {
                            Classification::Decided(frame) => frame,
                            Classification::Uncertain(_) => return Ok(None),
                        };
                        let first_radius_squared = self.radial_distance() * self.radial_distance();
                        let second_radius_squared =
                            other.radial_distance() * other.radial_distance();
                        Ok(Some(self.represented_coincident_pair_intersections(
                            other,
                            &first_frame,
                            &second_frame,
                            &first_radius_squared,
                            &second_radius_squared,
                            policy,
                        )?))
                    }
                    None => Ok(None),
                }
            }
            Some(BezierRecursiveCirclePairSupportRelation2::Discriminant(
                sign @ (RealSign::Zero | RealSign::Positive),
            )) => self.recursive_pair_contact_intersections(other, sign, policy),
            None => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-circle-pair-contact-blocker",
                    "missing-support-relation",
                );
                Ok(None)
            }
        }
    }

    /// Intersects two selected algebraic cusp semicircles through the exact
    /// circle-circle discriminant. Each of the at most two support branches is
    /// replayed against both oriented half-circle predicates.
    pub(crate) fn pair_intersections(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicirclePairIntersections2>> {
        // Equal selected frames prove equal centers without materializing any
        // algebraic coordinates. Radius and radial signs then enter the same
        // coincident-support topology publisher as every represented frame.
        if self.data.frame == other.data.frame {
            let radius_squared_difference = self.radial_distance() * self.radial_distance()
                - other.radial_distance() * other.radial_distance();
            match real_sign(&radius_squared_difference, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts,
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                Some(RealSign::Zero) => {}
            }
            let radial_dot =
                match real_sign(&(self.radial_distance() * other.radial_distance()), policy) {
                    Some(RealSign::Positive) => RealSign::Positive,
                    Some(RealSign::Negative) => RealSign::Negative,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a selected circle retained a zero signed radius".into(),
                        ));
                    }
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
            return self.coincident_pair_intersections_from_radial_signs(
                other,
                RealSign::Zero,
                Some(radial_dot),
                None,
                policy,
            );
        }
        let center_relation = self.represented_structural_center_relation(other, policy)?;
        let translation = self.represented_structural_translation(other, policy)?;
        let represented = || {
            self.represented_pair_intersections(
                other,
                center_relation.as_ref(),
                translation.as_ref(),
                policy,
            )
        };
        // An exact translation keeps the circle formula in Real arithmetic.
        // A center-distance certificate alone does not bound the cost of
        // reconstructing angular coordinates, so prefer shared fields there.
        if translation.is_some()
            && let Classification::Decided(intersections) =
                policy.strict_predicate_pass(represented)?
        {
            return Ok(Classification::Decided(intersections));
        }
        if let Some(Classification::Decided(intersections)) =
            policy.strict_predicate_pass(|| self.recursive_pair_intersections(other, policy))?
        {
            return Ok(Classification::Decided(intersections));
        }
        represented()
    }
}
