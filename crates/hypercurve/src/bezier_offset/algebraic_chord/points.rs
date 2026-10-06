//! Chord-pair, chord-parallel and cusp-chord-derived points.

use super::*;

impl BezierAlgebraicChordPairPoint2 {
    pub(in crate::bezier_offset) fn accepts_policy(&self, policy: &CurveContext) -> bool {
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
    pub(in crate::bezier_offset) fn anchor_separation_order_on_owner(
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

    pub(in crate::bezier_offset) fn new(
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

    pub(in crate::bezier_offset) fn new_with_anchor_orders(
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

    pub(in crate::bezier_offset) fn new_with_location(
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
    pub(in crate::bezier_offset) fn visit_incidence_on_parallel(
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

    pub(in crate::bezier_offset) fn translated(
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

    /// Transforms both defining supports. A retained similarity, when the
    /// map is one, lets procedural parallel endpoints of those supports keep
    /// their construction instead of requiring an affine coordinate image.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn transform_affine(
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
        let first = match self.data.first.transform_affine_with_similarity(
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
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match self.data.second.transform_affine_with_similarity(
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

    pub(in crate::bezier_offset) fn constant_axis_support_point(
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

    pub(in crate::bezier_offset) fn exact_axis_coordinate(
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
    pub(in crate::bezier_offset) fn recursive_projective_point(
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
    pub(in crate::bezier_offset) fn represented_coordinates(
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
                if first.overlaps_with_policy(&second, policy) == Classification::Decided(false) {
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
    pub(in crate::bezier_offset) fn flat_recursive_oriented_side_to_chord(
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
        let sign = |value: &RecursiveQuadraticValue| {
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
    pub(in crate::bezier_offset) fn endpoint_incidence_oriented_side_to_chord(
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

    pub(in crate::bezier_offset) fn oriented_side_to_chord(
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
            let coordinate = |bounds: &Aabb2, axis| real_interval_from_axis(bounds, axis);
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

    pub(in crate::bezier_offset) fn intersection_bounds_refined(
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
                Some(real_interval_from_axis(&bounds, axis))
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
        let coordinate = |bounds: &Aabb2, axis| real_interval_from_axis(bounds, axis);
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
            if pair_bounds.overlaps_with_policy(&other_bounds, &CurveContext::STRICT)
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

    pub(in crate::bezier_offset) fn cmp_on_chord_to_evidence(
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
            let reversed = match owner.collinear_traversal_reversed(chord, policy)? {
                Classification::Decided(reversed) => reversed,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
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
            let owner_reversed = match owner.collinear_traversal_reversed(chord, policy)? {
                Classification::Decided(reversed) => reversed,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let retained_cross = retained_pair_cross.map(|mut sign| {
                if !shares_first_support {
                    sign = product_sign(sign, RealSign::Negative);
                }
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
        Ok(owner
            .collinear_traversal_reversed(chord, policy)?
            .map(|reversed| if reversed { order.reverse() } else { order }))
    }

    pub(in crate::bezier_offset) fn cmp_on_common_chord(
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
    pub(in crate::bezier_offset) fn accepts_policy(&self, policy: &CurveContext) -> bool {
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

    pub(in crate::bezier_offset) fn tangent_endpoint(
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

    pub(in crate::bezier_offset) fn normal_displaced_point(
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

    pub(in crate::bezier_offset) fn tangent_displaced_point(
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

    pub(in crate::bezier_offset) fn source_endpoint(&self) -> &CurvePoint2 {
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
    pub(in crate::bezier_offset) fn source_direction_endpoints<'a>(
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
    pub(in crate::bezier_offset) fn strict_exact_point(
        &self,
        policy: &CurveContext,
    ) -> Option<Point2> {
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

    pub(in crate::bezier_offset) fn recursive_projective_frame(
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
            let coordinate = |coordinate_origin: &RecursiveQuadraticValue,
                              unit: &RecursiveQuadraticValue,
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
    pub(in crate::bezier_offset) fn recursive_projective_point(
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

    pub(in crate::bezier_offset) fn represented_coordinates(
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

    pub(in crate::bezier_offset) fn shares_carrier(&self, other: &Self) -> bool {
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
    pub(in crate::bezier_offset) fn shares_normal_offset_carrier(&self, other: &Self) -> bool {
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
    pub(in crate::bezier_offset) fn normal_offset_oriented_side_to_chord(
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
        let lift = |value: &RecursiveQuadraticValue| field.lift(value);
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
    pub(in crate::bezier_offset) fn analytic_tangent_side_interval_sign(
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
                    real_interval_from_parameter(&parameter)
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
            let direction_x = real_interval_from_axis(&direction_end, Axis2::X)
                .subtract(&real_interval_from_axis(&direction_start, Axis2::X));
            let direction_y = real_interval_from_axis(&direction_end, Axis2::Y)
                .subtract(&real_interval_from_axis(&direction_start, Axis2::Y));
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
                let physical_x = real_interval_from_axis(&end, Axis2::X)
                    .subtract(&real_interval_from_axis(&start, Axis2::X));
                let physical_y = real_interval_from_axis(&end, Axis2::Y)
                    .subtract(&real_interval_from_axis(&start, Axis2::Y));
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
    pub(in crate::bezier_offset) fn recursive_general_oriented_side_to_analytic_tangent_chord(
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
        let sign = RecursiveQuadraticValue::nested_positive_root_affine_sign(
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
    pub(in crate::bezier_offset) fn oriented_side_to_analytic_tangent_chord(
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
        let sign = match RecursiveQuadraticValue::nested_positive_root_affine_sign(
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

    pub(in crate::bezier_offset) fn translated(
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

    pub(in crate::bezier_offset) fn transform_similarity_cached(
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

    pub(in crate::bezier_offset) fn conservative_local_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, true)
    }

    pub(in crate::bezier_offset) fn conservative_bounds_refined_impl(
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
        let delta_x = real_interval_from_axis(&end, Axis2::X)
            .subtract(&real_interval_from_axis(&start, Axis2::X));
        let delta_y = real_interval_from_axis(&end, Axis2::Y)
            .subtract(&real_interval_from_axis(&start, Axis2::Y));
        // An outward dyadic speed enclosure is all this box needs; exact
        // radical endpoints would replay a symbolic square root at every
        // refinement.
        let speed_precision = i32::try_from(refinement_steps)
            .ok()
            .and_then(|steps| steps.checked_add(128))
            .map_or(i32::MIN, |bits| -bits);
        let Some(speed) = delta_x
            .square()
            .and_then(|x| delta_y.square().map(|y| x.add(&y)))
            .and_then(|speed_squared| speed_squared.nonnegative_square_root(Some(speed_precision)))
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
        let x = real_interval_from_axis(&origin, Axis2::X)
            .add(&offset_x)
            .add(&RealInterval {
                lower: self.data.translation_x.clone(),
                upper: self.data.translation_x.clone(),
            });
        let y = real_interval_from_axis(&origin, Axis2::Y)
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
    pub(in crate::bezier_offset) fn concentric_circle_incidence_sign(
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
            let radial_x = real_interval_from_axis(&point, Axis2::X)
                .subtract(&real_interval_from_axis(&center, Axis2::X));
            let radial_y = real_interval_from_axis(&point, Axis2::Y)
                .subtract(&real_interval_from_axis(&center, Axis2::Y));
            let chord_x = real_interval_from_axis(&chord_end, Axis2::X)
                .subtract(&real_interval_from_axis(&chord_start, Axis2::X));
            let chord_y = real_interval_from_axis(&chord_end, Axis2::Y)
                .subtract(&real_interval_from_axis(&chord_start, Axis2::Y));
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

    pub(in crate::bezier_offset) fn strict_cardinal_axis_shift(
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

    /// Returns this point's exact displacement from its source endpoint along
    /// one axis when the source chord retains an exact unit tangent.
    pub(in crate::bezier_offset) fn exact_axis_shift(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Option<Real> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return None;
        }
        let (tangent_x, tangent_y) = self.data.source.certified_unit_tangent()?;
        let component = match (self.data.direction, axis) {
            (BezierAlgebraicChordUnitDisplacement2::Tangent, Axis2::X) => tangent_x,
            (BezierAlgebraicChordUnitDisplacement2::Tangent, Axis2::Y) => tangent_y,
            (BezierAlgebraicChordUnitDisplacement2::LeftNormal, Axis2::X) => -tangent_y,
            (BezierAlgebraicChordUnitDisplacement2::LeftNormal, Axis2::Y) => tangent_x,
        };
        let translation = match axis {
            Axis2::X => &self.data.translation_x,
            Axis2::Y => &self.data.translation_y,
        };
        Some(translation + &self.data.distance * component)
    }

    pub(in crate::bezier_offset) fn strict_cardinal_shifts(
        &self,
        policy: &CurveContext,
    ) -> Option<[Real; 2]> {
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
    pub(in crate::bezier_offset) fn strict_cardinal_point_evidence(
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

    pub(in crate::bezier_offset) fn strict_preserved_axis_source<'a>(
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
    pub(in crate::bezier_offset) fn cardinal_axis_order_to_parallel(
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

    pub(in crate::bezier_offset) fn exact_axis_coordinate(
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

    pub(in crate::bezier_offset) fn axis_coordinate_order_to_real(
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
    pub(in crate::bezier_offset) fn from_mapped_source(
        parameter: Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
        point: Option<CurvePoint2>,
        radial_scale: Real,
    ) -> Self {
        Self::from_mapped_source_with_rotation(parameter, point, radial_scale, Real::zero())
    }

    pub(in crate::bezier_offset) fn from_mapped_source_with_rotation(
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

    pub(in crate::bezier_offset) fn rotated_from_mapped_source(
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

    pub(in crate::bezier_offset) fn translated(
        &self,
        translation_x: &Real,
        translation_y: &Real,
    ) -> Self {
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
    pub(in crate::bezier_offset) fn common_radial_source_axis_order(
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
    pub(in crate::bezier_offset) fn common_untranslated_radial_difference_sign(
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
    pub(in crate::bezier_offset) fn common_untranslated_radial_line_oriented_side(
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

    pub(in crate::bezier_offset) fn recursive_projective_point(
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
    pub(in crate::bezier_offset) fn recursive_projective_axis_order(
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

    pub(in crate::bezier_offset) fn recursive_projective_axis_order_to_chord(
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
    pub(in crate::bezier_offset) fn is_similarity_image_of(
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
    pub(in crate::bezier_offset) fn exact_supporting_circle(
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
    pub(in crate::bezier_offset) fn represented_coordinates(
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

    pub(in crate::bezier_offset) fn affine_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.affine_bounds_refined_impl(refinement_steps, policy, false)
    }

    pub(in crate::bezier_offset) fn affine_bounds_refined_impl(
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
        let point_x = real_interval_from_axis(&point, Axis2::X);
        let point_y = real_interval_from_axis(&point, Axis2::Y);
        let center_x = real_interval_from_axis(&center, Axis2::X);
        let center_y = real_interval_from_axis(&center, Axis2::Y);
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

    pub(in crate::bezier_offset) fn refined_linear_order_to_real(
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
            let x = real_interval_from_axis(&bounds, Axis2::X);
            let y = real_interval_from_axis(&bounds, Axis2::Y);
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

    pub(in crate::bezier_offset) fn conservative_local_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, true)
    }

    pub(in crate::bezier_offset) fn conservative_bounds_refined_impl(
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

    pub(in crate::bezier_offset) fn axis_coordinate_order_to_real(
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
        // With no perpendicular term, Q = C + a (P - C) + T. When the source
        // P is its own source endpoint E displaced by an exact vector and E is
        // the circle center C, then Q.axis = C.axis + a * shift + T.axis, so
        // the comparison moves to the center's own exact axis order.
        if self.data.perpendicular_scale.zero_status() == ZeroKnowledge::Zero
            && let Some(CurvePoint2(CurvePointData2::AlgebraicChordParallel(source))) =
                self.data.source.retained_point()
            && let Some(shift) = source.exact_axis_shift(axis, policy)
            && let Ok(Classification::Decided(center)) =
                self.data.source.semicircle().center_point_evidence(policy)
            && center.same_point(source.source_endpoint(), &CurveContext::STRICT)
                == Classification::Decided(true)
        {
            let translation = match axis {
                Axis2::X => &self.data.translation_x,
                Axis2::Y => &self.data.translation_y,
            };
            let target = value - &self.data.radial_scale * &shift - translation;
            if let Classification::Decided(center_order) = policy.strict_predicate_pass(|| {
                BezierAlgebraicChord2::point_axis_order_to_real(&center, axis, &target, policy)
            })? {
                return Ok(Classification::Decided(center_order));
            }
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
    pub(in crate::bezier_offset) fn complementary_mapped_axis_order(
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
    pub(in crate::bezier_offset) fn strict_parallel_complementary_mapped_axis_order(
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
    pub(in crate::bezier_offset) fn strict_pair_contact_join_unit_tangent(
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

    pub(in crate::bezier_offset) fn complementary_mapped_axis_order_with_authority(
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

    pub(in crate::bezier_offset) fn linear_order_to_real(
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

    pub(in crate::bezier_offset) fn oriented_side_to_chord(
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

    pub(in crate::bezier_offset) fn same_source_transform(
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

    pub(in crate::bezier_offset) fn center_relative_coordinate_expression(
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

    pub(in crate::bezier_offset) fn unshifted_concentric_circle_incidence_residual(
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
    pub(in crate::bezier_offset) fn equal_radius_pair_other_circle_incidence_sign(
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
    pub(in crate::bezier_offset) fn concentric_circle_incidence_sign(
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
            let Some(rational_dot) = TrivariatePolynomial::linear_combination(&[
                (&system.point_x.rational, &self.data.translation_x),
                (&system.center_x, &negative_translation_x),
                (&system.point_y.rational, &self.data.translation_y),
                (&system.center_y, &negative_translation_y),
            ]) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(radical_dot) = TrivariatePolynomial::linear_combination(&[
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
    pub(in crate::bezier_offset) fn concentric_axis_contact_minus_point_sign(
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
            if first.overlaps_with_policy(&second, policy) == Classification::Decided(false) {
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
    pub(in crate::bezier_offset) fn exact_one_field_point_image(
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
    pub(in crate::bezier_offset) fn selected_chord_normal_parallel_distance_delta(
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

    pub(in crate::bezier_offset) fn same_selected_chord_normal_parallel_point(
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
    pub(in crate::bezier_offset) fn transverse_cardinal_chord_parallel_axis_order(
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

    pub(in crate::bezier_offset) fn selected_chord_normal_parallel_axis_order(
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
