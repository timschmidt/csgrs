//! Rational Bezier intersection candidates, contacts and self-contacts.

use super::*;

impl RationalBezier2 {
    #[cfg(test)]
    /// Returns exact resultant candidates for all finite curve contacts.
    ///
    /// Homogeneous coordinate equations are eliminated in each parameter
    /// direction. Roots are retained as represented or algebraically isolated
    /// [`BezierParameter2`] values. The two projections are not paired or
    /// accepted as contacts until a later exact replay proves equal images.
    pub(crate) fn intersection_candidates(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveIntersectionCandidates2> {
        match self.intersection_candidates_classified(other, policy) {
            Ok(Classification::Decided(candidates)) => Ok(candidates),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    pub(super) fn intersection_context_classified(
        &self,
        other: &Self,
        policy: &CurveContext,
        circle_relation: Option<&CircleCircleRelation>,
        first_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
        second_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
    ) -> CurveResult<Classification<RationalBezierIntersectionContext>> {
        if self == other {
            let overlap = RationalBezierIntersectionOverlap2 {
                first_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                second_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                orientation: CurveOverlapOrientation2::Same,
                endpoint_inclusion: [true, true],
            };
            let contacts = RationalBezierIntersectionContacts2::Overlap(overlap);
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if self
            .homogeneous_controls()
            .iter()
            .rev()
            .eq(other.homogeneous_controls().iter())
            && self.weights().iter().rev().eq(other.weights().iter())
        {
            let overlap = RationalBezierIntersectionOverlap2 {
                first_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                second_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::one()),
                    BezierParameter2::Exact(Real::zero()),
                ),
                orientation: CurveOverlapOrientation2::Reversed,
                endpoint_inclusion: [true, true],
            };
            let contacts = RationalBezierIntersectionContacts2::Overlap(overlap);
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if self.certified_bounds_are_disjoint(other, policy) {
            let contacts = OnceLock::new();
            let _ = contacts.set(Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates: CurveIntersectionCandidates2::NoIntersection,
                    contacts,
                },
            }));
        }
        if let Some(contacts) = self.retained_lineage_intersection_contacts(other, policy)? {
            let contacts = match contacts {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if let Some(contacts) = self.circular_conic_intersection_contacts(
            other,
            policy,
            circle_relation,
            first_circle_parameters,
            second_circle_parameters,
        )? {
            let contacts = match contacts {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if let Some(contacts) = self.retained_linear_image_contacts(other, policy)? {
            let contacts = match contacts {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if let Some(Classification::Decided(contacts)) =
            self.certified_linear_image_contacts(other, policy)?
        {
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }

        // A line image can arrive degree-elevated through several native
        // carriers. Align it with the nonlinear operand before elimination so
        // the resultant does not retain parameterization-only base factors.
        if self.degree() < other.degree()
            && matches!(
                self.fit_exact_line_image(policy)?,
                Classification::Decided(BezierLineImageFitRelation::Fit(_))
            )
        {
            let elevated = match self.elevated_to_degree(other.degree()) {
                Ok(elevated) => elevated,
                Err(ExactCurveError::Blocked(blocker)) => {
                    return Ok(Classification::Uncertain(blocker.reason()));
                }
                Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            };
            return elevated.intersection_context_classified(other, policy, None, None, None);
        }
        if other.degree() < self.degree()
            && matches!(
                other.fit_exact_line_image(policy)?,
                Classification::Decided(BezierLineImageFitRelation::Fit(_))
            )
        {
            let elevated = match other.elevated_to_degree(self.degree()) {
                Ok(elevated) => elevated,
                Err(ExactCurveError::Blocked(blocker)) => {
                    return Ok(Classification::Uncertain(blocker.reason()));
                }
                Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            };
            return self.intersection_context_classified(&elevated, policy, None, None, None);
        }

        let line_image_contacts =
            if let Some(contacts) = self.exact_line_image_intersection_contacts(other, policy)? {
                Some(contacts)
            } else {
                other
                    .exact_line_image_intersection_contacts(self, policy)?
                    .map(|contacts| contacts.map(reverse_rational_intersection_contacts))
            };
        if let Some(Classification::Decided(contacts)) = line_image_contacts {
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if matches!(
            self.shares_implicit_quadratic_conic(other, policy),
            Classification::Decided(true)
        ) {
            match self.replay_intersection_candidate_set(
                other,
                &CurveIntersectionCandidates2::DegenerateResultant,
                policy,
            )? {
                Classification::Decided(
                    RationalBezierIntersectionContacts2::DegenerateResultant,
                ) => {}
                Classification::Decided(contacts) => {
                    let candidates = intersection_candidates_from_contacts(&contacts);
                    let contact_cache = OnceLock::new();
                    let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
                    return Ok(Classification::Decided(RationalBezierIntersectionContext {
                        data: RationalBezierIntersectionContextData {
                            first: self.clone(),
                            second: other.clone(),
                            policy: *policy,
                            candidates,
                            contacts: contact_cache,
                        },
                    }));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let special =
            if let Some(contacts) = self.implicit_conic_intersection_contacts(other, policy)? {
                Some(contacts)
            } else {
                other
                    .implicit_conic_intersection_contacts(self, policy)?
                    .map(|contacts| contacts.map(reverse_rational_intersection_contacts))
            };
        if let Some(Classification::Decided(contacts)) = special {
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        // Bounds were already checked before the implicit-conic fast path.
        // Continue directly so an overlapping pair is not boxed and compared
        // a second time before resultant construction.
        match self.intersection_candidates_after_bounds_check(other, policy)? {
            Classification::Decided(candidates) => {
                Ok(Classification::Decided(RationalBezierIntersectionContext {
                    data: RationalBezierIntersectionContextData {
                        first: self.clone(),
                        second: other.clone(),
                        policy: *policy,
                        candidates,
                        contacts: OnceLock::new(),
                    },
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(super) fn retained_linear_image_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let (Some(first), Some(second)) = (
            self.exact_linear_parameterization_line(),
            other.exact_linear_parameterization_line(),
        ) else {
            return Ok(None);
        };
        Ok(Some(match first.intersect_line(&second, policy)? {
            crate::LineLineIntersection::None => {
                Classification::Decided(RationalBezierIntersectionContacts2::NoIntersection)
            }
            crate::LineLineIntersection::Point {
                point,
                a_param,
                b_param,
                kind,
            } => {
                Classification::Decided(RationalBezierIntersectionContacts2::Contacts(Arc::from([
                    RationalBezierIntersectionContact2 {
                        first_parameter: BezierParameter2::Exact(a_param),
                        second_parameter: BezierParameter2::Exact(b_param),
                        point: CurvePoint2::from(point),
                        certified_transverse: kind == crate::IntersectionKind::Crossing,
                        tangent_cross_sign: None,
                    },
                ])))
            }
            crate::LineLineIntersection::Overlap { .. } => return Ok(None),
            crate::LineLineIntersection::Uncertain { reason } => Classification::Uncertain(reason),
        }))
    }

    pub(super) fn certified_linear_image_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let first = match self.fit_exact_line_image(policy)? {
            Classification::Decided(BezierLineImageFitRelation::Fit(first)) => first,
            Classification::Decided(BezierLineImageFitRelation::NotLine) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let second = match other.fit_exact_line_image(policy)? {
            Classification::Decided(BezierLineImageFitRelation::Fit(second)) => second,
            Classification::Decided(BezierLineImageFitRelation::NotLine) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        Ok(match first.line().intersect_line(second.line(), policy)? {
            crate::LineLineIntersection::None => Some(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )),
            crate::LineLineIntersection::Point { point, kind, .. } => {
                let unique_parameter =
                    |curve: &Self| match unique_point_incidence_parameter(curve, &point, policy) {
                        Classification::Decided(Some(parameter)) => Ok(parameter),
                        Classification::Decided(None) => Err(UncertaintyReason::Predicate),
                        Classification::Uncertain(reason) => Err(reason),
                    };
                let first_parameter = match unique_parameter(self) {
                    Ok(parameter) => parameter,
                    Err(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                };
                let second_parameter = match unique_parameter(other) {
                    Ok(parameter) => parameter,
                    Err(reason) => return Ok(Some(Classification::Uncertain(reason))),
                };
                Some(Classification::Decided(
                    RationalBezierIntersectionContacts2::Contacts(Arc::from([
                        RationalBezierIntersectionContact2 {
                            first_parameter,
                            second_parameter,
                            point: CurvePoint2::from(point),
                            certified_transverse: kind == crate::IntersectionKind::Crossing,
                            tangent_cross_sign: None,
                        },
                    ])),
                ))
            }
            crate::LineLineIntersection::Overlap { .. } => {
                match self.certified_line_image_overlap(other, policy) {
                    Classification::Decided(Some(overlap)) => Some(Classification::Decided(
                        RationalBezierIntersectionContacts2::Overlap(overlap),
                    )),
                    Classification::Decided(None) => None,
                    Classification::Uncertain(reason) => Some(Classification::Uncertain(reason)),
                }
            }
            crate::LineLineIntersection::Uncertain { reason } => {
                Some(Classification::Uncertain(reason))
            }
        })
    }

    #[cfg(test)]
    /// Replays all resultant projections into exact paired contacts.
    ///
    /// The result distinguishes complete contact sets from partial algebraic
    /// replay. No raw resultant root is accepted as a contact without exact
    /// equality of both constructed affine coordinates.
    pub(crate) fn intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalBezierIntersectionContacts2> {
        match self.intersection_contacts_classified(other, policy) {
            Ok(Classification::Decided(contacts)) => Ok(contacts),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    pub(crate) fn intersection_contacts_classified(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        if self.certified_bounds_are_disjoint(other, policy) {
            return Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            ));
        }
        if let Some(contacts) = self.retained_lineage_intersection_contacts(other, policy)? {
            return Ok(contacts);
        }
        if let Some(Classification::Decided(contacts)) =
            self.certified_linear_image_contacts(other, policy)?
        {
            return Ok(Classification::Decided(contacts));
        }

        // A symbolic quadratic conic is better served by its implicit
        // equation than by direct Bernstein line-side signs. The latter can
        // exhaust the sign budget before the exact algebraic replay that is
        // specifically able to retain the symbolic coefficient field.
        if self.degree() == 2
            && self
                .weights()
                .iter()
                .any(|weight| weight.exact_rational_ref().is_none())
            && let Some(Classification::Decided(contacts)) =
                self.implicit_conic_intersection_contacts(other, policy)?
        {
            return Ok(Classification::Decided(contacts));
        }
        if other.degree() == 2
            && other
                .weights()
                .iter()
                .any(|weight| weight.exact_rational_ref().is_none())
            && let Some(Classification::Decided(contacts)) =
                other.implicit_conic_intersection_contacts(self, policy)?
        {
            return Ok(Classification::Decided(
                reverse_rational_intersection_contacts(contacts),
            ));
        }

        // A line-image shortcut can be unable to classify a transcendental
        // conic even though the implicit-conic route below can replay the
        // contact exactly. Treat only decided shortcut results as terminal.
        if let Some(Classification::Decided(contacts)) =
            self.exact_line_image_intersection_contacts(other, policy)?
        {
            return Ok(Classification::Decided(contacts));
        }
        if let Some(Classification::Decided(contacts)) =
            other.exact_line_image_intersection_contacts(self, policy)?
        {
            return Ok(Classification::Decided(
                reverse_rational_intersection_contacts(contacts),
            ));
        }

        if let Some(Classification::Decided(contacts)) =
            self.implicit_conic_intersection_contacts(other, policy)?
        {
            return Ok(Classification::Decided(contacts));
        }
        if let Some(Classification::Decided(contacts)) =
            other.implicit_conic_intersection_contacts(self, policy)?
        {
            return Ok(Classification::Decided(
                reverse_rational_intersection_contacts(contacts),
            ));
        }
        if matches!(
            self.shares_implicit_quadratic_conic(other, policy),
            Classification::Decided(true)
        ) {
            return self.replay_intersection_candidate_set(
                other,
                &CurveIntersectionCandidates2::DegenerateResultant,
                policy,
            );
        }
        // Bounds were already checked before the implicit-conic fast path.
        // Continue directly so an overlapping or inconclusive pair is not
        // boxed and compared a second time before resultant construction.
        let candidates = match self.intersection_candidates_after_bounds_check(other, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.replay_intersection_candidate_set(other, &candidates, policy)
    }

    #[cfg(test)]
    /// Returns every unordered off-diagonal self-contact of this curve.
    ///
    /// Both homogeneous coordinate-equality equations contain the universal
    /// `u - t` identity component. The self-contact authority divides that
    /// component exactly before elimination, projects the resulting symmetric
    /// system once, and reuses ordinary affine contact replay. A remaining
    /// positive-dimensional correspondence is reported as a degenerate
    /// resultant instead of being mistaken for isolated contacts.
    pub(crate) fn self_intersection_contacts(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalBezierIntersectionContacts2> {
        match self.self_intersection_contacts_classified(policy) {
            Ok(Classification::Decided(contacts)) => Ok(contacts),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    pub(crate) fn self_intersection_contacts_classified(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        self.self_intersection_contacts_with_point_evidence_classified(policy, &mut |_| Ok(None))
    }

    pub(crate) fn self_intersection_contacts_with_point_evidence_classified(
        &self,
        policy: &CurveContext,
        fallback_point_evidence: &mut dyn FnMut(
            &BezierParameter2,
        ) -> CurveResult<Option<CurvePoint2>>,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        if self.has_certified_injective_axis(policy) {
            return Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            ));
        }
        if let Classification::Uncertain(reason) =
            self.denominator_sign(&crate::CurveParameterRange2::unit())
        {
            return Ok(Classification::Uncertain(reason));
        }
        let basis = self.homogeneous_power_basis()?;
        let Some(equations) = rational_self_intersection_residual_system(basis) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let candidates = match project_symmetric_self_intersection_system(&equations, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let replayed = match &candidates {
            CurveIntersectionCandidates2::NoIntersection => {
                RationalBezierIntersectionContacts2::NoIntersection
            }
            CurveIntersectionCandidates2::DegenerateResultant => {
                RationalBezierIntersectionContacts2::DegenerateResultant
            }
            CurveIntersectionCandidates2::Candidates {
                first_parameters,
                second_parameters,
            } => match self.replay_intersection_candidates_with_pair_filter(
                self,
                first_parameters,
                second_parameters,
                true,
                Some(&equations),
                Some(fallback_point_evidence),
                policy,
            )? {
                Classification::Decided(replayed) => replayed,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        retain_unordered_rational_self_contacts(replayed, basis, policy)
    }

    pub(super) fn retained_lineage_intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        if !Arc::ptr_eq(&self.data.lineage.root, &other.data.lineage.root)
            || self == other
            || (self
                .homogeneous_controls()
                .iter()
                .rev()
                .eq(other.homogeneous_controls().iter())
                && self.weights().iter().rev().eq(other.weights().iter()))
        {
            return Ok(None);
        }
        self.retain_root_image_injectivity(policy);
        other.retain_root_image_injectivity(policy);
        if self.has_injective_root_chart(policy) && other.has_injective_root_chart(policy) {
            return Ok(None);
        }

        let source_overlap = match self.retained_source_parameter_overlap(other, policy) {
            Classification::Decided(overlap) => overlap,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let Some(equations) = rational_retained_lineage_residual_system(self, other)? else {
            return Ok(Some(Classification::Uncertain(
                UncertaintyReason::Unsupported,
            )));
        };
        let candidates = match project_retained_lineage_residual_system(&equations, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let replayed = if matches!(
            candidates,
            CurveIntersectionCandidates2::DegenerateResultant
        ) {
            RationalBezierIntersectionContacts2::DegenerateResultant
        } else {
            match self.replay_intersection_candidate_set(other, &candidates, policy)? {
                Classification::Decided(replayed) => replayed,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        };
        let replayed = match self.remove_same_source_parameter_contacts(other, replayed, policy)? {
            Classification::Decided(replayed) => replayed,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let replayed = if replayed.isolated_contacts().is_empty() {
            replayed
        } else {
            let tangent_cross = rational_pair_tangent_cross_polynomial(
                self.homogeneous_power_basis()?,
                other.homogeneous_power_basis()?,
            );
            match retain_rational_contact_tangent_cross_signs(
                replayed,
                tangent_cross.as_ref(),
                policy,
            )? {
                Classification::Decided(replayed) => replayed,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        };

        let replayed = if source_overlap.is_none() {
            let identity_contacts = match self.retained_lineage_touch_contacts(other, policy)? {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            append_complete_rational_contacts(replayed, identity_contacts)
        } else {
            replayed
        };
        let result = match (source_overlap, replayed) {
            (None, replayed) => replayed,
            (Some(overlap), RationalBezierIntersectionContacts2::NoIntersection) => {
                RationalBezierIntersectionContacts2::Overlap(overlap)
            }
            (Some(overlap), RationalBezierIntersectionContacts2::Contacts(contacts)) => {
                RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, overlap }
            }
            (
                Some(_),
                RationalBezierIntersectionContacts2::Incomplete { .. }
                | RationalBezierIntersectionContacts2::DegenerateResultant,
            ) => RationalBezierIntersectionContacts2::DegenerateResultant,
            (Some(_), RationalBezierIntersectionContacts2::Overlap(_))
            | (Some(_), RationalBezierIntersectionContacts2::ContactsAndOverlap { .. }) => {
                unreachable!("residual replay cannot produce an image overlap")
            }
        };
        Ok(Some(Classification::Decided(result)))
    }

    pub(super) fn remove_same_source_parameter_contacts(
        &self,
        other: &Self,
        replayed: RationalBezierIntersectionContacts2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        let first_range = self.source_parameter_range();
        let second_range = other.source_parameter_range();
        let numerator = vec![
            first_range.start() - second_range.start(),
            first_range.end() - first_range.start(),
        ];
        let denominator = vec![second_range.end() - second_range.start()];
        if is_zero(&denominator[0], policy) != Some(false) {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let retain = |contacts: Arc<[RationalBezierIntersectionContact2]>|
         -> CurveResult<Classification<Arc<[RationalBezierIntersectionContact2]>>> {
            let mut retained = Vec::with_capacity(contacts.len());
            for contact in contacts.iter() {
                match rational_parameter_image_matches(
                    contact.first_parameter(),
                    contact.second_parameter(),
                    &numerator,
                    &denominator,
                    policy,
                )? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => retained.push(contact.clone()),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(retained.into()))
        };
        Ok(match replayed {
            RationalBezierIntersectionContacts2::Contacts(contacts) => match retain(contacts)? {
                Classification::Decided(contacts) if contacts.is_empty() => {
                    Classification::Decided(RationalBezierIntersectionContacts2::NoIntersection)
                }
                Classification::Decided(contacts) => {
                    Classification::Decided(RationalBezierIntersectionContacts2::Contacts(contacts))
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
            RationalBezierIntersectionContacts2::Incomplete {
                contacts,
                candidates,
            } => match retain(contacts)? {
                Classification::Decided(contacts) => {
                    Classification::Decided(RationalBezierIntersectionContacts2::Incomplete {
                        contacts,
                        candidates,
                    })
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
            replayed => Classification::Decided(replayed),
        })
    }

    pub(super) fn retained_lineage_touch_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<RationalBezierIntersectionContact2>>> {
        let mut contacts = Vec::with_capacity(1);
        for (first_parameter, first_source) in [
            (Real::zero(), self.source_parameter_range().start()),
            (Real::one(), self.source_parameter_range().end()),
        ] {
            for (second_parameter, second_source) in [
                (Real::zero(), other.source_parameter_range().start()),
                (Real::one(), other.source_parameter_range().end()),
            ] {
                match compare_reals(first_source, second_source, policy) {
                    Some(Ordering::Equal) => {
                        let point = match self.point_at_classified(&first_parameter, policy) {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        contacts.push(RationalBezierIntersectionContact2 {
                            first_parameter: BezierParameter2::Exact(first_parameter.clone()),
                            second_parameter: BezierParameter2::Exact(second_parameter.clone()),
                            point: CurvePoint2::from(point),
                            certified_transverse: false,
                            tangent_cross_sign: None,
                        });
                    }
                    Some(_) => {}
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                }
            }
        }
        Ok(Classification::Decided(contacts))
    }

    pub(super) fn implicit_conic_intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let conic = match self.implicit_quadratic_conic(policy) {
            Classification::Decided(Some(conic)) => conic,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(_) => return Ok(None),
        };
        // Exact degree elevation preserves the authored local parameter, but
        // carrying its redundant higher-degree basis into rational root-image
        // transport can force dozens of unnecessary isolator refinements.
        // Recover an exact linear homogeneous representative only for the
        // allocation-light all-rational case. Every inverse-elevation step is
        // replayed below; mixed and symbolic Real carriers retain the general
        // certified path unchanged.
        let reduced_other = other.exact_linear_homogeneous_representative(policy)?;
        let parameter_curve = reduced_other.as_ref().unwrap_or(other);
        let other_basis = parameter_curve.homogeneous_power_basis()?;
        let Some(substituted) = substitute_implicit_conic(conic, other_basis) else {
            return Ok(None);
        };
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(substituted, policy) {
            Ok(Classification::Decided(polynomial)) => polynomial,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
            Err(CurveError::InvalidBezierPolynomial) => return Ok(None),
            Err(error) => return Err(error),
        };
        let other_parameters = match polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        if other_parameters.is_empty() {
            return Ok(Some(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )));
        }
        let simple_roots = polynomial.simple_root_classifications(&other_parameters, policy)?;
        let parameter_map = match conic_parameter_map(self, parameter_curve, policy)? {
            Classification::Decided(parameter_map) => parameter_map,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let primary_parameter_candidate = match conic_parameter_candidate(
            polynomial.coefficients(),
            &parameter_map.primary,
            policy,
        )? {
            Classification::Decided(candidate) => candidate,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let mut contacts = Vec::with_capacity(other_parameters.len());
        let common_weight_sign = matches!(
            other.control_weight_sign(),
            Classification::Decided(RealSign::Positive | RealSign::Negative)
        );
        let strict = policy.strict_counterpart();
        for (parameter, simple_root) in other_parameters.iter().zip(simple_roots) {
            // Clearing the implicit equation also retains projective contacts
            // at infinity. Only finite points may enter affine contact replay.
            // Common-sign Bernstein weights certify every root on this unit
            // chart; mixed weights require the original source denominator.
            if !common_weight_sign {
                match signed_coefficients_at_parameter(
                    &other.homogeneous_power_basis()?.weight,
                    parameter,
                    &strict,
                )? {
                    Classification::Decided(RealSign::Zero) => continue,
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                }
            }
            // The quadratic frame is nonsingular and the source denominator
            // is nonzero at this selected contact. Consequently a
            // simple root of the cleared implicit substitution has nonzero
            // directional derivative, which is exactly transversality of the
            // two regular affine images. Multiple or undecided roots retain
            // the existing tangent-based fallback.
            let certified_transverse = matches!(simple_root, Classification::Decided(true));
            let mapped = conic_parameter_from_curve_parameter(
                &parameter_map,
                &primary_parameter_candidate,
                polynomial.coefficients(),
                parameter,
                reduced_other.is_some(),
                policy,
            )?;
            match mapped {
                Classification::Decided(Some(conic_parameter)) => {
                    let point = match parameter {
                        BezierParameter2::Exact(_) => {
                            match exact_contact_point_evidence(other, parameter, policy)? {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(UncertaintyReason::Boundary) => {
                                    return Ok(Some(Classification::Uncertain(
                                        UncertaintyReason::Boundary,
                                    )));
                                }
                                Classification::Uncertain(_) => match exact_contact_point_evidence(
                                    self,
                                    &conic_parameter,
                                    policy,
                                )? {
                                    Classification::Decided(point) => point,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Some(Classification::Uncertain(reason)));
                                    }
                                },
                            }
                        }
                        BezierParameter2::Algebraic(parameter) => CurvePoint2::from(
                            RationalBezierAlgebraicPointImage2::from_parametric_source(
                                other.clone(),
                                parameter.clone(),
                                policy,
                            ),
                        ),
                    };
                    contacts.push(RationalBezierIntersectionContact2 {
                        first_parameter: conic_parameter,
                        second_parameter: parameter.clone(),
                        point,
                        certified_transverse,
                        tangent_cross_sign: None,
                    });
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        }
        if contacts.is_empty() {
            return Ok(Some(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )));
        }
        Ok(Some(Classification::Decided(
            RationalBezierIntersectionContacts2::Contacts(contacts.into()),
        )))
    }

    pub(super) fn exact_linear_homogeneous_representative(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Self>> {
        if self.degree() <= 1
            || self
                .weights()
                .iter()
                .any(|value| value.exact_rational_ref().is_none())
            || self.homogeneous_controls().iter().any(|point| {
                point.x().exact_rational_ref().is_none() || point.y().exact_rational_ref().is_none()
            })
        {
            return Ok(None);
        }
        let reduced = match self.retained_minimal_degree_representative(policy)? {
            Classification::Decided(Some(reduced)) if reduced.degree() == 1 => reduced,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
            Classification::Decided(Some(_)) => return Ok(None),
        };
        Ok(Some(reduced))
    }

    pub(super) fn exact_line_image_intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let line = match other.fit_exact_line_image(policy)? {
            Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => fit,
            Classification::Decided(BezierLineImageFitRelation::NotLine) => return Ok(None),
            Classification::Uncertain(_) => return Ok(None),
        };
        if matches!(
            self.fit_exact_line_image(policy)?,
            Classification::Decided(BezierLineImageFitRelation::Fit(_))
        ) {
            return Ok(None);
        }
        if self.degree() == 2
            && let Some(circle) = self.data.lineage.root.circular_conic.get()
        {
            let (line_dx, line_dy) = line.line().delta();
            let (from_center_x, from_center_y) = line.line().start().delta_from(&circle.center);
            let one = Real::one();
            let start_residual = Real::signed_product_sum(
                [true, true, false],
                [
                    [&from_center_x, &from_center_x],
                    [&from_center_y, &from_center_y],
                    [&circle.radius_squared, &one],
                ],
            );
            let (end_from_center_x, end_from_center_y) =
                line.line().end().delta_from(&circle.center);
            let end_residual = Real::signed_product_sum(
                [true, true, false],
                [
                    [&end_from_center_x, &end_from_center_x],
                    [&end_from_center_y, &end_from_center_y],
                    [&circle.radius_squared, &one],
                ],
            );
            let matches_curve_endpoint = |point: &Point2| {
                [self.start(), self.end()].into_iter().any(|endpoint| {
                    point == endpoint
                        || is_zero(&point.distance_squared(endpoint), policy) == Some(true)
                })
            };
            let point_on_curve = |point: &Point2| {
                matches_curve_endpoint(point)
                    || matches!(
                        self.point_incidence_on_range(point, &crate::CurveParameterRange2::unit(), policy),
                        Ok(Classification::Decided(
                            RationalBezierPointIncidence2::Parameters(parameters)
                        )) if !parameters.is_empty()
                    )
            };
            let start_matches_curve_endpoint = point_on_curve(line.line().start());
            let end_matches_curve_endpoint = point_on_curve(line.line().end());
            let start_value =
                if start_matches_curve_endpoint || is_zero(&start_residual, policy) == Some(true) {
                    Real::zero()
                } else {
                    start_residual.clone()
                };
            let end_value =
                if end_matches_curve_endpoint || is_zero(&end_residual, policy) == Some(true) {
                    Real::zero()
                } else {
                    end_residual
                };
            for (point, line_parameter, radius_x, radius_y, residual, at_start) in [
                (
                    line.line().start(),
                    Real::zero(),
                    &from_center_x,
                    &from_center_y,
                    &start_value,
                    true,
                ),
                (
                    line.line().end(),
                    Real::one(),
                    &end_from_center_x,
                    &end_from_center_y,
                    &end_value,
                    false,
                ),
            ] {
                let radial_direction = Real::dot2_refs([radius_x, radius_y], [&line_dx, &line_dy]);
                let inward_parameter_direction = if at_start {
                    radial_direction.clone()
                } else {
                    -radial_direction.clone()
                };
                let certified_orthogonal = is_zero(&radial_direction, policy) == Some(true)
                    || (is_zero(&(radius_x - radius_y), policy) == Some(true)
                        && is_zero(&(&line_dx + &line_dy), policy) == Some(true))
                    || (is_zero(&(radius_x + radius_y), policy) == Some(true)
                        && is_zero(&(&line_dx - &line_dy), policy) == Some(true))
                    || (is_zero(radius_x, policy) == Some(true)
                        && is_zero(&line_dy, policy) == Some(true))
                    || (is_zero(radius_y, policy) == Some(true)
                        && is_zero(&line_dx, policy) == Some(true));
                let direction_moves_outside = matches!(
                    real_sign(&inward_parameter_direction, policy),
                    Some(RealSign::Positive)
                );
                let mut curve_parameters =
                    [(self.start(), Real::zero()), (self.end(), one.clone())]
                        .into_iter()
                        .filter_map(|(endpoint, parameter)| {
                            (point == endpoint
                                || is_zero(&point.distance_squared(endpoint), policy) == Some(true))
                            .then_some(BezierParameter2::Exact(parameter))
                        })
                        .collect::<Vec<_>>();
                if curve_parameters.is_empty()
                    && let Classification::Decided(RationalBezierPointIncidence2::Parameters(
                        parameters,
                    )) = self.point_incidence_on_range(
                        point,
                        &crate::CurveParameterRange2::unit(),
                        policy,
                    )?
                {
                    curve_parameters = parameters;
                }
                if curve_parameters.is_empty()
                    && let Classification::Decided(Some(parameters)) =
                        quadratic_conic_point_parameters(point, self, policy)
                {
                    for parameter in parameters {
                        let BezierParameter2::Exact(exact) = &parameter else {
                            continue;
                        };
                        if matches!(
                            self.point_at_classified(exact, policy),
                            Classification::Decided(image)
                                if is_zero(&image.distance_squared(point), policy) == Some(true)
                        ) {
                            curve_parameters.push(parameter);
                        }
                    }
                }
                let point_is_on_full_circle = is_zero(residual, policy) == Some(true);
                if (point_is_on_full_circle || !curve_parameters.is_empty())
                    && (certified_orthogonal || direction_moves_outside)
                {
                    if let [curve_parameter] = curve_parameters.as_slice() {
                        return Ok(Some(Classification::Decided(
                            RationalBezierIntersectionContacts2::Contacts(Arc::from([
                                RationalBezierIntersectionContact2 {
                                    first_parameter: curve_parameter.clone(),
                                    second_parameter: BezierParameter2::Exact(line_parameter),
                                    point: CurvePoint2::from(point.clone()),
                                    certified_transverse: direction_moves_outside,
                                    tangent_cross_sign: None,
                                },
                            ])),
                        )));
                    }
                    if point_is_on_full_circle && certified_orthogonal {
                        let hull = self.certified_bounds_classified();
                        if matches!(
                            hull,
                            Classification::Decided(bounds)
                                if matches!(
                                    bounds.contains_point(point, policy),
                                    Classification::Decided(false)
                                )
                        ) {
                            return Ok(Some(Classification::Decided(
                                RationalBezierIntersectionContacts2::NoIntersection,
                            )));
                        }
                    }
                }
            }
            let half_linear =
                Real::dot2_refs([&from_center_x, &from_center_y], [&line_dx, &line_dy]);
            let quadratic = Real::dot2_refs([&line_dx, &line_dy], [&line_dx, &line_dy]);
            let roots = polynomial_roots_in_unit_interval_with_endpoints(
                start_value.clone(),
                Real::from(2_i8) * half_linear,
                quadratic,
                &start_value,
                &end_value,
                policy,
            );
            if let Classification::Decided(roots) = roots {
                let mut replayed = Vec::with_capacity(roots.len());
                for line_parameter in roots {
                    let point = Point2::new(
                        line.line().start().x() + &line_dx * &line_parameter,
                        line.line().start().y() + &line_dy * &line_parameter,
                    );
                    let endpoint_parameters =
                        [(self.start(), Real::zero()), (self.end(), one.clone())]
                            .into_iter()
                            .filter_map(|(endpoint, parameter)| {
                                (point == *endpoint
                                    || is_zero(&point.distance_squared(endpoint), policy)
                                        == Some(true))
                                .then_some(BezierParameter2::Exact(parameter))
                            })
                            .collect::<Vec<_>>();
                    let curve_parameters = if endpoint_parameters.is_empty() {
                        match quadratic_conic_point_parameters(&point, self, policy) {
                            Classification::Decided(Some(parameters)) => parameters,
                            Classification::Decided(None) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Some(Classification::Uncertain(reason)));
                            }
                        }
                    } else {
                        endpoint_parameters
                    };
                    let (radius_x, radius_y) = point.delta_from(&circle.center);
                    let certified_transverse = match is_zero(
                        &Real::dot2_refs([&radius_x, &radius_y], [&line_dx, &line_dy]),
                        policy,
                    ) {
                        Some(value) => !value,
                        None => {
                            return Ok(Some(Classification::Uncertain(
                                UncertaintyReason::RealSign,
                            )));
                        }
                    };
                    for curve_parameter in curve_parameters {
                        replayed.push(RationalBezierIntersectionContact2 {
                            first_parameter: curve_parameter,
                            second_parameter: BezierParameter2::Exact(line_parameter.clone()),
                            point: CurvePoint2::from(point.clone()),
                            certified_transverse,
                            tangent_cross_sign: None,
                        });
                    }
                }
                return Ok(Some(Classification::Decided(if replayed.is_empty() {
                    RationalBezierIntersectionContacts2::NoIntersection
                } else {
                    RationalBezierIntersectionContacts2::Contacts(replayed.into())
                })));
            }
        }
        let supporting_contacts = match self.relation_to_line_with_contacts(line.line(), policy) {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let contacts = match supporting_contacts {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => Vec::new(),
            BezierLineContactRelation::OnSupportingLine => return Ok(None),
            BezierLineContactRelation::Contacts { contacts } => contacts,
        };
        let parameter_graph = [Axis2::X, Axis2::Y].into_iter().find_map(|axis| {
            match other.polynomial_graph(axis, policy).ok()? {
                Classification::Decided(Some(graph)) => Some(graph),
                Classification::Decided(None) | Classification::Uncertain(_) => None,
            }
        });
        let Some(parameter_graph) = parameter_graph else {
            return Ok(None);
        };
        let basis = self.homogeneous_power_basis()?;
        let axis_numerator = match parameter_graph.axis {
            Axis2::X => &basis.x_numerator,
            Axis2::Y => &basis.y_numerator,
        };
        let parameter_numerator = subtract_power_polynomials(
            axis_numerator,
            &scale_power_polynomial(&basis.weight, &parameter_graph.origin),
        );
        let parameter_denominator = scale_power_polynomial(&basis.weight, &parameter_graph.scale);
        let mut replayed = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let source_parameter = contact.parameter().clone();
            let parameter = match source_parameter
                .clone()
                .promote_represented_exact_point(policy)?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(_) => source_parameter,
            };
            let Some(parameter_value) = parameter.scalar() else {
                let root = parameter_root_representation(&parameter, policy);
                let candidate = match conic_parameter_candidate(
                    &root.polynomial_coefficients,
                    &(parameter_numerator.clone(), parameter_denominator.clone()),
                    policy,
                )? {
                    Classification::Decided(candidate) => candidate,
                    Classification::Uncertain(_) => return Ok(None),
                };
                let mapped = match rational_image_parameter(&root, &candidate, policy)? {
                    Classification::Decided(mapped) => Classification::Decided(mapped),
                    Classification::Uncertain(_) => {
                        real_coefficient_rational_image_parameter(&parameter, &candidate, policy)?
                    }
                };
                let Classification::Decided(mapped) = mapped else {
                    return Ok(None);
                };
                let Some(mapped) = mapped else {
                    continue;
                };
                let point = match exact_contact_point_evidence(other, &mapped, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(UncertaintyReason::Boundary) => return Ok(None),
                    Classification::Uncertain(_) => {
                        match exact_contact_point_evidence(self, &parameter, policy)? {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(_) => return Ok(None),
                        }
                    }
                };
                replayed.push(RationalBezierIntersectionContact2 {
                    first_parameter: parameter,
                    second_parameter: mapped,
                    point,
                    certified_transverse: contact.kind() == BezierLineContactKind::Crossing,
                    tangent_cross_sign: None,
                });
                continue;
            };
            let point = match self.point_at_classified(parameter_value, policy) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let other_parameter = if other.exact_linear_parameterization_line().is_some() {
                let coordinate = match parameter_graph.axis {
                    Axis2::X => point.x(),
                    Axis2::Y => point.y(),
                };
                let mapped = ((coordinate - &parameter_graph.origin) / &parameter_graph.scale)?;
                match in_closed_unit_interval(&mapped, policy) {
                    Some(true) => BezierParameter2::Exact(mapped),
                    Some(false) => continue,
                    None => {
                        return Ok(Some(Classification::Uncertain(UncertaintyReason::Ordering)));
                    }
                }
            } else {
                match unique_point_incidence_parameter(other, &point, policy) {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => {
                        return Ok(Some(Classification::Uncertain(
                            UncertaintyReason::Predicate,
                        )));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                }
            };
            replayed.push(RationalBezierIntersectionContact2 {
                first_parameter: parameter,
                second_parameter: other_parameter,
                point: CurvePoint2::from(point),
                certified_transverse: contact.kind() == BezierLineContactKind::Crossing,
                tangent_cross_sign: None,
            });
        }
        Ok(Some(Classification::Decided(if replayed.is_empty() {
            RationalBezierIntersectionContacts2::NoIntersection
        } else {
            RationalBezierIntersectionContacts2::Contacts(replayed.into())
        })))
    }

    pub(super) fn circular_conic_intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
        circle_relation: Option<&CircleCircleRelation>,
        first_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
        second_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        for curve in [self, other] {
            if curve.data.lineage.root.circular_conic.get().is_some() {
                continue;
            }
            // Authored and trimmed conics may reach this query without a
            // retained circle certificate. Recognize their support once;
            // only certified recognition may enrich the shared root cache.
            let Classification::Decided(Some(arc)) = policy.strict_predicate_pass(|| {
                crate::arc_bezier::rational_bezier_circular_arc(curve, policy)
            })?
            else {
                return Ok(None);
            };
            let (implicit, circular) = crate::arc_bezier::circular_conic_provenance(&arc);
            let _ = curve
                .data
                .lineage
                .root
                .implicit_quadratic_conic
                .set(implicit);
            let _ = curve.data.lineage.root.circular_conic.set(circular);
        }
        let first = self.data.lineage.root.circular_conic.get().unwrap();
        let second = other.data.lineage.root.circular_conic.get().unwrap();
        let computed_relation;
        let circle_relation = match circle_relation {
            Some(relation) => relation,
            None => {
                computed_relation = circle_relation_from_supports(
                    &first.center,
                    &first.radius_squared,
                    &second.center,
                    &second.radius_squared,
                    policy,
                )?;
                &computed_relation
            }
        };
        let (points, certified_transverse) = match circle_relation {
            CircleCircleRelation::Coincident => return Ok(None),
            CircleCircleRelation::Disjoint => {
                return Ok(Some(Classification::Decided(
                    RationalBezierIntersectionContacts2::NoIntersection,
                )));
            }
            CircleCircleRelation::Tangent { point } => (vec![point.clone()], false),
            CircleCircleRelation::Secant {
                first_point,
                second_point,
            } => (vec![first_point.clone(), second_point.clone()], true),
            CircleCircleRelation::Uncertain { reason } => {
                return Ok(Some(Classification::Uncertain(*reason)));
            }
        };
        let mut contacts = Vec::with_capacity(points.len());
        for (point_index, point) in points.into_iter().enumerate() {
            let first_parameters =
                match first_circle_parameters.and_then(|parameters| parameters.get(point_index)) {
                    Some(Classification::Decided(parameters)) => Arc::clone(parameters),
                    Some(Classification::Uncertain(reason)) => {
                        return Ok(Some(Classification::Uncertain(*reason)));
                    }
                    None => match self.retained_circle_point_parameters(&point, policy)? {
                        Classification::Decided(parameters) => Arc::from(parameters),
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    },
                };
            if first_parameters.is_empty() {
                continue;
            }
            let second_parameters =
                match second_circle_parameters.and_then(|parameters| parameters.get(point_index)) {
                    Some(Classification::Decided(parameters)) => Arc::clone(parameters),
                    Some(Classification::Uncertain(reason)) => {
                        return Ok(Some(Classification::Uncertain(*reason)));
                    }
                    None => match other.retained_circle_point_parameters(&point, policy)? {
                        Classification::Decided(parameters) => Arc::from(parameters),
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    },
                };
            if second_parameters.is_empty() {
                continue;
            }
            for first_parameter in first_parameters.iter() {
                for second_parameter in second_parameters.iter() {
                    contacts.push(RationalBezierIntersectionContact2 {
                        first_parameter: first_parameter.clone(),
                        second_parameter: second_parameter.clone(),
                        point: CurvePoint2::from(point.clone()),
                        certified_transverse,
                        tangent_cross_sign: None,
                    });
                }
            }
        }
        Ok(Some(Classification::Decided(if contacts.is_empty() {
            RationalBezierIntersectionContacts2::NoIntersection
        } else {
            RationalBezierIntersectionContacts2::Contacts(contacts.into())
        })))
    }

    pub(super) fn replay_intersection_candidate_set(
        &self,
        other: &Self,
        candidates: &CurveIntersectionCandidates2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        match candidates {
            CurveIntersectionCandidates2::NoIntersection => Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )),
            CurveIntersectionCandidates2::DegenerateResultant => {
                match self.image_overlap(other, policy) {
                    Classification::Decided(RationalBezierSharedComponentReplay::Overlap(
                        overlap,
                    )) => Ok(Classification::Decided(
                        RationalBezierIntersectionContacts2::Overlap(overlap),
                    )),
                    Classification::Decided(RationalBezierSharedComponentReplay::Contacts(
                        contacts,
                    )) => {
                        // Endpoint replay is complete only when the shared
                        // component cannot cross itself between the arcs: a
                        // nondegenerate conic is smooth, and otherwise the
                        // union must be injective. Both branches through a
                        // node may lie on different individually injective
                        // arcs.
                        let node_free = self.has_certified_injective_union(other, true, policy)
                            || matches!(
                                self.shares_implicit_quadratic_conic(other, policy),
                                Classification::Decided(true)
                            );
                        if !node_free {
                            return Ok(Classification::Decided(
                                RationalBezierIntersectionContacts2::DegenerateResultant,
                            ));
                        }
                        let mut replayed = Vec::with_capacity(contacts.len());
                        for (first_parameter, second_parameter) in contacts {
                            let point = match self.point_at_classified(&first_parameter, policy) {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                            replayed.push(RationalBezierIntersectionContact2 {
                                first_parameter: BezierParameter2::Exact(first_parameter),
                                second_parameter: BezierParameter2::Exact(second_parameter),
                                point: CurvePoint2::from(point),
                                certified_transverse: false,
                                tangent_cross_sign: None,
                            });
                        }
                        Ok(Classification::Decided(if replayed.is_empty() {
                            RationalBezierIntersectionContacts2::NoIntersection
                        } else {
                            RationalBezierIntersectionContacts2::Contacts(replayed.into())
                        }))
                    }
                    Classification::Decided(RationalBezierSharedComponentReplay::Unresolved) => {
                        Ok(Classification::Decided(
                            RationalBezierIntersectionContacts2::DegenerateResultant,
                        ))
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
            CurveIntersectionCandidates2::Candidates {
                first_parameters,
                second_parameters,
            } => self.replay_intersection_candidates(
                other,
                first_parameters,
                second_parameters,
                policy,
            ),
        }
    }

    pub(crate) fn intersection_candidates_classified(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveIntersectionCandidates2>> {
        // Same-sign control-hull bounds are only a rejection accelerator. An
        // unavailable sign or ordering certificate must fall through to the
        // homogeneous resultant, whose affine replay independently rejects
        // projective poles and out-of-domain roots.
        if self.certified_bounds_are_disjoint(other, policy) {
            return Ok(Classification::Decided(
                CurveIntersectionCandidates2::NoIntersection,
            ));
        }

        self.intersection_candidates_after_bounds_check(other, policy)
    }

    pub(super) fn certified_bounds_are_disjoint(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> bool {
        let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) = (
            self.certified_bounds_classified(),
            other.certified_bounds_classified(),
        ) else {
            return false;
        };
        matches!(
            first_bounds.overlaps(&second_bounds, policy),
            Classification::Decided(false)
        )
    }

    pub(super) fn intersection_candidates_after_bounds_check(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveIntersectionCandidates2>> {
        match self.lineage_overlap(other, policy) {
            Classification::Decided(Some(_)) => {
                return Ok(Classification::Decided(
                    CurveIntersectionCandidates2::DegenerateResultant,
                ));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        for reversed in [false, true] {
            if self.same_projective_control_net(other, reversed, policy) == Some(true) {
                return Ok(Classification::Decided(
                    CurveIntersectionCandidates2::DegenerateResultant,
                ));
            }
        }
        let config = CurveIntersectionResultantConfig {
            min_precision: RATIONAL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_RATIONAL_INTERSECTION_RESULTANT_DEGREE,
        };
        let first = resultant_rational_parametric_curve_intersection_complete(
            self.homogeneous_power_basis()?,
            other.homogeneous_power_basis()?,
            CurveResultantParameter::First,
            config,
        );
        let second = resultant_rational_parametric_curve_intersection_complete(
            self.homogeneous_power_basis()?,
            other.homogeneous_power_basis()?,
            CurveResultantParameter::Second,
            config,
        );
        let first = match resultant_parameter_projection(
            first,
            CurveParameterDomain2::new(&CurveParameterRange2::unit(), None),
            policy,
        )? {
            Classification::Decided(projection) => projection,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match resultant_parameter_projection(
            second,
            CurveParameterDomain2::new(&CurveParameterRange2::unit(), None),
            policy,
        )? {
            Classification::Decided(projection) => projection,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(match (first, second) {
            (ResultantParameterProjection::Empty, _) | (_, ResultantParameterProjection::Empty) => {
                CurveIntersectionCandidates2::NoIntersection
            }
            (ResultantParameterProjection::Degenerate, _)
            | (_, ResultantParameterProjection::Degenerate) => {
                CurveIntersectionCandidates2::DegenerateResultant
            }
            (
                ResultantParameterProjection::Parameters(first_parameters)
                | ResultantParameterProjection::SelectedParameters(first_parameters),
                ResultantParameterProjection::Parameters(second_parameters)
                | ResultantParameterProjection::SelectedParameters(second_parameters),
            ) => CurveIntersectionCandidates2::Candidates {
                first_parameters,
                second_parameters,
            },
        }))
    }
}
