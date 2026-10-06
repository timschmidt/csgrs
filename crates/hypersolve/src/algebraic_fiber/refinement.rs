use super::*;

/// Reusable exact refinement authority for roots of one selected algebraic
/// fiber. The retained field, coefficient signs, inverses, and any required
/// Sturm sequence survive subsequent requests and can serve different roots
/// of the same fiber. Simple roots keep the division-free Bernstein path.
pub struct AlgebraicFiberRootRefiner {
    field: LocalAlgebraicField,
    polynomial: Vec<LocalFieldElement>,
    sequence: Option<Vec<Vec<LocalFieldElement>>>,
}

impl std::fmt::Debug for AlgebraicFiberRootRefiner {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("AlgebraicFiberRootRefiner")
            .field("fiber_degree", &self.polynomial.len().saturating_sub(1))
            .field(
                "sturm_sequence_length",
                &self.sequence.as_ref().map(Vec::len),
            )
            .finish()
    }
}

impl AlgebraicFiberRootRefiner {
    /// Admits a selected coefficient field and specializes the fiber once.
    /// Returns invalid or undecided evidence without caching a failed admission.
    pub fn try_new(
        polynomial: &BivariatePolynomial,
        retained_parameter: CurveResultantParameter,
        retained_root: &AlgebraicRootRepresentation,
        policy: PredicatePolicy,
    ) -> Result<Self, AlgebraicFiberRootIsolationReport> {
        let mut field = LocalAlgebraicField::new(retained_root, policy)
            .map_err(|error| fiber_root_isolation_error_report(error, Certainty::Exact))?;
        let polynomial = local_fiber_polynomial(polynomial, retained_parameter, &mut field)
            .map_err(|error| {
                fiber_root_isolation_error_report_with_progress(error, 0, 0, &field)
            })?;
        Ok(Self {
            field,
            polynomial,
            sequence: None,
        })
    }

    /// Requests `steps` additional dyadic refinements after certifying singleton
    /// ownership, stopping at an exact point witness if found. Nondegenerate
    /// input bounds must not be roots of this fiber. A caller's root-count field
    /// alone is not a proof; ownership is replayed against the retained fiber.
    pub fn refine(
        &mut self,
        root: &IsolatedRootInterval,
        steps: usize,
    ) -> AlgebraicFiberRootIsolationReport {
        let mut subdivisions = 0;
        let result = self.refine_interval(root, steps, &mut subdivisions);
        let sequence_length = self.sequence.as_ref().map_or(0, Vec::len);
        match result {
            Ok(interval) => AlgebraicFiberRootIsolationReport {
                status: AlgebraicFiberRootIsolationStatus::Isolated,
                intervals: vec![interval],
                sturm_sequence_length: sequence_length,
                subdivision_steps: subdivisions,
                retained_refinement_steps: self.field.refinement_steps,
                certainty: self.field.certainty,
                message: None,
            },
            Err(error) => fiber_root_isolation_error_report_with_progress(
                error,
                sequence_length,
                subdivisions,
                &self.field,
            ),
        }
    }
}

impl AlgebraicFiberRootRefiner {
    fn refine_interval(
        &mut self,
        root: &IsolatedRootInterval,
        steps: usize,
        subdivisions: &mut usize,
    ) -> Result<IsolatedRootInterval, LocalFieldError> {
        if root.distinct_root_count != 1 {
            return Err(LocalFieldError::InvalidEvidence);
        }
        if local_polynomial_is_zero(&self.polynomial, &mut self.field)? {
            return Err(LocalFieldError::InvalidEvidence);
        }
        if let Some(exact) = &root.exact_root {
            if self.field.compare(&root.lower, exact)? != Ordering::Equal
                || self.field.compare(&root.upper, exact)? != Ordering::Equal
                || local_polynomial_sign_at(&self.polynomial, exact, &mut self.field)?
                    != Ordering::Equal
            {
                return Err(LocalFieldError::InvalidEvidence);
            }
            return Ok(root.clone());
        }
        if self.field.compare(&root.lower, &root.upper)? != Ordering::Less {
            return Err(LocalFieldError::InvalidInterval);
        }
        for endpoint in [&root.lower, &root.upper] {
            if local_polynomial_sign_at(&self.polynomial, endpoint, &mut self.field)?
                == Ordering::Equal
            {
                return Err(LocalFieldError::InvalidEvidence);
            }
        }
        if self.sequence.is_none() {
            match isolate_local_polynomial_roots_bernstein(
                self.polynomial.clone(),
                &root.lower,
                &root.upper,
                AlgebraicFiberRootIsolationConfig {
                    max_subdivision_depth: steps.saturating_add(256),
                    refinement_steps: steps,
                },
                &mut self.field,
            ) {
                Ok((Some(mut intervals), count)) => {
                    *subdivisions = subdivisions.saturating_add(count);
                    if intervals.len() != 1 {
                        return Err(LocalFieldError::InvalidEvidence);
                    }
                    return Ok(intervals.pop().expect("one certified fiber root"));
                }
                Ok((None, count)) => *subdivisions = subdivisions.saturating_add(count),
                Err(LocalFieldError::Undecided) => {}
                Err(error) => return Err(error),
            }
            self.sequence = Some(local_sturm_sequence(
                self.polynomial.clone(),
                &mut self.field,
            )?);
        }
        let sequence = self.sequence.as_ref().expect("a prepared Sturm sequence");
        let mut lower = root.lower.clone();
        let mut upper = root.upper.clone();
        let mut lower_variations =
            local_sturm_boundary_variations(sequence, &lower, &mut self.field)?
                .ok_or(LocalFieldError::InvalidEvidence)?;
        let mut upper_variations =
            local_sturm_boundary_variations(sequence, &upper, &mut self.field)?
                .ok_or(LocalFieldError::InvalidEvidence)?;
        if lower_variations.checked_sub(upper_variations) != Some(1) {
            return Err(LocalFieldError::InvalidEvidence);
        }
        for _ in 0..steps {
            let midpoint =
                ((&lower + &upper) / Real::from(2_u8)).map_err(|_| LocalFieldError::Undecided)?;
            *subdivisions = subdivisions.saturating_add(1);
            if local_polynomial_sign_at(&self.polynomial, &midpoint, &mut self.field)?
                == Ordering::Equal
            {
                return Ok(IsolatedRootInterval {
                    lower: midpoint.clone(),
                    upper: midpoint.clone(),
                    exact_root: Some(midpoint),
                    distinct_root_count: 1,
                });
            }
            let variations = local_sturm_boundary_variations(sequence, &midpoint, &mut self.field)?
                .ok_or(LocalFieldError::Undecided)?;
            match (
                lower_variations.checked_sub(variations),
                variations.checked_sub(upper_variations),
            ) {
                (Some(1), Some(0)) => {
                    upper = midpoint;
                    upper_variations = variations;
                }
                (Some(0), Some(1)) => {
                    lower = midpoint;
                    lower_variations = variations;
                }
                _ => return Err(LocalFieldError::Undecided),
            }
        }
        Ok(IsolatedRootInterval {
            lower,
            upper,
            exact_root: None,
            distinct_root_count: 1,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::represented_root;
    use super::*;

    fn interval(lower: i64, upper: i64) -> IsolatedRootInterval {
        IsolatedRootInterval {
            lower: Real::from(lower),
            upper: Real::from(upper),
            exact_root: None,
            distinct_root_count: 1,
        }
    }

    #[test]
    fn repeated_root_refinement_reuses_one_sequence_across_roots_and_requests() {
        // (u^2-alpha)^2 has two repeated roots, one on either side of zero.
        let coefficients = vec![
            vec![
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ],
            vec![Real::zero(), Real::zero(), Real::from(-2)],
            vec![Real::one()],
        ];
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![Real::from(-2), Real::zero(), Real::one()],
                Real::one(),
                Real::from(2),
                policy,
            );
            for axis in [
                CurveResultantParameter::First,
                CurveResultantParameter::Second,
            ] {
                let polynomial = BivariatePolynomial::new(match axis {
                    CurveResultantParameter::First => coefficients.clone(),
                    CurveResultantParameter::Second => (0..5)
                        .map(|i| {
                            coefficients
                                .iter()
                                .map(|row| row.get(i).cloned().unwrap_or_else(Real::zero))
                                .collect()
                        })
                        .collect(),
                });
                let mut refiner =
                    AlgebraicFiberRootRefiner::try_new(&polynomial, axis, &alpha, policy).unwrap();
                let mut sequence_identity = None;
                for source in [interval(1, 2), interval(-2, -1), interval(1, 2)] {
                    let first = refiner.refine(&source, 4);
                    assert_eq!(first.status, AlgebraicFiberRootIsolationStatus::Isolated);
                    assert_eq!(first.certainty, Certainty::Exact);
                    let pointer = refiner.sequence.as_ref().unwrap().as_ptr();
                    if let Some(previous) = sequence_identity {
                        assert_eq!(previous, pointer);
                    }
                    sequence_identity = Some(pointer);
                    let second = refiner.refine(&first.intervals[0], 8);
                    assert_eq!(second.status, AlgebraicFiberRootIsolationStatus::Isolated);
                    assert_eq!(second.certainty, Certainty::Exact);
                    let refined = &second.intervals[0];
                    assert!(refined.lower >= first.intervals[0].lower);
                    assert!(refined.upper <= first.intervals[0].upper);
                    assert!(
                        &refined.upper - &refined.lower
                            < &first.intervals[0].upper - &first.intervals[0].lower
                    );
                    let replay = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
                        &polynomial,
                        axis,
                        &alpha,
                        &refined.lower,
                        &refined.upper,
                        PredicatePolicy::STRICT,
                    );
                    assert_eq!(replay.distinct_root_count, Some(1));
                    assert_eq!(pointer, refiner.sequence.as_ref().unwrap().as_ptr());
                }
                for forged in [interval(-2, 2), interval(2, 3)] {
                    assert_eq!(
                        refiner.refine(&forged, 3).status,
                        AlgebraicFiberRootIsolationStatus::InvalidEvidence
                    );
                }
            }
        }
    }

    #[test]
    fn simple_refinement_preserves_exact_base_coefficients_without_sturm() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![-Real::from(2).sqrt().unwrap(), Real::zero(), Real::one()],
                Real::one(),
                Real::from(2),
                policy,
            );
            let polynomial = BivariatePolynomial::new(vec![
                vec![Real::zero(), Real::one()],
                vec![Real::from(-1)],
            ]);
            let mut refiner = AlgebraicFiberRootRefiner::try_new(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                policy,
            )
            .unwrap();
            let refined = refiner.refine(&interval(1, 2), 12);
            assert_eq!(refined.status, AlgebraicFiberRootIsolationStatus::Isolated);
            assert_eq!(refined.certainty, Certainty::Exact);
            assert_eq!(refined.sturm_sequence_length, 0);
            assert!(refiner.sequence.is_none());
        }
    }

    #[test]
    fn refiner_replays_point_witnesses_and_rejects_stale_or_ambiguous_ownership() {
        let policy = PredicatePolicy::STRICT;
        let alpha = represented_root(
            vec![Real::from(-2), Real::zero(), Real::one()],
            Real::one(),
            Real::from(2),
            policy,
        );
        let polynomial = BivariatePolynomial::new(vec![vec![Real::from(-1), Real::one()]]);
        let mut refiner = AlgebraicFiberRootRefiner::try_new(
            &polynomial,
            CurveResultantParameter::First,
            &alpha,
            policy,
        )
        .unwrap();
        let witness = IsolatedRootInterval {
            lower: Real::one(),
            upper: Real::one(),
            exact_root: Some(Real::one()),
            distinct_root_count: 1,
        };
        assert_eq!(
            refiner.refine(&witness, 16).intervals,
            vec![witness.clone()]
        );
        for invalid in [
            interval(1, 2),
            interval(0, 1),
            interval(2, 3),
            IsolatedRootInterval {
                lower: Real::zero(),
                ..witness.clone()
            },
        ] {
            assert_eq!(
                refiner.refine(&invalid, 4).status,
                AlgebraicFiberRootIsolationStatus::InvalidEvidence
            );
        }
        let mut zero = AlgebraicFiberRootRefiner::try_new(
            &BivariatePolynomial::new(vec![vec![Real::zero()]]),
            CurveResultantParameter::First,
            &alpha,
            policy,
        )
        .unwrap();
        assert_eq!(
            zero.refine(&witness, 0).status,
            AlgebraicFiberRootIsolationStatus::InvalidEvidence
        );
        let mut stale = alpha;
        stale.polynomial_coefficients[2] = Real::zero();
        let error = AlgebraicFiberRootRefiner::try_new(
            &polynomial,
            CurveResultantParameter::First,
            &stale,
            policy,
        )
        .unwrap_err();
        assert_eq!(
            error.status,
            AlgebraicFiberRootIsolationStatus::InvalidEvidence
        );
    }
}
