//! Exact parallel construction: hodograph offsets, singularities and spans.

use super::*;

/// Exact rational parallel certified from a homogeneous Pythagorean hodograph.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedPythagoreanHodographOffset2 {
    pub(in crate::bezier_offset) curve: RationalBezier2,
    pub(in crate::bezier_offset) speed_polynomial: Arc<BezierParameterPolynomial>,
    pub(in crate::bezier_offset) source_degree: usize,
    pub(in crate::bezier_offset) distance: Real,
}

/// Exact rational image selected by one regular analytic-parallel range.
///
/// `support_line` is only an infinite-support certificate for a line-valued
/// branch. The rational curve retains its original global parameterization
/// and may retrace or extend beyond that finite witness segment.
#[derive(Debug)]
pub(crate) struct BezierParallelRationalComponent2 {
    pub(in crate::bezier_offset) curve: RationalBezier2,
    pub(in crate::bezier_offset) support_line: Option<LineSeg2>,
    pub(in crate::bezier_offset) regular_range: CurveParameterRange2,
}

impl BezierParallelRationalComponent2 {
    pub(crate) const fn curve(&self) -> &RationalBezier2 {
        &self.curve
    }

    pub(crate) const fn support_line(&self) -> Option<&LineSeg2> {
        self.support_line.as_ref()
    }

    pub(crate) const fn regular_range(&self) -> &CurveParameterRange2 {
        &self.regular_range
    }
}

impl CertifiedPythagoreanHodographOffset2 {
    /// Reuses the certified root-free unit speed sheet on its original chart.
    /// Other finite ranges first try Bernstein bounds, then exact root
    /// isolation. An incident extension keeps its own open barrier and sign.
    pub(super) fn speed_sign_on_domain(
        &self,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        policy.strict_predicate_pass(|| {
            let polynomial = self.speed_polynomial.as_ref();
            let coefficients = polynomial.coefficients();
            let sign = if domain.finite == &CurveParameterRange2::unit() {
                // Every PH constructor proves that this polynomial has no unit
                // root. Read its sheet sign without repeating that isolation.
                real_sign(&coefficients[0], policy)
            } else if let Some(sign) =
                strict_polynomial_sign_on_curve_region_range(coefficients, domain.finite, policy)?
            {
                Some(sign)
            } else {
                match domain.finite_roots(polynomial, policy)? {
                    Classification::Decided(roots) if roots.is_empty() => {}
                    Classification::Decided(_) | Classification::Uncertain(_) => return Ok(None),
                }
                let interior = match domain.finite.strict_interior_scalar(policy)? {
                    Classification::Decided(interior) => interior,
                    Classification::Uncertain(_) => return Ok(None),
                };
                real_sign(&polynomial.evaluate(&interior), policy)
            };
            let Some(sign @ (RealSign::Positive | RealSign::Negative)) = sign else {
                return Ok(None);
            };
            let Some(extension) = domain.extension else {
                return Ok(Some(sign));
            };
            if real_sign(&Real::eval_poly(coefficients, extension.anchor), policy) != Some(sign) {
                return Ok(None);
            }
            if coefficients.len() == 1 {
                return Ok(Some(sign));
            }
            let roots = match polynomial.isolate_incident_ray_roots(
                extension.anchor,
                extension.direction,
                policy,
            )? {
                Classification::Decided(roots) => roots,
                Classification::Uncertain(_) => return Ok(None),
            };
            for root in roots {
                let Some(barrier) = extension.barrier else {
                    return Ok(None);
                };
                let order = match root.cmp_by_refinement(barrier, policy)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(_) => return Ok(None),
                };
                let before_barrier = match extension.direction {
                    BezierParameterRayDirection2::Increasing => order == std::cmp::Ordering::Less,
                    BezierParameterRayDirection2::Decreasing => {
                        order == std::cmp::Ordering::Greater
                    }
                };
                if before_barrier {
                    return Ok(None);
                }
            }
            Ok(Some(sign))
        })
    }

    /// Returns the exact rational Bezier carrying the parallel image.
    pub const fn curve(&self) -> &RationalBezier2 {
        &self.curve
    }

    /// Returns `sigma`, where the homogeneous tangent numerator satisfies `H dot H = sigma^2`.
    pub fn speed_polynomial(&self) -> &[Real] {
        self.speed_polynomial.coefficients()
    }

    /// Returns the homogeneous source degree.
    pub const fn source_degree(&self) -> usize {
        self.source_degree
    }

    /// Returns the homogeneous degree of the exact rational parallel.
    pub fn rational_degree(&self) -> usize {
        self.curve.degree()
    }

    /// Returns the signed left-offset distance.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }
}

/// Exact singularity evidence for a retained Bezier parallel.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParallelSingularityAnalysis2 {
    pub(in crate::bezier_offset) range: CurveParameterRange2,
    pub(in crate::bezier_offset) source_singularities: Vec<BezierParameter2>,
    pub(in crate::bezier_offset) parallel_cusps: Vec<BezierParameter2>,
    pub(in crate::bezier_offset) source_speed_squared_degree: usize,
    pub(in crate::bezier_offset) parallel_cusp_polynomial_degree: Option<usize>,
}

impl BezierParallelSingularityAnalysis2 {
    /// Returns the closed exact range covered by this analysis.
    pub const fn range(&self) -> &CurveParameterRange2 {
        &self.range
    }

    /// Returns every isolated parameter where the source derivative vanishes.
    pub fn source_singularities(&self) -> &[BezierParameter2] {
        &self.source_singularities
    }

    /// Returns every isolated regular-source parameter where the parallel derivative vanishes.
    pub fn parallel_cusps(&self) -> &[BezierParameter2] {
        &self.parallel_cusps
    }

    /// Partitions this exact range into increasing cells with regular interiors. Singular
    /// endpoints stay in their original authority; consumers choose the
    /// appropriate one-sided frame. Both root inventories are already ordered.
    pub(crate) fn regular_subranges(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveParameterRange2>>> {
        let [lower, upper] = match self.range.ordered_endpoints(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut sources = self.source_singularities.iter().peekable();
        let mut cusps = self.parallel_cusps.iter().peekable();
        let mut start = lower.clone();
        let mut ranges = Vec::with_capacity(sources.len() + cusps.len() + 1);
        loop {
            let boundary = match (sources.peek(), cusps.peek()) {
                (Some(source), Some(cusp)) => match source.cmp_by_refinement(cusp, policy)? {
                    Classification::Decided(std::cmp::Ordering::Less) => sources.next(),
                    Classification::Decided(std::cmp::Ordering::Greater) => cusps.next(),
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        cusps.next();
                        sources.next()
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
                (Some(_), None) => sources.next(),
                (None, Some(_)) => cusps.next(),
                (None, None) => break,
            };
            let boundary = CurveParameter2::from(boundary.unwrap().clone());
            match boundary.cmp_by_refinement(&start, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => continue,
                Classification::Decided(std::cmp::Ordering::Greater) => {}
                Classification::Decided(std::cmp::Ordering::Less) => {
                    return Err(CurveError::Topology(
                        "parallel singularities are not ordered inside their range".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            match boundary.cmp_by_refinement(upper, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => break,
                Classification::Decided(std::cmp::Ordering::Less) => {}
                Classification::Decided(std::cmp::Ordering::Greater) => {
                    return Err(CurveError::Topology(
                        "parallel singularity lies outside its range".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            ranges.push(CurveParameterRange2::new_validated(start, boundary.clone()));
            start = boundary;
        }
        ranges.push(CurveParameterRange2::new_validated(start, upper.clone()));
        Ok(Classification::Decided(ranges))
    }

    /// Returns whether the source normal is defined over the requested closed range.
    pub fn source_is_regular(&self) -> bool {
        self.source_singularities.is_empty()
    }

    /// Returns whether the parallel is cusp-free on every regular source span in the range.
    pub fn parallel_is_cusp_free(&self) -> bool {
        self.parallel_cusps.is_empty()
    }

    /// Returns the degree of the exact source speed-squared polynomial.
    pub const fn source_speed_squared_degree(&self) -> usize {
        self.source_speed_squared_degree
    }

    /// Returns the degree of the squared parallel-cusp polynomial when nonconstant.
    pub const fn parallel_cusp_polynomial_degree(&self) -> Option<usize> {
        self.parallel_cusp_polynomial_degree
    }
}

/// Solves polynomial-quadratic parallel cusps in the canonical scalar tower.
///
/// A quadratic source has affine tangent, quadratic speed squared `S(t)`, and
/// constant signed curvature term `K=d(P'' x P')`.  On a regular source the
/// selected cusp equation is `K + S(t)^(3/2)=0`.  It has no solution for
/// nonnegative `K`; for negative `K` it reduces exactly to the quadratic
/// `S(t)-cuberoot(K^2)=0`.  Solving that equation directly avoids retaining the
/// degree-six polynomial introduced by squaring.  The linear/quadratic formula
/// is itself the exact construction certificate; it does not ask a generic
/// scalar equality predicate to rediscover cancellation in nested radicals.
/// If its degree, discriminant sign, or exact range membership is unresolved,
/// the caller retains the complete algebraic isolator.
pub(super) fn exact_quadratic_parallel_cusp_candidates(
    speed_squared: &[Real],
    signed_curvature_term: &[Real],
    policy: &CurveContext,
) -> CurveResult<Option<Vec<Real>>> {
    let curvature = polynomial_trim_structural_zeros(signed_curvature_term.to_vec());
    let [curvature] = curvature.as_slice() else {
        return Ok(None);
    };
    match real_sign(curvature, policy) {
        Some(RealSign::Positive | RealSign::Zero) => return Ok(Some(Vec::new())),
        Some(RealSign::Negative) => {}
        None => return Ok(None),
    }
    let target_speed_squared = (curvature * curvature).root_n(3)?;
    let mut equation = speed_squared.to_vec();
    equation[0] = &equation[0] - target_speed_squared;
    let equation = polynomial_trim_structural_zeros(equation);
    let candidates = match equation.as_slice() {
        [constant] => match real_sign(constant, policy) {
            Some(RealSign::Positive | RealSign::Negative) => Vec::new(),
            Some(RealSign::Zero) | None => return Ok(None),
        },
        [constant, linear] => match real_sign(linear, policy) {
            Some(RealSign::Positive | RealSign::Negative) => {
                vec![((-constant.clone()) / linear)?]
            }
            Some(RealSign::Zero) | None => return Ok(None),
        },
        [constant, linear, quadratic] => {
            match real_sign(quadratic, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) | None => return Ok(None),
            }
            let discriminant = linear * linear - Real::from(4_i8) * quadratic * constant;
            match real_sign(&discriminant, policy) {
                Some(RealSign::Negative) => Vec::new(),
                Some(RealSign::Zero) => {
                    vec![((-linear.clone()) / (Real::from(2_i8) * quadratic))?]
                }
                Some(RealSign::Positive) => {
                    let denominator = Real::from(2_i8) * quadratic;
                    let vertex = ((-linear.clone()) / &denominator)?;
                    let delta = (discriminant / (&denominator * &denominator))?.sqrt()?;
                    vec![&vertex - &delta, vertex + delta]
                }
                None => return Ok(None),
            }
        }
        _ => return Ok(None),
    };
    // These represented values carry the selected unsquared cusp identity.
    // The consuming range owns admission; pointwise derivative replay needs
    // the identity without discovering roots on an unrelated interval.
    Ok(Some(candidates))
}

impl QuadraticBezier2 {
    /// Retains this quadratic's exact analytic left parallel.
    pub fn parallel_left(&self, distance: Real) -> CurveResult<BezierParallel2> {
        Ok(BezierParallel2::from_source(
            BezierParallelSource2::Quadratic(self.clone()),
            distance,
        ))
    }

    /// Retains this quadratic's exact analytic right parallel.
    pub fn parallel_right(&self, distance: Real) -> CurveResult<BezierParallel2> {
        self.parallel_left(-distance)
    }

    /// Builds the deterministic Blend2D quadratic left-parallel candidate.
    pub fn blend2d_offset_left_candidate(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Blend2dQuadraticOffsetCandidate2>> {
        if real_sign(&distance, policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(Blend2dQuadraticOffsetCandidate2 {
                curve: self.clone(),
                radial_error_bound: Real::zero(),
                tangent_cosine: Real::one(),
                distance,
            }));
        }
        let start_delta = self.control().delta_from(self.start());
        let end_delta = self.end().delta_from(self.control());
        let start_length_squared =
            &start_delta.0 * &start_delta.0 + &start_delta.1 * &start_delta.1;
        let end_length_squared = &end_delta.0 * &end_delta.0 + &end_delta.1 * &end_delta.1;
        for length_squared in [&start_length_squared, &end_length_squared] {
            match real_sign(length_squared, policy) {
                Some(RealSign::Positive) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Some(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "Bezier tangent squared norm was certified negative".to_owned(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let start_length = start_length_squared.sqrt()?;
        let end_length = end_length_squared.sqrt()?;
        let start_normal = (
            ((Real::zero() - &start_delta.1) / &start_length)?,
            (start_delta.0.clone() / &start_length)?,
        );
        let end_normal = (
            ((Real::zero() - &end_delta.1) / &end_length)?,
            (end_delta.0.clone() / &end_length)?,
        );
        let normal_sum = (
            &start_normal.0 + &end_normal.0,
            &start_normal.1 + &end_normal.1,
        );
        let normal_sum_squared = &normal_sum.0 * &normal_sum.0 + &normal_sum.1 * &normal_sum.1;
        match real_sign(&normal_sum_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "summed unit-normal squared norm was certified negative".to_owned(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let tangent_cosine = ((&start_delta.0 * &end_delta.0 + &start_delta.1 * &end_delta.1)
            / (&start_length * &end_length))?;
        let one_plus_cosine = Real::one() + &tangent_cosine;
        match real_sign(&one_plus_cosine, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "endpoint tangent cosine was certified below -1".to_owned(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let middle_scale = ((&distance * Real::from(2_i8)) / &normal_sum_squared)?;
        let candidate = QuadraticBezier2::new(
            self.start()
                .translated(&distance * &start_normal.0, &distance * &start_normal.1),
            self.control()
                .translated(&middle_scale * &normal_sum.0, &middle_scale * &normal_sum.1),
            self.end()
                .translated(&distance * &end_normal.0, &distance * &end_normal.1),
        );
        let half_secant = ((Real::from(2_i8) / &one_plus_cosine)?).sqrt()?;
        let maximum_radial_distance = ((distance.abs() * (Real::from(3_i8) + &tangent_cosine))
            / Real::from(4_i8))?
            * half_secant;
        let radial_error_bound = maximum_radial_distance - distance.abs();
        Ok(Classification::Decided(Blend2dQuadraticOffsetCandidate2 {
            curve: candidate,
            radial_error_bound,
            tangent_cosine,
            distance,
        }))
    }

    /// Builds the deterministic Blend2D quadratic right-parallel candidate.
    pub fn blend2d_offset_right_candidate(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Blend2dQuadraticOffsetCandidate2>> {
        self.blend2d_offset_left_candidate(-distance, policy)
    }

    /// Adaptively constructs and certifies a connected Blend2D quadratic parallel path.
    pub fn approximate_parallel_blend2d_certified(
        &self,
        distance: Real,
        options: &BezierParallelVerificationOptions,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CertifiedBezierParallelPath2>> {
        let analysis = match self
            .parallel_left(distance.clone())?
            .singularity_analysis(&CurveParameterRange2::unit(), policy)?
        {
            Classification::Decided(analysis) => analysis,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !analysis.source_is_regular() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let mut trace = ParallelPathConstructionTrace::default();
        match construct_quadratic_parallel_spans(
            self.clone(),
            Real::zero(),
            Real::one(),
            &distance,
            options,
            policy,
            0,
            &mut trace,
        )? {
            Classification::Decided(()) => {
                Ok(Classification::Decided(CertifiedBezierParallelPath2 {
                    spans: trace.spans,
                    error_bound: options.max_error.clone(),
                    construction_maximum_depth: trace.maximum_depth,
                    verification_leaf_count: trace.verification_leaf_count,
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }
}

impl CubicBezier2 {
    /// Retains this cubic's exact analytic left parallel.
    pub fn parallel_left(&self, distance: Real) -> CurveResult<BezierParallel2> {
        Ok(BezierParallel2::from_source(
            BezierParallelSource2::Cubic(self.clone()),
            distance,
        ))
    }

    /// Retains this cubic's exact analytic right parallel.
    pub fn parallel_right(&self, distance: Real) -> CurveResult<BezierParallel2> {
        self.parallel_left(-distance)
    }

    /// Reduces this cubic to two joined quadratics using the Blend2D construction.
    pub fn blend2d_two_quadratic_reduction(&self) -> CurveResult<Blend2dCubicQuadraticReduction2> {
        let one_quarter = (Real::one() / Real::from(4_i8))?;
        let three_quarters = &one_quarter * Real::from(3_i8);
        let one_half = (Real::one() / Real::from(2_i8))?;
        let first_control = Point2::new(
            self.start().x() * &one_quarter + self.control1().x() * &three_quarters,
            self.start().y() * &one_quarter + self.control1().y() * &three_quarters,
        );
        let second_control = Point2::new(
            self.end().x() * &one_quarter + self.control2().x() * &three_quarters,
            self.end().y() * &one_quarter + self.control2().y() * &three_quarters,
        );
        let midpoint = first_control.lerp(&second_control, one_half);
        let third_difference_x = self.start().x() - self.control1().x() * Real::from(3_i8)
            + self.control2().x() * Real::from(3_i8)
            - self.end().x();
        let third_difference_y = self.start().y() - self.control1().y() * Real::from(3_i8)
            + self.control2().y() * Real::from(3_i8)
            - self.end().y();
        let third_difference_norm = (&third_difference_x * &third_difference_x
            + &third_difference_y * &third_difference_y)
            .sqrt()?;
        Ok(Blend2dCubicQuadraticReduction2 {
            first: QuadraticBezier2::new(self.start().clone(), first_control, midpoint.clone()),
            second: QuadraticBezier2::new(midpoint, second_control, self.end().clone()),
            same_parameter_error_bound: (third_difference_norm / Real::from(54_i8))?,
        })
    }

    /// Adaptively reduces, offsets, and certifies this cubic through Blend2D quadratics.
    pub fn approximate_parallel_blend2d_certified(
        &self,
        distance: Real,
        options: &BezierParallelVerificationOptions,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CertifiedBezierParallelPath2>> {
        let analysis = match self
            .parallel_left(distance.clone())?
            .singularity_analysis(&CurveParameterRange2::unit(), policy)?
        {
            Classification::Decided(analysis) => analysis,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !analysis.source_is_regular() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let mut trace = ParallelPathConstructionTrace::default();
        match construct_cubic_parallel_spans(
            self.clone(),
            Real::zero(),
            Real::one(),
            &distance,
            options,
            policy,
            0,
            &mut trace,
        )? {
            Classification::Decided(()) => {
                Ok(Classification::Decided(CertifiedBezierParallelPath2 {
                    spans: trace.spans,
                    error_bound: options.max_error.clone(),
                    construction_maximum_depth: trace.maximum_depth,
                    verification_leaf_count: trace.verification_leaf_count,
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }
}

impl RationalQuadraticBezier2 {
    /// Retains this rational quadratic's exact analytic left parallel.
    pub fn parallel_left(&self, distance: Real) -> CurveResult<BezierParallel2> {
        Ok(BezierParallel2::from_source(
            BezierParallelSource2::Rational(RationalBezier2::from(self.clone())),
            distance,
        ))
    }

    /// Retains this rational quadratic's exact analytic right parallel.
    pub fn parallel_right(&self, distance: Real) -> CurveResult<BezierParallel2> {
        self.parallel_left(-distance)
    }
}

impl RationalBezier2 {
    /// Retains this arbitrary-degree rational Bezier's exact analytic left parallel.
    pub fn parallel_left(&self, distance: Real) -> CurveResult<BezierParallel2> {
        Ok(BezierParallel2::from_source(
            BezierParallelSource2::Rational(self.clone()),
            distance,
        ))
    }

    /// Retains this arbitrary-degree rational Bezier's exact analytic right parallel.
    pub fn parallel_right(&self, distance: Real) -> CurveResult<BezierParallel2> {
        self.parallel_left(-distance)
    }
}

impl CurvePath2 {
    /// Constructs a connected certified left parallel for supported smooth paths.
    ///
    /// Native lines/arcs and polynomial or rational PH offsets remain exact.
    /// General quadratic/cubic spans use the adaptive Blend2D construction
    /// followed by the conservative verifier. If adjacent primitive parallels do not meet
    /// exactly (the usual case at an authored corner), this returns
    /// `Unsupported`; selecting a miter, round, or bevel join belongs to the
    /// region/string offset layer.
    pub fn approximate_parallel_blend2d_certified(
        &self,
        distance: Real,
        options: &BezierParallelVerificationOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<CertifiedCurvePathParallel2>> {
        let source_curve_count = self.curves().len();
        if real_sign(&distance, policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(CertifiedCurvePathParallel2 {
                path: self.clone(),
                max_parallel_error: Real::zero(),
                source_curve_count,
                output_curve_count: source_curve_count,
                exact_source_curve_count: source_curve_count,
                approximated_source_curve_count: 0,
                verification_leaf_count: 0,
            }));
        }
        if real_sign(&distance, policy).is_none() {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }

        let mut output = Vec::new();
        let mut exact_source_curve_count = 0;
        let mut approximated_source_curve_count = 0;
        let mut verification_leaf_count = 0;
        for source in self.curves() {
            match source.geometry() {
                None => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
                Some(CurveGeometry2::Line(line)) => {
                    output.push(Curve2::from(
                        line.offset_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                    ));
                    exact_source_curve_count += 1;
                }
                Some(CurveGeometry2::CircularArc(arc)) => {
                    let offset = match arc
                        .offset_left(distance.clone(), policy)
                        .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(offset) => offset,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    output.push(Curve2::from(offset));
                    exact_source_curve_count += 1;
                }
                Some(CurveGeometry2::QuadraticBezier(curve)) => {
                    match append_polynomial_parallel(
                        curve
                            .parallel_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                        || {
                            curve.approximate_parallel_blend2d_certified(
                                distance.clone(),
                                options,
                                policy,
                            )
                        },
                        policy,
                        &mut output,
                        &mut verification_leaf_count,
                    )
                    .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(true) => exact_source_curve_count += 1,
                        Classification::Decided(false) => approximated_source_curve_count += 1,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Some(CurveGeometry2::CubicBezier(curve)) => {
                    match append_polynomial_parallel(
                        curve
                            .parallel_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                        || {
                            curve.approximate_parallel_blend2d_certified(
                                distance.clone(),
                                options,
                                policy,
                            )
                        },
                        policy,
                        &mut output,
                        &mut verification_leaf_count,
                    )
                    .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(true) => exact_source_curve_count += 1,
                        Classification::Decided(false) => approximated_source_curve_count += 1,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Some(CurveGeometry2::RationalQuadraticBezier(curve)) => {
                    match append_exact_rational_parallel(
                        curve
                            .parallel_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                        policy,
                        &mut output,
                    )
                    .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(()) => exact_source_curve_count += 1,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Some(CurveGeometry2::RationalBezier(curve)) => {
                    match append_exact_rational_parallel(
                        curve
                            .parallel_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                        policy,
                        &mut output,
                    )
                    .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(()) => exact_source_curve_count += 1,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Some(CurveGeometry2::PolynomialBSpline(_)) | Some(CurveGeometry2::Nurbs(_)) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            }
        }

        if !parallel_curves_are_connected(&output) {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let source_closed = self.start().same_point(&self.end(), policy);
        let output_closed = output
            .first()
            .zip(output.last())
            .map_or(Classification::Decided(false), |(first, last)| {
                first.start().same_point(&last.end(), policy)
            });
        match (source_closed, output_closed) {
            (Classification::Decided(true), Classification::Decided(false)) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            (Classification::Decided(true), Classification::Uncertain(_))
            | (Classification::Uncertain(_), _) => {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
            _ => {}
        }
        let output_curve_count = output.len();
        let path = CurvePath2::try_new(output)?;
        Ok(Classification::Decided(CertifiedCurvePathParallel2 {
            path,
            max_parallel_error: options.max_error().clone(),
            source_curve_count,
            output_curve_count,
            exact_source_curve_count,
            approximated_source_curve_count,
            verification_leaf_count,
        }))
    }
}

pub(super) fn append_exact_rational_parallel(
    parallel: BezierParallel2,
    policy: &CurveContext,
    output: &mut Vec<Curve2>,
) -> CurveResult<Classification<()>> {
    let singularities =
        match parallel.singularity_analysis(&CurveParameterRange2::unit(), policy)? {
            Classification::Decided(singularities) => singularities,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    if !singularities.source_is_regular() || !singularities.parallel_is_cusp_free() {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    }
    match parallel.exact_pythagorean_hodograph_offset(policy)? {
        Classification::Decided(Some(exact)) => {
            output.push(Curve2::from(exact.curve().clone()));
            Ok(Classification::Decided(()))
        }
        Classification::Decided(None) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn append_polynomial_parallel<F>(
    parallel: BezierParallel2,
    approximate: F,
    policy: &CurveContext,
    output: &mut Vec<Curve2>,
    verification_leaf_count: &mut usize,
) -> CurveResult<Classification<bool>>
where
    F: FnOnce() -> CurveResult<Classification<CertifiedBezierParallelPath2>>,
{
    let singularities =
        match parallel.singularity_analysis(&CurveParameterRange2::unit(), policy)? {
            Classification::Decided(singularities) => singularities,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    if !singularities.source_is_regular() || !singularities.parallel_is_cusp_free() {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    }
    match parallel.exact_pythagorean_hodograph_offset(policy)? {
        Classification::Decided(Some(exact)) => {
            output.push(Curve2::from(exact.curve().clone()));
            return Ok(Classification::Decided(true));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let fitted = match approximate()? {
        Classification::Decided(fitted) => fitted,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    *verification_leaf_count += fitted.verification_leaf_count();
    output.extend(
        fitted
            .spans()
            .iter()
            .map(|span| match span.approximation().curve() {
                BezierParallelApproximationCurve2::Quadratic(curve) => Curve2::from(curve.clone()),
                BezierParallelApproximationCurve2::Cubic(curve) => Curve2::from(curve.clone()),
            }),
    );
    Ok(Classification::Decided(false))
}

pub(super) fn parallel_curves_are_connected(curves: &[Curve2]) -> bool {
    curves.windows(2).all(|pair| {
        pair[0]
            .end()
            .same_point(&pair[1].start(), &CurveContext::STRICT)
            == Classification::Decided(true)
    })
}

pub(super) fn parallel_path_error(source: &Curve2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Offset, source.family(), cause)
}

pub(super) fn polynomial_control_power_basis(
    controls: &[&Point2],
) -> CurveResult<(Vec<Real>, Vec<Real>)> {
    Ok((
        bernstein_to_power_coefficients(controls.iter().map(|point| point.x().clone()).collect())?,
        bernstein_to_power_coefficients(controls.iter().map(|point| point.y().clone()).collect())?,
    ))
}

pub(super) fn strict_interior_unit_parameter(
    parameter: &Real,
    policy: &CurveContext,
) -> Classification<()> {
    if in_closed_unit_interval(parameter, policy) != Some(true) {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    }
    match (
        compare_reals(parameter, &Real::zero(), policy),
        compare_reals(parameter, &Real::one(), policy),
    ) {
        (Some(std::cmp::Ordering::Greater), Some(std::cmp::Ordering::Less)) => {
            Classification::Decided(())
        }
        (Some(std::cmp::Ordering::Equal), _) | (_, Some(std::cmp::Ordering::Equal)) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        (Some(_), Some(_)) => Classification::Uncertain(UncertaintyReason::Ordering),
        _ => Classification::Uncertain(UncertaintyReason::Ordering),
    }
}

/// The first nonzero Taylor coefficient determines the exact local sign.
/// Reuse the parameter's existing field/root authority without scalar images
/// or root isolation; an identically zero polynomial stays zero.
pub(super) fn parameter_polynomial_side_sign(
    coefficients: &[Real],
    parameter: &CurveParameter2,
    side: BezierParameterRayDirection2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let mut derivative = Cow::Borrowed(coefficients);
    let mut reverse = false;
    while !derivative.is_empty() {
        match parameter.polynomial_sign(&derivative, policy)? {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(sign) => {
                return Ok(Classification::Decided(if reverse {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        derivative = Cow::Owned(polynomial_derivative(&derivative));
        reverse ^= side == BezierParameterRayDirection2::Decreasing;
    }
    Ok(Classification::Decided(RealSign::Zero))
}

/// On a regular source, sign(1-d*kappa) is sign(S^(3/2)+K).
/// K>=0 needs no squared equation; K<0 reverses sign(K^2-S^3).
pub(super) fn parallel_derivative_scale_from_curvature_sign(
    curvature: Classification<RealSign>,
    norm_sign: impl FnOnce() -> CurveResult<Classification<RealSign>>,
) -> CurveResult<Classification<RealSign>> {
    match curvature {
        Classification::Decided(RealSign::Positive | RealSign::Zero) => {
            Ok(Classification::Decided(RealSign::Positive))
        }
        Classification::Decided(RealSign::Negative) => {
            Ok(norm_sign()?.map(|sign| product_sign(sign, RealSign::Negative)))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn parallel_speed_squared_polynomial(
    differential: &BezierParallelDifferential2,
) -> Vec<Real> {
    polynomial_add(
        &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
        &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
    )
}

pub(super) fn parallel_signed_curvature_polynomial(
    differential: &BezierParallelDifferential2,
    weight: Option<&[Real]>,
    distance: &Real,
) -> Vec<Real> {
    let curvature_cross = polynomial_subtract(
        &polynomial_multiply(&differential.tangent_derivative_x, &differential.tangent_y),
        &polynomial_multiply(&differential.tangent_derivative_y, &differential.tangent_x),
    );
    let curvature_cross = match weight {
        Some(weight) => polynomial_multiply(&polynomial_multiply(weight, weight), &curvature_cross),
        None => curvature_cross,
    };
    polynomial_scale(&curvature_cross, distance)
}

pub(super) fn rational_parametric_tangent_numerator(
    curve: &RationalParametricCurve2,
) -> [Vec<Real>; 2] {
    let weight_derivative = polynomial_derivative(&curve.weight);
    [
        polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&curve.x_numerator), &curve.weight),
            &polynomial_multiply(&curve.x_numerator, &weight_derivative),
        ),
        polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&curve.y_numerator), &curve.weight),
            &polynomial_multiply(&curve.y_numerator, &weight_derivative),
        ),
    ]
}

pub(super) fn reduce_exact_rational_parameter_map(
    numerator: Vec<Real>,
    denominator: Vec<Real>,
) -> (Vec<Real>, Vec<Real>) {
    if let Some(gcd) =
        greatest_common_divisor_univariate_polynomials_exact(&numerator, &denominator)
        && gcd.len() > 1
        && let (Some(numerator), Some(denominator)) = (
            divide_univariate_polynomial_exact(&numerator, &gcd),
            divide_univariate_polynomial_exact(&denominator, &gcd),
        )
    {
        return (numerator, denominator);
    }
    (numerator, denominator)
}

pub(super) fn selected_rational_parameter_image(
    numerator: &[Real],
    denominator: &[Real],
    source: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<Classification<Option<BezierParameter2>>>> {
    let BezierParameter2::Algebraic(source) = source else {
        return Ok(None);
    };
    let degree = numerator.len().max(denominator.len());
    let incidence = BivariatePolynomial::new(
        (0..degree)
            .map(|index| {
                vec![
                    -numerator.get(index).cloned().unwrap_or_else(Real::zero),
                    denominator.get(index).cloned().unwrap_or_else(Real::zero),
                ]
            })
            .collect(),
    );
    Ok(Some(
        match selected_fiber_parameters(
            &incidence,
            &BezierParameter2::Algebraic(source.clone()),
            &crate::CurveParameterRange2::unit(),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                mut parameters,
            )) => match parameters.len() {
                0 => Classification::Decided(None),
                1 => Classification::Decided(parameters.pop()),
                _ => Classification::Uncertain(UncertaintyReason::Predicate),
            },
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => Classification::Uncertain(UncertaintyReason::Boundary),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    ))
}

#[derive(Default)]
pub(super) struct ParallelPathConstructionTrace {
    pub(in crate::bezier_offset) spans: Vec<CertifiedBezierParallelSpan2>,
    pub(in crate::bezier_offset) maximum_depth: usize,
    pub(in crate::bezier_offset) verification_leaf_count: usize,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn construct_quadratic_parallel_spans(
    source: QuadraticBezier2,
    source_start: Real,
    source_end: Real,
    distance: &Real,
    options: &BezierParallelVerificationOptions,
    policy: &CurveContext,
    depth: usize,
    trace: &mut ParallelPathConstructionTrace,
) -> CurveResult<Classification<()>> {
    trace.maximum_depth = trace.maximum_depth.max(depth);
    if let Classification::Decided(candidate) =
        source.blend2d_offset_left_candidate(distance.clone(), policy)?
    {
        let parallel = source.parallel_left(distance.clone())?;
        if let Classification::Decided(approximation) = parallel.verify_polynomial_candidate(
            candidate.curve().clone().into(),
            options,
            policy,
        )? {
            trace.verification_leaf_count += approximation.leaf_count();
            trace.spans.push(CertifiedBezierParallelSpan2 {
                source_start,
                source_end,
                approximation,
            });
            return Ok(Classification::Decided(()));
        }
    }
    if depth >= options.max_depth {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let half = (Real::one() / Real::from(2_i8))?;
    let midpoint = (&source_start + &source_end) * &half;
    let (left, right) = source.split_at_exact(half);
    match construct_quadratic_parallel_spans(
        left,
        source_start,
        midpoint.clone(),
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    construct_quadratic_parallel_spans(
        right,
        midpoint,
        source_end,
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn construct_cubic_parallel_spans(
    source: CubicBezier2,
    source_start: Real,
    source_end: Real,
    distance: &Real,
    options: &BezierParallelVerificationOptions,
    policy: &CurveContext,
    depth: usize,
    trace: &mut ParallelPathConstructionTrace,
) -> CurveResult<Classification<()>> {
    trace.maximum_depth = trace.maximum_depth.max(depth);
    let parallel = source.parallel_left(distance.clone())?;
    let candidate = match parallel.levien_cubic_candidate(policy) {
        Ok(Classification::Decided(candidate)) => Some(candidate),
        Ok(Classification::Uncertain(_)) | Err(CurveError::Real(_)) => None,
        Err(error) => return Err(error),
    };
    if let Some(candidate) = candidate {
        let approximation = match parallel.verify_polynomial_candidate(
            candidate.curve().clone().into(),
            options,
            policy,
        ) {
            Ok(Classification::Decided(approximation)) => Some(approximation),
            Ok(Classification::Uncertain(_)) | Err(CurveError::Real(_)) => None,
            Err(error) => return Err(error),
        };
        if let Some(approximation) = approximation {
            trace.verification_leaf_count += approximation.leaf_count();
            trace.spans.push(CertifiedBezierParallelSpan2 {
                source_start,
                source_end,
                approximation,
            });
            return Ok(Classification::Decided(()));
        }
    }
    if depth >= options.max_depth {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let half = (Real::one() / Real::from(2_i8))?;
    let midpoint = (&source_start + &source_end) * &half;
    let (source_left, source_right) = source.split_at_exact(half.clone());
    let reduction = source.blend2d_two_quadratic_reduction()?;
    match construct_cubic_reduced_half(
        source_left,
        reduction.first().clone(),
        source_start,
        midpoint.clone(),
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    construct_cubic_reduced_half(
        source_right,
        reduction.second().clone(),
        midpoint,
        source_end,
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn construct_cubic_reduced_half(
    source: CubicBezier2,
    reduced: QuadraticBezier2,
    source_start: Real,
    source_end: Real,
    distance: &Real,
    options: &BezierParallelVerificationOptions,
    policy: &CurveContext,
    depth: usize,
    trace: &mut ParallelPathConstructionTrace,
) -> CurveResult<Classification<()>> {
    trace.maximum_depth = trace.maximum_depth.max(depth);
    let candidate = match reduced.blend2d_offset_left_candidate(distance.clone(), policy) {
        Ok(Classification::Decided(candidate)) => Some(candidate),
        Ok(Classification::Uncertain(_)) | Err(CurveError::Real(_)) => None,
        Err(error) => return Err(error),
    };
    if let Some(candidate) = candidate {
        let parallel = source.parallel_left(distance.clone())?;
        let approximation = match parallel.verify_polynomial_candidate(
            candidate.curve().clone().into(),
            options,
            policy,
        ) {
            Ok(Classification::Decided(approximation)) => Some(approximation),
            Ok(Classification::Uncertain(_)) | Err(CurveError::Real(_)) => None,
            Err(error) => return Err(error),
        };
        if let Some(approximation) = approximation {
            trace.verification_leaf_count += approximation.leaf_count();
            trace.spans.push(CertifiedBezierParallelSpan2 {
                source_start,
                source_end,
                approximation,
            });
            return Ok(Classification::Decided(()));
        }
    }
    if depth >= options.max_depth {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    construct_cubic_parallel_spans(
        source,
        source_start,
        source_end,
        distance,
        options,
        policy,
        depth,
        trace,
    )
}

/// Builds the squared image equation for
/// `z = translated/weight + frame/sqrt(speed_squared)`.
///
/// The positive radical sheet is deliberately not encoded by squaring; the
/// caller selects it later with exact convergent bounds from the retained
/// analytic point.
pub(super) fn polynomial_unit_frame_coordinate_relation(
    translated: &[Real],
    weight: &[Real],
    frame: &[Real],
    speed_squared: &[Real],
) -> BivariatePolynomial {
    let weight_squared = polynomial_multiply(weight, weight);
    let constant = polynomial_subtract(
        &polynomial_multiply(&polynomial_multiply(translated, translated), speed_squared),
        &polynomial_multiply(&weight_squared, &polynomial_multiply(frame, frame)),
    );
    let linear = polynomial_scale(
        &polynomial_multiply(&polynomial_multiply(weight, translated), speed_squared),
        &Real::from(-2_i8),
    );
    let quadratic = polynomial_multiply(&weight_squared, speed_squared);
    let coefficients = [constant, linear, quadratic];
    let source_count = coefficients.iter().map(Vec::len).max().unwrap_or(1);
    BivariatePolynomial::new(
        (0..source_count)
            .map(|source_power| {
                coefficients
                    .iter()
                    .map(|coefficient| {
                        coefficient
                            .get(source_power)
                            .cloned()
                            .unwrap_or_else(Real::zero)
                    })
                    .collect()
            })
            .collect(),
    )
}

/// Certifies an oriented PH speed on the requested finite domain. Complete
/// source fields retain their unit certificate across distances; primitive
/// regular fields carry their own range proof. Uncertainty is not absence.
pub(super) fn certify_ph_speed_on_range(
    tangent_x: &[Real],
    tangent_y: &[Real],
    orientation_parameter: &Real,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Arc<BezierParameterPolynomial>>>> {
    let speed_squared = polynomial_add(
        &polynomial_multiply(tangent_x, tangent_x),
        &polynomial_multiply(tangent_y, tangent_y),
    );
    let mut speed = match polynomial_square_root(&speed_squared, policy)? {
        Classification::Decided(Some(speed)) => speed,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let speed_at_orientation = Real::eval_poly(&speed, orientation_parameter);
    match real_sign(&speed_at_orientation, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Negative) => {
            speed = polynomial_scale(&speed, &Real::from(-1_i8));
        }
        Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let speed_polynomial = match polynomial_from_coefficients(speed, policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let speed_roots =
        match CurveParameterDomain2::new(range, None).finite_roots(&speed_polynomial, policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    if !speed_roots.is_empty() {
        return Ok(Classification::Decided(None));
    }
    Ok(Classification::Decided(Some(Arc::new(speed_polynomial))))
}

pub(super) fn polynomial_square_root(
    coefficients: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    let mut normalized = coefficients.to_vec();
    while let Some(coefficient) = normalized.last() {
        match real_sign(coefficient, policy) {
            Some(RealSign::Zero) => {
                normalized.pop();
            }
            Some(RealSign::Positive | RealSign::Negative) => break,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    if normalized.is_empty() {
        return Ok(Classification::Decided(Some(vec![Real::zero()])));
    }
    let mut valuation = 0;
    while valuation < normalized.len() {
        match real_sign(&normalized[valuation], policy) {
            Some(RealSign::Zero) => valuation += 1,
            Some(RealSign::Positive | RealSign::Negative) => break,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    if !valuation.is_multiple_of(2) {
        return Ok(Classification::Decided(None));
    }
    let reduced = &normalized[valuation..];
    let degree = reduced.len() - 1;
    if !degree.is_multiple_of(2) {
        return Ok(Classification::Decided(None));
    }
    let root_degree = degree / 2;
    let constant = reduced[0].clone();
    match real_sign(&constant, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Ok(Classification::Decided(None));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let constant_root = constant.sqrt()?;
    let mut root = vec![Real::zero(); valuation / 2 + root_degree + 1];
    root[valuation / 2] = constant_root.clone();
    for index in 1..=root_degree {
        let mut known = Real::zero();
        for left in 1..index {
            let right = index - left;
            known = &known + &root[valuation / 2 + left] * &root[valuation / 2 + right];
        }
        let residual = &reduced[index] - known;
        root[valuation / 2 + index] = (residual / (&constant_root * Real::from(2_i8)))?;
    }
    let replay = polynomial_multiply(&root, &root);
    let difference = polynomial_subtract(&replay, &normalized);
    for coefficient in difference {
        match real_sign(&coefficient, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(None));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    Ok(Classification::Decided(Some(root)))
}

pub(super) fn polynomial_from_coefficients(
    coefficients: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameterPolynomial>>> {
    match BezierParameterPolynomial::try_new_power_basis(coefficients, policy) {
        Ok(Classification::Decided(polynomial)) => Ok(Classification::Decided(Some(polynomial))),
        Err(CurveError::InvalidBezierPolynomial) => Ok(Classification::Decided(None)),
        Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
        Err(error) => Err(error),
    }
}
