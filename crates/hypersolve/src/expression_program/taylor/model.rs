//! Exact centered coefficients with an explicit finite Taylor remainder.
use super::*;
use crate::expression_program::{BinaryOperation, RegularJacobianEnclosure, UnaryOperation};
use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval, SignSet};
use std::collections::BTreeSet;

/// Exact center, finite Cartesian domain and requested polynomial degree.
/// Reconstruct the model from this input and its original source for replay.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TaylorRequest {
    /// Exact singleton dyadics in the original binding order.
    pub center: Vec<DyadicInterval>,
    /// Cartesian domain in the original binding order, containing the center.
    pub domain: Vec<DyadicInterval>,
    /// Total degree retained in the polynomial; no omitted term is assumed zero.
    pub order: usize,
    /// Outward grid for native enclosure operations, not a final-width bound.
    pub precision: i32,
}
/// How a Taylor model bounds what its polynomial omits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TaylorRemainder {
    /// Order n+1 coefficients bounded over the whole domain (the integral form
    /// of the remainder): one term per omitted monomial.
    #[default]
    Lagrange,
    /// Mean-value form, for order-one models only: the deviation of each
    /// regular Jacobian entry over the domain from its exact value at the
    /// center, times the shift of its variable. It reuses the regularity
    /// proof's Jacobian enclosure, so no second-order coefficients are
    /// bounded. It is also of second order in the domain width, usually with
    /// a larger constant.
    MeanValue,
    /// Mean-value form when, for every output, its width is at most 1/16 of
    /// the polynomial's range over the domain. Otherwise the order n+1 terms
    /// are also bounded and the remainder is the intersection of both forms,
    /// so it is never wider than `Lagrange`. Models of order other than one
    /// use `Lagrange`.
    Adaptive,
}
/// One exact coefficient of the centered polynomial.
#[derive(Clone, Debug)]
pub struct TaylorTerm {
    request: usize,
    coefficient: Real,
}
impl TaylorTerm {
    /// Index in the model's coefficient-program requests (output and sparse powers).
    pub fn request_index(&self) -> usize {
        self.request
    }
    /// Authoritative exact coefficient, not an enclosure midpoint.
    pub fn coefficient(&self) -> &Real {
        &self.coefficient
    }
}
/// One degree-(n+1) term contributing to the certified integral remainder.
#[derive(Clone, Debug)]
pub struct TaylorRemainderTerm {
    request: usize,
    kind: TaylorRemainder,
    bound: DyadicInterval,
    monomial: Option<DyadicInterval>,
    contribution: DyadicInterval,
}
impl TaylorRemainderTerm {
    /// Which remainder form this term belongs to: `Lagrange` (an omitted
    /// order n+1 coefficient) or `MeanValue` (a first-order Jacobian entry's
    /// deviation). An adaptive model may hold both; its remainder is then the
    /// intersection of the two sums.
    pub fn kind(&self) -> TaylorRemainder {
        self.kind
    }
    /// Index in the model's coefficient-program requests.
    pub fn request_index(&self) -> usize {
        self.request
    }
    /// Enclosure of D^alpha f / alpha! throughout the original domain.
    pub fn coefficient_bound(&self) -> &DyadicInterval {
        &self.bound
    }
    /// Centered monomial range; absent only when a proved zero coefficient suffices.
    pub fn monomial(&self) -> Option<&DyadicInterval> {
        self.monomial.as_ref()
    }
    /// Full signed contribution included in the remainder.
    pub fn contribution(&self) -> &DyadicInterval {
        &self.contribution
    }
}
/// Source-bound centered polynomial with exact coefficients and a certified remainder.
///
/// For each output f and every x in the retained domain, f(x) is in P(x)+R.
/// All degree-(n+1) normalized partials are bounded on the convex domain and their
/// centered monomial contributions are included in R. The current Expr grammar's
/// selected-variable open regularity conditions are analytic; an exhaustive
/// primitive check binds this stronger claim to that grammar. C1 alone is not
/// assumed to imply arbitrary-order Taylor regularity.
///
/// `image` intersects the polynomial-plus-remainder range with the independently
/// certified natural source image. Exact coefficients, all remainder contributions,
/// source regularity, domain and derivative association remain inspectable. Empty
/// products have no coefficient/regularity claim. These reports are not deserialized
/// as proof; replay the original definition and request through `taylor_model`.
#[derive(Clone, Debug)]
pub struct TaylorModel {
    source: ExpressionProgram,
    request: TaylorRequest,
    requests: Arc<[CoefficientRequest]>,
    regularity: Option<RegularJacobianEnclosure>,
    terms: Vec<TaylorTerm>,
    remainder_terms: Vec<TaylorRemainderTerm>,
    remainder: Vec<DyadicInterval>,
    polynomial_image: Vec<DyadicInterval>,
    image: Vec<DyadicInterval>,
    empty: bool,
}
impl TaylorModel {
    /// Original exact source, including all guards and binding order.
    pub fn source(&self) -> &ExpressionProgram {
        &self.source
    }
    /// Original center, domain, degree and grid.
    pub fn request(&self) -> &TaylorRequest {
        &self.request
    }
    /// Requested normalized partials (output and sparse powers), in term order.
    pub fn coefficient_requests(&self) -> &[CoefficientRequest] {
        &self.requests
    }
    /// Generate the source-bound partial formulas of these requests.
    ///
    /// Construction uses Taylor-mode recurrences and generates these formulas
    /// only when a recurrence is unavailable, so they are not retained. This
    /// replays `ExpressionProgram::coefficients` on the original source.
    pub fn coefficient_program(
        &self,
        graph: &mut ExpressionBudget,
    ) -> Result<CoefficientProgram, TaylorError> {
        self.source.coefficients(&self.requests, graph)
    }
    /// Original regularity/derivative evidence; the exhaustive grammar check supplies analyticity.
    pub fn regularity(&self) -> Option<&RegularJacobianEnclosure> {
        self.regularity.as_ref()
    }
    /// Exact polynomial coefficients, in generation order (including proved zeros).
    pub fn terms(&self) -> &[TaylorTerm] {
        &self.terms
    }
    /// Every retained contribution to the remainder; none is silently discarded.
    pub fn remainder_terms(&self) -> &[TaylorRemainderTerm] {
        &self.remainder_terms
    }
    /// Signed remainder enclosure for each output on the whole original domain.
    pub fn remainder(&self) -> &[DyadicInterval] {
        &self.remainder
    }
    /// Polynomial-only range; this alone is not an enclosure of the source.
    pub fn polynomial_image(&self) -> &[DyadicInterval] {
        &self.polynomial_image
    }
    /// Certified source image, including the full remainder and natural intersection.
    pub fn image(&self) -> &[DyadicInterval] {
        &self.image
    }
    /// Whether the original Cartesian domain is empty; distinguishes empty/unit products.
    pub fn is_empty(&self) -> bool {
        self.empty
    }
    /// Reuse the model on a certified subdomain, retaining the original global remainder.
    /// No extrapolation or new source-regularity assumption is permitted.
    pub fn enclose_on(
        &self,
        domain: &[DyadicInterval],
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Vec<DyadicInterval>, TaylorError> {
        Ok(self.evaluate_on(domain, graph, arithmetic)?.1)
    }
    fn evaluate_on(
        &self,
        domain: &[DyadicInterval],
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<(Vec<DyadicInterval>, Vec<DyadicInterval>), TaylorError> {
        graph.charge(0)?;
        let source = self.source.definition();
        if domain.len() != self.request.domain.len() {
            return Err(ExpressionError::BindingCount {
                expected: self.request.domain.len(),
                actual: domain.len(),
            }
            .into());
        }
        graph.check_entries(&[domain.len(), source.outputs.len(), source.outputs.len()])?;
        graph.charge(domain.len())?;
        if domain.iter().any(DyadicInterval::is_empty) {
            graph.charge(
                source
                    .outputs
                    .len()
                    .checked_mul(2)
                    .ok_or(ExpressionError::SizeExhausted)?,
            )?;
            graph.charge(0)?;
            return Ok((
                vec![DyadicInterval::empty(); source.outputs.len()],
                vec![DyadicInterval::empty(); source.outputs.len()],
            ));
        }
        for (slot, (value, original)) in domain.iter().zip(&self.request.domain).enumerate() {
            graph.charge(1)?;
            if !value.subset_of(original, arithmetic)? {
                return Err(TaylorError::OutsideDomain { slot });
            }
        }
        let shifts = shifts(domain, &self.request.center, graph, arithmetic)?;
        graph.charge(source.outputs.len())?;
        let mut polynomial = vec![zero()?; source.outputs.len()];
        for term in &self.terms {
            graph.charge(1)?;
            let coefficient =
                DyadicInterval::from_real(&term.coefficient, self.request.precision, arithmetic)?;
            let request = &self.requests[term.request];
            let (_, contribution) = contribution(
                request,
                &coefficient,
                &shifts,
                source,
                self.request.precision,
                graph,
                arithmetic,
            )?;
            polynomial[request.output] =
                polynomial[request.output].add(&contribution, arithmetic)?;
        }
        let regular = self
            .regularity
            .as_ref()
            .expect("nonempty subdomain of a nonempty model");
        let mut image = Vec::new();
        for (output, (value, remainder)) in polynomial.iter().zip(&self.remainder).enumerate() {
            graph.charge(1)?;
            let enclosed = value
                .add(remainder, arithmetic)?
                .intersection(&regular.values()[output], arithmetic)?;
            if enclosed.is_empty() {
                return Err(TaylorError::InconsistentEnclosure { output });
            }
            image.push(enclosed);
        }
        graph.charge(0)?;
        Ok((polynomial, image))
    }
}
impl ExpressionProgram {
    /// Construct an exact-coefficient Taylor model with every remainder contribution.
    ///
    /// Only variables reached by an original output generate its sparse monomials.
    /// Original guards still participate in this dependency analysis and regularity
    /// proof. Orders zero through n form the polynomial; order n+1 bounds the
    /// integral remainder. A finite graph/native arithmetic limit may stop the build.
    /// Scalar point arithmetic/refinement, scalar payloads and total heap bytes
    /// retain the existing accounting exclusions. There is no persistent failed cache.
    pub fn taylor_model(
        &self,
        request: TaylorRequest,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<TaylorModel, TaylorError> {
        self.taylor_model_with(request, TaylorRemainder::Lagrange, graph, arithmetic)
    }

    /// [`Self::taylor_model`] with a chosen remainder form.
    ///
    /// With [`TaylorRemainder::MeanValue`] each remainder term is keyed by the
    /// first-order request whose Jacobian entry it bounds; its `bound` is that
    /// entry's deviation from the center coefficient over the domain.
    pub fn taylor_model_with(
        &self,
        request: TaylorRequest,
        remainder_form: TaylorRemainder,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<TaylorModel, TaylorError> {
        graph.charge(0)?;
        let mean_value = remainder_form == TaylorRemainder::MeanValue;
        if mean_value && request.order != 1 {
            return Err(TaylorError::RemainderOrder {
                order: request.order,
            });
        }
        let source = self.definition();
        let dimensions = source.variables.len();
        let outputs = source.outputs.len();
        if request.domain.len() != dimensions {
            return Err(ExpressionError::BindingCount {
                expected: dimensions,
                actual: request.domain.len(),
            }
            .into());
        }
        if request.center.len() != dimensions {
            return Err(TaylorError::CenterCount {
                expected: dimensions,
                actual: request.center.len(),
            });
        }
        graph.check_entries(&[dimensions, dimensions, outputs])?;
        for (slot, center) in request.center.iter().enumerate() {
            graph.charge(1)?;
            if !center.is_singleton() {
                return Err(TaylorError::NonPointCenter { slot });
            }
        }
        graph.charge(dimensions)?;
        if request.domain.iter().any(DyadicInterval::is_empty) {
            graph.check_entries(&[dimensions, dimensions, outputs, outputs, outputs])?;
            graph.charge(
                outputs
                    .checked_mul(3)
                    .ok_or(ExpressionError::SizeExhausted)?,
            )?;
            let model = TaylorModel {
                source: self.clone(),
                requests: Arc::from([]),
                request,
                regularity: None,
                terms: Vec::new(),
                remainder_terms: Vec::new(),
                remainder: vec![DyadicInterval::empty(); outputs],
                polynomial_image: vec![DyadicInterval::empty(); outputs],
                image: vec![DyadicInterval::empty(); outputs],
                empty: true,
            };
            graph.charge(0)?;
            return Ok(model);
        }
        for (slot, (center, domain)) in request.center.iter().zip(&request.domain).enumerate() {
            graph.charge(1)?;
            if !center.subset_of(domain, arithmetic)? {
                return Err(TaylorError::CenterOutsideDomain { slot });
            }
        }
        // Every allowed active primitive is analytic on the open domains checked
        // by enclose_regular (with full-domain guards for powers/division/logs).
        // Exhaustive matches force an explicit proof review when grammar grows.
        for node in &source.nodes {
            graph.charge(1)?;
            match node {
                ExpressionNode::Constant(_) | ExpressionNode::Variable(_) => {}
                ExpressionNode::Unary { operation, .. } => match operation {
                    UnaryOperation::Neg
                    | UnaryOperation::PowI(_)
                    | UnaryOperation::Sqrt
                    | UnaryOperation::Sin
                    | UnaryOperation::Cos
                    | UnaryOperation::Ln
                    | UnaryOperation::Log10
                    | UnaryOperation::Asin
                    | UnaryOperation::Acos
                    | UnaryOperation::Acosh
                    | UnaryOperation::Atanh => {}
                },
                ExpressionNode::Binary { operation, .. } => match operation {
                    BinaryOperation::Add
                    | BinaryOperation::Sub
                    | BinaryOperation::Mul
                    | BinaryOperation::Div => {}
                },
            }
        }
        let mut requests = Vec::new();
        let mut degrees = Vec::new();
        let mut used = BTreeSet::new();
        let mut powers = 0usize;
        for (output, &root) in source.outputs.iter().enumerate() {
            let variables = dependencies(source, root, graph)?;
            for &variable in &variables {
                graph.charge(1)?;
                used.insert(variable);
            }
            generate(
                output,
                &variables,
                request.order,
                &mut requests,
                &mut degrees,
                &mut powers,
                graph,
            )?;
        }
        graph.charge(used.len())?;
        let variables: Vec<_> = used.into_iter().collect();
        let regularity = self.jacobian(&variables, graph)?.enclose_regular(
            &request.domain,
            request.precision,
            graph,
            arithmetic,
        )?;
        // The generated formulas are built only if a recurrence is unavailable.
        let mut program = None;
        let mut center = Vec::new();
        for value in &request.center {
            graph.charge(1)?;
            let rational = value.to_rational(arithmetic)?;
            center.push(Real::from(
                rational.lower().expect("singleton center").clone(),
            ));
        }
        // Exact Taylor-mode recurrences at the center avoid evaluating the
        // swollen generated formulas; those formulas remain the authority
        // whenever an exact scalar operation is refused.
        let exact_degree = if variables.is_empty() {
            0
        } else {
            request.order
        };
        let (exact, mut formulas) =
            match recurrence::exact(self, &variables, &center, exact_degree, graph)? {
                Some(exact) => (Some(exact), Vec::new().into_iter()),
                None => {
                    let mut values = formulas_for(&mut program, self, &requests, graph)?
                        .program()
                        .eval_real(&center, graph)?;
                    // Original outputs precede the requested coefficients.
                    values.drain(..outputs);
                    (None, values.into_iter())
                }
            };
        // Without variables the order is irrelevant and may be usize::MAX.
        let last = if variables.is_empty() {
            0
        } else {
            request
                .order
                .checked_add(1)
                .ok_or(TaylorError::DegreeOverflow)?
        };
        // Interval Taylor-mode recurrences avoid the expression swell of
        // repeatedly differentiated partial formulas. Natural evaluation of
        // those formulas remains the bound when a recurrence is unavailable.
        let shifts = shifts(&request.domain, &request.center, graph, arithmetic)?;
        graph.check_entries(&[requests.len(), powers, outputs])?;
        graph.charge(outputs)?;
        let mut terms = Vec::new();
        let mut omitted = Vec::new();
        for (index, degree) in degrees.into_iter().enumerate() {
            graph.charge(1)?;
            let formula = formulas.next();
            if degree <= request.order {
                let coefficient = match &exact {
                    Some(exact) => exact.coefficient(&requests[index]).clone(),
                    None => formula.expect("one generated value per request"),
                };
                terms.push(TaylorTerm {
                    request: index,
                    coefficient,
                });
            } else {
                omitted.push(index);
            }
        }
        let mut remainder_terms = Vec::new();
        let mut remainder = vec![zero()?; outputs];
        let first_order = request.order == 1 && !variables.is_empty();
        let mut lagrange = remainder_form == TaylorRemainder::Lagrange
            || (remainder_form == TaylorRemainder::Adaptive && !first_order);
        if !lagrange {
            // f(x) - f(c) - J(c)(x - c) = (J(xi) - J(c))(x - c) for some xi
            // on the segment, and J(xi) lies in the regular Jacobian enclosure.
            for term in &terms {
                graph.charge(1)?;
                let spec = &requests[term.request];
                let [(variable, 1)] = spec.powers[..] else {
                    continue;
                };
                let column = variables
                    .iter()
                    .position(|&v| v == variable)
                    .expect("first-order request of a used variable");
                let entry = outputs
                    .checked_mul(column + 1)
                    .and_then(|v| v.checked_add(spec.output))
                    .ok_or(ExpressionError::SizeExhausted)?;
                let center =
                    DyadicInterval::from_real(&term.coefficient, request.precision, arithmetic)?;
                let bound = regularity.values()[entry].sub(&center, arithmetic)?;
                let (monomial, contribution) = contribution(
                    spec,
                    &bound,
                    &shifts,
                    source,
                    request.precision,
                    graph,
                    arithmetic,
                )?;
                remainder[spec.output] = remainder[spec.output].add(&contribution, arithmetic)?;
                remainder_terms.push(TaylorRemainderTerm {
                    request: term.request,
                    kind: TaylorRemainder::MeanValue,
                    bound,
                    monomial,
                    contribution,
                });
            }
        }
        let mut model = TaylorModel {
            source: self.clone(),
            request,
            requests: requests.into(),
            regularity: Some(regularity),
            terms,
            remainder_terms,
            remainder,
            polynomial_image: Vec::new(),
            image: Vec::new(),
            empty: false,
        };
        if !lagrange {
            (model.polynomial_image, model.image) =
                model.evaluate_on(&model.request.domain, graph, arithmetic)?;
            if remainder_form == TaylorRemainder::Adaptive {
                // Keep the mean-value form only where it is negligible next to
                // the polynomial's own range; otherwise also bound the omitted
                // terms and keep the intersection of both valid remainders.
                for (rest, range) in model.remainder.iter().zip(&model.polynomial_image) {
                    graph.charge(1)?;
                    let sixteen = DyadicInterval::point(16.into(), 0)?;
                    let scaled = rest.width(arithmetic)?.mul(&sixteen, arithmetic)?;
                    let allowed = zero()?.hull(&range.width(arithmetic)?, arithmetic)?;
                    if !scaled.subset_of(&allowed, arithmetic)? {
                        lagrange = true;
                    }
                }
            }
        }
        if lagrange {
            // Interval Taylor-mode recurrences avoid the expression swell of
            // repeatedly differentiated partial formulas. Natural evaluation of
            // those formulas remains the bound when a recurrence is unavailable.
            let partials = match recurrence::bounds(
                self,
                &variables,
                &model.request,
                last,
                graph,
                arithmetic,
            )? {
                Some(recurrence) => Partials::Recurrence(recurrence),
                None => Partials::Natural(
                    formulas_for(&mut program, self, &model.requests, graph)?
                        .program()
                        .eval_dyadic(
                            &model.request.domain,
                            model.request.precision,
                            graph,
                            arithmetic,
                        )?,
                ),
            };
            let mut lagrange_remainder = vec![zero()?; outputs];
            for &index in &omitted {
                graph.charge(1)?;
                let spec = &model.requests[index];
                let bound = match &partials {
                    Partials::Recurrence(recurrence) => recurrence.coefficient(spec).clone(),
                    Partials::Natural(bounds) => bounds[outputs + index].clone(),
                };
                let (monomial, contribution) = contribution(
                    spec,
                    &bound,
                    &shifts,
                    source,
                    model.request.precision,
                    graph,
                    arithmetic,
                )?;
                lagrange_remainder[spec.output] =
                    lagrange_remainder[spec.output].add(&contribution, arithmetic)?;
                model.remainder_terms.push(TaylorRemainderTerm {
                    request: index,
                    kind: TaylorRemainder::Lagrange,
                    bound,
                    monomial,
                    contribution,
                });
            }
            model.remainder = if model
                .remainder_terms
                .iter()
                .any(|t| t.kind == TaylorRemainder::MeanValue)
            {
                // Both forms enclose the same remainder over the whole domain.
                let mut both = Vec::new();
                for (output, (mean, integral)) in
                    model.remainder.iter().zip(&lagrange_remainder).enumerate()
                {
                    graph.charge(1)?;
                    let meet = mean.intersection(integral, arithmetic)?;
                    if meet.is_empty() {
                        return Err(TaylorError::InconsistentEnclosure { output });
                    }
                    both.push(meet);
                }
                both
            } else {
                lagrange_remainder
            };
            (model.polynomial_image, model.image) =
                model.evaluate_on(&model.request.domain, graph, arithmetic)?;
        }
        graph.charge(0)?;
        Ok(model)
    }
}
/// How one binding slot of an ODE right-hand side is fed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OdeBinding {
    /// The independent time variable.
    Time,
    /// State lane `j` of `x' = f(t, x)`.
    State(usize),
}
impl ExpressionProgram {
    /// Taylor-mode coefficients of solutions of `x' = f(t, x)` over boxes.
    ///
    /// Output `j` of this program is `f_j`; `bindings[i]` feeds binding slot `i`.
    /// The result `x[j][k]`, for `k` in `0..=degree`, encloses `x_j^(k)(tau)/k!`
    /// for every solution with `tau` in `start` and `x(tau)` in `initial`
    /// (`x[j][0]` is `initial[j]`). Each degree comes from the univariate
    /// Taylor-mode recurrence `x_(k+1) = [f(t, x)]_k / (k + 1)` of the same
    /// per-primitive recurrences as `taylor_model`, so no Lie-derivative formulas
    /// are generated. Degree zero of every primitive is its certified image,
    /// which enforces its guard over the boxes. This bounds derivative values;
    /// it does not prove that a solution exists or that `f` is analytic there.
    /// `None` when a recurrence is unavailable (a guard or zero divisor met on
    /// the boxes, or an empty box); resource limits are reported as errors.
    #[allow(clippy::too_many_arguments)]
    pub fn ode_taylor_coefficients(
        &self,
        bindings: &[OdeBinding],
        start: &DyadicInterval,
        initial: &[DyadicInterval],
        degree: usize,
        precision: i32,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Option<Vec<Vec<DyadicInterval>>>, TaylorError> {
        graph.charge(0)?;
        let source = self.definition();
        if bindings.len() != source.variables.len() {
            return Err(ExpressionError::BindingCount {
                expected: source.variables.len(),
                actual: bindings.len(),
            }
            .into());
        }
        let shape = |lane| TaylorError::OdeShape {
            outputs: source.outputs.len(),
            state: initial.len(),
            lane,
        };
        if source.outputs.len() != initial.len() {
            return Err(shape(None));
        }
        graph.check_entries(&[bindings.len(), initial.len()])?;
        let mut slots = Vec::new();
        for binding in bindings {
            graph.charge(1)?;
            slots.push(match *binding {
                OdeBinding::Time => None,
                OdeBinding::State(lane) if lane < initial.len() => Some(lane),
                OdeBinding::State(lane) => return Err(shape(Some(lane))),
            });
        }
        if start.is_empty() || initial.iter().any(DyadicInterval::is_empty) {
            return Ok(None);
        }
        recurrence::ode(
            self, &slots, start, initial, degree, precision, graph, arithmetic,
        )
    }
}
// Generate the requested partial formulas once, on first use.
fn formulas_for<'a>(
    program: &'a mut Option<CoefficientProgram>,
    source: &ExpressionProgram,
    requests: &[CoefficientRequest],
    graph: &mut ExpressionBudget,
) -> Result<&'a CoefficientProgram, TaylorError> {
    if program.is_none() {
        *program = Some(source.coefficients(requests, graph)?);
    }
    Ok(program.as_ref().expect("generated above"))
}
// Enclosures of the degree-(n+1) normalized partials on the whole domain.
enum Partials {
    Recurrence(recurrence::Bounds<DyadicInterval>),
    // Natural evaluation of every coefficient-program output, originals first.
    Natural(Vec<DyadicInterval>),
}
fn zero() -> Result<DyadicInterval, hyperinterval::IntervalError> {
    DyadicInterval::point(0.into(), 0)
}
fn shifts(
    domain: &[DyadicInterval],
    center: &[DyadicInterval],
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Result<Vec<DyadicInterval>, TaylorError> {
    graph.check_entries(&[domain.len()])?;
    domain
        .iter()
        .zip(center)
        .map(|(value, center)| {
            graph.charge(1)?;
            Ok(value.sub(center, arithmetic)?)
        })
        .collect()
}
fn contribution(
    request: &CoefficientRequest,
    coefficient: &DyadicInterval,
    shifts: &[DyadicInterval],
    source: &ExpressionDefinition,
    precision: i32,
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Result<(Option<DyadicInterval>, DyadicInterval), TaylorError> {
    if coefficient.signs() == SignSet::Zero {
        return Ok((None, zero()?));
    }
    let mut monomial = DyadicInterval::point(1.into(), 0)?;
    for &(variable, exponent) in &request.powers {
        graph.charge(source.variables.len())?;
        let slot = source
            .variables
            .iter()
            .position(|&v| v == variable)
            .expect("validated sparse variable");
        let power = monomial_power(&shifts[slot], exponent, precision, arithmetic)?
            .ok_or(TaylorError::MonomialDomain { variable, exponent })?;
        monomial = monomial.mul(&power, arithmetic)?;
    }
    let contribution = coefficient.mul(&monomial, arithmetic)?;
    Ok((Some(monomial), contribution))
}
// Positive integer powers use the shared exact-power path; see positive_power.
fn monomial_power(
    shift: &DyadicInterval,
    exponent: usize,
    precision: i32,
    arithmetic: &mut DyadicBudget,
) -> Result<Option<DyadicInterval>, TaylorError> {
    if let Ok(power @ 1..) = u32::try_from(exponent)
        && let Some(range) =
            crate::expression_program::dyadic::positive_power(shift, power, precision, arithmetic)?
    {
        return Ok(Some(range));
    }
    general_monomial_power(shift, exponent, precision, arithmetic)
}
fn general_monomial_power(
    shift: &DyadicInterval,
    exponent: usize,
    precision: i32,
    arithmetic: &mut DyadicBudget,
) -> Result<Option<DyadicInterval>, TaylorError> {
    let image = shift.pow_rational(
        &BigInt::from(exponent),
        &BigInt::from(1),
        precision,
        arithmetic,
    )?;
    if image.domain != DomainCoverage::Complete {
        return Ok(None);
    }
    Ok(Some(
        image.range.hull(arithmetic)?.finite_range(arithmetic)?,
    ))
}
fn dependencies(
    source: &ExpressionDefinition,
    root: usize,
    graph: &mut ExpressionBudget,
) -> Result<Vec<SymbolId>, ExpressionError> {
    graph.check_entries(&[source.nodes.len(), 1])?;
    graph.charge(source.nodes.len() + 1)?;
    let mut seen = vec![false; source.nodes.len()];
    let mut tasks = vec![root];
    let mut variables = BTreeSet::new();
    while let Some(index) = tasks.pop() {
        graph.charge(1)?;
        if seen[index] {
            continue;
        }
        seen[index] = true;
        let node = &source.nodes[index];
        if let ExpressionNode::Variable(reference) = node {
            graph.check_entries(&[seen.len(), tasks.len(), variables.len(), 1])?;
            graph.charge(1)?;
            variables.insert(reference.id);
        }
        let inputs = node.inputs();
        let count = inputs.iter().flatten().count();
        graph.check_entries(&[seen.len(), tasks.len(), variables.len(), count])?;
        graph.charge(count)?;
        if let Some(right) = inputs[1] {
            tasks.push(right);
        }
        if let Some(left) = inputs[0] {
            tasks.push(left);
        }
    }
    graph.charge(variables.len())?;
    Ok(variables.into_iter().collect())
}
fn generate(
    output: usize,
    variables: &[SymbolId],
    order: usize,
    requests: &mut Vec<CoefficientRequest>,
    degrees: &mut Vec<usize>,
    powers: &mut usize,
    graph: &mut ExpressionBudget,
) -> Result<(), TaylorError> {
    graph.check_entries(&[requests.len(), degrees.len(), *powers, 3])?;
    graph.charge(3)?;
    let mut frontier = vec![requests.len()];
    requests.push(CoefficientRequest {
        output,
        powers: Vec::new(),
    });
    degrees.push(0);
    if variables.is_empty() {
        return Ok(());
    }
    let last = order.checked_add(1).ok_or(TaylorError::DegreeOverflow)?;
    for degree in 1..=last {
        let mut next = Vec::new();
        for parent in frontier.iter().copied() {
            let lower = requests[parent].powers.last().map(|&(v, _)| v);
            for &variable in variables {
                graph.charge(1)?;
                if lower.is_some_and(|lower| variable < lower) {
                    continue;
                }
                let grow = usize::from(lower != Some(variable));
                let length = requests[parent]
                    .powers
                    .len()
                    .checked_add(grow)
                    .ok_or(ExpressionError::SizeExhausted)?;
                let total = powers
                    .checked_add(length)
                    .ok_or(ExpressionError::SizeExhausted)?;
                graph.check_entries(&[
                    requests.len(),
                    degrees.len(),
                    total,
                    frontier.len(),
                    next.len(),
                    3,
                ])?;
                graph.charge(
                    length
                        .checked_add(3)
                        .ok_or(ExpressionError::SizeExhausted)?,
                )?;
                let mut monomial = requests[parent].powers.clone();
                if grow == 0 {
                    monomial.last_mut().expect("last variable").1 += 1;
                } else {
                    monomial.push((variable, 1));
                }
                next.push(requests.len());
                requests.push(CoefficientRequest {
                    output,
                    powers: monomial,
                });
                degrees.push(degree);
                *powers = total;
            }
        }
        frontier = next;
    }
    Ok(())
}

mod recurrence;

#[cfg(feature = "taylor-affine")]
mod affine;
#[cfg(feature = "taylor-affine")]
pub use affine::{
    TaylorAffineError, TaylorAffineMap, TaylorAffineModel, TaylorAffineRange, TaylorAffineRow,
    TaylorAffineTerm,
};

#[cfg(feature = "taylor-polynomial")]
mod polynomial;
#[cfg(feature = "taylor-polynomial")]
pub use affine::TaylorAffineRange as TaylorRange;
#[cfg(feature = "taylor-polynomial")]
pub use polynomial::{
    TaylorPolynomialError, TaylorPolynomialModel, TaylorPolynomialProduct, TaylorPolynomialTerms,
};

#[cfg(feature = "taylor-noise")]
pub use affine::{TaylorNoiseColumn, TaylorNoiseModel};
#[cfg(feature = "taylor-combination")]
pub use polynomial::TaylorOperand;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integer_monomial_power_equals_the_general_rational_power_image() {
        let mut budget = DyadicBudget::new(16_384, 1_000_000_000);
        let mut checked = 0;
        for scale in [-12, -3, 0, 2] {
            for lower in -9_i64..=9 {
                for upper in lower..=9 {
                    let shift = DyadicInterval::new(lower.into(), upper.into(), scale).unwrap();
                    for exponent in 1..=7 {
                        for precision in [-64, -8, -2, 0, 3] {
                            let fast =
                                monomial_power(&shift, exponent, precision, &mut budget).unwrap();
                            let general =
                                general_monomial_power(&shift, exponent, precision, &mut budget)
                                    .unwrap();
                            assert_eq!(fast, general, "{shift:?}^{exponent} on 2^{precision}");
                            checked += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(checked, 4 * 190 * 7 * 5);
    }

    #[test]
    fn oversized_exact_powers_keep_the_general_path() {
        // 2^40 + 1 needs 41 bits; its seventh power needs 287 > 256.
        let shift = DyadicInterval::new(1.into(), ((1_i64 << 40) + 1).into(), 0).unwrap();
        let mut budget = DyadicBudget::new(256, 1_000_000_000);
        assert_eq!(
            crate::expression_program::dyadic::positive_power(&shift, 7, -8, &mut budget).unwrap(),
            None
        );
        let mut fast = DyadicBudget::new(256, 1_000_000_000);
        let mut general = DyadicBudget::new(256, 1_000_000_000);
        assert_eq!(
            monomial_power(&shift, 7, -8, &mut fast).map_err(|e| format!("{e:?}")),
            general_monomial_power(&shift, 7, -8, &mut general).map_err(|e| format!("{e:?}"))
        );
        assert_eq!(fast.remaining_work(), general.remaining_work());
    }
}
