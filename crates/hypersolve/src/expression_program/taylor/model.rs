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
    bound: DyadicInterval,
    monomial: Option<DyadicInterval>,
    contribution: DyadicInterval,
}
impl TaylorRemainderTerm {
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
    program: CoefficientProgram,
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
    /// Generated source-bound normalized partials; regenerate this association for replay.
    pub fn coefficient_program(&self) -> &CoefficientProgram {
        &self.program
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
            let request = &self.program.requests()[term.request];
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
        graph.charge(0)?;
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
                program: self.coefficients(&[], graph)?,
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
        let program = self.coefficients(&requests, graph)?;
        let mut center = Vec::new();
        for value in &request.center {
            graph.charge(1)?;
            let rational = value.to_rational(arithmetic)?;
            center.push(Real::from(
                rational.lower().expect("singleton center").clone(),
            ));
        }
        let exact = program.program().eval_real(&center, graph)?;
        let bounds =
            program
                .program()
                .eval_dyadic(&request.domain, request.precision, graph, arithmetic)?;
        let shifts = shifts(&request.domain, &request.center, graph, arithmetic)?;
        graph.check_entries(&[requests.len(), powers, outputs])?;
        graph.charge(outputs)?;
        let mut remainder = vec![zero()?; outputs];
        let mut terms = Vec::new();
        let mut remainder_terms = Vec::new();
        for (index, ((coefficient, bound), degree)) in exact
            .into_iter()
            .skip(outputs)
            .zip(bounds.into_iter().skip(outputs))
            .zip(degrees)
            .enumerate()
        {
            graph.charge(1)?;
            if degree <= request.order {
                terms.push(TaylorTerm {
                    request: index,
                    coefficient,
                });
            } else {
                let spec = &requests[index];
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
                    request: index,
                    bound,
                    monomial,
                    contribution,
                });
            }
        }
        let mut model = TaylorModel {
            source: self.clone(),
            request,
            program,
            regularity: Some(regularity),
            terms,
            remainder_terms,
            remainder,
            polynomial_image: Vec::new(),
            image: Vec::new(),
            empty: false,
        };
        (model.polynomial_image, model.image) =
            model.evaluate_on(&model.request.domain, graph, arithmetic)?;
        graph.charge(0)?;
        Ok(model)
    }
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
        let image = shifts[slot].pow_rational(
            &BigInt::from(exponent),
            &BigInt::from(1),
            precision,
            arithmetic,
        )?;
        if image.domain != DomainCoverage::Complete {
            return Err(TaylorError::MonomialDomain { variable, exponent });
        }
        let power = image.range.hull(arithmetic)?.finite_range(arithmetic)?;
        monomial = monomial.mul(&power, arithmetic)?;
    }
    let contribution = coefficient.mul(&monomial, arithmetic)?;
    Ok((Some(monomial), contribution))
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

#[cfg(feature = "taylor-affine")]
mod affine;
#[cfg(feature = "taylor-affine")]
pub use affine::{
    TaylorAffineError, TaylorAffineMap, TaylorAffineModel, TaylorAffineRange, TaylorAffineRow,
    TaylorAffineTerm,
};
