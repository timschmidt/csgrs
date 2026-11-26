//! Polynomial maps over existing centered polynomials and certified residuals.
use super::*;

/// Canonically ordered sparse powers of original source variables and exact coefficients.
/// Zero coefficients may remain; no interval endpoint replaces a coefficient.
pub type TaylorPolynomialTerms = BTreeMap<Vec<(crate::SymbolId, usize)>, Real>;

/// Map validation or an original Taylor/native resource failure.
#[derive(Clone, Debug)]
pub enum TaylorPolynomialError {
    /// The map's ordered bindings do not match the immediate input outputs.
    InputCount { expected: usize, actual: usize },
    /// A reached node is outside constant/input/negation/add/subtract/multiply grammar.
    UnsupportedOperation { node: usize },
    /// Original domain, precision, resource, cancellation or scalar diagnostic.
    Taylor(TaylorError),
}
impl fmt::Display for TaylorPolynomialError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Taylor polynomial map: {self:?}")
    }
}
impl std::error::Error for TaylorPolynomialError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Taylor(e) => Some(e),
            _ => None,
        }
    }
}
impl From<TaylorError> for TaylorPolynomialError {
    fn from(e: TaylorError) -> Self {
        Self::Taylor(e)
    }
}
impl From<ExpressionError> for TaylorPolynomialError {
    fn from(e: ExpressionError) -> Self {
        Self::Taylor(e.into())
    }
}
impl From<hyperinterval::IntervalError> for TaylorPolynomialError {
    fn from(e: hyperinterval::IntervalError) -> Self {
        Self::Taylor(e.into())
    }
}

/// All contributions added when multiplying `(P+R)*(Q+S)`.
/// Bounds refer to the original full parameter domain, not a later subdomain.
#[derive(Clone, Debug)]
pub struct TaylorPolynomialProduct {
    discarded: TaylorPolynomialTerms,
    truncation: DyadicInterval,
    left_cross: DyadicInterval,
    right_cross: DyadicInterval,
    residual_product: DyadicInterval,
}
impl TaylorPolynomialProduct {
    /// Exact combined monomials above the retained total degree.
    pub fn discarded_terms(&self) -> &TaylorPolynomialTerms {
        &self.discarded
    }
    /// Enclosure of every discarded monomial, including its exact coefficient.
    pub fn truncation(&self) -> &DyadicInterval {
        &self.truncation
    }
    /// Enclosure of P*S.
    pub fn left_polynomial_times_right_remainder(&self) -> &DyadicInterval {
        &self.left_cross
    }
    /// Enclosure of Q*R.
    pub fn right_polynomial_times_left_remainder(&self) -> &DyadicInterval {
        &self.right_cross
    }
    /// Enclosure of R*S; equal bounds do not imply equal residual functions.
    pub fn remainder_product(&self) -> &DyadicInterval {
        &self.residual_product
    }
}
#[derive(Clone, Debug)]
struct Value {
    terms: TaylorPolynomialTerms,
    remainder: DyadicInterval,
    product: Option<TaylorPolynomialProduct>,
}
#[derive(Debug)]
struct Stage {
    map: ExpressionProgram,
    #[cfg(feature = "taylor-composition")]
    model: Option<Arc<TaylorModel>>,
    #[cfg(feature = "taylor-combination")]
    join: Option<combination::Join>,
    // Flat retained proof nodes share repeated map edges. Stages have no owning
    // parent link, so long composition chains do not recursively destroy parents.
    values: Vec<Option<Arc<Value>>>,
    outputs: Vec<Arc<Value>>,
}
impl Stage {
    /// Number of rows this stage passes to the next map.
    fn arity(&self) -> usize {
        #[cfg(feature = "taylor-combination")]
        if let Some(join) = &self.join {
            return join.arity();
        }
        self.map.definition().outputs.len()
    }
}
/// A retained polynomial/remainder representation transported through exact maps.
///
/// The original model supplies source guards, regularity, domain, center, order
/// and grid. Each map binds its variables, in order, to all immediate input
/// outputs. Raw expression maps support constants, variables, negation, addition,
/// subtraction and multiplication. Unreached nodes are not evaluated. No new
/// derivatives are constructed. All truncated monomials and residual cross terms
/// remain evidence; residual intervals never assert independence or equality.
///
/// Replay the original model definition/request and each `maps()` definition.
/// With `taylor-composition`, `compose` also accepts certified Taylor maps. Their
/// original definitions are exposed by `maps`, their certificates by `model_map`,
/// and their generated polynomial graphs by `polynomial_map`. Retain each outer
/// definition/request when replaying those stages. The resulting coefficients
/// define an exact polynomial plus residual; they need not be the derivatives
/// of the exact composite at the original center.
///
/// These private certificates are not deserialized as proof. Shared stages retain
/// exact sources and generated objects without expanding them into expression
/// trees. A transform copies only the flat list of shared stage handles.
///
/// Structural entry/work limits cover new map traversal, coefficient keys, proof
/// nodes and stage handles; entry checks are stage limits, not total live heap
/// bounds. Scalar coefficient arithmetic/refinement, payload sizes and in-scalar
/// cancellation retain TaylorModel's exclusions. Enclosures use the supplied
/// dyadic budget. Failures publish no model and do not mutate the source or input.
#[derive(Clone, Debug)]
pub struct TaylorPolynomialModel {
    source: Arc<TaylorModel>,
    stages: Arc<[Arc<Stage>]>,
    range: Arc<[DyadicInterval]>,
}
impl TaylorPolynomialModel {
    /// Transport an existing model through an exact polynomial map.
    pub fn new(
        source: Arc<TaylorModel>,
        map: ExpressionProgram,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Self, TaylorPolynomialError> {
        let mut model = Self {
            source,
            stages: Arc::from([]),
            range: Arc::from([]),
        };
        model.append(map, graph, arithmetic)?;
        Ok(model)
    }
    /// Original shared source certificate, including all original outputs/guards.
    pub fn source(&self) -> &Arc<TaylorModel> {
        &self.source
    }
    /// Exact map definitions in application order; bindings refer to immediate outputs.
    pub fn maps(&self) -> impl Iterator<Item = &ExpressionProgram> {
        self.stages.iter().map(|s| {
            #[cfg(feature = "taylor-composition")]
            if let Some(model) = &s.model {
                return model.source();
            }
            #[cfg(feature = "taylor-combination")]
            if let Some(combination::Join {
                operand: TaylorOperand::Model(model),
                ..
            }) = &s.join
            {
                return model.source();
            }
            &s.map
        })
    }
    /// Number of retained map stages.
    pub fn stage_count(&self) -> usize {
        self.stages.len()
    }
    /// Whether the original parameter product was empty, including zero output maps.
    pub fn is_empty(&self) -> bool {
        self.source.is_empty()
    }
    /// Certified final image on the full original parameter domain.
    pub fn range(&self) -> &[DyadicInterval] {
        &self.range
    }
    /// Final exact polynomial; absent for an invalid output or empty parameter product.
    pub fn terms(&self, output: usize) -> Option<&TaylorPolynomialTerms> {
        self.stages.last()?.outputs.get(output).map(|v| &v.terms)
    }
    /// Full-domain final residual; absent for an invalid output or empty product.
    pub fn remainder(&self, output: usize) -> Option<&DyadicInterval> {
        self.stages
            .last()?
            .outputs
            .get(output)
            .map(|v| &v.remainder)
    }
    /// Multiplication evidence at a polynomial graph node; absent for other/unreached nodes.
    /// For a composed-model stage, use `polynomial_map` to obtain those node indices.
    pub fn product(&self, stage: usize, node: usize) -> Option<&TaylorPolynomialProduct> {
        let stage = self.stages.get(stage)?;
        if !matches!(
            stage.map.definition().nodes.get(node)?,
            ExpressionNode::Binary {
                operation: BinaryOperation::Mul,
                ..
            }
        ) {
            return None;
        }
        stage.values.get(node)?.as_ref()?.product.as_ref()
    }
    /// Consume generated output as the next polynomial map's input without re-differentiation.
    pub fn transform(
        &self,
        map: ExpressionProgram,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Self, TaylorPolynomialError> {
        graph.charge(0)?;
        let mut next = self.clone();
        next.append(map, graph, arithmetic)?;
        Ok(next)
    }
    fn append(
        &mut self,
        map: ExpressionProgram,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<(), TaylorPolynomialError> {
        let count = self
            .stages
            .last()
            .map_or(self.source.image().len(), |s| s.arity());
        let uses = validate(&map, count, graph)?;
        let history = self
            .stages
            .len()
            .checked_add(1)
            .ok_or(ExpressionError::SizeExhausted)?;
        graph.check_entries(&[history, uses.len(), map.definition().outputs.len()])?;
        let mut context = Context {
            source: &self.source,
            shifts: Vec::new(),
            graph,
            arithmetic,
            entries: history,
        };
        context.reserve(uses.len())?;
        // The demand counts and the value cache coexist throughout construction.
        context.reserve(uses.len())?;
        context.reserve(map.definition().outputs.len())?;
        let mut values: Vec<Option<Arc<Value>>> = vec![None; uses.len()];
        let mut inputs = Vec::new();
        if !self.is_empty() {
            context.shifts = super::shifts(
                &self.source.request.domain,
                &self.source.request.center,
                context.graph,
                context.arithmetic,
            )?;
            context.reserve(context.shifts.len())?;
            context.reserve(count)?;
            if let Some(stage) = self.stages.last() {
                context.graph.charge(count)?;
                inputs.extend(stage.outputs.iter().cloned());
            } else {
                inputs = context.rows(&self.source, count)?;
            }
            context.evaluate(&map, &uses, &inputs, &mut values)?;
        }
        let outputs = if self.is_empty() {
            Vec::new()
        } else {
            map.definition()
                .outputs
                .iter()
                .map(|&i| {
                    context.graph.charge(1)?;
                    Ok(values[i].as_ref().expect("demanded output").clone())
                })
                .collect::<Result<Vec<_>, ExpressionError>>()?
        };
        context.graph.charge(history)?;
        let mut stages = self.stages.to_vec();
        stages.push(Arc::new(Stage {
            map,
            #[cfg(feature = "taylor-composition")]
            model: None,
            #[cfg(feature = "taylor-combination")]
            join: None,
            values,
            outputs,
        }));
        self.stages = stages.into();
        self.range = self
            .enclose_on(
                &self.source.request.domain,
                context.graph,
                context.arithmetic,
            )?
            .into_bounds()
            .into();
        context.graph.charge(0)?;
        Ok(())
    }
    /// Enclose a certified subdomain using retained global residuals and grids.
    /// The source domain is validated even for constant or zero-output maps.
    pub fn enclose_on(
        &self,
        domain: &[DyadicInterval],
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<TaylorRange, TaylorPolynomialError> {
        let mut previous = self.source.enclose_on(domain, graph, arithmetic)?;
        graph.charge(domain.len())?;
        let empty = domain.iter().any(DyadicInterval::is_empty);
        let shifts = if empty {
            Vec::new()
        } else {
            super::shifts(domain, &self.source.request.center, graph, arithmetic)?
        };
        for stage in self.stages.iter() {
            graph.charge(1)?;
            let outputs = stage.arity();
            graph.check_entries(&[shifts.len(), previous.len(), outputs, outputs])?;
            if empty {
                graph.charge(outputs)?;
                previous = vec![DyadicInterval::empty(); outputs];
                continue;
            }
            #[cfg(feature = "taylor-combination")]
            if let Some(join) = &stage.join {
                previous =
                    self.enclose_join(join, stage, previous, domain, &shifts, graph, arithmetic)?;
                continue;
            }
            #[cfg(feature = "taylor-composition")]
            let model = stage.model.as_ref();
            #[cfg(not(feature = "taylor-composition"))]
            let model: Option<&Arc<TaylorModel>> = None;
            let independent = if let Some(model) = model {
                // The full-range inclusion was proved at composition. Intersect
                // query enclosures with that retained domain before evaluation.
                let mut inputs = Vec::new();
                graph.check_entries(&[shifts.len(), previous.len(), outputs, previous.len()])?;
                for (slot, (input, domain)) in
                    previous.iter().zip(&model.request.domain).enumerate()
                {
                    graph.charge(1)?;
                    let input = input.intersection(domain, arithmetic)?;
                    if input.is_empty() {
                        return Err(TaylorError::InconsistentEnclosure { output: slot }.into());
                    }
                    inputs.push(input);
                }
                model
                    .source
                    .eval_dyadic(&inputs, model.request.precision, graph, arithmetic)?
            } else {
                stage.map.eval_dyadic(
                    &previous,
                    self.source.request.precision,
                    graph,
                    arithmetic,
                )?
            };
            let mut result = Vec::new();
            for (output, (value, natural)) in stage.outputs.iter().zip(independent).enumerate() {
                graph.charge(1)?;
                let polynomial = bound(&value.terms, &shifts, &self.source, graph, arithmetic)?;
                let enclosed = polynomial
                    .add(&value.remainder, arithmetic)?
                    .intersection(&natural, arithmetic)?;
                if enclosed.is_empty() {
                    return Err(TaylorError::InconsistentEnclosure { output }.into());
                }
                result.push(enclosed);
            }
            previous = result;
        }
        graph.charge(0)?;
        Ok(TaylorRange::new(previous, empty))
    }
}
fn validate(
    map: &ExpressionProgram,
    inputs: usize,
    graph: &mut ExpressionBudget,
) -> Result<Vec<usize>, TaylorPolynomialError> {
    graph.charge(0)?;
    let definition = map.definition();
    if definition.variables.len() != inputs {
        return Err(TaylorPolynomialError::InputCount {
            expected: inputs,
            actual: definition.variables.len(),
        });
    }
    graph.check_entries(&[
        definition.nodes.len(),
        definition.nodes.len(),
        definition.outputs.len(),
    ])?;
    let uses = map.operand_uses(graph)?;
    for (index, node) in definition.nodes.iter().enumerate() {
        graph.charge(1)?;
        if uses[index] == 0 {
            continue;
        }
        if !matches!(
            node,
            ExpressionNode::Constant(_)
                | ExpressionNode::Variable(_)
                | ExpressionNode::Unary {
                    operation: UnaryOperation::Neg,
                    ..
                }
                | ExpressionNode::Binary {
                    operation: BinaryOperation::Add | BinaryOperation::Sub | BinaryOperation::Mul,
                    ..
                }
        ) {
            return Err(TaylorPolynomialError::UnsupportedOperation { node: index });
        }
    }
    Ok(uses)
}
struct Context<'a> {
    source: &'a TaylorModel,
    shifts: Vec<DyadicInterval>,
    graph: &'a mut ExpressionBudget,
    arithmetic: &'a mut DyadicBudget,
    entries: usize,
}
impl Context<'_> {
    /// Exact source rows of an original model, before any map.
    fn rows(
        &mut self,
        model: &TaylorModel,
        count: usize,
    ) -> Result<Vec<Arc<Value>>, TaylorPolynomialError> {
        let mut inputs = Vec::new();
        #[allow(
            clippy::needless_range_loop,
            reason = "Clippy 1.100 extended this lint; the index loop is kept so generated code is unchanged"
        )]
        for output in 0..count {
            self.graph.charge(1)?;
            self.reserve(2)?;
            inputs.push(Arc::new(Value {
                terms: BTreeMap::new(),
                remainder: model.remainder[output].clone(),
                product: None,
            }));
        }
        for term in model.terms() {
            self.graph.charge(1)?;
            let request = &model.requests[term.request];
            // No map node has shared these newly constructed rows yet.
            let input =
                Arc::get_mut(&mut inputs[request.output]).expect("new input row has one owner");
            self.insert(&mut input.terms, &request.powers, term.coefficient.clone())?;
        }
        Ok(inputs)
    }
    /// Evaluate demanded polynomial-map nodes over retained input rows.
    fn evaluate(
        &mut self,
        map: &ExpressionProgram,
        uses: &[usize],
        inputs: &[Arc<Value>],
        values: &mut [Option<Arc<Value>>],
    ) -> Result<(), TaylorPolynomialError> {
        // Child indices are strictly earlier. The demanded set preserves lazy
        // source semantics; no unused operation is invoked by this traversal.
        for (index, node) in map.definition().nodes.iter().enumerate() {
            self.graph.charge(1)?;
            if uses[index] == 0 {
                continue;
            }
            let value = match node {
                ExpressionNode::Variable(_) => {
                    inputs[map.slots[index].expect("validated binding")].clone()
                }
                ExpressionNode::Constant(c) => {
                    let mut terms = BTreeMap::new();
                    self.insert(&mut terms, &[], c.clone())?;
                    self.reserve(2)?;
                    Arc::new(Value {
                        terms,
                        remainder: zero()?,
                        product: None,
                    })
                }
                ExpressionNode::Unary {
                    operation: UnaryOperation::Neg,
                    input,
                } => {
                    let input = values[*input].as_ref().expect("earlier demanded input");
                    Arc::new(self.linear(input, None, true)?)
                }
                ExpressionNode::Binary {
                    operation,
                    left,
                    right,
                } => {
                    let left = values[*left].as_ref().expect("earlier demanded left");
                    let right = values[*right].as_ref().expect("earlier demanded right");
                    Arc::new(match operation {
                        BinaryOperation::Add => self.linear(left, Some(right), false)?,
                        BinaryOperation::Sub => self.linear(left, Some(right), true)?,
                        BinaryOperation::Mul => self.multiply(left, right)?,
                        BinaryOperation::Div => unreachable!("validated polynomial map"),
                    })
                }
                _ => unreachable!("validated polynomial map"),
            };
            values[index] = Some(value);
        }
        Ok(())
    }
    fn reserve(&mut self, count: usize) -> Result<(), TaylorPolynomialError> {
        self.entries = self
            .entries
            .checked_add(count)
            .ok_or(ExpressionError::SizeExhausted)?;
        self.graph.check_entries(&[self.entries])?;
        self.graph.charge(count)?;
        Ok(())
    }
    fn insert(
        &mut self,
        terms: &mut TaylorPolynomialTerms,
        powers: &[(crate::SymbolId, usize)],
        coefficient: Real,
    ) -> Result<(), TaylorPolynomialError> {
        self.graph.charge(powers.len())?;
        self.graph.charge(1)?;
        if let Some(old) = terms.get_mut(powers) {
            *old = old.clone() + coefficient;
        } else {
            self.reserve(
                powers
                    .len()
                    .checked_add(2)
                    .ok_or(ExpressionError::SizeExhausted)?,
            )?;
            terms.insert(powers.to_vec(), coefficient);
        }
        Ok(())
    }
    fn linear(
        &mut self,
        left: &Value,
        right: Option<&Value>,
        negative: bool,
    ) -> Result<Value, TaylorPolynomialError> {
        let mut terms = BTreeMap::new();
        for (p, c) in &left.terms {
            self.insert(
                &mut terms,
                p,
                if negative && right.is_none() {
                    -c.clone()
                } else {
                    c.clone()
                },
            )?;
        }
        let remainder = if let Some(right) = right {
            for (p, c) in &right.terms {
                self.insert(&mut terms, p, if negative { -c.clone() } else { c.clone() })?;
            }
            if negative {
                left.remainder.sub(&right.remainder, self.arithmetic)?
            } else {
                left.remainder.add(&right.remainder, self.arithmetic)?
            }
        } else {
            left.remainder.neg(self.arithmetic)?
        };
        self.reserve(2)?;
        Ok(Value {
            terms,
            remainder,
            product: None,
        })
    }
    fn multiply(&mut self, left: &Value, right: &Value) -> Result<Value, TaylorPolynomialError> {
        let mut terms = BTreeMap::new();
        let mut discarded = BTreeMap::new();
        for (p, a) in &left.terms {
            for (q, b) in &right.terms {
                self.graph.charge(1)?;
                self.graph
                    .check_entries(&[self.entries, p.len(), q.len()])?;
                let mut powers = Vec::new();
                let (mut i, mut j, mut degree) = (0, 0, 0usize);
                while i < p.len() || j < q.len() {
                    self.graph.charge(1)?;
                    let term = if j == q.len() || (i < p.len() && p[i].0 < q[j].0) {
                        let v = p[i];
                        i += 1;
                        v
                    } else if i == p.len() || q[j].0 < p[i].0 {
                        let v = q[j];
                        j += 1;
                        v
                    } else {
                        let v = (
                            p[i].0,
                            p[i].1
                                .checked_add(q[j].1)
                                .ok_or(TaylorError::DegreeOverflow)?,
                        );
                        i += 1;
                        j += 1;
                        v
                    };
                    degree = degree
                        .checked_add(term.1)
                        .ok_or(TaylorError::DegreeOverflow)?;
                    powers.push(term);
                }
                let target = if degree > self.source.request.order {
                    &mut discarded
                } else {
                    &mut terms
                };
                self.insert(target, &powers, a * b)?;
            }
        }
        self.reserve(7)?;
        let truncation = bound(
            &discarded,
            &self.shifts,
            self.source,
            self.graph,
            self.arithmetic,
        )?;
        let p = bound(
            &left.terms,
            &self.shifts,
            self.source,
            self.graph,
            self.arithmetic,
        )?;
        let q = bound(
            &right.terms,
            &self.shifts,
            self.source,
            self.graph,
            self.arithmetic,
        )?;
        let left_cross = p.mul(&right.remainder, self.arithmetic)?;
        let right_cross = q.mul(&left.remainder, self.arithmetic)?;
        let residual_product = left.remainder.mul(&right.remainder, self.arithmetic)?;
        let remainder = truncation
            .add(&left_cross, self.arithmetic)?
            .add(&right_cross, self.arithmetic)?
            .add(&residual_product, self.arithmetic)?;
        Ok(Value {
            terms,
            remainder,
            product: Some(TaylorPolynomialProduct {
                discarded,
                truncation,
                left_cross,
                right_cross,
                residual_product,
            }),
        })
    }
}
fn bound(
    terms: &TaylorPolynomialTerms,
    shifts: &[DyadicInterval],
    source: &TaylorModel,
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Result<DyadicInterval, TaylorPolynomialError> {
    let mut result = zero()?;
    for (powers, coefficient) in terms {
        graph.charge(1)?;
        graph.charge(powers.len())?;
        graph.check_entries(&[powers.len(), shifts.len()])?;
        let request = CoefficientRequest {
            output: 0,
            powers: powers.clone(),
        };
        let coefficient =
            DyadicInterval::from_real(coefficient, source.request.precision, arithmetic)?;
        let (_, value) = super::contribution(
            &request,
            &coefficient,
            shifts,
            source.source.definition(),
            source.request.precision,
            graph,
            arithmetic,
        )?;
        result = result.add(&value, arithmetic)?;
    }
    Ok(result)
}

#[cfg(feature = "taylor-combination")]
mod combination;
#[cfg(feature = "taylor-composition")]
mod composition;
#[cfg(feature = "taylor-combination")]
pub use combination::TaylorOperand;
