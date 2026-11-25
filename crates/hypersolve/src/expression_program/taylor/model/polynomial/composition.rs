//! Substitute an already certified Taylor map through the existing polynomial engine.
use super::*;

impl TaylorModel {
    /// Compose a certified outer model with these ordered outputs.
    ///
    /// Every input image must fit the outer model's certified domain, including
    /// unused bindings. The inner center, domain and polynomial degree remain
    /// authoritative. No derivatives of either model are regenerated. The outer
    /// polynomial uses the existing truncation/cross-remainder engine, then its
    /// complete residual is added. Both source definitions and proofs remain
    /// shared. Replay both definitions/requests and this ordered composition;
    /// serialized certificates are not accepted as proof.
    ///
    /// Structural graph/stage limits and native arithmetic share caller budgets;
    /// scalar payload/refinement and total live-heap exclusions are unchanged.
    pub fn compose(
        self: Arc<Self>,
        outer: Arc<TaylorModel>,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<TaylorPolynomialModel, TaylorPolynomialError> {
        let mut model = TaylorPolynomialModel {
            source: self,
            stages: Arc::from([]),
            range: Arc::from([]),
        };
        model.append_model(outer, graph, arithmetic)?;
        Ok(model)
    }
}
impl TaylorModel {
    /// Compose an exact outer program through a certified model of it built over
    /// this model's certified image.
    ///
    /// The outer model's domain is the whole image, centered at its exact dyadic
    /// midpoint, at the given order and this model's grid. Division, roots and
    /// the other source primitives thus become model maps; a later `join` lets
    /// the outer program combine rows from independent models, for example `u/v`.
    /// Outer regularity failures, such as a reciprocal over an image containing
    /// zero, are returned as Taylor errors. `model_map` exposes the generated
    /// outer certificate; replay it from `outer` and this image.
    pub fn apply(
        self: Arc<Self>,
        outer: &ExpressionProgram,
        order: usize,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<TaylorPolynomialModel, TaylorPolynomialError> {
        let request = over(self.image(), order, self.request.precision, arithmetic)?;
        let model = Arc::new(outer.taylor_model(request, graph, arithmetic)?);
        self.compose(model, graph, arithmetic)
    }
}
// A request covering a certified image, centered at its exact midpoint.
fn over(
    image: &[DyadicInterval],
    order: usize,
    precision: i32,
    arithmetic: &mut DyadicBudget,
) -> Result<TaylorRequest, TaylorPolynomialError> {
    let mut center = Vec::new();
    for value in image {
        center.push(if value.is_empty() {
            DyadicInterval::point(0.into(), 0)?
        } else {
            value.midpoint(arithmetic)?
        });
    }
    Ok(TaylorRequest {
        center,
        domain: image.to_vec(),
        order,
        precision,
    })
}
impl TaylorPolynomialModel {
    /// Compose an exact outer program over this model's certified range.
    /// See [`TaylorModel::apply`] for the domain, center and replay contracts.
    pub fn apply(
        &self,
        outer: &ExpressionProgram,
        order: usize,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Self, TaylorPolynomialError> {
        let request = over(
            &self.range,
            order,
            self.source.request.precision,
            arithmetic,
        )?;
        let model = Arc::new(outer.taylor_model(request, graph, arithmetic)?);
        self.compose(model, graph, arithmetic)
    }
    /// Consume this generated model in another certified Taylor map.
    /// See [`TaylorModel::compose`] for domain, remainder and replay contracts.
    /// A failed attempt leaves this model and both source certificates unchanged.
    pub fn compose(
        &self,
        outer: Arc<TaylorModel>,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Self, TaylorPolynomialError> {
        graph.charge(0)?;
        let mut model = self.clone();
        model.append_model(outer, graph, arithmetic)?;
        Ok(model)
    }
    /// Retained outer certificate for a composed-model stage; None for a raw map.
    pub fn model_map(&self, stage: usize) -> Option<&Arc<TaylorModel>> {
        self.stages.get(stage)?.model.as_ref()
    }
    /// Generated polynomial graph whose node indices identify `product` evidence.
    /// For raw polynomial stages this is the same definition returned by `maps`.
    pub fn polynomial_map(&self, stage: usize) -> Option<&ExpressionProgram> {
        self.stages.get(stage).map(|s| &s.map)
    }
    fn append_model(
        &mut self,
        outer: Arc<TaylorModel>,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<(), TaylorPolynomialError> {
        graph.charge(0)?;
        let image = if self.stages.is_empty() {
            self.source.image()
        } else {
            &self.range
        };
        if outer.request.domain.len() != image.len() {
            return Err(TaylorPolynomialError::InputCount {
                expected: image.len(),
                actual: outer.request.domain.len(),
            });
        }
        if !self.is_empty() {
            for (slot, (input, domain)) in image.iter().zip(&outer.request.domain).enumerate() {
                graph.charge(1)?;
                if !input.subset_of(domain, arithmetic)? {
                    return Err(TaylorError::OutsideDomain { slot }.into());
                }
            }
        }
        let outputs = outer.source.definition().outputs.len();
        let map = expansion(&outer, None, outputs, graph, arithmetic)?;
        self.append(map, graph, arithmetic)?;
        let mut context = Context {
            source: &self.source,
            shifts: Vec::new(),
            graph,
            arithmetic,
            entries: self.stages.len(),
        };
        let stages = Arc::get_mut(&mut self.stages).expect("append creates a new stage list");
        let stage = Arc::get_mut(stages.last_mut().expect("appended stage"))
            .expect("new last stage is uniquely owned");
        context.reserve(stage.outputs.len())?;
        for (output, remainder) in stage.outputs.iter_mut().zip(outer.remainder()) {
            let mut terms = BTreeMap::new();
            for (powers, coefficient) in &output.terms {
                context.insert(&mut terms, powers, coefficient.clone())?;
            }
            let remainder = output.remainder.add(remainder, context.arithmetic)?;
            context.reserve(2)?;
            *output = Arc::new(Value {
                terms,
                remainder,
                product: None,
            });
        }
        context.reserve(1)?;
        stage.model = Some(outer);
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
}

// Each row is a sum of exact coefficients times powers of (x - c) about the
// model's center, in its parameter order. Without explicit rows, the model's
// own exact terms are used.
pub(super) fn expansion(
    model: &TaylorModel,
    rows: Option<&[Arc<Value>]>,
    outputs: usize,
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Result<ExpressionProgram, TaylorPolynomialError> {
    let source = model.source.definition();
    graph.check_entries(&[source.variables.len(), source.variables.len(), outputs])?;
    graph.charge(source.variables.len())?;
    let mut builder = Builder {
        definition: ExpressionDefinition {
            variables: source.variables.clone(),
            nodes: Vec::new(),
            outputs: Vec::new(),
        },
        outputs,
        budget: graph,
    };
    let zero = builder.push(ExpressionNode::Constant(Real::zero()))?;
    builder.budget.charge(outputs)?;
    builder.definition.outputs = vec![zero; outputs];
    let mut shifts = BTreeMap::new();
    for (&variable, center) in source.variables.iter().zip(&model.request.center) {
        builder.budget.charge(1)?;
        let input = builder.push(ExpressionNode::Variable(crate::SymbolRef::new(
            variable, None,
        )))?;
        let rational = center.to_rational(arithmetic)?;
        let center = builder.push(ExpressionNode::Constant(Real::from(
            rational.lower().expect("point center").clone(),
        )))?;
        let shifted = builder.push(ExpressionNode::Binary {
            operation: BinaryOperation::Sub,
            left: input,
            right: center,
        })?;
        shifts.insert(variable, shifted);
    }
    let mut term = |output: usize,
                    powers: &[(crate::SymbolId, usize)],
                    coefficient: &Real|
     -> Result<(), ExpressionError> {
        builder.budget.charge(1)?;
        let mut value = builder.push(ExpressionNode::Constant(coefficient.clone()))?;
        for &(variable, mut exponent) in powers {
            let mut power = shifts[&variable];
            // Exact graph exponentiation; each emitted multiply spends work and
            // will be certified by the existing polynomial remainder engine.
            while exponent != 0 {
                builder.budget.charge(1)?;
                if exponent & 1 != 0 {
                    value = builder.push(ExpressionNode::Binary {
                        operation: BinaryOperation::Mul,
                        left: value,
                        right: power,
                    })?;
                }
                exponent >>= 1;
                if exponent != 0 {
                    power = builder.push(ExpressionNode::Binary {
                        operation: BinaryOperation::Mul,
                        left: power,
                        right: power,
                    })?;
                }
            }
        }
        let left = builder.definition.outputs[output];
        builder.definition.outputs[output] = builder.push(ExpressionNode::Binary {
            operation: BinaryOperation::Add,
            left,
            right: value,
        })?;
        Ok(())
    };
    match rows {
        None => {
            for t in model.terms() {
                let request = &model.program.requests()[t.request];
                term(request.output, &request.powers, &t.coefficient)?;
            }
        }
        Some(rows) => {
            for (output, row) in rows.iter().enumerate() {
                for (powers, coefficient) in &row.terms {
                    term(output, powers, coefficient)?;
                }
            }
        }
    }
    Ok(ExpressionProgram::from_definition(
        builder.definition,
        builder.budget,
    )?)
}
