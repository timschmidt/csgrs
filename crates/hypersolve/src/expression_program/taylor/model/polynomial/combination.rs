//! Arithmetic across independently built Taylor models over shared parameters.
use super::composition::expansion;
use super::*;

/// A certified model joined onto another model's rows.
///
/// Its exact polynomial is re-expanded about the receiving model's center, and
/// its complete remainder is retained. Shared parameters are matched by symbol.
#[derive(Clone, Debug)]
pub enum TaylorOperand {
    /// An original source certificate.
    Model(Arc<TaylorModel>),
    /// A generated model, including its whole stage history.
    Polynomial(TaylorPolynomialModel),
}
impl From<Arc<TaylorModel>> for TaylorOperand {
    fn from(model: Arc<TaylorModel>) -> Self {
        Self::Model(model)
    }
}
impl From<TaylorPolynomialModel> for TaylorOperand {
    fn from(model: TaylorPolynomialModel) -> Self {
        Self::Polynomial(model)
    }
}
impl TaylorOperand {
    fn base(&self) -> &TaylorModel {
        match self {
            Self::Model(model) => model,
            Self::Polynomial(model) => &model.source,
        }
    }
    fn outputs(&self) -> usize {
        match self {
            Self::Model(model) => model.image().len(),
            Self::Polynomial(model) => model.range.len(),
        }
    }
    fn enclose_on(
        &self,
        domain: &[DyadicInterval],
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Vec<DyadicInterval>, TaylorPolynomialError> {
        Ok(match self {
            Self::Model(model) => model.enclose_on(domain, graph, arithmetic)?,
            Self::Polynomial(model) => model.enclose_on(domain, graph, arithmetic)?.into_bounds(),
        })
    }
}
#[derive(Debug)]
pub(super) struct Join {
    pub(super) operand: TaylorOperand,
    // Receiving rows passed through before the operand's rows.
    inputs: usize,
    // Receiving parameter slot of each operand parameter.
    slots: Vec<usize>,
}
impl Join {
    pub(super) fn arity(&self) -> usize {
        self.inputs + self.operand.outputs()
    }
}

impl TaylorModel {
    /// Append another certified model's outputs after these outputs.
    ///
    /// Each operand parameter must be one of this model's parameters, matched by
    /// symbol, and this model's domain for it must lie inside the operand's
    /// domain. Parameters the operand does not use are unconstrained. The
    /// operand polynomial is re-expanded exactly about this center through the
    /// existing truncation/cross-remainder engine, then its complete residual
    /// is added. No derivatives are regenerated. Later maps can then add,
    /// subtract or multiply rows of both models. Replay both original
    /// definitions/requests and the ordered maps; serialized certificates are not
    /// accepted as proof.
    pub fn join(
        self: Arc<Self>,
        operand: impl Into<TaylorOperand>,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<TaylorPolynomialModel, TaylorPolynomialError> {
        let mut model = TaylorPolynomialModel {
            source: self,
            stages: Arc::from([]),
            range: Arc::from([]),
        };
        model.append_join(operand.into(), graph, arithmetic)?;
        Ok(model)
    }
}
impl TaylorPolynomialModel {
    /// Append another certified model's outputs after these outputs.
    /// See [`TaylorModel::join`] for domain, remainder and replay contracts.
    /// A failed attempt leaves this model and the operand unchanged.
    pub fn join(
        &self,
        operand: impl Into<TaylorOperand>,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Self, TaylorPolynomialError> {
        graph.charge(0)?;
        let mut model = self.clone();
        model.append_join(operand.into(), graph, arithmetic)?;
        Ok(model)
    }
    /// Retained operand of a join stage; None for other stages.
    /// The stage's `polynomial_map` is the operand's re-expansion graph.
    pub fn joined(&self, stage: usize) -> Option<&TaylorOperand> {
        Some(&self.stages.get(stage)?.join.as_ref()?.operand)
    }
    fn append_join(
        &mut self,
        operand: TaylorOperand,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<(), TaylorPolynomialError> {
        graph.charge(0)?;
        let inputs = self
            .stages
            .last()
            .map_or(self.source.image().len(), |s| s.arity());
        let parameters = &self.source.source.definition().variables;
        let base = operand.base();
        let variables = &base.source.definition().variables;
        graph.check_entries(&[variables.len(), parameters.len()])?;
        let mut slots = Vec::new();
        for (j, &variable) in variables.iter().enumerate() {
            graph.charge(parameters.len().max(1))?;
            let slot = parameters
                .iter()
                .position(|&p| p == variable)
                .ok_or(ExpressionError::UnboundVariable(variable))?;
            if !self.source.request.domain[slot].subset_of(&base.request.domain[j], arithmetic)? {
                return Err(TaylorError::OutsideDomain { slot: j }.into());
            }
            slots.push(slot);
        }
        let rows = match &operand {
            TaylorOperand::Model(_) => None,
            TaylorOperand::Polynomial(model) => model.stages.last().map(|s| s.outputs.as_slice()),
        };
        let outputs = operand.outputs();
        let map = expansion(base, rows, outputs, graph, arithmetic)?;
        let uses = validate(&map, variables.len(), graph)?;
        let history = self
            .stages
            .len()
            .checked_add(1)
            .ok_or(ExpressionError::SizeExhausted)?;
        let total = inputs
            .checked_add(outputs)
            .ok_or(ExpressionError::SizeExhausted)?;
        graph.check_entries(&[history, uses.len(), total])?;
        let mut context = Context {
            source: &self.source,
            shifts: Vec::new(),
            graph,
            arithmetic,
            entries: history,
        };
        context.reserve(uses.len())?;
        context.reserve(total)?;
        let mut values: Vec<Option<Arc<Value>>> = vec![None; uses.len()];
        let mut result = Vec::new();
        if !self.is_empty() {
            context.shifts = super::super::shifts(
                &self.source.request.domain,
                &self.source.request.center,
                context.graph,
                context.arithmetic,
            )?;
            context.reserve(context.shifts.len())?;
            let previous = match self.stages.last() {
                Some(stage) => stage.outputs.clone(),
                None => context.rows(&self.source, inputs)?,
            };
            // Each operand parameter x is c + (x - c) about the receiving center.
            let mut identity = Vec::new();
            for &slot in &slots {
                context.graph.charge(1)?;
                let center = self.source.request.center[slot].to_rational(context.arithmetic)?;
                let mut terms = BTreeMap::new();
                context.insert(
                    &mut terms,
                    &[],
                    Real::from(center.lower().expect("singleton center").clone()),
                )?;
                context.insert(&mut terms, &[(parameters[slot], 1)], Real::one())?;
                context.reserve(2)?;
                identity.push(Arc::new(Value {
                    terms,
                    remainder: zero()?,
                    product: None,
                }));
            }
            context.evaluate(&map, &uses, &identity, &mut values)?;
            let remainders: Vec<&DyadicInterval> = match &operand {
                TaylorOperand::Model(model) => model.remainder.iter().collect(),
                TaylorOperand::Polynomial(model) => model
                    .stages
                    .last()
                    .map(|s| s.outputs.iter().map(|v| &v.remainder).collect())
                    .unwrap_or_default(),
            };
            result.extend(previous);
            for (&node, remainder) in map.definition().outputs.iter().zip(remainders) {
                context.graph.charge(1)?;
                let value = values[node].as_ref().expect("demanded output");
                let mut terms = BTreeMap::new();
                for (powers, coefficient) in &value.terms {
                    context.insert(&mut terms, powers, coefficient.clone())?;
                }
                context.reserve(2)?;
                result.push(Arc::new(Value {
                    terms,
                    remainder: value.remainder.add(remainder, context.arithmetic)?,
                    product: None,
                }));
            }
        }
        context.graph.charge(history)?;
        let mut stages = self.stages.to_vec();
        stages.push(Arc::new(Stage {
            map,
            model: None,
            join: Some(Join {
                operand,
                inputs,
                slots,
            }),
            values,
            outputs: result,
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
    #[allow(clippy::too_many_arguments, reason = "private enclosure step")]
    pub(super) fn enclose_join(
        &self,
        join: &Join,
        stage: &Stage,
        mut previous: Vec<DyadicInterval>,
        domain: &[DyadicInterval],
        shifts: &[DyadicInterval],
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Vec<DyadicInterval>, TaylorPolynomialError> {
        graph.check_entries(&[join.slots.len(), join.arity()])?;
        graph.charge(join.slots.len())?;
        // The query lies in the receiving domain, which lies in the operand's.
        let query: Vec<_> = join.slots.iter().map(|&s| domain[s].clone()).collect();
        let natural = join.operand.enclose_on(&query, graph, arithmetic)?;
        for (offset, (value, natural)) in
            stage.outputs[join.inputs..].iter().zip(natural).enumerate()
        {
            graph.charge(1)?;
            let polynomial = bound(&value.terms, shifts, &self.source, graph, arithmetic)?;
            let enclosed = polynomial
                .add(&value.remainder, arithmetic)?
                .intersection(&natural, arithmetic)?;
            if enclosed.is_empty() {
                return Err(TaylorError::InconsistentEnclosure {
                    output: join.inputs + offset,
                }
                .into());
            }
            previous.push(enclosed);
        }
        Ok(previous)
    }
}
