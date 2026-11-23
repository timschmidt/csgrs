//! Retained affine uncertainty through checked trajectory Taylor steps.
use super::*;
use hyperinterval::SignSet;
use hypersolve::{
    SymbolRef,
    expression_program::{
        CompositionError, CompositionProgram, TaylorAffineError, TaylorAffineMap,
        TaylorAffineModel, TaylorAffineRow, TaylorNoiseModel, TaylorRequest,
    },
};

/// Invalid initial time or an original construction/proof/resource diagnostic.
#[derive(Clone, Debug)]
pub enum FlowNoiseError {
    /// Initial time must be a nonempty exact singleton.
    NonPointTime,
    /// Original field, native arithmetic, graph or cancellation failure.
    Field(FieldError),
    /// Original trajectory proof failure, separately retained in the attempt.
    Flow(FlowTaylorError),
    /// Original source/parameter composition failure.
    Composition(CompositionError),
    /// Original finite polynomial regularity or remainder failure.
    Taylor(TaylorError),
    /// Original affine projection or retained-column failure.
    Affine(TaylorAffineError),
}
impl fmt::Display for FlowNoiseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "correlated flow endpoint: {self:?}")
    }
}
impl std::error::Error for FlowNoiseError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::NonPointTime => None,
            Self::Field(e) => Some(e),
            Self::Flow(e) => Some(e),
            Self::Composition(e) => Some(e),
            Self::Taylor(e) => Some(e),
            Self::Affine(e) => Some(e),
        }
    }
}
impl From<FieldError> for FlowNoiseError {
    fn from(e: FieldError) -> Self {
        Self::Field(e)
    }
}
impl From<ExpressionError> for FlowNoiseError {
    fn from(e: ExpressionError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalError> for FlowNoiseError {
    fn from(e: IntervalError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalLatticeError> for FlowNoiseError {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Field(e.into())
    }
}
impl From<CompositionError> for FlowNoiseError {
    fn from(e: CompositionError) -> Self {
        Self::Composition(e)
    }
}
impl From<TaylorError> for FlowNoiseError {
    fn from(e: TaylorError) -> Self {
        Self::Taylor(e)
    }
}
impl From<TaylorAffineError> for FlowNoiseError {
    fn from(e: TaylorAffineError) -> Self {
        Self::Affine(e)
    }
}

/// A complete checked endpoint, canonical empty endpoint, or explicit failure.
#[derive(Clone, Debug)]
pub enum FlowNoiseStatus {
    /// Both trajectory and affine endpoint proofs completed.
    Certified,
    /// Validated empty initial product; no trajectory existence claim.
    Empty,
    /// No correlated endpoint is published for this attempt.
    Rejected(FlowNoiseError),
}

#[derive(Clone, Debug)]
struct Form<const N: usize> {
    center: [Real; N],
    columns: Vec<[Real; N]>,
    domains: Vec<DyadicInterval>,
    residual: [DyadicInterval; N],
}

/// Immutable affine outer set at an exact time, retaining its construction proof.
///
/// The set is center + sum(column * parameter) + a Cartesian residual. Parameters
/// range independently. This is an enclosure of reachable states, not equality
/// with the reachable set. Its full affine image supplies the next Picard input;
/// an independently tighter box never silently removes parameter combinations.
/// Rebuild from the original box/time, field definitions and step requests.
#[derive(Clone, Debug)]
pub struct FlowNoiseState<const N: usize> {
    time: DyadicInterval,
    form: Option<Form<N>>,
    enclosure: BoxN<DyadicInterval, N>,
    evidence: Option<Arc<FlowNoiseEvidence<N>>>,
}
impl<const N: usize> Drop for FlowNoiseState<N> {
    fn drop(&mut self) {
        // Unique history tails are released iteratively. A caller retaining any
        // intermediate state/evidence keeps that shared suffix alive unchanged.
        let mut link = self.evidence.take();
        while let Some(evidence) = link {
            let Ok(evidence) = Arc::try_unwrap(evidence) else {
                break;
            };
            let FlowNoiseEvidence { initial, .. } = evidence;
            let Ok(mut initial) = Arc::try_unwrap(initial) else {
                break;
            };
            link = initial.evidence.take();
        }
    }
}
impl<const N: usize> FlowNoiseState<N> {
    /// Adopt an exact initial Cartesian set, validating time before empty shortcuts.
    /// This construction makes no trajectory existence claim.
    pub fn new(
        time: DyadicInterval,
        initial: BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<Self, FlowNoiseError> {
        budget.graph.charge(0)?;
        if !time.is_singleton() {
            return Err(FlowNoiseError::NonPointTime);
        }
        let form = if let Some(lanes) = initial.lanes() {
            let entries = N.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
            budget.graph.check_entries(&[entries, N, N, N])?;
            budget.graph.charge(entries)?;
            budget
                .graph
                .charge(N.checked_mul(3).ok_or(ExpressionError::SizeExhausted)?)?;
            Some(Form {
                center: std::array::from_fn(|_| Real::zero()),
                columns: (0..N)
                    .map(|column| std::array::from_fn(|row| Real::from(u8::from(row == column))))
                    .collect(),
                domains: lanes.to_vec(),
                residual: std::array::from_fn(|_| zero()),
            })
        } else {
            None
        };
        budget.graph.charge(0)?;
        Ok(Self {
            time,
            form,
            enclosure: initial,
            evidence: None,
        })
    }
    /// Exact time at which this set is an initial or certified outer endpoint set.
    pub fn time(&self) -> &DyadicInterval {
        &self.time
    }
    /// Enclosure of every independent affine parameter combination, not an intersection.
    pub fn enclosure(&self) -> &BoxN<DyadicInterval, N> {
        &self.enclosure
    }
    /// Exact center, absent for canonical empty.
    pub fn center(&self) -> Option<&[Real; N]> {
        self.form.as_ref().map(|f| &f.center)
    }
    /// Exact shared coefficient columns, absent for canonical empty.
    pub fn columns(&self) -> Option<&[[Real; N]]> {
        self.form.as_ref().map(|f| f.columns.as_slice())
    }
    /// Independent parameter domains corresponding to the retained columns.
    pub fn domains(&self) -> Option<&[DyadicInterval]> {
        self.form.as_ref().map(|f| f.domains.as_slice())
    }
    /// Complete nonlinear, discarded-column and trajectory residual bounds.
    pub fn residual(&self) -> Option<&[DyadicInterval; N]> {
        self.form.as_ref().map(|f| &f.residual)
    }
    /// Previous checked step, absent only for an authored initial box.
    pub fn evidence(&self) -> Option<&Arc<FlowNoiseEvidence<N>>> {
        self.evidence.as_ref()
    }
}

/// Checked-step request; the initial time and set come from the retained state.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FlowNoiseRequest<const N: usize> {
    /// Exact singleton end time, strictly after the state's time.
    pub end: DyadicInterval,
    /// Proposed full trajectory box, verified by the existing Picard owner.
    pub candidate: BoxN<DyadicInterval, N>,
    /// Maximum retained nonconstant affine columns; zero charges all to residuals.
    pub maximum_noise: usize,
    /// Round the endpoint center and columns down onto the policy grid.
    ///
    /// Each rounding error, scaled by its parameter domain, joins its row's
    /// residual, which is then enclosed outward on the same grid. Values already
    /// on the grid are kept exactly. This bounds coefficient sizes over long
    /// nonlinear horizons; `false` keeps every coefficient exact.
    #[cfg_attr(feature = "serde", serde(default))]
    pub round_coefficients: bool,
    /// Fold retained columns whose certified variation is below `2^-bits`
    /// times the largest into residuals; `None` keeps up to `maximum_noise`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub column_cutoff_bits: Option<u32>,
}

/// Provenance of one independently ranged input coordinate for the next step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FlowNoiseInput {
    /// Index of a retained column in the previous state.
    Retained(usize),
    /// State coordinate whose previous residual becomes a fresh independent input.
    Residual(usize),
}

/// Original sources and completed intermediate proofs, including failed attempts.
#[derive(Clone, Debug)]
pub struct FlowNoiseEvidence<const N: usize> {
    program: FlowTaylorProgram<N>,
    initial: Arc<FlowNoiseState<N>>,
    request: FlowNoiseRequest<N>,
    flow: Option<FlowTaylorReport<N>>,
    polynomial: Option<ExpressionProgram>,
    composition: Option<CompositionProgram>,
    inputs: Vec<FlowNoiseInput>,
    noise: Option<Arc<TaylorNoiseModel>>,
    status: FlowNoiseStatus,
}
impl<const N: usize> FlowNoiseEvidence<N> {
    /// Prepared trajectory source and order.
    pub fn program(&self) -> &FlowTaylorProgram<N> {
        &self.program
    }
    /// Retained initial affine set and all prior shared proof nodes.
    pub fn initial(&self) -> &Arc<FlowNoiseState<N>> {
        &self.initial
    }
    /// Original end time, candidate and retention capacity.
    pub fn request(&self) -> &FlowNoiseRequest<N> {
        &self.request
    }
    /// Original Picard/Taylor attempt, absent if bookkeeping failed first.
    pub fn flow(&self) -> Option<&FlowTaylorReport<N>> {
        self.flow.as_ref()
    }
    /// Exact finite endpoint polynomial, including all original source guards.
    pub fn polynomial(&self) -> Option<&ExpressionProgram> {
        self.polynomial.as_ref()
    }
    /// Source-bound substitution of the complete affine outer input.
    pub fn composition(&self) -> Option<&CompositionProgram> {
        self.composition.as_ref()
    }
    /// Input coordinate meanings in the composition's binding order.
    pub fn inputs(&self) -> &[FlowNoiseInput] {
        &self.inputs
    }
    /// Parameter Taylor model, selected columns and all omitted contributions.
    pub fn noise(&self) -> Option<&Arc<TaylorNoiseModel>> {
        self.noise.as_ref()
    }
    /// Completion or original failure; partial evidence is never success.
    pub fn status(&self) -> &FlowNoiseStatus {
        &self.status
    }
}

/// One complete or rejected attempt; only complete attempts expose a new state.
#[derive(Clone, Debug)]
pub struct FlowNoiseReport<const N: usize> {
    evidence: Arc<FlowNoiseEvidence<N>>,
    end: Option<Arc<FlowNoiseState<N>>>,
}
impl<const N: usize> FlowNoiseReport<N> {
    /// All original and generated proof stages, including failed stages.
    pub fn evidence(&self) -> &Arc<FlowNoiseEvidence<N>> {
        &self.evidence
    }
    /// Whole-step result.
    pub fn status(&self) -> &FlowNoiseStatus {
        &self.evidence.status
    }
    /// Certified outer state, directly consumable by a subsequent checked step.
    pub fn end_state(&self) -> Option<&Arc<FlowNoiseState<N>>> {
        self.end.as_ref()
    }
}

impl<const N: usize> FlowTaylorProgram<N> {
    /// Certify a trajectory and retain affine uncertainty for subsequent steps.
    ///
    /// One caller budget covers all stages. Structural and dyadic work/entry
    /// limits apply; scalar payloads/refinement, total heap and within-scalar
    /// cancellation retain EnclosureBudget's exclusions. Prior proof storage is
    /// shared. Failed attempts leave the input unchanged for a fresh-budget retry.
    pub fn certify_noise_step(
        &self,
        initial: Arc<FlowNoiseState<N>>,
        request: FlowNoiseRequest<N>,
        budget: &mut EnclosureBudget,
    ) -> FlowNoiseReport<N> {
        certify_from(self, initial, request, budget, |e, budget| {
            Ok(e.program.certify_step(
                PicardProposal {
                    start: e.initial.time.clone(),
                    end: e.request.end.clone(),
                    initial: e.initial.enclosure.clone(),
                    candidate: e.request.candidate.clone(),
                },
                budget,
            ))
        })
    }
}

fn certify_from<const N: usize>(
    program: &FlowTaylorProgram<N>,
    initial: Arc<FlowNoiseState<N>>,
    request: FlowNoiseRequest<N>,
    budget: &mut EnclosureBudget,
    prove: impl FnOnce(
        &FlowNoiseEvidence<N>,
        &mut EnclosureBudget,
    ) -> Result<FlowTaylorReport<N>, FlowNoiseError>,
) -> FlowNoiseReport<N> {
    let mut evidence = FlowNoiseEvidence {
        program: program.clone(),
        initial,
        request,
        flow: None,
        polynomial: None,
        composition: None,
        inputs: Vec::new(),
        noise: None,
        status: FlowNoiseStatus::Empty,
    };
    let result = advance(&mut evidence, budget, prove);
    evidence.status = match &result {
        Ok((None, _)) => FlowNoiseStatus::Empty,
        Ok(_) => FlowNoiseStatus::Certified,
        Err(e) => FlowNoiseStatus::Rejected(e.clone()),
    };
    let evidence = Arc::new(evidence);
    let end = result.ok().map(|(form, enclosure)| {
        Arc::new(FlowNoiseState {
            time: evidence.request.end.clone(),
            form,
            enclosure,
            evidence: Some(evidence.clone()),
        })
    });
    FlowNoiseReport { evidence, end }
}

fn zero() -> DyadicInterval {
    DyadicInterval::point(0.into(), 0).expect("zero")
}
fn real_point(
    value: &DyadicInterval,
    budget: &mut EnclosureBudget,
) -> Result<Real, FlowNoiseError> {
    let rational = value.to_rational(&mut budget.arithmetic)?;
    Ok(Real::from(
        rational.lower().expect("checked singleton").clone(),
    ))
}

// Only the trajectory sum and affine input are assembled here. Differentiation,
// composition, regularity, Taylor remainder and selection stay in Hypersolve.
fn node(
    definition: &mut ExpressionDefinition,
    value: ExpressionNode,
    budget: &mut EnclosureBudget,
) -> Result<usize, FlowNoiseError> {
    let next = definition
        .nodes
        .len()
        .checked_add(1)
        .ok_or(ExpressionError::SizeExhausted)?;
    budget
        .graph
        .check_entries(&[next, definition.variables.len(), definition.outputs.len()])?;
    budget.graph.charge(1)?;
    definition.nodes.push(value);
    Ok(next - 1)
}
fn binary(
    definition: &mut ExpressionDefinition,
    operation: BinaryOperation,
    left: usize,
    right: usize,
    budget: &mut EnclosureBudget,
) -> Result<usize, FlowNoiseError> {
    node(
        definition,
        ExpressionNode::Binary {
            operation,
            left,
            right,
        },
        budget,
    )
}

fn endpoint_polynomial<const N: usize>(
    program: &FlowTaylorProgram<N>,
    h: &DyadicInterval,
    budget: &mut EnclosureBudget,
) -> Result<(ExpressionProgram, [usize; N]), FlowNoiseError> {
    let source = program
        .stages
        .last()
        .expect("remainder stage")
        .program
        .definition();
    budget.graph.check_entries(&[
        source.nodes.len(),
        source.outputs.len(),
        source.variables.len(),
        N,
    ])?;
    budget.graph.charge(source.nodes.len())?;
    budget.graph.charge(source.outputs.len())?;
    budget.graph.charge(source.variables.len())?;
    let mut definition = source.clone();
    let mut sums = [0; N];
    for (row, variable) in program.field.identity().state().iter().enumerate() {
        sums[row] = node(
            &mut definition,
            ExpressionNode::Variable(SymbolRef::new(*variable, None)),
            budget,
        )?;
    }
    let mut power = DyadicInterval::point(1.into(), 0)?;
    for stage in program.stages.iter().take(program.order) {
        budget.graph.charge(1)?;
        power = power.mul(h, &mut budget.arithmetic)?;
        let weight = node(
            &mut definition,
            ExpressionNode::Constant(real_point(&power, budget)?),
            budget,
        )?;
        for (row, sum) in sums.iter_mut().enumerate() {
            let value = source.outputs[stage.outputs[row]];
            let term = binary(&mut definition, BinaryOperation::Mul, weight, value, budget)?;
            *sum = binary(&mut definition, BinaryOperation::Add, *sum, term, budget)?;
        }
    }
    let first = definition.outputs.len();
    let outputs = first.checked_add(N).ok_or(ExpressionError::SizeExhausted)?;
    budget
        .graph
        .check_entries(&[definition.nodes.len(), outputs, N])?;
    budget.graph.charge(N)?;
    definition.outputs.extend(sums);
    Ok((
        ExpressionProgram::from_definition(definition, &mut budget.graph)?,
        std::array::from_fn(|i| first + i),
    ))
}

fn advance<const N: usize>(
    e: &mut FlowNoiseEvidence<N>,
    budget: &mut EnclosureBudget,
    prove: impl FnOnce(
        &FlowNoiseEvidence<N>,
        &mut EnclosureBudget,
    ) -> Result<FlowTaylorReport<N>, FlowNoiseError>,
) -> Result<(Option<Form<N>>, BoxN<DyadicInterval, N>), FlowNoiseError> {
    budget.graph.charge(0)?;
    budget.graph.check_entries(&[N, N, N])?;
    budget.graph.charge(
        N.checked_mul(2)
            .and_then(|v| v.checked_add(2))
            .ok_or(ExpressionError::SizeExhausted)?,
    )?;
    e.flow = Some(prove(e, budget)?);
    let flow = e.flow.as_ref().expect("stored attempt");
    match flow.status() {
        FlowTaylorStatus::Rejected(error) => return Err(FlowNoiseError::Flow(error.clone())),
        FlowTaylorStatus::Empty => {
            let empty = BoxN::empty()?;
            budget.graph.charge(0)?;
            return Ok((None, empty));
        }
        FlowTaylorStatus::Certified => {}
    }
    let h = flow
        .picard()
        .evidence()
        .duration()
        .expect("certified duration");
    let (polynomial, selected) = endpoint_polynomial(&e.program, h, budget)?;
    e.polynomial = Some(polynomial);
    let form = e.initial.form.as_ref().expect("nonempty state");
    let count = form
        .columns
        .len()
        .checked_add(N)
        .ok_or(ExpressionError::SizeExhausted)?;
    let entries = count.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
    budget.graph.check_entries(&[count, count, entries, N])?;
    let mut domains = Vec::new();
    for (i, domain) in form.domains.iter().enumerate() {
        budget.graph.charge(2)?;
        domains.push(domain.clone());
        e.inputs.push(FlowNoiseInput::Retained(i));
    }
    for (row, residual) in form.residual.iter().enumerate() {
        budget.graph.charge(1)?;
        if residual.signs() != SignSet::Zero {
            budget.graph.charge(2)?;
            domains.push(residual.clone());
            e.inputs.push(FlowNoiseInput::Residual(row));
        }
    }
    let mut definition = ExpressionDefinition {
        variables: Vec::new(),
        nodes: Vec::new(),
        outputs: Vec::new(),
    };
    let mut inputs = Vec::new();
    for i in 0..domains.len() {
        budget.graph.charge(2)?;
        let variable = SymbolId(u32::try_from(i).map_err(|_| ExpressionError::SizeExhausted)?);
        definition.variables.push(variable);
        inputs.push(node(
            &mut definition,
            ExpressionNode::Variable(SymbolRef::new(variable, None)),
            budget,
        )?);
    }
    let mut values = [0; N];
    for (row, slot) in values.iter_mut().enumerate() {
        let mut value = node(
            &mut definition,
            ExpressionNode::Constant(form.center[row].clone()),
            budget,
        )?;
        for (column, kind) in e.inputs.iter().enumerate() {
            budget.graph.charge(1)?;
            let coefficient = match kind {
                FlowNoiseInput::Retained(index) => form.columns[*index][row].clone(),
                FlowNoiseInput::Residual(axis) => Real::from(u8::from(*axis == row)),
            };
            let weight = node(
                &mut definition,
                ExpressionNode::Constant(coefficient),
                budget,
            )?;
            let term = binary(
                &mut definition,
                BinaryOperation::Mul,
                weight,
                inputs[column],
                budget,
            )?;
            value = binary(&mut definition, BinaryOperation::Add, value, term, budget)?;
        }
        *slot = value;
    }
    let start = node(
        &mut definition,
        ExpressionNode::Constant(real_point(&e.initial.time, budget)?),
        budget,
    )?;
    // Lie coefficient construction preserves the field's validated binding map.
    for input in &e.program.field.data.inputs {
        budget.graph.charge(1)?;
        let output = match input {
            Input::Time => start,
            Input::State(row) => values[*row],
        };
        budget.graph.check_entries(&[
            definition.nodes.len(),
            definition
                .outputs
                .len()
                .checked_add(1)
                .ok_or(ExpressionError::SizeExhausted)?,
            definition.variables.len(),
        ])?;
        definition.outputs.push(output);
    }
    let inner = ExpressionProgram::from_definition(definition, &mut budget.graph)?;
    budget.graph.charge(inner.definition().outputs.len())?;
    let bindings: Vec<_> = (0..inner.definition().outputs.len()).collect();
    e.composition = Some(e.polynomial.as_ref().expect("stored polynomial").compose(
        &inner,
        &bindings,
        &mut budget.graph,
    )?);
    let composition = e.composition.as_ref().expect("stored composition");
    let mut center = Vec::new();
    for domain in &domains {
        budget.graph.charge(1)?;
        center.push(domain.midpoint(&mut budget.arithmetic)?);
    }
    let model = Arc::new(composition.program().taylor_model(
        TaylorRequest {
            center,
            domain: domains,
            order: 1,
            precision: budget.policy.precision,
        },
        &mut budget.graph,
        &mut budget.arithmetic,
    )?);
    let input_count = composition.program().definition().outputs.len();
    let map_entries = input_count
        .checked_mul(N)
        .ok_or(ExpressionError::SizeExhausted)?;
    budget.graph.check_entries(&[map_entries, N])?;
    budget.graph.charge(map_entries)?;
    budget.graph.charge(N)?;
    let map = TaylorAffineMap {
        input_count,
        rows: selected
            .iter()
            .map(|&output| {
                let selected = composition
                    .composed_output(output)
                    .expect("endpoint output");
                TaylorAffineRow {
                    coefficients: (0..input_count)
                        .map(|i| Real::from(u8::from(i == selected)))
                        .collect(),
                    offset: Real::zero(),
                }
            })
            .collect(),
    };
    let affine = Arc::new(TaylorAffineModel::new(
        model,
        map,
        &mut budget.graph,
        &mut budget.arithmetic,
    )?);
    let mut noise = TaylorNoiseModel::new(
        affine.clone(),
        e.request.maximum_noise,
        &mut budget.graph,
        &mut budget.arithmetic,
    )?;
    // Columns arrive in descending certified variation. Rebuilding with the
    // significant prefix as capacity selects those columns again and folds
    // the rest into residuals through the original omission proof.
    if let (Some(bits), Some(first)) = (e.request.column_cutoff_bits, noise.columns().first()) {
        let exponent = i32::try_from(bits).map_err(|_| ExpressionError::SizeExhausted)?;
        let cutoff = first.variation().mul(
            &DyadicInterval::point(1.into(), -exponent)?,
            &mut budget.arithmetic,
        )?;
        let mut kept = 0;
        for column in noise.columns() {
            budget.graph.charge(1)?;
            if column
                .variation()
                .compare_singleton(&cutoff, &mut budget.arithmetic)?
                == core::cmp::Ordering::Less
            {
                break;
            }
            kept += 1;
        }
        if kept < noise.columns().len() {
            noise = TaylorNoiseModel::new(affine, kept, &mut budget.graph, &mut budget.arithmetic)?;
        }
    }
    e.noise = Some(Arc::new(noise));
    let noise = e.noise.as_ref().expect("stored noise proof");
    let constant = noise
        .source()
        .terms()
        .iter()
        .find(|t| t.powers().is_empty())
        .expect("constant term");
    budget.graph.charge(N)?;
    let mut result = Form {
        center: std::array::from_fn(|i| constant.coefficients()[i].clone()),
        columns: Vec::new(),
        domains: Vec::new(),
        residual: std::array::from_fn(|_| zero()),
    };
    budget.graph.check_entries(&[
        noise
            .columns()
            .len()
            .checked_mul(N)
            .ok_or(ExpressionError::SizeExhausted)?,
        noise.columns().len(),
        N,
        N,
        N,
    ])?;
    for column in noise.columns() {
        budget
            .graph
            .charge(N.checked_add(1).ok_or(ExpressionError::SizeExhausted)?)?;
        let term = &noise.source().terms()[column.term_index()];
        result
            .columns
            .push(std::array::from_fn(|i| term.coefficients()[i].clone()));
        result.domains.push(column.domain().clone());
    }
    let remainder = flow
        .evidence()
        .end_terms()
        .last()
        .expect("full trajectory remainder")
        .lanes()
        .expect("nonempty remainder");
    for (row, remainder) in remainder.iter().enumerate() {
        budget.graph.charge(1)?;
        result.residual[row] = noise.remainder()[row].add(remainder, &mut budget.arithmetic)?;
    }
    if e.request.round_coefficients {
        let cells = result
            .columns
            .len()
            .checked_add(1)
            .and_then(|v| v.checked_mul(N))
            .ok_or(ExpressionError::SizeExhausted)?;
        budget.graph.check_entries(&[cells])?;
        budget.graph.charge(cells)?;
        for (row, residual) in result.residual.iter_mut().enumerate() {
            round_down(&mut result.center[row], None, residual, budget)?;
            for (column, domain) in result.columns.iter_mut().zip(&result.domains) {
                round_down(&mut column[row], Some(domain), residual, budget)?;
            }
            *residual = residual.regrid(budget.policy.precision, &mut budget.arithmetic)?;
        }
    }
    budget.graph.check_entries(&[N, N, N, N])?;
    let mut enclosure = enclose_coefficients(&result.center, budget)?;
    for (column, domain) in result.columns.iter().zip(&result.domains) {
        budget.graph.charge(N)?;
        let coefficient = enclose_coefficients(column, budget)?;
        let contribution = coefficient.scale(domain, &mut budget.arithmetic)?;
        enclosure = enclosure.add(&contribution, &mut budget.arithmetic)?;
    }
    budget.graph.charge(N)?;
    let residual = IntervalVector::new(result.residual.clone())?;
    enclosure = enclosure.add(&residual, &mut budget.arithmetic)?;
    budget.graph.charge(0)?;
    Ok((Some(result), enclosure))
}

// The set c + sum(a_k p_k) + r, with p_k in D_k, lies inside
// c' + sum(a'_k p_k) + (r + [0, c - c'] + sum([0, a_k - a'_k] D_k)) when each
// primed value is the grid point at or below its exact value. On-grid values
// keep their exact source.
fn round_down(
    value: &mut Real,
    domain: Option<&DyadicInterval>,
    residual: &mut DyadicInterval,
    budget: &mut EnclosureBudget,
) -> Result<(), FlowNoiseError> {
    let enclosure =
        DyadicInterval::from_real(value, budget.policy.precision, &mut budget.arithmetic)?;
    let Some((lower, upper, exponent)) = enclosure.bounds() else {
        return Ok(());
    };
    if lower == upper {
        return Ok(());
    }
    let mut error = DyadicInterval::new(0.into(), upper - lower, exponent)?;
    if let Some(domain) = domain {
        error = error.mul(domain, &mut budget.arithmetic)?;
    }
    *residual = residual.add(&error, &mut budget.arithmetic)?;
    *value = real_point(&DyadicInterval::point(lower.clone(), exponent)?, budget)?;
    Ok(())
}

// Exact scalar conversion only; linear images use the existing native vector owner.
fn enclose_coefficients<const N: usize>(
    values: &[Real; N],
    budget: &mut EnclosureBudget,
) -> Result<IntervalVector<DyadicInterval, N>, FlowNoiseError> {
    budget.graph.check_entries(&[N])?;
    let mut native = Vec::new();
    for value in values {
        budget.graph.charge(1)?;
        native.push(DyadicInterval::from_real(
            value,
            budget.policy.precision,
            &mut budget.arithmetic,
        )?);
    }
    Ok(IntervalVector::new(
        native.try_into().expect("N coefficients"),
    )?)
}

/// Automatic bounded horizons retaining affine endpoint uncertainty.
#[cfg(feature = "flow-noise-evolution")]
pub mod evolution;
