//! Taylor-mode recurrences for centered coefficients and remainder bounds.
//!
//! Every node becomes a dense truncated series whose coefficients are the
//! normalized partials D^alpha g(xi) / alpha! of that node. Variables are
//! X_i + h_i. The same exact Taylor-arithmetic recurrence of each analytic
//! primitive runs over two arithmetics:
//!
//! - outward dyadic intervals over the domain box, where degree zero is the
//!   primitive's certified interval image of the operand's degree-zero
//!   enclosure, so inclusion monotonicity keeps every coefficient sound for
//!   each xi in the box separately;
//! - exact Hyperreal values at the center, where every coefficient is the
//!   exact normalized partial itself.
//!
//! The recurrences only reuse lower degrees, so repeated symbolic
//! differentiation and its expression swell do not occur. An operand outside
//! a primitive's certified domain, a divisor or square-root enclosure meeting
//! zero, or a refused exact scalar operation makes that recurrence
//! unavailable instead of failing the model; the caller then uses the
//! generated partial formulas. Resource limits are still reported.
use super::*;
use crate::expression_program::dyadic::unary_image;
use hyperinterval::IntervalError;
use hyperreal::Problem;

/// Every normalized partial up to one total degree, per output.
pub(super) struct Bounds<V> {
    indices: Indices,
    outputs: Vec<Arc<Vec<V>>>,
    variables: Vec<SymbolId>,
}
impl<V> Bounds<V> {
    /// The requested normalized partial (an enclosure on the whole domain box,
    /// or the exact value at the center).
    pub(super) fn coefficient(&self, request: &CoefficientRequest) -> &V {
        let mut exponents = vec![0; self.variables.len()];
        for &(variable, power) in &request.powers {
            let slot = self
                .variables
                .iter()
                .position(|&v| v == variable)
                .expect("used variable");
            exponents[slot] = power;
        }
        &self.outputs[request.output][self.indices.lookup[&exponents]]
    }
}

/// Dense multi-indices of total degree at most the bound degree, by degree.
struct Indices {
    exponents: Vec<Vec<usize>>,
    lookup: BTreeMap<Vec<usize>, usize>,
    // For each multi-index alpha: (beta, alpha - beta) for every beta <= alpha.
    pairs: Vec<Vec<(usize, usize)>>,
}
impl Indices {
    fn new(dimensions: usize, degree: usize, graph: &mut ExpressionBudget) -> Result<Self, Stop> {
        let mut exponents = vec![vec![0; dimensions]];
        let mut frontier = vec![0];
        for _ in 0..degree {
            if frontier.is_empty() {
                break;
            }
            let mut next = Vec::new();
            for parent in frontier {
                let lowest = exponents[parent].iter().rposition(|&e| e > 0).unwrap_or(0);
                for slot in lowest..dimensions {
                    graph.check_entries(&[exponents.len(), dimensions])?;
                    graph.charge(dimensions.max(1))?;
                    let mut child = exponents[parent].clone();
                    child[slot] += 1;
                    next.push(exponents.len());
                    exponents.push(child);
                }
            }
            frontier = next;
        }
        let mut lookup = BTreeMap::new();
        for (index, exponent) in exponents.iter().enumerate() {
            graph.charge(1)?;
            lookup.insert(exponent.clone(), index);
        }
        let mut pairs = Vec::new();
        for alpha in &exponents {
            let mut list = Vec::new();
            let mut beta = vec![0; dimensions];
            loop {
                graph.check_entries(&[list.len(), dimensions])?;
                graph.charge(dimensions.max(1))?;
                let gamma: Vec<_> = alpha.iter().zip(&beta).map(|(a, b)| a - b).collect();
                list.push((lookup[&beta], lookup[&gamma]));
                let Some(slot) = (0..dimensions).find(|&slot| beta[slot] < alpha[slot]) else {
                    break;
                };
                beta[slot] += 1;
                beta[..slot].fill(0);
            }
            pairs.push(list);
        }
        Ok(Self {
            exponents,
            lookup,
            pairs,
        })
    }
    fn len(&self) -> usize {
        self.exponents.len()
    }
    // First coordinate along which alpha is differentiated.
    fn axis(&self, alpha: usize) -> usize {
        self.exponents[alpha]
            .iter()
            .position(|&e| e > 0)
            .expect("nonconstant index")
    }
}

enum Stop {
    // Domain, zero-divisor or exact scalar obstruction: use generated formulas.
    Unavailable,
    Error(TaylorError),
}
impl From<IntervalError> for Stop {
    fn from(error: IntervalError) -> Self {
        match error {
            IntervalError::ContainsZero
            | IntervalError::EmptyDomain
            | IntervalError::Pole
            | IntervalError::UnsupportedBranch => Self::Unavailable,
            error => Self::Error(error.into()),
        }
    }
}
impl From<ExpressionError> for Stop {
    fn from(error: ExpressionError) -> Self {
        Self::Error(error.into())
    }
}
impl From<Problem> for Stop {
    // The generated formulas replay the same scalar operation and report it.
    fn from(_: Problem) -> Self {
        Self::Unavailable
    }
}

type Series<F> = Vec<<F as Field>::Value>;
type Operation<F> =
    fn(&mut F, &<F as Field>::Value, &<F as Field>::Value) -> Result<<F as Field>::Value, Stop>;

/// Coefficient arithmetic of one recurrence instance.
trait Field {
    type Value: Clone;
    fn graph(&mut self) -> &mut ExpressionBudget;
    fn integer(&mut self, value: BigInt) -> Result<Self::Value, Stop>;
    fn real(&mut self, value: &Real) -> Result<Self::Value, Stop>;
    fn is_zero(value: &Self::Value) -> bool;
    fn add(&mut self, left: &Self::Value, right: &Self::Value) -> Result<Self::Value, Stop>;
    fn sub(&mut self, left: &Self::Value, right: &Self::Value) -> Result<Self::Value, Stop>;
    fn mul(&mut self, left: &Self::Value, right: &Self::Value) -> Result<Self::Value, Stop>;
    fn div(&mut self, left: &Self::Value, right: &Self::Value) -> Result<Self::Value, Stop>;
    fn neg(&mut self, value: &Self::Value) -> Result<Self::Value, Stop>;
    // Rounding after an accumulated sum; exact arithmetic keeps the value.
    fn tidy(&mut self, value: Self::Value) -> Result<Self::Value, Stop>;
    // The primitive's own image, which also enforces its domain guard.
    fn image(
        &mut self,
        operation: UnaryOperation,
        value: &Self::Value,
    ) -> Result<Self::Value, Stop>;
    fn ln10(&mut self) -> Result<Self::Value, Stop>;
}

struct Interval<'a> {
    precision: i32,
    graph: &'a mut ExpressionBudget,
    arithmetic: &'a mut DyadicBudget,
}
impl Field for Interval<'_> {
    type Value = DyadicInterval;
    fn graph(&mut self) -> &mut ExpressionBudget {
        self.graph
    }
    fn integer(&mut self, value: BigInt) -> Result<DyadicInterval, Stop> {
        Ok(DyadicInterval::point(value, 0)?)
    }
    fn real(&mut self, value: &Real) -> Result<DyadicInterval, Stop> {
        Ok(DyadicInterval::from_real(
            value,
            self.precision,
            self.arithmetic,
        )?)
    }
    fn is_zero(value: &DyadicInterval) -> bool {
        value.signs() == SignSet::Zero
    }
    fn add(
        &mut self,
        left: &DyadicInterval,
        right: &DyadicInterval,
    ) -> Result<DyadicInterval, Stop> {
        Ok(left.add(right, self.arithmetic)?)
    }
    fn sub(
        &mut self,
        left: &DyadicInterval,
        right: &DyadicInterval,
    ) -> Result<DyadicInterval, Stop> {
        Ok(left.sub(right, self.arithmetic)?)
    }
    fn mul(
        &mut self,
        left: &DyadicInterval,
        right: &DyadicInterval,
    ) -> Result<DyadicInterval, Stop> {
        Ok(left.mul(right, self.arithmetic)?)
    }
    fn div(
        &mut self,
        left: &DyadicInterval,
        right: &DyadicInterval,
    ) -> Result<DyadicInterval, Stop> {
        Ok(left.div(right, self.precision, self.arithmetic)?)
    }
    fn neg(&mut self, value: &DyadicInterval) -> Result<DyadicInterval, Stop> {
        Ok(value.neg(self.arithmetic)?)
    }
    fn tidy(&mut self, value: DyadicInterval) -> Result<DyadicInterval, Stop> {
        Ok(value.regrid(self.precision, self.arithmetic)?)
    }
    fn image(
        &mut self,
        operation: UnaryOperation,
        value: &DyadicInterval,
    ) -> Result<DyadicInterval, Stop> {
        unary_image(operation, value.clone(), self.precision, self.arithmetic)?
            .ok_or(Stop::Unavailable)
    }
    fn ln10(&mut self) -> Result<DyadicInterval, Stop> {
        let ten = DyadicInterval::point(10.into(), 0)?;
        unary_image(
            UnaryOperation::Ln,
            ten,
            self.precision.min(0),
            self.arithmetic,
        )?
        .ok_or(Stop::Unavailable)
    }
}

struct Exact<'a> {
    graph: &'a mut ExpressionBudget,
}
impl Field for Exact<'_> {
    type Value = Real;
    fn graph(&mut self) -> &mut ExpressionBudget {
        self.graph
    }
    fn integer(&mut self, value: BigInt) -> Result<Real, Stop> {
        Ok(Real::new(Rational::from_bigint(value)))
    }
    fn real(&mut self, value: &Real) -> Result<Real, Stop> {
        Ok(value.clone())
    }
    fn is_zero(value: &Real) -> bool {
        value.definitely_zero()
    }
    fn add(&mut self, left: &Real, right: &Real) -> Result<Real, Stop> {
        Ok(left.clone() + right.clone())
    }
    fn sub(&mut self, left: &Real, right: &Real) -> Result<Real, Stop> {
        Ok(left.clone() - right.clone())
    }
    fn mul(&mut self, left: &Real, right: &Real) -> Result<Real, Stop> {
        Ok(left.clone() * right.clone())
    }
    fn div(&mut self, left: &Real, right: &Real) -> Result<Real, Stop> {
        Ok((left.clone() / right.clone())?)
    }
    fn neg(&mut self, value: &Real) -> Result<Real, Stop> {
        Ok(-value.clone())
    }
    fn tidy(&mut self, value: Real) -> Result<Real, Stop> {
        Ok(value)
    }
    fn image(&mut self, operation: UnaryOperation, value: &Real) -> Result<Real, Stop> {
        Ok(operation.eval(value.clone())?)
    }
    fn ln10(&mut self) -> Result<Real, Stop> {
        Ok(Real::from(10).ln()?)
    }
}

/// Enclose all normalized partials of total degree at most `degree` over the
/// request domain, or `None` when some primitive's recurrence is unavailable.
pub(super) fn bounds(
    program: &ExpressionProgram,
    variables: &[SymbolId],
    request: &TaylorRequest,
    degree: usize,
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Result<Option<Bounds<DyadicInterval>>, TaylorError> {
    let field = Interval {
        precision: request.precision,
        graph,
        arithmetic,
    };
    solve(field, program, variables, &request.domain, degree)
}

/// Exact normalized partials of total degree at most `degree` at the center,
/// or `None` when an exact scalar operation is refused.
pub(super) fn exact(
    program: &ExpressionProgram,
    variables: &[SymbolId],
    center: &[Real],
    degree: usize,
    graph: &mut ExpressionBudget,
) -> Result<Option<Bounds<Real>>, TaylorError> {
    solve(Exact { graph }, program, variables, center, degree)
}

fn solve<F: Field>(
    mut field: F,
    program: &ExpressionProgram,
    variables: &[SymbolId],
    points: &[F::Value],
    degree: usize,
) -> Result<Option<Bounds<F::Value>>, TaylorError> {
    let indices = match Indices::new(variables.len(), degree, field.graph()) {
        Ok(indices) => indices,
        Err(Stop::Unavailable) => return Ok(None),
        Err(Stop::Error(error)) => return Err(error),
    };
    let mut recurrence = Recurrence {
        indices: &indices,
        field,
    };
    let outputs = match recurrence.run(program, Inputs::Centered { variables, points }) {
        Ok(outputs) => outputs,
        Err(Stop::Unavailable) => return Ok(None),
        Err(Stop::Error(error)) => return Err(error),
    };
    Ok(Some(Bounds {
        indices,
        outputs,
        variables: variables.to_vec(),
    }))
}

/// Variable series: centered coordinates X + h, or caller-supplied series.
#[derive(Clone, Copy)]
enum Inputs<'a, V> {
    Centered {
        variables: &'a [SymbolId],
        points: &'a [V],
    },
    // One complete series per binding slot, of the indices' length.
    Series(&'a [Vec<V>]),
}

/// Taylor-mode ODE coefficients x_0, ..., x_degree of x' = f(t, x) over boxes.
///
/// `slots[i]` names binding slot i as time (`None`) or state lane `Some(j)`.
/// Output j of the program is f_j. With t = start + s and x(s) = sum x_k s^k,
/// x_(k+1) = [f(t, x)]_k / (k + 1), where [.]_k is the univariate Taylor-mode
/// coefficient. Each enclosure contains x^(k)(tau)/k! for every solution with
/// x(tau) in `initial` and tau in `start`. `None` when a recurrence is unavailable.
#[allow(clippy::too_many_arguments)]
pub(crate) fn ode(
    program: &ExpressionProgram,
    slots: &[Option<usize>],
    start: &DyadicInterval,
    initial: &[DyadicInterval],
    degree: usize,
    precision: i32,
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Result<Option<Vec<Vec<DyadicInterval>>>, TaylorError> {
    let mut field = Interval {
        precision,
        graph,
        arithmetic,
    };
    let lanes = initial.len();
    let mut x: Vec<Vec<DyadicInterval>> = initial.iter().map(|v| vec![v.clone()]).collect();
    for k in 1..=degree {
        // Series of degree k - 1 determine the degree-(k - 1) coefficient of f.
        let indices = match Indices::new(1, k - 1, field.graph()) {
            Ok(indices) => indices,
            Err(Stop::Unavailable) => return Ok(None),
            Err(Stop::Error(error)) => return Err(error),
        };
        let step = (|| -> Result<Vec<DyadicInterval>, Stop> {
            field.graph().check_entries(&[slots.len(), k, lanes])?;
            let mut series = Vec::new();
            for slot in slots {
                field.graph().charge(k)?;
                series.push(match slot {
                    None => {
                        let mut time = vec![field.integer(0.into())?; k];
                        time[0] = start.clone();
                        if k > 1 {
                            time[1] = field.integer(1.into())?;
                        }
                        time
                    }
                    Some(lane) => x[*lane].clone(),
                });
            }
            let mut recurrence = Recurrence {
                indices: &indices,
                field: Interval {
                    precision: field.precision,
                    graph: &mut *field.graph,
                    arithmetic: &mut *field.arithmetic,
                },
            };
            let outputs = recurrence.run(program, Inputs::Series(&series))?;
            let order = recurrence.field.integer(k.into())?;
            let mut next = Vec::new();
            for output in &outputs {
                next.push(recurrence.field.div(&output[k - 1], &order)?);
            }
            Ok(next)
        })();
        match step {
            Ok(next) => {
                for (lane, value) in next.into_iter().enumerate() {
                    x[lane].push(value);
                }
            }
            Err(Stop::Unavailable) => return Ok(None),
            Err(Stop::Error(error)) => return Err(error),
        }
    }
    Ok(Some(x))
}

struct Recurrence<'a, F> {
    indices: &'a Indices,
    field: F,
}

impl<F: Field> Recurrence<'_, F> {
    fn charge(&mut self, work: usize) -> Result<(), Stop> {
        Ok(self.field.graph().charge(work)?)
    }
    fn one(&mut self) -> Result<F::Value, Stop> {
        self.field.integer(1.into())
    }
    fn zero(&mut self) -> Result<F::Value, Stop> {
        self.field.integer(0.into())
    }
    fn run(
        &mut self,
        program: &ExpressionProgram,
        inputs: Inputs<'_, F::Value>,
    ) -> Result<Vec<Arc<Vec<F::Value>>>, Stop> {
        let source = program.definition();
        let count = source.nodes.len();
        self.field
            .graph()
            .check_entries(&[count, self.indices.len()])?;
        self.charge(count)?;
        let mut reached = vec![false; count];
        for &root in &source.outputs {
            reached[root] = true;
        }
        for index in (0..count).rev() {
            self.charge(1)?;
            if reached[index] {
                for input in source.nodes[index].inputs().into_iter().flatten() {
                    reached[input] = true;
                }
            }
        }
        let mut series: Vec<Option<Arc<Vec<F::Value>>>> = vec![None; count];
        for index in 0..count {
            self.charge(1)?;
            if !reached[index] {
                continue;
            }
            let value = match &source.nodes[index] {
                ExpressionNode::Constant(value) => {
                    let value = self.field.real(value)?;
                    self.constant(value)?
                }
                ExpressionNode::Variable(reference) => {
                    let slot = program.slots[index].expect("validated binding");
                    match inputs {
                        Inputs::Centered { variables, points } => {
                            let mut value = self.constant(points[slot].clone())?;
                            let dimension = variables
                                .iter()
                                .position(|&v| v == reference.id)
                                .expect("reached variable is used");
                            let mut exponent = vec![0; variables.len()];
                            exponent[dimension] = 1;
                            // Degree-zero series (order-zero coefficients) have no unit slot.
                            if let Some(&slot) = self.indices.lookup.get(&exponent) {
                                value[slot] = self.one()?;
                            }
                            value
                        }
                        Inputs::Series(series) => {
                            self.charge(self.indices.len())?;
                            series[slot].clone()
                        }
                    }
                }
                ExpressionNode::Unary { operation, input } => {
                    let input = series[*input].as_ref().expect("dependency order");
                    self.unary(*operation, input)?
                }
                ExpressionNode::Binary {
                    operation,
                    left,
                    right,
                } => {
                    let left = series[*left].as_ref().expect("dependency order");
                    let right = series[*right].as_ref().expect("dependency order");
                    match operation {
                        BinaryOperation::Add => self.zip(left, right, F::add)?,
                        BinaryOperation::Sub => self.zip(left, right, F::sub)?,
                        BinaryOperation::Mul => self.mul(left, right)?,
                        BinaryOperation::Div => self.div(left, right)?,
                    }
                }
            };
            series[index] = Some(Arc::new(value));
        }
        Ok(source
            .outputs
            .iter()
            .map(|&root| series[root].clone().expect("reached output"))
            .collect())
    }
    fn unary(&mut self, operation: UnaryOperation, u: &[F::Value]) -> Result<Vec<F::Value>, Stop> {
        let base = self.field.image(operation, &u[0])?;
        // Every branch keeps the primitive's own image, which also enforces
        // its domain guard, as the degree-zero coefficient.
        let w = match operation {
            UnaryOperation::Neg => {
                let mut w = Vec::new();
                for value in u {
                    self.charge(1)?;
                    w.push(self.field.neg(value)?);
                }
                w
            }
            UnaryOperation::PowI(0) => {
                let one = self.one()?;
                self.constant(one)?
            }
            UnaryOperation::PowI(exponent) => self.compose_power(u, exponent)?,
            UnaryOperation::Sqrt => self.sqrt(u, base)?,
            UnaryOperation::Sin | UnaryOperation::Cos => {
                let cos = self.field.image(UnaryOperation::Cos, &u[0])?;
                let sin = self.field.image(UnaryOperation::Sin, &u[0])?;
                let (sin, cos) = self.sine_cosine(u, sin, cos)?;
                if operation == UnaryOperation::Sin {
                    sin
                } else {
                    cos
                }
            }
            UnaryOperation::Ln | UnaryOperation::Log10 => {
                let one = self.one()?;
                let one = self.constant(one)?;
                let mut h = self.div(&one, u)?;
                if operation == UnaryOperation::Log10 {
                    let ln10 = self.field.ln10()?;
                    let mut scaled = Vec::new();
                    for value in &h {
                        self.charge(1)?;
                        scaled.push(self.field.div(value, &ln10)?);
                    }
                    h = scaled;
                }
                self.integrate(u, &h, base)?
            }
            UnaryOperation::Asin | UnaryOperation::Acos | UnaryOperation::Acosh => {
                // d/du = +-1/sqrt(+-(1 - u^2)).
                let square = self.mul(u, u)?;
                let one = self.one()?;
                let one = self.constant(one)?;
                let radicand = if operation == UnaryOperation::Acosh {
                    self.zip(&square, &one, F::sub)?
                } else {
                    self.zip(&one, &square, F::sub)?
                };
                let root = self.field.image(UnaryOperation::Sqrt, &radicand[0])?;
                let root = self.sqrt(&radicand, root)?;
                let mut h = self.div(&one, &root)?;
                if operation == UnaryOperation::Acos {
                    for value in &mut h {
                        self.charge(1)?;
                        *value = self.field.neg(value)?;
                    }
                }
                self.integrate(u, &h, base)?
            }
            UnaryOperation::Atanh => {
                // d/du = 1/(1 - u^2).
                let square = self.mul(u, u)?;
                let one = self.one()?;
                let one = self.constant(one)?;
                let denominator = self.zip(&one, &square, F::sub)?;
                let h = self.div(&one, &denominator)?;
                self.integrate(u, &h, base)?
            }
        };
        Ok(w)
    }
    fn constant(&mut self, value: F::Value) -> Result<Vec<F::Value>, Stop> {
        let count = self.indices.len();
        self.charge(count)?;
        let mut series = vec![self.zero()?; count];
        series[0] = value;
        Ok(series)
    }
    fn zip(
        &mut self,
        left: &[F::Value],
        right: &[F::Value],
        operation: Operation<F>,
    ) -> Result<Vec<F::Value>, Stop> {
        let mut series = Vec::new();
        for (left, right) in left.iter().zip(right) {
            self.charge(1)?;
            series.push(operation(&mut self.field, left, right)?);
        }
        Ok(series)
    }
    // Sum of left_beta * right_gamma over the pairs of alpha selected by `keep`.
    fn convolve(
        &mut self,
        alpha: usize,
        left: &[F::Value],
        right: &[F::Value],
        keep: impl Fn(usize, usize) -> bool,
    ) -> Result<F::Value, Stop> {
        let mut sum = self.zero()?;
        let mut exact = true;
        let indices = self.indices;
        for &(beta, gamma) in &indices.pairs[alpha] {
            self.charge(1)?;
            // Polynomial-like series are mostly exact zeros; skip them.
            if keep(beta, gamma) && !F::is_zero(&left[beta]) && !F::is_zero(&right[gamma]) {
                let product = self.field.mul(&left[beta], &right[gamma])?;
                sum = self.field.add(&sum, &product)?;
                exact = false;
            }
        }
        if exact { Ok(sum) } else { self.field.tidy(sum) }
    }
    fn mul(&mut self, left: &[F::Value], right: &[F::Value]) -> Result<Vec<F::Value>, Stop> {
        let mut series = Vec::new();
        for alpha in 0..self.indices.len() {
            series.push(self.convolve(alpha, left, right, |_, _| true)?);
        }
        Ok(series)
    }
    // u^k = sum_m binom(k, m) u_0^(k-m) (u - u_0)^m. Over intervals each
    // u_0^(k-m) is the certified power image of the whole degree-zero
    // enclosure. Dividing by a wide u_0 repeatedly, as a quotient recurrence
    // would, compounds the dependency between numerator and denominator.
    fn compose_power(&mut self, u: &[F::Value], exponent: i64) -> Result<Vec<F::Value>, Stop> {
        let count = self.indices.len();
        let mut shifted = u.to_vec();
        shifted[0] = self.zero()?;
        let base = self.field.image(UnaryOperation::PowI(exponent), &u[0])?;
        let mut sum = self.constant(base)?;
        let degree: usize = self.indices.exponents[count - 1].iter().sum();
        let mut term = shifted.clone();
        let mut binomial = BigInt::from(1);
        let mut m: i64 = 1;
        while usize::try_from(m).is_ok_and(|m| m <= degree) {
            self.charge(1)?;
            binomial = binomial * (exponent - m + 1) / m;
            if binomial == BigInt::from(0) {
                break;
            }
            let base = if exponent == m {
                self.one()?
            } else {
                self.field
                    .image(UnaryOperation::PowI(exponent - m), &u[0])?
            };
            let scale = self.field.integer(binomial.clone())?;
            let coefficient = self.field.mul(&base, &scale)?;
            for alpha in 1..count {
                self.charge(1)?;
                if F::is_zero(&term[alpha]) {
                    continue;
                }
                let product = self.field.mul(&term[alpha], &coefficient)?;
                let total = self.field.add(&sum[alpha], &product)?;
                sum[alpha] = self.field.tidy(total)?;
            }
            m += 1;
            term = self.mul(&term, &shifted)?;
        }
        Ok(sum)
    }
    // w = u / v: v_0 w_alpha = u_alpha - sum_{beta != 0} v_beta w_{alpha-beta}.
    fn div(&mut self, u: &[F::Value], v: &[F::Value]) -> Result<Vec<F::Value>, Stop> {
        let mut w = Vec::new();
        for (alpha, value) in u.iter().enumerate() {
            let tail = self.convolve(alpha, v, &w, |beta, _| beta != 0)?;
            let numerator = self.field.sub(value, &tail)?;
            w.push(self.field.div(&numerator, &v[0])?);
        }
        Ok(w)
    }
    // w = sqrt(u): 2 w_0 w_alpha = u_alpha - sum_{beta, gamma != 0} w_beta w_gamma.
    fn sqrt(&mut self, u: &[F::Value], base: F::Value) -> Result<Vec<F::Value>, Stop> {
        let two = self.field.add(&base, &base)?;
        let mut w = vec![base];
        for (alpha, value) in u.iter().enumerate().skip(1) {
            let inner = self.convolve(alpha, &w, &w, |beta, gamma| beta != 0 && gamma != 0)?;
            let numerator = self.field.sub(value, &inner)?;
            w.push(self.field.div(&numerator, &two)?);
        }
        Ok(w)
    }
    // dw = h du along the first differentiated axis i:
    // alpha_i w_alpha = sum_{beta_i >= 1} beta_i u_beta h_{alpha-beta}.
    fn derivative_sum(
        &mut self,
        alpha: usize,
        axis: usize,
        u: &[F::Value],
        h: &[F::Value],
    ) -> Result<F::Value, Stop> {
        let mut sum = self.zero()?;
        let indices = self.indices;
        for &(beta, gamma) in &indices.pairs[alpha] {
            self.charge(1)?;
            let weight = indices.exponents[beta][axis];
            if weight > 0 && !F::is_zero(&u[beta]) {
                let mut product = self.field.mul(&u[beta], &h[gamma])?;
                if weight > 1 {
                    let weight = self.field.integer(weight.into())?;
                    product = self.field.mul(&product, &weight)?;
                }
                sum = self.field.add(&sum, &product)?;
            }
        }
        let order = self.field.integer(indices.exponents[alpha][axis].into())?;
        self.field.div(&sum, &order)
    }
    fn integrate(
        &mut self,
        u: &[F::Value],
        h: &[F::Value],
        base: F::Value,
    ) -> Result<Vec<F::Value>, Stop> {
        let mut w = vec![base];
        for alpha in 1..self.indices.len() {
            let axis = self.indices.axis(alpha);
            w.push(self.derivative_sum(alpha, axis, u, h)?);
        }
        Ok(w)
    }
    fn sine_cosine(
        &mut self,
        u: &[F::Value],
        sin: F::Value,
        cos: F::Value,
    ) -> Result<(Series<F>, Series<F>), Stop> {
        let mut s = vec![sin];
        let mut c = vec![cos];
        for alpha in 1..self.indices.len() {
            let axis = self.indices.axis(alpha);
            let next = self.derivative_sum(alpha, axis, u, &c)?;
            let other = self.derivative_sum(alpha, axis, u, &s)?;
            let other = self.field.neg(&other)?;
            s.push(next);
            c.push(other);
        }
        Ok((s, c))
    }
}
