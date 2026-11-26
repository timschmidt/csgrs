//! Interval Taylor-mode recurrences for the integral-remainder coefficients.
//!
//! Every node becomes a dense truncated series whose coefficients enclose the
//! normalized partials D^alpha g(xi) / alpha! of that node for every xi in the
//! domain box. Variables are X_i + h_i. Each coefficient is the interval
//! evaluation of the exact Taylor-arithmetic recurrence of the analytic
//! primitive, and degree zero is the primitive's certified interval image of
//! the operand's degree-zero enclosure, so inclusion monotonicity keeps every
//! coefficient sound for each xi separately. The recurrences only reuse lower
//! degrees, so repeated symbolic differentiation and its expression swell do
//! not occur. An operand outside a primitive's certified domain, or a divisor
//! or square root enclosure meeting zero, makes the whole bound unavailable
//! instead of failing the model; resource limits are still reported.
use super::*;
use crate::expression_program::dyadic::unary_image;
use hyperinterval::IntervalError;

/// Enclosures of every normalized partial up to one total degree, per output.
pub(super) struct Bounds {
    indices: Indices,
    outputs: Vec<Arc<Vec<DyadicInterval>>>,
    variables: Vec<SymbolId>,
}
impl Bounds {
    /// Enclosure of the requested normalized partial on the whole domain box.
    pub(super) fn coefficient(&self, request: &CoefficientRequest) -> &DyadicInterval {
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
    // Domain or zero-divisor obstruction: fall back to the symbolic bound alone.
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

type Series = Vec<DyadicInterval>;

struct Recurrence<'a> {
    indices: &'a Indices,
    precision: i32,
    graph: &'a mut ExpressionBudget,
    arithmetic: &'a mut DyadicBudget,
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
) -> Result<Option<Bounds>, TaylorError> {
    let indices = match Indices::new(variables.len(), degree, graph) {
        Ok(indices) => indices,
        Err(Stop::Unavailable) => return Ok(None),
        Err(Stop::Error(error)) => return Err(error),
    };
    let mut recurrence = Recurrence {
        indices: &indices,
        precision: request.precision,
        graph,
        arithmetic,
    };
    let outputs = match recurrence.run(program, variables, &request.domain) {
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

impl Recurrence<'_> {
    fn run(
        &mut self,
        program: &ExpressionProgram,
        variables: &[SymbolId],
        domain: &[DyadicInterval],
    ) -> Result<Vec<Arc<Series>>, Stop> {
        let source = program.definition();
        let count = source.nodes.len();
        self.graph.check_entries(&[count, self.indices.len()])?;
        self.graph.charge(count)?;
        let mut reached = vec![false; count];
        for &root in &source.outputs {
            reached[root] = true;
        }
        for index in (0..count).rev() {
            self.graph.charge(1)?;
            if reached[index] {
                for input in source.nodes[index].inputs().into_iter().flatten() {
                    reached[input] = true;
                }
            }
        }
        let mut series: Vec<Option<Arc<Series>>> = vec![None; count];
        for index in 0..count {
            self.graph.charge(1)?;
            if !reached[index] {
                continue;
            }
            let value = match &source.nodes[index] {
                ExpressionNode::Constant(value) => {
                    let value = DyadicInterval::from_real(value, self.precision, self.arithmetic)?;
                    self.constant(value)?
                }
                ExpressionNode::Variable(reference) => {
                    let slot = program.slots[index].expect("validated binding");
                    let mut value = self.constant(domain[slot].clone())?;
                    let dimension = variables
                        .iter()
                        .position(|&v| v == reference.id)
                        .expect("reached variable is used");
                    let mut exponent = vec![0; variables.len()];
                    exponent[dimension] = 1;
                    value[self.indices.lookup[&exponent]] = one()?;
                    value
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
                        BinaryOperation::Add => self.zip(left, right, DyadicInterval::add)?,
                        BinaryOperation::Sub => self.zip(left, right, DyadicInterval::sub)?,
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
    fn unary(&mut self, operation: UnaryOperation, u: &Series) -> Result<Series, Stop> {
        let base = unary_image(operation, u[0].clone(), self.precision, self.arithmetic)?
            .ok_or(Stop::Unavailable)?;
        // Every branch keeps the primitive's own image, which also enforces
        // its domain guard, as the degree-zero enclosure.
        let w = match operation {
            UnaryOperation::Neg => {
                let mut w = Vec::new();
                for value in u {
                    self.graph.charge(1)?;
                    w.push(value.neg(self.arithmetic)?);
                }
                w
            }
            UnaryOperation::PowI(0) => self.constant(one()?)?,
            UnaryOperation::PowI(exponent) => self.compose_power(u, exponent)?,
            UnaryOperation::Sqrt => self.sqrt(u, base.clone())?,
            UnaryOperation::Sin | UnaryOperation::Cos => {
                let cos = unary_image(
                    UnaryOperation::Cos,
                    u[0].clone(),
                    self.precision,
                    self.arithmetic,
                )?
                .ok_or(Stop::Unavailable)?;
                let sin = unary_image(
                    UnaryOperation::Sin,
                    u[0].clone(),
                    self.precision,
                    self.arithmetic,
                )?
                .ok_or(Stop::Unavailable)?;
                let (sin, cos) = self.sine_cosine(u, sin, cos)?;
                if operation == UnaryOperation::Sin {
                    sin
                } else {
                    cos
                }
            }
            UnaryOperation::Ln | UnaryOperation::Log10 => {
                let one = self.constant(one()?)?;
                let mut h = self.div(&one, u)?;
                if operation == UnaryOperation::Log10 {
                    let ten = DyadicInterval::point(10.into(), 0)?;
                    let ln10 = unary_image(
                        UnaryOperation::Ln,
                        ten,
                        self.precision.min(0),
                        self.arithmetic,
                    )?
                    .ok_or(Stop::Unavailable)?;
                    let mut scaled = Vec::new();
                    for value in &h {
                        self.graph.charge(1)?;
                        scaled.push(value.div(&ln10, self.precision, self.arithmetic)?);
                    }
                    h = scaled;
                }
                self.integrate(u, &h, base.clone())?
            }
            UnaryOperation::Asin | UnaryOperation::Acos | UnaryOperation::Acosh => {
                // d/du = +-1/sqrt(+-(1 - u^2)).
                let square = self.mul(u, u)?;
                let one = self.constant(one()?)?;
                let radicand = if operation == UnaryOperation::Acosh {
                    self.zip(&square, &one, DyadicInterval::sub)?
                } else {
                    self.zip(&one, &square, DyadicInterval::sub)?
                };
                let root = unary_image(
                    UnaryOperation::Sqrt,
                    radicand[0].clone(),
                    self.precision,
                    self.arithmetic,
                )?
                .ok_or(Stop::Unavailable)?;
                let root = self.sqrt(&radicand, root)?;
                let mut h = self.div(&one, &root)?;
                if operation == UnaryOperation::Acos {
                    for value in &mut h {
                        self.graph.charge(1)?;
                        *value = value.neg(self.arithmetic)?;
                    }
                }
                self.integrate(u, &h, base.clone())?
            }
            UnaryOperation::Atanh => {
                // d/du = 1/(1 - u^2).
                let square = self.mul(u, u)?;
                let one = self.constant(one()?)?;
                let denominator = self.zip(&one, &square, DyadicInterval::sub)?;
                let h = self.div(&one, &denominator)?;
                self.integrate(u, &h, base.clone())?
            }
        };
        Ok(w)
    }
    fn constant(&mut self, value: DyadicInterval) -> Result<Series, Stop> {
        let count = self.indices.len();
        self.graph.charge(count)?;
        let mut series = vec![zero()?; count];
        series[0] = value;
        Ok(series)
    }
    fn tidy(&mut self, value: &DyadicInterval) -> Result<DyadicInterval, Stop> {
        Ok(value.regrid(self.precision, self.arithmetic)?)
    }
    fn zip(
        &mut self,
        left: &Series,
        right: &Series,
        operation: fn(
            &DyadicInterval,
            &DyadicInterval,
            &mut DyadicBudget,
        ) -> Result<DyadicInterval, IntervalError>,
    ) -> Result<Series, Stop> {
        let mut series = Vec::new();
        for (left, right) in left.iter().zip(right) {
            self.graph.charge(1)?;
            series.push(operation(left, right, self.arithmetic)?);
        }
        Ok(series)
    }
    // Sum of left_beta * right_gamma over the pairs of alpha selected by `keep`.
    fn convolve(
        &mut self,
        alpha: usize,
        left: &[DyadicInterval],
        right: &[DyadicInterval],
        keep: impl Fn(usize, usize) -> bool,
    ) -> Result<DyadicInterval, Stop> {
        let mut sum = zero()?;
        let mut exact = true;
        for &(beta, gamma) in &self.indices.pairs[alpha] {
            self.graph.charge(1)?;
            // Polynomial-like series are mostly exact zeros; skip them.
            if keep(beta, gamma) && !is_zero(&left[beta]) && !is_zero(&right[gamma]) {
                let product = left[beta].mul(&right[gamma], self.arithmetic)?;
                sum = sum.add(&product, self.arithmetic)?;
                exact = false;
            }
        }
        if exact { Ok(sum) } else { self.tidy(&sum) }
    }
    fn mul(&mut self, left: &Series, right: &Series) -> Result<Series, Stop> {
        let mut series = Vec::new();
        for alpha in 0..self.indices.len() {
            series.push(self.convolve(alpha, left, right, |_, _| true)?);
        }
        Ok(series)
    }
    // u^k = sum_m binom(k, m) u_0^(k-m) (u - u_0)^m, with each u_0^(k-m)
    // the certified interval power image of the whole degree-zero enclosure.
    // Dividing by a wide u_0 repeatedly, as a quotient recurrence would,
    // compounds the dependency between numerator and denominator instead.
    fn compose_power(&mut self, u: &Series, exponent: i64) -> Result<Series, Stop> {
        let count = self.indices.len();
        let mut shifted = u.clone();
        shifted[0] = zero()?;
        let base = unary_image(
            UnaryOperation::PowI(exponent),
            u[0].clone(),
            self.precision,
            self.arithmetic,
        )?
        .ok_or(Stop::Unavailable)?;
        let mut sum = self.constant(base)?;
        let degree: usize = self.indices.exponents[count - 1].iter().sum();
        let mut term = shifted.clone();
        let mut binomial = num::BigInt::from(1);
        let mut m: i64 = 1;
        while usize::try_from(m).is_ok_and(|m| m <= degree) {
            self.graph.charge(1)?;
            binomial = binomial * (exponent - m + 1) / m;
            if binomial == num::BigInt::from(0) {
                break;
            }
            let base = if exponent == m {
                one()?
            } else {
                unary_image(
                    UnaryOperation::PowI(exponent - m),
                    u[0].clone(),
                    self.precision,
                    self.arithmetic,
                )?
                .ok_or(Stop::Unavailable)?
            };
            let coefficient = base.mul(
                &DyadicInterval::point(binomial.clone(), 0)?,
                self.arithmetic,
            )?;
            for alpha in 1..count {
                self.graph.charge(1)?;
                if is_zero(&term[alpha]) {
                    continue;
                }
                let product = term[alpha].mul(&coefficient, self.arithmetic)?;
                let total = sum[alpha].add(&product, self.arithmetic)?;
                sum[alpha] = self.tidy(&total)?;
            }
            m += 1;
            term = self.mul(&term, &shifted)?;
        }
        Ok(sum)
    }
    // w = u / v: v_0 w_alpha = u_alpha - sum_{beta != 0} v_beta w_{alpha-beta}.
    fn div(&mut self, u: &Series, v: &Series) -> Result<Series, Stop> {
        let mut w: Series = Vec::new();
        for (alpha, value) in u.iter().enumerate() {
            let tail = self.convolve(alpha, v, &w, |beta, _| beta != 0)?;
            let numerator = value.sub(&tail, self.arithmetic)?;
            w.push(numerator.div(&v[0], self.precision, self.arithmetic)?);
        }
        Ok(w)
    }
    // w = sqrt(u): 2 w_0 w_alpha = u_alpha - sum_{beta, gamma != 0} w_beta w_gamma.
    fn sqrt(&mut self, u: &Series, base: DyadicInterval) -> Result<Series, Stop> {
        let two = base.add(&base, self.arithmetic)?;
        let mut w: Series = vec![base];
        for (alpha, value) in u.iter().enumerate().skip(1) {
            let inner = self.convolve(alpha, &w, &w, |beta, gamma| beta != 0 && gamma != 0)?;
            let numerator = value.sub(&inner, self.arithmetic)?;
            w.push(numerator.div(&two, self.precision, self.arithmetic)?);
        }
        Ok(w)
    }
    // dw = h du along the first differentiated axis i:
    // alpha_i w_alpha = sum_{beta_i >= 1} beta_i u_beta h_{alpha-beta}.
    fn derivative_sum(
        &mut self,
        alpha: usize,
        axis: usize,
        u: &Series,
        h: &[DyadicInterval],
    ) -> Result<DyadicInterval, Stop> {
        let mut sum = zero()?;
        for &(beta, gamma) in &self.indices.pairs[alpha] {
            self.graph.charge(1)?;
            let weight = self.indices.exponents[beta][axis];
            if weight > 0 && !is_zero(&u[beta]) {
                let mut product = u[beta].mul(&h[gamma], self.arithmetic)?;
                if weight > 1 {
                    let weight = DyadicInterval::point(weight.into(), 0)?;
                    product = product.mul(&weight, self.arithmetic)?;
                }
                sum = sum.add(&product, self.arithmetic)?;
            }
        }
        let order = DyadicInterval::point(self.indices.exponents[alpha][axis].into(), 0)?;
        Ok(sum.div(&order, self.precision, self.arithmetic)?)
    }
    fn integrate(&mut self, u: &Series, h: &Series, base: DyadicInterval) -> Result<Series, Stop> {
        let mut w = vec![base];
        for alpha in 1..self.indices.len() {
            let axis = self.indices.axis(alpha);
            w.push(self.derivative_sum(alpha, axis, u, h)?);
        }
        Ok(w)
    }
    fn sine_cosine(
        &mut self,
        u: &Series,
        sin: DyadicInterval,
        cos: DyadicInterval,
    ) -> Result<(Series, Series), Stop> {
        let mut s = vec![sin];
        let mut c = vec![cos];
        for alpha in 1..self.indices.len() {
            let axis = self.indices.axis(alpha);
            let next = self.derivative_sum(alpha, axis, u, &c)?;
            let other = self
                .derivative_sum(alpha, axis, u, &s)?
                .neg(self.arithmetic)?;
            s.push(next);
            c.push(other);
        }
        Ok((s, c))
    }
}
fn is_zero(value: &DyadicInterval) -> bool {
    value.signs() == SignSet::Zero
}
fn one() -> Result<DyadicInterval, IntervalError> {
    DyadicInterval::point(1.into(), 0)
}
