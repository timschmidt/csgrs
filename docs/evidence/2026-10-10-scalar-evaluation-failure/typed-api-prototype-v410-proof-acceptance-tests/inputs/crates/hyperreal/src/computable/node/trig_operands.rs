// Unlike PiLinearForm, this form retains unsupported subexpressions as atoms.
// Its identities are conditional on the complete original graph being real;
// callers must establish that domain before discarding cancelled operands.
#[derive(Clone)]
struct PiOperandTerm {
    exact: PiLinearTerm,
    atom: Option<Computable>,
}

#[derive(Default)]
struct PiOperandForm {
    terms: Vec<PiOperandTerm>,
    saw_pi: bool,
    conditional: bool,
}

pub(super) struct PiOperandReduction {
    pub(super) quadrant: u8,
    pub(super) argument: Computable,
    pub(super) exact_index_when_real: Option<u8>,
}

impl PiOperandForm {
    fn insert(&mut self, mut term: PiOperandTerm) -> Option<()> {
        loop {
            term.exact.normalize()?;
            if term.exact.coefficient.is_zero() {
                return Some(());
            }
            let position = self.terms.iter().position(|old| {
                old.exact.pi == term.exact.pi
                    && old.exact.exponent == term.exact.exponent
                    && match (&old.atom, &term.atom) {
                        (None, None) => true,
                        (Some(left), Some(right)) => {
                            Computable::internal_structural_eq(left, right)
                        }
                        _ => false,
                    }
            });
            if let Some(index) = position {
                term.exact.coefficient =
                    term.exact.coefficient + self.terms.swap_remove(index).exact.coefficient;
            } else {
                if self.terms.len() == MAX_TRIG_TERMS {
                    return None;
                }
                self.terms.push(term);
                return Some(());
            }
        }
    }

    fn leaf(
        pi: bool,
        coefficient: Rational,
        atom: Option<Computable>,
        conditional: bool,
    ) -> Option<Self> {
        let mut result = Self {
            terms: Vec::new(),
            saw_pi: pi,
            conditional,
        };
        result.insert(PiOperandTerm {
            exact: PiLinearTerm {
                pi,
                coefficient,
                exponent: 0,
            },
            atom,
        })?;
        Some(result)
    }

    fn atom(node: &Computable) -> Option<Self> {
        // This existing bounded certificate is used only conditionally: even a
        // cancelled partial operand remains in the original graph's domain walk.
        if let Some(value) = node.bounded_rational_value()
            && bounded_trig_coefficient(&value)
        {
            return Self::leaf(false, value, None, true);
        }
        Self::leaf(false, Rational::one(), Some(node.clone()), true)
    }

    fn scalar(&self) -> bool {
        self.terms
            .iter()
            .all(|term| !term.exact.pi && term.atom.is_none())
    }

    fn visit(node: &Computable, depth: u16, work: &mut usize) -> Option<Self> {
        if depth == 0 || *work == 0 {
            return None;
        }
        *work -= 1;
        use Approximation::*;
        match &node.internal.approximation {
            One => Self::leaf(false, Rational::one(), None, false),
            Int(n) if n.bits() <= MAX_TRIG_COEFFICIENT_BITS => {
                Self::leaf(false, Rational::from_bigint(n.clone()), None, false)
            }
            Ratio(q) if bounded_trig_coefficient(q) => Self::leaf(false, q.clone(), None, false),
            Constant(SharedConstant::Pi) => Self::leaf(true, Rational::one(), None, false),
            Constant(SharedConstant::Tau) => Self::leaf(true, Rational::new(2), None, false),
            Negate(child) => {
                let mut result = Self::visit(child, depth - 1, work)?;
                for term in &mut result.terms {
                    term.exact.coefficient = -term.exact.coefficient.clone();
                }
                Some(result)
            }
            Offset(child, shift) => {
                let mut result = Self::visit(child, depth - 1, work)?;
                for term in &mut result.terms {
                    term.exact.exponent = term.exact.exponent.checked_add(i64::from(*shift))?;
                }
                Some(result)
            }
            Add(left, right) => {
                let mut result = Self::visit(left, depth - 1, work)?;
                let right = Self::visit(right, depth - 1, work)?;
                result.saw_pi |= right.saw_pi;
                result.conditional |= right.conditional;
                for term in right.terms {
                    result.insert(term)?;
                }
                Some(result)
            }
            Multiply(left, right) => {
                let left = Self::visit(left, depth - 1, work)?;
                let right = Self::visit(right, depth - 1, work)?;
                if !left.scalar() && !right.scalar() {
                    // A nonlinear factor stays intact; pi inside that atom is
                    // not an independently removable periodic coefficient.
                    return Self::atom(node);
                }
                let mut result = Self {
                    terms: Vec::new(),
                    saw_pi: left.saw_pi || right.saw_pi,
                    conditional: left.conditional || right.conditional,
                };
                for a in &left.terms {
                    for b in &right.terms {
                        *work = work.checked_sub(1)?;
                        result.insert(PiOperandTerm {
                            exact: PiLinearTerm {
                                pi: a.exact.pi || b.exact.pi,
                                coefficient: &a.exact.coefficient * &b.exact.coefficient,
                                exponent: a.exact.exponent.checked_add(b.exact.exponent)?,
                            },
                            atom: a.atom.clone().or_else(|| b.atom.clone()),
                        })?;
                    }
                }
                Some(result)
            }
            _ => Self::atom(node),
        }
    }

    fn reduce(self) -> Option<PiOperandReduction> {
        if !self.saw_pi || !self.conditional {
            return None;
        }
        let mut pi = PiLinearForm::default();
        let mut residual = None;
        for term in self.terms {
            if term.exact.pi {
                pi.insert(term.exact)?;
                continue;
            }
            let mut value = if let Some(atom) = term.atom {
                if term.exact.coefficient.is_one() {
                    atom
                } else {
                    Computable::raw_periodic_operand(Approximation::Multiply(
                        atom,
                        Computable::rational(term.exact.coefficient),
                    ))
                }
            } else {
                Computable::rational(term.exact.coefficient)
            };
            let mut exponent = term.exact.exponent;
            // Keep binary scales symbolic. At most 256 source visits/products
            // can contribute offsets, so an unrepresentable aggregate declines.
            let mut shifts = 256;
            while exponent != 0 {
                if shifts == 0 {
                    return None;
                }
                shifts -= 1;
                let shift = exponent.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
                value = Computable::raw_periodic_operand(Approximation::Offset(value, shift));
                exponent -= i64::from(shift);
            }
            residual = Some(match residual {
                Some(previous) => {
                    Computable::raw_periodic_operand(Approximation::Add(previous, value))
                }
                None => value,
            });
        }
        let (quadrant, coefficient, zero) = pi.reduced_angle()?;
        debug_assert!(zero.is_zero());
        let exact_index_when_real =
            (coefficient.is_zero() && residual.is_none()).then_some(quadrant);
        if !coefficient.is_zero() {
            let angle = Computable::raw_periodic_operand(Approximation::Multiply(
                Computable::pi(),
                Computable::rational(coefficient),
            ));
            residual = Some(match residual {
                Some(value) => Computable::raw_periodic_operand(Approximation::Add(angle, value)),
                None => angle,
            });
        }
        Some(PiOperandReduction {
            quadrant,
            argument: residual.unwrap_or_else(Computable::zero),
            exact_index_when_real,
        })
    }
}

impl Computable {
    fn raw_periodic_operand(approximation: Approximation) -> Self {
        Self {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Conditional identity only. The original graph must be validated before
    /// numerical use or a fact derived from cancelled atoms can be published.
    pub(super) fn pi_operand_reduction(&self) -> Option<PiOperandReduction> {
        PiOperandForm::visit(self, 128, &mut 256)?.reduce()
    }

    fn half_pi_index_when_real(&self) -> Option<u8> {
        self.exact_half_pi_index()
            .or_else(|| self.pi_operand_reduction()?.exact_index_when_real)
    }
}
