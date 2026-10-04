// Exact signs in iterated square-root towers `Q(sqrt(r_1))...(sqrt(r_k))`.
//
// Each element is `a + b sqrt(r_l)` with `a`, `b` and `r_l` over the lower
// generators. A sign follows from the signs of `a` and `b` and, when they
// disagree, the sign of `a^2 - b^2 r_l`. Those identities hold for the
// principal root even when `sqrt(r_l)` already lies in a lower field, so
// repeated or dependent radicals stay exact. Unsupported nodes, a negative
// radicand or a query beyond its work budget return `None`, leaving the
// general sign cascade authoritative.

#[derive(Clone, Debug, PartialEq)]
enum RadicalElement {
    Rational(Rational),
    /// `low + high * sqrt(generator level)`; both parts use lower levels.
    Extension {
        level: usize,
        low: Box<RadicalElement>,
        high: Box<RadicalElement>,
    },
}

struct RadicalTower {
    radicands: Vec<RadicalElement>,
}

// One query's work budget. This reduction runs only after bounded
// refinement left a sign undecided, which is frequent for exact zeros that
// callers can also settle by other means. The budget keeps that common case
// cheap; a declined query leaves the existing exact cascade in charge and
// restricts no representable value.
const RADICAL_TOWER_MAX_NODES: usize = 256;
const RADICAL_TOWER_MAX_GENERATORS: usize = 6;
const RADICAL_TOWER_MAX_TERMS: usize = 256;
const RADICAL_TOWER_MAX_COEFFICIENT_BITS: u64 = 4_096;

impl RadicalElement {
    fn zero() -> Self {
        Self::Rational(Rational::zero())
    }

    fn level(&self) -> Option<usize> {
        match self {
            Self::Rational(_) => None,
            Self::Extension { level, .. } => Some(*level),
        }
    }

    fn terms(&self) -> usize {
        match self {
            Self::Rational(_) => 1,
            Self::Extension { low, high, .. } => low.terms() + high.terms(),
        }
    }

    fn coefficient_bits(&self) -> u64 {
        match self {
            Self::Rational(value) => value.numerator().bits() + value.denominator().bits(),
            Self::Extension { low, high, .. } => {
                low.coefficient_bits().max(high.coefficient_bits())
            }
        }
    }

    fn within_budget(&self) -> bool {
        self.terms() <= RADICAL_TOWER_MAX_TERMS
            && self.coefficient_bits() <= RADICAL_TOWER_MAX_COEFFICIENT_BITS
    }

    fn is_structurally_zero(&self) -> bool {
        match self {
            Self::Rational(value) => value.sign() == Sign::NoSign,
            Self::Extension { low, high, .. } => {
                low.is_structurally_zero() && high.is_structurally_zero()
            }
        }
    }

    fn generator(level: usize) -> Self {
        Self::Extension {
            level,
            low: Box::new(Self::zero()),
            high: Box::new(Self::Rational(Rational::one())),
        }
    }

    /// Splits this element into its parts over generator `level`.
    fn split(self, level: usize) -> (Self, Self) {
        match self {
            Self::Extension {
                level: own,
                low,
                high,
            } if own == level => (*low, *high),
            other => (other, Self::zero()),
        }
    }

    fn join(level: usize, low: Self, high: Self) -> Self {
        if high.is_structurally_zero() {
            low
        } else {
            Self::Extension {
                level,
                low: Box::new(low),
                high: Box::new(high),
            }
        }
    }

    fn neg(self) -> Self {
        match self {
            Self::Rational(value) => Self::Rational(-value),
            Self::Extension { level, low, high } => Self::Extension {
                level,
                low: Box::new(low.neg()),
                high: Box::new(high.neg()),
            },
        }
    }

    fn scale(self, factor: &Rational) -> Self {
        match self {
            Self::Rational(value) => Self::Rational(&value * factor),
            Self::Extension { level, low, high } => {
                Self::join(level, low.scale(factor), high.scale(factor))
            }
        }
    }

    fn add(self, other: Self) -> Self {
        match (self.level(), other.level()) {
            (None, None) => {
                let (Self::Rational(left), Self::Rational(right)) = (self, other) else {
                    unreachable!("levelless radical elements are rational")
                };
                Self::Rational(&left + &right)
            }
            (left, right) => {
                let level = left.max(right).expect("one element has a generator");
                let (left_low, left_high) = self.split(level);
                let (right_low, right_high) = other.split(level);
                Self::join(level, left_low.add(right_low), left_high.add(right_high))
            }
        }
    }
}

impl RadicalTower {
    fn mul(&self, left: RadicalElement, right: RadicalElement) -> Option<RadicalElement> {
        let product = match (left.level(), right.level()) {
            (None, None) => {
                let (RadicalElement::Rational(left), RadicalElement::Rational(right)) =
                    (left, right)
                else {
                    unreachable!("levelless radical elements are rational")
                };
                RadicalElement::Rational(&left * &right)
            }
            (None, Some(_)) => {
                let RadicalElement::Rational(factor) = left else {
                    unreachable!("levelless radical elements are rational")
                };
                right.scale(&factor)
            }
            (Some(_), None) => {
                let RadicalElement::Rational(factor) = right else {
                    unreachable!("levelless radical elements are rational")
                };
                left.scale(&factor)
            }
            (Some(left_level), Some(right_level)) => {
                let level = left_level.max(right_level);
                let (left_low, left_high) = left.split(level);
                let (right_low, right_high) = right.split(level);
                // (a + b s)(c + d s) = ac + bd r + (ad + bc) s, with s^2 = r.
                let high_product = self.mul(left_high.clone(), right_high.clone())?;
                let low = self
                    .mul(left_low.clone(), right_low.clone())?
                    .add(self.mul(high_product, self.radicands[level].clone())?);
                let high = self
                    .mul(left_low, right_high)?
                    .add(self.mul(left_high, right_low)?);
                RadicalElement::join(level, low, high)
            }
        };
        product.within_budget().then_some(product)
    }

    fn sign(&self, value: &RadicalElement) -> Option<RealSign> {
        let RadicalElement::Extension { level, low, high } = value else {
            let RadicalElement::Rational(value) = value else {
                unreachable!("levelless radical elements are rational")
            };
            return Some(rational_sign(value));
        };
        let low_sign = self.sign(low)?;
        let high_sign = self.sign(high)?;
        if high_sign == RealSign::Zero || low_sign == high_sign {
            return Some(low_sign);
        }
        if low_sign == RealSign::Zero {
            // Radicands are certified positive, so the root is positive.
            return Some(high_sign);
        }
        // a + b s with opposite signs: compare a^2 with b^2 r.
        let low_square = self.mul((**low).clone(), (**low).clone())?;
        let high_square = self.mul((**high).clone(), (**high).clone())?;
        let scaled = self.mul(high_square, self.radicands[*level].clone())?;
        let difference = low_square.add(scaled.neg());
        let difference_sign = self.sign(&difference)?;
        Some(match (low_sign, difference_sign) {
            (_, RealSign::Zero) => RealSign::Zero,
            (RealSign::Positive, sign) => sign,
            (RealSign::Negative, RealSign::Positive) => RealSign::Negative,
            (RealSign::Negative, RealSign::Negative) => RealSign::Positive,
            (RealSign::Zero, _) => unreachable!("a zero low part returned above"),
        })
    }

    fn inverse(&self, value: RadicalElement) -> Option<RadicalElement> {
        match value {
            RadicalElement::Rational(value) => {
                if value.sign() == Sign::NoSign {
                    return None;
                }
                Some(RadicalElement::Rational(Rational::one() / value))
            }
            RadicalElement::Extension { level, low, high } => {
                // 1 / (a + b s) = (a - b s) / (a^2 - b^2 r).
                let low_square = self.mul((*low).clone(), (*low).clone())?;
                let high_square = self.mul((*high).clone(), (*high).clone())?;
                let norm = low_square
                    .add(self.mul(high_square, self.radicands[level].clone())?.neg());
                // A zero norm means sqrt(r) is already a lower-field value
                // dependent on this element; the conjugate cannot invert it.
                if self.sign(&norm)? == RealSign::Zero {
                    return None;
                }
                let inverse_norm = self.inverse(norm)?;
                let conjugate = RadicalElement::join(level, *low, high.neg());
                self.mul(conjugate, inverse_norm)
            }
        }
    }

    fn square_root(&mut self, radicand: RadicalElement) -> Option<RadicalElement> {
        match self.sign(&radicand)? {
            RealSign::Zero => Some(RadicalElement::zero()),
            RealSign::Negative => None,
            RealSign::Positive => {
                if let RadicalElement::Rational(value) = &radicand
                    && let Some(root) = rational_square_root(value)
                {
                    return Some(RadicalElement::Rational(root));
                }
                // Separately built roots of one radicand name one generator,
                // so cancellations between them reduce structurally.
                if let Some(level) = self.radicands.iter().position(|known| known == &radicand) {
                    return Some(RadicalElement::generator(level));
                }
                if self.radicands.len() >= RADICAL_TOWER_MAX_GENERATORS {
                    return None;
                }
                self.radicands.push(radicand);
                Some(RadicalElement::generator(self.radicands.len() - 1))
            }
        }
    }
}

/// Exact rational square root when both reduced parts are perfect squares.
fn rational_square_root(value: &Rational) -> Option<Rational> {
    let (root, remainder) = square_parts(value.numerator());
    let (denominator_root, denominator_remainder) = square_parts(value.denominator());
    (remainder.is_one() && denominator_remainder.is_one()).then(|| &root / &denominator_root)
}

fn radical_tower_sign_of(value: &Computable) -> Option<RealSign> {
    // Traverse the immutable DAG in postorder, as the quadratic tower does.
    // Shared subexpressions reduce once.
    let key = |value: &Computable| Arc::as_ptr(&value.internal);
    let mut tower = RadicalTower {
        radicands: Vec::new(),
    };
    let mut memo = std::collections::HashMap::<*const Node, RadicalElement>::new();
    let mut pending = vec![(value, false)];
    while let Some((current, ready)) = pending.pop() {
        let current_key = key(current);
        if memo.contains_key(&current_key) {
            continue;
        }
        if memo.len() >= RADICAL_TOWER_MAX_NODES {
            return None;
        }
        if !ready {
            if let Some(rational) = current.exact_rational() {
                memo.insert(current_key, RadicalElement::Rational(rational));
                continue;
            }
            pending.push((current, true));
            match &current.internal.approximation {
                Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3) => {}
                Approximation::Negate(child)
                | Approximation::Offset(child, _)
                | Approximation::Inverse(child)
                | Approximation::Square(child)
                | Approximation::Sqrt(child)
                | Approximation::NthRoot(child, 2) => pending.push((child, false)),
                Approximation::Add(left, right) | Approximation::Multiply(left, right) => {
                    pending.push((right, false));
                    pending.push((left, false));
                }
                Approximation::LinearCombination3(combination) => {
                    pending.extend(
                        combination
                            .coefficients
                            .iter()
                            .rev()
                            .map(|child| (child, false)),
                    );
                }
                _ => return None,
            }
            continue;
        }
        let child = |value: &Computable| memo.get(&key(value)).cloned();
        let element = match &current.internal.approximation {
            Approximation::Constant(SharedConstant::Sqrt2) => {
                tower.square_root(RadicalElement::Rational(Rational::new(2)))?
            }
            Approximation::Constant(SharedConstant::Sqrt3) => {
                tower.square_root(RadicalElement::Rational(Rational::new(3)))?
            }
            Approximation::Negate(value) => child(value)?.neg(),
            Approximation::Offset(value, shift) => {
                let factor = shifted_coefficient(&Rational::one(), *shift)?;
                child(value)?.scale(&factor)
            }
            Approximation::Add(left, right) => child(left)?.add(child(right)?),
            Approximation::Multiply(left, right) => tower.mul(child(left)?, child(right)?)?,
            Approximation::Inverse(value) => tower.inverse(child(value)?)?,
            Approximation::Square(value) => {
                let value = child(value)?;
                tower.mul(value.clone(), value)?
            }
            Approximation::Sqrt(value) | Approximation::NthRoot(value, 2) => {
                tower.square_root(child(value)?)?
            }
            Approximation::LinearCombination3(combination) => {
                let mut sum = RadicalElement::zero();
                for (coefficient, weight) in combination
                    .coefficients
                    .iter()
                    .zip(combination.values.iter())
                {
                    sum = sum.add(child(coefficient)?.scale(weight));
                }
                sum
            }
            _ => return None,
        };
        if !element.within_budget() {
            return None;
        }
        memo.insert(current_key, element);
    }
    let value = memo.remove(&key(value))?;
    tower.sign(&value)
}

impl Computable {
    /// Exact sign by reduction in an iterated square-root tower.
    ///
    /// `None` means the expression leaves that tower or exceeds this query's
    /// work budget; the value itself is unchanged.
    pub(crate) fn radical_tower_sign(&self) -> Option<RealSign> {
        // A declined reduction is deterministic for this root; remembering it
        // keeps repeated undecided queries from re-walking a large DAG.
        if self.internal.cache.radical_tower_declined() {
            return None;
        }
        let Some(sign) = radical_tower_sign_of(self) else {
            self.internal.cache.mark_radical_tower_declined();
            return None;
        };
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(private_sign(sign)));
        Some(sign)
    }
}
