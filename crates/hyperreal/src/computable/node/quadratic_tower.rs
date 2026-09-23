// Exact values in `Q(sqrt(d))(sqrt(a + b sqrt(d)))`.
//
// Products and inverses reduce by the outer square relation. A node outside
// this tower, or a third independent radical, returns `None` so the general
// sign cascade stays authoritative.

#[derive(Clone)]
struct Quad {
    rational: Rational,
    scale: Rational,
    disc: Option<Rational>,
}

#[derive(Clone)]
struct Tower {
    even: Quad,
    odd: Quad,
    radicand: Option<Quad>,
}

impl Quad {
    fn rational(value: Rational) -> Self {
        Self {
            rational: value,
            scale: Rational::zero(),
            disc: None,
        }
    }

    fn zero() -> Self {
        Self::rational(Rational::zero())
    }

    fn is_zero(&self) -> bool {
        self.rational.sign() == Sign::NoSign && self.scale.sign() == Sign::NoSign
    }

    fn same_disc(&self, other: &Self) -> Option<Option<Rational>> {
        match (&self.disc, &other.disc) {
            (Some(left), Some(right)) if left == right => Some(Some(left.clone())),
            (Some(disc), None) | (None, Some(disc)) => Some(Some(disc.clone())),
            (None, None) => Some(None),
            _ => None,
        }
    }

    fn add(self, other: Self) -> Option<Self> {
        admit_rational(&self.rational)?;
        admit_rational(&self.scale)?;
        admit_rational(&other.rational)?;
        admit_rational(&other.scale)?;
        let disc = self.same_disc(&other)?;
        let mut sum = Self {
            rational: &self.rational + &other.rational,
            scale: &self.scale + &other.scale,
            disc,
        };
        if sum.scale.sign() == Sign::NoSign {
            sum.disc = None;
        }
        Some(sum)
    }

    fn neg(self) -> Self {
        Self {
            rational: -self.rational,
            scale: -self.scale,
            disc: self.disc,
        }
    }

    fn mul(self, other: Self) -> Option<Self> {
        admit_rational(&self.rational)?;
        admit_rational(&self.scale)?;
        admit_rational(&other.rational)?;
        admit_rational(&other.scale)?;
        let disc = self.same_disc(&other)?;
        let cross = &self.rational * &other.scale + &self.scale * &other.rational;
        admit_rational(&cross)?;
        let mut rational = &self.rational * &other.rational;
        if self.scale.sign() != Sign::NoSign && other.scale.sign() != Sign::NoSign {
            let disc = disc.as_ref()?;
            admit_rational(disc)?;
            rational = rational + &self.scale * &other.scale * disc;
        }
        admit_rational(&rational)?;
        let mut product = Self {
            rational,
            scale: cross,
            disc,
        };
        if product.scale.sign() == Sign::NoSign {
            product.disc = None;
        }
        Some(product)
    }

    fn scale(self, factor: &Rational) -> Self {
        if factor.sign() == Sign::NoSign {
            return Self::zero();
        }
        let mut scaled = Self {
            rational: &self.rational * factor,
            scale: &self.scale * factor,
            disc: self.disc,
        };
        if scaled.scale.sign() == Sign::NoSign {
            scaled.disc = None;
        }
        scaled
    }

    fn inverse(self) -> Option<Self> {
        admit_rational(&self.rational)?;
        admit_rational(&self.scale)?;
        if self.is_zero() {
            return None;
        }
        let Some(disc) = &self.disc else {
            return Some(Self::rational(self.rational.inverse().ok()?));
        };
        let norm = &self.rational * &self.rational - &self.scale * &self.scale * disc;
        if norm.sign() == Sign::NoSign {
            return None;
        }
        let inverse_norm = norm.inverse().ok()?;
        Some(Self {
            rational: &self.rational * &inverse_norm,
            scale: -&self.scale * &inverse_norm,
            disc: Some(disc.clone()),
        })
    }

    fn sign(&self) -> Option<RealSign> {
        admit_rational(&self.rational)?;
        admit_rational(&self.scale)?;
        if self.scale.sign() == Sign::NoSign {
            return Some(rational_sign(&self.rational));
        }
        let disc = self.disc.as_ref()?;
        if disc.sign() != Sign::Plus {
            return None;
        }
        let left = &self.rational * &self.rational;
        let right = &self.scale * &self.scale * disc;
        Some(match left.partial_cmp(&right)? {
            core::cmp::Ordering::Greater => rational_sign(&self.rational),
            core::cmp::Ordering::Less => rational_sign(&self.scale),
            core::cmp::Ordering::Equal => {
                if self.rational.sign() == Sign::NoSign || self.rational.sign() != self.scale.sign()
                {
                    RealSign::Zero
                } else {
                    rational_sign(&self.rational)
                }
            }
        })
    }
}

impl Tower {
    fn quad(even: Quad) -> Self {
        Self {
            even,
            odd: Quad::zero(),
            radicand: None,
        }
    }

    fn rational(value: Rational) -> Self {
        Self::quad(Quad::rational(value))
    }

    fn zero() -> Self {
        Self::rational(Rational::zero())
    }

    fn scale_rational(self, factor: &Rational) -> Self {
        let even = self.even.scale(factor);
        let odd = self.odd.scale(factor);
        Self {
            even,
            odd: odd.clone(),
            radicand: if odd.is_zero() { None } else { self.radicand },
        }
    }

    fn neg(self) -> Self {
        Self {
            even: self.even.neg(),
            odd: self.odd.neg(),
            radicand: self.radicand,
        }
    }

    fn match_radicand(&self, other: &Self) -> Option<Option<Quad>> {
        match (&self.radicand, &other.radicand) {
            (Some(left), Some(right)) if quad_eq(left, right) => Some(Some(left.clone())),
            (Some(radicand), None) | (None, Some(radicand)) => Some(Some(radicand.clone())),
            (None, None) => Some(None),
            _ => None,
        }
    }

    fn add(self, other: Self) -> Option<Self> {
        let radicand = self.match_radicand(&other)?;
        let even = self.even.add(other.even)?;
        let odd = self.odd.add(other.odd)?;
        Some(Self {
            even,
            odd: odd.clone(),
            radicand: if odd.is_zero() { None } else { radicand },
        })
    }

    fn mul(self, other: Self) -> Option<Self> {
        let shared = (|| {
            let radicand = self.match_radicand(&other)?;
            let mut even = self.even.clone().mul(other.even.clone())?;
            let odd = self
                .even
                .clone()
                .mul(other.odd.clone())?
                .add(self.odd.clone().mul(other.even.clone())?)?;
            if !self.odd.is_zero() && !other.odd.is_zero() {
                let radicand = radicand.as_ref()?;
                even = even.add(
                    self.odd
                        .clone()
                        .mul(other.odd.clone())?
                        .mul(radicand.clone())?,
                )?;
            }
            Some(Self {
                even,
                odd: odd.clone(),
                radicand: if odd.is_zero() { None } else { radicand },
            })
        })();
        shared.or_else(|| {
            // Different outer radicals can have a product in the same small
            // tower. Combine their squares before adjoining another root:
            // sqrt(a) / sqrt(b) needs only sqrt(a/b), with its sign retained.
            let square = self.pure_square()?.mul(other.pure_square()?)?;
            let product = sqrt_quad(square)?;
            Some(if self.sign()? == other.sign()? {
                product
            } else {
                product.neg()
            })
        })
    }

    /// Square of a value with no sum across the outer quadratic generator.
    /// A mixed value keeps its two terms and uses the general sign fallback.
    fn pure_square(&self) -> Option<Quad> {
        if self.odd.is_zero() {
            return self.even.clone().mul(self.even.clone());
        }
        if self.even.is_zero() {
            return self
                .odd
                .clone()
                .mul(self.odd.clone())?
                .mul(self.radicand.clone()?);
        }
        None
    }

    fn inverse(self) -> Option<Self> {
        if self.odd.is_zero() {
            return Some(Self::quad(self.even.inverse()?));
        }
        let radicand = self.radicand.as_ref()?;
        let norm = self.even.clone().mul(self.even.clone())?.add(
            self.odd
                .clone()
                .mul(self.odd.clone())?
                .mul(radicand.clone())?
                .neg(),
        )?;
        let inverse_norm = norm.inverse()?;
        Some(Self {
            even: self.even.mul(inverse_norm.clone())?,
            odd: self.odd.mul(inverse_norm)?.neg(),
            radicand: self.radicand,
        })
    }

    /// Sign of `even + odd * R` with principal `R > 0` and `R^2 = radicand`.
    fn sign(&self) -> Option<RealSign> {
        if self.odd.is_zero() {
            return self.even.sign();
        }
        let radicand = self.radicand.as_ref()?;
        match radicand.sign()? {
            RealSign::Negative => return None,
            RealSign::Zero => return self.even.sign(),
            RealSign::Positive => {}
        }
        if self.even.is_zero() {
            return self.odd.sign();
        }
        let even_square = self.even.clone().mul(self.even.clone())?;
        let odd_square = self
            .odd
            .clone()
            .mul(self.odd.clone())?
            .mul(radicand.clone())?;
        match even_square.add(odd_square.neg())?.sign()? {
            RealSign::Positive => self.even.sign(),
            RealSign::Negative => self.odd.sign(),
            RealSign::Zero => {
                let even_sign = self.even.sign()?;
                let odd_sign = self.odd.sign()?;
                if even_sign == RealSign::Zero || even_sign != odd_sign {
                    Some(RealSign::Zero)
                } else {
                    Some(even_sign)
                }
            }
        }
    }
}

fn quad_eq(left: &Quad, right: &Quad) -> bool {
    left.rational == right.rational && left.scale == right.scale && left.disc == right.disc
}

const TOWER_RATIONAL_BIT_LIMIT: u64 = 8_192;

fn admit_rational(value: &Rational) -> Option<()> {
    if value.numerator().bits() <= TOWER_RATIONAL_BIT_LIMIT
        && value.denominator().bits() <= TOWER_RATIONAL_BIT_LIMIT
    {
        Some(())
    } else {
        None
    }
}

fn rational_sign(value: &Rational) -> RealSign {
    match value.sign() {
        Sign::Minus => RealSign::Negative,
        Sign::NoSign => RealSign::Zero,
        Sign::Plus => RealSign::Positive,
    }
}

fn quad_from_square_root(square: &Rational) -> Option<Quad> {
    match square.sign() {
        Sign::Minus => None,
        Sign::NoSign => Some(Quad::zero()),
        Sign::Plus => {
            let (scale, disc) = square.clone().extract_square_reduced_retained();
            if disc.sign() != Sign::Plus || disc.is_one() {
                Some(Quad::rational(scale))
            } else {
                Some(Quad {
                    rational: Rational::zero(),
                    scale,
                    disc: Some(disc),
                })
            }
        }
    }
}

fn signed_magnitude(sign: Sign, magnitude: BigUint) -> BigInt {
    match sign {
        Sign::Minus => -BigInt::from(magnitude),
        Sign::NoSign | Sign::Plus => BigInt::from(magnitude),
    }
}

fn exact_quotient(value: &BigInt, divisor: &BigUint) -> Option<BigInt> {
    if divisor.is_zero() {
        return None;
    }
    let divisor = BigInt::from(divisor.clone());
    if !(value % &divisor).is_zero() {
        return None;
    }
    Some(value / divisor)
}

fn square_parts(magnitude: &BigUint) -> (Rational, Rational) {
    let (square, rest) =
        Rational::from_bigint(BigInt::from(magnitude.clone())).extract_square_reduced_retained();
    // A perfect square is `root^2 * 1`. The legacy encoding uses a zero residual.
    if rest.sign() == Sign::NoSign || rest.is_one() {
        (square, Rational::one())
    } else {
        (square, rest)
    }
}

/// `sqrt(k^2 * q) = |k| * sqrt(q)` for one fixed primitive radicand.
fn sqrt_quad(quad: Quad) -> Option<Tower> {
    match quad.sign()? {
        RealSign::Negative => return None,
        RealSign::Zero => return Some(Tower::zero()),
        RealSign::Positive => {}
    }
    if quad.scale.sign() == Sign::NoSign {
        return Some(Tower::quad(quad_from_square_root(&quad.rational)?));
    }
    let disc = quad.disc.clone()?;
    if disc.sign() != Sign::Plus || disc.denominator() != &BigUint::one() {
        return None;
    }
    let rational_den = quad.rational.denominator().clone();
    let scale_den = quad.scale.denominator().clone();
    let den_gcd = num::Integer::gcd(&rational_den, &scale_den);
    let denominator = (&rational_den / &den_gcd) * &scale_den;
    let rational_coeff = signed_magnitude(
        quad.rational.sign(),
        quad.rational.numerator() * (&denominator / &rational_den),
    );
    let scale_coeff = signed_magnitude(
        quad.scale.sign(),
        quad.scale.numerator() * (&denominator / &scale_den),
    );
    let content = num::Integer::gcd(rational_coeff.magnitude(), scale_coeff.magnitude());
    if content.is_zero() {
        return Some(Tower::zero());
    }
    let (square_content, free_content) = square_parts(&content);
    let primitive_rational = exact_quotient(&rational_coeff, &content)?;
    let primitive_scale = exact_quotient(&scale_coeff, &content)?;
    let (square_denominator, free_denominator) = square_parts(&denominator);
    let odd_factor = &square_content / &(&square_denominator * &free_denominator);
    let radicand_factor = &free_content * &free_denominator;
    let radicand = Quad {
        rational: &radicand_factor * &Rational::from_bigint(primitive_rational),
        scale: &radicand_factor * &Rational::from_bigint(primitive_scale),
        disc: Some(disc),
    };
    if radicand.sign()? != RealSign::Positive {
        return None;
    }
    Some(Tower {
        even: Quad::zero(),
        odd: Quad::rational(odd_factor),
        radicand: Some(radicand),
    })
}

const TOWER_NODE_BUDGET: usize = 2048;

fn power_of_two_factor(shift: i32) -> Option<Rational> {
    if shift >= 0 {
        let bits = usize::try_from(shift).ok()?;
        Some(Rational::from_bigint(BigInt::one() << bits))
    } else {
        let bits = usize::try_from(shift.checked_neg()?).ok()?;
        Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << bits).ok()
    }
}

fn tower_from_computable(value: &Computable) -> Option<Tower> {
    fn parse(
        value: &Computable,
        remaining: &mut usize,
        memo: &mut Vec<(usize, Option<Tower>)>,
    ) -> Option<Tower> {
        let key = std::sync::Arc::as_ptr(&value.internal) as usize;
        if let Some((_, cached)) = memo.iter().find(|(candidate, _)| *candidate == key) {
            return cached.clone();
        }
        if *remaining == 0 {
            return None;
        }
        *remaining -= 1;
        let parsed = if let Some(rational) = value.exact_rational() {
            admit_rational(&rational)?;
            Some(Tower::rational(rational))
        } else {
            match &value.internal.approximation {
                Approximation::Constant(SharedConstant::Sqrt2) => {
                    sqrt_quad(Quad::rational(Rational::new(2)))
                }
                Approximation::Constant(SharedConstant::Sqrt3) => {
                    sqrt_quad(Quad::rational(Rational::new(3)))
                }
                Approximation::Negate(child) => Some(parse(child, remaining, memo)?.neg()),
                Approximation::Offset(child, shift) => Some(
                    parse(child, remaining, memo)?.scale_rational(&power_of_two_factor(*shift)?),
                ),
                Approximation::Add(left, right) => {
                    Some(parse(left, remaining, memo)?.add(parse(right, remaining, memo)?)?)
                }
                Approximation::Multiply(left, right) => {
                    Some(parse(left, remaining, memo)?.mul(parse(right, remaining, memo)?)?)
                }
                Approximation::Inverse(child) => Some(parse(child, remaining, memo)?.inverse()?),
                Approximation::Square(child) => {
                    let child = parse(child, remaining, memo)?;
                    Some(child.clone().mul(child)?)
                }
                Approximation::Sqrt(child) => {
                    sqrt_quad(parse(child, remaining, memo)?.even_quad()?)
                }
                Approximation::NthRoot(child, 2) => {
                    sqrt_quad(parse(child, remaining, memo)?.even_quad()?)
                }
                Approximation::LinearCombination3(combination) => {
                    let mut sum = Tower::zero();
                    for (coefficient, weight) in combination
                        .coefficients
                        .iter()
                        .zip(combination.values.iter())
                    {
                        sum =
                            sum.add(parse(coefficient, remaining, memo)?.scale_rational(weight))?;
                    }
                    Some(sum)
                }
                _ => None,
            }
        };
        memo.push((key, parsed.clone()));
        parsed
    }
    let mut budget = TOWER_NODE_BUDGET;
    parse(value, &mut budget, &mut Vec::new())
}

impl Tower {
    fn even_quad(self) -> Option<Quad> {
        if self.odd.is_zero() {
            Some(self.even)
        } else {
            None
        }
    }
}

fn quad_rational_parts(quad: &Quad) -> Option<(Rational, Rational, Option<Rational>)> {
    admit_rational(&quad.rational)?;
    admit_rational(&quad.scale)?;
    if quad.scale.sign() == Sign::NoSign {
        return Some((quad.rational.clone(), Rational::zero(), None));
    }
    Some((
        quad.rational.clone(),
        quad.scale.clone(),
        Some(quad.disc.clone()?),
    ))
}

fn one_real_outer_square_root(radicand: &Quad) -> bool {
    if radicand.sign() != Some(RealSign::Positive) {
        return false;
    }
    if radicand.scale.sign() == Sign::NoSign {
        return true;
    }
    let conjugate = Quad {
        rational: radicand.rational.clone(),
        scale: -radicand.scale.clone(),
        disc: radicand.disc.clone(),
    };
    conjugate.sign() == Some(RealSign::Negative)
}

fn positive_rational_branch(tower: &Tower) -> Option<Rational> {
    if tower.even.scale.sign() != Sign::NoSign || tower.odd.scale.sign() != Sign::NoSign {
        return None;
    }
    if tower.odd.rational.sign() != Sign::Plus {
        return None;
    }
    if !one_real_outer_square_root(tower.radicand.as_ref()?) {
        return None;
    }
    admit_rational(&tower.even.rational)?;
    Some(tower.even.rational.clone())
}

fn quad_annihilator(quad: &Quad) -> Option<Vec<Rational>> {
    let (constant, scale, disc) = quad_rational_parts(quad)?;
    if scale.sign() == Sign::NoSign {
        return Some(vec![-constant, Rational::one()]);
    }
    let disc = disc?;
    Some(vec![
        &constant * &constant - &disc * &scale * &scale,
        -&Rational::new(2) * &constant,
        Rational::one(),
    ])
}

fn tower_annihilator(tower: &Tower) -> Option<Vec<Rational>> {
    if tower.odd.is_zero() {
        return quad_annihilator(&tower.even);
    }
    let radicand = tower.radicand.as_ref()?;
    let carried = tower
        .odd
        .clone()
        .mul(tower.odd.clone())?
        .mul(radicand.clone())?;
    let remainder = tower
        .even
        .clone()
        .mul(tower.even.clone())?
        .add(carried.neg())?;
    let (a0, a1, disc_a) = quad_rational_parts(&tower.even)?;
    let (c0, c1, disc_c) = quad_rational_parts(&remainder)?;
    let disc = match (disc_a, disc_c) {
        (Some(left), Some(right)) if left == right => left,
        (Some(disc), None) | (None, Some(disc)) => disc,
        (None, None) => Rational::zero(),
        _ => return None,
    };
    let four = Rational::new(4);
    Some(vec![
        &c0 * &c0 - &disc * &c1 * &c1,
        -&four * &a0 * &c0 + &four * &disc * &a1 * &c1,
        &four * &a0 * &a0 + &Rational::new(2) * &c0 - &four * &disc * &a1 * &a1,
        -&four * &a0,
        Rational::one(),
    ])
}

impl Computable {
    pub(crate) fn quadratic_tower_sign(&self) -> Option<RealSign> {
        tower_from_computable(self)?.sign()
    }

    pub(crate) fn quadratic_tower_positive_rational_branch(&self) -> Option<Rational> {
        positive_rational_branch(&tower_from_computable(self)?)
    }

    pub(crate) fn quadratic_tower_annihilating_polynomial(&self) -> Option<Vec<Rational>> {
        tower_annihilator(&tower_from_computable(self)?)
    }

    pub(crate) fn sign_polynomial_at_rational_square_root(
        coefficients: &[Self],
        square: &Rational,
        positive_root: bool,
    ) -> Option<RealSign> {
        if square.sign() == Sign::Minus {
            return None;
        }
        let mut root = if square.sign() == Sign::NoSign {
            Tower::zero()
        } else {
            Tower::quad(quad_from_square_root(square)?)
        };
        if !positive_root {
            root = root.neg();
        }
        let mut power = Tower::rational(Rational::one());
        let mut sum = Tower::zero();
        for coefficient in coefficients {
            sum = sum.add(power.clone().mul(tower_from_computable(coefficient)?)?)?;
            power = power.mul(root.clone())?;
        }
        sum.sign()
    }
}

#[cfg(test)]
mod quadratic_tower_tests {
    use super::*;

    fn sqrt2() -> Computable {
        Computable::rational(Rational::new(2)).sqrt()
    }

    #[test]
    fn nested_unit_normal_square_cancels() {
        let alpha = Computable::rational(Rational::fraction(1, 2).unwrap()).sqrt();
        let radicand =
            Computable::one().add(Computable::rational(Rational::new(4)).multiply(alpha));
        let norm = radicand.clone().sqrt();
        let squared = radicand.multiply(norm.clone().inverse().multiply(norm.inverse()));
        let difference = squared.add(Computable::one().negate());
        assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
    }

    #[test]
    fn square_content_uses_one_primitive_radicand() {
        let primitive =
            Computable::one().add(Computable::rational(Rational::new(2)).multiply(sqrt2()));
        let scaled = Computable::rational(Rational::new(4)).multiply(primitive.clone());
        let difference = scaled.sqrt().add(
            Computable::rational(Rational::new(2))
                .multiply(primitive.sqrt())
                .negate(),
        );
        assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
    }

    #[test]
    fn nested_radical_below_two_is_negative() {
        let inside =
            Computable::one().add(Computable::rational(Rational::new(2)).multiply(sqrt2()));
        let value = inside
            .sqrt()
            .add(Computable::rational(Rational::new(2)).negate());
        assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Negative));
    }

    #[test]
    fn transcendental_nodes_stay_undecided() {
        assert_eq!(Computable::pi().quadratic_tower_sign(), None);
    }
}
