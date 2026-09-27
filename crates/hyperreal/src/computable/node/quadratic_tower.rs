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
        let disc = self.same_disc(&other)?;
        let cross = Rational::signed_product_sum2(
            [true, true],
            [
                [&self.rational, &other.scale],
                [&self.scale, &other.rational],
            ],
        );
        let rational = if self.scale.sign() != Sign::NoSign && other.scale.sign() != Sign::NoSign {
            let one = Rational::one();
            Rational::signed_product_sum(
                [true, true],
                [
                    [&self.rational, &other.rational, &one],
                    [&self.scale, &other.scale, disc.as_ref()?],
                ],
            )
        } else {
            &self.rational * &other.rational
        };
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
        if self.is_zero() {
            return None;
        }
        let Some(disc) = &self.disc else {
            return Some(Self::rational(self.rational.inverse().ok()?));
        };
        let one = Rational::one();
        let norm = Rational::signed_product_sum(
            [true, false],
            [
                [&self.rational, &self.rational, &one],
                [&self.scale, &self.scale, disc],
            ],
        );
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
        if self.scale.sign() == Sign::NoSign {
            return Some(rational_sign(&self.rational));
        }
        let disc = self.disc.as_ref()?;
        if disc.sign() != Sign::Plus {
            return None;
        }
        if self.rational.sign() == Sign::NoSign || self.rational.sign() == self.scale.sign() {
            return Some(rational_sign(&self.scale));
        }
        let one = Rational::one();
        Some(
            match Rational::signed_product_sum_ordering(
                [true, false],
                [
                    [&self.rational, &self.rational, &one],
                    [&self.scale, &self.scale, disc],
                ],
            ) {
                core::cmp::Ordering::Greater => rational_sign(&self.rational),
                core::cmp::Ordering::Less => rational_sign(&self.scale),
                core::cmp::Ordering::Equal => RealSign::Zero,
            },
        )
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

    fn scale_power_of_two(mut self, shift: i32) -> Option<Self> {
        for quad in [&mut self.even, &mut self.odd] {
            quad.rational = shifted_coefficient(&quad.rational, shift)?;
            quad.scale = shifted_coefficient(&quad.scale, shift)?;
        }
        Some(self)
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

    fn add(mut self, mut other: Self) -> Option<Self> {
        let mut inner = None;
        let compatible = [&self.even, &self.odd, &other.even, &other.odd]
            .into_iter()
            .chain(self.radicand.iter())
            .chain(other.radicand.iter())
            .filter_map(|quad| quad.disc.as_ref())
            .all(|disc| match inner {
                Some(known) => known == disc,
                None => {
                    inner = Some(disc);
                    true
                }
            });
        let mut radicand = self.match_radicand(&other);
        if radicand.is_none() || !compatible {
            (self, other) = self.common_biquadratic_field(&other)?;
            radicand = self.match_radicand(&other);
        }
        let radicand = radicand?;
        let even = self.even.add(other.even)?;
        let odd = self.odd.add(other.odd)?;
        Some(Self {
            even,
            odd: odd.clone(),
            radicand: if odd.is_zero() { None } else { radicand },
        })
    }

    /// Expand only an unnested quadratic tower into rational square classes.
    /// The existing representation then holds any two independent classes;
    /// their product is a basis term, not a third independent extension.
    fn biquadratic_terms(&self) -> Option<[Quad; 3]> {
        let Some(radicand) = &self.radicand else {
            return Some([self.even.clone(), Quad::zero(), Quad::zero()]);
        };
        if radicand.scale.sign() != Sign::NoSign {
            return None;
        }
        let rational_term = quad_from_square_root(&radicand.rational)?.scale(&self.odd.rational);
        let radical_term = match &self.odd.disc {
            Some(disc) => {
                let square = &radicand.rational * disc;
                quad_from_square_root(&square)?.scale(&self.odd.scale)
            }
            None => Quad::zero(),
        };
        Some([self.even.clone(), rational_term, radical_term])
    }

    fn common_biquadratic_field(&self, other: &Self) -> Option<(Self, Self)> {
        let first = self.biquadratic_terms()?;
        let second = other.biquadratic_terms()?;
        let mut discs = first
            .iter()
            .chain(&second)
            .filter_map(|term| term.disc.as_ref());
        let inner = discs.next();
        let outer = discs
            .find(|disc| Some(*disc) != inner)
            .cloned()
            .unwrap_or_else(Rational::one);
        let inverse_outer = outer.clone().inverse().ok()?;
        let rebase = |terms: &[Quad; 3]| {
            let mut even = Quad::zero();
            let mut odd = Quad::zero();
            for term in terms {
                let Some(disc) = &term.disc else {
                    even = even.add(term.clone())?;
                    continue;
                };
                if Some(disc) == inner {
                    even = even.add(term.clone())?;
                    continue;
                }
                // sqrt(disc) = sqrt(disc/outer) * sqrt(outer), with
                // principal positive roots. The quotient must belong to
                // the chosen inner field; a third independent class declines.
                let square = disc * &inverse_outer;
                let coefficient = quad_from_square_root(&square)?;
                if coefficient.disc.is_some() && coefficient.disc.as_ref() != inner {
                    return None;
                }
                even = even.add(Quad::rational(term.rational.clone()))?;
                odd = odd.add(coefficient.scale(&term.scale))?;
            }
            Some(Self {
                even,
                radicand: if odd.is_zero() {
                    None
                } else {
                    Some(Quad::rational(outer.clone()))
                },
                odd,
            })
        };
        Some((rebase(&first)?, rebase(&second)?))
    }

    fn mul(self, other: Self) -> Option<Self> {
        self.multiply_in_tower(&other)
            // Keep an existing biquadratic product in its coefficient field.
            // Squaring a mixed quadratic value and adjoining its square root
            // would obscure the same finite basis behind a nested radical.
            .or_else(|| {
                let (left, right) = self.common_biquadratic_field(&other)?;
                left.multiply_in_tower(&right)
            })
            .or_else(|| {
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

    fn multiply_in_tower(&self, other: &Self) -> Option<Self> {
        let radicand = self.match_radicand(other)?;
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
        let even_sign = self.even.sign()?;
        let odd_sign = self.odd.sign()?;
        if even_sign == RealSign::Zero || even_sign == odd_sign {
            return Some(odd_sign);
        }
        if odd_sign == RealSign::Zero {
            return Some(even_sign);
        }
        let even_square = self.even.clone().mul(self.even.clone())?;
        let odd_square = self
            .odd
            .clone()
            .mul(self.odd.clone())?
            .mul(radicand.clone())?;
        match even_square.add(odd_square.neg())?.sign()? {
            RealSign::Positive => Some(even_sign),
            RealSign::Negative => Some(odd_sign),
            RealSign::Zero => Some(RealSign::Zero),
        }
    }
}

fn quad_eq(left: &Quad, right: &Quad) -> bool {
    left.rational == right.rational && left.scale == right.scale && left.disc == right.disc
}

// Explicit binary shifts can encode enormous allocations in one node.
// This fast path declines those expansions; ordinary field coefficients and
// their retained reductions have no size gate.
const TOWER_SHIFT_ALLOCATION_BITS: u64 = 8_192;

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

fn shifted_coefficient(value: &Rational, shift: i32) -> Option<Rational> {
    if value.sign() == Sign::NoSign || shift == 0 {
        return Some(value.clone());
    }
    let numerator = value.numerator();
    let denominator = value.denominator();
    let bits = u64::from(shift.unsigned_abs());
    // Cancel existing powers of two before checking the result size or
    // allocating. A large shift can still leave a small exact coefficient.
    let (shrink_n, shrink_d, grow_n, grow_d) = if shift > 0 {
        let cancel = bits.min(denominator.trailing_zeros()?);
        (0, cancel, bits - cancel, 0)
    } else {
        let cancel = bits.min(numerator.trailing_zeros()?);
        (cancel, 0, 0, bits - cancel)
    };
    // Only the new expansion is bounded. An already represented wide
    // coefficient remains eligible for cancellation and ordinary small shifts.
    if grow_n > TOWER_SHIFT_ALLOCATION_BITS || grow_d > TOWER_SHIFT_ALLOCATION_BITS {
        return None;
    }
    let numerator =
        (numerator >> usize::try_from(shrink_n).ok()?) << usize::try_from(grow_n).ok()?;
    let denominator =
        (denominator >> usize::try_from(shrink_d).ok()?) << usize::try_from(grow_d).ok()?;
    Rational::from_bigint_fraction(signed_magnitude(value.sign(), numerator), denominator).ok()
}

fn tower_from_computable(value: &Computable) -> Option<Tower> {
    // Traverse the immutable DAG in postorder. Work is proportional to distinct
    // nodes, with no recursive descent or arbitrary expression-width cutoff.
    // Field compatibility controls arithmetic. Successful reductions remain
    // reusable regardless of coefficient size; only explicit binary shifts
    // have a preallocation guard.
    let key = |value: &Computable| Arc::as_ptr(&value.internal);
    let mut memo = std::collections::HashMap::<*const Node, Tower>::new();
    let mut pending = vec![(value, false)];
    while let Some((current, ready)) = pending.pop() {
        let current_key = key(current);
        if memo.contains_key(&current_key) {
            continue;
        }
        if !ready {
            if let Some(tower) = current.internal.cache.quadratic_tower() {
                memo.insert(current_key, tower);
                continue;
            }
            if let Some(rational) = current.exact_rational() {
                memo.insert(current_key, Tower::rational(rational));
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
        let tower = match &current.internal.approximation {
            Approximation::Constant(SharedConstant::Sqrt2) => {
                sqrt_quad(Quad::rational(Rational::new(2)))?
            }
            Approximation::Constant(SharedConstant::Sqrt3) => {
                sqrt_quad(Quad::rational(Rational::new(3)))?
            }
            Approximation::Negate(value) => child(value)?.neg(),
            Approximation::Offset(value, shift) => child(value)?.scale_power_of_two(*shift)?,
            Approximation::Add(left, right) => child(left)?.add(child(right)?)?,
            Approximation::Multiply(left, right) => child(left)?.mul(child(right)?)?,
            Approximation::Inverse(value) => child(value)?.inverse()?,
            Approximation::Square(value) => {
                let value = child(value)?;
                value.clone().mul(value)?
            }
            Approximation::Sqrt(value) | Approximation::NthRoot(value, 2) => {
                sqrt_quad(child(value)?.even_quad()?)?
            }
            Approximation::LinearCombination3(combination) => {
                let mut sum = Tower::zero();
                for (coefficient, weight) in combination
                    .coefficients
                    .iter()
                    .zip(combination.values.iter())
                {
                    sum = sum.add(child(coefficient)?.scale_rational(weight))?;
                }
                sum
            }
            _ => return None,
        };
        // A completed shared reduction proves this subexpression even if its
        // parent later leaves the supported field. No failed result is cached.
        if Arc::strong_count(&current.internal) > 1
            && !matches!(current.internal.approximation, Approximation::Constant(_))
        {
            current.internal.cache.store_quadratic_tower(&tower);
        }
        memo.insert(current_key, tower);
    }
    memo.remove(&key(value))
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
    /// Reuse an already proved exact reduction when evaluating this node.
    /// This does not discover a tower or change the original expression: a
    /// missing reduction leaves the ordinary approximation kernel in charge.
    fn cached_quadratic_tower_approximation(
        &self,
        signal: &Option<Signal>,
        precision: Precision,
    ) -> Option<BigInt> {
        fn quadratic(value: Quad) -> Option<Computable> {
            let rational = Computable::rational(value.rational);
            if value.scale.sign() == Sign::NoSign {
                Some(rational)
            } else {
                Some(
                    rational.add(
                        Computable::rational(value.disc?)
                            .sqrt()
                            .multiply_rational(value.scale),
                    ),
                )
            }
        }
        let tower = self.internal.cache.quadratic_tower()?;
        let even = quadratic(tower.even)?;
        let reduced = if tower.odd.is_zero() {
            even
        } else {
            even.add(quadratic(tower.odd)?.multiply(quadratic(tower.radicand?)?.sqrt()))
        };
        // A canonical shared constant or already compact node may reconstruct
        // itself. Its normal approximation kernel remains the terminal path.
        if Arc::ptr_eq(&self.internal, &reduced.internal) {
            return None;
        }
        crate::trace_dispatch!("computable", "approx", "cached-quadratic-tower");
        Some(reduced.approx_signal(signal, precision))
    }

    pub(crate) fn quadratic_tower_parts(&self) -> Option<[[Rational; 3]; 3]> {
        let tower = tower_from_computable(self)?;
        // Reuse the same exact reduction for compaction, signs and subsequent
        // arithmetic. The rows describe even + odd * sqrt(radicand), each as
        // a + b * sqrt(d); they do not expose another public scalar carrier.
        self.internal.cache.store_quadratic_tower(&tower);
        let parts = |quad: Quad| {
            [
                quad.rational,
                quad.scale,
                quad.disc.unwrap_or_else(Rational::zero),
            ]
        };
        Some([
            parts(tower.even),
            parts(tower.odd),
            parts(tower.radicand.unwrap_or_else(Quad::zero)),
        ])
    }

    pub(crate) fn quadratic_tower_sign(&self) -> Option<RealSign> {
        let tower = tower_from_computable(self)?;
        let sign = tower.sign()?;
        // Retain the successful proof at the queried root. Later arithmetic
        // can reuse its finite basis without replaying its construction DAG.
        // Failed reductions remain retryable.
        self.internal.cache.store_quadratic_tower(&tower);
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(private_sign(sign)));
        Some(sign)
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

    #[test]
    fn quadratic_norms_preserve_wide_cancellation_and_products() {
        for a in -5..=5 {
            for b in -5..=5 {
                let value = Quad {
                    rational: Rational::new(a),
                    scale: Rational::new(b),
                    disc: Some(Rational::new(4)),
                };
                assert_eq!(value.sign(), Some(rational_sign(&Rational::new(a + 2 * b))));
            }
        }
        let epsilon = Rational::from_bigint(BigInt::one() << 1024)
            .inverse()
            .unwrap();
        for direction in [-1, 1] {
            let value = Quad {
                rational: Rational::new(2) + &epsilon * Rational::new(direction),
                scale: Rational::new(-1),
                disc: Some(Rational::new(4)),
            };
            let expected = if direction < 0 {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            assert_eq!(value.sign(), Some(expected));
            assert_eq!(
                value.neg().sign(),
                Some(if direction < 0 {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                })
            );
        }
        for a in [-7, -1, 0, 1, 7] {
            for b in [-7, -1, 0, 1, 7] {
                let value = Quad {
                    rational: Rational::new(a),
                    scale: Rational::new(b),
                    disc: Some(Rational::new(5)),
                };
                let conjugate = Quad {
                    scale: Rational::new(-b),
                    ..value.clone()
                };
                let norm = value.clone().mul(conjugate).unwrap();
                assert_eq!(norm.rational, Rational::new(a * a - 5 * b * b));
                assert!(norm.scale.is_zero());
                if a != 0 || b != 0 {
                    let inverse = value.clone().inverse().unwrap();
                    let identity = value.mul(inverse).unwrap();
                    assert_eq!(identity.rational, Rational::one());
                    assert!(identity.scale.is_zero());
                }
            }
        }
    }

    #[test]
    fn tower_signs_preserve_equal_opposed_and_zero_coefficient_branches() {
        for (even, odd, expected) in [
            (0, 0, RealSign::Zero),
            (0, 1, RealSign::Positive),
            (0, -1, RealSign::Negative),
            (1, 1, RealSign::Positive),
            (-1, -1, RealSign::Negative),
            (1, -1, RealSign::Negative),
            (-1, 1, RealSign::Positive),
            (2, -1, RealSign::Positive),
            (-2, 1, RealSign::Negative),
        ] {
            let value = Tower {
                even: Quad::rational(Rational::new(even)),
                odd: Quad::rational(Rational::new(odd)),
                radicand: Some(Quad::rational(Rational::new(2))),
            };
            assert_eq!(value.sign(), Some(expected));
        }
        for direction in [-1, 1] {
            let value = Tower {
                even: Quad::rational(Rational::new(2 * direction)),
                odd: Quad::rational(Rational::new(-direction)),
                radicand: Some(Quad::rational(Rational::new(4))),
            };
            assert_eq!(value.sign(), Some(RealSign::Zero));
        }
        let cancelled = Quad {
            rational: Rational::new(-2),
            scale: Rational::one(),
            disc: Some(Rational::new(4)),
        };
        for (even, odd, expected) in [
            (
                cancelled.clone(),
                Quad::rational(Rational::new(-1)),
                RealSign::Negative,
            ),
            (
                Quad::rational(Rational::one()),
                cancelled,
                RealSign::Positive,
            ),
        ] {
            let value = Tower {
                even,
                odd,
                radicand: Some(Quad::rational(Rational::new(2))),
            };
            assert_eq!(value.sign(), Some(expected));
        }
    }

    #[test]
    fn cold_tower_proofs_replay_wide_dags_without_refinement() {
        let opaque = |approximation| Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Unknown,
            )),
            signal: None,
        };
        let root = Computable::sqrt_rational(Rational::new(3));
        let mut layer = (1..=1024)
            .map(|index| {
                opaque(Approximation::Add(
                    root.clone(),
                    Computable::rational(Rational::new(index)),
                ))
            })
            .collect::<Vec<_>>();
        while layer.len() > 1 {
            layer = layer
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pair| opaque(Approximation::Add(pair[0].clone(), pair[1].clone())))
                .collect();
        }
        let sum = layer.pop().unwrap();
        // This cold DAG exceeds the former visit limit while its reduced
        // value has only two small coefficients.
        assert_eq!(sum.immediate_sign(), None);
        assert_eq!(sum.quadratic_tower_sign(), Some(RealSign::Positive));
        assert_eq!(sum.clone().immediate_sign(), Some(RealSign::Positive));
        let expected = root
            .multiply_rational(Rational::new(1024))
            .add(Computable::rational(Rational::new(1024 * 1025 / 2)));
        let zero = opaque(Approximation::Add(sum, expected.negate()));
        assert_eq!(zero.quadratic_tower_sign(), Some(RealSign::Zero));
        assert_eq!(zero.clone().immediate_sign(), Some(RealSign::Zero));
        assert!(zero.cached().is_none());

        let outside = opaque(Approximation::Add(
            Computable::sqrt_rational(Rational::new(2))
                .add(Computable::sqrt_rational(Rational::new(3))),
            Computable::sqrt_rational(Rational::new(5)),
        ));
        assert_eq!(outside.quadratic_tower_sign(), None);
        assert_eq!(outside.immediate_sign(), None);
        assert!(outside.internal.cache.cell().is_none());
    }

    fn sqrt2() -> Computable {
        Computable::rational(Rational::new(2)).sqrt()
    }

    #[test]
    fn tower_replay_handles_deep_and_exponentially_shared_graphs() {
        let opaque = |approximation| Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Unknown,
            )),
            signal: None,
        };
        let root = sqrt2();
        let mut chain = vec![root.clone()];
        for _ in 0..8192 {
            chain.push(opaque(Approximation::Negate(chain.last().unwrap().clone())));
        }
        assert_eq!(
            chain.last().unwrap().quadratic_tower_sign(),
            Some(RealSign::Positive)
        );
        // Keep teardown independent of recursive Arc destruction.
        while chain.pop().is_some() {}

        let mut doubled = root.clone();
        for _ in 0..32 {
            doubled = opaque(Approximation::Add(doubled.clone(), doubled));
        }
        assert_eq!(doubled.quadratic_tower_sign(), Some(RealSign::Positive));
        let expected = root.multiply_rational(Rational::from_bigint(BigInt::one() << 32));
        assert_eq!(
            doubled.add(expected.negate()).quadratic_tower_sign(),
            Some(RealSign::Zero)
        );
        for shift in [i32::MIN, -100_000, 100_000, i32::MAX] {
            let wide = opaque(Approximation::Offset(sqrt2(), shift));
            assert_eq!(wide.quadratic_tower_sign(), None);
            assert_eq!(wide.immediate_sign(), None);
            assert!(wide.internal.cache.cell().is_none());
            let zero = opaque(Approximation::Offset(Computable::zero(), shift));
            assert_eq!(zero.quadratic_tower_sign(), Some(RealSign::Zero));
        }
        let large = Rational::from_bigint(BigInt::one() << 8000);
        for (coefficient, shift, expected) in [
            (large.clone().inverse().unwrap(), 16000, large.clone()),
            (large.clone(), -16000, large.inverse().unwrap()),
        ] {
            let scaled = opaque(Approximation::Offset(
                opaque(Approximation::Multiply(
                    sqrt2(),
                    Computable::rational(coefficient),
                )),
                shift,
            ));
            assert_eq!(scaled.quadratic_tower_sign(), Some(RealSign::Positive));
            let difference = opaque(Approximation::Add(
                scaled,
                sqrt2().multiply_rational(expected).negate(),
            ));
            assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
        }
    }

    #[test]
    fn retained_forms_preserve_shared_dag_memoization() {
        let root = sqrt2().add(Computable::one());
        assert_eq!(root.quadratic_tower_sign(), Some(RealSign::Positive));
        let mut layer = vec![root.clone(); 2048];
        while layer.len() > 1 {
            layer = layer
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pair| Computable {
                    internal: Arc::new(Node::new(
                        Approximation::Add(pair[0].clone(), pair[1].clone()),
                        BoundCache::Invalid,
                        ExactSignCache::Unknown,
                    )),
                    signal: None,
                })
                .collect();
        }
        // Shared retained values participate in the same per-call memo as
        // freshly reduced nodes.
        let sum = layer.pop().unwrap();
        assert_eq!(sum.quadratic_tower_sign(), Some(RealSign::Positive));
        let difference = sum.add(root.multiply_rational(Rational::new(2048)).negate());
        assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
    }

    #[test]
    fn approximation_reuses_a_retained_tower_without_evaluating_its_history() {
        let opaque = |approximation| Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Unknown,
            )),
            signal: None,
        };
        let mut chain = vec![sqrt2()];
        for _ in 0..2048 {
            chain.push(opaque(Approximation::Multiply(
                chain.last().unwrap().clone(),
                Computable::one(),
            )));
        }
        let value = chain.last().unwrap();
        assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Positive));
        for precision in [-32, -96, -48] {
            let approximation = value.approx(precision);
            let lower = &approximation - BigInt::one();
            let upper = &approximation + BigInt::one();
            let square = BigInt::from(2) << (-2 * precision) as usize;
            assert!(&lower * &lower <= square);
            assert!(&upper * &upper >= square);
            assert!(
                chain[1..chain.len() - 1]
                    .iter()
                    .all(|node| node.cached().is_none())
            );
        }
        // The same evidence is useful below an unreduced general expression,
        // not only when the caller asks for the retained root directly.
        let parent = opaque(Approximation::Multiply(value.clone(), Computable::pi()));
        let expected = sqrt2().multiply(Computable::pi());
        assert!((parent.approx(-160) - expected.approx(-160)).abs() <= BigInt::from(2));
        assert!(
            chain[1..chain.len() - 1]
                .iter()
                .all(|node| node.cached().is_none())
        );
        drop(parent);
        while chain.pop().is_some() {}
    }

    #[test]
    fn retained_tower_approximation_preserves_nested_radicals_and_signed_coefficients() {
        let expression = || {
            let root = sqrt2();
            let nested = Computable::one()
                .add(root.clone().multiply_rational(Rational::new(2)))
                .sqrt();
            Computable::rational(Rational::new(3))
                .add(root.clone().multiply_rational(Rational::new(-2)))
                .add(
                    root.add(Computable::rational(Rational::new(-5)))
                        .multiply(nested),
                )
        };
        for inverse in [false, true] {
            let original = expression();
            let retained = expression();
            let (original, retained) = if inverse {
                (original.inverse(), retained.inverse())
            } else {
                (original, retained)
            };
            assert_eq!(retained.quadratic_tower_sign(), Some(RealSign::Negative));
            assert!(retained.cached().is_none());
            for precision in [-16, -64, -128, -32] {
                // Both independent evaluations must enclose the same exact
                // value within one unit at the requested binary scale.
                assert!(
                    (retained.approx(precision) - original.approx(precision)).abs()
                        <= BigInt::from(2)
                );
            }
        }
    }

    #[test]
    fn exact_tower_replay_retains_wide_cancellation_proofs() {
        let opaque = |approximation| Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Unknown,
            )),
            signal: None,
        };
        let large = Rational::from_bigint((BigInt::one() << 5000) + BigInt::from(3));
        let root = Computable::one().add(sqrt2()).sqrt();
        let scaled = opaque(Approximation::Multiply(
            Computable::rational(large.clone()),
            Computable::one().add(root.clone()),
        ));
        let square = opaque(Approximation::Square(scaled));
        // (1 + sqrt(1 + sqrt(2)))^2 = 2 + sqrt(2) + 2 sqrt(1 + sqrt(2)).
        let expected = opaque(Approximation::Multiply(
            Computable::rational(&large * &large),
            Computable::rational(Rational::new(2))
                .add(sqrt2())
                .add(root.multiply_rational(Rational::new(2))),
        ));
        let difference = opaque(Approximation::Add(
            square.clone(),
            opaque(Approximation::Negate(expected.clone())),
        ));
        assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
        assert!(difference.cached().is_none());
        assert!(difference.internal.cache.quadratic_tower().is_some());
        assert!(square.internal.cache.quadratic_tower().is_some());
        assert!(square.quadratic_tower_parts().is_some());
        assert!(square.quadratic_tower_annihilating_polynomial().is_some());
        assert!(expected.internal.cache.quadratic_tower().is_some());
        // A small binary shift must not discard an already retained wide value.
        for shift in [-1, 1] {
            let shifted = opaque(Approximation::Offset(square.clone(), shift));
            assert_eq!(shifted.quadratic_tower_sign(), Some(RealSign::Positive));
            assert!(shifted.internal.cache.quadratic_tower().is_some());
            assert!(shifted.cached().is_none());
        }
        let epsilon = Rational::from_bigint(BigInt::one() << 600)
            .inverse()
            .unwrap();
        for (delta, sign) in [
            (epsilon.clone(), RealSign::Positive),
            (-epsilon, RealSign::Negative),
        ] {
            assert_eq!(
                difference
                    .clone()
                    .add(Computable::rational(delta))
                    .quadratic_tower_sign(),
                Some(sign),
            );
        }
    }

    #[test]
    fn tower_and_approximation_caches_can_publish_concurrently() {
        let value = sqrt2().add(Computable::sqrt_rational(Rational::new(3)));
        std::thread::scope(|scope| {
            for precision in [-48, -96, -64, -128] {
                let value = &value;
                scope.spawn(move || {
                    assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Positive));
                    let approximation = value.approx(precision);
                    assert!(approximation > (BigInt::from(3) << -precision));
                    assert!(approximation < (BigInt::from(4) << -precision));
                    assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Positive));
                });
            }
        });
        assert!(matches!(value.cached(), Some((precision, _)) if precision <= -128));
        assert!(value.internal.cache.quadratic_tower().is_some());
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
