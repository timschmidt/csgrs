// Selected principal arctangents already normalized by native construction.
// Keep the forward angle and recovered argument in one correspondence so
// subsequent trig operations can consume the normalized pi representation.
#[derive(Clone, Copy)]
enum AtanPiAnchor {
    Unit,
    SqrtThree,
    InverseSqrtThree,
}

impl AtanPiAnchor {
    fn from_root_three_scale(scale: &Rational) -> Option<Self> {
        if scale.is_one() {
            Some(Self::SqrtThree)
        } else if *scale == Rational::fraction(1, 3).expect("three is nonzero") {
            Some(Self::InverseSqrtThree)
        } else {
            None
        }
    }

    fn from_positive_turn(turn: &Rational) -> Option<Self> {
        if turn.sign() != Sign::Plus || !turn.numerator().is_one() {
            return None;
        }
        Self::from_denominator(turn.denominator())
    }

    fn from_denominator(denominator: &BigUint) -> Option<Self> {
        match num::ToPrimitive::to_u64(denominator)? {
            3 => Some(Self::SqrtThree),
            4 => Some(Self::Unit),
            6 => Some(Self::InverseSqrtThree),
            _ => None,
        }
    }

    fn angle(self, sign: Sign) -> Computable {
        debug_assert_ne!(sign, Sign::NoSign);
        if let Self::Unit = self {
            let angle = Computable::pi().shift_right(2);
            return if sign == Sign::Minus {
                angle.negate()
            } else {
                angle
            };
        }
        let denominator = match self {
            Self::SqrtThree => 3,
            Self::InverseSqrtThree => 6,
            Self::Unit => unreachable!("unit angle returned above"),
        };
        Computable::pi().multiply_rational(
            Rational::fraction(if sign == Sign::Minus { -1 } else { 1 }, denominator)
                .expect("anchor denominator is nonzero"),
        )
    }

    fn argument(self) -> Computable {
        match self {
            Self::Unit => Computable::one(),
            Self::SqrtThree => Computable::sqrt_constant(3).expect("sqrt(3) is canonical"),
            Self::InverseSqrtThree => Computable::sqrt_constant(3)
                .expect("sqrt(3) is canonical")
                .inverse(),
        }
    }

    fn periodic_trig_argument(pi: Rational) -> Option<(Sign, Sign, Computable)> {
        Self::from_denominator(pi.denominator())?;
        let negative = pi.sign() == Sign::Minus;
        let magnitude = if negative { pi.neg() } else { pi };
        let odd = magnitude.shifted_big_integer(0).bit(0);
        let fraction = magnitude.fract();
        let reflected = fraction > *HALF_RATIONAL;
        let reduced = if reflected {
            Rational::one() - fraction
        } else {
            fraction
        };
        let argument = Self::from_positive_turn(&reduced)?.argument();
        // For q=k+f>0, integer-pi parity changes both signs. Reflection
        // f -> 1-f changes only cosine; q -> -q changes only sine.
        Some((
            if negative ^ odd {
                Sign::Minus
            } else {
                Sign::Plus
            },
            if reflected ^ odd {
                Sign::Minus
            } else {
                Sign::Plus
            },
            argument,
        ))
    }
}
