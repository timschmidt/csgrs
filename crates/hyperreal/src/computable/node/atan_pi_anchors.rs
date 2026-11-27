#[derive(Clone, Copy)]
/// Exact correspondence between three positive principal arctangent arguments and rational multiples of pi.
/// Keep forward angle construction and argument recovery together so later trig
/// operations can consume the normalized pi representation without numerical
/// rediscovery. This finite anchor table is not a general inverse-trig solver.
enum AtanPiAnchor {
    /// Argument 1, whose principal arctangent is pi/4.
    Unit,
    /// Argument sqrt(3), whose principal arctangent is pi/3.
    SqrtThree,
    /// Argument 1/sqrt(3) = sqrt(3)/3, whose principal arctangent is pi/6.
    InverseSqrtThree,
}

impl AtanPiAnchor {
    /// Recognize the exact coefficient of a positive sqrt(3) argument: one or one third.
    /// The caller identifies the radicand and handles sign separately. Other scales
    /// return None; exact rational comparison, not a tolerance, selects an anchor.
    fn from_root_three_scale(scale: &Rational) -> Option<Self> {
        if scale.is_one() {
            Some(Self::SqrtThree)
        } else if *scale == Rational::fraction(1, 3).expect("three is nonzero") {
            Some(Self::InverseSqrtThree)
        } else {
            None
        }
    }

    /// Recognize a positive coefficient q of pi whose canonical numerator is one and denominator is 3, 4 or 6.
    /// Despite the turn name, q is measured in pi units, not full 2*pi revolutions.
    /// Reject other signs/numerators before the denominator lookup. Rational access
    /// may normalize stored data; there is no independent coefficient-work limit.
    fn from_positive_turn(turn: &Rational) -> Option<Self> {
        if turn.sign() != Sign::Plus || !turn.numerator().is_one() {
            return None;
        }
        Self::from_denominator(turn.denominator())
    }

    /// Map exactly 3, 4 or 6 to the anchor for pi divided by that denominator.
    /// A failed u64 conversion or another value declines. This only examines the
    /// denominator; callers establish any numerator, sign and reduction conditions.
    fn from_denominator(denominator: &BigUint) -> Option<Self> {
        match num::ToPrimitive::to_u64(denominator)? {
            3 => Some(Self::SqrtThree),
            4 => Some(Self::Unit),
            6 => Some(Self::InverseSqrtThree),
            _ => None,
        }
    }

    /// Construct the signed principal angle for this anchor, assuming sign is Plus or Minus.
    /// Use the compact binary shift for pi/4 and exact rational multiplication for
    /// pi/3 or pi/6. Fixed nonzero denominators justify the construction expects.
    /// NoSign violates the caller contract and triggers a debug assertion; it is not
    /// interpreted as a request for a zero angle. No numerical angle evaluation occurs.
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

    /// Construct the positive exact tangent argument corresponding to this principal angle.
    /// Return one, the canonical sqrt(3), or its inverse. Availability of the canonical
    /// root-three constructor is an invariant behind expect; no approximate fitting is used.
    fn argument(self) -> Computable {
        match self {
            Self::Unit => Computable::one(),
            Self::SqrtThree => Computable::sqrt_constant(3).expect("sqrt(3) is canonical"),
            Self::InverseSqrtThree => Computable::sqrt_constant(3)
                .expect("sqrt(3) is canonical")
                .inverse(),
        }
    }

    /// Recover (sine sign, cosine sign, positive principal tangent argument) from an exact coefficient q of pi.
    /// First require canonical denominator 3, 4 or 6. For |q|=k+f, reduce f to the
    /// first quadrant by reflecting fractions above one half. Integer parity flips
    /// both trig signs, reflection flips cosine, and a negative q flips sine.
    /// The returned argument describes the reduced anchor, not generally tan(q*pi)
    /// with its sign. Integers, half-integers and all other denominators decline.
    ///
    /// All reduction uses rational/integer arithmetic and preserves arbitrarily large
    /// finite period counts. Numerator size still affects normalization, division and
    /// allocation; this helper adds no total-work, memory or cancellation bound.
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
