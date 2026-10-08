// Best efforts only, definitely not adequate for Eq
// Requirements: PartialEq should be transitive and symmetric
// however it needn't be complete or reflexive.
impl Real {
    /// Returns whether both values have the same exact symbolic basis and
    /// opposite rational scales.
    ///
    /// This is a structural exactness query: `true` certifies `self == -other`
    /// without constructing a negated [`Real`], while `false` does not rule out
    /// equality through a more general symbolic identity.
    #[must_use]
    #[inline]
    pub fn is_structural_negation_of(&self, other: &Self) -> bool {
        if !self.same_symbolic_basis(other) {
            return false;
        }
        let left = &self.rational;
        let right = &other.rational;
        if left.is_zero() || right.is_zero() {
            return left.is_zero() && right.is_zero();
        }
        left.is_negative() != right.is_negative()
            && left.numerator() == right.numerator()
            && left.denominator() == right.denominator()
    }
}

impl PartialEq for Real {
    fn eq(&self, other: &Self) -> bool {
        self.rational == other.rational && self.same_symbolic_basis(other)
    }
}

impl PartialOrd for Real {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.certified_cmp_until(other, Self::PARTIAL_CMP_MIN_PRECISION)
            .ordering()
    }
}

impl PartialEq<f64> for Real {
    fn eq(&self, other: &f64) -> bool {
        Real::try_from(*other).is_ok_and(|other| self == &other)
    }
}

impl PartialEq<Real> for f64 {
    fn eq(&self, other: &Real) -> bool {
        other == self
    }
}

impl PartialOrd<f64> for Real {
    fn partial_cmp(&self, other: &f64) -> Option<Ordering> {
        Real::try_from(*other)
            .ok()
            .and_then(|other| self.partial_cmp(&other))
    }
}

impl PartialOrd<Real> for f64 {
    fn partial_cmp(&self, other: &Real) -> Option<Ordering> {
        Real::try_from(*self)
            .ok()
            .and_then(|this| this.partial_cmp(other))
    }
}

// For a rational this definitely works
impl PartialEq<Rational> for Real {
    fn eq(&self, other: &Rational) -> bool {
        self.class == Class::One && self.rational == *other
    }
}

// Symmetry
impl PartialEq<Real> for Rational {
    fn eq(&self, other: &Real) -> bool {
        other.class == Class::One && *self == other.rational
    }
}
