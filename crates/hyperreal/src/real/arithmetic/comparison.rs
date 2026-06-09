// Best efforts only, definitely not adequate for Eq
// Requirements: PartialEq should be transitive and symmetric
// however it needn't be complete or reflexive.
impl PartialEq for Real {
    fn eq(&self, other: &Self) -> bool {
        self.rational == other.rational && self.class == other.class
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

