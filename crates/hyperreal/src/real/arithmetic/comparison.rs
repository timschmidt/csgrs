/// Structural opposite-scale recognition without negating or refining the basis.
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

/// Conservative representation equality: equal rational scales and matching symbolic bases.
/// Named bases compare their class parameters; opaque bases compare retained graph
/// structure. False means this test did not establish equality, not a proof of
/// distinct mathematical values. Use Real::certified_eq_until for a qualified
/// semantic decision; this implementation deliberately does not provide Eq.
impl PartialEq for Real {
    /// Compare scales first, then basis identity; do not run numerical refinement.
    fn eq(&self, other: &Self) -> bool {
        self.rational == other.rational && self.same_symbolic_basis(other)
    }
}

/// Project certified scalar ordering to Option using Real::PARTIAL_CMP_MIN_PRECISION.
/// None retains an unresolved comparison, but the trait discards the certificate
/// and failure detail. The precision floor is not a total-work bound. This path
/// can prove semantic equality that the structural PartialEq test does not recognize.
/// For such representations, Some(Equal) therefore does not imply `==`; callers
/// cannot rely on that usual PartialOrd/PartialEq consistency rule.
impl PartialOrd for Real {
    /// Run the certified decision cascade at the default floor and retain only its ordering.
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.certified_cmp_until(other, Self::PARTIAL_CMP_MIN_PRECISION)
            .ordering()
    }
}

/// Compare against the exact binary value of a finite f64 through structural Real equality.
/// NaN and either infinity compare false. Decimal 0.1 and 0.1_f64 need not agree:
/// this conversion preserves the binary fraction rather than rounding to a decimal.
impl PartialEq<f64> for Real {
    /// Convert the float exactly; conversion rejection maps to false for this trait.
    fn eq(&self, other: &f64) -> bool {
        Real::try_from(*other).is_ok_and(|other| self == &other)
    }
}

/// Symmetric float-left form of Real structural equality with an exact finite float import.
impl PartialEq<Real> for f64 {
    /// Delegate to the Real-left implementation, including its nonfinite rejection.
    fn eq(&self, other: &Real) -> bool {
        other == self
    }
}

/// Compare a Real with the exact finite binary float value; reject nonfinite inputs as None.
impl PartialOrd<f64> for Real {
    /// Import the float exactly, then use the default certified Real ordering projection.
    fn partial_cmp(&self, other: &f64) -> Option<Ordering> {
        Real::try_from(*other)
            .ok()
            .and_then(|other| self.partial_cmp(&other))
    }
}

/// Float-left ordering after exact import, with the same precision floor and None outcomes.
impl PartialOrd<Real> for f64 {
    /// Preserve operand order while mapping invalid floats and unresolved decisions to None.
    fn partial_cmp(&self, other: &Real) -> Option<Ordering> {
        Real::try_from(*self)
            .ok()
            .and_then(|this| this.partial_cmp(other))
    }
}

/// Recognize a Rational only when the Real already uses the unit basis and equal scale.
/// Other bases decline without attempting semantic reduction, even if their value is rational.
impl PartialEq<Rational> for Real {
    /// Require Class::One before comparing the exact rational coefficient.
    fn eq(&self, other: &Rational) -> bool {
        self.class == Class::One && self.rational == *other
    }
}

/// Symmetric Rational-left unit-basis recognition; no approximation or general identity search.
impl PartialEq<Real> for Rational {
    /// Require the Real unit basis, then compare the retained exact rational value.
    fn eq(&self, other: &Real) -> bool {
        other.class == Class::One && *self == other.rational
    }
}
