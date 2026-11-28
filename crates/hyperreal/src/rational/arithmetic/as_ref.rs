/// Borrow a rational for generic arithmetic adapters without cloning or reducing.
impl AsRef<Rational> for Rational {
    /// Return this same handle by reference, preserving storage and cache state.
    fn as_ref(&self) -> &Rational {
        self
    }
}

