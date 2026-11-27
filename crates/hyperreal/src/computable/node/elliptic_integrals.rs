/// Retain Carlson integral definitions with domain-encoding children and structural positivity.
impl Computable {
    /// Build RF or RD from `[root_x, log_y, log_z]`, not three unencoded arguments.
    /// The kernel reconstructs `[root_x^2, exp(log_y), exp(log_z)]`, giving its
    /// admitted domain when all children are finite and valid. This constructor
    /// performs no child-domain validation. Box the retained arguments, select RD
    /// when second is true, and initialize a positive sign fact without a magnitude
    /// bound. Inherit the first present child signal in argument order; signals are
    /// not merged. Numerical approximation and caching remain with the shared node.
    pub(crate) fn elliptic_carlson(arguments: [Self; 3], second: bool) -> Self {
        let signal = arguments.iter().find_map(|value| value.signal.clone());
        let payload = Box::new(arguments);
        let approximation = if second {
            Approximation::EllipticRd(payload)
        } else {
            Approximation::EllipticRf(payload)
        };
        Self {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal,
        }
    }
}
