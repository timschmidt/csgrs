impl Computable {
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
