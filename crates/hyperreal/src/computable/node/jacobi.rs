impl Computable {
    fn jacobi_square_product(&self, other: &Self) -> bool {
        matches!(self.internal.approximation,
            Approximation::JacobiSn(_) | Approximation::JacobiCn(_) | Approximation::JacobiDn(_))
            && Self::internal_structural_eq(self, other)
    }

    // The payload stores an amplitude, not an integral value. This makes
    // every finite imported payload valid and keeps all three values on the
    // same retained inverse node and its cache.
    pub(crate) fn jacobi_component(self, log_complement: Self, kind: usize) -> Self {
        let signal = self.signal.clone().or_else(|| log_complement.signal.clone());
        if kind == 2 && log_complement.exact_sign() == Some(Sign::NoSign) {
            let mut value = Self::one();
            value.signal = signal;
            return value;
        }
        let payload = Box::new([self, log_complement]);
        let approximation = match kind {
            0 => Approximation::JacobiSn(payload),
            1 => Approximation::JacobiCn(payload),
            2 => Approximation::JacobiDn(payload),
            _ => unreachable!("three Jacobi components"),
        };
        Self {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                if kind == 2 { ExactSignCache::Valid(Sign::Plus) } else { ExactSignCache::Invalid },
            )),
            signal,
        }
    }

    fn jacobi_square_term(&self, budget: u8) -> Option<(usize, &[Computable; 2], Self)> {
        if budget == 0 { return None; }
        match &self.internal.approximation {
            Approximation::Square(child) => {
                let (kind, arguments) = match &child.internal.approximation {
                    Approximation::JacobiSn(arguments) => (0, arguments),
                    Approximation::JacobiCn(arguments) => (1, arguments),
                    Approximation::JacobiDn(arguments) => (2, arguments),
                    _ => return None,
                };
                Some((kind, arguments, Self::one()))
            }
            Approximation::Negate(child) => {
                let (kind, arguments, scale) = child.jacobi_square_term(budget - 1)?;
                Some((kind, arguments, scale.negate()))
            }
            Approximation::Offset(child, shift) => {
                let (kind, arguments, scale) = child.jacobi_square_term(budget - 1)?;
                Some((kind, arguments, scale.shift_left(*shift)))
            }
            Approximation::Multiply(left, right) => {
                // Follow at most one successful square spine. The depth and
                // coefficient comparison are bounded optional recognition.
                if let Some((kind, arguments, scale)) = left.jacobi_square_term(budget - 1) {
                    Some((kind, arguments, scale.multiply(right.clone())))
                } else {
                    let (kind, arguments, scale) = right.jacobi_square_term(budget - 1)?;
                    Some((kind, arguments, scale.multiply(left.clone())))
                }
            }
            _ => None,
        }
    }

    fn jacobi_square_sum(&self, other: &Self) -> Option<Self> {
        fn equal(left: &Computable, right: &Computable) -> bool {
            Computable::internal_structural_eq(left, right)
                || Computable {
                    internal: Arc::new(Node::new(
                        Approximation::Add(left.clone(), right.clone().negate()),
                        BoundCache::Invalid, ExactSignCache::Invalid,
                    )),
                    signal: left.signal.clone().or_else(|| right.signal.clone()),
                }.bounded_ring_rational()
                    .is_some_and(|value| value.is_zero())
        }
        let (a, left, x) = self.jacobi_square_term(8)?;
        let (b, right, y) = other.jacobi_square_term(8)?;
        if a == b || !(0..2).all(|i| Self::internal_structural_eq(&left[i], &right[i])) {
            return None;
        }
        let (a, b, x, y) = if a < b { (a, b, x, y) } else { (b, a, y, x) };
        let c = left[1].retained_log_argument(&mut 64)
            .unwrap_or_else(|| left[1].clone().prescaled_exp());
        let result = match (a, b) {
            (0, 1) if equal(&x, &y) => y,
            (0, 1) if equal(&x, &y.clone().multiply(c.clone())) => {
                y.multiply(left[0].clone().jacobi_component(left[1].clone(), 2).square())
            }
            (0, 2) if equal(&x, &y.clone().multiply(Self::one().add(c.clone().negate()))) => y,
            (1, 2) if equal(&x, &y.clone().multiply(c.clone().add(Self::one().negate()))) => y.multiply(c),
            _ => return None,
        };
        crate::trace_dispatch!("computable", "jacobi", "retained-square-relation");
        let mut result = result;
        result.signal = self.signal.clone().or_else(|| other.signal.clone()).or(result.signal);
        Some(result)
    }
}
