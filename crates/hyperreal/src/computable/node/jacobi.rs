impl Computable {
    /// Recognize multiplication of two structurally matching Jacobi component nodes as a square.
    /// Require an sn/cn/dn tag on self and the existing structural recognizer; this
    /// checks the retained amplitude and log-complement shape, not numerical equality.
    /// It does not validate operand domains or reconcile cancellation handles.
    fn jacobi_square_product(&self, other: &Self) -> bool {
        matches!(
            self.internal.approximation,
            Approximation::JacobiSn(_) | Approximation::JacobiCn(_) | Approximation::JacobiDn(_)
        ) && Self::internal_structural_eq(self, other)
    }

    /// Construct one retained Jacobi component from this amplitude phi and log_complement = ln(c).
    /// Kinds 0, 1 and 2 denote sin(phi), cos(phi), and the positive root of
    /// `cos(phi)^2 + c*sin(phi)^2`, respectively, where c=exp(log_complement)>0
    /// and the elliptic parameter is m=1-c. The amplitude is already supplied;
    /// this constructor does not solve the inverse elliptic integral for a Jacobi u.
    /// Defined real payloads admit arbitrary finite amplitude and log-complement,
    /// but these unchecked graph inputs are not domain-validated here.
    ///
    /// Keep both child wrappers so components can share the same amplitude node/cache.
    /// The outer signal prefers the amplitude wrapper, then the log-complement wrapper.
    /// For dn with a proved zero log-complement, return one with that chosen signal;
    /// this shortcut does not inspect discarded descendant control identities.
    /// Other dn nodes retain positive sign, while sn/cn signs start unresolved.
    /// A kind outside 0..=2 panics as a caller-invariant violation.
    pub(crate) fn jacobi_component(self, log_complement: Self, kind: usize) -> Self {
        let signal = self
            .signal
            .clone()
            .or_else(|| log_complement.signal.clone());
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
                if kind == 2 {
                    ExactSignCache::Valid(Sign::Plus)
                } else {
                    ExactSignCache::Invalid
                },
            )),
            signal,
        }
    }

    /// Extract (component kind, borrowed amplitude/log-complement pair, exact scale) from a weighted component square.
    /// Accept a direct Square of sn/cn/dn, then wrappers for negation, binary offsets
    /// and multiplication. On a product, try the left spine first and try the right
    /// only if the left fails; keep the other factor in the recovered scale.
    /// The u8 budget bounds path depth, not total visits across both branches,
    /// structural comparison, coefficient growth or constructor work. Zero budget or
    /// unrecognized syntax returns None; no general polynomial search is performed.
    fn jacobi_square_term(&self, budget: u8) -> Option<(usize, &[Computable; 2], Self)> {
        if budget == 0 {
            return None;
        }
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

    /// Apply four retained square relations to two weighted Jacobi components sharing both argument shapes.
    /// With c=exp(log_complement), use `sn²+cn²=1`, `c*sn²+cn²=dn²`,
    /// `(1-c)*sn²+dn²=1`, or `(c-1)*cn²+dn²=c`, multiplied by the
    /// common proved scale. Extract each square with depth eight and require distinct
    /// component kinds and structurally matching amplitude and log-complement.
    ///
    /// Recover c with 64 log-recognition visits, falling back to an exact retained
    /// exponential, before testing the coefficient guards in their written order.
    /// Coefficient equality uses structural recognition followed by a bounded ring
    /// zero proof; a decline leaves the caller's original sum available. Success
    /// emits the retained-square-relation dispatch event. The result signal prefers
    /// self, then other, then the reconstructed result; there is no exhaustive
    /// reconciliation of discarded descendant controls in this helper.
    /// These optional recognition limits are not a total-work or cancellation bound.
    fn jacobi_square_sum(&self, other: &Self) -> Option<Self> {
        /// Prove coefficient equality by structural recognition or an exact zero projection of their difference.
        /// The fallback builds a temporary graph with left-first signal selection and uses
        /// the bounded ring recognizer. False includes unproved equality and guard/abort
        /// failure, rather than proving unequal coefficients; no numeric approximation is requested.
        fn equal(left: &Computable, right: &Computable) -> bool {
            Computable::internal_structural_eq(left, right)
                || Computable {
                    internal: Arc::new(Node::new(
                        Approximation::Add(left.clone(), right.clone().negate()),
                        BoundCache::Invalid,
                        ExactSignCache::Invalid,
                    )),
                    signal: left.signal.clone().or_else(|| right.signal.clone()),
                }
                .bounded_ring_rational()
                .is_some_and(|value| value.is_zero())
        }
        let (a, left, x) = self.jacobi_square_term(8)?;
        let (b, right, y) = other.jacobi_square_term(8)?;
        if a == b || !(0..2).all(|i| Self::internal_structural_eq(&left[i], &right[i])) {
            return None;
        }
        let (a, b, x, y) = if a < b { (a, b, x, y) } else { (b, a, y, x) };
        let c = left[1]
            .retained_log_argument(&mut 64)
            .unwrap_or_else(|| left[1].clone().prescaled_exp());
        let result = match (a, b) {
            (0, 1) if equal(&x, &y) => y,
            (0, 1) if equal(&x, &y.clone().multiply(c.clone())) => y.multiply(
                left[0]
                    .clone()
                    .jacobi_component(left[1].clone(), 2)
                    .square(),
            ),
            (0, 2) if equal(&x, &y.clone().multiply(Self::one().add(c.clone().negate()))) => y,
            (1, 2) if equal(&x, &y.clone().multiply(c.clone().add(Self::one().negate()))) => {
                y.multiply(c)
            }
            _ => return None,
        };
        crate::trace_dispatch!("computable", "jacobi", "retained-square-relation");
        let mut result = result;
        result.signal = self
            .signal
            .clone()
            .or_else(|| other.signal.clone())
            .or(result.signal);
        Some(result)
    }
}
