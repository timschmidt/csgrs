/// One admitted summand kept as a rational scale times bounded polynomial factors.
/// Retaining this factorization lets recovery prove C*0=0 without expanding C.
/// Factors use one recognition attempt's opaque-atom numbering and domain evidence;
/// quotient admission checks encountered inverse guards but assumes native opaque
/// leaves are defined. The carrier does not validate that precondition itself.
struct RingProduct {
    /// Exact accumulated constant multiplier, including folded signs and dyadic shifts.
    /// Zero does not permit skipping the domain checks on remaining factors.
    scale: Rational,
    /// Nonconstant, individually admitted factors in collection order, with repeats.
    /// Common-factor removal may reorder them; they are not a canonical factorization.
    factors: Vec<RingPolynomial>,
    /// Sum of maximum monomial degrees accumulated during factor collection.
    /// Ordinary recovery uses this admission bound; late recovery limits factor count
    /// instead. It can advance on failed admission and is stale after factor removal.
    degree: usize,
}

/// Initialize a factorized summand before collecting its admitted operands.
impl RingProduct {
    /// Create the multiplicative identity: rational scale one and no factors.
    /// The initial degree tally is zero. This constructor does not debit work, poll
    /// cancellation, allocate factor elements or establish any operand domain.
    fn new() -> Self {
        Self {
            scale: Rational::one(),
            factors: Vec::new(),
            degree: 0,
        }
    }
}

/// Recover exact zero product sums using the existing attempt's proofs and budgets.
impl<'a> RingRecognition<'a> {
    /// Multiply the accumulated rational scale, charging before exact arithmetic.
    /// Replace it only if the product fits the stored coefficient cap. Failure leaves
    /// the old scale intact but may spend work; even zero scale does not exempt later
    /// factors from domain admission. Temporary arithmetic can exceed the stored cap.
    fn scale_product(&mut self, product: &mut RingProduct, scale: &Rational) -> Option<()> {
        self.charge()?;
        let scale = &product.scale * scale;
        if !Self::coefficient_fits(&scale) {
            return None;
        }
        product.scale = scale;
        Some(())
    }

    /// Peel product wrappers into a rational scale and admitted polynomial factors.
    /// Require positive depth. Multiply, Square, Negate and Offset each spend a work
    /// charge and node visit, even if ordinary expansion visited them already. Traverse
    /// both children left-to-right, and visit a squared child twice. Negation/offset
    /// update the scale before visiting the child; excessive dyadic shifts decline
    /// before materialization. Other recipes use the local quotient expander: a quotient
    /// with a denominator must project to a rational, while a polynomial remains a factor.
    ///
    /// Zero and rational constants fold into the scale, but remaining operands are
    /// still checked. Charge every term of a nonconstant polynomial to find its degree.
    /// Ordinary mode bounds the sum of these degrees by the active degree limit. The
    /// late EXTENDED_RECOVERY mode instead permits at most EXTENDED_DEGREE factors
    /// (twelve), each individually admitted; it does not raise polynomial or work caps.
    /// The caller supplies a valid partial product in the same attempt's atom table.
    /// On None, scale, degree, factors and local proof state may already be changed;
    /// discard the partial result. Opaque leaves retain quotient admission's defined-real
    /// precondition; this traversal is not a general native-function domain validator.
    fn collect_product_factors<const EXTENDED_RECOVERY: bool>(
        &mut self,
        value: &'a Computable,
        depth: usize,
        product: &mut RingProduct,
    ) -> Option<()> {
        if depth == 0 {
            return None;
        }
        let form = &value.internal.approximation;
        if matches!(
            form,
            Approximation::Multiply(..)
                | Approximation::Square(..)
                | Approximation::Negate(..)
                | Approximation::Offset(..)
        ) {
            // Peeling a product still consumes a visit, even if an earlier
            // failed expansion already visited this node. No fresh budget.
            self.charge()?;
            self.nodes_left = self.nodes_left.checked_sub(1)?;
            return match form {
                Approximation::Multiply(a, b) => {
                    self.collect_product_factors::<EXTENDED_RECOVERY>(a, depth - 1, product)?;
                    self.collect_product_factors::<EXTENDED_RECOVERY>(b, depth - 1, product)
                }
                Approximation::Square(child) => {
                    self.collect_product_factors::<EXTENDED_RECOVERY>(child, depth - 1, product)?;
                    self.collect_product_factors::<EXTENDED_RECOVERY>(child, depth - 1, product)
                }
                Approximation::Negate(child) => {
                    self.scale_product(product, &Rational::from(-1))?;
                    self.collect_product_factors::<EXTENDED_RECOVERY>(child, depth - 1, product)
                }
                Approximation::Offset(child, shift) => {
                    if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {
                        return None;
                    }
                    self.scale_product(product, &Computable::power_of_two_rational(*shift))?;
                    self.collect_product_factors::<EXTENDED_RECOVERY>(child, depth - 1, product)
                }
                _ => unreachable!("only product wrappers are peeled"),
            };
        }
        let quotient = self.expand_quotient(value, depth)?;
        let polynomial = if quotient.denominator.is_some() {
            Self::constant(self.project_quotient(&quotient)?)?
        } else {
            quotient.numerator
        };
        if polynomial.terms.is_empty() {
            // Continue checking the remaining factors, including inverses.
            return self.scale_product(product, &Rational::zero());
        }
        if let [(term, coefficient)] = polynomial.terms.as_slice()
            && term.is_empty()
        {
            return self.scale_product(product, coefficient);
        }
        let mut degree = 0;
        for (term, _) in &polynomial.terms {
            self.charge()?;
            degree = degree.max(term.len());
        }
        product.degree += degree;
        if EXTENDED_RECOVERY {
            // Used only by the final retry, after the original extended
            // expansion and recoveries declined. Bound retained factors,
            // allowing common factors to cancel before their degrees add.
            // Every factor and any expanded residual keep the usual limits.
            if product.factors.len() == Self::EXTENDED_DEGREE {
                return None;
            }
        } else if product.degree > self.degree_limit() {
            return None;
        }
        // Both modes retain at most twelve bounded factor polynomials; the
        // ordinary mode also bounds their total degree as before.
        product.factors.push(polynomial);
        Some(())
    }

    /// Multiply a retained residual's factors in their current order, starting at its scale.
    /// Every factor must already have been admitted with the same atom IDs and strategy.
    /// The stored degree tally is unused here: each actual multiplication enforces the
    /// ordinary active caps and relation rules. No factors means only the constant
    /// coefficient guard runs; there is no unconditional entry charge or stop poll.
    /// None reports a declined intermediate expansion, not proof of a nonzero residual.
    fn expand_product_residual(&mut self, product: RingProduct) -> Option<RingPolynomial> {
        let mut result = Self::constant(product.scale)?;
        for factor in product.factors {
            result = self.multiply(&result, &factor)?;
        }
        Some(result)
    }

    /// Collect a sum of scaled products without distributing products of nonconstant sums.
    /// Require positive depth and an admitted rational incoming scale. Peel Add left
    /// before right, Negate and Offset, charging a node and work for each. A Multiply
    /// is treated as scalar distribution only when a child is directly One, Int or Ratio
    /// (prefer the left child); expand and project that scalar before descending.
    /// Other representations go through factor collection and append one product,
    /// provided fewer than MAX_TERMS products (64) are already retained.
    ///
    /// A zero scale still requires child admission. Already appended products and spent
    /// budgets survive a declined later branch; the caller discards partial collections.
    /// This bounds retained summands and factors, not total temporary bytes or all
    /// coefficient arithmetic. It does not create a Cartesian product of sum factors.
    fn collect_sum_products<const EXTENDED_RECOVERY: bool>(
        &mut self,
        value: &'a Computable,
        depth: usize,
        scale: Rational,
        products: &mut Vec<RingProduct>,
    ) -> Option<()> {
        if depth == 0 {
            return None;
        }
        let mut product = RingProduct::new();
        product.scale = scale;
        let form = &value.internal.approximation;
        let coefficient = if let Approximation::Multiply(a, b) = form {
            let constant = |value: &Computable| {
                matches!(
                    value.internal.approximation,
                    Approximation::One | Approximation::Int(..) | Approximation::Ratio(..)
                )
            };
            if constant(a) {
                Some((a, b))
            } else if constant(b) {
                Some((b, a))
            } else {
                None
            }
        } else {
            None
        };
        if coefficient.is_some()
            || matches!(
                form,
                Approximation::Add(..) | Approximation::Negate(..) | Approximation::Offset(..)
            )
        {
            self.charge()?;
            self.nodes_left = self.nodes_left.checked_sub(1)?;
            match form {
                Approximation::Add(a, b) => {
                    self.collect_sum_products::<EXTENDED_RECOVERY>(
                        a,
                        depth - 1,
                        product.scale.clone(),
                        products,
                    )?;
                    return self.collect_sum_products::<EXTENDED_RECOVERY>(
                        b,
                        depth - 1,
                        product.scale,
                        products,
                    );
                }
                Approximation::Negate(child) => {
                    self.scale_product(&mut product, &Rational::from(-1))?;
                    return self.collect_sum_products::<EXTENDED_RECOVERY>(
                        child,
                        depth - 1,
                        product.scale,
                        products,
                    );
                }
                Approximation::Offset(child, shift) => {
                    if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {
                        return None;
                    }
                    self.scale_product(&mut product, &Computable::power_of_two_rational(*shift))?;
                    return self.collect_sum_products::<EXTENDED_RECOVERY>(
                        child,
                        depth - 1,
                        product.scale,
                        products,
                    );
                }
                _ => {
                    let (coefficient, child) = coefficient?;
                    let coefficient = self.expand_quotient(coefficient, depth - 1)?;
                    let scale = self.project_quotient(&coefficient)?;
                    self.scale_product(&mut product, &scale)?;
                    return self.collect_sum_products::<EXTENDED_RECOVERY>(
                        child,
                        depth - 1,
                        product.scale,
                        products,
                    );
                }
            }
        }
        if products.len() == Self::MAX_TERMS {
            return None;
        }
        // Only addition and rational scaling are distributed. Products of
        // sums retain their factors; this never builds a Cartesian product.
        self.collect_product_factors::<EXTENDED_RECOVERY>(value, depth, &mut product)?;
        products.push(product);
        Some(())
    }

    /// Try to prove a+b=0 by retaining common factors after ordinary expansion declined.
    /// First distribute additive/scalar wrappers into products and remove common factors.
    /// Only the late EXTENDED_RECOVERY retry, after that route declines, also tries
    /// keeping each whole summand's Add factors intact; this can expose a common C in
    /// C*U-C. Both routes share remaining work, node visits, atom IDs and child projections.
    /// No allowance is reset and failed first-route work is not refunded.
    ///
    /// All retained operands must pass quotient admission, including inverse guards;
    /// opaque native values still have its defined-real precondition. Success is an
    /// exact zero quotient and publishes no root fact. None combines no common factor,
    /// unproved residual/domain, local limits and cancellation, not proof of inequality.
    fn recover_factored_zero<const EXTENDED_RECOVERY: bool>(
        &mut self,
        a: &'a Computable,
        b: &'a Computable,
        depth: usize,
    ) -> Option<RingQuotient> {
        let distributed = (|| {
            let mut products = Vec::new();
            // Do not omit a common factor's domain merely because it will cancel.
            self.collect_sum_products::<EXTENDED_RECOVERY>(
                a,
                depth,
                Rational::one(),
                &mut products,
            )?;
            self.collect_sum_products::<EXTENDED_RECOVERY>(
                b,
                depth,
                Rational::one(),
                &mut products,
            )?;
            self.cancel_product_sum::<EXTENDED_RECOVERY>(products)
        })();
        distributed.or_else(|| {
            if !EXTENDED_RECOVERY {
                return None;
            }
            // Distribution can hide a shared sum in C*U-C. Try preserving
            // each summand's Add factors, using the same remaining budget.
            // Both strategies require a common admitted polynomial factor.
            let mut left = RingProduct::new();
            self.collect_product_factors::<true>(a, depth, &mut left)?;
            let mut right = RingProduct::new();
            self.collect_product_factors::<true>(b, depth, &mut right)?;
            self.cancel_product_sum::<true>(vec![left, right])
        })
    }

    /// Prove a sum zero by removing formally matching common polynomial factors.
    /// Require admitted products sharing this attempt's atom table. For each factor of
    /// the first product, find one equal stored polynomial in every other product;
    /// remove matches with swap_remove, respecting multiplicity but changing residual
    /// factor order. Equality checks charge work. The original degree tallies are not
    /// updated and are not used when expanding the residual. Empty input or no common
    /// factor declines, even if the sum could be proved zero another way.
    ///
    /// Expand and add all scaled residual products under the active polynomial limits.
    /// Only EXTENDED_RECOVERY may then reduce a nonempty residual with already registered
    /// unit-circle relations. Return zero exactly when the remaining polynomial is empty.
    /// The proof is C*0=0 and needs no nonzero assumption on C; it cannot infer a value
    /// from a residual not proved zero. All factors' domains must be admitted even when
    /// removed. This helper performs no final stop poll or fact publication; its query
    /// caller supplies that completion boundary. None is an unsuccessful proof attempt.
    fn cancel_product_sum<const EXTENDED_RECOVERY: bool>(
        &mut self,
        mut products: Vec<RingProduct>,
    ) -> Option<RingQuotient> {
        let (first, others) = products.split_first_mut()?;
        let mut common = false;
        let mut index = 0;
        while index < first.factors.len() {
            let mut matches = Vec::with_capacity(others.len());
            for product in others.iter() {
                let mut matched = None;
                for (other, factor) in product.factors.iter().enumerate() {
                    if self.same_polynomial(&first.factors[index], factor)? {
                        matched = Some(other);
                        break;
                    }
                }
                if let Some(other) = matched {
                    matches.push(other);
                } else {
                    break;
                }
            }
            if matches.len() == others.len() {
                first.factors.swap_remove(index);
                for (product, other) in others.iter_mut().zip(matches) {
                    product.factors.swap_remove(other);
                }
                common = true;
            } else {
                index += 1;
            }
        }
        if !common {
            return None;
        }
        let mut residual = RingPolynomial::default();
        for product in products {
            let term = self.expand_product_residual(product)?;
            residual = self.add(residual, term)?;
        }
        // The final retry may prove the smaller residual zero using the
        // already registered unit-circle identities. It uses only the work
        // remaining from expansion and factoring, with no new arithmetic cap.
        if EXTENDED_RECOVERY && !residual.terms.is_empty() {
            residual = self.reduce_unit_circle_pairs(residual)?;
        }
        // C * 0 = 0 needs no nonzero assumption about C. A residual not proved
        // zero supplies no proof of the original large product and is declined.
        residual
            .terms
            .is_empty()
            .then(|| RingQuotient::polynomial(residual))
    }
}
