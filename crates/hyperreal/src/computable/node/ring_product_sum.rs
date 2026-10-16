// Keep common factors out of an opposite product sum after ordinary expansion
// has declined. All factors are still expanded and domain-checked individually.
struct RingProduct {
    scale: Rational,
    factors: Vec<RingPolynomial>,
    degree: usize,
}

impl RingProduct {
    fn new() -> Self {
        Self {
            scale: Rational::one(),
            factors: Vec::new(),
            degree: 0,
        }
    }
}

impl<'a> RingRecognition<'a> {
    fn scale_product(&mut self, product: &mut RingProduct, scale: &Rational) -> Option<()> {
        self.charge()?;
        let scale = &product.scale * scale;
        if !Self::coefficient_fits(&scale) {
            return None;
        }
        product.scale = scale;
        Some(())
    }

    fn collect_product_factors(
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
                    self.collect_product_factors(a, depth - 1, product)?;
                    self.collect_product_factors(b, depth - 1, product)
                }
                Approximation::Square(child) => {
                    self.collect_product_factors(child, depth - 1, product)?;
                    self.collect_product_factors(child, depth - 1, product)
                }
                Approximation::Negate(child) => {
                    self.scale_product(product, &Rational::from(-1))?;
                    self.collect_product_factors(child, depth - 1, product)
                }
                Approximation::Offset(child, shift) => {
                    if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {
                        return None;
                    }
                    self.scale_product(product, &Computable::power_of_two_rational(*shift))?;
                    self.collect_product_factors(child, depth - 1, product)
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
        if product.degree > Self::MAX_DEGREE {
            return None;
        }
        // Every retained nonconstant factor has degree at least one, so
        // each product retains at most MAX_DEGREE bounded polynomials.
        product.factors.push(polynomial);
        Some(())
    }

    fn expand_product_residual(&mut self, product: RingProduct) -> Option<RingPolynomial> {
        let mut result = Self::constant(product.scale)?;
        for factor in product.factors {
            result = self.multiply(&result, &factor)?;
        }
        Some(result)
    }

    fn collect_sum_products(
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
                    self.collect_sum_products(a, depth - 1, product.scale.clone(), products)?;
                    return self.collect_sum_products(b, depth - 1, product.scale, products);
                }
                Approximation::Negate(child) => {
                    self.scale_product(&mut product, &Rational::from(-1))?;
                    return self.collect_sum_products(child, depth - 1, product.scale, products);
                }
                Approximation::Offset(child, shift) => {
                    if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {
                        return None;
                    }
                    self.scale_product(&mut product, &Computable::power_of_two_rational(*shift))?;
                    return self.collect_sum_products(child, depth - 1, product.scale, products);
                }
                _ => {
                    let (coefficient, child) = coefficient?;
                    let coefficient = self.expand_quotient(coefficient, depth - 1)?;
                    let scale = self.project_quotient(&coefficient)?;
                    self.scale_product(&mut product, &scale)?;
                    return self.collect_sum_products(child, depth - 1, product.scale, products);
                }
            }
        }
        if products.len() == Self::MAX_TERMS {
            return None;
        }
        // Only addition and rational scaling are distributed. Products of
        // sums retain their factors; this never builds a Cartesian product.
        self.collect_product_factors(value, depth, &mut product)?;
        products.push(product);
        Some(())
    }

    fn recover_factored_zero(
        &mut self,
        a: &'a Computable,
        b: &'a Computable,
        depth: usize,
    ) -> Option<RingQuotient> {
        let mut products = Vec::new();
        // Do not omit a common factor's domain merely because it will cancel.
        self.collect_sum_products(a, depth, Rational::one(), &mut products)?;
        self.collect_sum_products(b, depth, Rational::one(), &mut products)?;
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
        // C * 0 = 0 needs no nonzero assumption about C. A nonzero residual
        // supplies no proof of the original large product and is declined.
        residual
            .terms
            .is_empty()
            .then(|| RingQuotient::polynomial(residual))
    }
}
