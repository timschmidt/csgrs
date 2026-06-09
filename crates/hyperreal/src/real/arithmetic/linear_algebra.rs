impl Real {

    /// Return the two-lane dot product of borrowed reals.
    ///
    /// Sibling of [`Self::dot3_refs`] / [`Self::dot4_refs`] for the
    /// two-component case (2D coordinates, complex products, planar dot
    /// products, etc.). Same exact-rational shared-denominator fast path;
    /// same symbolic fallback policy.
    pub fn dot2_refs(left: [&Real; 2], right: [&Real; 2]) -> Real {
        if let (Some(l0), Some(l1), Some(r0), Some(r1)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "dot2-exact-rational-shared-denom");
            return Real::new(Rational::dot_products([l0, l1], [r0, r1]));
        }

        Self::dot2_refs_fallback(left, right)
    }

    /// Return a two-lane dot product whose lanes were already classified active.
    ///
    /// See [`Self::active_dot3_refs`].
    pub fn active_dot2_refs(left: [&Real; 2], right: [&Real; 2]) -> Real {
        if let (Some(l0), Some(l1), Some(r0), Some(r1)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "active-dot2-exact-rational");
            return Real::new(Rational::dot_products([l0, l1], [r0, r1]));
        }

        crate::trace_dispatch!("real", "dot_product", "active-dot2-real-tree");
        Self::sum_dot2_terms(
            Some(Self::dot_product_active_term(left[0], right[0])),
            Some(Self::dot_product_active_term(left[1], right[1])),
        )
    }

    #[inline(never)]
    fn dot2_refs_fallback(left: [&Real; 2], right: [&Real; 2]) -> Real {
        // See `dot3_refs_fallback` for the code-layout rationale.
        if Self::dot_product_has_structural_term(left[0], right[0])
            || Self::dot_product_has_structural_term(left[1], right[1])
        {
            crate::trace_dispatch!("real", "dot_product", "dot2-structural-real-tree");
            return Self::sum_dot2_terms(
                Self::dot_product_term(left[0], right[0]),
                Self::dot_product_term(left[1], right[1]),
            );
        }

        if left[0].rational.sign() == Sign::NoSign
            || right[0].rational.sign() == Sign::NoSign
            || left[1].rational.sign() == Sign::NoSign
            || right[1].rational.sign() == Sign::NoSign
        {
            let p0 = Self::dot_product_term(left[0], right[0]);
            let p1 = Self::dot_product_term(left[1], right[1]);
            let active_terms = usize::from(p0.is_some()) + usize::from(p1.is_some());

            match active_terms {
                0 => {
                    crate::trace_dispatch!("real", "dot_product", "dot2-all-zero-real-tree");
                    return Real::zero();
                }
                1 => {
                    crate::trace_dispatch!("real", "dot_product", "dot2-generic-real-tree-sparse");
                    return Self::sum_dot2_terms(p0, p1);
                }
                _ => {
                    crate::trace_dispatch!("real", "dot_product", "dot2-generic-real-tree");
                    return Self::sum_dot2_terms(p0, p1);
                }
            }
        }

        let p0 = left[0] * right[0];
        let p1 = left[1] * right[1];
        crate::trace_dispatch!("real", "dot_product", "dot2-generic-real-tree");
        &p0 + &p1
    }

    /// Return the three-lane dot product of borrowed reals.
    ///
    /// Exact-rational lanes are accumulated with one shared denominator and a
    /// single final canonicalization. This is the vector/matrix analogue of the
    /// fraction-delaying exact linear-algebra algorithms discussed around
    /// Bareiss elimination and common factors in
    /// <https://link.springer.com/article/10.1007/s11786-020-00495-9>. The
    /// fallback intentionally preserves the previous product-then-pairwise-add
    /// tree for non-rational symbolic values; sharing that path with the
    /// rational fast path regressed expression-heavy scalar rows. Mixed
    /// symbolic/rational lanes use a narrower structural fallback: exact
    /// rational scales are applied directly and exact-zero terms are omitted,
    /// but dense symbolic lanes still take the original tree. 2026-05
    /// scalar_micro, 200 samples/8s: mixed dot3/dot4 moved from ~848 ns/~1.006
    /// us to ~697 ns/~753 ns; dense dot3/dot4 moved from ~4.01 us/~7.72 us
    /// to ~3.95 us/~7.11 us.
    pub fn dot3_refs(left: [&Real; 3], right: [&Real; 3]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(r0), Some(r1), Some(r2)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "dot3-exact-rational-shared-denom");
            return Real::new(Rational::dot_products([l0, l1, l2], [r0, r1, r2]));
        }

        Self::dot3_refs_fallback(left, right)
    }

    /// Return a three-lane dot product whose lanes were already classified active.
    ///
    /// This is for callers that already paid for zero-lane facts. It preserves
    /// the shared-denominator exact-rational reducer while avoiding fresh
    /// scalar zero probes in fixed-size matrix lanes.
    pub fn active_dot3_refs(left: [&Real; 3], right: [&Real; 3]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(r0), Some(r1), Some(r2)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "active-dot3-exact-rational");
            return Real::new(Rational::dot_products([l0, l1, l2], [r0, r1, r2]));
        }

        crate::trace_dispatch!("real", "dot_product", "active-dot3-real-tree");
        Self::sum_dot3_terms(
            Some(Self::dot_product_active_term(left[0], right[0])),
            Some(Self::dot_product_active_term(left[1], right[1])),
            Some(Self::dot_product_active_term(left[2], right[2])),
        )
    }

    #[inline(never)]
    fn dot3_refs_fallback(left: [&Real; 3], right: [&Real; 3]) -> Real {
        // Keep the symbolic fallback out of line so the matrix hot path that
        // exits through the exact-rational branch above remains small enough
        // for LLVM to inline consistently. An inline prototype improved mixed
        // symbolic dots but regressed hyperlattice hyperreal mat4 borrowed
        // multiply by ~2.6% through code layout alone.
        // Keep zero-sparse symbolic rows fast by skipping exact-zero lanes
        // before building intermediate symbolic terms.
        if Self::dot_product_has_structural_term(left[0], right[0])
            || Self::dot_product_has_structural_term(left[1], right[1])
            || Self::dot_product_has_structural_term(left[2], right[2])
        {
            crate::trace_dispatch!("real", "dot_product", "dot3-structural-real-tree");
            return Self::sum_dot3_terms(
                Self::dot_product_term(left[0], right[0]),
                Self::dot_product_term(left[1], right[1]),
                Self::dot_product_term(left[2], right[2]),
            );
        }

        if left[0].rational.sign() == Sign::NoSign
            || right[0].rational.sign() == Sign::NoSign
            || left[1].rational.sign() == Sign::NoSign
            || right[1].rational.sign() == Sign::NoSign
            || left[2].rational.sign() == Sign::NoSign
            || right[2].rational.sign() == Sign::NoSign
        {
            let p0 = Self::dot_product_term(left[0], right[0]);
            let p1 = Self::dot_product_term(left[1], right[1]);
            let p2 = Self::dot_product_term(left[2], right[2]);
            let active_terms =
                usize::from(p0.is_some()) + usize::from(p1.is_some()) + usize::from(p2.is_some());

            match active_terms {
                0 => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-all-zero-real-tree");
                    return Real::zero();
                }
                1..=2 => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree-sparse");
                    return Self::sum_dot3_terms(p0, p1, p2);
                }
                _ => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree");
                    return Self::sum_dot3_terms(p0, p1, p2);
                }
            }
        }

        let p0 = left[0] * right[0];
        let p1 = left[1] * right[1];
        let p2 = left[2] * right[2];
        crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree");
        let sum01 = &p0 + &p1;
        &sum01 + &p2
    }

    /// Return the four-lane dot product of borrowed reals.
    ///
    /// See [`Self::dot3_refs`] for the performance policy. Four-lane matrix
    /// multiplication gets the largest win from delaying rational
    /// canonicalization because each output cell otherwise builds four product
    /// rationals plus three partial-sum rationals.
    ///
    /// 2026-05 hyperlattice benchmarks: mat4 mul refs on hyperreal moved
    /// from roughly 10.46 us to 4.33 us after this path, and trace constructors
    /// for one borrowed mat4 multiply dropped from 448 rational Reals to 64.
    pub fn dot4_refs(left: [&Real; 4], right: [&Real; 4]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(l3), Some(r0), Some(r1), Some(r2), Some(r3)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            left[3].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
            right[3].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "dot4-exact-rational-shared-denom");
            return Real::new(Rational::dot_products([l0, l1, l2, l3], [r0, r1, r2, r3]));
        }

        Self::dot4_refs_fallback(left, right)
    }

    /// Return a four-lane dot product whose lanes were already classified active.
    ///
    /// See [`Self::active_dot3_refs`].
    pub fn active_dot4_refs(left: [&Real; 4], right: [&Real; 4]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(l3), Some(r0), Some(r1), Some(r2), Some(r3)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            left[3].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
            right[3].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "active-dot4-exact-rational");
            return Real::new(Rational::dot_products([l0, l1, l2, l3], [r0, r1, r2, r3]));
        }

        crate::trace_dispatch!("real", "dot_product", "active-dot4-real-tree");
        Self::sum_dot4_terms(
            Some(Self::dot_product_active_term(left[0], right[0])),
            Some(Self::dot_product_active_term(left[1], right[1])),
            Some(Self::dot_product_active_term(left[2], right[2])),
            Some(Self::dot_product_active_term(left[3], right[3])),
        )
    }

    /// Return the three-lane affine combination `c0 * x0 + c1 * x1 + c2 * x2`.
    ///
    /// The first increment keeps the representation boundary: these forms are
    /// currently delegates so existing transform callers can target a named
    /// constructor before stronger symbolic preservation is introduced.
    pub fn linear_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3]) -> Real {
        Self::dot3_refs(coeffs, values)
    }

    /// Return a three-lane linear combination whose lanes were already classified active.
    pub fn active_linear_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3]) -> Real {
        Self::active_dot3_refs(coeffs, values)
    }

    /// Return the four-lane affine combination `c0 * x0 + c1 * x1 + c2 * x2 + c3 * x3`.
    ///
    /// As with [`Self::linear_combination3_refs`], this is intentionally a
    /// thin constructor for the representation slotting work.
    pub fn linear_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4]) -> Real {
        Self::dot4_refs(coeffs, values)
    }

    /// Return a four-lane linear combination whose lanes were already classified active.
    pub fn active_linear_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4]) -> Real {
        Self::active_dot4_refs(coeffs, values)
    }

    /// Return the three-lane affine sum with an explicit offset.
    pub fn affine_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3], offset: &Real) -> Real {
        let zero0 = coeffs[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coeffs[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coeffs[2].definitely_zero() || values[2].definitely_zero();
        if zero0 && zero1 && zero2 {
            crate::trace_dispatch!("real", "affine_combination", "affine-combination3-all-zero");
            return offset.clone();
        }

        if offset.definitely_zero() {
            crate::trace_dispatch!(
                "real",
                "affine_combination",
                "affine-combination3-offset-zero"
            );
            return Self::masked_linear_combination3_refs(coeffs, values, [zero0, zero1, zero2]);
        }

        let linear = Self::masked_linear_combination3_refs(coeffs, values, [zero0, zero1, zero2]);
        crate::trace_dispatch!("real", "affine_combination", "affine-combination3");
        offset + linear
    }

    /// Return the four-lane affine sum with an explicit offset.
    pub fn affine_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4], offset: &Real) -> Real {
        let zero0 = coeffs[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coeffs[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coeffs[2].definitely_zero() || values[2].definitely_zero();
        let zero3 = coeffs[3].definitely_zero() || values[3].definitely_zero();
        if zero0 && zero1 && zero2 && zero3 {
            crate::trace_dispatch!("real", "affine_combination", "affine-combination4-all-zero");
            return offset.clone();
        }

        if offset.definitely_zero() {
            crate::trace_dispatch!(
                "real",
                "affine_combination",
                "affine-combination4-offset-zero"
            );
            return Self::masked_linear_combination4_refs(
                coeffs,
                values,
                [zero0, zero1, zero2, zero3],
            );
        }

        let linear =
            Self::masked_linear_combination4_refs(coeffs, values, [zero0, zero1, zero2, zero3]);
        crate::trace_dispatch!("real", "affine_combination", "affine-combination4");
        offset + linear
    }

    #[inline]
    fn masked_linear_combination3_refs(
        coeffs: [&Real; 3],
        values: [&Real; 3],
        zero: [bool; 3],
    ) -> Real {
        if !zero[0] && !zero[1] && !zero[2] {
            return Self::active_linear_combination3_refs(coeffs, values);
        }

        crate::trace_dispatch!(
            "real",
            "affine_combination",
            "active-linear-combination3-sparse"
        );
        Self::sum_dot3_terms(
            (!zero[0]).then(|| Self::dot_product_active_term(coeffs[0], values[0])),
            (!zero[1]).then(|| Self::dot_product_active_term(coeffs[1], values[1])),
            (!zero[2]).then(|| Self::dot_product_active_term(coeffs[2], values[2])),
        )
    }

    #[inline]
    fn masked_linear_combination4_refs(
        coeffs: [&Real; 4],
        values: [&Real; 4],
        zero: [bool; 4],
    ) -> Real {
        if !zero[0] && !zero[1] && !zero[2] && !zero[3] {
            return Self::active_linear_combination4_refs(coeffs, values);
        }

        crate::trace_dispatch!(
            "real",
            "affine_combination",
            "active-linear-combination4-sparse"
        );
        Self::sum_dot4_terms(
            (!zero[0]).then(|| Self::dot_product_active_term(coeffs[0], values[0])),
            (!zero[1]).then(|| Self::dot_product_active_term(coeffs[1], values[1])),
            (!zero[2]).then(|| Self::dot_product_active_term(coeffs[2], values[2])),
            (!zero[3]).then(|| Self::dot_product_active_term(coeffs[3], values[3])),
        )
    }

    #[inline(never)]
    fn dot4_refs_fallback(left: [&Real; 4], right: [&Real; 4]) -> Real {
        // See `dot3_refs_fallback` for the code-layout rationale.
        if Self::dot_product_has_structural_term(left[0], right[0])
            || Self::dot_product_has_structural_term(left[1], right[1])
            || Self::dot_product_has_structural_term(left[2], right[2])
            || Self::dot_product_has_structural_term(left[3], right[3])
        {
            crate::trace_dispatch!("real", "dot_product", "dot4-structural-real-tree");
            return Self::sum_dot4_terms(
                Self::dot_product_term(left[0], right[0]),
                Self::dot_product_term(left[1], right[1]),
                Self::dot_product_term(left[2], right[2]),
                Self::dot_product_term(left[3], right[3]),
            );
        }

        if left[0].rational.sign() == Sign::NoSign
            || right[0].rational.sign() == Sign::NoSign
            || left[1].rational.sign() == Sign::NoSign
            || right[1].rational.sign() == Sign::NoSign
            || left[2].rational.sign() == Sign::NoSign
            || right[2].rational.sign() == Sign::NoSign
            || left[3].rational.sign() == Sign::NoSign
            || right[3].rational.sign() == Sign::NoSign
        {
            let p0 = Self::dot_product_term(left[0], right[0]);
            let p1 = Self::dot_product_term(left[1], right[1]);
            let p2 = Self::dot_product_term(left[2], right[2]);
            let p3 = Self::dot_product_term(left[3], right[3]);
            let active_terms = usize::from(p0.is_some())
                + usize::from(p1.is_some())
                + usize::from(p2.is_some())
                + usize::from(p3.is_some());

            match active_terms {
                0 => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-all-zero-real-tree");
                    return Real::zero();
                }
                1..=3 => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree-sparse");
                    return Self::sum_dot4_terms(p0, p1, p2, p3);
                }
                _ => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree");
                    return Self::sum_dot4_terms(p0, p1, p2, p3);
                }
            }
        }
        let p0 = left[0] * right[0];
        let p1 = left[1] * right[1];
        let p2 = left[2] * right[2];
        let p3 = left[3] * right[3];
        let sum01 = &p0 + &p1;
        let sum23 = &p2 + &p3;
        crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree");
        &sum01 + &sum23
    }

    #[inline]
    fn dot_product_has_structural_term(left: &Real, right: &Real) -> bool {
        // Gate only on the symbolic class. A broader rational-sign precheck
        // also caught malformed zero-scaled symbolic terms, but the extra
        // field reads regressed the dense symbolic dot3 probe by about 4%.
        // Normal `Real` constructors canonicalize exact zero as `Class::One`,
        // so this still covers the practical zero-term shortcut.
        matches!(left.class, One) || matches!(right.class, One)
    }

    #[inline]
    fn dot_product_term(left: &Real, right: &Real) -> Option<Real> {
        if left.rational.sign() == Sign::NoSign || right.rational.sign() == Sign::NoSign {
            return None;
        }
        Some(Self::dot_product_active_term(left, right))
    }

    #[inline]
    fn dot_product_active_term(left: &Real, right: &Real) -> Real {
        if matches!(left.class, One) {
            return right.scaled_by_rational(&left.rational);
        }
        if matches!(right.class, One) {
            return left.scaled_by_rational(&right.rational);
        }
        left * right
    }

    #[inline]
    fn product_term<const FACTORS: usize>(factors: [&Real; FACTORS]) -> Option<Real> {
        let mut product = None::<Real>;
        for factor in factors {
            if factor.rational.sign() == Sign::NoSign {
                return None;
            }

            product = Some(match product.take() {
                None => factor.clone(),
                Some(product) if matches!(factor.class, One) => {
                    product.scaled_by_rational(&factor.rational)
                }
                Some(product) if matches!(product.class, One) => {
                    factor.scaled_by_rational(&product.rational)
                }
                Some(product) => &product * factor,
            });
        }

        product
    }

    #[inline]
    fn sum_dot2_terms(p0: Option<Real>, p1: Option<Real>) -> Real {
        match (p0, p1) {
            (None, None) => Real::zero(),
            (Some(p), None) | (None, Some(p)) => p,
            (Some(a), Some(b)) => &a + &b,
        }
    }

    #[inline]
    fn sum_dot3_terms(p0: Option<Real>, p1: Option<Real>, p2: Option<Real>) -> Real {
        match (p0, p1, p2) {
            (None, None, None) => Real::zero(),
            (Some(p), None, None) | (None, Some(p), None) | (None, None, Some(p)) => p,
            (Some(a), Some(b), None) | (Some(a), None, Some(b)) | (None, Some(a), Some(b)) => {
                &a + &b
            }
            (Some(p0), Some(p1), Some(p2)) => {
                let sum01 = &p0 + &p1;
                &sum01 + &p2
            }
        }
    }

    #[inline]
    fn sum_dot4_terms(
        p0: Option<Real>,
        p1: Option<Real>,
        p2: Option<Real>,
        p3: Option<Real>,
    ) -> Real {
        match (p0, p1, p2, p3) {
            (None, None, None, None) => Real::zero(),
            (Some(p0), Some(p1), Some(p2), Some(p3)) => {
                let sum01 = &p0 + &p1;
                let sum23 = &p2 + &p3;
                &sum01 + &sum23
            }
            (p0, p1, p2, p3) => Self::sum_dot_terms([p0, p1, p2, p3]),
        }
    }

    #[inline]
    fn sum_dot_terms<const N: usize>(terms: [Option<Real>; N]) -> Real {
        let mut total = None;
        for term in terms {
            let Some(term) = term else {
                continue;
            };
            total = Some(match total.take() {
                Some(total) => &total + &term,
                None => term,
            });
        }
        total.unwrap_or_else(Real::zero)
    }

}
