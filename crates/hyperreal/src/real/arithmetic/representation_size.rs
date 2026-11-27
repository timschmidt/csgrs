/// Sum canonical unsigned numerator and positive denominator bit lengths, ignoring sign/storage overhead.
/// The Rational getters may reduce an internal lazy fraction and populate its cache.
fn stored_rational_bits(value: &Rational) -> u64 {
    value.numerator().bits() + value.denominator().bits()
}

/// Account for rational parameters of a named basis without counting its materialized graph.
impl Class {
    /// Maximum canonical bit sum among this class's rational parameters, not their total.
    /// Parameter-free classes and opaque tags return zero. Small integer tags,
    /// allocation overhead and computable payloads are excluded; getters may canonicalize.
    fn stored_rational_bits(&self) -> u64 {
        match self {
            One | Pi | PiPow(_) | PiInv | Irrational => 0,
            PiExp(r) | PiInvExp(r) | PiSqrt(r) | Sqrt(r) | Exp(r) | Ln(r) | Log10(r) | Log2(r)
            | Pow10(r) | Pow2(r) | SinPi(r) | TanPi(r) => stored_rational_bits(r),
            ConstProduct(product) => stored_rational_bits(&product.exp_power),
            ConstOffset(offset) => {
                stored_rational_bits(&offset.exp_power).max(stored_rational_bits(&offset.offset))
            }
            ConstProductSqrt(product) => stored_rational_bits(&product.exp_power)
                .max(stored_rational_bits(&product.radicand)),
            LnAffine(affine) => {
                stored_rational_bits(&affine.offset).max(stored_rational_bits(&affine.base))
            }
            LnProduct(product) => {
                stored_rational_bits(&product.left).max(stored_rational_bits(&product.right))
            }
        }
    }
}

/// Observe selected representation dimensions without numerical refinement.
impl Real {
    /// Report the size of the exact representation this value stores.
    ///
    /// An exact rational has no expression nodes, and a recognized closed
    /// form such as `sqrt(2)` or `sin(pi/16)` counts as one. Any other value
    /// counts the distinct nodes of its expression graph, visiting at most
    /// `node_limit` of them; [`RepresentationSize::complete`] reports whether
    /// the limit stopped the traversal. The rational scale and every stored
    /// rational parameter or leaf contribute to `max_rational_bits`.
    ///
    /// Named classes count as one even when node_limit is zero; that limit
    /// applies only to opaque graph traversal. Materialized named-certificate
    /// payloads, approximation caches, small exponent tags and allocation
    /// overhead are excluded. This is not retained heap-byte accounting.
    ///
    /// No numeric approximation or refinement is requested, but rational
    /// getters expose canonical parts: they may reduce lazy internal ratios
    /// and populate canonicalization caches. Thus the bit metric need not be
    /// the raw stored width. Equal values built differently can report different
    /// graph sizes. Traversal bookkeeping is linear in visited graph nodes;
    /// rational reduction and coefficient-size costs are additional. Neither
    /// node_limit nor this result is a total-work or memory bound.
    pub fn representation_size(&self, node_limit: usize) -> RepresentationSize {
        let scale_bits = stored_rational_bits(&self.rational);
        let mut size = match (&self.class, &self.computable) {
            (One, _) => RepresentationSize {
                nodes: 0,
                depth: 0,
                max_rational_bits: 0,
                complete: true,
            },
            (Irrational, Some(computable)) => computable.representation_size(node_limit),
            // A materialized certificate of a closed form is a cache of that
            // form, so it does not count as stored structure.
            (class, _) => RepresentationSize {
                nodes: 1,
                depth: 1,
                max_rational_bits: class.stored_rational_bits(),
                complete: true,
            },
        };
        size.max_rational_bits = size.max_rational_bits.max(scale_bits);
        size
    }
}

#[cfg(test)]
/// Regression witnesses for selected representation metrics and truncated graph traversal.
mod representation_size_tests {
    use super::*;

    #[test]
    /// Check exact metric fields for -255/7 and scaled sqrt(2), including the maximum rather than sum of parameter sizes.
    fn rationals_and_closed_forms_have_bounded_sizes() {
        let rational = Real::new(Rational::fraction(-255, 7).unwrap());
        assert_eq!(
            rational.representation_size(16),
            RepresentationSize {
                nodes: 0,
                depth: 0,
                max_rational_bits: 8 + 3,
                complete: true,
            }
        );
        let root = Real::from(2_u8).sqrt().unwrap() * Real::from(3_u8);
        let size = root.representation_size(16);
        assert_eq!((size.nodes, size.depth, size.complete), (1, 1, true));
        assert_eq!(size.max_rational_bits, 2 + 1);
    }

    #[test]
    /// Build forty squaring/addition stages with reused operands, check bounded node growth and a deep path,
    /// then require a five-node traversal to report incompleteness. This tests graph sharing, not byte usage.
    fn shared_operands_count_once_and_depth_follows_the_longest_chain() {
        let atom = Real::pi() + Real::from(1_u8);
        let mut chain = atom.clone();
        for _ in 0..40 {
            // Each step reuses the previous value twice; a tree count would
            // double every step, while the graph grows by a bounded amount.
            chain = &chain * &chain + &atom;
        }
        let size = chain.representation_size(10_000);
        assert!(size.complete);
        assert!(size.nodes < 40 * 8, "{size:?}");
        assert!(size.depth >= 40, "{size:?}");
        let truncated = chain.representation_size(5);
        assert!(!truncated.complete);
        assert_eq!(truncated.nodes, 5);
        assert!(truncated.depth <= size.depth);
    }

    #[test]
    /// Exercise a 4,000-term radical sum and require complete traversal with depth at least 1,000.
    /// Successful traversal is a deep-chain regression, not a measured stack bound for arbitrary inputs.
    fn deep_chains_do_not_use_the_call_stack() {
        let mut chain = Real::pi();
        for index in 1..=4_000_u32 {
            chain += Real::from(index).sqrt().unwrap();
        }
        let size = chain.representation_size(usize::MAX);
        assert!(size.complete);
        assert!(size.depth >= 1_000, "{size:?}");
    }
}
