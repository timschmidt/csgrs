// Graph-growth diagnostics without numeric approximation. Rational payload
// access may canonicalize lazy ratios and retain the resulting cache; these
// metrics do not account for all stored bytes or the cost of that reduction.

/// Sum canonical rational-part bit lengths; getters may reduce/cache an internally unreduced value.
fn rational_storage_bits(value: &Rational) -> u64 {
    value.numerator().bits() + value.denominator().bits()
}

/// Enumerate graph edges and selected numeric payload widths for representation diagnostics.
impl Approximation {
    /// Visit the direct operand graphs of this node.
    ///
    /// The match is exhaustive so a new variant must state its operands.
    /// Visit only explicit Computable children, in the stored match order;
    /// scalar payloads, retained facts and caches are not graph edges here.
    fn for_each_operand<'a>(&'a self, mut visit: impl FnMut(&'a Computable)) {
        match self {
            Self::Int(_)
            | Self::One
            | Self::Constant(_)
            | Self::Ratio(_)
            | Self::PrescaledLnRational(_)
            | Self::BinaryScaledLnRational { .. }
            | Self::IntegralAtan(_)
            | Self::AtanRational(_)
            | Self::AsinRational(_)
            | Self::AcosPositiveRational(_)
            | Self::AcosNegativeRational(_)
            | Self::AsinhRational(_)
            | Self::AtanhRational(_)
            | Self::PrescaledCosRational(_)
            | Self::CosLargeRational(_)
            | Self::PrescaledCosHalfPiMinusRational(_)
            | Self::PrescaledSinRational(_)
            | Self::SinLargeRational(_)
            | Self::PrescaledSinHalfPiMinusRational(_)
            | Self::PrescaledCotHalfPiMinusRational(_)
            | Self::TanLargeRational(_)
            | Self::PrescaledTanRational(_) => {}
            Self::Inverse(child)
            | Self::Negate(child)
            | Self::Square(child)
            | Self::Offset(child, _)
            | Self::PrescaledExp(child)
            | Self::Expm1(child)
            | Self::Sqrt(child)
            | Self::PrescaledLn(child)
            | Self::PrescaledAtan(child)
            | Self::AtanDeferred(child)
            | Self::PrescaledAsin(child)
            | Self::AsinDeferred(child)
            | Self::AcosPositive(child)
            | Self::AcoshNearOne(child)
            | Self::AcoshDirect(child)
            | Self::AsinhNearZero(child)
            | Self::AsinhDirect(child)
            | Self::PrescaledAsinh(child)
            | Self::AtanhDirect(child)
            | Self::PrescaledAtanh(child)
            | Self::PrescaledCos(child)
            | Self::PrescaledSin(child)
            | Self::PrescaledTan(child)
            | Self::PrescaledCot(child)
            | Self::ErfSeries(child)
            | Self::Erfc(child)
            | Self::NormalSf(child)
            | Self::LogPnorm(child)
            | Self::LogNormalSf(child)
            | Self::LogDnorm(child)
            | Self::NthRoot(child, _)
            | Self::SincSmall(child)
            | Self::CoscSmall(child) => visit(child),
            Self::Add(left, right)
            | Self::Multiply(left, right)
            | Self::NormalInterval {
                lo: left,
                hi: right,
            }
            | Self::InverseIsometricLatitude {
                psi: left,
                log_axis_ratio: right,
            } => {
                visit(left);
                visit(right);
            }
            Self::LinearCombination3(combination) => {
                combination.coefficients.iter().for_each(visit);
            }
            Self::NormalQuantile(data) => visit(&data.p),
            Self::EllipticRf(children) | Self::EllipticRd(children) => {
                children.iter().for_each(visit);
            }
            Self::EllipticF(children)
            | Self::EllipticE(children)
            | Self::JacobiAmplitude(children)
            | Self::InverseEllipticE(children)
            | Self::JacobiSn(children)
            | Self::JacobiCn(children)
            | Self::JacobiDn(children) => children.iter().for_each(visit),
        }
    }

    /// Largest selected rational/integer payload bit metric in this node, excluding child graphs.
    /// Rational parts use canonical numerator-plus-denominator widths; integer
    /// and quantile-seed payloads use magnitude bits. Offset/root-order/exponent
    /// tags, other metadata and cached approximations are not counted. A default
    /// zero means no counted payload, not zero allocated storage.
    fn stored_rational_bits(&self) -> u64 {
        match self {
            Self::Int(value) | Self::IntegralAtan(value) => value.bits(),
            Self::Ratio(value)
            | Self::PrescaledLnRational(value)
            | Self::BinaryScaledLnRational {
                residual: value, ..
            }
            | Self::AtanRational(value)
            | Self::AsinRational(value)
            | Self::AcosPositiveRational(value)
            | Self::AcosNegativeRational(value)
            | Self::AsinhRational(value)
            | Self::AtanhRational(value)
            | Self::PrescaledCosRational(value)
            | Self::CosLargeRational(value)
            | Self::PrescaledCosHalfPiMinusRational(value)
            | Self::PrescaledSinRational(value)
            | Self::SinLargeRational(value)
            | Self::PrescaledSinHalfPiMinusRational(value)
            | Self::PrescaledCotHalfPiMinusRational(value)
            | Self::TanLargeRational(value)
            | Self::PrescaledTanRational(value) => rational_storage_bits(value),
            Self::LinearCombination3(combination) => combination
                .values
                .iter()
                .map(rational_storage_bits)
                .max()
                .unwrap_or(0),
            Self::NormalQuantile(data) => data.seed.bits(),
            _ => 0,
        }
    }
}

/// Traverse shared expression structure iteratively for diagnostic size/depth metrics.
impl Computable {
    /// Distinct nodes, longest path and largest stored rational of this
    /// expression graph, visiting at most `node_limit` distinct nodes.
    /// Allocation identity deduplicates shared nodes in the valid acyclic graph;
    /// it is not mathematical equality. An explicit stack performs postorder
    /// depth calculation. If the limit is reached, return partial node/bit counts
    /// and the largest depth completed so far, which can still be zero.
    ///
    /// Zero limit is incomplete even for a single node. Traversal bookkeeping
    /// is linear in visited nodes; rational canonicalization and bigint costs
    /// are additional. The limit does not cap bytes, all work or cancellation
    /// latency, and this diagnostic has no abort check.
    pub(crate) fn representation_size(&self, node_limit: usize) -> RepresentationSize {
        // Iterative post-order over the shared graph: each node's depth is
        // derived once from its operands, so shared subgraphs stay linear and
        // deep chains cannot overflow the call stack.
        let mut depths = std::collections::HashMap::<*const Node, usize>::new();
        let mut stack = vec![(self, false)];
        let mut started = 0_usize;
        let mut max_rational_bits = 0;
        let mut depth = 0;
        while let Some((value, expanded)) = stack.pop() {
            let key = Arc::as_ptr(&value.internal);
            if expanded {
                let mut operand_depth = 0;
                value.internal.approximation.for_each_operand(|operand| {
                    let operand_key = Arc::as_ptr(&operand.internal);
                    operand_depth =
                        operand_depth.max(depths.get(&operand_key).copied().unwrap_or(0));
                });
                depth = depth.max(operand_depth + 1);
                depths.insert(key, operand_depth + 1);
                continue;
            }
            // An acyclic graph cannot reach a node whose operands are still
            // pending, so a repeated entry is either complete or unvisited.
            if depths.contains_key(&key) {
                continue;
            }
            if started == node_limit {
                return RepresentationSize {
                    nodes: started,
                    depth,
                    max_rational_bits,
                    complete: false,
                };
            }
            started += 1;
            max_rational_bits =
                max_rational_bits.max(value.internal.approximation.stored_rational_bits());
            stack.push((value, true));
            value.internal.approximation.for_each_operand(|operand| {
                if !depths.contains_key(&Arc::as_ptr(&operand.internal)) {
                    stack.push((operand, false));
                }
            });
        }
        RepresentationSize {
            nodes: depths.len(),
            depth,
            max_rational_bits,
            complete: true,
        }
    }
}
