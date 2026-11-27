// Keep an already admitted parameter quotient as one scalar atom. Expanding
// its unrelated internal constants (pi, logarithms, etc.) is unnecessary for
// the Jacobi identities. Inverses in the traversed arithmetic skeleton are
// checked first; other native leaves must already be admitted. Structural
// equality reuses that query-local admission on later copies.
/// Compress selected, already admitted Jacobi parameter arithmetic without expanding unrelated constants.
impl<'a> RingRecognition<'a> {
    /// Find a structurally equal parameter already compressed during this recognition attempt.
    /// Charge each comparison; Some(false) means no match, while None means a work
    /// or stop check declined. Structural matching has its own cost, not a unit-time
    /// guarantee. A match reuses admission in this query, not global scalar equality.
    fn is_parameter_atom(&mut self, value: &Computable) -> Option<bool> {
        for index in 0..self.parameter_atoms.len() {
            self.charge()?;
            if Computable::internal_structural_eq(&self.parameter_atoms[index], value) {
                return Some(true);
            }
        }
        Some(false)
    }

    /// Check the arithmetic skeleton before hiding a parameter quotient inside an atom.
    /// Charge an operation and node per visited recipe, and require a positive depth.
    /// Every encountered Inverse needs an immediate nonzero sign, then its child is
    /// checked recursively. Add/Multiply short-circuit left before right; unary
    /// arithmetic and all three linear-form coefficient nodes recurse. Other recipes
    /// are accepted as already admitted leaves, without inspecting their descendants.
    /// This is not whole-graph domain validation. Some(false) is a failed guard; None
    /// is a visit/depth/work or cancellation limit, with spent counters retained.
    fn guard_parameter_arithmetic(&mut self, value: &Computable, depth: usize) -> Option<bool> {
        self.charge()?;
        if depth == 0 {
            return None;
        }
        self.nodes_left = self.nodes_left.checked_sub(1)?;
        match &value.internal.approximation {
            Approximation::Inverse(child) => {
                if !matches!(
                    child.immediate_sign(),
                    Some(RealSign::Positive | RealSign::Negative)
                ) {
                    return Some(false);
                }
                self.guard_parameter_arithmetic(child, depth - 1)
            }
            Approximation::Add(a, b) | Approximation::Multiply(a, b) => Some(
                self.guard_parameter_arithmetic(a, depth - 1)?
                    && self.guard_parameter_arithmetic(b, depth - 1)?,
            ),
            Approximation::Square(child)
            | Approximation::Negate(child)
            | Approximation::Offset(child, _) => self.guard_parameter_arithmetic(child, depth - 1),
            Approximation::LinearCombination3(combination) => {
                for child in &combination.coefficients {
                    if !self.guard_parameter_arithmetic(child, depth - 1)? {
                        return Some(false);
                    }
                }
                Some(true)
            }
            _ => Some(true),
        }
    }

    /// Collect selected quotient subexpressions from a retained log-complement recipe.
    /// For PrescaledLn, inspect its log1p residual and peel at most eight negations,
    /// offsets or rational-leaf products. Retain a direct product with an inverse
    /// factor only after structural deduplication and arithmetic-domain guarding.
    /// Recurse through sums and rational scaling of logarithms using the supplied
    /// depth; depth zero is a successful no-op, not a complete scan. At the atom cap
    /// or on a false domain guard, simply omit that candidate. None reports a charged
    /// substep declining; earlier collected atoms/counter changes remain query-local.
    fn parameter_from_logarithm(&mut self, log: &Computable, depth: usize) -> Option<()> {
        self.charge()?;
        if depth == 0 {
            return Some(());
        }
        match &log.internal.approximation {
            Approximation::PrescaledLn(residual) => {
                let mut base = residual;
                for _ in 0..8 {
                    self.charge()?;
                    base = match &base.internal.approximation {
                        Approximation::Negate(child) | Approximation::Offset(child, _) => child,
                        Approximation::Multiply(a, b) if ExpRelationForm::leaf(a).is_some() => b,
                        Approximation::Multiply(a, b) if ExpRelationForm::leaf(b).is_some() => a,
                        _ => break,
                    };
                }
                let quotient = matches!(&base.internal.approximation, Approximation::Multiply(a,b)
                    if matches!(a.internal.approximation, Approximation::Inverse(_))
                    || matches!(b.internal.approximation, Approximation::Inverse(_)));
                if quotient
                    && !self.is_parameter_atom(base)?
                    && self.parameter_atoms.len() < Self::MAX_ATOMS
                    && self.guard_parameter_arithmetic(base, Self::MAX_DEPTH)?
                {
                    self.parameter_atoms.push(base.clone());
                }
            }
            Approximation::Add(a, b) => {
                self.parameter_from_logarithm(a, depth - 1)?;
                self.parameter_from_logarithm(b, depth - 1)?;
            }
            Approximation::Negate(child) | Approximation::Offset(child, _) => {
                self.parameter_from_logarithm(child, depth - 1)?
            }
            Approximation::Multiply(a, b) => {
                if ExpRelationForm::leaf(a).is_some() {
                    self.parameter_from_logarithm(b, depth - 1)?;
                } else if ExpRelationForm::leaf(b).is_some() {
                    self.parameter_from_logarithm(a, depth - 1)?;
                }
            }
            _ => {}
        }
        Some(())
    }

    /// Scan the authoritative borrowed DAG for Jacobi log-complement parameters to compress.
    /// Charge each popped visit before depth/identity checks, then charge a node only
    /// for a new live Arc identity. Retained rational reductions terminate a branch.
    /// Arithmetic children are pushed in stored order and therefore scanned in reverse
    /// by the LIFO stack; Jacobi sn/cn/dn inspect only their log-complement payload.
    /// Do not insert temporary replay expressions into the identity set. The scan
    /// shares this attempt's counters and stop handle; its limits do not meter all
    /// structural-matcher or rational-cache work. None leaves partial local state.
    fn collect_parameter_atoms(&mut self, root: &'a Computable) -> Option<()> {
        let mut pending = vec![(root, Self::MAX_DEPTH)];
        let mut visited = std::collections::HashSet::new();
        while let Some((value, depth)) = pending.pop() {
            self.charge()?;
            if depth == 0 {
                return None;
            }
            // All pointers belong to the borrowed authoritative root. Never
            // insert temporary logarithm-replay expressions into this set.
            if !visited.insert(Arc::as_ptr(&value.internal)) {
                continue;
            }
            self.nodes_left = self.nodes_left.checked_sub(1)?;
            if value.retained_rational_reduction().is_some() {
                continue;
            }
            match &value.internal.approximation {
                Approximation::JacobiSn(a)
                | Approximation::JacobiCn(a)
                | Approximation::JacobiDn(a) => self.parameter_from_logarithm(&a[1], 8)?,
                Approximation::Add(a, b) | Approximation::Multiply(a, b) => {
                    pending.push((a, depth - 1));
                    pending.push((b, depth - 1));
                }
                Approximation::Square(child)
                | Approximation::Negate(child)
                | Approximation::Offset(child, _)
                | Approximation::Inverse(child) => pending.push((child, depth - 1)),
                Approximation::LinearCombination3(combination) => {
                    for child in &combination.coefficients {
                        pending.push((child, depth - 1));
                    }
                }
                _ => {}
            }
        }
        Some(())
    }
}
