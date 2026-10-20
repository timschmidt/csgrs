// Keep an already admitted parameter quotient as one scalar atom. Expanding
// its unrelated internal constants (pi, logarithms, etc.) is unnecessary for
// the Jacobi identities. Every inverse hidden by this compression is checked
// first; structural equality reuses that exact domain proof on later copies.
impl<'a> RingRecognition<'a> {
    fn is_parameter_atom(&mut self, value: &Computable) -> Option<bool> {
        for index in 0..self.parameter_atoms.len() {
            self.charge()?;
            if Computable::internal_structural_eq(&self.parameter_atoms[index], value) {
                return Some(true);
            }
        }
        Some(false)
    }

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
