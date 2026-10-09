impl Real {
    /// Rebuilds a bounded Laurent polynomial in π with rational/radical coefficients.
    ///
    /// Coefficients are rational linear combinations of principal square roots
    /// of nonnegative rationals. This uses the existing bounded symbolic normal
    /// form, without approximation or increased recognition limits. It can keep
    /// repeated exact sums usable when their unreduced arithmetic graphs would
    /// exceed a later predicate's optional symbolic recognition budget.
    ///
    /// Already compact symbolic classes are retained. `None` means that an
    /// opaque remainder or a recognition limit prevented reconstruction; the
    /// original exact value remains available. The outer rational scale and
    /// any attached abort signal are preserved, along with an already proved
    /// exact sign for subsequent queries at lower precision.
    pub fn compact_pi_laurent(&self) -> Option<Self> {
        if !matches!(self.class, Irrational) {
            return Some(self.clone());
        }
        let computable = self.computable_ref().compact_pi_laurent()?;
        let mut compact = match computable.exact_rational() {
            Some(rational) => Self::new(rational),
            None => Self::irrational_from_computable(computable),
        };
        compact *= Self::new(self.rational.clone());
        if let Some(signal) = self.abort_signal() {
            compact.abort(signal.clone());
        }
        Some(compact)
    }
}

#[cfg(test)]
mod pi_laurent_compaction_tests {
    use super::*;

    #[test]
    fn compact_pi_laurent_replays_independent_binomial_identities() {
        for radicand in [2, 3, 5, 13, 34] {
            let root = Real::from(radicand).sqrt().unwrap();
            for power in [-3_i32, -1, 0, 1, 4] {
                // Build the Laurent representation explicitly. General powi
                // may instead retain an exp/log graph, outside this reducer.
                let mut pi = Real::one();
                for _ in 0..power.unsigned_abs() {
                    pi *= Real::pi();
                }
                if power < 0 {
                    pi = pi.inverse().unwrap();
                }
                for scale in [Rational::fraction(-7, 11).unwrap(), Rational::from(3)] {
                    let scale = Real::from(scale);
                    let binomial = &pi + &root;
                    let expanded = Real::diff_of_products(
                        &binomial,
                        &binomial,
                        &pi,
                        &pi,
                    ) * &scale;
                    // (p+r)^2-p^2 = 2pr+r^2, for either sign of the
                    // independent rational scale and positive/negative powers.
                    let expected = (Real::from(2) * &pi * &root + Real::from(radicand)) * scale;
                    let compact = expanded.compact_pi_laurent().unwrap_or_else(|| {
                        panic!("unrecognized Laurent identity: radicand={radicand}, power={power}")
                    });
                    assert_eq!(
                        (&compact - &expected).exact_rational_normal_form(),
                        Some(Rational::zero()),
                    );
                    assert_eq!(
                        (compact.compact_pi_laurent().unwrap() - &expected)
                            .exact_rational_normal_form(),
                        Some(Rational::zero()),
                    );
                }
            }
        }
    }

    #[test]
    fn compact_pi_laurent_retains_rational_results_and_abort_signal() {
        let root = Real::from(13).sqrt().unwrap();
        let a = Real::pi() + &root;
        let b = Real::pi() - &root;
        let mut value = Real::diff_of_products(&a, &b, &Real::pi(), &Real::pi());
        let signal = Arc::new(AtomicBool::new(false));
        value.abort(signal.clone());
        let compact = value.compact_pi_laurent().unwrap();
        assert_eq!(compact.exact_rational_ref(), Some(&Rational::from(-13)));
        assert!(Arc::ptr_eq(compact.abort_signal().unwrap(), &signal));
        signal.store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(compact.is_aborted());
        let aborted = value.compact_pi_laurent().unwrap();
        assert!(aborted.is_aborted());
        assert_eq!(aborted.exact_rational_ref(), Some(&Rational::from(-13)));
    }

    #[test]
    fn compact_pi_laurent_declines_opaque_remainders() {
        let value = Real::from(7).sin() + Real::from(3).sqrt().unwrap();
        assert!(value.compact_pi_laurent().is_none());
        let zero = &value - &value;
        assert_eq!(
            zero.compact_pi_laurent().unwrap().exact_rational_ref(),
            Some(&Rational::zero()),
        );
    }

    #[test]
    fn compact_pi_laurent_keeps_the_exp_log_power_frontier_explicit() {
        // Preserve the first failed matrix representation. Integer powers
        // spelled through this API can retain exp(log(pi) * exponent), which
        // the existing Laurent recognizer treats as an opaque atom.
        let pi = Real::pi().powi_i64(-3).unwrap();
        let binomial = &pi + Real::from(2).sqrt().unwrap();
        let value = Real::diff_of_products(&binomial, &binomial, &pi, &pi)
            * Real::from(Rational::fraction(-7, 11).unwrap());
        assert!(value.compact_pi_laurent().is_none());
    }

    #[test]
    fn compact_pi_laurent_retains_certified_signs_at_lower_precision() {
        let pi = Real::pi();
        let lower = pi.certified_rational_interval(-1024).unwrap()[0].clone();
        let delta = &pi - Real::new(lower);
        assert_eq!(
            delta.certified_sign_until(-1100).sign(),
            Some(RealSign::Positive),
        );
        let square = &delta * &delta;
        for (scale, expected) in [(-7, RealSign::Negative), (1, RealSign::Positive)] {
            let value = &square * Real::from(scale);
            assert_eq!(value.certified_sign_until(0).sign(), Some(expected));
            let compact = value.compact_pi_laurent().unwrap();
            assert_eq!(compact.immediate_sign(), Some(expected));
            for precision in [0, -512, 0] {
                assert_eq!(compact.certified_sign_until(precision).sign(), Some(expected));
            }
            let repeated = compact.compact_pi_laurent().unwrap();
            assert_eq!(repeated.certified_sign_until(0).sign(), Some(expected));
            assert_eq!(value.certified_sign_until(0).sign(), Some(expected));
        }
    }
}
