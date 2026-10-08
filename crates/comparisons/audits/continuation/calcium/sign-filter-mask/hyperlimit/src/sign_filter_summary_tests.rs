use super::*;
use hyperreal::Rational;

fn unresolved() -> Real {
    let value = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());
    assert!(value.structural_facts().sign.is_none());
    value
}

fn reference(terms: &[(&Real, Sign)]) -> Option<PredicateOutcome<Sign>> {
    let mut signs = Vec::with_capacity(terms.len());
    for (value, multiplier) in terms {
        if let Some(sign) = signed_nonzero_term(value, *multiplier)? {
            signs.push(sign);
        }
    }
    finish_signed_term_filter(&signs)
}

fn expected(tokens: &[usize]) -> Option<PredicateOutcome<Sign>> {
    // 0 is a known zero; 1/4 positive, 2/3 negative, 5 known times zero.
    // 6/7 have unknown signs, even when the multiplier is zero.
    if tokens.iter().any(|t| *t >= 6) {
        return None;
    }
    let positive = tokens.iter().any(|t| matches!(t, 1 | 4));
    let negative = tokens.iter().any(|t| matches!(t, 2 | 3));
    if positive && negative {
        None
    } else {
        Some(PredicateOutcome::decided(
            if positive { Sign::Positive } else if negative { Sign::Negative } else { Sign::Zero },
            Certainty::Filtered,
            Escalation::Filter,
        ))
    }
}

#[test]
fn sign_summary_exhaustive_short_sequences_match_independent_sign_table() {
    let zero = Real::zero();
    let positive = Real::from(7);
    let negative = Real::from(-11);
    let unknown = unresolved();
    let choices = [(&zero, Sign::Positive), (&positive, Sign::Positive),
        (&negative, Sign::Positive), (&positive, Sign::Negative),
        (&negative, Sign::Negative), (&positive, Sign::Zero),
        (&unknown, Sign::Positive), (&unknown, Sign::Zero)];
    let mut cases = 0;
    for len in 0..=6_u32 {
        for code in 0..8_usize.pow(len) {
            let mut digits = code;
            let tokens = (0..len).map(|_| { let token = digits % 8; digits /= 8; token }).collect::<Vec<_>>();
            let terms = tokens.iter().map(|i| choices[*i]).collect::<Vec<_>>();
            let want = expected(&tokens);
            assert_eq!(signed_term_filter(&terms), want, "{tokens:?}");
            assert_eq!(reference(&terms), want, "reference {tokens:?}");
            cases += 1;
        }
    }
    assert_eq!(cases, 299_593);
}

#[test]
fn sign_summary_long_sequences_preserve_late_unknown_and_zero_cases() {
    let zero = Real::zero();
    let positive = Real::pi();
    let negative = -Real::e();
    let unknown = unresolved();
    let choices = [(&zero, Sign::Positive), (&positive, Sign::Positive),
        (&negative, Sign::Positive), (&positive, Sign::Negative),
        (&negative, Sign::Negative), (&positive, Sign::Zero),
        (&unknown, Sign::Positive), (&unknown, Sign::Zero)];
    let mut cases = 0;
    for len in [5, 8, 32, 128, 512] {
        for base in [0, 1, 2, 5] {
            for inserted in 0..8 {
                for index in [0, 1, len / 2, len - 1] {
                    let mut tokens = vec![base; len];
                    tokens[index] = inserted;
                    for mixed_prefix in [false, true] {
                        if mixed_prefix { tokens[0] = 1; tokens[1] = 2; }
                        let terms = tokens.iter().map(|i| choices[*i]).collect::<Vec<_>>();
                        let want = expected(&tokens);
                        assert_eq!(signed_term_filter(&terms), want, "len={len}, index={index}, inserted={inserted}");
                        assert_eq!(reference(&terms), want);
                        cases += 1;
                    }
                }
            }
        }
    }
    assert_eq!(cases, 1280);
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn sign_summary_keeps_complete_trace_counts_including_unknown_after_mixed() {
    use hyperreal::dispatch_trace;
    let zero = Real::zero();
    let positive = Real::from(7);
    let negative = Real::from(-11);
    let unknown = unresolved();
    for len in [5, 8, 32, 128] {
        for unknown_position in [None, Some(0), Some(2), Some(len - 1)] {
            for multiplier in [Sign::Positive, Sign::Zero] {
                let mut terms = vec![(&zero, Sign::Positive); len];
                terms[0] = (&positive, Sign::Positive);
                terms[1] = (&negative, Sign::Positive);
                if let Some(index) = unknown_position { terms[index] = (&unknown, multiplier); }
                let want = reference(&terms);
                assert_eq!(signed_term_filter(&terms), want);
                dispatch_trace::reset();
                let old = dispatch_trace::with_recording(|| reference(&terms));
                let old_trace = dispatch_trace::take_trace();
                dispatch_trace::reset();
                let new = dispatch_trace::with_recording(|| signed_term_filter(&terms));
                let new_trace = dispatch_trace::take_trace();
                assert_eq!(old, new);
                assert_eq!(old_trace, new_trace);
                assert_eq!(new_trace.path_count("hyperlimit", "signed_term_filter", "mixed-signs"), u64::from(unknown_position.is_none()));
            }
        }
    }
}
