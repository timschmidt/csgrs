from pathlib import Path
import hashlib,json

A=Path(__file__).resolve().parent;W=A.parent
C=A/'rational-resultant-samples-candidate-v679'
assert not C.exists()
paths=['hypersolve/src/resultant.rs','hypersolve/src/curve_resultant.rs','hypercurve/src/bezier_offset.rs']
base={name:hashlib.sha256((W/name).read_bytes()).hexdigest()for name in paths}
for name in paths:
 p=C/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((W/name).read_bytes())
(A/'rational-resultant-samples-base-v679.json').write_text(json.dumps(base,indent=2)+'\n')

p=C/'hypersolve/src/resultant.rs';s=p.read_text()
a=s.index('    if let (Some(left_integers), Some(right_integers)) = (',s.index('pub(crate) fn resultant_exact_rational_polynomials_value('))
b=s.index('        let dimension =',a)
s=s[:a]+'''    // Normalize each input once, instead of clearing the same polynomial's
    // denominators independently in every Sylvester row. Restore both signed
    // contents to their resultant homogeneity powers after integer elimination.
    let integer_content = |coefficients: &[Real]| {
        let rational = coefficients
            .iter()
            .map(Real::exact_rational_ref)
            .collect::<Option<Vec<_>>>()?;
        let integers = Rational::primitive_bigint_ratio(&rational);
        let index = rational.iter().position(|coefficient| !coefficient.is_zero())?;
        let scale = rational[index] / Rational::from_bigint(integers[index].clone());
        Some((integers, scale))
    };
    if let (Some((left_integers, left_scale)), Some((right_integers, right_scale))) =
        (integer_content(left), integer_content(right))
    {
''' +s[b:]
old='''            return Ok(Real::from(Rational::from_bigint(determinant)));'''
assert s.count(old)==1
s=s.replace(old,'''            return Ok(if determinant.is_zero() {
                Real::zero()
            } else {
                Real::from(Rational::from_bigint(determinant))
                    * real_pow(&Real::from(left_scale), right_degree)
                    * real_pow(&Real::from(right_scale), left_degree)
            });''')
# Existing property compares two independently implemented determinant paths.
# Extend its exact coefficient domain to differing positive denominators.
start=s.index('fn generated_exact_rational_scalar_resultants_match_full_reports(')
end=s.index('\n        #[test]',start)
test=s[start:end]
test=test.replace('prop::collection::vec(-4_i16..=4, 1..=4)','prop::collection::vec((-4_i16..=4, 1_u16..=11), 1..=4)')
test=test.replace('.map(|coefficient| real(i64::from(coefficient)))','.map(|(numerator, denominator)| Real::from(Rational::fraction(i64::from(numerator), u64::from(denominator)).unwrap()))')
test=test.replace('''            prop_assert_eq!(
                resultant_exact_rational_polynomials_value(&left, &right, -64),
                Ok(expected),
            );''','''            prop_assert!(
                resultant_exact_rational_polynomials_value(&left, &right, -64) == Ok(expected)
            );''')
s=s[:start]+test+s[end:]
s=s.replace('fn scalar_resultant_keeps_the_exact_rational_fallback()', 'fn scalar_resultant_preserves_exact_fractional_scale()')
p.write_text(s)

p=C/'hypersolve/src/curve_resultant.rs';s=p.read_text()
s=s.replace('    UnivariateResultantError, resultant_univariate_polynomials, sylvester_matrix,','    UnivariateResultantError, resultant_exact_rational_polynomials_value,\n    resultant_univariate_polynomials, sylvester_matrix,')
old='''        let resultant =
            match resultant_univariate_polynomials(&first, &second, config.min_precision) {
                Ok(report) => report.resultant,
                Err(error) => {
                    return curve_resultant_report(
                        CurveIntersectionResultantStatus::ResultantError,
                        retained_parameter,
                        eliminated_parameter,
                        degree_bound,
                        Vec::new(),
                        Some(error),
                    );
                }
            };'''
assert s.count(old)==1
s=s.replace(old,'''        // Rationality was established from every original coefficient, so
        // specialization stays in that exact field. Samples need only the
        // value; constructing and discarding pivot reports loses useful work.
        let sampled = if rational_coefficients {
            resultant_exact_rational_polynomials_value(&first, &second, config.min_precision)
        } else {
            resultant_univariate_polynomials(&first, &second, config.min_precision)
                .map(|report| report.resultant)
        };
        let resultant = match sampled {
            Ok(value) => value,
            Err(error) => {
                return curve_resultant_report(
                    CurveIntersectionResultantStatus::ResultantError,
                    retained_parameter,
                    eliminated_parameter,
                    degree_bound,
                    Vec::new(),
                    Some(error),
                );
            }
        };''')
needle='''    #[test]
    fn rational_resultant_preserves_specialized_degree_drop_identity()'''
assert s.count(needle)==1
s=s.replace(needle,'''    #[test]
    fn rational_interpolation_preserves_signed_scale_at_degree_drops_on_either_axis() {
        // F=c(t u^7-(t+2)), G=d((t-1)u^6-(t+3)). Coprime exponents
        // give Res(F,G)=c^6 d^7 ((t-1)^7(t+2)^6-t^6(t+3)^7).
        // Dimension 13 uses interpolation; t=0 and t=1 must be skipped as
        // samples without losing their values in the completed polynomial.
        let c = Real::from(Rational::fraction(2, 3).unwrap());
        let d = Real::from(Rational::fraction(-5, 7).unwrap());
        let mut first = vec![vec![real(0); 8]; 2];
        first[0][0] = -real(2) * &c;
        first[1][0] = -c.clone();
        first[1][7] = c.clone();
        let first = BivariatePolynomial::new(first);
        let mut second = vec![vec![real(0); 7]; 2];
        second[0][0] = -real(3) * &d;
        second[1][0] = -d.clone();
        second[0][6] = -d.clone();
        second[1][6] = d.clone();
        let second = BivariatePolynomial::new(second);
        let pow = |value: &Real, exponent: usize| {
            (0..exponent).fold(Real::one(), |result, _| result * value)
        };
        let scale = pow(&c, 6) * pow(&d, 7);
        for retained in [CurveResultantParameter::First, CurveResultantParameter::Second] {
            let (first, second, orientation) = match retained {
                CurveResultantParameter::First => (first.clone(), second.clone(), real(1)),
                CurveResultantParameter::Second =>
                    (swap_bivariate(&first), swap_bivariate(&second), real(-1)),
            };
            let report = resultant_bivariate_polynomial_system(
                &first, &second, retained, CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
            assert_eq!(report.degree_bound, 13);
            assert!(report.resultant_coefficients.len() <= 14);
            // Nineteen independent values determine the entire polynomial
            // of degree at most 13, including both exceptional fibers.
            for parameter in (-3..=15).map(real) {
                let expected = &orientation * &scale *
                    (pow(&(&parameter-real(1)), 7) * pow(&(&parameter+real(2)), 6)
                    - pow(&parameter, 6) * pow(&(&parameter+real(3)), 7));
                assert!(eval_univariate(&report.resultant_coefficients, &parameter) == expected);
            }
        }
    }

'''+needle)
p.write_text(s)

p=C/'hypercurve/src/bezier_offset.rs';s=p.read_text()
assert 'mod structural_overlap_trace_regression' not in s
s+='\n'+(A/'structural-overlap-trace-probe-v634.rs').read_text()
p.write_text(s)
print('Isolated two-file Hypersolve candidate plus unchanged four-case Hypercurve stress fixture')
