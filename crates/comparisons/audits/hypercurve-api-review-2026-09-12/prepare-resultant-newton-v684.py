from pathlib import Path
import json,shutil
A=Path(__file__).resolve().parent;old=A/'rational-resultant-samples-candidate-v679';new=A/'resultant-newton-candidate-v684'
assert not new.exists();shutil.copytree(old,new)
(A/'rational-resultant-samples-20260928-v680-reaped.json').write_text(json.dumps({'outer_session':66933,'outer_exit_code':1},indent=2)+'\n')
p=new/'hypersolve/src/curve_resultant.rs';s=p.read_text();a=s.index('fn interpolate_samples(');b=s.index('\nfn multiply_by_linear_factor',a)
s=s[:a]+'''fn interpolate_samples(samples: &[PolynomialSample], min_precision: i32) -> Option<Vec<Real>> {
    // Divided differences retain the exact sampled polynomial on arbitrary
    // distinct nodes, including grids with omitted degree-drop fibers. Unlike
    // rebuilding every Lagrange basis, construction takes quadratic work and
    // linear live storage. Coefficients remain arbitrary exact Real values.
    let mut differences = samples.iter().map(|sample| sample.value.clone()).collect::<Vec<_>>();
    for order in 1..samples.len() {
        for index in (order..samples.len()).rev() {
            differences[index] = ((&differences[index] - &differences[index - 1])
                / (&samples[index].parameter_value - &samples[index - order].parameter_value))
                .ok()?;
        }
    }
    let mut result = Vec::new();
    for (sample, coefficient) in samples.iter().zip(differences).rev() {
        result = multiply_by_linear_factor(result, -sample.parameter_value.clone());
        result[0] += coefficient;
    }
    trim_trailing_zeroes(result, min_precision).ok()
}
''' +s[b:]
needle='''    #[test]
    fn bivariate_affine_substitution_preserves_ragged_exact_polynomials()'''
assert s.count(needle)==1
s=s.replace(needle,'''    #[test]
    fn interpolation_preserves_arbitrary_exact_values_on_nonuniform_nodes() {
        let coefficients = vec![
            Real::pi(), real(2).sqrt().unwrap(),
            Real::from(Rational::fraction(-3, 5).unwrap()), real(7),
        ];
        let nodes = [real(-3), Real::from(Rational::fraction(1, 2).unwrap()), real(2), real(5)];
        for reversed in [false, true] {
            let mut samples = nodes.iter().map(|parameter| PolynomialSample {
                parameter_value: parameter.clone(),
                value: eval_univariate(&coefficients, parameter),
            }).collect::<Vec<_>>();
            if reversed { samples.reverse(); }
            let interpolated = interpolate_samples(&samples, -512).expect("distinct exact nodes interpolate");
            assert_eq!(interpolated.len(), coefficients.len());
            for (actual, expected) in interpolated.iter().zip(&coefficients) {
                assert!(matches!((actual-expected).certified_sign_until(-512),
                    CertifiedRealSign::Known { sign: RealSign::Zero, .. }));
            }
        }
        let duplicate = vec![
            PolynomialSample { parameter_value: real(1), value: real(2) },
            PolynomialSample { parameter_value: real(1), value: real(3) },
        ];
        assert!(interpolate_samples(&duplicate, -512).is_none());
        assert!(interpolate_samples(&[], -512).unwrap() == vec![real(0)]);
    }

'''+needle)
p.write_text(s)
prefix='resultant-newton-20260928-v685';s=(A/'probe-rational-resultant-samples-20260928-v680.py').read_text().replace('rational-resultant-samples-20260928-v680',prefix).replace('rational-resultant-samples-candidate-v679','resultant-newton-candidate-v684')
s=s.replace("assert (A/'spline-authoring-20260928-v678-committed.json').exists()", "assert (A/'spline-authoring-20260928-v678-committed.json').exists()\nassert json.loads((A/'rational-resultant-samples-20260928-v680-reaped.json').read_text())['outer_exit_code']==1")
p=A/f'probe-{prefix}.py';p.write_text(s);compile(s,str(p),'exec')
print('Prepared isolated quadratic-work interpolation candidate and rerun')
