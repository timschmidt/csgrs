from pathlib import Path
import json

A = Path(__file__).resolve().parent
W = A.parent
assert json.loads((A / 'retained-structural-pair-diagnostic-20260928-v624-reaped.json').read_text())['outer_exit_code'] == 1
source = (W / 'hypercurve/src/bezier_split.rs').read_text()
old = '''            let roots = match polynomial.isolate_interval_roots(outer_lower, outer_upper, policy)? {'''
new = '''            // The domain's retained endpoints decide ownership below. Use
            // certified rational envelopes to schedule isolation when possible;
            // an unrelated exact boundary must not enter scalar endpoint
            // deflation or every subsequent Sturm subdivision. Keep the
            // original polynomial and its selected-root evidence unchanged.
            let rational_bound = |bound: &Real, upper: bool| {
                bound.exact_rational_ref().is_none().then(|| {
                    bound
                        .certified_rational_interval(-16)
                        .map(|bounds| Real::new(bounds[usize::from(upper)].clone()))
                }).flatten()
            };
            let lower_bound = rational_bound(outer_lower, false);
            let upper_bound = rational_bound(outer_upper, true);
            let roots = match polynomial.isolate_interval_roots(
                lower_bound.as_ref().unwrap_or(outer_lower),
                upper_bound.as_ref().unwrap_or(outer_upper),
                policy,
            )? {'''
assert source.count(old) == 1
source = source.replace(old, new)
test = '''
#[cfg(test)]
mod finite_root_envelope_regression {
    use super::*;

    #[test]
    fn rational_isolation_envelopes_retain_exact_endpoint_ownership() {
        // t(t-2)(t^2-2) has the four independently known roots
        // -sqrt(2), 0, sqrt(2), and 2. Pi is an unrelated exact boundary.
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(polynomial) = crate::BezierParameterPolynomial::try_new_power_basis(
                [0, 4, -2, -2, 1].into_iter().map(Real::from).collect(),
                &policy,
            ).unwrap() else { panic!("the rational polynomial must be admitted") };
            let root = Real::from(2).sqrt().unwrap();
            for (lower, upper, endpoints, interior) in [
                (-root.clone(), root.clone(), [true, true], Real::zero()),
                (root.clone(), Real::pi(), [true, false], Real::from(2)),
            ] {
                for inclusion in [[true, true], [false, true], [true, false], [false, false]] {
                    for reversed in [false, true] {
                        let (start, end) = if reversed { (&upper, &lower) } else { (&lower, &upper) };
                        let range = CurveParameterRange2::new_validated(start.clone().into(), end.clone().into());
                        let domain = CurveParameterDomain2::new(&range, None).with_finite_inclusion(inclusion);
                        let Classification::Decided(roots) = domain.finite_roots(&polynomial, &policy).unwrap()
                        else { panic!("exact finite boundaries must retain the complete root inventory") };
                        let mut expected = Vec::new();
                        if endpoints[0] && inclusion[0] { expected.push(lower.clone()); }
                        expected.push(interior.clone());
                        if endpoints[1] && inclusion[1] { expected.push(upper.clone()); }
                        assert_eq!(roots.len(), expected.len());
                        for (actual, expected) in roots.iter().zip(expected) {
                            assert_eq!(actual.same_value(&BezierParameter2::Exact(expected), &policy).unwrap(), Classification::Decided(true));
                        }
                    }
                }
            }
        }
    }
}
'''
(A / 'finite-root-envelope-test-v625.rs').write_text(test)
(A / 'finite-root-envelope-candidate-v625.rs').write_text(source + test)
pair = (A / 'retained-structural-pair-candidate-v613.rs').read_text() + '\n' + (A / 'retained-structural-pair-domain-test-v623.rs').read_text()
(A / 'retained-structural-pair-candidate-v625.rs').write_text(pair)
driver = (A / 'probe-retained-structural-pair-20260928-v614.py').read_text()
driver = driver.replace('retained-structural-pair-20260928-v614', 'retained-structural-pair-20260928-v625').replace('retained-structural-pair-candidate-v613.rs', 'retained-structural-pair-candidate-v625.rs')
driver = driver.replace('composition=prior', "assert json.loads((A/'retained-structural-pair-diagnostic-20260928-v624-reaped.json').read_text())['outer_exit_code']==1\ncomposition=prior")
driver = driver.replace('guard=json.loads(', "envelope=A/'finite-root-envelope-candidate-v625.rs'\nenvelope_sha=hashlib.sha256(envelope.read_bytes()).hexdigest()\nguard=json.loads(", 1)
driver = driver.replace(" manifest[name]=hashlib.sha256(data).hexdigest()", " if name=='hypercurve/src/bezier_split.rs':data=envelope.read_bytes()\n manifest[name]=hashlib.sha256(data).hexdigest()")
driver = driver.replace('def verify():', 'def verify():\n assert hashlib.sha256(envelope.read_bytes()).hexdigest()==envelope_sha')
driver = driver.replace('report=dict(', "names['hypercurve'] += ['bezier_split::finite_root_envelope_regression::rational_isolation_envelopes_retain_exact_endpoint_ownership','bezier_offset::retained_structural_pair_domain_regression::retained_structural_correspondence_preserves_off_diagonal_contact_domains']\nreport=dict(", 1)
(A / 'probe-retained-structural-pair-20260928-v625.py').write_text(driver)
print('Prepared V625 rational-envelope candidate and 17-case probe')
