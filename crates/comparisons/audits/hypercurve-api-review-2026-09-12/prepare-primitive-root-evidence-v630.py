from pathlib import Path
import json

A=Path(__file__).resolve().parent;W=A.parent
assert json.loads((A/'radical-polynomial-evaluation-20260928-v629-reaped.json').read_text())['outer_exit_code']==1
source=(W/'hypersolve/src/root_isolation.rs').read_text()
start=source.index('pub fn square_free_part(');end=source.index('\npub(crate) fn polynomial_div_rem(',start)
helper=source[start:end]
helper=helper.replace('let polynomial = trim_polynomial(polynomial, policy)?;', '''let polynomial = trim_polynomial(polynomial, policy)?;
    // Root evidence is unchanged by a nonzero rational content. Keep that
    // content out of the defining polynomial, its boundary evaluations and
    // subsequent coefficient fields. Arbitrary exact coefficients retain
    // their existing field and never undergo rational reconstruction.
    let polynomial = primitive_integer_polynomial(&polynomial).unwrap_or(polynomial);''')
helper=helper.replace('monic_normalize(gcd, policy)?', 'gcd')
source=source[:start]+helper+source[end:]
source=source.replace('/// The result has the same distinct roots as `polynomial`. `None` means that', '/// The result has the same distinct roots as `polynomial`. Rational input\n/// discards its common denominator and integer content. `None` means that',1)
test='''
#[cfg(test)]
mod primitive_root_evidence_regression {
    use super::*;

    #[test]
    fn square_free_root_evidence_removes_common_rational_scale() {
        // This is (25t^2-14)(400t^6+300t^4+75t^2-14).
        // alpha^2=(cbrt(81/25)-1)/4 is a root of the second factor.
        let original = [196, 0, -1400, 0, -2325, 0, 1900, 0, 10000].map(Real::from);
        let alpha = ((Real::new(HyperRational::fraction(81,25).unwrap()).root_n(3).unwrap()-Real::one())/Real::from(4)).unwrap().sqrt().unwrap();
        let wide = HyperRational::from_bigint_fraction(BigInt::from(1_u8)<<1024_usize, num::BigUint::from(625_u32)).unwrap();
        for scale in [HyperRational::fraction(16777216,625).unwrap(), wide] {
            for negative in [false,true] {
                let scale = Real::new(if negative {-scale.clone()}else{scale.clone()});
                let input=original.iter().map(|value| value*&scale).collect();
                let reduced=square_free_part(input,PredicatePolicy::STRICT).expect("rational root evidence must normalize");
                let expected=original.iter().map(|value|if negative {-value.clone()}else{value.clone()}).collect::<Vec<_>>();
                assert_eq!(reduced,expected);
                assert_eq!(Real::eval_poly(&reduced,&alpha).certified_sign_until(-512).sign(),Some(hyperreal::RealSign::Zero));
            }
        }
    }

    #[test]
    fn square_free_root_evidence_keeps_primitive_repeated_factor_quotients() {
        // (2t-1)^2(3t+1) -> (2t-1)(3t+1), without a monic
        // GCD reintroducing a redundant factor of two in the quotient.
        let scale=Real::new(HyperRational::fraction(-17,625).unwrap());
        let input=[1,-1,-8,12].map(|value|Real::from(value)*&scale).to_vec();
        let reduced=square_free_part(input,PredicatePolicy::STRICT).expect("exact repeated factor division");
        assert_eq!(reduced,[1,1,-6].map(Real::from));
        for root in [HyperRational::fraction(1,2).unwrap(),HyperRational::fraction(-1,3).unwrap()] {
            assert!(Real::eval_poly(&reduced,&Real::new(root)).definitely_zero());
        }
    }
}
'''
(A/'primitive-root-evidence-tests-v630.rs').write_text(test)
(A/'primitive-root-evidence-candidate-v630.rs').write_text(source+test)
driver=(A/'probe-retained-structural-pair-20260928-v625.py').read_text()
driver=driver.replace('retained-structural-pair-20260928-v625','primitive-root-evidence-20260928-v630')
driver=driver.replace('composition=prior',"assert json.loads((A/'radical-polynomial-evaluation-20260928-v629-reaped.json').read_text())['outer_exit_code']==1\ncomposition=prior")
driver=driver.replace('guard=json.loads(',"root_candidate=A/'primitive-root-evidence-candidate-v630.rs'\nroot_candidate_sha=hashlib.sha256(root_candidate.read_bytes()).hexdigest()\nguard=json.loads(",1)
driver=driver.replace(" manifest[name]=hashlib.sha256(data).hexdigest()"," if name=='hypersolve/src/root_isolation.rs':data=root_candidate.read_bytes()\n manifest[name]=hashlib.sha256(data).hexdigest()")
driver=driver.replace('def verify():','def verify():\n assert hashlib.sha256(root_candidate.read_bytes()).hexdigest()==root_candidate_sha')
# First measure the public closure on the ordinary integration binary; the
# owning-layer regressions are retained in the copied source for the next run.
start=driver.index('names=');end=driver.index('report=dict(',start)
driver=driver[:start]+"names={'hypercurve_analytic_parallel_region':['radical_parallel_cusp_offsets_exactly_under_both_policies','radical_parallel_cusp_spans_connect_under_both_policies']}\n"+driver[end:]
driver=driver.replace("[cargo,'test','--lib','--test'","[cargo,'test','--test'")
(A/'probe-primitive-root-evidence-20260928-v630.py').write_text(driver)
print('Prepared primitive root-evidence candidate and public composition probe')
