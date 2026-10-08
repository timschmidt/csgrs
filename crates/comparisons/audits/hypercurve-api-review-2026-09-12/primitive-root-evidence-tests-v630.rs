
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
