use hypercurve::Real;
use hypersolve::{DenseTensorPolynomial, PredicatePolicy};
use std::hint::black_box;
use std::time::Instant;

fn main() {
    for steps in [2_000, 8_000, 32_000] {
        let (mut first, mut second) = (Real::zero(), Real::one());
        for _ in 0..steps {
            (first, second) = (second.clone(), first + second);
        }
        let top = &first + &second;
        let expected = vec![&first + Real::from(2) * &top, second.clone()];
        let tensor = DenseTensorPolynomial::try_new(vec![3], vec![first, second, top]).unwrap();
        for scale in [Real::one(), Real::from(-3)] {
            let modulus = [Real::from(-2) * &scale, Real::zero(), scale];
            let start = Instant::now();
            let result = black_box(&tensor).reduce_axis_modulo(0, &modulus, PredicatePolicy::STRICT).unwrap();
            let elapsed = start.elapsed();
            assert_eq!(result.dimensions(), &[2]);
            assert_eq!(result.coefficients(), expected.as_slice());
            println!("fibonacci_steps={steps} elapsed={elapsed:?} exact=true");
        }
    }
}
