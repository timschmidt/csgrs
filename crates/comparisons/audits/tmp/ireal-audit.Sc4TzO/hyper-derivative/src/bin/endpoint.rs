use hypercurve::Real;
use std::{hint::black_box,time::{Duration,Instant}};
fn before(
    coefficients: &[Real],
    at_end: bool,
    max_order: usize,
) -> Option<Vec<Real>> {
    let value_count = max_order.checked_add(1)?;
    let mut derivatives = vec![Real::zero(); value_count];
    if !at_end {
        let mut factorial = 1_u64;
        for (order, derivative) in derivatives.iter_mut().enumerate() {
            if order > 1 {
                factorial = factorial.checked_mul(u64::try_from(order).ok()?)?;
            }
            if let Some(coefficient) = coefficients.get(order) {
                *derivative = if factorial == 1 {
                    coefficient.clone()
                } else {
                    Real::from(factorial) * coefficient
                };
            }
        }
        return Some(derivatives);
    }

    for coefficient in coefficients.iter().rev() {
        for order in (1..=max_order).rev() {
            let scale = Real::from(u64::try_from(order).ok()?);
            derivatives[order] = &derivatives[order] + &scale * &derivatives[order - 1];
        }
        derivatives[0] = &derivatives[0] + coefficient;
    }
    Some(derivatives)
}


fn after(
    coefficients: &[Real],
    at_end: bool,
    max_order: usize,
) -> Option<Vec<Real>> {
    let value_count = max_order.checked_add(1)?;
    let mut derivatives = Vec::new();
    derivatives.try_reserve_exact(value_count).ok()?;
    derivatives.resize(value_count, Real::zero());
    if !at_end {
        // At zero, the k-th derivative is k! times coefficient k. The
        // remaining derivatives vanish without computing larger factorials.
        let mut factorial = Real::one();
        for (order, (derivative, coefficient)) in
            derivatives.iter_mut().zip(coefficients).enumerate()
        {
            if order > 1 {
                factorial *= Real::from(u64::try_from(order).ok()?);
            }
            *derivative = if order < 2 {
                coefficient.clone()
            } else {
                &factorial * coefficient
            };
        }
        return Some(derivatives);
    }

    for coefficient in coefficients.iter().rev() {
        for order in (1..=max_order).rev() {
            let scale = Real::from(u64::try_from(order).ok()?);
            derivatives[order] = &derivatives[order] + &scale * &derivatives[order - 1];
        }
        derivatives[0] = &derivatives[0] + coefficient;
    }
    Some(derivatives)
}


fn main(){
 let args:Vec<_>=std::env::args().skip(1).collect();
 let mode=&args[0];let degree=args[1].parse::<usize>().unwrap();let order=args[2].parse::<usize>().unwrap();
 let coefficients:Vec<_>=(0..=degree).map(|i|Real::from((i+1) as u64)).collect();
 let f=if mode=="before"{before}else{after};
 let start=Instant::now();let mut iterations=0_u64;
 loop {black_box(f(black_box(&coefficients),false,black_box(order)).unwrap());iterations+=1;
 if start.elapsed()>=Duration::from_millis(200){break;}}
 println!("{mode}\t{degree}\t{order}\t{iterations}\t{:.3}",start.elapsed().as_nanos() as f64/iterations as f64);
}
