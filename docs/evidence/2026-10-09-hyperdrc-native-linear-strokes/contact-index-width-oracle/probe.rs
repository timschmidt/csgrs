/// Upper bound on the width of a finite, ordered binary64 interval.
fn width_upper_bound(low: f64, high: f64) -> f64 {
    if low == high {
        return 0.0;
    }
    let width = high - low;
    if !width.is_finite() {
        return width;
    }
    // Two-Diff recovers the subtraction error. Preserve an exact result or
    // an existing upper bound; step upward only for downward rounding.
    // See https://people.eecs.berkeley.edu/~jrs/papers/robustr.pdf, Theorem 7.
    let virtual_low = high - width;
    let residual = (high - (width + virtual_low)) + (virtual_low - low);
    if residual.is_finite() && residual <= 0.0 {
        width
    } else {
        // Nonfinite intermediates cannot establish a tighter upper bound.
        width.next_up()
    }
}

use std::collections::BTreeMap;
fn add(cases: &mut BTreeMap<(u64,u64), &'static str>, a: f64, b: f64, bucket: &'static str) {
    if !a.is_finite() || !b.is_finite() { return; }
    let (low,high) = if a > b { (b,a) } else { (a,b) };
    cases.entry((low.to_bits(),high.to_bits())).or_insert(bucket);
}
fn main() {
    let mut values = vec![-f64::MAX,-1.0,-0.0,0.0,1.0,f64::MAX];
    for exponent in [-1022,-1000,-600,-54,-53,0,53,600,1000,1023] {
        let power = f64::from_bits(((1023+exponent) as u64)<<52);
        for value in [power.next_down(),power,power.next_up()] { values.extend([value,-value]); }
    }
    for bits in [1,2,3,(1_u64<<52)-1] { let value=f64::from_bits(bits); values.extend([value,-value]); }
    let mut cases = BTreeMap::new();
    for &a in &values { for &b in &values { add(&mut cases,a,b,"critical_grid"); } }
    for exponent in 1..2047_u64 {
        let value=f64::from_bits(exponent<<52);
        for (a,b) in [
            (value.next_down(),value.next_up()),(-value,value),(-value.next_up(),value),
            (-value,value.next_up()),(-value*2_f64.powi(-54),value),(0.0,value),(-value,0.0),
            (-value,-value.next_down()),(value,f64::MAX),(-f64::MAX,value),(value*0.5,value),
            (value,value),(-value,-value),
        ] { add(&mut cases,a,b,"exponent_edges"); }
    }
    let half=f64::MAX*0.5;
    for (a,b) in [(-half,half),(-half.next_down(),half),(-half.next_up(),half),(0.0,f64::MAX),(-f64::MAX,0.0)] {
        add(&mut cases,a,b,"max_width_edges");
    }
    let mut state=0x9e3779b97f4a7c15_u64;
    for _ in 0..65536 {
        state=state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let a=f64::from_bits(state);
        state=state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let b=f64::from_bits(state);
        add(&mut cases,a,b,"deterministic_bits");
    }
    for ((low_bits,high_bits),bucket) in cases {
        let low=f64::from_bits(low_bits); let high=f64::from_bits(high_bits);
        let upper=width_upper_bound(low,high);
        println!("{{\"bucket\":\"{bucket}\",\"low_bits\":{low_bits},\"high_bits\":{high_bits},\"upper_bits\":{}}}",upper.to_bits());
    }
}
