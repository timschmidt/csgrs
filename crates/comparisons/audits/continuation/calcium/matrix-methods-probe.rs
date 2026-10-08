use hyperreal::{Rational, Real};
use hyperlimit::{PredicatePolicy, compare_reals};
use hypersolve::determinant_bareiss;
use std::cmp::Ordering;

// Mathematical comparison implementations, not production routing changes.
// Faddeev-LeVerrier uses the same exact -1/k coefficient recurrence as the
// frozen Hypersolve fallback. The border recurrence is division-free Berkowitz:
// each border supplies a Toeplitz column [1,-a,-v*u,-v*M*u,...].
fn faddeev(a: &[Vec<Real>]) -> Real {
    let n = a.len();
    if n == 0 { return Real::one(); }
    let mut b = vec![vec![Real::zero();n];n];
    for (i,row) in b.iter_mut().enumerate() { row[i] = Real::one(); }
    for k in 1..=n {
        let mut product = vec![vec![Real::zero();n];n];
        for i in 0..n { for j in 0..n {
            product[i][j] = (0..n).fold(Real::zero(),|s,t| s + &a[i][t] * &b[t][j]);
        } }
        let trace = (0..n).fold(Real::zero(),|s,i| s + &product[i][i]);
        let c = trace * Real::from(Rational::fraction(-1,k as u64).unwrap());
        if k == n { return if n % 2 == 0 {c} else {-c}; }
        for (i,row) in product.iter_mut().enumerate() { row[i] += &c; }
        b = product;
    }
    unreachable!()
}

fn berkowitz(a: &[Vec<Real>]) -> Real {
    let n = a.len();
    let mut coefficients = vec![Real::one()]; // descending monic coefficients
    for m in 1..=n {
        let border = m-1;
        let mut column = vec![Real::one(), -a[border][border].clone()];
        let mut power: Vec<_> = a[..border].iter().map(|row|row[border].clone()).collect();
        for k in 0..border {
            column.push(-(0..border).fold(Real::zero(),|s,i|s + &a[border][i] * &power[i]));
            if k+1 < border {
                power = (0..border).map(|i|(0..border).fold(Real::zero(),|s,j|s + &a[i][j] * &power[j])).collect();
            }
        }
        coefficients = (0..=m).map(|i|(0..=i.min(border)).fold(Real::zero(),|s,j|s + &coefficients[j] * &column[i-j])).collect();
    }
    let det = coefficients.pop().unwrap();
    if n % 2 == 0 {det} else {-det}
}

fn check(family: &str, kind: usize, n: usize, case: usize, a: &[Vec<Real>], expected: &Real) {
    // This is capability qualification only. Clocks during other qualification
    // processes are deliberately not treated as benchmark observations.
    for (method, actual) in [
        ("Bareiss",determinant_bareiss(a,-128).unwrap().determinant),
        ("Faddeev",faddeev(a)), ("Berkowitz",berkowitz(a)),
    ] {
        let equality = match compare_reals(&actual,expected,PredicatePolicy::STRICT).value() {
            Some(Ordering::Equal) => "Equal", None => "Unknown",
            Some(_) => panic!("disproved determinant: {family}/{kind}/{n}/{case}/{method}"),
        };
        println!("{family},{kind},{n},{case},{method},{equality}");
    }
}

fn base(kind: usize) -> Real {
    match kind {
        0 => Real::one(), 1 => Real::from(2).sqrt().unwrap(),
        2 => Real::from(2).ln().unwrap(),
        3 => {
            let [_,upper] = Real::pi().certified_dyadic_interval(-768).unwrap();
            Real::from(upper) - Real::pi()
        },
        _ => unreachable!(),
    }
}

// Independent finite Leibniz oracle in i128. n<=6 and coefficients within3
// bound the absolute sum by6!*3^6=524880, far below this accumulator's range.
fn integer_determinant(a: &[Vec<i64>], row: usize, used: u64) -> i128 {
    if row == a.len() { return 1; }
    (0..a.len()).filter(|&col|used & (1<<col) == 0).map(|col| {
        let sign = if (used >> (col+1)).count_ones() % 2 == 0 {1} else {-1};
        sign * i128::from(a[row][col]) * integer_determinant(a,row+1,used | (1<<col))
    }).sum()
}

fn main() {
    println!("family,kind,n,case,method,equality");
    for kind in 0..3 { for n in 0..=10 { for shape in 0..6 {
        let b = base(kind);
        let mut a = vec![vec![Real::zero();n];n]; let mut expected = Real::one();
        for (i,row) in a.iter_mut().enumerate() { row[i] = &b * Real::from((i+1) as i64); expected *= &row[i]; }
        if (1..=3).contains(&shape) && n > 0 {
            let i = if shape == 1 {0} else if shape == 2 {n/2} else {n-1}; a[i][i] = Real::zero(); expected = Real::zero();
        }
        if shape == 4 && n > 1 { a.swap(0,n-1); expected = -expected; }
        if shape == 5 && n > 1 { a[n-1] = a[0].clone(); expected = Real::zero(); }
        check("structured",kind,n,shape,&a,&expected);
    } } }
    for kind in 0..4 { for n in 0..=6 { for case in 0..4 {
        let mut seed = 907u32 + case as u32 * 171;
        let integers: Vec<Vec<_>> = (0..n).map(|_|(0..n).map(|_| {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            i64::from((seed >> 16) % 7) - 3
        }).collect()).collect();
        let b = base(kind);
        let a: Vec<Vec<_>> = integers.iter().map(|row|row.iter().map(|&c|&b * Real::from(c)).collect()).collect();
        let det = i64::try_from(integer_determinant(&integers,0,0)).unwrap();
        let expected = Real::from(det) * b.powi_i64(n as i64).unwrap();
        check("dense",kind,n,case,&a,&expected);
    } } }
    println!("{{\"suite\":\"matrix-methods\",\"inputs\":310,\"queries\":930}}");
}
