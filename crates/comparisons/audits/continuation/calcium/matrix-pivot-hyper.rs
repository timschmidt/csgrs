use hyperreal::Real;
use hyperlimit::{PredicatePolicy, compare_reals};
use hypersolve::{BareissDeterminantMethod, determinant_bareiss};
use std::cmp::Ordering;

fn main() {
    let mut cases = 0;
    let mut unknown = 0;
    let mut pivot_free = 0;
    println!("kind,n,shape,expected_zero,method,equality");
    for kind in 0..3 { for n in 0..=10 { for shape in 0..6 {
        let base = match kind {
            0 => Real::one(), 1 => Real::from(2).sqrt().unwrap(),
            _ => Real::from(2).ln().unwrap(),
        };
        let mut matrix = vec![vec![Real::zero();n];n];
        let mut expected = Real::one();
        for (i,row) in matrix.iter_mut().enumerate() {
            row[i] = &base * Real::from((i+1) as i64);
            expected *= &row[i];
        }
        let expected_zero = n > 0 && ((1..=3).contains(&shape) || (shape == 5 && n > 1));
        if (1..=3).contains(&shape) && n > 0 {
            let i = if shape == 1 {0} else if shape == 2 {n/2} else {n-1};
            matrix[i][i] = Real::zero();
        }
        if shape == 4 && n > 1 { matrix.swap(0,n-1); expected = -expected; }
        if shape == 5 && n > 1 { matrix[n-1] = matrix[0].clone(); }
        if expected_zero { expected = Real::zero(); }
        let report = determinant_bareiss(&matrix, -128).unwrap();
        let method = match report.method {
            BareissDeterminantMethod::FractionFree => "FractionFree",
            BareissDeterminantMethod::PivotFreeFaddeevLeverrier => { pivot_free += 1; "PivotFree" },
        };
        let equality = match compare_reals(&report.determinant, &expected, PredicatePolicy::STRICT).value() {
            Some(Ordering::Equal) => "Equal",
            None => { unknown += 1; "Unknown" },
            Some(_) => panic!("incorrect determinant: kind={kind} n={n} shape={shape}"),
        };
        println!("{kind},{n},{shape},{},{method},{equality}",usize::from(expected_zero));
        cases += 1;
    } } }
    println!("{{\"suite\":\"hyper-matrix-pivots\",\"cases\":{cases},\"unknown\":{unknown},\"pivot_free\":{pivot_free}}}");
}
