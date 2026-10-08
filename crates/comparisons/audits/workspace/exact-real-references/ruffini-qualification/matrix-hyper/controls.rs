use hyperreal::{Rational, Real};
use hyperlattice::{Complex, Matrix3, Matrix4};
use hypersolve::{BareissError, determinant_bareiss, solve_dense_linear_system_bareiss};
use hyperlimit::PredicatePolicy;

fn values(m: usize, n: usize, seed: i64) -> Vec<Vec<i64>> {
    (0..m).map(|i| (0..n).map(|j| (((i+1)*71+(j+3)*37+(i+1)*(j+1)*13) as i64+seed*29).rem_euclid(31)-15).collect()).collect()
}
fn determinant(a: &[Vec<i64>]) -> i64 {
    if a.is_empty() { return 1; }
    (0..a.len()).map(|j| {
        let minor: Vec<Vec<_>> = a.iter().skip(1).map(|r| r.iter().enumerate().filter_map(|(k,&v)| (k!=j).then_some(v)).collect()).collect();
        (if j%2==0 {1} else {-1}) * a[0][j] * determinant(&minor)
    }).sum()
}
fn main() {
    let mut checks=0;
    for n in 0..=5 { for seed in 0..12 {
        let a=values(n,n,seed);let real:Vec<Vec<Real>>=a.iter().map(|r|r.iter().copied().map(Real::from).collect()).collect();
        assert_eq!(determinant_bareiss(&real,-256).unwrap().determinant,Real::from(determinant(&a)));
        checks+=1;println!("PASS\tdeterminant-{n}-{seed}");
    }}
    for n in 1..=8 { for seed in 0..12 {
        let mut a=values(n,n,seed);for(i,r)in a.iter_mut().enumerate(){r[i]+=200;}
        let solution:Vec<i64>=(0..n).map(|i|i as i64-3).collect();
        let rhs:Vec<Real>=a.iter().map(|r|Real::from(r.iter().zip(&solution).map(|(x,y)|x*y).sum::<i64>())).collect();
        let real:Vec<Vec<Real>>=a.iter().map(|r|r.iter().copied().map(Real::from).collect()).collect();
        let got=solve_dense_linear_system_bareiss(&real,&rhs,-256,PredicatePolicy::STRICT).unwrap();
        assert_eq!(got.solution,solution.into_iter().map(Real::from).collect::<Vec<_>>());
        checks+=1;println!("PASS\tsolve-{n}-{seed}");
    }}
    assert!(matches!(determinant_bareiss(&[vec![Real::from(1),Real::from(2)]],-128),Err(BareissError::DimensionMismatch)));checks+=1;
    assert!(matches!(solve_dense_linear_system_bareiss(&[vec![Real::from(0)]],&[Real::from(1)],-128,PredicatePolicy::STRICT),Err(BareissError::Singular{..})));checks+=1;
    for seed in 0..40 {
        let a=values(3,3,seed);let b=values(3,3,seed+17);
        let ma=Matrix3::new(std::array::from_fn(|i|std::array::from_fn(|j|Real::from(a[i][j]))));
        let mb=Matrix3::new(std::array::from_fn(|i|std::array::from_fn(|j|Real::from(b[i][j]))));
        let product=&ma*&mb;
        for i in 0..3 {for j in 0..3 {assert_eq!(product[i][j],Real::from((0..3).map(|k|a[i][k]*b[k][j]).sum::<i64>()));}}
        assert_eq!(ma.determinant(),Real::from(determinant(&a)));assert_eq!(ma.transpose().transpose(),ma);
        checks+=3;println!("PASS\tmatrix3-{seed}");
        let a=values(4,4,seed);let b=values(4,4,seed+17);
        let ma=Matrix4::new(std::array::from_fn(|i|std::array::from_fn(|j|Real::from(a[i][j]))));
        let mb=Matrix4::new(std::array::from_fn(|i|std::array::from_fn(|j|Real::from(b[i][j]))));
        let product=&ma*&mb;
        for i in 0..4 {for j in 0..4 {assert_eq!(product[i][j],Real::from((0..4).map(|k|a[i][k]*b[k][j]).sum::<i64>()));}}
        assert_eq!(ma.determinant(),Real::from(determinant(&a)));assert_eq!(ma.transpose().transpose(),ma);
        checks+=3;println!("PASS\tmatrix4-{seed}");
    }
    assert_eq!(Matrix3::identity().inverse().unwrap(),Matrix3::identity());checks+=1;
    assert_eq!(Matrix4::identity().inverse().unwrap(),Matrix4::identity());checks+=1;
    assert_eq!(Complex::one().reciprocal().unwrap(),Complex::one());checks+=1;
    assert_eq!(Complex::i().reciprocal().unwrap(),-Complex::i());checks+=1;
    assert_eq!(Complex::i().norm_squared(),Real::from(1));checks+=1;
    for exponent in [17,64,128,512] {
        // Exact rational construction avoids confusing a precision-dependent view with equality.
        let scale=Real::from(2).powi_i64(exponent).unwrap();let tiny=(Real::from(1)/scale).unwrap();
        let a=vec![vec![tiny.clone(),Real::from(0)],vec![Real::from(0),Real::from(1)]];
        let rhs=vec![tiny,Real::from(2)];let solved=solve_dense_linear_system_bareiss(&a,&rhs,-1024,PredicatePolicy::STRICT).unwrap();
        assert_eq!(solved.solution,vec![Real::from(1),Real::from(2)]);checks+=1;println!("PASS\ttiny-pivot-{exponent}");
    }
    assert!(Rational::fraction(1,0).is_err());checks+=1;
    println!("SUMMARY\t{checks}");
}
