use hyperreal::Real;
use hypersolve::{BivariatePolynomial, divide_bivariate_polynomial_exact, divide_univariate_polynomial_exact, greatest_common_divisor_univariate_polynomials_exact};

fn values(n: usize, seed: i64) -> Vec<i64> {
    let mut a:Vec<_>=(0..n).map(|i|((i*17+i*i*3) as i64+seed*13).rem_euclid(11)-5).collect();
    if a[n-1]==0 {a[n-1]=1;} a
}
fn multiply(a:&[i64],b:&[i64])->Vec<i64>{let mut c=vec![0;a.len()+b.len()-1];for(i,&x)in a.iter().enumerate(){for(j,&y)in b.iter().enumerate(){c[i+j]+=x*y;}}c}
fn real(a:&[i64])->Vec<Real>{a.iter().copied().map(Real::from).collect()}
fn trim(mut a:Vec<Real>)->Vec<Real>{while a.len()>1&&a.last()==Some(&Real::zero()){a.pop();}if a.is_empty(){a.push(Real::zero());}a}
fn grid(m:usize,n:usize,seed:i64)->Vec<Vec<i64>>{(0..m).map(|i|values(n,seed+i as i64*7)).collect()}
fn product_grid(a:&[Vec<i64>],b:&[Vec<i64>])->Vec<Vec<i64>> {
    let mut c=vec![vec![0;a[0].len()+b[0].len()-1];a.len()+b.len()-1];
    for(i,row)in a.iter().enumerate(){for(j,&x)in row.iter().enumerate(){for(k,rowb)in b.iter().enumerate(){for(l,&y)in rowb.iter().enumerate(){c[i+k][j+l]+=x*y;}}}}c
}
fn bivariate(a:&[Vec<i64>])->BivariatePolynomial{BivariatePolynomial::new(a.iter().map(|r|real(r)).collect())}
fn matches_grid(got:&BivariatePolynomial,expected:&[Vec<i64>])->bool {
    let m=got.coefficients.len().max(expected.len());
    for i in 0..m {let n=got.coefficients.get(i).map_or(0,Vec::len).max(expected.get(i).map_or(0,Vec::len));for j in 0..n {
        let actual=got.coefficients.get(i).and_then(|r|r.get(j)).cloned().unwrap_or_else(Real::zero);
        let expected=expected.get(i).and_then(|r|r.get(j)).copied().unwrap_or(0);
        if actual!=Real::from(expected){return false;}
    }}true
}
fn main(){
    let mut count=0;
    for n in 1..=12 {for seed in 0..5 {for x in -3..=3 {
        let a=values(n,seed);let expected=a.iter().rev().fold(0_i64,|acc,c|acc*x+c);
        assert_eq!(Real::eval_poly(&real(&a),&Real::from(x)),Real::from(expected));count+=1;
    }}}
    println!("PASS\tevaluation\t{count}");
    let before=count;
    for n in 1..=9 {for m in 1..=5 {for seed in 0..5 {for scale in [-3,1,2] {
        let q=values(n,seed);let mut b=values(m,seed+7);b[m-1]=1;for c in &mut b{*c*=scale;}
        let a=multiply(&q,&b);assert_eq!(divide_univariate_polynomial_exact(&real(&a),&real(&b)),Some(real(&q)));count+=1;
    }}}}
    println!("PASS\texact-division\t{}",count-before);
    let before=count;
    for n in 0..=16 {for m in 0..=n.min(8) {
        let mut a=vec![0;n+1];a[n]=1;let mut b=vec![0;m+1];b[m]=1;let mut q=vec![0;n-m+1];q[n-m]=1;
        assert_eq!(divide_univariate_polynomial_exact(&real(&a),&real(&b)),Some(real(&q)));count+=1;
    }}
    println!("PASS\tmonomial-division\t{}",count-before);
    assert_eq!(Real::eval_poly(&[],&Real::from(5)),Real::zero());count+=1;
    assert_eq!(divide_univariate_polynomial_exact(&real(&[1,0]),&real(&[1])),Some(real(&[1])));count+=1;
    assert_eq!(divide_univariate_polynomial_exact(&real(&[0,0]),&real(&[1])),Some(real(&[0])));count+=1;
    assert!(divide_univariate_polynomial_exact(&real(&[1]),&real(&[0])).is_none());count+=1;
    assert!(divide_univariate_polynomial_exact(&real(&[1,0,1]),&real(&[0,1])).is_none());count+=1;
    assert_eq!(Real::eval_poly(&real(&[1,0,0]),&Real::from(5)),Real::from(1));count+=1;
    let before=count;
    for r in -3..=3 {for s in -3..=3 {
        let common=vec![-r,1];let a=multiply(&common,&[-s,1]);
        let b:Vec<_>=common.iter().map(|c|c*3).collect();
        assert_eq!(greatest_common_divisor_univariate_polynomials_exact(&real(&a),&real(&b)).map(trim),Some(real(&common)));count+=1;
    }}
    println!("PASS\tmonic-gcd\t{}",count-before);
    let before=count;
    for qm in 1..=3 {for qn in 1..=3 {for bm in 1..=3 {for bn in 1..=3 {for seed in 0..3 {
        let q=grid(qm,qn,seed);let b=grid(bm,bn,seed+11);let a=product_grid(&q,&b);
        let got=divide_bivariate_polynomial_exact(&bivariate(&a),&bivariate(&b)).unwrap();assert!(matches_grid(&got,&q));count+=1;
    }}}}}
    println!("PASS\tbivariate-division\t{}",count-before);
    assert!(divide_bivariate_polynomial_exact(&bivariate(&[vec![1]]),&bivariate(&[vec![0]])).is_none());count+=1;
    assert!(matches_grid(&divide_bivariate_polynomial_exact(&bivariate(&[vec![0]]),&bivariate(&[vec![1]])).unwrap(),&[vec![0]]));count+=1;
    for d in [2,3,5,7] {
        let root=Real::from(d).sqrt().unwrap();assert_eq!(Real::eval_poly(&real(&[-d,0,1]),&root),Real::zero());count+=1;
        let inverse=(Real::from(1)/&root).unwrap();assert_eq!(&root*&inverse,Real::from(1));count+=1;
    }
    println!("SUMMARY\t{count}");
}
