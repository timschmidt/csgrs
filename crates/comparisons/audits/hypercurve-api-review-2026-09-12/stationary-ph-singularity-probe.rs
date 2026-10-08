use hypercurve::*;
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn binomial(n:usize,k:usize)->i32 {(0..k).fold(1,|c,i|c*(n-i)/(i+1)) as i32}
fn bernstein(c:&[Real])->Vec<Real>{let n=c.len()-1;(0..=n).map(|i|(0..=i).fold(Real::zero(),|v,k|v+&c[k]*q(binomial(i,k),binomial(n,k)))).collect()}
fn main(){
        let sources = [
            (
                vec![q(0, 1), q(0, 1), q(0, 1), q(1, 3), q(0, 1), q(-1, 5)],
                vec![q(0, 1), q(0, 1), q(0, 1), q(0, 1), q(1, 2), q(0, 1)],
                1,
            ),
            (
                vec![q(0, 1), q(-1, 1), q(1, 1), q(1, 3), q(-1, 2)],
                vec![q(0, 1), q(0, 1), q(-1, 1), q(4, 3), q(0, 1)],
                2,
            ),
            (
                vec![q(0, 1), q(-1, 1), q(0, 1), q(1, 1), q(0, 1), q(-2, 5)],
                vec![q(0, 1), q(0, 1), q(-1, 1), q(0, 1), q(1, 1), q(0, 1)],
                2,
            ),
        ];

for (index,(x,y,_)) in sources.into_iter().enumerate(){
let degree=x.len()-1;
let source=RationalBezier2::try_new(bernstein(&x).into_iter().zip(bernstein(&y)).map(|(x,y)|Point2::new(x,y)).collect(),vec![Real::one();degree+1]).unwrap();
for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{for reversed in [false,true]{let source=if reversed{source.reversed()}else{source.clone()};for distance in [q(-1,20),q(1,20)]{
let parallel=source.parallel_left(distance.clone()).unwrap();
println!("source {index}, {policy:?}, reversed {reversed}, distance {distance:?}: {:?}",parallel.singularity_analysis(&policy).map(|v|v.map(|a|(a.source_singularities().len(),a.parallel_cusps().len()))));
}}}}
}