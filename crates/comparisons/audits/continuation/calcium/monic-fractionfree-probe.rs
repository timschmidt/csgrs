#[allow(dead_code)]
mod corpus {
    include!("monic-cost-corpus.rs");
    pub fn input(kind: usize, code: usize) -> Vec<hyperreal::Real> { polynomial(kind,code) }
}
use hyperreal::Real;
use hypersolve::subresultant_chain_univariate_polynomials;
fn main() {
    let mut known=0; let mut unknown=0;
    println!("kind,code,floor,expected_gcd_degree,outcome,result_degree");
    for kind in 0..3 { for code in 0..27 { for floor in [-32,-512] {
        let p=corpus::input(kind,code);
        let mut derivative: Vec<_>=p.iter().enumerate().skip(1).map(|(i,c)| c*Real::from(i as u64)).collect();
        if derivative.is_empty() { derivative.push(Real::zero()); }
        let expected=[code%3,(code/3)%3,(code/9)%3].iter().filter(|&&e|e==2).count();
        match subresultant_chain_univariate_polynomials(&p,&derivative,floor) {
            Ok(report)=> {
                assert_eq!(report.last_nonzero_degree,expected,"kind={kind} code={code}");
                assert_eq!(report.has_nonconstant_common_factor,expected>0);
                println!("{kind},{code},{floor},{expected},Known,{}",report.last_nonzero_degree); known+=1;
            }
            Err(_)=> { println!("{kind},{code},{floor},{expected},Unknown,-1"); unknown+=1; }
        }
    } } }
    println!("{{\"suite\":\"monic-fractionfree\",\"cases\":162,\"known\":{known},\"unknown\":{unknown}}}");
}
