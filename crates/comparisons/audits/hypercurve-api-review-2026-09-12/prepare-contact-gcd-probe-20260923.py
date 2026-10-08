from pathlib import Path
from fractions import Fraction
import json
root=Path('/tmp/hypercurve-gcd-replay-2026-09-23'); repo=root/'hypersolve'; audit=Path(__file__).resolve().parent
def mixed(v):
    if ' ' not in v: return Fraction(v)
    w,p=v.split(' ');return Fraction(w)+(1 if not w.startswith('-') else -1)*Fraction(p)
def convert(o):
    if isinstance(o,str):
        f=mixed(o);return str(f.numerator)+'/'+str(f.denominator)
    if isinstance(o,list):return [convert(x) for x in o]
    if isinstance(o,dict):return {k:convert(v) for k,v in o.items()}
    return o
data=json.loads((root/'dumps/gcd-14.json').read_text()); assert not data['denominators'] and not data['point_witness']
(repo/'examples/contact_gcd_probe_20260923.json').write_text(json.dumps(convert(data),indent=2)+'\n')
(repo/'examples/contact_gcd_probe_20260923.rs').write_text('''use hyperreal::{Rational, Real};
use hyperlimit::PredicatePolicy;
use hypersolve::*;
fn main() {
    let data: serde_json::Value = serde_json::from_str(include_str!("contact_gcd_probe_20260923.json")).unwrap();
    let real = |v: &serde_json::Value| Real::new(v.as_str().unwrap().parse::<Rational>().unwrap());
    let mut root = AlgebraicRootRepresentation {
        constraint_index: 0, symbol: SymbolId(0), interval_index: 0,
        polynomial_coefficients: data["base"].as_array().unwrap().iter().map(&real).collect(),
        interval: IsolatedRootInterval { lower: real(&data["base_interval"][0]), upper: real(&data["base_interval"][1]), exact_root: None, distinct_root_count: 1 },
        validation: AlgebraicRootValidationReport { status: AlgebraicRootValidationStatus::Valid, message: None },
    };
    root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT); assert!(root.is_valid());
    let polynomial = |key: &str| {
        let rows=data[key].as_array().unwrap();
        let mut coefficients=vec![vec![Real::zero();rows.len()];rows.iter().map(|r|r.as_array().unwrap().len()).max().unwrap()];
        for (fiber,row) in rows.iter().enumerate() { for (base,v) in row.as_array().unwrap().iter().enumerate() { coefficients[base][fiber]=real(v); } }
        BivariatePolynomial::new(coefficients)
    };
    eprintln!("begin common-root replay"); let start=std::time::Instant::now();
    let report=count_bivariate_common_fiber_roots_at_algebraic_parameter(&polynomial("first"),&polynomial("second"),CurveResultantParameter::First,&root,&real(&data["fiber_interval"][0]),&real(&data["fiber_interval"][1]),PredicatePolicy::STRICT);
    eprintln!("status={:?} count={:?} elapsed={:?}",report.status,report.distinct_root_count,start.elapsed());
}
''')
s=(audit/'run-contact-fiber-probe-20260923.py').read_text().replace('/tmp/hypercurve-contact-isolation-2026-09-23',str(root)).replace('contact-fiber-20260923-','contact-gcd-20260923-').replace('contact_fiber_probe_20260923','contact_gcd_probe_20260923')
(audit/'run-contact-gcd-probe-20260923.py').write_text(s)
print('Captured distinct degree-8/degree-6 inputs in the selected degree-41 field.')
