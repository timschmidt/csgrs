from pathlib import Path
from fractions import Fraction as Q
import json
root=Path('/tmp/hypercurve-endpoint-sign-2026-09-23');repo=root/'hypersolve';audit=Path(__file__).resolve().parent
def real(v):
 if ' ' in v:
  w,p=v.split(' ');value=Q(w)+(1 if not w.startswith('-') else -1)*Q(p)
 else:value=Q(v)
 return f'{value.numerator}/{value.denominator}'
rows=[]
for i in range(2):
 d=json.loads((root/f'dumps/sign-{i}.json').read_text());rows.append({k:list(map(real,v)) for k,v in d.items()})
(repo/'examples/endpoint_sign_probe_20260923.json').write_text(json.dumps(rows,indent=2)+'\n')
(repo/'examples/endpoint_sign_probe_20260923.rs').write_text('''use hyperreal::{Real,Rational};
use hypersolve::{IsolatedRootInterval,sign_at_selected_root};
fn main() {
    let data:serde_json::Value=serde_json::from_str(include_str!("endpoint_sign_probe_20260923.json")).unwrap();
    let real=|value:&serde_json::Value|Real::new(value.as_str().unwrap().parse::<Rational>().unwrap());
    for (id,row) in data.as_array().unwrap().iter().enumerate() {
        let defining=row["base"].as_array().unwrap().iter().map(&real).collect::<Vec<_>>();
        let predicate=row["predicate"].as_array().unwrap().iter().map(&real).collect::<Vec<_>>();
        let interval=IsolatedRootInterval { lower:real(&row["interval"][0]),upper:real(&row["interval"][1]),exact_root:None,distinct_root_count:1 };
        eprintln!("begin query {id}");let start=std::time::Instant::now();
        let sign=sign_at_selected_root(&defining,&predicate,&interval);
        eprintln!("query {id}: {sign:?}, elapsed {:?}",start.elapsed());assert!(sign.is_some());
    }
}
''')
s=(audit/'run-contact-gcd-probe-20260923.py').read_text().replace('/tmp/hypercurve-gcd-replay-2026-09-23',str(root)).replace('contact-gcd-20260923-','endpoint-sign-20260923-').replace('contact_gcd_probe_20260923','endpoint_sign_probe_20260923')
(audit/'run-endpoint-sign-probe-20260923.py').write_text(s)
