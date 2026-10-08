import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
const o=json('e-plan-origin.json'),path='hyperreal/src/computable/approximation/constants.rs';
assert.equal(sha(o.baseline+'/'+path),o.liveSources[path]);
const original=readFileSync(o.baseline+'/'+path,'utf8'),candidate=readFileSync(o.candidate+'/'+path,'utf8');
const start=s=>s.indexOf('fn e_terms_for_precision('),end=s=>s.indexOf('// Returns (P, Q)');
assert(start(original)>0&&start(candidate)>0);
assert.equal(original.slice(0,start(original)),candidate.slice(0,start(candidate)));
assert.equal(original.slice(end(original)),candidate.slice(end(candidate)));
const additions=[];
for(const variant of ['baseline','candidate']){
 const root=variant==='baseline'?o.baseline:o.candidate;
 const s=readFileSync(root+'/'+path,'utf8').slice(start(original));
 const kernel='use num::{BigInt, BigUint, One};\ntype Precision = i32;\n'+s.replace('fn e_terms_for_precision(','pub fn e_terms_for_precision(').replace('fn e(p:','pub fn e(p:');
 additions.push(['e-plan-app-'+variant+'/kernel.rs',kernel]);
 additions.push(['e-plan-app-'+variant+'/Cargo.toml',`[package]
name = "calcium-e-plan-${variant}"
version = "0.0.0"
edition = "2024"
[workspace]
[dependencies]
hyperreal = { path = "../${root}/hyperreal", features = ["serde"] }
num = "0.4.3"
rug = { version = "1.30.0", default-features = false, features = ["integer", "float", "std"] }
serde_json = "1.0.149"
[[bin]]
name = "e-plan-${variant}-check"
path = "../e-plan-check.rs"
[[bin]]
name = "e-plan-${variant}-cpu"
path = "../e-plan-cpu.rs"
[[bin]]
name = "e-plan-${variant}-allocation"
path = "../e-plan-allocation.rs"
`]);
}
let invariant=candidate.slice(start(candidate),end(candidate)).trimEnd();
invariant=invariant.replace('fn e_terms_for_precision(','pub fn check_lower_invariant(')
 .replace('    loop {','    let mut witness = rug::Integer::from(1);\n    loop {')
 .replace('        let bits =',`        witness *= n + 1;
        let mut lower = rug::Integer::from(next);
        lower <<= u32::try_from(shift).unwrap();
        assert!(lower <= witness);
        assert!(mantissa > 0);
        let bits =`);
additions.push(['e-plan-invariant.rs','type Precision = i32;\n'+invariant+'\n']);
const allocation=readFileSync('complex-product-allocation.rs','utf8');
const stop=allocation.indexOf('include!("complex-product-corpus.rs");');assert(stop>0);
additions.push(['e-plan-allocation.rs',allocation.slice(0,stop)+'include!("e-plan-cost.rs");\nfn main() { run(Some(allocation::snapshot)); }\n']);
console.log('*** Begin Patch\n'+additions.map(([p,s])=>'*** Add File: '+resolve(p)+'\n'+s.trimEnd().split('\n').map(l=>'+'+l).join('\n')).join('\n')+'\n*** End Patch');
