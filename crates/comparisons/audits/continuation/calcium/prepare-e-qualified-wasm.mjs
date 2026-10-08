import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {sources,json} from './e-plan-qualified-sources.mjs';
sources();const o=json('e-plan-qualified-origin.json'),added=[];
for(const variant of ['baseline','candidate']) {
 const root=variant==='baseline'?o.baseline:o.candidate;
 const s=readFileSync(root+'/hyperreal/src/computable/approximation/constants.rs','utf8');
 const kernel='use num::{BigInt, BigUint, One};\ntype Precision = i32;\n'+s.slice(s.indexOf('fn e_terms_for_precision('))
  .replace('fn e_terms_for_precision(','pub fn e_terms_for_precision(').replace('fn e(p:','pub fn e(p:');
 const app='e-qualified-wasm-'+variant;
 added.push([app+'/kernel.rs',kernel]);
 added.push([app+'/Cargo.toml',`[package]
name = "calcium-e-qualified-wasm-${variant}"
version = "0.0.0"
edition = "2024"
[workspace]
[dependencies]
hyperreal = { path = "../${root}/hyperreal", features = ["serde"] }
num = "0.4.3"
[lib]
crate-type = ["cdylib"]
path = "../e-qualified-wasm.rs"
`]);
}
console.log('*** Begin Patch\n'+added.map(([p,s])=>'*** Add File: '+resolve(p)+'\n'+s.trimEnd().split('\n').map(l=>'+'+l).join('\n')).join('\n')+'\n*** End Patch');
