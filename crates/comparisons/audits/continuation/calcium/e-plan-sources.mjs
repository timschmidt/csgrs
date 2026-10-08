import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
export {sha,json};
export function sources() {
 const o=json('e-plan-origin.json'),previous=json('mag-series-experiment.json'),candidate={},changed=[],files={};
 assert.deepEqual(o.liveSources,previous.liveSources);assert.equal(Object.keys(o.liveSources).length,955);
 for(const[p,h]of Object.entries(o.liveSources)) {
  assert.equal(sha(resolve('../../../..',p)),h,p);
  if(!p.startsWith('hyperreal/'))continue;
  assert.equal(sha(o.baseline+'/'+p),h,p);const ch=sha(o.candidate+'/'+p);candidate[p]=ch;if(ch!==h)changed.push(p);
 }
 assert.equal(Object.keys(candidate).length,180);assert.deepEqual(changed,['hyperreal/src/computable/approximation/constants.rs']);
 const path=changed[0],read=p=>readFileSync(p,'utf8'),a=read(o.baseline+'/'+path),b=read(o.candidate+'/'+path);
 const start=s=>s.indexOf('fn e_terms_for_precision('),end=s=>s.indexOf('// Returns (P, Q)');
 assert(start(a)>0&&end(a)>start(a));assert.equal(a.slice(0,start(a)),b.slice(0,start(b)));assert.equal(a.slice(end(a)),b.slice(end(b)));
 for(const v of ['baseline','candidate']) {
  const original=v==='baseline'?a:b;
  assert.equal(read('e-plan-app-'+v+'/kernel.rs'),'use num::{BigInt, BigUint, One};\ntype Precision = i32;\n'+original.slice(start(original))
   .replace('fn e_terms_for_precision(','pub fn e_terms_for_precision(').replace('fn e(p:','pub fn e(p:'));
 }
 let invariant=b.slice(start(b),end(b)).trimEnd().replace('fn e_terms_for_precision(','pub fn check_lower_invariant(')
  .replace('    loop {','    let mut witness = rug::Integer::from(1);\n    loop {')
  .replace('        let bits =','        witness *= n + 1;\n        let mut lower = rug::Integer::from(next);\n        lower <<= u32::try_from(shift).unwrap();\n        assert!(lower <= witness);\n        assert!(mantissa > 0);\n        let bits =');
 assert.equal(read('e-plan-invariant.rs'),'type Precision = i32;\n'+invariant+'\n');
 for(const p of ['prepare-e-plan.mjs','prepare-e-plan-apps.mjs','e-plan-origin.json','e-plan-proof.md','e-plan-check.rs',
  'e-plan-invariant.rs','e-plan-cost.rs','e-plan-cpu.rs','e-plan-allocation.rs','e-plan-sources.mjs','run-e-plan-costs.mjs',
  ...['baseline','candidate'].flatMap(v=>['Cargo.toml','Cargo.lock','kernel.rs'].map(p=>'e-plan-app-'+v+'/'+p))])files[p]=sha(p);
 return{live:o.liveSources,candidate,changed,files};
}
