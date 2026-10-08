import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {retainedSources,sha,json} from './point-retained-sources-v67.mjs';
export {sha,json};
export const root='power-rebased-v68',binding='power-rebased-fixed-binding-v68.json';
export function fixedSources(){
 const original=json('power-rebased-origin-v68.json'),initial=json('power-rebased-binding-v68.json'),current=retainedSources();
 assert.deepEqual(current,initial.current);assert.deepEqual(current,original.current);
 assert.equal(sha('power-sums-manifest.json'),original.originalManifestSha256);
 assert.equal(sha('point-retained-v67-manifest.json'),original.previousManifestSha256);
 const file=initial.changed[0];assert.deepEqual(initial.changed,['hypersolve/src/algebraic_binary.rs']);
 assert.equal(sha('power-rebased-initial-v68.rs'),initial.source[file]);
 for(const[p,h]of Object.entries(original.originalSources))assert.equal(sha(initial.baseline+'/'+p),h,p);
 for(const[p,h]of Object.entries(original.originalPowerSources))assert.equal(sha('power-sums-candidate/'+p),h,p);
 const source={};for(const[p,h]of Object.entries(initial.source)){
  source[p]=sha(root+'/'+p);if(p!==file)assert.equal(source[p],h,p);
 }
 let expected=readFileSync('power-rebased-initial-v68.rs','utf8');
 for(const [from,to,count]of [['compose_with_signed_unit_linear(&right_polynomial,','compose_with_signed_unit_linear(right_polynomial,',2],
  ['reciprocal_product_polynomial(&right_polynomial,','reciprocal_product_polynomial(right_polynomial,',1],
  ['quotient_product_polynomial(&right_polynomial,','quotient_product_polynomial(right_polynomial,',1],
  ['resultant_exact_rational_polynomials_value(&left_polynomial,','resultant_exact_rational_polynomials_value(left_polynomial,',1]]){
  assert.equal(expected.split(from).length-1,count);expected=expected.replaceAll(from,to);
 }
 assert.equal(readFileSync(root+'/'+file,'utf8'),expected);
 // Reconstruct the initial composition against the two immutable parents.
 const prior=readFileSync(initial.baseline+'/'+file,'utf8'),first=readFileSync('power-rebased-initial-v68.rs','utf8'),
  power=readFileSync('power-sums-candidate/'+file,'utf8'),start='fn resultant_polynomial_for_binary_image(',end='fn binary_image_interval(';
 assert.equal(first.slice(0,first.indexOf(start)).replace('mod power_sums;\n\n',''),prior.slice(0,prior.indexOf(start)));
 assert.equal(first.slice(first.indexOf(end)),prior.slice(prior.indexOf(end)));
 assert.equal(first.slice(first.indexOf(start),first.indexOf(end)),power.slice(power.indexOf(start),power.indexOf(end)));
 for(const[p,h]of Object.entries(initial.harness))assert.equal(sha(p),h,p);
 const g=json('results/power-rebased-gates-v68.json'),lint=json('results/power-rebased-clippy-v68.json');
 assert.equal(g.code,1);assert.equal(g.signal,null);assert.equal(lint.code,101);assert.equal(lint.signal,null);
 const errors=readFileSync('results/power-rebased-clippy-v68.stderr','utf8');
 assert.equal((errors.match(/error: this expression creates a reference/g)??[]).length,5);
 return {...initial,source,initialBindingSha256:sha('power-rebased-binding-v68.json'),
  initialArchiveSha256:sha('power-rebased-initial-v68.rs'),initialFailureSha256:sha('results/power-rebased-gates-v68.json'),
  amendment:'Remove exactly five redundant references in extracted sampled fallback; unchanged algorithm/public contracts. Initial failed lint/source/binaries/captures preserved.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const s=fixedSources();if(process.argv.includes('--record'))writeFileSync(binding,JSON.stringify(s,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(s,json(binding));
 console.log(JSON.stringify({checkpoint:68,amended:true,files:Object.keys(s.source).length,changed:s.changed,currentLiveFiles:956}));
}
