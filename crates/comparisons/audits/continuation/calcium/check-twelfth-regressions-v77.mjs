import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {json} from './zero-factor-retained-sources-v75.mjs';
export function regressions(){
 const results=[],allNames={};
 for(const features of ['default','all'])for(const profile of ['debug','release']){
  const observed={};
  for(const variant of ['baseline','candidate']){
   const tag='twelfth-'+variant+'-'+features+'-'+profile+'-final-v77',g=json('results/'+tag+'.json');
   assert.equal(g.code,0);assert.equal(g.signal,null);
   const stdout=readFileSync('results/'+tag+'.stdout','utf8'),stderr=readFileSync('results/'+tag+'.stderr','utf8');
   assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(stderr));
   const names=[...stdout.matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]).sort();
   const suites=[...stdout.matchAll(/^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/gm)].map(m=>m.slice(1).map(Number));
   assert.equal(suites.length,features==='all'?14:13);assert(names.length>750);
   const counts=suites.reduce((s,r)=>s.map((v,i)=>v+r[i]),[0,0,0,0,0]);assert.deepEqual(counts,[names.length,0,0,0,0]);
   observed[variant]=names;
   const key=variant+':'+features;if(key in allNames)assert.deepEqual(names,allNames[key]);else allNames[key]=names;
   results.push({tag,variant,features,profile,suites:suites.length,passed:names.length,failed:0,ignored:0,finished:g.finished});
  }
  const added=observed.candidate.filter(n=>n.startsWith('computable::node::twelfth_relation_tests::'));
  assert.equal(added.length,features==='all'?8:7);
  assert.deepEqual(observed.candidate.filter(n=>!added.includes(n)),observed.baseline);
 }
 return {checkpoint:77,results,baselineMembershipPreserved:true,defaultAddedTests:7,allFeatureAddedTests:8,
  scope:'Matched baseline/candidate library and integration tests with default/all features in debug/release. Same original test sources; no failed or ignored tests. Bench, WASM, downstream, fuzz and full CI qualification remain separate.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(regressions()));
