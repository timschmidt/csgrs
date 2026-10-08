import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json} from './e-plan-sources.mjs';
export function checkEPlanRegressions() {
 const runs=[];let names,counts;
 for(const variant of ['baseline','candidate'])for(const profile of ['debug','release']) {
  const tag='e-plan-tests-'+variant+'-'+profile,g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
  const stdout=readFileSync('results/'+tag+'.stdout','utf8'),stderr=readFileSync('results/'+tag+'.stderr','utf8');
  assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(stderr));
  const observed=[...stdout.matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]).sort();
  const suites=[...stdout.matchAll(/^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/gm)].map(m=>m.slice(1).map(Number));
  assert.equal(suites.length,14);assert.equal(observed.length,855);
  assert.deepEqual(suites.reduce((s,r)=>s.map((v,i)=>v+r[i]),[0,0,0,0,0]),[855,0,0,0,0]);
  if(names) {assert.deepEqual(observed,names);assert.deepEqual(suites,counts);}else {names=observed;counts=suites;}
  runs.push({variant,profile,suites:14,passed:855,failed:0,ignored:0,finished:g.finished});
 }
 return{runs,identicalTestMembership:true,scope:'All features, library and integration tests, debug and release. No new default-only, doctest, bench, fuzz, Clippy, WASM, other-target or downstream qualification in this checkpoint.'};
}
