import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-sources.mjs';
export {sha,json};
export function sources(checkLive=true) {
 const o=json('e-plan-qualified-origin.json'),m=json('e-plan-experiment.json'),candidate={},changed=[];
 assert.equal(sha(o.predecessor),o.predecessorSha256);assert.deepEqual(o.liveSources,m.sourceMap.live);
 for(const[p,h]of Object.entries(o.liveSources)) {
  assert.equal(sha(o.baseline+'/'+p),h,p);if(checkLive)assert.equal(sha(resolve('../../../..',p)),h,p);
  const ch=sha(o.candidate+'/'+p);candidate[p]=ch;if(ch!==h)changed.push(p);
 }
 assert.deepEqual(changed,['hyperreal/src/computable/approximation.rs','hyperreal/src/computable/approximation/constants.rs']);
 const tests='hyperreal/src/computable/approximation/e_plan_tests.rs';candidate[tests]=sha(o.candidate+'/'+tests);
 assert.equal(candidate[changed[1]],m.sourceMap.candidate[changed[1]]);
 const a=readFileSync(o.baseline+'/'+changed[0],'utf8'),b=readFileSync(o.candidate+'/'+changed[0],'utf8');
 assert.equal(b,a.replace('include!("approximation/statistics.rs");\n','include!("approximation/statistics.rs");\n\n#[cfg(test)]\n#[path = "approximation/e_plan_tests.rs"]\nmod e_plan_tests;\n'));
 return{baseline:o.liveSources,candidate,changed,added:[tests]};
}
