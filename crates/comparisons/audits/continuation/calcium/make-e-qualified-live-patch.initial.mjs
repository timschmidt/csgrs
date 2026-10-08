import {readFileSync,existsSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,json} from './e-plan-qualified-sources.mjs';
const sourceMap=sources(),o=json('e-plan-qualified-origin.json'),draft=json('e-qualified-experiment-draft.json');
assert(draft.draft);assert.deepEqual(draft.sourceMap,sourceMap);assert.equal(json('results/e-qualified-verify-draft.json').code,0);
const read=p=>readFileSync(p,'utf8');let patch='*** Begin Patch\n';
const constants='hyperreal/src/computable/approximation/constants.rs';
const a=read(o.baseline+'/'+constants),b=read(o.candidate+'/'+constants),start=s=>s.indexOf('fn e_terms_for_precision('),end=s=>s.indexOf('// Returns (P, Q)');
assert.equal(a.slice(0,start(a)),b.slice(0,start(b)));assert.equal(a.slice(end(a)),b.slice(end(b)));
patch+='*** Update File: '+resolve('../../../..',constants)+'\n@@\n'+a.slice(start(a),end(a)).trimEnd().split('\n').map(l=>'-'+l).join('\n')+'\n'+
 b.slice(start(b),end(b)).trimEnd().split('\n').map(l=>'+'+l).join('\n')+'\n';
patch+='*** Update File: '+resolve('../../../../hyperreal/src/computable/approximation.rs')+'\n@@\n include!("approximation/statistics.rs");\n \n+#[cfg(test)]\n+#[path = "approximation/e_plan_tests.rs"]\n+mod e_plan_tests;\n+\n #[cfg(test)]\n mod chudnovsky_pi_tests {\n';
const tests=sourceMap.added[0],target=resolve('../../../..',tests);assert(!existsSync(target));
patch+='*** Add File: '+target+'\n'+read(o.candidate+'/'+tests).trimEnd().split('\n').map(l=>'+'+l).join('\n')+'\n*** End Patch\n';
console.log(patch);
