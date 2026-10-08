import {mkdirSync,copyFileSync,constants,statSync,writeFileSync} from 'node:fs';
import {resolve,dirname} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-sources.mjs';
const m=json('e-plan-experiment.json'),baseline='derivative-demand-candidate',candidate='e-plan-qualified-candidate';
mkdirSync(candidate);let bytes=0;const copied={};
for(const[p,h]of Object.entries(m.sourceMap.live)) {
 assert.equal(sha(resolve('../../../..',p)),h,p);assert.equal(sha(baseline+'/'+p),h,p);
 const from=p.startsWith('hyperreal/')?'e-plan-candidate/'+p:baseline+'/'+p;
 const expected=m.sourceMap.candidate[p]??h;assert.equal(sha(from),expected,p);
 const to=candidate+'/'+p;mkdirSync(dirname(to),{recursive:true});copyFileSync(from,to,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(to),expected);bytes+=statSync(to).size;copied[p]=expected;
}
assert.equal(Object.keys(copied).length,955);
writeFileSync('e-plan-qualified-origin.json',JSON.stringify({schema:1,recorded:new Date().toISOString(),baseline,candidate,
 predecessor:'e-plan-experiment.json',predecessorSha256:sha('e-plan-experiment.json'),liveSources:m.sourceMap.live,
 copiedSources:copied,copiedFiles:955,copiedBytes:bytes,
 note:'Reuses baseline; copies only one qualified candidate tree, preserving checkpoint42. Durable regression additions follow separately. No live edit.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({baseline,candidate,files:955,bytes}));
