import {readFileSync,mkdirSync,copyFileSync,constants,writeFileSync} from 'node:fs';
import {resolve,dirname} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
const m=json('mag-series-experiment.json'),baseline='derivative-demand-candidate',candidate='e-plan-candidate';
mkdirSync(candidate);let bytes=0;
const selected=Object.entries(m.liveSources).filter(([p])=>p.startsWith('hyperreal/'));assert.equal(selected.length,180);
for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve('../../../..',p)),h,p);
for(const[p,h]of selected){
 const source=resolve(baseline,p),dest=resolve(candidate,p);assert.equal(sha(source),h,p);
 mkdirSync(dirname(dest),{recursive:true});copyFileSync(source,dest,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(dest),h,p);bytes+=readFileSync(dest).length;
}
writeFileSync('e-plan-origin.json',JSON.stringify({schema:1,recorded:new Date().toISOString(),baseline,candidate,
 liveSources:m.liveSources,copiedFiles:selected.length,copiedBytes:bytes,
 note:'Only the180-file Hyperreal tree is copied; its external dependencies come from the existing offline registry/build cache. No live/donor modification. Baseline reuses the frozen retained snapshot.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({copiedFiles:selected.length,copiedBytes:bytes,baseline,candidate}));
