import {readFileSync,writeFileSync,mkdirSync,copyFileSync,constants,statSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './point-image-sources.mjs';
const prior=json('point-image-manifest.json'),live=json('mpoly-bridge-experiment.json');
const baseline='e-plan-qualified-candidate',candidate='point-image-qualified-candidate',changed='hypersolve/src/algebraic_binary.rs';
assert.equal(Object.keys(live.liveSources).length,956);
for(const[p,h]of Object.entries(live.liveSources)){
 assert.equal(sha(resolve('../../../..',p)),h,p);assert.equal(sha(baseline+'/'+p),h,p);
}
for(const[p,h]of Object.entries(prior.guardSources))assert.equal(sha('point-image-guard/'+p),h,p);
mkdirSync(candidate);let bytes=0;const sources={};
for(const[p,h]of Object.entries(live.liveSources)){
 const from=p===changed?'point-image-guard/'+p:baseline+'/'+p,to=candidate+'/'+p;
 mkdirSync(dirname(to),{recursive:true});copyFileSync(from,to,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 sources[p]=sha(to);assert.equal(sources[p],p===changed?prior.guardSources[p]:h,p);bytes+=statSync(to).size;
}
const result={schema:1,checkpoint:54,recorded:new Date().toISOString(),baseline,candidate,liveSources:live.liveSources,candidateSources:sources,
 predecessor:'point-image-manifest.json',predecessorSha256:sha('point-image-manifest.json'),changed:[changed],copiedFiles:956,copiedBytes:bytes,
 note:'One complete isolated consumer/application tree, copied from the retained baseline except the byte-identical guarded solver file. Original manifests and all scalar/consumer sources are unchanged. Old candidate/source evidence and executables remain intact.'};
writeFileSync('point-qualified-origin.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({baseline,candidate,files:956,bytes,changed:result.changed}));
