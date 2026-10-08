import {readFileSync,writeFileSync,mkdirSync,copyFileSync,constants,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve,dirname} from 'node:path';
import assert from 'node:assert/strict';
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const m=json('mpoly-bridge-experiment.json'),baseline='e-plan-qualified-candidate',candidate='power-sums-candidate';
assert.equal(Object.keys(m.liveSources).length,956);
// Validate every input before creating the isolated, exclusive destination.
for(const[p,h]of Object.entries(m.liveSources)){
 assert.equal(sha(resolve('../../../..',p)),h,p);assert.equal(sha(baseline+'/'+p),h,p);
}
mkdirSync(candidate);let bytes=0;
for(const[p,h]of Object.entries(m.liveSources)){
 const to=candidate+'/'+p;mkdirSync(dirname(to),{recursive:true});
 copyFileSync(baseline+'/'+p,to,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(to),h);bytes+=statSync(to).size;
}
const origin={schema:1,checkpoint:52,recorded:new Date().toISOString(),baseline,candidate,
 sourceManifest:'mpoly-bridge-experiment.json',sourceManifestSha256:sha('mpoly-bridge-experiment.json'),
 liveSources:m.liveSources,copiedFiles:956,copiedBytes:bytes,
 note:'One isolated candidate copied from the current retained frozen baseline. Existing baseline reused. No live edit; candidate algorithm follows separately.'};
writeFileSync('power-sums-origin.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({baseline,candidate,files:956,bytes}));
