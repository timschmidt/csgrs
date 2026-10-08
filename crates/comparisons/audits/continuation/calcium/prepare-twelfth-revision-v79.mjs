import {copyFileSync,constants,mkdirSync,statSync,writeFileSync,readFileSync} from 'node:fs';
import {dirname} from 'node:path';
import assert from 'node:assert/strict';
import {retainedSources,sha,json} from './zero-factor-retained-sources-v75.mjs';
const current=retainedSources(),prior=json('twelfth-cost-v78-manifest.json');
for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
const gate=json('results/twelfth-current78-before-v79.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);
const rows=readFileSync('results/twelfth-current78-before-v79.stdout','utf8').trim().split('\n');assert.equal(rows.length,1);
assert.equal(JSON.parse(rows[0]).status,'verified-current-version-not-selected');
assert.equal(readFileSync('results/twelfth-current78-before-v79.stderr').length,0);
const root='twelfth-revision-candidate-v79',sources=prior.candidateSources;
assert.equal(Object.keys(sources).length,183);mkdirSync(root);let bytes=0;
for(const[p,h]of Object.entries(sources)){
 const src=prior.candidateRoot+'/'+p,dest=root+'/'+p;assert.equal(sha(src),h,p);
 mkdirSync(dirname(dest),{recursive:true});copyFileSync(src,dest,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(dest),h,p);bytes+=statSync(dest).size;
}
const origin={checkpoint:79,recorded:new Date().toISOString(),root,current,originalRoot:prior.candidateRoot,originalSources:sources,
 previousManifestSha256:sha('twelfth-cost-v78-manifest.json'),prepareSha256:sha('prepare-twelfth-revision-v79.mjs'),copiedFiles:183,copiedBytes:bytes,
 note:'Exclusive frozen-candidate Hyperreal-only copy. No live, prior-candidate or donor mutation; no retention claim.'};
writeFileSync('twelfth-revision-origin-v79.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:79,status:'prepared',root,copiedFiles:183,copiedBytes:bytes}));
