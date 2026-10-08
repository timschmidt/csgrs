import {copyFileSync,constants,mkdirSync,statSync,writeFileSync} from 'node:fs';
import {dirname} from 'node:path';
import assert from 'node:assert/strict';
import {retainedSources,sha,json,workspace} from './zero-factor-retained-sources-v75.mjs';
const current=retainedSources(),previous=json('qqbar-trig-v76-manifest.json');
for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
const sources=Object.fromEntries(Object.entries(previous.liveSources).filter(([p])=>p.startsWith('hyperreal/')));
assert.equal(Object.keys(sources).length,181);
const root='twelfth-relation-candidate-v77';mkdirSync(root);
let bytes=0;
for(const[p,h]of Object.entries(sources)){
 assert.equal(sha(workspace+'/'+p),h,p);
 const dest=root+'/'+p;mkdirSync(dirname(dest),{recursive:true});
 copyFileSync(workspace+'/'+p,dest,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(dest),h,p);bytes+=statSync(dest).size;
}
const origin={checkpoint:77,recorded:new Date().toISOString(),root,current,baseline:workspace,
 previousManifestSha256:sha('qqbar-trig-v76-manifest.json'),prepareSha256:sha('prepare-twelfth-relation-v77.mjs'),
 originalSources:sources,copiedFiles:181,copiedBytes:bytes,
 note:'Exclusive Hyperreal-only isolated copy; no live or historical candidate mutation. Candidate patch and qualification follow separately.'};
writeFileSync('twelfth-relation-origin-v77.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:77,root,files:181,bytes,current}));
