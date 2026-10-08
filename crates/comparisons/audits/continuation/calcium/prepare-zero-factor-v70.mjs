import {copyFileSync,constants,mkdirSync,statSync,writeFileSync} from 'node:fs';
import {dirname} from 'node:path';
import assert from 'node:assert/strict';
import {retainedSources,sha,json} from './point-retained-sources-v67.mjs';
const current=retainedSources(),previous=json('power-wide-v69-manifest.json');
for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
const final=json('results/power-wide-verify-canonical-v69.json');assert.equal(final.code,0);assert.equal(final.signal,null);
const source=json('point-demand-source-binding.json'),root='zero-factor-candidate-v70';
assert.equal(Object.keys(source.sources).length,175);mkdirSync(root);
let bytes=0;
for(const[p,h]of Object.entries(source.sources)){
 const dest=root+'/'+p;mkdirSync(dirname(dest),{recursive:true});
 copyFileSync(source.root+'/'+p,dest,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(dest),h,p);bytes+=statSync(dest).size;
}
const o={checkpoint:70,recorded:new Date().toISOString(),root,baseline:source.root,current,
 previousManifestSha256:sha('power-wide-v69-manifest.json'),originalSources:source.sources,copiedFiles:175,copiedBytes:bytes,
 note:'Exclusive solver-only copy of retained baseline, not the power-sum trial. Shared frozen scalar dependencies; no live edits. Implementation follows separately.'};
writeFileSync('zero-factor-origin-v70.json',JSON.stringify(o,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:70,root,files:175,bytes,current}));
