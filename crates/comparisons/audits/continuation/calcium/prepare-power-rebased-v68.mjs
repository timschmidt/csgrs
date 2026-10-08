import {copyFileSync,constants,mkdirSync,statSync,writeFileSync,readFileSync} from 'node:fs';
import {dirname} from 'node:path';
import assert from 'node:assert/strict';
import {retainedSources,sha,json} from './point-retained-sources-v67.mjs';
const current=retainedSources(),previous=json('point-retained-v67-manifest.json');
for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
const final=json('results/point-retained-verify-v67.json');assert.equal(final.code,0);assert.equal(final.signal,null);
assert.equal(readFileSync('results/point-retained-verify-v67.stderr').length,0);
const original=json('power-sums-manifest.json');
for(const[p,h]of Object.entries(original.files))assert.equal(sha(p),h,p);
for(const[p,h]of Object.entries(original.candidateSources))assert.equal(sha('power-sums-candidate/'+p),h,p);
const source=json('point-demand-source-binding.json'),root='power-rebased-v68';
assert.equal(Object.keys(source.sources).length,175);mkdirSync(root);
let bytes=0;
for(const[p,h]of Object.entries(source.sources)){
 const dest=root+'/'+p;mkdirSync(dirname(dest),{recursive:true});
 copyFileSync(source.root+'/'+p,dest,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(dest),h,p);bytes+=statSync(dest).size;
}
const o={checkpoint:68,recorded:new Date().toISOString(),root,baseline:source.root,
 current,previousManifestSha256:sha('point-retained-v67-manifest.json'),
 originalManifestSha256:sha('power-sums-manifest.json'),originalSources:source.sources,
 originalPowerSources:original.candidateSources,copiedFiles:175,copiedBytes:bytes,
 note:'Exclusive solver-only copy. Baseline already contains the retained witness repair; original power-sum candidate and all live sources remain untouched. Algorithm patch follows separately.'};
writeFileSync('power-rebased-origin-v68.json',JSON.stringify(o,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:68,root,baseline:source.root,files:175,bytes,current}));
