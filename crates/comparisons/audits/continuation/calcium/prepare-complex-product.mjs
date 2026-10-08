import { readFileSync, mkdirSync, copyFileSync, constants, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)), workspace=resolve(here,'../../../..');
const m=JSON.parse(readFileSync(resolve(here,'arf-fused-experiment.json'),'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const origin='derivative-demand-candidate', destination='complex-product-candidate';
assert.equal(Object.keys(m.liveSources).length,955);
mkdirSync(resolve(here,destination));
let bytes=0;
for(const [p,h]of Object.entries(m.liveSources)) {
  const source=resolve(here,origin,p), target=resolve(here,destination,p);
  assert.equal(sha(source),h,p);assert.equal(sha(resolve(workspace,p)),h,p);
  mkdirSync(dirname(target),{recursive:true});
  copyFileSync(source,target,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  assert.equal(sha(target),h,p);bytes+=readFileSync(source).length;
}
writeFileSync(resolve(here,'complex-product-origin.json'),JSON.stringify({
  schema:1,recorded:new Date().toISOString(),origin,destination,sourceHashes:m.liveSources,
  copiedFiles:955,copiedBytes:bytes,
  note:'Baseline reuses the retained immutable snapshot. All six candidate crate trees are copied so path dependencies resolve to the candidate Hyperreal. No build artifacts or live-source edits.'
},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({copiedFiles:955,copiedBytes:bytes,origin,destination}));
