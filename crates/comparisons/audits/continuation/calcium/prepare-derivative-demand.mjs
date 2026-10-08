import { readFileSync, mkdirSync, copyFileSync, symlinkSync, constants, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const retained=JSON.parse(readFileSync(resolve(here,'retained-monic.json'),'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const frozen=resolve(here,retained.frozenSnapshot),copied={};
for(const variant of ['baseline','candidate']) {
  const destination=resolve(here,`derivative-demand-${variant}`);mkdirSync(destination);
  for(const name of ['hyperreal','hyperlattice','hyperlimit','hypersolve','hypertri'])
    symlinkSync(resolve(frozen,name),resolve(destination,name),'dir');
  let count=0,bytes=0;
  for(const [p,h]of Object.entries(retained.liveSources)) {
    const source=resolve(frozen,p);assert.equal(sha(source),h,p);
    if(!p.startsWith('hypercurve/'))continue;
    const target=resolve(destination,p);mkdirSync(dirname(target),{recursive:true});
    copyFileSync(source,target,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);assert.equal(sha(target),h);
    count++;bytes+=readFileSync(source).length;
  }
  assert.equal(count,354);copied[variant]={count,bytes};
}
writeFileSync(resolve(here,'derivative-demand-origin.json'),JSON.stringify({
  schema:1,recorded:new Date().toISOString(),origin:retained.frozenSnapshot,sourceHashes:retained.liveSources,copied,
  note:'Only Hypercurve is copied; five unchanged dependencies point to immutable qualified snapshots. No live source change. Subsequent same regression additions in both variants and candidate kernel change must be bound independently.'
},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify(copied));
