import {copyFileSync,constants,writeFileSync,existsSync} from 'node:fs';
import {sha,json} from './point-image-sources.mjs';
import assert from 'node:assert/strict';
assert(!existsSync('point-image-manifest.json'));assert.equal(json('results/point-image-bind-draft.json').code,1);
const files={};
for(const p of ['point-image-evidence.mjs','record-point-image.mjs','point-image-findings.md']){
 const out=p.replace(/(\.[^.]+)$/,'-bind-draft$1');copyFileSync(p,out,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);files[out]=sha(out);
}
writeFileSync('point-image-binding-draft.json',JSON.stringify({files,
 classification:'Bookkeeping assertion too strong: candidate and guarded collectors have equal errors/lost/reachable/allocation counts, but cumulative requested bytes differ by six. No mathematical oracle or measured output is changed; preserve both exact totals.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({files}));
