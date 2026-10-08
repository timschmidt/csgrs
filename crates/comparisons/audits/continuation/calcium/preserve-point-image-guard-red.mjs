import {copyFileSync,constants,readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
assert.equal(JSON.parse(readFileSync('results/point-image-guard-red.json')).code,101);
const p='point-image-guard/hypersolve/src/algebraic_binary.rs',out='point-image-guard-red-source.rs';
copyFileSync(p,out,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
writeFileSync('point-image-guard-red-source.json',JSON.stringify({source:out,sha256:sha(out),
 classification:'Harness precondition failed: selected product endpoints compare Unknown under STRICT, not Equal. This run does not demonstrate a candidate witness defect; original failure/source preserved.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({source:out,sha256:sha(out)}));
