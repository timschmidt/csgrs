import {readFileSync,copyFileSync,constants,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const b=JSON.parse(readFileSync('point-image-binaries.json')),p='hypersolve/src/algebraic_binary.rs';
assert.equal(sha('point-image-candidate/'+p),b.sourceHashes[p]);
assert.equal(JSON.parse(readFileSync('results/point-image-candidate-clippy.json')).code,101);
copyFileSync('point-image-candidate/'+p,'point-image-v1-algebraic_binary.rs',constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
writeFileSync('point-image-test-cleanup.json',JSON.stringify({checkpoint:53,before:sha('point-image-v1-algebraic_binary.rs'),
 source:p,preserved:'point-image-v1-algebraic_binary.rs',change:'Remove unnecessary u64-to-u64 cast from a new cfg(test) regression only; no algorithm, oracle or captured result changes.',
 failedGate:'results/point-image-candidate-clippy.json',failedGateSha256:sha('results/point-image-candidate-clippy.json')},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({preserved:'point-image-v1-algebraic_binary.rs',sha256:sha('point-image-v1-algebraic_binary.rs')}));
