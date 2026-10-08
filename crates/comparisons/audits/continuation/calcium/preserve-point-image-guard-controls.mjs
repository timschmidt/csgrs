import {readFileSync,copyFileSync,constants,writeFileSync} from 'node:fs';
import {sha,json} from './point-image-sources.mjs';
import assert from 'node:assert/strict';
assert.equal(json('results/point-image-guard-controls.json').code,0);
const p='point-image-guard/hypersolve/src/algebraic_binary.rs',out='point-image-guard-unstrengthened.rs';
copyFileSync(p,out,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
assert.equal(readFileSync(out,'utf8').split('#[cfg(test)]')[0],readFileSync('point-image-v2-algebraic_binary.rs','utf8').split('#[cfg(test)]')[0]);
writeFileSync('point-image-guard-strengthening.json',JSON.stringify({checkpoint:53,source:out,sha256:sha(out),
 classification:'Both ordering controls withhold witnesses in the original candidate. No approximate-extrema witness defect was reproduced. The next guard closes a source-level proof obligation: an exact witness must use certified image construction, not merely certify equality of approximately selected extrema.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({source:out,sha256:sha(out)}));
