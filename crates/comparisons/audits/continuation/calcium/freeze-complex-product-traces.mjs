import {copyFileSync,constants,statSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './complex-product-sources.mjs';
const frozen=json('complex-product-binaries.json');assert.deepEqual(sources(),frozen.sourceMap);
const binaries={};
for(const variant of ['baseline','candidate']) {
 assert.equal(json('results/complex-product-trace-build-'+variant+'.json').code,0);
 const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/complex-product-'+variant+'-trace';
 const path=frozen.root+'/'+variant+'-trace';
 copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 binaries[variant]={path,sha256:sha(path),bytes:statSync(path).size};assert.equal(sha(source),sha(path));
}
writeFileSync('complex-product-trace-binaries.json',JSON.stringify(binaries,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify(binaries));
