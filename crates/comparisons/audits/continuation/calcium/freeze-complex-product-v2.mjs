import {copyFileSync,constants,statSync,mkdtempSync,writeFileSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import assert from 'node:assert/strict';
import {sources,sha,json} from './complex-product-v2-sources.mjs';
const sourceMap=sources(),previous=json('complex-product-binaries.json');
if(process.argv.includes('--trace')) {
 const frozen=json('complex-product-v2-binaries.json');assert.deepEqual(sourceMap,frozen.sourceMap);
 assert.equal(json('results/complex-product-v2-trace-build-candidate.json').code,0);
 const baseline=json('complex-product-trace-binaries.json').baseline;
 assert.equal(sha(baseline.path),baseline.sha256);
 const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/complex-product-v2-candidate-trace',path=frozen.root+'/candidate-trace';
 copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 const candidate={path,sha256:sha(path),bytes:statSync(path).size};assert.equal(sha(source),sha(path));
 writeFileSync('complex-product-v2-trace-binaries.json',JSON.stringify({baseline,candidate},null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({baseline,candidate}));
} else {
 assert.equal(json('results/complex-product-v2-build-candidate.json').code,0);
 const root=mkdtempSync('/tmp/calcium-complex-product-v2.'),binaries={};
 for(const kind of ['cpu','allocation','check']) {
  const baseline=previous.binaries['baseline-'+kind];assert.equal(sha(baseline.path),baseline.sha256);
  binaries['baseline-'+kind]=baseline;
  const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/complex-product-v2-candidate-'+kind,path=root+'/candidate-'+kind;
  copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  binaries['candidate-'+kind]={path,sha256:sha(path),bytes:statSync(path).size,size:execFileSync('size',[path],{encoding:'utf8'}).trim()};
  assert.equal(sha(source),sha(path));
 }
 writeFileSync('complex-product-v2-binaries.json',JSON.stringify({root,binaries,sourceMap},null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({root,binaries}));
}
