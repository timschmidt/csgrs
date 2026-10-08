import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
import {sources,sha,json} from './e-plan-sources.mjs';
const mode=process.argv[2];assert(['native','memcheck'].includes(mode));
const frozen=json('e-plan-binaries.json');assert.deepEqual(sources(),frozen.sourceMap);
for(const b of Object.values(frozen.binaries))assert.equal(sha(b.path),b.sha256);
for(const variant of ['baseline','candidate'])for(const kind of mode==='native'?['plans','numeric','state']:['plans','numeric']) {
 const binary=frozen.binaries[variant+'-check'].path;
 const command=mode==='native'?[binary,kind]:['valgrind','--vgdb=no','--tool=memcheck','--error-exitcode=97',
  '--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',binary,kind];
 const tag='e-plan-'+mode+'-'+variant+'-'+kind;
 await new Promise((ok,fail)=>{
  const child=spawn(process.execPath,['capture.mjs',tag,'.',...command],{stdio:'inherit'});child.on('error',fail);
  child.on('close',(code,signal)=>code===0&&!signal?ok():fail(Error(JSON.stringify({tag,code,signal}))));
 });
}
assert.deepEqual(sources(),frozen.sourceMap);
