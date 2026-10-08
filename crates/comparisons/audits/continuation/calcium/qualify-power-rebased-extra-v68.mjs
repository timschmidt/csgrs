import assert from 'node:assert/strict';
import {fixedSources,sha,json,binding,root} from './power-rebased-fixed-sources-v68.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const s=fixedSources();assert.deepEqual(s,json(binding));
const done=json('results/power-rebased-fixed-gates-v68.json');assert.equal(done.code,0);assert.equal(done.signal,null);
const b=json('power-rebased-fixed-binaries-v68.json');assert.equal(b.bindingSha256,sha(binding));
for(const a of b.artifacts)assert.equal(sha(a.path),a.sha256);
for(const variant of ['baseline','candidate']){
 await captured('power-rebased-app-clippy-'+variant+'-v68','power-rebased-'+variant+'-app-v68','env',
  [...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--','-D','warnings']);
 for(const mode of ['approx','extended']){
  const a=b.artifacts.find(a=>a.variant===variant&&a.mode===mode);
  await captured('power-rebased-memory-'+variant+'-'+mode+'-v68','.','valgrind',
   ['--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',a.path]);
 }
}
await captured('power-rebased-wasm-build-v68',root+'/hypersolve','env',
 [...cargoEnv,'cargo','build','--offline','--locked','--release','--lib','--all-features','--target','wasm32-unknown-unknown']);
assert.deepEqual(fixedSources(),s);console.log(JSON.stringify({checkpoint:68,status:'extra-gates-terminal',memoryRuns:4,wasm:'compile only'}));
