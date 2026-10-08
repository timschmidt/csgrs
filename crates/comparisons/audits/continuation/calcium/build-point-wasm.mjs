import {mkdtempSync,writeFileSync,copyFileSync,constants,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {coldBindings} from './point-cold-protocol.mjs';
import {sha} from './point-demand-sources.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const previous=coldBindings(),root=mkdtempSync('/tmp/calcium-point-wasm.'),artifacts=[];
writeFileSync('point-wasm-origin.json',JSON.stringify({root,recorded:new Date().toISOString(),
 predecessor:'point-cold-manifest.json',predecessorSha256:sha('point-cold-manifest.json'),
 sourceBindingSha256:sha('point-demand-source-binding.json')},null,2)+'\n',{flag:'wx'});
for(const variant of ['baseline','eager','demand']){
 const app='point-wasm-'+variant;
 await captured(app+'-lock',app,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
 await captured(app+'-build',app,'env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--target','wasm32-unknown-unknown','--lib']);
 await captured(app+'-clippy',app,'env',[...cargoEnv,'cargo','clippy','--locked','--offline','--target','wasm32-unknown-unknown','--lib','--','-D','warnings']);
 const path=root+'/'+variant+'.wasm',from=target+'/wasm32-unknown-unknown/release/point_wasm_'+variant+'.wasm';
 copyFileSync(from,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 artifacts.push({variant,path,bytes:statSync(path).size,sha256:sha(path)});
}
assert.deepEqual(coldBindings(),previous);
const result={root,artifacts,originSha256:sha('point-wasm-origin.json'),harnessSources:Object.fromEntries(
 ['point-history-base.rs','point-history-work.rs','point-wasm-work.rs','point-wasm-platform.rs'].map(p=>[p,sha(p)])),recorded:new Date().toISOString()};
writeFileSync('point-wasm-binaries.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
