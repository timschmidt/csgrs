import {writeFileSync,copyFileSync,constants,statSync,readdirSync} from 'node:fs';
import assert from 'node:assert/strict';
import {demandBindings} from './point-demand-bindings.mjs';
import {sha,json} from './point-demand-sources.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const previous=demandBindings(),origin=json('point-cold-origin.json'),root=origin.root,artifacts=[];
assert.equal(origin.predecessorSha256,sha('point-demand-manifest.json'));
assert.equal(origin.sourceBindingSha256,sha('point-demand-source-binding.json'));
assert.equal(json('results/point-cold-build.json').code,1);
assert.equal(json('results/point-cold-baseline-native-clippy.json').code,101);
assert.deepEqual(readdirSync(root),[]);
for(const variant of ['baseline','eager','demand']){
 const app='point-cold-'+variant;
 await captured(app+'-lock',app,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
 await captured(app+'-native-build-v1',app,'env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bin',app]);
 await captured(app+'-native-clippy-v1',app,'env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--','-D','warnings']);
 await captured(app+'-wasm-build',app,'env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--target','wasm32-unknown-unknown','--lib']);
 await captured(app+'-wasm-clippy',app,'env',[...cargoEnv,'cargo','clippy','--locked','--offline','--target','wasm32-unknown-unknown','--lib','--','-D','warnings']);
 for(const platform of ['native','wasm']){
  const path=root+'/'+variant+(platform==='wasm'?'.wasm':'');
  const from=platform==='native'?target+'/release/'+app:target+'/wasm32-unknown-unknown/release/point_cold_'+variant+'.wasm';
  copyFileSync(from,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  artifacts.push({variant,platform,path,bytes:statSync(path).size,sha256:sha(path)});
 }
}
assert.deepEqual(demandBindings(),previous);
const result={root,artifacts,originSha256:sha('point-cold-origin.json'),harnessSources:Object.fromEntries(
 ['point-history-base.rs','point-cold-common.rs','point-cold-native.rs','point-cold-platform.rs'].map(p=>[p,sha(p)])),recorded:new Date().toISOString()};
writeFileSync('point-cold-binaries.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
