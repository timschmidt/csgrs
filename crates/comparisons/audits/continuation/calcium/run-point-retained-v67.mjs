import assert from 'node:assert/strict';
import {retainedSources,workspace} from './point-retained-sources-v67.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const source=retainedSources(),live=workspace+'/hypersolve';
for(const profile of ['debug','release'])await captured('point-retained-'+profile+'-v67',live,'env',[
 ...cargoEnv,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--all-features','--no-fail-fast','--','--test-threads=2']);
await captured('point-retained-clippy-v67',live,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);
await captured('point-retained-fmt-v67',live,'env',[...cargoEnv,'cargo','fmt','--','--check']);
await captured('point-retained-wasm-v67',live,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--lib','--all-features','--target','wasm32-unknown-unknown']);
for(const[variant,cwd]of [['candidate','point-demand-candidate/hypersolve'],['live',live]])await captured('point-retained-metadata-'+variant+'-v67',cwd,'env',[
 ...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1','--all-features']);
await captured('point-retained-environment-v67','.','node',['point-qualified-environment.mjs']);
assert.deepEqual(retainedSources(),source);
console.log(JSON.stringify({checkpoint:67,status:'live-gates-pass',source}));
