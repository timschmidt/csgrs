import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import {retainedSources,workspace,sha,json} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const source=retainedSources(),live=workspace+'/hypersolve';
const inputs=['run-zero-factor-retained-approved-v75.mjs','run-zero-factor-retained-v75.mjs','zero-factor-retained-sources-v75.mjs','zero-factor-retained-protocol-v75.md',
 'zero-factor-retained-origin-v75.json','point-qualified-capture.mjs','capture.mjs','point-qualified-environment.mjs'];
const bound=Object.fromEntries(inputs.map(p=>[p,sha(p)]));
writeFileSync('zero-factor-retained-run-origin-approved-v75.json',JSON.stringify({checkpoint:75,recorded:new Date().toISOString(),source,files:bound,cargoEnv},null,2)+'\n',{flag:'wx'});
await captured('zero-factor-retained-environment-before-approved-v75','.','node',['point-qualified-environment.mjs']);
for(const profile of ['default','all-features','release'])await captured('zero-factor-retained-'+profile+'-v75',live,'env',[
 ...cargoEnv,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),
 ...(profile==='default'?[]:['--all-features']),'--no-fail-fast','--','--test-threads=2']);
await captured('zero-factor-retained-clippy-v75',live,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);
await captured('zero-factor-retained-fmt-v75',live,'env',[...cargoEnv,'cargo','fmt','--','--check']);
await captured('zero-factor-retained-wasm-v75',live,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--lib','--all-features','--target','wasm32-unknown-unknown']);
for(const[variant,cwd]of [['candidate','zero-factor-candidate-v70/hypersolve'],['live',live]])await captured('zero-factor-retained-metadata-'+variant+'-v75',cwd,'env',[
 ...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1','--all-features']);
await captured('zero-factor-retained-environment-after-v75','.','node',['point-qualified-environment.mjs']);
for(const[p,h]of Object.entries(bound))assert.equal(sha(p),h,p);
assert.deepEqual(json('zero-factor-retained-run-origin-approved-v75.json').source,source);
assert.deepEqual(retainedSources(),source);
console.log(JSON.stringify({checkpoint:75,status:'live-gates-pass',source}));
