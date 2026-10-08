import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {retainedSources,sha} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const source=retainedSources(),cwd='qqbar-trig-hyper-v76';
const paths=['run-qqbar-trig-hyper-v76.mjs','qqbar-trig-hyper-protocol-v76.md','qqbar-trig-hyper-initial-v76.rs',
 cwd+'/src/main.rs',cwd+'/Cargo.toml',cwd+'/Cargo.lock','point-qualified-capture.mjs','point-qualified-environment.mjs','capture.mjs',
 ...['json','stdout','stderr'].map(e=>'results/qqbar-trig-hyper-fmt-initial-v76.'+e)];
const files=Object.fromEntries(paths.map(p=>[p,sha(p)]));
writeFileSync('qqbar-trig-hyper-origin-v76.json',JSON.stringify({checkpoint:76,recorded:new Date().toISOString(),source,files,cargoEnv},null,2)+'\n',{flag:'wx'});
await captured('qqbar-trig-hyper-fmt-v76',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']);
await captured('qqbar-trig-hyper-metadata-v76',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);
for(const profile of ['debug','release'])await captured('qqbar-trig-hyper-'+profile+'-v76',cwd,'env',[
 ...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]);
await captured('qqbar-trig-hyper-clippy-v76',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);
await captured('qqbar-trig-hyper-environment-v76','.','node',['point-qualified-environment.mjs']);
for(const[p,h]of Object.entries(files))assert.equal(sha(p),h,p);assert.deepEqual(retainedSources(),source);
console.log(JSON.stringify({checkpoint:76,status:'hyper-probe-gates-pass',source}));
