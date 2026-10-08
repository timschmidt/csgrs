import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {retainedSources,sha,json} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const source=retainedSources(),cwd='qqbar-inverse-hyper-v80';
assert.deepEqual(source,json('qqbar-inverse-origin-v80.json').current);
const paths=['run-qqbar-inverse-hyper-v80.mjs','qqbar-inverse-hyper-protocol-v80.md',
 cwd+'/src/main.rs',cwd+'/Cargo.toml',cwd+'/Cargo.lock','point-qualified-capture.mjs','point-qualified-environment.mjs','capture.mjs'];
const files=Object.fromEntries(paths.map(p=>[p,sha(p)]));
writeFileSync('qqbar-inverse-hyper-origin-v80.json',JSON.stringify({checkpoint:80,recorded:new Date().toISOString(),source,files,cargoEnv},null,2)+'\n',{flag:'wx'});
const check=()=>{for(const[p,h]of Object.entries(files))assert.equal(sha(p),h,p);assert.deepEqual(retainedSources(),source);};
await captured('qqbar-inverse-hyper-fmt-v80',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']);check();
await captured('qqbar-inverse-hyper-metadata-v80',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);check();
for(const profile of ['debug','release']){
 await captured('qqbar-inverse-hyper-'+profile+'-v80',cwd,'env',[
  ...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]);check();
}
await captured('qqbar-inverse-hyper-clippy-v80',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);check();
await captured('qqbar-inverse-hyper-environment-v80','.','node',['point-qualified-environment.mjs']);check();
console.log(JSON.stringify({checkpoint:80,status:'hyper-capability-collected',expectedRows:1849,profiles:['debug','release'],retained:false}));
