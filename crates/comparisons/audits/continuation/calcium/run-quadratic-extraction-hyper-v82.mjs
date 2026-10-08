import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const current=retainedSources(),cwd='quadratic-extraction-hyper-v82';assert.deepEqual(current,json('quadratic-extraction-origin-v82.json').current);
const paths=['run-quadratic-extraction-hyper-v82.mjs','quadratic-extraction-hyper-protocol-v82.md',cwd+'/src/main.rs',cwd+'/Cargo.toml',cwd+'/Cargo.lock',
 'quadratic-extraction-input-v82.json','point-qualified-capture.mjs','point-qualified-environment.mjs','capture.mjs'];
const files=Object.fromEntries(paths.map(p=>[p,sha(p)])),extraHyperReads={'hyperreal/src/structural.rs':[[115,175]],'hyperreal/src/real/arithmetic/facts.rs':[[1310,1365]]};
writeFileSync('quadratic-extraction-hyper-origin-v82.json',JSON.stringify({checkpoint:82,recorded:new Date().toISOString(),current,files,cargoEnv,extraHyperReads,
 extraHyperHashes:Object.fromEntries(Object.keys(extraHyperReads).map(p=>[p,sha(workspace+'/'+p)]))},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),current);for(const[p,h]of Object.entries(files))assert.equal(sha(p),h,p);};
await captured('quadratic-extraction-hyper-fmt-v82',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']);sources();
await captured('quadratic-extraction-hyper-metadata-v82',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);sources();
for(const profile of ['debug','release']){
 await captured('quadratic-extraction-hyper-'+profile+'-v82',cwd,'env',[...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]);sources();
}
await captured('quadratic-extraction-hyper-clippy-v82',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);sources();
await captured('quadratic-extraction-hyper-environment-v82','.','node',['point-qualified-environment.mjs']);sources();
console.log(JSON.stringify({checkpoint:82,status:'hyper-capability-collected',expectedRows:935,liveFiles:957,productionChanges:0}));
