import {writeFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv}from './point-qualified-capture.mjs';
const cwd='symbolic-boundary-hyper-v83',current=retainedSources();assert.deepEqual(current,json('symbolic-boundary-origin-v83.json').current);
const paths=['run-symbolic-boundary-hyper-v83.mjs','symbolic-boundary-hyper-protocol-v83.md',cwd+'/src/main.rs',cwd+'/Cargo.toml',cwd+'/Cargo.lock','point-qualified-capture.mjs','capture.mjs'];
const files=Object.fromEntries(paths.map(p=>[p,sha(p)])),extraHyperReads={'hyperreal/src/real/arithmetic/elementary_functions.rs':[[2370,2670]],'hyperreal/src/real/arithmetic/facts.rs':[[1370,1435]]};
writeFileSync('symbolic-boundary-hyper-origin-v83.json',JSON.stringify({checkpoint:83,recorded:new Date().toISOString(),current,files,cargoEnv,extraHyperReads,
 extraHyperHashes:Object.fromEntries(Object.keys(extraHyperReads).map(p=>[p,sha(workspace+'/'+p)]))},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),current);for(const[p,h]of Object.entries(files))assert.equal(sha(p),h,p);};
await captured('symbolic-boundary-hyper-fmt-v83',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']);sources();
await captured('symbolic-boundary-hyper-metadata-v83',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);sources();
for(const profile of ['debug','release']){
 await captured('symbolic-boundary-hyper-'+profile+'-v83',cwd,'timeout',['180s','env',...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]);sources();
}
await captured('symbolic-boundary-hyper-clippy-v83',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);sources();
console.log(JSON.stringify({checkpoint:83,status:'hyper-comparison-collected',expectedRows:585,liveFiles:957,productionChanges:0}));
