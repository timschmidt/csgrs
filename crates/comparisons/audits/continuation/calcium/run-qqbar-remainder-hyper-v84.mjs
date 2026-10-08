import {writeFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv}from './point-qualified-capture.mjs';
const cwd='qqbar-remainder-hyper-v84',current=retainedSources();assert.deepEqual(current,json('qqbar-remainder-origin-v84.json').current);
const paths=['run-qqbar-remainder-hyper-v84.mjs',cwd+'/src/main.rs',cwd+'/Cargo.toml',cwd+'/Cargo.lock','point-qualified-capture.mjs','capture.mjs'];
const files=Object.fromEntries(paths.map(p=>[p,sha(p)])),extraHyperReads={'hyperreal/src/real/arithmetic/elementary_functions.rs':[[3880,4025]],'hyperreal/src/real/arithmetic/linear_algebra.rs':[[2140,2313]],'hypersolve/src/polynomial.rs':[[1,260]]};
writeFileSync('qqbar-remainder-hyper-origin-v84.json',JSON.stringify({checkpoint:84,recorded:new Date().toISOString(),current,files,cargoEnv,extraHyperReads,
 extraHyperHashes:Object.fromEntries(Object.keys(extraHyperReads).map(p=>[p,sha(workspace+'/'+p)]))},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),current);for(const[p,h]of Object.entries(files))assert.equal(sha(p),h,p);};
await captured('qqbar-remainder-hyper-fmt-v84',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']);sources();
await captured('qqbar-remainder-hyper-metadata-v84',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);sources();
for(const profile of ['debug','release']){
 await captured('qqbar-remainder-hyper-'+profile+'-v84',cwd,'timeout',['180s','env',...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]);sources();
}
await captured('qqbar-remainder-hyper-clippy-v84',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);sources();
console.log(JSON.stringify({checkpoint:84,status:'hyper-comparison-collected-not-independently-qualified',expectedRows:117,liveFiles:957,productionChanges:0}));
