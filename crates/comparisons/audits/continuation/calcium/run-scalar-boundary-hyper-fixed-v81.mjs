import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {retainedSources,sha,json,workspace} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const current=retainedSources(),cwd='scalar-boundary-hyper-v81',old=json('scalar-boundary-hyper-origin-v81.json');assert.deepEqual(current,old.current);
for(const[p,h]of Object.entries(old.files))assert.equal(sha(p===cwd+'/src/main.rs'?'scalar-boundary-hyper-initial-v81.rs':p),h,p);
const compact=p=>readFileSync(p,'utf8').replace(/\s/g,'');
assert.equal(compact(cwd+'/src/main.rs'),compact('scalar-boundary-hyper-initial-v81.rs').replace('value.as_f64_lossy()','value.to_f64_lossy()')
 .replace('angle.sec_pi()','Real::one()/angle.cos_pi()').replace('angle.csc_pi()','Real::one()/angle.sin_pi()'));
const paths=['run-scalar-boundary-hyper-fixed-v81.mjs','scalar-boundary-hyper-correction-v81.md',cwd+'/src/main.rs',cwd+'/Cargo.toml',cwd+'/Cargo.lock',
 'scalar-boundary-hyper-initial-v81.rs','scalar-boundary-hyper-origin-v81.json','point-qualified-capture.mjs','point-qualified-environment.mjs','capture.mjs'];
const files=Object.fromEntries(paths.map(p=>[p,sha(p)])),extraHyperReads={'hyperreal/src/real/convert.rs':[[375,425],[490,560]]};
const o={checkpoint:81,recorded:new Date().toISOString(),current,files,cargoEnv,extraHyperReads,
 extraHyperHashes:Object.fromEntries(Object.keys(extraHyperReads).map(p=>[p,sha(workspace+'/'+p)]))};
writeFileSync('scalar-boundary-hyper-origin-fixed-v81.json',JSON.stringify(o,null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),current);for(const[p,h]of Object.entries(files))assert.equal(sha(p),h,p);};
await captured('scalar-boundary-hyper-fmt-fixed-v81',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']);sources();
await captured('scalar-boundary-hyper-metadata-fixed-v81',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);sources();
for(const profile of ['debug','release']){
 await captured('scalar-boundary-hyper-'+profile+'-fixed-v81',cwd,'env',[...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]);sources();
}
await captured('scalar-boundary-hyper-clippy-fixed-v81',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);sources();
await captured('scalar-boundary-hyper-environment-fixed-v81','.','node',['point-qualified-environment.mjs']);sources();
console.log(JSON.stringify({checkpoint:81,status:'hyper-capability-collected',expectedRows:19341,liveFiles:current.liveFiles,productionChanges:0}));
