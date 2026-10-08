import {mkdtempSync,writeFileSync,copyFileSync,constants,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {fixedZeroSources,sha,json} from './zero-factor-fixed-sources-v70.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const source=fixedZeroSources();assert.deepEqual(source,json('zero-factor-fixed-binding-v70.json'));
const files=['zero-factor-degree-cases-v70.mjs','zero-factor-degree-input-v70.json','power-wide-v69.rs','power-wide-input-v69.json',
 'power-sums-public.rs','power-sums-cpu.rs','point-qualified-approx.rs','point-demand-extended.rs','point-history-base.rs','point-extended.rs',
 'zero-factor-candidate-app-v70/Cargo.toml','run-zero-factor-v70.mjs'];
const origin={checkpoint:70,recorded:new Date().toISOString(),source,files:Object.fromEntries(files.map(p=>[p,sha(p)]))};
writeFileSync('zero-factor-gates-origin-v70.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
const cwd=source.root+'/hypersolve';
for(const [tag,args]of [
 ['default',['test','--offline','--locked']],['all-features',['test','--offline','--locked','--all-features']],
 ['clippy',['clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']],
 ['fmt',['fmt','--all','--','--check']],
 ['wasm',['build','--offline','--locked','--release','--all-features','--target','wasm32-unknown-unknown']]
])await captured('zero-factor-'+tag+'-v70',cwd,'env',[...cargoEnv,'cargo',...args]);
const app='zero-factor-candidate-app-v70';
await captured('zero-factor-lock-v70',app,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
await captured('zero-factor-metadata-v70',app,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);
await captured('zero-factor-build-v70',app,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--bins']);
const dir=mkdtempSync('/tmp/calcium-zero-factor.'),binaries=[];
for(const name of ['wide','public','approx','extended']){
 const path=dir+'/'+name;copyFileSync(target+'/release/zero-factor-'+name+'-v70',path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 binaries.push({name,path,sha256:sha(path),bytes:statSync(path).size});
 await captured('zero-factor-'+name+'-run-v70','.',path,name==='wide'?[resolve('power-wide-input-v69.json')]:['check']);
 if(name==='wide')await captured('zero-factor-degree-run-v70','.',path,[resolve('zero-factor-degree-input-v70.json')]);
}
await captured('zero-factor-app-clippy-v70',app,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--','-D','warnings']);
for(const[p,h]of Object.entries(origin.files))assert.equal(sha(p),h,p);assert.deepEqual(fixedZeroSources(),source);
writeFileSync('zero-factor-binaries-v70.json',JSON.stringify({binaries,originSha256:sha('zero-factor-gates-origin-v70.json'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:70,status:'execution-gates-terminal',binaries,limits:'Independent output qualification still required; no performance/consumer/retention claims.'}));
