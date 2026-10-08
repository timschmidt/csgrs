import {readFileSync,writeFileSync,copyFileSync,constants,mkdtempSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {consumerSources,crate,sha,json} from './point-consumer-sources-v66.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
import {coldBindings} from './point-cold-protocol.mjs';
import {wasmBindings} from './point-wasm-protocol.mjs';
const source=consumerSources(),release=json('results/point-consumer-release-v66.json');
assert.equal(release.code,0);assert.equal(release.signal,null);
const manifest=crate+'/Cargo.toml';
await captured('point-consumer-fmt-v66','.','env',[...cargoEnv,'cargo','fmt','--manifest-path',manifest,'--','--check']);
await captured('point-consumer-clippy-v66','.','env',[...cargoEnv,'cargo','clippy','--offline','--locked','--manifest-path',manifest,
 '--all-targets','--all-features','--','-D','warnings']);
await captured('point-consumer-wasm-v66','.','env',[...cargoEnv,'cargo','build','--offline','--locked','--manifest-path',manifest,
 '--release','--lib','--features','triangulation,svg,hershey','--target','wasm32-unknown-unknown']);
const prior=json('point-qualified-apps-summary.json'),baseline=json('e-qualified-app-size-summary.json');
const root=mkdtempSync('/tmp/calcium-point-consumer.');
writeFileSync('point-consumer-app-origin-v66.json',JSON.stringify({root,recorded:new Date().toISOString(),
 sourceBindingSha256:sha('point-consumer-binding-v66.json')},null,2)+'\n',{flag:'wx'});
await captured('point-consumer-app-build-v66',crate,'env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--example','basic','--example','arrangement']);
const artifacts=[];
for(const example of ['basic','arrangement']){
 const path=root+'/demand-'+example,stripped=path+'.stripped';
 copyFileSync(target+'/release/examples/'+example,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 await captured('point-consumer-'+example+'-strip-v66','.','strip',['--strip-all','-o',stripped,path]);
 const refs=['baseline','candidate'].map(v=>{
  const a=prior.artifacts.find(a=>a.variant===v&&a.crate==='hypercurve'&&a.example===example);assert(a);
  if(v==='baseline')assert.deepEqual(a.files,baseline.artifacts.find(b=>b.variant==='candidate'&&b.crate==='hypercurve'&&b.example===example).files);
  for(const f of a.files){assert.equal(sha(f.path),f.sha256);assert.equal(statSync(f.path).size,f.bytes);}
  return{...a,variant:v==='baseline'?'baseline':'eager',reusedFrom:'point-qualified-apps-summary.json'};
 });
 const candidate={variant:'demand',crate:'hypercurve',example,files:[path,stripped].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}))};
 for(const a of [...refs,candidate])await captured('point-consumer-'+example+'-'+a.variant+'-run-v66','.',a.files[1].path,[]);
 for(const a of refs)assert.equal(sha('results/point-consumer-'+example+'-'+a.variant+'-run-v66.stdout'),
  sha('results/point-consumer-'+example+'-demand-run-v66.stdout'));
 await captured('point-consumer-'+example+'-size-v66','.','size',[...refs,candidate].flatMap(a=>a.files.map(f=>f.path)));
 artifacts.push(...refs,candidate);
}
const cold=coldBindings(),wasm=wasmBindings();
await captured('point-consumer-cold-size-v66','.','size',cold.artifacts.filter(a=>a.platform==='native').map(a=>a.path));
await captured('point-consumer-environment-v66','.','node',['point-qualified-environment.mjs']);
assert.deepEqual(consumerSources(),source);
const result={checkpoint:66,recorded:new Date().toISOString(),root,sourceBindingSha256:sha('point-consumer-binding-v66.json'),
 priorAppsSha256:sha('point-qualified-apps-summary.json'),baselineAppsSha256:sha('e-qualified-app-size-summary.json'),artifacts,
 reusedCold:{bindingSha256:sha('point-cold-binaries.json'),artifacts:cold.artifacts},
 reusedWasm:{bindingSha256:sha('point-wasm-binaries.json'),artifacts:wasm.artifacts},
 dedicatedBytes:artifacts.filter(a=>a.variant==='demand').flatMap(a=>a.files).reduce((n,f)=>n+f.bytes,0),
 limits:'Two unchanged default-feature release examples and separate stripped copies. Prior frozen baseline/eager examples rerun; source/configuration matched, different build paths and linker layout remain confounders. The examples are representative consumers, not proof that the repaired binary-image path was executed. Cold/history collector binary sizes are reused frozen observations with matched harnesses, not newly rebuilt binaries. No full application, LTO/size-profile, allocator or timing qualification.'};
writeFileSync('point-consumer-apps-v66.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:66,status:'pass',root,examples:2,dedicatedFiles:4,dedicatedBytes:result.dedicatedBytes}));
