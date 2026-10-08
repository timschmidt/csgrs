import {copyFileSync,constants,existsSync,mkdtempSync,statSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
import {captured,target,cargoEnv} from './point-qualified-capture.mjs';
const o=sources(),old=json('e-qualified-app-size-summary.json'),artifacts=[];
assert.deepEqual(old.sourceMap.candidate,o.liveSources);
const root=mkdtempSync('/tmp/calcium-point-qualified-apps.');
writeFileSync('point-qualified-apps-origin.json',JSON.stringify({root,recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
for(const[crate,examples]of [['hyperreal',['readme_quickstart']],['hypercurve',['basic','arrangement']]]){
 for(const example of examples){
  const baseline=old.artifacts.find(a=>a.variant==='candidate'&&a.crate===crate&&a.example===example);assert(baseline);
  for(const f of baseline.files){assert.equal(statSync(f.path).size,f.bytes);assert.equal(sha(f.path),f.sha256);}
  artifacts.push({...baseline,variant:'baseline',reusedFrom:'e-qualified-app-size-summary.json'});
 }
 await captured('point-qualified-app-'+crate+'-build',o.candidate+'/'+crate,'env',[
  ...cargoEnv,'cargo','build','--locked','--offline','--release',...examples.flatMap(e=>['--example',e])]);
 for(const example of examples){
  const path=root+'/candidate-'+crate+'-'+example,stripped=path+'.stripped',id=crate+'-'+example.replaceAll('_','-');
  assert(!existsSync(path));copyFileSync(target+'/release/examples/'+example,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  await captured('point-qualified-app-'+id+'-strip','.','strip',['--strip-all','-o',stripped,path]);
  await captured('point-qualified-app-'+id+'-size','.','size',[path,stripped]);
  const baseline=artifacts.find(a=>a.variant==='baseline'&&a.crate===crate&&a.example===example);
  await captured('point-qualified-app-'+id+'-baseline-run','.',baseline.files[1].path,[]);
  await captured('point-qualified-app-'+id+'-candidate-run','.',stripped,[]);
  artifacts.push({variant:'candidate',crate,example,files:[path,stripped].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}))});
 }
}
assert.deepEqual(sources(),o);
const result={recorded:new Date().toISOString(),root,originSha256:sha('point-qualified-origin.json'),
 reusedBaselineSummarySha256:sha('e-qualified-app-size-summary.json'),artifacts,
 limits:'Three unchanged source examples, default features, ordinary release profile and separately stripped copies. Prior retained-baseline executables are source/hash/build-gate matched and rerun. These examples need not exercise binary root transforms. Not full Alumina, all-feature/LTO/size-optimized builds or application timings. Includes build-path and linker-layout effects.'};
writeFileSync('point-qualified-apps-summary.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
