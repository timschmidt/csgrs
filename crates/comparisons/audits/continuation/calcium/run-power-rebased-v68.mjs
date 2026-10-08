import {mkdtempSync,copyFileSync,constants,statSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {rebasedSources,sha,json,binding,root} from './power-rebased-sources-v68.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const source=rebasedSources();assert.deepEqual(source,json(binding));
const artifacts=[],dir=mkdtempSync('/tmp/calcium-power-rebased.');
writeFileSync('power-rebased-build-origin-v68.json',JSON.stringify({dir,bindingSha256:sha(binding),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
for(const variant of ['baseline','candidate']){
 const cwd='power-rebased-'+variant+'-app-v68',tag='power-rebased-'+variant;
 await captured(tag+'-lock-v68',cwd,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
 await captured(tag+'-metadata-v68',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);
 await captured(tag+'-build-v68',cwd,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--bins']);
 for(const mode of ['cpu','allocation','approx','extended']){
  const path=dir+'/'+variant+'-'+mode;copyFileSync(target+'/release/power-rebased-'+mode+'-v68',path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  artifacts.push({variant,mode,path,bytes:statSync(path).size,sha256:sha(path)});
 }
}
writeFileSync('power-rebased-binaries-v68.json',JSON.stringify({dir,artifacts,bindingSha256:sha(binding),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
for(const profile of ['debug','release'])await captured('power-rebased-tests-'+profile+'-v68',root+'/hypersolve','env',
 [...cargoEnv,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--all-features','--no-fail-fast','--','--test-threads=2']);
await captured('power-rebased-kernel-v68',root+'/hypersolve','env',[...cargoEnv,'cargo','test','--offline','--locked','--all-features','--lib',
 'algebraic_binary::power_sums::tests::audit_kernel_corpus','--','--exact','--nocapture','--test-threads=1']);
await captured('power-rebased-kernel-check-v68','.','node',['check-power-sums-kernel.mjs','results/power-rebased-kernel-v68.stdout']);
for(const variant of ['baseline','candidate'])for(const mode of ['cpu','approx','extended']){
 const a=artifacts.find(a=>a.variant===variant&&a.mode===mode);
 await captured('power-rebased-'+variant+'-'+(mode==='cpu'?'public':mode)+'-v68','.',a.path,mode==='cpu'?['check']:[]);
}
await captured('power-rebased-public-check-v68','.','node',['check-power-sums-public.mjs',
 'results/power-rebased-baseline-public-v68.stdout','results/power-rebased-candidate-public-v68.stdout']);
await captured('power-rebased-approx-check-v68','.','node',['check-power-sums-public.mjs',
 'results/power-rebased-baseline-approx-v68.stdout','results/power-rebased-candidate-approx-v68.stdout']);
await captured('power-rebased-clippy-v68',root+'/hypersolve','env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);
await captured('power-rebased-fmt-v68',root+'/hypersolve','env',[...cargoEnv,'cargo','fmt','--','--check']);
await captured('power-rebased-environment-v68','.','node',['point-qualified-environment.mjs']);
assert.deepEqual(rebasedSources(),source);
console.log(JSON.stringify({checkpoint:68,status:'initial-gates-terminal',artifacts:artifacts.length,bytes:artifacts.reduce((n,a)=>n+a.bytes,0),
 limits:'Read complete outputs and independently check full metadata/extended records. No timing/allocation or production retention yet.'}));
