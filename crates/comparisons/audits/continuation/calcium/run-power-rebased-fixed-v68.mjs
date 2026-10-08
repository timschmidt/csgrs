import {copyFileSync,constants,statSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {fixedSources,sha,json,binding,root} from './power-rebased-fixed-sources-v68.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const source=fixedSources();assert.deepEqual(source,json(binding));
const original=json('power-rebased-binaries-v68.json'),dir=original.dir,artifacts=[];
for(const a of original.artifacts){assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);}
await captured('power-rebased-fixed-build-v68','power-rebased-candidate-app-v68','env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--bins']);
for(const mode of ['cpu','allocation','approx','extended']){
 const path=dir+'/fixed-'+mode;copyFileSync(target+'/release/power-rebased-'+mode+'-v68',path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 artifacts.push({variant:'candidate',mode,path,bytes:statSync(path).size,sha256:sha(path)});
}
artifacts.push(...original.artifacts.filter(a=>a.variant==='baseline'));
writeFileSync('power-rebased-fixed-binaries-v68.json',JSON.stringify({dir,artifacts,bindingSha256:sha(binding),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
for(const profile of ['debug','release'])await captured('power-rebased-fixed-tests-'+profile+'-v68',root+'/hypersolve','env',
 [...cargoEnv,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--all-features','--no-fail-fast','--','--test-threads=2']);
await captured('power-rebased-fixed-kernel-v68',root+'/hypersolve','env',[...cargoEnv,'cargo','test','--offline','--locked','--all-features','--lib',
 'algebraic_binary::power_sums::tests::audit_kernel_corpus','--','--exact','--nocapture','--test-threads=1']);
await captured('power-rebased-fixed-kernel-check-v68','.','node',['check-power-sums-kernel.mjs','results/power-rebased-fixed-kernel-v68.stdout']);
for(const mode of ['cpu','approx','extended']){
 const a=artifacts.find(a=>a.variant==='candidate'&&a.mode===mode);
 await captured('power-rebased-fixed-'+(mode==='cpu'?'public':mode)+'-v68','.',a.path,mode==='cpu'?['check']:[]);
}
for(const mode of ['public','approx'])await captured('power-rebased-fixed-'+mode+'-check-v68','.','node',['check-power-sums-public.mjs',
 'results/power-rebased-baseline-'+mode+'-v68.stdout','results/power-rebased-fixed-'+mode+'-v68.stdout']);
await captured('power-rebased-fixed-clippy-v68',root+'/hypersolve','env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);
await captured('power-rebased-fixed-fmt-v68',root+'/hypersolve','env',[...cargoEnv,'cargo','fmt','--','--check']);
await captured('power-rebased-fixed-environment-v68','.','node',['point-qualified-environment.mjs']);
assert.deepEqual(fixedSources(),source);
console.log(JSON.stringify({checkpoint:68,status:'amended-gates-terminal',artifacts:artifacts.length,newFiles:4,
 limits:'Baseline runs/binaries reused; corrected candidate rerun. All original evidence preserved. No timing/allocation or retention yet.'}));
