import {mkdtempSync,writeFileSync,copyFileSync,statSync,constants} from 'node:fs';
import assert from 'node:assert/strict';
import {demandSources,sha} from './point-demand-sources.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const source=demandSources(),root=mkdtempSync('/tmp/calcium-point-demand.'),artifacts=[];
writeFileSync('point-demand-build-origin.json',JSON.stringify({root,sourceBindingSha256:sha('point-demand-source-binding.json'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
await captured('point-demand-app-lock','point-demand-app','env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
await captured('point-demand-app-build','point-demand-app','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bins']);
await captured('point-demand-app-clippy','point-demand-app','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--','-D','warnings']);
for(const mode of ['cpu','allocation','extended','public','approx']){
 const path=root+'/'+mode;copyFileSync(target+'/release/point-demand-'+mode,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 artifacts.push({variant:'demand',mode,path,bytes:statSync(path).size,sha256:sha(path)});
}
for(const profile of ['debug','release']){
 await captured('point-demand-solver-'+profile,source.root+'/hypersolve','env',[...cargoEnv,'cargo','test','--locked','--offline',
  ...(profile==='release'?['--release']:[]),'--all-features','--lib']);
}
await captured('point-demand-solver-clippy',source.root+'/hypersolve','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']);
assert.deepEqual(demandSources(),source);
const result={root,artifacts,sourceBindingSha256:sha('point-demand-source-binding.json'),
 harnessSources:Object.fromEntries(['point-history-base.rs','point-history-work.rs','point-history-cpu.rs','point-history-allocation.rs',
 'point-demand-extended.rs','point-qualified-platform.rs','point-qualified-approx.rs','power-sums-public.rs'].map(p=>[p,sha(p)])),recorded:new Date().toISOString()};
writeFileSync('point-demand-binaries.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
