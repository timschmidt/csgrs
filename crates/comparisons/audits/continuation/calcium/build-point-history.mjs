import {mkdtempSync,writeFileSync,statSync,copyFileSync,constants} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha} from './point-qualified-sources.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const o=sources(),root=mkdtempSync('/tmp/calcium-point-history.'),artifacts=[];
writeFileSync('point-history-origin.json',JSON.stringify({root,sourceOriginSha256:sha('point-qualified-origin.json'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
for(const variant of ['baseline','candidate']){
 const app='point-history-'+variant;
 await captured(app+'-lock',app,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
 await captured(app+'-build',app,'env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bins']);
 await captured(app+'-clippy',app,'env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--','-D','warnings']);
 for(const mode of ['cpu','allocation']){
  const path=root+'/'+variant+'-'+mode;copyFileSync(target+'/release/'+app+'-'+mode,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  artifacts.push({variant,mode,path,bytes:statSync(path).size,sha256:sha(path)});
 }
}
assert.deepEqual(sources(),o);
const result={root,artifacts,sourceOriginSha256:sha('point-qualified-origin.json'),
 harnessSources:Object.fromEntries(['point-history-base.rs','point-history-work.rs','point-history-cpu.rs','point-history-allocation.rs'].map(p=>[p,sha(p)])),
 recorded:new Date().toISOString()};
writeFileSync('point-history-binaries.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
