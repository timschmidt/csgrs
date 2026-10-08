import {mkdtempSync,writeFileSync,statSync,copyFileSync,constants} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha} from './point-qualified-sources.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const o=sources(),root=mkdtempSync('/tmp/calcium-point-extended.'),artifacts=[];
writeFileSync('point-extended-origin.json',JSON.stringify({root,sourceOriginSha256:sha('point-qualified-origin.json'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
for(const variant of ['baseline','candidate']){
 const app='point-extended-'+variant;
 await captured(app+'-lock',app,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
 await captured(app+'-build',app,'env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bin',app+'-cpu']);
 const path=root+'/'+variant+'-cpu';copyFileSync(target+'/release/'+app+'-cpu',path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 artifacts.push({variant,path,bytes:statSync(path).size,sha256:sha(path)});
}
assert.deepEqual(sources(),o);
const result={root,artifacts,sourceOriginSha256:sha('point-qualified-origin.json'),harnessSha256:sha('point-extended.rs'),
 wrapperSha256:sha('point-extended-cpu.rs'),recorded:new Date().toISOString()};
writeFileSync('point-extended-binaries.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
