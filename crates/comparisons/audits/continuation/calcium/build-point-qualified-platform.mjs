import {mkdtempSync,copyFileSync,constants,statSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha} from './point-qualified-sources.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const o=sources(),root=mkdtempSync('/tmp/calcium-point-qualified-platform.'),artifacts=[];
writeFileSync('point-qualified-platform-origin.json',JSON.stringify({root,recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
for(const variant of ['baseline','candidate']){
 const app='point-qualified-platform-'+variant,lib='calcium_point_qualified_platform_'+variant;
 await captured(app+'-lock',app,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
 await captured(app+'-native-build',app,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--bin',app]);
 await captured(app+'-wasm-build',app,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--target','wasm32-unknown-unknown','--lib']);
 for(const [platform,from,suffix]of [['native',target+'/release/'+app,''],['wasm',target+'/wasm32-unknown-unknown/release/'+lib+'.wasm','.wasm']]){
  const path=root+'/'+variant+suffix;copyFileSync(from,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  artifacts.push({variant,platform,path,bytes:statSync(path).size,sha256:sha(path)});
 }
}
assert.deepEqual(sources(),o);
const result={root,artifacts,originSha256:sha('point-qualified-origin.json'),harnessSha256:sha('point-qualified-platform.rs'),
 sharedCollectorSha256:sha('power-sums-public.rs'),recorded:new Date().toISOString()};
writeFileSync('point-qualified-platform-binaries.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
