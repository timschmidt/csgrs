import {mkdtempSync,writeFileSync,copyFileSync,constants,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {fixedSources,sha,json} from './power-rebased-fixed-sources-v68.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const prior=json('power-rebased-v68-manifest.json');for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
const source=fixedSources();assert.deepEqual(source,prior.source);
const done=json('results/power-rebased-verify-v68.json');assert.equal(done.code,0);assert.equal(done.signal,null);
const files=['power-wide-cases-v69.mjs','power-wide-input-v69.json','power-wide-v69.rs','power-sums-public.rs',
 'power-wide-baseline-app-v69/Cargo.toml','power-wide-candidate-app-v69/Cargo.toml','run-power-wide-v69.mjs'];
const dir=mkdtempSync('/tmp/calcium-power-wide.'),origin={checkpoint:69,recorded:new Date().toISOString(),dir,source,
 previousSha256:sha('power-rebased-v68-manifest.json'),files:Object.fromEntries(files.map(p=>[p,sha(p)]))};
writeFileSync('power-wide-origin-v69.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
const binaries=[];
for(const variant of ['baseline','candidate']){
 const cwd='power-wide-'+variant+'-app-v69',tag='power-wide-'+variant;
 await captured(tag+'-lock-v69',cwd,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
 await captured(tag+'-metadata-v69',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);
 await captured(tag+'-build-v69',cwd,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--bin','power-wide-v69']);
 const path=dir+'/'+variant;copyFileSync(target+'/release/power-wide-v69',path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 binaries.push({variant,path,sha256:sha(path),bytes:statSync(path).size});
 await captured(tag+'-run-v69','.',path,[resolve('power-wide-input-v69.json')]);
 await captured(tag+'-clippy-v69',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--','-D','warnings']);
}
for(const[p,h]of Object.entries(origin.files))assert.equal(sha(p),h,p);assert.deepEqual(fixedSources(),source);
writeFileSync('power-wide-binaries-v69.json',JSON.stringify({binaries,originSha256:sha('power-wide-origin-v69.json'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:69,status:'wide-executions-terminal',binaries,limits:'Independent oracle check still required; no numerical/performance qualification implied by exit codes.'}));
