import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
import {sources,json} from './e-plan-sources.mjs';
const frozen=json('e-plan-binaries.json');assert.deepEqual(sources(),frozen.sourceMap);
const o=json('e-plan-origin.json');
for(const variant of ['baseline','candidate'])for(const profile of ['debug','release']) {
 const root=variant==='baseline'?o.baseline:o.candidate;
 const tag='e-plan-tests-'+variant+'-'+profile;
 const args=['capture.mjs',tag,'.','env','CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',
  'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2','cargo','test','--locked','--offline',
  '--manifest-path',root+'/hyperreal/Cargo.toml','--all-features','--lib','--tests',...(profile==='release'?['--release']:[]),
  '--','--test-threads=4'];
 await new Promise((ok,fail)=>{
  const child=spawn(process.execPath,args,{stdio:'inherit'});child.on('error',fail);
  child.on('close',(code,signal)=>code===0&&!signal?ok():fail(Error(JSON.stringify({tag,code,signal}))));
 });
}
assert.deepEqual(sources(),frozen.sourceMap);
