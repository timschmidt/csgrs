import {spawn} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,json} from './e-plan-qualified-sources.mjs';
const mode=process.argv[2];assert(['scalar','quality','consumer'].includes(mode));
const initial=sources(),o=json('e-plan-qualified-origin.json'),jobs=[];
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
const add=(tag,cwd,args)=>jobs.push({tag:'e-qualified-'+tag,cwd,command:'env',args:[...env,'cargo',...args]});
if(mode==='scalar') {
 for(const variant of ['baseline','candidate'])for(const profile of ['debug','release'])
  add(variant+'-default-'+profile,(variant==='baseline'?o.baseline:o.candidate)+'/hyperreal',
   ['test','--locked','--offline','--lib','--tests',...(profile==='release'?['--release']:[]),'--','--test-threads=4']);
 for(const profile of ['debug','release'])add('candidate-all-'+profile,o.candidate+'/hyperreal',
  ['test','--locked','--offline','--all-features','--lib','--tests',...(profile==='release'?['--release']:[]),'--','--test-threads=4']);
 add('candidate-doc',o.candidate+'/hyperreal',['test','--locked','--offline','--all-features','--doc']);
} else if(mode==='quality') {
 add('candidate-fmt',o.candidate+'/hyperreal',['fmt','--all','--','--check']);
 add('candidate-clippy-all',o.candidate+'/hyperreal',['clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']);
 add('candidate-check-default',o.candidate+'/hyperreal',['check','--locked','--offline','--all-targets']);
 add('candidate-fuzz',o.candidate+'/hyperreal',['check','--locked','--offline','--manifest-path','fuzz/Cargo.toml','--bins']);
 for(const variant of ['baseline','candidate']) {
  const root=variant==='baseline'?o.baseline:o.candidate;
  add(variant+'-wasm-scalar',root+'/hyperreal',['build','--locked','--offline','--release','--lib','--all-features','--target','wasm32-unknown-unknown']);
  add(variant+'-wasm-consumer',root+'/hypercurve',['build','--locked','--offline','--release','--lib','--features','triangulation,svg,hershey','--target','wasm32-unknown-unknown']);
 }
} else {
 for(const variant of ['baseline','candidate'])for(const crate of ['hypersolve','hypercurve'])
  add(variant+'-'+crate+'-release',(variant==='baseline'?o.baseline:o.candidate)+'/'+crate,
   ['test','--locked','--offline','--release','--all-features','--no-fail-fast','--','--test-threads=2']);
 add('candidate-consumer-clippy',o.candidate+'/hypercurve',['clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']);
}
for(const j of jobs) {
 await new Promise((ok,fail)=>{
  const c=spawn(process.execPath,['capture.mjs',j.tag,j.cwd,j.command,...j.args],{stdio:'inherit'});c.on('error',fail);
  c.on('close',(code,signal)=>code===0&&!signal?ok():fail(Error(JSON.stringify({...j,code,signal}))));
 });
}
assert.deepEqual(sources(),initial);
