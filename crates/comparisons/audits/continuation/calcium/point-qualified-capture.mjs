import {spawn} from 'node:child_process';
import {existsSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {json} from './point-qualified-sources.mjs';
export const target='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse';
export const cargoEnv=['CARGO_TARGET_DIR='+target,'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
export async function captured(tag,cwd,command,args){
 if(existsSync('results/'+tag+'.json')){
  const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
  assert.equal(g.cwd,resolve(cwd));assert.equal(g.command,command);assert.deepEqual(g.args,args);
  console.log(JSON.stringify({reusedGate:tag,finished:g.finished}));return;
 }
 // capture.mjs refuses nonterminal output collisions before spawning anything.
 await new Promise((ok,fail)=>{
  const p=spawn(process.execPath,['capture.mjs',tag,cwd,command,...args],{stdio:'inherit'});
  p.on('error',fail);p.on('close',(code,signal)=>code===0&&!signal?ok():fail(Error(JSON.stringify({tag,code,signal}))));
 });
}
