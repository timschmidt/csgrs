import { spawn } from 'node:child_process';
import { writeFileSync, copyFileSync, constants, statSync, mkdtempSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { sources, sha, json } from './derivative-demand-sources.mjs';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const target='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse';
const root=mkdtempSync('/tmp/calcium-derivative-apps.');
sources();
async function captured(tag,cwd,command,args) {
 await new Promise((ok,fail)=>{
  const c=spawn(process.execPath,[resolve(here,'capture.mjs'),tag,cwd,command,...args],{stdio:'inherit'});
  c.on('error',fail);c.on('close',code=>code===0?ok():fail(Error(tag+': '+code)));
 });
}
const artifacts=[];
for(const variant of ['baseline','candidate']) {
 await captured('derivative-app-'+variant+'-build',resolve(here,'derivative-demand-'+variant+'/hypercurve'),'env',[
  'CARGO_TARGET_DIR='+target,'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2',
  'cargo','build','--offline','--locked','--release','--example','basic','--example','arrangement']);
 for(const example of ['basic','arrangement']) {
  const path=resolve(root,variant+'-'+example),stripped=path+'.stripped';
  copyFileSync(resolve(target,'release/examples',example),path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  await captured('derivative-app-'+variant+'-strip-'+example,here,'strip',['--strip-all','-o',stripped,path]);
  await captured('derivative-app-'+variant+'-size-'+example,here,'size',[path,stripped]);
  await captured('derivative-app-'+variant+'-run-'+example,here,stripped,[]);
  artifacts.push({variant,example,files:[path,stripped].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}))});
 }
}
const state=[];
for(const variant of ['baseline','candidate']) {
 assert.equal(json('results/derivative-state-'+variant+'-release.json').code,0);
 const path=resolve(root,variant+'-state');
 copyFileSync(resolve(target,'release/calcium-derivative-state-'+variant),path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 state.push({variant,path,bytes:statSync(path).size,sha256:sha(path)});
}
sources();
writeFileSync(resolve(here,'derivative-app-size-summary.json'),JSON.stringify({
 recorded:new Date().toISOString(),root,artifacts,state,
 limits:'Matched frozen Hypercurve basic/arrangement examples, default-feature standard release profile and separately stripped copies. Same retained scalar and Hypersolve dependencies. Not complete Alumina, all-feature/LTO/size-optimized binaries or CPU benchmarks. State binaries are separately copied for memory qualification.'
},null,2)+'\n',{flag:'wx'});
