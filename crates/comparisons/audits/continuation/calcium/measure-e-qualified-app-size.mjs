import {spawn} from 'node:child_process';
import {copyFileSync,constants,existsSync,statSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {sources,sha,json} from './e-plan-qualified-sources.mjs';
import assert from 'node:assert/strict';
const initial=sources(),o=json('e-plan-qualified-origin.json'),target='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse';
// Resume the sole directory created by the preserved failed tag-validation
// attempt. Its successful baseline build and original executable stay intact.
const root='/tmp/calcium-e-qualified-apps.AHiwfn',artifacts=[];
assert(statSync(root).isDirectory());
async function captured(tag,cwd,command,args) {
 if(existsSync('results/'+tag+'.json')) {
  const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
  assert.equal(g.cwd,resolve(cwd));assert.equal(g.command,command);assert.deepEqual(g.args,args);
  console.log(JSON.stringify({reusedGate:tag,finished:g.finished}));return;
 }
 await new Promise((ok,fail)=>{
  const c=spawn(process.execPath,['capture.mjs',tag,cwd,command,...args],{stdio:'inherit'});c.on('error',fail);
  c.on('close',(code,signal)=>code===0&&!signal?ok():fail(Error(JSON.stringify({tag,code,signal}))));
 });
}
for(const variant of ['baseline','candidate'])for(const[crate,examples]of [['hyperreal',['readme_quickstart']],['hypercurve',['basic','arrangement']]]) {
 const source=(variant==='baseline'?o.baseline:o.candidate)+'/'+crate;
 await captured('e-qualified-app-'+variant+'-'+crate+'-build',source,'env',[
  'CARGO_TARGET_DIR='+target,'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2',
  'cargo','build','--locked','--offline','--release',...examples.flatMap(e=>['--example',e])]);
 for(const example of examples) {
  const path=root+'/'+variant+'-'+crate+'-'+example,stripped=path+'.stripped';
  if(existsSync(path))assert.equal(sha(path),sha(target+'/release/examples/'+example));
  else copyFileSync(target+'/release/examples/'+example,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  const tagExample=example.replaceAll('_','-');
  await captured('e-qualified-app-'+variant+'-'+crate+'-'+tagExample+'-strip','.','strip',['--strip-all','-o',stripped,path]);
  await captured('e-qualified-app-'+variant+'-'+crate+'-'+tagExample+'-size','.','size',[path,stripped]);
  await captured('e-qualified-app-'+variant+'-'+crate+'-'+tagExample+'-run','.',stripped,[]);
  artifacts.push({variant,crate,example,files:[path,stripped].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}))});
 }
}
assert.deepEqual(sources(),initial);
writeFileSync('e-qualified-app-size-summary.json',JSON.stringify({recorded:new Date().toISOString(),root,sourceMap:initial,artifacts,
 limits:'Three unchanged source examples, default features, standard release profile and separately stripped copies. All execute their built-in assertions. Not complete Alumina, all-feature/LTO/size-optimized apps or whole-application timing. These examples do not specifically exercise a deep e query; the separate scalar oracle and lifecycle benchmarks do. Linker layout and build-path effects are included.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({root,artifacts}));
