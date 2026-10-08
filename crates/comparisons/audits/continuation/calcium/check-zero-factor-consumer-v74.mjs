import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {consumerSources,crate,baseline,sha,json} from './zero-factor-consumer-sources-v74.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
const read=p=>readFileSync(p,'utf8');
export function gate(tag,command,args,cwd='.'){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(g.signal,null);assert(!g.error);
 assert.equal(g.cwd,resolve(cwd));assert.equal(g.command,command);assert.deepEqual(g.args,args);
 assert(Date.parse(g.finished)>=Date.parse(g.started));assert(g.elapsedSeconds>=0);return g;
}
function tests(text){
 const suites=[...text.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)].map(m=>m.slice(1).map(Number));
 assert(!text.includes('test result: FAILED'));
 return{suites:suites.length,totals:suites.reduce((a,s)=>a.map((n,i)=>n+s[i]),[0,0,0,0,0]),
  names:[...text.matchAll(/^test (.+) \.\.\. (ok|ignored)(?:[^\n]*)$/gm)].map(m=>m[1]+':'+m[2]).sort()};
}
function sections(tag,paths){
 gate(tag,'size',paths);assert.equal(read('results/'+tag+'.stderr'),'');
 const rows=read('results/'+tag+'.stdout').trimEnd().split('\n').slice(1).map(line=>{
  const m=line.match(/^\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+([0-9a-f]+)\s+(.+)$/);assert(m);
  const[text,data,bss,total]=m.slice(1,5).map(Number);assert.equal(text+data+bss,total);assert.equal(parseInt(m[5],16),total);
  return{text,data,bss,total,path:m[6]};
 });assert.deepEqual(rows.map(r=>r.path),paths);return rows;
}
export function consumerEvidence(){
 const source=consumerSources(),origin=json('zero-factor-consumer-run-origin-v74.json');assert.deepEqual(origin.source,source);
 assert.equal(origin.scriptSha256,sha('qualify-zero-factor-consumer-v74.mjs'));assert.equal(origin.environmentScriptSha256,sha('point-qualified-environment.mjs'));
 const manifest=crate+'/Cargo.toml',metas={};
 for(const[v,path]of [['baseline',baseline+'/Cargo.toml'],['candidate',manifest]]){
  const tag='zero-factor-consumer-'+v+'-metadata-v74';gate(tag,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--all-features','--format-version','1','--manifest-path',path]);
  assert.equal(read('results/'+tag+'.stderr'),'');metas[v]=read('results/'+tag+'.stdout');
 }
 const candidate=JSON.parse(metas.candidate),normalized=JSON.parse(metas.candidate.replaceAll(resolve(crate),resolve(baseline))
  .replaceAll(resolve('zero-factor-candidate-v70/hypersolve'),resolve('point-demand-candidate/hypersolve')));
 assert.deepEqual(normalized,JSON.parse(metas.baseline));assert.equal(candidate.packages.length,187);assert.equal(candidate.resolve.nodes.length,187);
 const names=['hypercurve','hyperreal','hyperlattice','hyperlimit','hypersolve','hypertri'];
 for(const name of names){const ps=candidate.packages.filter(p=>p.name===name);assert.equal(ps.length,1);
  const path=name==='hypercurve'?crate:name==='hypersolve'?'zero-factor-candidate-v70/hypersolve':'e-plan-qualified-candidate/'+name;
  assert.equal(ps[0].manifest_path,resolve(path,'Cargo.toml'));
 }
 assert.equal(sha(crate+'/Cargo.lock'),sha(baseline+'/Cargo.lock'));
 const release=gate('zero-factor-consumer-release-v74','env',[...cargoEnv,'cargo','test','--offline','--locked','--manifest-path',manifest,
  '--release','--all-features','--no-fail-fast','--','--test-threads=2']);
 const current=tests(read('results/zero-factor-consumer-release-v74.stdout')),prior=tests(read('results/point-consumer-release-v66.stdout'));
 assert.deepEqual(current,prior);assert.equal(current.suites,46);assert.deepEqual(current.totals,[1764,0,9,0,0]);assert.equal(current.names.length,1773);
 gate('zero-factor-consumer-fmt-v74','env',[...cargoEnv,'cargo','fmt','--manifest-path',manifest,'--','--check']);
 const clippy=gate('zero-factor-consumer-clippy-v74','env',[...cargoEnv,'cargo','clippy','--offline','--locked','--manifest-path',manifest,
  '--all-targets','--all-features','--','-D','warnings']);
 const wasm=gate('zero-factor-consumer-wasm-v74','env',[...cargoEnv,'cargo','build','--offline','--locked','--manifest-path',manifest,
  '--release','--lib','--features','triangulation,svg,hershey','--target','wasm32-unknown-unknown']);
 for(const tag of ['zero-factor-consumer-release-v74','zero-factor-consumer-clippy-v74','zero-factor-consumer-wasm-v74'])assert(!read('results/'+tag+'.stderr').includes('warning:'));
 assert(Date.parse(clippy.started)>=Date.parse(release.finished));assert(Date.parse(wasm.started)>=Date.parse(clippy.finished));
 const app=json('zero-factor-consumer-apps-v74.json'),ao=json('zero-factor-consumer-app-origin-v74.json'),old=json('point-consumer-apps-v66.json');
 assert.equal(app.root,ao.root);assert.equal(app.sourceBindingSha256,sha('zero-factor-consumer-binding-v74.json'));assert.equal(ao.sourceBindingSha256,app.sourceBindingSha256);
 assert.equal(app.baselineAppsSha256,sha('point-consumer-apps-v66.json'));assert.equal(ao.baselineAppsSha256,app.baselineAppsSha256);assert.equal(app.artifacts.length,4);
 gate('zero-factor-consumer-app-build-v74','env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--example','basic','--example','arrangement'],crate);
 const sizes=[];
 for(const example of ['basic','arrangement']){
  const artifacts=['baseline','candidate'].map(variant=>{
   const selected=app.artifacts.filter(a=>a.variant===variant&&a.example===example);assert.equal(selected.length,1);const a=selected[0];assert.equal(a.crate,'hypercurve');assert.equal(a.files.length,2);
   for(const f of a.files){assert.equal(statSync(f.path).size,f.bytes);assert.equal(sha(f.path),f.sha256);}
   if(variant==='baseline')assert.deepEqual(a.files,old.artifacts.find(o=>o.variant==='demand'&&o.example===example).files);
   gate('zero-factor-consumer-'+example+'-'+variant+'-run-v74',a.files[1].path,[]);
   assert.equal(read('results/zero-factor-consumer-'+example+'-'+variant+'-run-v74.stderr'),'');return a;
  });
  const[b,c]=artifacts;assert.equal(sha('results/zero-factor-consumer-'+example+'-baseline-run-v74.stdout'),sha('results/zero-factor-consumer-'+example+'-candidate-run-v74.stdout'));
  assert.equal(c.files[0].path,app.root+'/candidate-'+example);assert.equal(c.files[1].path,c.files[0].path+'.stripped');
  gate('zero-factor-consumer-'+example+'-strip-v74','strip',['--strip-all','-o',c.files[1].path,c.files[0].path]);
  sizes.push({example,artifacts,deltas:c.files.map((f,i)=>f.bytes-b.files[i].bytes),sections:sections('zero-factor-consumer-'+example+'-size-v74',artifacts.flatMap(a=>a.files.map(f=>f.path)))});
 }
 const environment={};for(const phase of ['before','after']){
  const tag='zero-factor-consumer-environment-'+phase+'-v74';gate(tag,'node',['point-qualified-environment.mjs']);environment[phase]=json('results/'+tag+'.stdout');
  assert.equal(environment[phase].platform,'linux');assert.equal(environment[phase].arch,'x64');assert.equal(environment[phase].versions.length,4);
 }
 assert.deepEqual(environment.before.versions,environment.after.versions);
 const oldEnvironment=json('results/point-consumer-environment-v66.stdout');
 assert.deepEqual(environment.before.versions,oldEnvironment.versions);assert.equal(environment.before.platform,oldEnvironment.platform);assert.equal(environment.before.arch,oldEnvironment.arch);
 const dedicated=app.artifacts.filter(a=>a.variant==='candidate').flatMap(a=>a.files);assert.equal(dedicated.length,4);assert.equal(app.dedicatedBytes,dedicated.reduce((n,f)=>n+f.bytes,0));
 const childTags=['zero-factor-consumer-environment-before-v74','zero-factor-consumer-baseline-metadata-v74','zero-factor-consumer-candidate-metadata-v74',
  'zero-factor-consumer-release-v74','zero-factor-consumer-fmt-v74','zero-factor-consumer-clippy-v74','zero-factor-consumer-wasm-v74','zero-factor-consumer-app-build-v74',
  ...['basic','arrangement'].flatMap(e=>['strip','baseline-run','candidate-run','size'].map(k=>'zero-factor-consumer-'+e+'-'+k+'-v74')),'zero-factor-consumer-environment-after-v74'];
 assert.equal(childTags.length,17);
 const outer=gate('zero-factor-consumer-qualification-v74','node',['qualify-zero-factor-consumer-v74.mjs']),rows=read('results/zero-factor-consumer-qualification-v74.stdout').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,35);assert.deepEqual(rows.at(-1),app);assert.equal(read('results/zero-factor-consumer-qualification-v74.stderr'),'');
 let previous=Date.parse(outer.started);for(const [i,t]of childTags.entries()){
  const g=json('results/'+t+'.json');assert.deepEqual(rows[i*2+1],g);assert(Date.parse(g.started)>=previous);previous=Date.parse(g.finished);
  const begin=rows[i*2];assert.deepEqual({...begin,pid:undefined},{tag:t,started:g.started,cwd:g.cwd,command:g.command,args:g.args,pid:undefined});assert(Number.isInteger(begin.pid)&&begin.pid>0);
 }assert(Date.parse(outer.finished)>=previous);
 const callerPaths=['point-demand-candidate/hypersolve/src/algebraic.rs','point-demand-candidate/hypersolve/src/algebraic_rational_image.rs',
  ...['src/bezier_algebraic_image.rs','src/bezier_offset.rs','src/bezier_arrangement.rs','src/bezier_tangent_order.rs','examples/basic.rs','examples/arrangement.rs'].map(p=>baseline+'/'+p)];
 for(const p of callerPaths){const destination=p.startsWith('point-demand-candidate/')?p.replace('point-demand-candidate','zero-factor-candidate-v70'):p.replace(baseline,crate);assert.equal(sha(p),sha(destination));}
 assert.deepEqual(consumerSources(),source);
 return {checkpoint:74,status:'isolated-consumer-size-qualified',sourceBindingSha256:sha('zero-factor-consumer-binding-v74.json'),
  liveFiles:956,solverCandidateFiles:176,consumerFiles:355,copiedBytes:source.copiedBytes,gates:[...childTags,'zero-factor-consumer-qualification-v74'],
  graph:{packages:187,nodes:187,oneOfEachHyper:names,fullPathNormalizedMetadataEqual:true,lockEqual:true},
  tests:{suites:46,passed:1764,ignored:9,failed:0,names:1773,namesAndOutcomesUnchanged:true},
  lint:'all targets/all features, warnings denied',wasm:'release library build only; triangulation,svg,hershey',sizes,
  callerReview:{path:'zero-factor-consumer-caller-review-v74.md',sha256:sha('zero-factor-consumer-caller-review-v74.md'),sources:Object.fromEntries(callerPaths.map(p=>[p,sha(p)])),newDonorLines:0},
  dedicatedFiles:4,dedicatedBytes:app.dedicatedBytes,environment,baselineToolVersionsMatch:true,retainedContinuationTransfers:6,limits:app.limits};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(consumerEvidence()));
