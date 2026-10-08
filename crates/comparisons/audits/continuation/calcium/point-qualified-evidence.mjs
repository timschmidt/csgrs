import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
import {checkPublic} from './check-point-qualified-public.mjs';
import {checkApproxPublic} from './check-point-qualified-approx-public.mjs';
const read=p=>readFileSync(p,'utf8');
const artifact=f=>{assert.equal(statSync(f.path).size,f.bytes,f.path);assert.equal(sha(f.path),f.sha256,f.path);return f;};
function gate(s){
 const g=json('results/'+s.tag+'.json');
 for(const k of ['tag','cwd','command','args'])assert.deepEqual(g[k],s[k],s.tag+' '+k);
 assert.equal(g.code,0,s.tag);assert.equal(g.signal,null,s.tag);
 assert(Number.isFinite(Date.parse(g.started))&&Date.parse(g.finished)>=Date.parse(g.started));
 assert(g.elapsedSeconds>=0);assert(!g.error);
 return g;
}
function sizeRows(tag,files){
 const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 assert.equal(g.command,'size');assert.deepEqual(g.args,files.map(f=>f.path));
 const rows=read('results/'+tag+'.stdout').trim().split('\n').slice(1).map(line=>{
  const m=line.match(/^\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+([0-9a-f]+)\s+(.+)$/);assert(m,line);
  const [text,data,bss,decimal]=m.slice(1,5).map(Number);assert.equal(text+data+bss,decimal);assert.equal(parseInt(m[5],16),decimal);
  return{text,data,bss,decimal,path:m[6]};
 });
 assert.deepEqual(rows.map(r=>r.path),files.map(f=>f.path));return rows;
}
export function pointQualifiedEvidence(){
 const o=sources(),specs=[];
 assert.equal(o.copiedBytes,45463105);assert.equal(o.copiedFiles,956);
 const add=(tag,cwd,command,args)=>specs.push({tag:'point-qualified-'+tag,cwd:resolve(cwd),command,args});
 const cargo=(tag,cwd,args)=>add(tag,cwd,'env',[...cargoEnv,'cargo',...args]);
 const manifest='point-image-qualified-candidate/hypercurve/Cargo.toml';
 cargo('consumer-release','.', ['test','--offline','--locked','--manifest-path',manifest,'--release','--all-features','--no-fail-fast','--','--test-threads=2']);
 cargo('consumer-wasm','.', ['build','--offline','--locked','--manifest-path',manifest,'--release','--lib','--features','triangulation,svg,hershey','--target','wasm32-unknown-unknown']);
 cargo('consumer-clippy','.', ['clippy','--offline','--locked','--manifest-path',manifest,'--all-targets','--all-features','--','-D','warnings']);
 add('apps','.','node',['measure-point-qualified-apps.mjs']);
 add('environment','.','node',['point-qualified-environment.mjs']);
 const apps=json('point-qualified-apps-summary.json'),oldApps=json('e-qualified-app-size-summary.json'),applicationSizes=[];
 assert.equal(apps.originSha256,sha('point-qualified-origin.json'));
 assert.equal(apps.reusedBaselineSummarySha256,sha('e-qualified-app-size-summary.json'));
 assert.deepEqual(oldApps.sourceMap.candidate,o.liveSources);
 assert.equal(apps.artifacts.length,6);
 for(const[crate,examples]of [['hyperreal',['readme_quickstart']],['hypercurve',['basic','arrangement']]]){
  cargo('app-'+crate+'-build',o.candidate+'/'+crate,['build','--locked','--offline','--release',...examples.flatMap(e=>['--example',e])]);
  const priorBuild=json('results/e-qualified-app-candidate-'+crate+'-build.json');
  gate({...priorBuild,tag:'e-qualified-app-candidate-'+crate+'-build',cwd:resolve(o.baseline+'/'+crate),command:'env',
   args:[...cargoEnv,'cargo','build','--locked','--offline','--release',...examples.flatMap(e=>['--example',e])]});
  for(const example of examples){
   const id=crate+'-'+example.replaceAll('_','-');
   const baseline=apps.artifacts.find(a=>a.variant==='baseline'&&a.crate===crate&&a.example===example);
   const candidate=apps.artifacts.find(a=>a.variant==='candidate'&&a.crate===crate&&a.example===example);
   assert(baseline&&candidate);baseline.files.forEach(artifact);candidate.files.forEach(artifact);
   assert.deepEqual(baseline.files,oldApps.artifacts.find(a=>a.variant==='candidate'&&a.crate===crate&&a.example===example).files);
   const [plain,stripped]=candidate.files;
   add('app-'+id+'-strip','.','strip',['--strip-all','-o',stripped.path,plain.path]);
   add('app-'+id+'-size','.','size',[plain.path,stripped.path]);
   add('app-'+id+'-baseline-run','.',baseline.files[1].path,[]);
   add('app-'+id+'-candidate-run','.',stripped.path,[]);
   assert.equal(sha('results/point-qualified-app-'+id+'-baseline-run.stdout'),sha('results/point-qualified-app-'+id+'-candidate-run.stdout'));
   for(const v of ['baseline','candidate'])assert.equal(read('results/point-qualified-app-'+id+'-'+v+'-run.stderr'),'');
   applicationSizes.push({crate,example,baseline:baseline.files.map(f=>f.bytes),candidate:candidate.files.map(f=>f.bytes),
    byteDeltas:candidate.files.map((f,i)=>f.bytes-baseline.files[i].bytes),
    baselineSections:sizeRows('e-qualified-app-candidate-'+id+'-size',baseline.files),
    candidateSections:sizeRows('point-qualified-app-'+id+'-size',candidate.files),sameStdout:true});
  }
 }
 const platformArtifacts=[],platformSizes=[];let normalizedLock;
 for(const mode of ['platform','approx']){
  const b=json('point-qualified-'+mode+'-binaries.json'),harness='point-qualified-'+mode+'.rs';
  assert.equal(b.originSha256,sha('point-qualified-origin.json'));assert.equal(b.harnessSha256,sha(harness));
  assert.equal(b.sharedCollectorSha256,sha('power-sums-public.rs'));assert.equal(b.artifacts.length,4);
  add(mode+'-build','.','node',['build-point-qualified-'+mode+'.mjs']);
  add(mode+'-run','.','node',['run-point-qualified-'+mode+'.mjs']);
  for(const variant of ['baseline','candidate']){
   const app='point-qualified-'+mode+'-'+variant;
   cargo(mode+'-'+variant+'-lock',app,['generate-lockfile','--offline']);
   cargo(mode+'-'+variant+'-native-build',app,['build','--offline','--locked','--release','--bin',app]);
   cargo(mode+'-'+variant+'-wasm-build',app,['build','--offline','--locked','--release','--target','wasm32-unknown-unknown','--lib']);
   const manifest=read(app+'/Cargo.toml');
   for(const crate of ['hyperreal','hyperlimit','hypersolve'])assert(manifest.includes(crate+' = { path = "../'+o[variant]+'/'+crate+'" }'));
   assert(manifest.includes('path = "../'+harness+'"'));
   const lock=read(app+'/Cargo.lock').replaceAll('calcium-'+app,'calcium-point-qualified-normalized');
   if(normalizedLock===undefined)normalizedLock=lock;else assert.equal(lock,normalizedLock,app+' dependency lock equality');
   const native=b.artifacts.find(f=>f.variant===variant&&f.platform==='native');
   const wasm=b.artifacts.find(f=>f.variant===variant&&f.platform==='wasm');
   [native,wasm].forEach(artifact);platformArtifacts.push(...[native,wasm].map(f=>({mode,...f})));
   assert.equal(readFileSync(native.path).subarray(0,4).toString('hex'),'7f454c46');
   assert.equal(readFileSync(wasm.path).subarray(0,4).toString('hex'),'0061736d');
   const prefix=mode==='platform'?'public-':'approx-public-';
   add(prefix+variant+'-native','.',native.path,[]);
   add(prefix+variant+'-wasm','.','node',[mode==='platform'?'run-point-qualified-wasm.mjs':'run-point-qualified-approx-wasm.mjs',variant]);
  }
  add(mode==='platform'?'public-check':'approx-public-check','.','node',
   [mode==='platform'?'check-point-qualified-public.mjs':'check-point-qualified-approx-public.mjs']);
  for(const platform of ['native','wasm']){
   const a=b.artifacts.find(f=>f.platform===platform&&f.variant==='baseline'),z=b.artifacts.find(f=>f.platform===platform&&f.variant==='candidate');
   platformSizes.push({mode,platform,baselineBytes:a.bytes,candidateBytes:z.bytes,byteDelta:z.bytes-a.bytes});
  }
 }
 let derived=read('point-qualified-platform.rs');
 for(const old of ['query(&a, &b, operation(op))','query(&a,&b,op)']){
  assert.equal(derived.split(old).length,2);
  derived=derived.replace(old,'transform_algebraic_roots_binary('+old.slice(6,-1)+',PredicatePolicy::APPROXIMATE_512)');
 }
 assert.equal(derived,read('point-qualified-approx.rs'),'only two call-site policy changes');
 const gates=specs.map(gate);assert.equal(gates.length,45);assert.equal(new Set(gates.map(g=>g.tag)).size,45);
 const suites=[...read('results/point-qualified-consumer-release.stdout').matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out/g)].map(m=>m.slice(1).map(Number));
 const totals=suites.reduce((a,s)=>a.map((n,i)=>n+s[i]),[0,0,0,0,0]);assert.equal(suites.length,46);assert.deepEqual(totals,[1764,0,9,0,0]);
 assert(!read('results/point-qualified-consumer-release.stdout').includes('test result: FAILED'));
 const mathematical=checkPublic(),approximateMathematical=checkApproxPublic();
 assert.deepEqual(JSON.parse(read('results/point-qualified-public-check.stdout')),mathematical);
 assert.deepEqual(JSON.parse(read('results/point-qualified-approx-public-check.stdout')),approximateMathematical);
 const environment=JSON.parse(read('results/point-qualified-environment.stdout'));
 for(const prefix of ['point-qualified-public','point-qualified-approx-public'])for(const variant of ['baseline','candidate']){
  const metadata=JSON.parse(read('results/'+prefix+'-'+variant+'-wasm.stderr'));
  assert.equal(metadata.node,environment.node);assert.equal(metadata.v8,environment.v8);
 }
 const appFiles=apps.artifacts.filter(a=>a.variant==='candidate').flatMap(a=>a.files);
 const dedicatedFiles=[...appFiles,...platformArtifacts],dedicatedBytes=dedicatedFiles.reduce((n,f)=>n+f.bytes,0);
 assert.equal(dedicatedFiles.length,14);assert.equal(dedicatedBytes,68928629);
 return{gates:gates.map(g=>g.tag),sourceOriginSha256:sha('point-qualified-origin.json'),changed:o.changed,
  consumer:{passed:totals[0],failed:totals[1],ignored:totals[2],suites:46,profile:'release',features:'all',
   wasmFeatures:'default plus triangulation,svg,hershey; build only',clippy:'all targets/all features; warnings denied'},
  mathematical,approximateMathematical,applicationSizes,platformSizes,platformArtifacts,dedicatedBytes,environment,
  sizeLimits:'Matched source/configuration and hash-preserved executables, including prior baseline application reuse. Not isolated algorithm-only byte changes: paths/linker layout contribute, demonstrated by -784 stripped bytes in unchanged scalar code. No whole-application timings, LTO/size-profile qualification or generalized size gain.',
  costLimits:'No new CPU or allocator measurements. Checkpoint 53 native rational/preconditioned-cache costs remain their original observations; execution durations here are not benchmarks. Nonrational/Unknown endpoints, deeper state histories and WASM performance remain pending.'};
}
