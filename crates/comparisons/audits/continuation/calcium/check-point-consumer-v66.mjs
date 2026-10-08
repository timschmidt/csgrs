import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {consumerSources,crate,sha,json} from './point-consumer-sources-v66.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
import {coldBindings} from './point-cold-protocol.mjs';
import {wasmBindings} from './point-wasm-protocol.mjs';
const read=p=>readFileSync(p,'utf8');
function gate(tag,command,args,cwd='.'){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.command,command);
 assert.deepEqual(g.args,args);assert.equal(g.cwd,resolve(cwd));assert.equal(g.code,0);assert.equal(g.signal,null);
 assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);assert(Date.parse(g.finished)>=Date.parse(g.started));
 return g;
}
export function graphEvidence(){
 consumerSources();const prefix='point-consumer-metadata-',snapshots={};
 for(const[v,manifest]of [['baseline','e-plan-qualified-candidate/hypercurve/Cargo.toml'],['demand',crate+'/Cargo.toml']]){
  const tag=prefix+v+'-v66';
  gate(tag,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1','--all-features','--manifest-path',manifest]);
  assert.equal(read('results/'+tag+'.stderr'),'');snapshots[v]=read('results/'+tag+'.stdout');
 }
 const normalize=s=>s.replaceAll(resolve('point-demand-consumer-v66'),resolve('e-plan-qualified-candidate'))
  .replaceAll(resolve('point-demand-candidate'),resolve('e-plan-qualified-candidate'));
 assert.deepEqual(JSON.parse(normalize(snapshots.demand)),JSON.parse(snapshots.baseline));
 const metadata=JSON.parse(snapshots.demand),names=['hypercurve','hyperlattice','hyperlimit','hyperreal','hypersolve','hypertri'];
 assert.equal(metadata.packages.length,187);assert.equal(metadata.resolve.nodes.length,187);
 assert.equal(new Set(metadata.packages.map(p=>p.id)).size,187);
 for(const name of names){const ps=metadata.packages.filter(p=>p.name===name);assert.equal(ps.length,1);
  const root=name==='hypercurve'?'point-demand-consumer-v66':name==='hypersolve'?'point-demand-candidate':'e-plan-qualified-candidate';
  assert.equal(ps[0].manifest_path,resolve(root,name,'Cargo.toml'));
 }
 assert.equal(sha(crate+'/Cargo.lock'),sha('e-plan-qualified-candidate/hypercurve/Cargo.lock'));
 return{packages:187,nodes:187,oneOfEachHyper:names,pathOnlyNormalization:true,unchangedLock:true};
}
const tests=text=>{
 const suites=[...text.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)].map(m=>m.slice(1).map(Number));
 assert(!text.includes('test result: FAILED'));
 return{suites:suites.length,totals:suites.reduce((a,s)=>a.map((n,i)=>n+s[i]),[0,0,0,0,0]),
  names:[...text.matchAll(/^test (.+) \.\.\. (ok|ignored)(?:[^\n]*)$/gm)].map(m=>m[1]+':'+m[2]).sort()};
};
function sections(tag,paths){
 gate(tag,'size',paths);assert.equal(read('results/'+tag+'.stderr'),'');
 const rows=read('results/'+tag+'.stdout').trimEnd().split('\n').slice(1).map(line=>{
  const m=line.match(/^\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+([0-9a-f]+)\s+(.+)$/);assert(m);
  const[text,data,bss,total]=m.slice(1,5).map(Number);assert.equal(text+data+bss,total);assert.equal(parseInt(m[5],16),total);
  return{text,data,bss,total,path:m[6]};
 });assert.deepEqual(rows.map(r=>r.path),paths);return rows;
}
export function consumerEvidence(){
 const source=consumerSources(),graph=graphEvidence(),manifest=crate+'/Cargo.toml';
 const release=gate('point-consumer-release-v66','env',[...cargoEnv,'cargo','test','--offline','--locked','--manifest-path',manifest,
  '--release','--all-features','--no-fail-fast','--','--test-threads=2']);
 const current=tests(read('results/point-consumer-release-v66.stdout')),prior=tests(read('results/point-qualified-consumer-release.stdout'));
 assert.deepEqual(current,prior);assert.equal(current.suites,46);assert.deepEqual(current.totals,[1764,0,9,0,0]);
 assert.equal(current.names.length,1773);
 gate('point-consumer-fmt-v66','env',[...cargoEnv,'cargo','fmt','--manifest-path',manifest,'--','--check']);
 const clippy=gate('point-consumer-clippy-v66','env',[...cargoEnv,'cargo','clippy','--offline','--locked','--manifest-path',manifest,
  '--all-targets','--all-features','--','-D','warnings']);
 const wasmGate=gate('point-consumer-wasm-v66','env',[...cargoEnv,'cargo','build','--offline','--locked','--manifest-path',manifest,
  '--release','--lib','--features','triangulation,svg,hershey','--target','wasm32-unknown-unknown']);
 for(const tag of ['point-consumer-release-v66','point-consumer-clippy-v66','point-consumer-wasm-v66'])assert(!read('results/'+tag+'.stderr').includes('warning:'));
 assert(Date.parse(clippy.started)>=Date.parse(release.finished));assert(Date.parse(wasmGate.started)>=Date.parse(clippy.finished));
 const a=json('point-consumer-apps-v66.json'),o=json('point-consumer-app-origin-v66.json'),old=json('point-qualified-apps-summary.json');
 assert.equal(a.root,o.root);assert.equal(a.sourceBindingSha256,sha('point-consumer-binding-v66.json'));
 assert.equal(o.sourceBindingSha256,a.sourceBindingSha256);assert.equal(a.priorAppsSha256,sha('point-qualified-apps-summary.json'));
 assert.equal(a.baselineAppsSha256,sha('e-qualified-app-size-summary.json'));assert.equal(a.artifacts.length,6);
 gate('point-consumer-app-build-v66','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--example','basic','--example','arrangement'],crate);
 const applicationSizes=[];
 for(const example of ['basic','arrangement']){
  const artifacts=['baseline','eager','demand'].map(variant=>{const fs=a.artifacts.filter(f=>f.variant===variant&&f.example===example);
   assert.equal(fs.length,1);const artifact=fs[0];assert.equal(artifact.crate,'hypercurve');assert.equal(artifact.files.length,2);
   for(const f of artifact.files){assert.equal(statSync(f.path).size,f.bytes);assert.equal(sha(f.path),f.sha256);}
   if(variant!=='demand')assert.deepEqual(artifact.files,old.artifacts.find(f=>f.variant===(variant==='eager'?'candidate':'baseline')&&f.example===example&&f.crate==='hypercurve').files);
   gate(`point-consumer-${example}-${variant}-run-v66`,artifact.files[1].path,[]);
   assert.equal(read(`results/point-consumer-${example}-${variant}-run-v66.stderr`),'');
   assert.equal(sha(`results/point-consumer-${example}-${variant}-run-v66.stdout`),sha(`results/point-consumer-${example}-demand-run-v66.stdout`));
   return artifact;
  });
  const [baseline,eager,demand]=artifacts;
  assert.equal(demand.files[0].path,a.root+'/demand-'+example);assert.equal(demand.files[1].path,demand.files[0].path+'.stripped');
  gate(`point-consumer-${example}-strip-v66`,'strip',['--strip-all','-o',demand.files[1].path,demand.files[0].path]);
  applicationSizes.push({example,artifacts,baselineDelta:demand.files.map((f,i)=>f.bytes-baseline.files[i].bytes),
   eagerDelta:demand.files.map((f,i)=>f.bytes-eager.files[i].bytes),sections:sections(`point-consumer-${example}-size-v66`,artifacts.flatMap(a=>a.files.map(f=>f.path)))});
 }
 const cold=coldBindings(),wasm=wasmBindings();
 assert.equal(a.reusedCold.bindingSha256,sha('point-cold-binaries.json'));assert.deepEqual(a.reusedCold.artifacts,cold.artifacts);
 assert.equal(a.reusedWasm.bindingSha256,sha('point-wasm-binaries.json'));assert.deepEqual(a.reusedWasm.artifacts,wasm.artifacts);
 const coldSections=sections('point-consumer-cold-size-v66',cold.artifacts.filter(a=>a.platform==='native').map(a=>a.path));
 gate('point-consumer-environment-v66','node',['point-qualified-environment.mjs']);
 const environment=json('results/point-consumer-environment-v66.stdout');
 assert.equal(environment.platform,'linux');assert.equal(environment.arch,'x64');assert.equal(environment.versions.length,4);
 const dedicated=a.artifacts.filter(a=>a.variant==='demand').flatMap(a=>a.files);assert.equal(dedicated.length,4);
 assert.equal(a.dedicatedBytes,dedicated.reduce((n,f)=>n+f.bytes,0));
 assert.deepEqual(consumerSources(),source);
 return{checkpoint:66,status:'pass',sourceBindingSha256:sha('point-consumer-binding-v66.json'),graph,
  consumer:{suites:46,passed:1764,ignored:9,namesUnchanged:true,release:true,clippy:'all targets/all features, warnings denied',
   wasm:'release library build only; triangulation,svg,hershey'},applicationSizes,coldSections,
  frozenCold:executableSizes(cold.artifacts),frozenHistoryWasm:executableSizes(wasm.artifacts.map(a=>({...a,platform:'wasm'}))),
  dedicatedBytes:a.dedicatedBytes,environment,limits:a.limits};
}
function executableSizes(artifacts){return[...new Set(artifacts.map(a=>a.platform))].map(platform=>{
 const select=variant=>artifacts.find(a=>a.platform===platform&&a.variant===variant);
 const b=select('baseline'),e=select('eager'),d=select('demand');assert(b&&e&&d);
 return{platform,baselineBytes:b.bytes,eagerBytes:e.bytes,demandBytes:d.bytes,baselineDelta:d.bytes-b.bytes,eagerDelta:d.bytes-e.bytes};
});}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(process.argv.includes('--graph-only')?graphEvidence():consumerEvidence()));
