import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {retainedSources,workspace,sha,json} from './point-retained-sources-v67.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
const read=p=>readFileSync(p,'utf8'),live=workspace+'/hypersolve';
function gate(tag,cwd,command,args){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.cwd,resolve(cwd));assert.equal(g.command,command);
 assert.deepEqual(g.args,args);assert.equal(g.code,0);assert.equal(g.signal,null);assert(!g.error);
 assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);assert(Date.parse(g.finished)>=Date.parse(g.started));return g;
}
function testRecords(text){
 const suites=[...text.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)].map(m=>m.slice(1).map(Number));
 assert.equal(suites.length,8);assert.deepEqual(suites.reduce((a,s)=>a.map((n,i)=>n+s[i]),[0,0,0,0,0]),[811,0,0,0,0]);
 const names=[...text.matchAll(/^test (.+) \.\.\. (ok|ignored)(?:[^\n]*)$/gm)].map(m=>m[1]+':'+m[2]).sort();
 assert.equal(names.length,811);assert(!text.includes('test result: FAILED'));return names;
}
export function retentionEvidence(){
 const source=retainedSources(),qualified=json('point-consumer-v66-manifest.json'),gates=[];
 for(const profile of ['debug','release']){
  const tag='point-retained-'+profile+'-v67';gates.push(gate(tag,live,'env',[...cargoEnv,'cargo','test','--offline','--locked',
   ...(profile==='release'?['--release']:[]),'--all-features','--no-fail-fast','--','--test-threads=2']));
  assert.deepEqual(testRecords(read('results/'+tag+'.stdout')),testRecords(read('results/point-demand-solver-full-'+profile+'.stdout')));
  assert(!read('results/'+tag+'.stderr').includes('warning:'));
 }
 gates.push(gate('point-retained-clippy-v67',live,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']));
 gates.push(gate('point-retained-fmt-v67',live,'env',[...cargoEnv,'cargo','fmt','--','--check']));
 gates.push(gate('point-retained-wasm-v67',live,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--lib','--all-features','--target','wasm32-unknown-unknown']));
 for(const tag of ['point-retained-clippy-v67','point-retained-wasm-v67'])assert(!read('results/'+tag+'.stderr').includes('warning:'));
 const metadata={};
 for(const[variant,cwd]of [['candidate','point-demand-candidate/hypersolve'],['live',live]]){
  const tag='point-retained-metadata-'+variant+'-v67';gates.push(gate(tag,cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1','--all-features']));
  assert.equal(read('results/'+tag+'.stderr'),'');metadata[variant]=read('results/'+tag+'.stdout');
 }
 const normalized=metadata.candidate.replaceAll(resolve('point-demand-candidate'),workspace).replaceAll(resolve('e-plan-qualified-candidate'),workspace);
 assert.deepEqual(JSON.parse(normalized),JSON.parse(metadata.live));
 const graph=JSON.parse(metadata.live),hyper=graph.packages.filter(p=>p.name.startsWith('hyper'));
 assert.deepEqual(hyper.map(p=>p.name).sort(),['hyperlattice','hyperlimit','hyperreal','hypersolve']);
 assert.equal(new Set(graph.packages.map(p=>p.id)).size,graph.packages.length);
 for(const p of hyper)assert.equal(p.manifest_path,resolve(workspace,p.name,'Cargo.toml'));
 assert.equal(sha(live+'/Cargo.lock'),sha('point-demand-candidate/hypersolve/Cargo.lock'));
 gates.push(gate('point-retained-environment-v67','.','node',['point-qualified-environment.mjs']));
 const environment=json('results/point-retained-environment-v67.stdout');
 assert.deepEqual(environment.versions,qualified.evidence.environment.versions);
 assert.equal(environment.node,qualified.evidence.environment.node);assert.equal(environment.v8,qualified.evidence.environment.v8);
 for(let i=1;i<gates.length;i++)assert(Date.parse(gates[i].started)>=Date.parse(gates[i-1].finished));
 const driver=gate('point-retained-gates-v67','.','node',['run-point-retained-v67.mjs']);
 assert(Date.parse(gates[0].started)>=Date.parse(driver.started));assert(Date.parse(gates.at(-1).finished)<=Date.parse(driver.finished));
 const rows=read('results/point-retained-gates-v67.stdout').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,17);assert.deepEqual(rows.at(-1),{checkpoint:67,status:'live-gates-pass',source});
 assert.equal(read('results/point-retained-gates-v67.stderr'),'');
 assert.deepEqual(retainedSources(),source);
 return{checkpoint:67,status:'retention-qualified',source,gates:[...gates.map(g=>g.tag),driver.tag],
  liveTests:{debug:811,release:811,suitesPerProfile:8,failed:0,ignored:0,namesMatchQualifiedCandidate:true},
  clippy:'all targets/all features, warnings denied',wasm:'all-feature release library build only',format:'check pass',
  dependencyGraph:{packages:graph.packages.length,nodes:graph.resolve.nodes.length,pathOnlyNormalization:true,lockUnchanged:true},
  consumer:qualified.evidence.consumer,consumerRerunOnLivePath:false,
  sizes:qualified.evidence.applicationSizes.map(s=>({example:s.example,baseline:s.baselineDelta,eager:s.eagerDelta})),
  environment,limits:'Live solver source is byte-identical to the fully qualified isolated candidate; all other 955 tracked source/support identities are unchanged. Hypercurve and prior numerical/benchmark runs belong to their frozen source trees and were not rerun on the live path. Earlier live-source verification flags are historical, not checks for this new state. No fresh timing/allocation campaign, universal performance bound, cleanup, commit or push. Original full ecosystem audit remains incomplete.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(retentionEvidence()));
