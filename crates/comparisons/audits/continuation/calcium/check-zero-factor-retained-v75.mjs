import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {retainedSources,workspace,sha,json} from './zero-factor-retained-sources-v75.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
const read=p=>readFileSync(p,'utf8'),live=workspace+'/hypersolve';
function gate(tag,cwd,command,args,code=0){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.cwd,resolve(cwd));assert.equal(g.command,command);
 assert.deepEqual(g.args,args);assert.equal(g.code,code);assert.equal(g.signal,null);assert(!g.error);
 assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);
 assert(Number.isFinite(Date.parse(g.started)));assert(Date.parse(g.finished)>=Date.parse(g.started));return g;
}
function testRecords(text,count){
 const suites=[...text.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)].map(m=>m.slice(1).map(Number));
 assert.equal(suites.length,8);assert.deepEqual(suites.reduce((a,s)=>a.map((n,i)=>n+s[i]),[0,0,0,0,0]),[count,0,0,0,0]);
 const names=[...text.matchAll(/^test (.+) \.\.\. (ok|ignored)(?:[^\n]*)$/gm)].map(m=>m[1]+':'+m[2]).sort();
 assert.equal(names.length,count);assert.equal(new Set(names).size,count);assert(names.every(n=>n.endsWith(':ok')));
 assert(!text.includes('test result: FAILED'));return names;
}
export function retentionEvidence(){
 const source=retainedSources(),qualified=json('zero-factor-consumer-v74-manifest.json'),gates=[],tests={};
 const original=json('zero-factor-retained-run-origin-v75.json'),run=json('zero-factor-retained-run-origin-approved-v75.json');
 for(const o of [original,run]){
  assert.equal(o.checkpoint,75);assert.deepEqual(o.source,source);assert.deepEqual(o.cargoEnv,cargoEnv);
  for(const[p,h]of Object.entries(o.files))assert.equal(sha(p),h,p);
 }
 const failedEnvironment=gate('zero-factor-retained-environment-before-v75','.','node',['point-qualified-environment.mjs'],1);
 assert(read('results/zero-factor-retained-environment-before-v75.stderr').includes('Error: spawnSync rustc EPERM'));
 assert.equal(read('results/zero-factor-retained-environment-before-v75.stdout'),'');
 const failedDriver=gate('zero-factor-retained-gates-v75','.','node',['run-zero-factor-retained-v75.mjs'],1);
 assert(read('results/zero-factor-retained-gates-v75.stderr').includes('zero-factor-retained-environment-before-v75'));
 assert(Date.parse(failedEnvironment.started)>=Date.parse(failedDriver.started));
 assert(Date.parse(failedEnvironment.finished)<=Date.parse(failedDriver.finished));
 gates.push(gate('zero-factor-retained-environment-before-approved-v75','.','node',['point-qualified-environment.mjs']));
 for(const profile of ['default','all-features','release']){
  const tag='zero-factor-retained-'+profile+'-v75',count=profile==='default'?817:818;
  gates.push(gate(tag,live,'env',[...cargoEnv,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),
   ...(profile==='default'?[]:['--all-features']),'--no-fail-fast','--','--test-threads=2']));
  const text=read('results/'+tag+'.stdout');tests[profile]=testRecords(text,count);
  assert.deepEqual(tests[profile],testRecords(read('results/zero-factor-final-'+profile+'-v70.stdout'),count));
  assert(!read('results/'+tag+'.stderr').includes('warning:'));
  assert.throws(()=>testRecords(text.replace(/^(test .+ \.\.\.) ok$/m,'$1 ignored'),count));
  assert.throws(()=>testRecords(text.replace('test result: ok.','test result: FAILED.'),count));
 }
 assert.deepEqual(tests['all-features'],tests.release);
 assert.deepEqual(tests['all-features'].filter(n=>!tests.default.includes(n)),[
  'tensor_resultant::tests::dense_tensor_multiplication_does_not_refine_opaque_coefficients_for_zero_pruning:ok']);
 assert(tests.default.every(n=>tests['all-features'].includes(n)));
 gates.push(gate('zero-factor-retained-clippy-v75',live,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']));
 gates.push(gate('zero-factor-retained-fmt-v75',live,'env',[...cargoEnv,'cargo','fmt','--','--check']));
 gates.push(gate('zero-factor-retained-wasm-v75',live,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--lib','--all-features','--target','wasm32-unknown-unknown']));
 for(const tag of ['zero-factor-retained-clippy-v75','zero-factor-retained-wasm-v75'])assert(!read('results/'+tag+'.stderr').includes('warning:'));
 assert.equal(read('results/zero-factor-retained-fmt-v75.stdout'),'');assert.equal(read('results/zero-factor-retained-fmt-v75.stderr'),'');
 const metadata={};
 for(const[variant,cwd]of [['candidate','zero-factor-candidate-v70/hypersolve'],['live',live]]){
  const tag='zero-factor-retained-metadata-'+variant+'-v75';gates.push(gate(tag,cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1','--all-features']));
  assert.equal(read('results/'+tag+'.stderr'),'');metadata[variant]=read('results/'+tag+'.stdout');
 }
 const normalized=metadata.candidate.replaceAll(resolve('zero-factor-candidate-v70'),workspace).replaceAll(resolve('e-plan-qualified-candidate'),workspace);
 assert.deepEqual(JSON.parse(normalized),JSON.parse(metadata.live));
 const graph=JSON.parse(metadata.live),hyper=graph.packages.filter(p=>p.name.startsWith('hyper'));
 assert.deepEqual(hyper.map(p=>p.name).sort(),['hyperlattice','hyperlimit','hyperreal','hypersolve']);
 assert.equal(new Set(graph.packages.map(p=>p.id)).size,graph.packages.length);
 assert.equal(new Set(graph.resolve.nodes.map(p=>p.id)).size,graph.resolve.nodes.length);
 for(const p of hyper)assert.equal(p.manifest_path,resolve(workspace,p.name,'Cargo.toml'));
 assert.equal(sha(live+'/Cargo.lock'),sha('zero-factor-candidate-v70/hypersolve/Cargo.lock'));
 gates.push(gate('zero-factor-retained-environment-after-v75','.','node',['point-qualified-environment.mjs']));
 const environment={before:json('results/zero-factor-retained-environment-before-approved-v75.stdout'),after:json('results/zero-factor-retained-environment-after-v75.stdout')};
 for(const e of [environment.before,environment.after])for(const historical of [qualified.evidence.environment.before,qualified.evidence.environment.after]){
  for(const key of ['versions','node','v8','platform','arch','kernel'])assert.deepEqual(e[key],historical[key]);
 }
 for(let i=1;i<gates.length;i++)assert(Date.parse(gates[i].started)>=Date.parse(gates[i-1].finished));
 const driver=gate('zero-factor-retained-gates-approved-v75','.','node',['run-zero-factor-retained-approved-v75.mjs']);
 assert(Date.parse(driver.started)>=Date.parse(failedDriver.finished));
 assert(Date.parse(run.recorded)>=Date.parse(driver.started));assert(Date.parse(run.recorded)<=Date.parse(gates[0].started));
 assert(Date.parse(gates[0].started)>=Date.parse(driver.started));assert(Date.parse(gates.at(-1).finished)<=Date.parse(driver.finished));
 const rows=read('results/zero-factor-retained-gates-approved-v75.stdout').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,21);assert.deepEqual(rows.at(-1),{checkpoint:75,status:'live-gates-pass',source});
 for(let i=0;i<gates.length;i++){
  assert.deepEqual(rows[2*i+1],gates[i]);const start=rows[2*i];
  for(const k of ['tag','started','cwd','command','args'])assert.deepEqual(start[k],gates[i][k]);
  assert(Number.isInteger(start.pid)&&start.pid>0);
 }
 assert.equal(read('results/zero-factor-retained-gates-approved-v75.stderr'),'');
 assert.deepEqual(retainedSources(),source);
 return {checkpoint:75,status:'retention-qualified',source,runOriginSha256:sha('zero-factor-retained-run-origin-approved-v75.json'),
  gates:[...gates.map(g=>g.tag),driver.tag],preservedFailures:[failedEnvironment.tag,failedDriver.tag],
  failureReason:'Sandbox spawnSync rustc EPERM before tests; separately captured approved rerun passes. Original runner, origin and failed outputs preserved.',
  liveTests:{default:817,allFeaturesDebug:818,allFeaturesRelease:818,suitesPerConfiguration:8,failed:0,ignored:0,namesMatchQualifiedCandidate:true},
  parserNegativeControls:6,clippy:'all targets/all features, warnings denied',wasm:'all-feature release library build only',format:'check pass',
  dependencyGraph:{packages:graph.packages.length,nodes:graph.resolve.nodes.length,pathOnlyNormalization:true,lockUnchanged:true},
  consumer:qualified.evidence.tests,consumerRerunOnLivePath:false,sizes:qualified.evidence.sizes,
  historicalEvidence:'Checkpoints 70–74 remain source-bound historical numerical/native/WASM/consumer/size qualification; not fresh live-path or timing runs.',
  environment,limits:'Only the qualified main-file diff and seven-test module promoted. All other 955 previously recorded live identities unchanged. No fresh consumer, timing/allocation campaign, universal performance bound, dedicated executable snapshot, cleanup, commit or push. The original full ecosystem audit remains incomplete.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(retentionEvidence()));
