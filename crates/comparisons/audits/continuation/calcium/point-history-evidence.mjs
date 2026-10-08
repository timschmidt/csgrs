import {readFileSync,statSync,readdirSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
import {checkPointHistory} from './check-point-history.mjs';
const read=p=>readFileSync(p,'utf8');
export async function pointHistoryEvidence(){
 const origin=sources(),b=json('point-history-binaries.json'),specs=[];
 const add=(tag,command,args,cwd='.')=>specs.push({tag:'point-history-'+tag,cwd:resolve(cwd),command,args,code:0});
 add('build','node',['build-point-history.mjs']);add('qualify','node',['qualify-point-history.mjs']);
 add('campaign','node',['run-point-history-campaign.mjs']);
 add('cost-environment','node',['point-image-cost-environment.mjs']);
 add('cost-environment-after','node',['point-image-cost-environment.mjs']);
 add('cost-cpu','node',['run-point-history-costs.mjs','cpu']);
 add('cost-allocation','node',['run-point-history-costs.mjs','allocation']);
 add('check','node',['check-point-history.mjs']);
 add('capacity','df',['-B1','/tmp','.']);
 let lock;
 for(const variant of ['baseline','candidate']){
  const app='point-history-'+variant;
  add(variant+'-lock','env',[...cargoEnv,'cargo','generate-lockfile','--offline'],app);
  add(variant+'-build','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bins'],app);
  add(variant+'-clippy','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--','-D','warnings'],app);
  const manifest=read(app+'/Cargo.toml'),normalized=read(app+'/Cargo.lock').replaceAll('calcium-'+app,'calcium-point-history-normalized');
  if(lock===undefined)lock=normalized;else assert.equal(normalized,lock);
  for(const crate of ['hyperreal','hyperlimit','hypersolve'])assert(manifest.includes('path = "../'+origin[variant]+'/'+crate+'"'));
  for(const mode of ['cpu','allocation'])assert(manifest.includes('path = "../point-history-'+mode+'.rs"'));
  assert(manifest.includes('features = ["serde"]'));
  for(const suffix of ['build','clippy'])assert(!read('results/'+app+'-'+suffix+'.stderr').includes('warning:'),app+' '+suffix);
 }
 assert.equal(specs.length,15);
 for(const s of specs){
  const g=json('results/'+s.tag+'.json');for(const k of ['tag','cwd','command','args','code'])assert.deepEqual(g[k],s[k],s.tag+' '+k);
  assert.equal(g.signal,null);assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));
 }
 const prefix=read('point-extended.rs').split('fn run(')[0].replace('let mut visit =','let visit =').replace('use std::time::Instant;\n','');
 assert.equal(read('point-history-base.rs'),prefix.trimEnd()+'\n');
 assert.equal(read('point-history-allocation.rs').split('include!("point-history-base.rs");')[0].trim(),
  read('power-sums-allocation.rs').split('include!("power-sums-public.rs");')[0].trim());
 assert.equal(json('point-history-origin.json').root,b.root);
 assert.equal(json('point-history-origin.json').sourceOriginSha256,b.sourceOriginSha256);
 assert.deepEqual(readdirSync(b.root).sort(),['baseline-allocation','baseline-cpu','candidate-allocation','candidate-cpu']);
 const dedicatedBytes=b.artifacts.reduce((n,f)=>n+statSync(f.path).size,0);assert.equal(dedicatedBytes,12772864);
 const costs=await checkPointHistory();assert.deepEqual(costs,JSON.parse(read('results/point-history-check.stdout')));
 const build=json('results/point-history-build.json'),qualify=json('results/point-history-qualify.json');
 const campaign=json('results/point-history-campaign.json'),cpu=json('results/point-history-cost-cpu.json'),alloc=json('results/point-history-cost-allocation.json');
 assert(Date.parse(build.finished)<=Date.parse(qualify.started));assert(Date.parse(qualify.finished)<=Date.parse(campaign.started));
 assert(Date.parse(campaign.started)<=Date.parse(cpu.started));assert(Date.parse(cpu.finished)<=Date.parse(alloc.started));
 assert(Date.parse(alloc.finished)<=Date.parse(campaign.finished));
 const environments=['cost-environment','cost-environment-after'].map(tag=>JSON.parse(read('results/point-history-'+tag+'.stdout')));
 for(const e of environments){assert.equal(e.arch,'x64');assert.equal(e.platform,'linux');assert(e.cpuCount>6);}
 return{gates:specs.map(s=>s.tag),sourceOriginSha256:b.sourceOriginSha256,artifacts:b.artifacts,dedicatedBytes,
  costs,environments,qualification:json('point-history-qualification.json'),
  memoryLimits:'Separate instrumented allocation measurements, not a new Memcheck/RSS campaign. Final result remains alive; cold input graphs are not cold process-wide caches. Prior whole-collector reachability differences remain disclosed.',
  next:'Investigate demand-gated witness recovery because unchanged-result costs regress; qualify any revision before relevant extended WASM execution/costs and an explicit retention decision. Preserve the eager variant and measurements. Separate power-sum performance work and the full reference inventory remain open.'};
}
