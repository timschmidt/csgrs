import {readFileSync,readdirSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
import {wasmBindings,variants,flags} from './point-wasm-protocol.mjs';
import {checkWasmCosts} from './check-point-wasm-costs.mjs';
const read=p=>readFileSync(p,'utf8');
export async function pointWasmEvidence(){
 const b=wasmBindings(),specs=[];
 const add=(tag,command,args,cwd='.')=>specs.push({tag:'point-wasm-'+tag,cwd:resolve(cwd),command,args,code:0});
 add('build','node',['build-point-wasm.mjs']);
 for(const v of variants){const app='point-wasm-'+v;
  add(v+'-lock','env',[...cargoEnv,'cargo','generate-lockfile','--offline'],app);
  add(v+'-build','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--target','wasm32-unknown-unknown','--lib'],app);
  add(v+'-clippy','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--target','wasm32-unknown-unknown','--lib','--','-D','warnings'],app);
 }
 add('qualify','taskset',['-c','6','node',...flags,'qualify-point-wasm.mjs']);
 add('qualification-check','node',['check-point-wasm-qualification.mjs']);
 add('campaign','node',['run-point-wasm-campaign.mjs']);
 add('cost','taskset',['-c','6','node',...flags,'run-point-wasm-costs.mjs']);
 for(const tag of ['environment','environment-after'])add(tag,'node',['point-image-cost-environment.mjs']);
 add('capacity','df',['-B1','/tmp','.']);
 add('cost-check','node',['check-point-wasm-costs.mjs']);
 add('format','rustfmt',['--edition','2024','--check','point-wasm-work.rs','point-wasm-platform.rs']);
 const gates={};
 for(const s of specs){const g=json('results/'+s.tag+'.json');gates[s.tag]=g;
  for(const k of ['tag','cwd','command','args','code'])assert.deepEqual(g[k],s[k],s.tag+' '+k);
  assert.equal(g.signal,null);assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));
 }
 for(const v of variants)for(const suffix of ['build','clippy'])assert(!read('results/point-wasm-'+v+'-'+suffix+'.stderr').includes('warning:'));
 for(const tag of ['qualify','qualification-check','cost','cost-check','format'])assert.equal(read('results/point-wasm-'+tag+'.stderr'),'');
 for(const[a,z]of [['build','qualify'],['qualify','qualification-check'],['qualification-check','cost'],['environment','cost'],['cost','environment-after'],['campaign','cost-check']])
  assert(Date.parse(gates['point-wasm-'+a].finished)<=Date.parse(gates['point-wasm-'+z].started));
 assert.deepEqual(readdirSync(b.root).sort(),variants.map(v=>v+'.wasm').sort());
 const normalizedLock=v=>read('point-wasm-'+v+'/Cargo.lock').replace('name = "calcium-point-wasm-'+v+'"','name = "calcium-point-wasm-COMMON"');
 for(const v of variants)assert.equal(normalizedLock(v),normalizedLock('baseline'),'matched dependency locks '+v);
 const costs=await checkWasmCosts();assert.deepEqual(costs,JSON.parse(read('results/point-wasm-cost-check.stdout')));
 const qualification=json('point-wasm-qualification.json');
 assert.deepEqual(JSON.parse(read('results/point-wasm-qualification-check.stdout')),{status:qualification.status,observations:qualification.observations,
  independentChecks:qualification.totalChecks,groups:qualification.groups,rowsSha256:qualification.rowsSha256,
  binariesSha256:qualification.binariesSha256,runtime:qualification.runtime});
 const environments=['environment','environment-after'].map(tag=>JSON.parse(read('results/point-wasm-'+tag+'.stdout')));
 for(const e of environments){assert.equal(e.platform,'linux');assert.equal(e.arch,'x64');assert(e.cpuCount>6);}
 const rawPaths=['results/point-wasm-qualification.jsonl','results/point-wasm-qualification-failures.jsonl',
  'results/point-wasm-cost.jsonl','results/point-wasm-cost-pilots.jsonl','results/point-wasm-cost-failures.jsonl'];
 const dedicatedBytes=b.artifacts.reduce((n,a)=>n+a.bytes,0),rawBytes=rawPaths.reduce((n,p)=>n+statSync(p).size,0);
 assert.equal(dedicatedBytes,4843385);
 return{gates:specs.map(s=>s.tag),sourceBindingSha256:sha('point-demand-source-binding.json'),sourceFiles:175,
  artifacts:b.artifacts,dedicatedBytes,rawBytes,qualification,costs,environments,
  next:'Extended optimized-tier WASM costs are measured, not inferred from correctness. Investigate volatile paired/marginal discrepancies with a bounded diagnostic replay of both unfavorable and favorable controls, preserving all original samples; do not attribute them to the algorithm yet. Then qualify final consumers and representative sizes for the selected demand version before retention. Separate power-sum work and all remaining original references/support reads and inventory reconciliation remain open.'};
}
