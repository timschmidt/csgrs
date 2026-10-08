import {statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {demandSources,sha,json} from './point-demand-sources.mjs';
import {checkBindings} from './point-history-protocol.mjs';
export function demandBindings(){
 demandSources();const b=json('point-demand-binaries.json'),prior=checkBindings();
 assert.equal(b.sourceBindingSha256,sha('point-demand-source-binding.json'));assert.equal(b.artifacts.length,5);
 for(const[p,h]of Object.entries(b.harnessSources))assert.equal(sha(p),h,p);
 for(const a of b.artifacts){assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);}
 for(const tag of ['build','app-lock','app-build','app-clippy','solver-debug','solver-release','solver-clippy']){
  const g=json('results/point-demand-'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 }
 for(const mode of ['cpu','allocation']){
  for(const p of ['point-history-base.rs','point-history-work.rs','point-history-'+mode+'.rs'])assert.equal(b.harnessSources[p],prior.harnessSources[p]);
 }
 return{artifacts:[...prior.artifacts.map(a=>({...a,variant:a.variant==='candidate'?'eager':'baseline'})),...b.artifacts],demand:b};
}
