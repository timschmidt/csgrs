import {readFileSync,statSync,readdirSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {demandSources,sha,json} from './point-demand-sources.mjs';
import {demandBindings} from './point-demand-bindings.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
import {checkDemandPublic} from './check-point-demand-public.mjs';
import {checkDemandCosts} from './check-point-demand-costs.mjs';
const read=p=>readFileSync(p,'utf8'),number=s=>Number(s.replaceAll(',',''));
function memory(tag){
 const text=read('results/'+tag+'.stderr'),get=r=>{const m=text.match(r);assert(m,tag+' '+r);return m.slice(1).map(number);};
 const [errors]=get(/ERROR SUMMARY: ([\d,]+) errors/),[liveBytes,liveBlocks]=get(/in use at exit: ([\d,]+) bytes in ([\d,]+) blocks/);
 const [allocations,frees,cumulativeBytes]=get(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/);
 const [definite]=get(/definitely lost: ([\d,]+) bytes/),[indirect]=get(/indirectly lost: ([\d,]+) bytes/),[possible]=get(/possibly lost: ([\d,]+) bytes/);
 const [reachableBytes,reachableBlocks]=get(/still reachable: ([\d,]+) bytes in ([\d,]+) blocks/);
 assert.deepEqual([errors,definite,indirect,possible],[0,0,0,0]);assert.equal(liveBytes,reachableBytes);assert.equal(liveBlocks,reachableBlocks);
 assert.equal(allocations-frees,liveBlocks);
 return{errors,definite,indirect,possible,reachableBytes,reachableBlocks,allocations,frees,cumulativeBytes};
}
function tests(tag,expected,suites){
 const matches=[...read('results/'+tag+'.stdout').matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored/g)];
 assert.equal(matches.length,suites);const totals=matches.reduce((a,m)=>a.map((n,i)=>n+Number(m[i+1])),[0,0,0]);
 assert.deepEqual(totals,[expected,0,0]);return{passed:totals[0],failed:totals[1],ignored:totals[2],suites};
}
export async function pointDemandEvidence(){
 const source=demandSources(),binding=demandBindings(),b=binding.demand,specs=[];
 const add=(tag,command,args,cwd='.')=>specs.push({tag:'point-demand-'+tag,cwd:resolve(cwd),command,args,code:0});
 add('build','node',['build-point-demand.mjs']);add('qualification','node',['run-point-demand-qualification.mjs']);
 add('history-qualify','node',['qualify-point-demand-history.mjs']);add('campaign','node',['run-point-demand-campaign.mjs']);
 add('app-lock','env',[...cargoEnv,'cargo','generate-lockfile','--offline'],'point-demand-app');
 add('app-build','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bins'],'point-demand-app');
 add('app-clippy','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--','-D','warnings'],'point-demand-app');
 add('solver-clippy','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings'],source.root+'/hypersolve');
 const solverTests={};
 for(const profile of ['debug','release']){
  const args=[...cargoEnv,'cargo','test','--locked','--offline',...(profile==='release'?['--release']:[]),'--all-features'];
  add('solver-'+profile,'env',[...args,'--lib'],source.root+'/hypersolve');
  add('solver-full-'+profile,'env',args,source.root+'/hypersolve');
  solverTests[profile]={library:tests('point-demand-solver-'+profile,447,1),full:tests('point-demand-solver-full-'+profile,811,8)};
 }
 const memories={};
 for(const mode of ['public','approx','extended']){
  const f=b.artifacts.find(f=>f.mode===mode);add(mode,f.path,[]);
  if(mode!=='public'){
   add('memory-'+mode,'valgrind',['--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',f.path]);
   memories[mode]=memory('point-demand-memory-'+mode);
   assert.equal(sha('results/point-demand-memory-'+mode+'.stdout'),sha('results/point-demand-'+mode+'.stdout'));
  }
 }
 add('public-check','node',['check-point-demand-public.mjs']);
 for(const mode of ['cpu','allocation'])add('cost-'+mode,'node',['run-point-demand-costs.mjs',mode]);
 for(const tag of ['cost-environment','cost-environment-after'])add(tag,'node',['point-image-cost-environment.mjs']);
 add('cost-check','node',['check-point-demand-costs.mjs']);add('capacity','df',['-B1','/tmp','.']);
 for(const s of specs){
  const g=json('results/'+s.tag+'.json');for(const k of ['tag','cwd','command','args','code'])assert.deepEqual(g[k],s[k],s.tag+' '+k);
  assert.equal(g.signal,null);assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));
 }
 for(const tag of ['app-build','app-clippy','solver-clippy'])assert(!read('results/point-demand-'+tag+'.stderr').includes('warning:'));
 assert.deepEqual(readdirSync(b.root).sort(),['allocation','approx','cpu','extended','public']);
 const dedicatedBytes=b.artifacts.reduce((n,f)=>n+statSync(f.path).size,0);assert.equal(dedicatedBytes,14950736);
 const publicChecks=checkDemandPublic();assert.deepEqual(publicChecks,JSON.parse(read('results/point-demand-public-check.stdout')));
 const costs=await checkDemandCosts();assert.deepEqual(costs,JSON.parse(read('results/point-demand-cost-check.stdout')));
 const gates=Object.fromEntries(specs.map(s=>[s.tag,json('results/'+s.tag+'.json')]));
 assert(Date.parse(gates['point-demand-build'].finished)<=Date.parse(gates['point-demand-qualification'].started));
 assert(Date.parse(gates['point-demand-qualification'].finished)<=Date.parse(gates['point-demand-campaign'].started));
 assert(Date.parse(gates['point-demand-history-qualify'].finished)<=Date.parse(gates['point-demand-campaign'].started));
 assert(Date.parse(gates['point-demand-cost-cpu'].finished)<=Date.parse(gates['point-demand-cost-allocation'].started));
 const environments=['cost-environment','cost-environment-after'].map(tag=>JSON.parse(read('results/point-demand-'+tag+'.stdout')));
 for(const e of environments){assert.equal(e.platform,'linux');assert.equal(e.arch,'x64');assert(e.cpuCount>6);}
 const origin=json('point-demand-origin.json');
 const parts=p=>read(p).split('#[cfg(test)]');
 const template=parts(origin.template+'/hypersolve/src/algebraic_binary.rs'),candidate=parts(source.root+'/hypersolve/src/algebraic_binary.rs');
 assert.equal(template.length,2);assert.equal(candidate.length,2);
 const lines=s=>s.split('\n').length-1;
 const lineDeltas={algorithm:lines(candidate[0])-lines(template[0]),tests:lines(candidate[1])-lines(template[1])};
 return{gates:specs.map(s=>s.tag),sourceBindingSha256:sha('point-demand-source-binding.json'),sourceFiles:175,
  artifacts:b.artifacts,dedicatedBytes,solverTests,publicChecks,history:json('point-demand-history.json'),costs,memories,environments,lineDeltas,
  memoryLimits:'Focused authored collectors, zero errors/lost blocks but nonzero reachability. Cumulative request demand includes constructors, output and additional certified answers; not marginal costs, RSS or a lifetime boundedness proof.',
  next:'Evaluate paired tradeoffs, then qualify fresh-process first queries without preconditioning before applicable extended WASM execution/costs, final consumer/representative-size gates and retention. Separate power-sum work and the entire original reference inventory remain open.'};
}
