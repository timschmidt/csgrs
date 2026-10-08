import {readFileSync,readdirSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
import {coldBindings,variants,platforms,metadataPath,outputPath} from './point-cold-protocol.mjs';
import {checkCold} from './check-point-cold.mjs';
const read=p=>readFileSync(p,'utf8');
export async function pointColdEvidence(){
 const b=coldBindings(),specs=[];
 const add=(tag,command,args,cwd='.',code=0)=>specs.push({tag:'point-cold-'+tag,cwd:resolve(cwd),command,args,code});
 add('build','node',['build-point-cold.mjs'],'.',1);
 add('build-resume','node',['resume-point-cold-build.mjs'],'.',1);
 add('build-resume-v2','node',['resume-point-cold-build.mjs']);
 add('baseline-native-build','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bin','point-cold-baseline'],'point-cold-baseline');
 add('baseline-native-clippy','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--','-D','warnings'],'point-cold-baseline',101);
 for(const v of variants){
  const app='point-cold-'+v;
  add(v+'-lock','env',[...cargoEnv,'cargo','generate-lockfile','--offline'],app);
  add(v+'-native-build-v1','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bin',app],app);
  add(v+'-native-clippy-v1','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--','-D','warnings'],app);
  add(v+'-wasm-build','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--target','wasm32-unknown-unknown','--lib'],app);
  add(v+'-wasm-clippy','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--target','wasm32-unknown-unknown','--lib','--','-D','warnings'],app);
 }
 add('campaign','node',['run-point-cold-campaign.mjs']);
 for(const v of variants){
  add('native-'+v,'node',['run-point-cold-native.mjs',v]);
  add('wasm-'+v,'node',['--expose-gc','run-point-cold-wasm.mjs',v]);
 }
 add('capacity','df',['-B1','/tmp','.']);
 add('check','node',['check-point-cold.mjs']);
 add('format','rustfmt',['--edition','2024','--check','point-cold-common.rs','point-cold-native.rs','point-cold-platform.rs']);
 const gates={};
 for(const s of specs){
  const g=json('results/'+s.tag+'.json');gates[s.tag]=g;
  for(const k of ['tag','cwd','command','args','code'])assert.deepEqual(g[k],s[k],s.tag+' '+k);
  assert.equal(g.signal,null);assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));
 }
 assert(read('results/point-cold-baseline-native-clippy.stderr').includes('#unnecessary_unwrap'));
 assert(read('results/point-cold-build-resume.stderr').includes('101 !== 1'));
 for(const v of variants)for(const suffix of ['native-build-v1','native-clippy-v1','wasm-build','wasm-clippy'])
  assert(!read('results/point-cold-'+v+'-'+suffix+'.stderr').includes('warning:'));
 assert(Date.parse(gates['point-cold-build-resume-v2'].finished)<=Date.parse(gates['point-cold-campaign'].started));
 assert(Date.parse(gates['point-cold-campaign'].finished)<=Date.parse(gates['point-cold-check'].started));
 assert.deepEqual(readdirSync(b.root).sort(),variants.flatMap(v=>[v,v+'.wasm']).sort());
 const checked=await checkCold();assert.deepEqual(checked,JSON.parse(read('results/point-cold-check.stdout')));assert.equal(checked.status,'pass');
 assert.equal(checked.totalQueries,62208);assert.equal(checked.totalChecks,1751136);
 assert.equal(checked.stateChanges.length,24);assert.equal(checked.firstLifecycleDifferences.length,0);
 for(const p of platforms){
  const c=checked.comparisons[p];assert.equal(c['eager/demand'].equal,10368);assert.equal(c['eager/demand'].different,0);
  for(const name of ['baseline/eager','baseline/demand']){
   assert.equal(c[name].equal,6912);assert.equal(c[name].gainedAnswer,3456);assert.equal(c[name].firstQueryDifferences,384);
   assert(c[name].differences.every(d=>d.inputEqual&&d.from.status==='InvalidTransformedEvidence'&&d.to.status==='Transformed'&&d.to.witness));
  }
 }
 for(const c of checked.stateChanges){
  assert(['36:0:2:roundtrip','36:1:2:roundtrip','37:0:2:roundtrip','37:1:2:roundtrip'].includes(c.group));
  assert.deepEqual(c.changedCalls,[1,2,3,4,5,6,7,8]);assert.deepEqual(c.inputChangedCalls,[]);
  assert.deepEqual(c.initial,{status:'Transformed',witness:true});
  assert(c.sequence.slice(1).every(s=>!s.witness&&s.status===(c.group.split(':')[1]==='0'?'Undecided':'NonIsolatingImageInterval')));
 }
 const comparisons=Object.fromEntries(Object.entries(checked.comparisons).map(([p,cs])=>[p,Object.fromEntries(
  Object.entries(cs).map(([k,{differences,...rest}])=>[k,{...rest,differences: {count:differences.length,
   location:'results/point-cold-check.stdout',sha256:sha('results/point-cold-check.stdout')}}]))]));
 const semantic={...checked,comparisons};
 const rawPaths=platforms.flatMap(p=>variants.flatMap(v=>[outputPath(p,v),metadataPath(p,v)]));
 const dedicatedBytes=b.artifacts.reduce((n,a)=>n+a.bytes,0),rawBytes=rawPaths.reduce((n,p)=>n+statSync(p).size,0);
 assert.equal(dedicatedBytes,14358510);assert.equal(rawBytes,201468576);
 return{gates:specs.map(s=>s.tag),successfulGates:specs.filter(s=>s.code===0).length,
  preservedFailures:specs.filter(s=>s.code!==0).map(s=>({tag:s.tag,code:s.code})),
  sourceBindingSha256:sha('point-demand-source-binding.json'),sourceFiles:175,
  artifacts:b.artifacts,dedicatedBytes,rawBytes,semantic,
  next:'The cold-first-query and extended native/WASM semantic gates pass for the unchanged demand candidate. Assess measured retry tradeoffs and run matched extended WASM costs plus final consumer/representative-size qualification before a retention decision. No algorithm revision is implied; separate power-sum work and the complete original reference inventory remain open.'};
}
