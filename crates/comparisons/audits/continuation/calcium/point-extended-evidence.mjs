import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
import {checkExtended} from './check-point-extended.mjs';
import {checkExtended as checkInitial} from './check-point-extended-v1.mjs';
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
export function pointExtendedEvidence(){
 const o=sources(),b=json('point-extended-binaries.json'),specs=[];
 assert.equal(b.sourceOriginSha256,sha('point-qualified-origin.json'));
 assert.equal(b.harnessSha256,sha('point-extended.rs'));assert.equal(b.wrapperSha256,sha('point-extended-cpu.rs'));
 assert.equal(b.artifacts.length,2);assert.equal(b.artifacts.reduce((n,f)=>n+f.bytes,0),6379848);
 const add=(tag,command,args,cwd='.',code=0)=>specs.push({tag:'point-extended-'+tag,cwd:resolve(cwd),command,args,code});
 add('build','node',['build-point-extended.mjs']);add('public','node',['run-point-extended.mjs']);
 add('memory','node',['run-point-extended-memory.mjs']);
 add('decode-v1','node',['probe-point-extended-field.mjs','point-extended-field-v1.mjs'],'.',1);
 add('decode','node',['probe-point-extended-field.mjs']);
 add('mathematical','node',['check-point-extended.mjs']);
 add('mathematical-complete','node',['check-point-extended.mjs']);
 let lock;const memories={};
 for(const f of b.artifacts){
  assert.equal(sha(f.path),f.sha256);assert.equal(statSync(f.path).size,f.bytes);
  const app='point-extended-'+f.variant;
  add(f.variant+'-lock','env',[...cargoEnv,'cargo','generate-lockfile','--offline'],app);
  add(f.variant+'-build','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--bin',app+'-cpu'],app);
  add('public-'+f.variant,f.path,['check']);
  add('memory-'+f.variant,'valgrind',['--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',f.path,'check']);
  const manifest=read(app+'/Cargo.toml'),normalized=read(app+'/Cargo.lock').replaceAll('calcium-'+app,'calcium-point-extended-normalized');
  if(lock===undefined)lock=normalized;else assert.equal(normalized,lock);
  for(const crate of ['hyperreal','hyperlimit','hypersolve'])assert(manifest.includes('path = "../'+o[f.variant]+'/'+crate+'"'));
  assert(manifest.includes('features = ["serde"]'));assert(manifest.includes('path = "../point-extended-cpu.rs"'));
  assert.equal(sha('results/point-extended-memory-'+f.variant+'.stdout'),sha('results/point-extended-public-'+f.variant+'.stdout'));
  assert.equal(read('results/point-extended-public-'+f.variant+'.stderr'),'');
  const build=read('results/point-extended-'+f.variant+'-build.stderr');
  assert(build.includes('warning: variable does not need to be mutable'));assert(build.includes('let mut visit ='));
  memories[f.variant]=memory('point-extended-memory-'+f.variant);
 }
 assert.deepEqual(memories.baseline,{errors:0,definite:0,indirect:0,possible:0,reachableBytes:47632,reachableBlocks:346,allocations:675746,frees:675400,cumulativeBytes:123272240});
 assert.deepEqual(memories.candidate,{errors:0,definite:0,indirect:0,possible:0,reachableBytes:48648,reachableBlocks:353,allocations:729203,frees:728850,cumulativeBytes:131069337});
 for(const s of specs){
  const g=json('results/'+s.tag+'.json');for(const k of ['tag','cwd','command','args','code'])assert.deepEqual(g[k],s[k],s.tag+' '+k);
  assert.equal(g.signal,null);assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));
 }
 assert.equal(specs.length,15);
 const firstError=JSON.parse(read('results/point-extended-decode-v1.stderr').split('\n')[0]);
 assert.deepEqual(firstError,{variant:'baseline',case:40,policy:0,history:0,side:'left',field:'lower',decodedBeforeFailure:4236});
 assert(read('results/point-extended-decode-v1.stderr').includes("assert(['Pi','E'].includes(value))"));
 const old="  case 'Constant':assert(['Pi','E'].includes(value));out=variable(value==='Pi'?0:1);break;";
 const replacement="  case 'Constant':\n   assert(['Pi','E','InvPi','Sqrt2'].includes(value));\n   out=value==='InvPi'?ri(variable(0)):value==='Sqrt2'?literal(root2()):variable(value==='Pi'?0:1);break;";
 assert.equal(read('point-extended-field-v1.mjs').split(old).length,2);
 assert.equal(read('point-extended-field.mjs'),read('point-extended-field-v1.mjs').replace(old,replacement));
 const decoded=read('results/point-extended-decode.stdout').trim().split('\n').map(JSON.parse);
 assert.equal(decoded.length,2);assert.equal(decoded[1].values,10664);assert.equal(decoded[1].status,'all-values-decoded');
 const initial=checkInitial(),mathematical=checkExtended();
 assert.deepEqual(initial,JSON.parse(read('results/point-extended-mathematical.stdout')));
 assert.deepEqual(mathematical,JSON.parse(read('results/point-extended-mathematical-complete.stdout')));
 assert.equal(mathematical.status,'pass');assert.equal(initial.status,'pass');
 assert.equal(initial.summaries.baseline.totalChecks+initial.summaries.candidate.totalChecks,17640);
 assert.equal(mathematical.summaries.baseline.totalChecks+mathematical.summaries.candidate.totalChecks,21216);
 assert.equal(mathematical.improved,128);assert.equal(mathematical.unchanged,257);
 assert.equal(mathematical.historyChanges.length,8);
 for(const h of mathematical.historyChanges){assert([36,37].includes(h.case));assert.equal(h.statuses[2],'Transformed');}
 return{gates:specs.map(s=>s.tag),failedGate:'point-extended-decode-v1',sourceOriginSha256:b.sourceOriginSha256,
  artifacts:b.artifacts,dedicatedBytes:6379848,mathematical,initialChecks:17640,decodedValues:10664,memories,
  memoryLimits:'Whole ordered collector including constructors, serialization and all histories. Zero errors/lost blocks, not zero live, peak RSS, marginal kernel costs or a general boundedness proof. Candidate performs additional certified work.',
  harnessLimits:'Both builds preserve an unnecessary-mut warning in the collector closure. No Clippy or debug/full-suite qualification of this new harness. Its CPU/allocation modes are present but have not been exercised or accepted as benchmark evidence.'};
}
