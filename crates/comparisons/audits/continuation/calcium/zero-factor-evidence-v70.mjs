import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {finalZeroSources,sha,json} from './zero-factor-final-sources-v70.mjs';
import {fixedSources} from './power-rebased-fixed-sources-v68.mjs';
import {checkZeroFactor} from './check-zero-factor-v70.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {fieldSelfTest} from './point-extended-field.mjs';
const initial=['default','all-features','clippy','fmt'].map(t=>'zero-factor-'+t+'-v70');
const nested=[...['default','all-features','release','clippy','fmt','wasm'].map(t=>'zero-factor-final-'+t+'-v70'),
 ...['lock','metadata','build','wide-run','degree-run','public-run','approx-run','extended-run','app-clippy','degree-baseline'].map(t=>'zero-factor-'+t+'-v70')];
const extras=['memory-public','memory-extended','memory-wide','environment'].map(t=>'zero-factor-'+t+'-v70');
export const gates=['zero-factor-unit-v70',...initial,'zero-factor-gates-v70',...nested,'zero-factor-final-gates-v70',
 'zero-factor-check-v70',...extras,'zero-factor-extra-v70'];
const output=t=>'results/'+t+'.stdout',read=t=>readFileSync(output(t),'utf8').trimEnd().split('\n').map(JSON.parse);
const failed=new Map([['zero-factor-unit-v70',101],['zero-factor-fmt-v70',1],['zero-factor-gates-v70',1]]);
function gate(t){const g=json('results/'+t+'.json');assert.equal(g.tag,t);assert.equal(g.code,failed.get(t)??0);assert.equal(g.signal,null);
 assert(Date.parse(g.finished)>=Date.parse(g.started));assert(g.elapsedSeconds>=0);return g;}
function outer(tag,tags,success){
 const g=gate(tag),rows=read(tag);assert.equal(rows.length,tags.length*2+Number(success));
 let last=Date.parse(g.started);for(let i=0;i<tags.length;i++){
  const child=gate(tags[i]),begin=rows[2*i];assert.deepEqual(rows[2*i+1],child);
  assert.deepEqual({...begin,pid:undefined},{tag:child.tag,started:child.started,cwd:child.cwd,command:child.command,args:child.args,pid:undefined});
  assert(Number.isInteger(begin.pid)&&begin.pid>0);assert(Date.parse(child.started)>=last);last=Date.parse(child.finished);
 }assert(Date.parse(g.finished)>=last);
 if(success){assert.equal(rows.at(-1).checkpoint,70);assert.equal(readFileSync('results/'+tag+'.stderr').length,0);}
 return {tag,started:g.started,finished:g.finished,code:g.code,records:rows.length};
}
function tests(tag){
 const text=readFileSync(output(tag),'utf8'),suites=[...text.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;/g)].map(m=>m.slice(1).map(Number));
 assert.equal(suites.length,8);assert(suites.every(s=>s[1]===0&&s[2]===0));
 const names=[...text.matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]).sort();assert.equal(names.length,suites.reduce((n,s)=>n+s[0],0));
 return {suites,names,total:names.length};
}
function memory(tag){
 const s=readFileSync('results/'+tag+'.stderr','utf8'),value=(pattern,n=1)=>{const m=s.match(pattern);assert(m);return Number(m[n].replaceAll(',',''));};
 const r={errors:value(/ERROR SUMMARY: ([\d,]+) errors/),definite:value(/definitely lost: ([\d,]+) bytes/),
  indirect:value(/indirectly lost: ([\d,]+) bytes/),possible:value(/possibly lost: ([\d,]+) bytes/),
  reachableBytes:value(/still reachable: ([\d,]+) bytes in ([\d,]+) blocks/),reachableBlocks:value(/still reachable: ([\d,]+) bytes in ([\d,]+) blocks/,2),
  allocations:value(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/),frees:value(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/,2),
  cumulativeRequestedBytes:value(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/,3)};
 assert.deepEqual([r.errors,r.definite,r.indirect,r.possible],[0,0,0,0]);return r;
}
export function zeroEvidence(){
 const source=finalZeroSources();assert.deepEqual(source,json('zero-factor-final-binding-v70.json'));
 const previous=json('power-wide-v69-manifest.json');for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 assert.deepEqual(fixedSources(),json('power-wide-origin-v69.json').source);
 assert.equal(sha('results/point-demand-public.stdout'),sha('results/power-rebased-baseline-public-v68.stdout'));
 const origin=json('zero-factor-final-gates-origin-v70.json');assert.deepEqual(source,origin.source);
 for(const o of [origin,json('zero-factor-gates-origin-v70.json')])for(const[p,h]of Object.entries(o.files))assert.equal(sha(p),h,p);
 assert.deepEqual(json('zero-factor-gates-origin-v70.json').source,json('zero-factor-fixed-binding-v70.json'));
 assert.equal(gates.length,29);assert.equal(nested.length,16);for(const t of gates)gate(t);
 const captures=[outer('zero-factor-gates-v70',initial,false),outer('zero-factor-final-gates-v70',nested,true),outer('zero-factor-extra-v70',extras,true)];
 assert(Date.parse(captures[1].started)>Date.parse(captures[0].finished));assert(Date.parse(captures[2].started)>Date.parse(captures[1].finished));
 const testResults={default:tests('zero-factor-final-default-v70'),allFeatures:tests('zero-factor-final-all-features-v70'),release:tests('zero-factor-final-release-v70')};
 assert.equal(testResults.default.total,817);assert.equal(testResults.allFeatures.total,818);assert.deepEqual(testResults.allFeatures,testResults.release);
 assert.deepEqual(tests('zero-factor-default-v70'),testResults.default);assert.deepEqual(tests('zero-factor-all-features-v70'),testResults.allFeatures);
 const old=tests('point-retained-debug-v67'),added=testResults.allFeatures.names.filter(n=>!old.names.includes(n));
 assert.equal(added.length,7);assert(added.every(n=>n.startsWith('algebraic_binary::zero_factor_tests::')));
 assert.deepEqual(testResults.allFeatures.names.filter(n=>!added.includes(n)),old.names);
 const featureOnly=testResults.allFeatures.names.filter(n=>!testResults.default.names.includes(n));
 assert.deepEqual(featureOnly,['tensor_resultant::tests::dense_tensor_multiplication_does_not_refine_opaque_coefficients_for_zero_pruning']);
 const baseline=json(output('power-rebased-baseline-metadata-v68')),raw=json(output('zero-factor-metadata-v70')),
  normalize=s=>s.replaceAll(resolve('zero-factor-candidate-v70/hypersolve'),resolve('point-demand-candidate/hypersolve'));
 const candidate=JSON.parse(normalize(JSON.stringify(raw))),br=baseline.resolve.root,cr=candidate.resolve.root;
 assert.equal(baseline.packages.length,33);assert.equal(candidate.packages.length,33);
 assert.deepEqual(candidate.packages.filter(p=>p.id!==cr),baseline.packages.filter(p=>p.id!==br));
 assert.deepEqual(candidate.resolve.nodes.filter(p=>p.id!==cr),baseline.resolve.nodes.filter(p=>p.id!==br));
 assert.deepEqual({...candidate.resolve.nodes.find(p=>p.id===cr),id:null},{...baseline.resolve.nodes.find(p=>p.id===br),id:null});
 assert.equal(readFileSync('zero-factor-candidate-app-v70/Cargo.lock','utf8').replace('calcium-zero-factor-v70','calcium-power-rebased-v68'),readFileSync('power-rebased-baseline-app-v68/Cargo.lock','utf8'));
 const numerical=checkZeroFactor();assert.equal(numerical.status,'pass');assert.deepEqual(numerical,json(output('zero-factor-check-v70')));
 assert.equal(numerical.totalChecks,128652);assert.equal(numerical.independentDeterminants,6441);
 assert.deepEqual(Object.values(numerical.summaries).map(s=>s.changed),[52,52,60,20]);
 fieldSelfTest();const rows=read('zero-factor-extended-run-v70');assert.equal(rows.length,385);
 assert.equal(sha(output('zero-factor-extended-run-v70')),sha(output('power-rebased-baseline-extended-v68')));
 assert.deepEqual(rows.at(-1),{type:'terminal',cases:48,histories:4,policies:2,rows:384});
 const checks={},statuses={};for(const row of rows.slice(0,-1)){
  const r=validateExtendedObservation(row);assert.deepEqual(r.failures,[]);
  for(const[k,n]of Object.entries(r.checks))checks[k]=(checks[k]??0)+n;statuses[row.report.status]=(statuses[row.report.status]??0)+1;
 }assert.equal(Object.values(checks).reduce((a,b)=>a+b,0),11248);
 const binary=json('zero-factor-binaries-v70.json');assert.equal(binary.originSha256,sha('zero-factor-final-gates-origin-v70.json'));
 assert.equal(binary.binaries.length,4);for(const a of [...binary.binaries,binary.baseline]){assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);}
 const memoryResults={};for(const [name,normal,args]of [['public','public-run',['check']],['extended','extended-run',['check']],['wide','degree-run',[resolve('zero-factor-degree-input-v70.json')]]]){
  const t='zero-factor-memory-'+name+'-v70',g=gate(t),a=binary.binaries.find(b=>b.name===name);
  assert.equal(g.command,'valgrind');assert.deepEqual(g.args,['--leak-check=full','--show-leak-kinds=all',
   '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',a.path,...args]);
  assert.equal(sha(output(t)),sha(output('zero-factor-'+normal+'-v70')));memoryResults[name]=memory(t);
 }
 const environment=json(output('zero-factor-environment-v70'));assert.deepEqual(environment.versions,previous.evidence.environment.versions);
 assert.equal(environment.node,previous.evidence.environment.node);assert.equal(environment.v8,previous.evidence.environment.v8);
 assert.deepEqual(finalZeroSources(),source);
 return {checkpoint:70,status:'isolated-zero-factor-numerical-qualified',sourceBindingSha256:sha('zero-factor-final-binding-v70.json'),
  liveFiles:956,candidateFiles:176,productionDiff:{added:56,removed:1,newTestLines:260,newTests:7},
  tests:Object.fromEntries(Object.entries(testResults).map(([k,v])=>[k,{total:v.total,suites:v.suites}])),featureOnly,addedTests:added,
  dependencyGraph:{packages:33,nodes:33,unchangedDependencyPackages:32,unchangedDependencyNodes:32,rootDependenciesEqual:true,lockEqualAfterAppName:true,
   note:'Collector package/binary targets differ intentionally; dependency packages/nodes agree after the one solver-path normalization.'},
  gates,captures,successfulCaptures:26,preservedFailedCaptures:3,numerical,extended:{queries:384,unchanged:true,checks,statuses,totalChecks:11248},
  memory:memoryResults,binaries:{newFiles:4,newBytes:binary.binaries.reduce((n,a)=>n+a.bytes,0),reusedFiles:1,dir:resolve(binary.binaries[0].path,'..')},environment,
  limits:'No production algorithm change after initial trial. One sign expectation corrected and formatted; three failed captures preserved. WASM compile only. Native Memcheck totals include setup/collector/output, not matched costs, peak/RSS or boundedness. No timing, representative consumer/size gate, live edit or seventh retention. Full ecosystem audit remains open.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(zeroEvidence()));
