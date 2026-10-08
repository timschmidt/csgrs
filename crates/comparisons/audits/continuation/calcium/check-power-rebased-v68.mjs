import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {fixedSources,sha,json,binding} from './power-rebased-fixed-sources-v68.mjs';
import {checkPowerSumsKernel} from './check-power-sums-kernel.mjs';
import {checkPowerSumsPublic} from './check-power-sums-public.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {fieldSelfTest} from './point-extended-field.mjs';
const initialTags=[...['baseline','candidate'].flatMap(v=>['lock','metadata','build'].map(k=>'power-rebased-'+v+'-'+k+'-v68')),
 ...['debug','release'].map(p=>'power-rebased-tests-'+p+'-v68'),'power-rebased-kernel-v68','power-rebased-kernel-check-v68',
 ...['baseline','candidate'].flatMap(v=>['public','approx','extended'].map(k=>'power-rebased-'+v+'-'+k+'-v68')),
 'power-rebased-public-check-v68','power-rebased-approx-check-v68','power-rebased-clippy-v68'];
const fixedTags=['build','tests-debug','tests-release','kernel','kernel-check','public','approx','extended','public-check',
 'approx-check','clippy','fmt','environment'].map(k=>'power-rebased-fixed-'+k+'-v68');
const extraTags=['baseline','candidate'].flatMap(v=>['power-rebased-app-clippy-'+v+'-v68',
 ...['approx','extended'].map(k=>'power-rebased-memory-'+v+'-'+k+'-v68')]).concat('power-rebased-wasm-build-v68');
export const gateTags=[...initialTags,'power-rebased-gates-v68',...fixedTags,'power-rebased-fixed-gates-v68',...extraTags,'power-rebased-extra-v68'];
const lines=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
const output=t=>'results/'+t+'.stdout';
function gate(t,code=0){const g=json('results/'+t+'.json');assert.equal(g.tag,t);assert.equal(g.code,code);assert.equal(g.signal,null);
 assert(Date.parse(g.finished)>=Date.parse(g.started));assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);return g;}
function outer(t,tags,code){const g=gate(t,code),rows=lines(output(t)),terminal=rows.filter(r=>r.finished);
 assert.deepEqual(terminal.map(r=>r.tag),tags);assert.equal(rows.length,tags.length*2+(code===0?1:0));
 let last=Date.parse(g.started);for(let i=0;i<tags.length;i++){
  const begin=rows[2*i],end=rows[2*i+1],r=json('results/'+tags[i]+'.json');assert.deepEqual(end,r);
  assert.deepEqual({...begin,pid:undefined},{tag:r.tag,started:r.started,cwd:r.cwd,command:r.command,args:r.args,pid:undefined});
  assert(Number.isInteger(begin.pid)&&begin.pid>0);assert(Date.parse(begin.started)>=last);last=Date.parse(end.finished);
 }assert(Date.parse(g.finished)>=last);if(code===0){assert.equal(rows.at(-1).checkpoint,68);assert.equal(readFileSync('results/'+t+'.stderr').length,0);}
 return {tag:t,code,started:g.started,finished:g.finished,records:rows.length};
}
function tests(tag){const s=readFileSync(output(tag),'utf8'),suites=[...s.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;/g)]
 .map(m=>[+m[1],+m[2],+m[3]]);assert.equal(suites.length,8);assert(suites.every(r=>r[1]===0&&r[2]===0));
 const names=[...s.matchAll(/^test (.+) \.\.\. (ok|ignored)$/gm)].map(m=>m[1]+' '+m[2]).sort();
 assert.equal(names.length,suites.reduce((n,r)=>n+r[0],0));return names;}
function memory(tag){const s=readFileSync('results/'+tag+'.stderr','utf8');
 const value=(pattern,n=1)=>{const m=s.match(pattern);assert(m,tag+' '+pattern);return Number(m[n].replaceAll(',',''));};
 const r={errors:value(/ERROR SUMMARY: ([\d,]+) errors/),definite:value(/definitely lost: ([\d,]+) bytes/),
  indirect:value(/indirectly lost: ([\d,]+) bytes/),possible:value(/possibly lost: ([\d,]+) bytes/),
  reachableBytes:value(/still reachable: ([\d,]+) bytes in ([\d,]+) blocks/),reachableBlocks:value(/still reachable: ([\d,]+) bytes in ([\d,]+) blocks/,2),
  allocations:value(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/),
  frees:value(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/,2),
  cumulativeRequestedBytes:value(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/,3)};
 assert.equal(r.errors,0);assert.equal(r.definite,0);assert.equal(r.indirect,0);assert.equal(r.possible,0);return r;
}
export function rebasedEvidence(){
 const source=fixedSources();assert.deepEqual(source,json(binding));
 for(const p of ['point-retained-v67-manifest.json','power-sums-manifest.json'])for(const[f,h]of Object.entries(json(p).files))assert.equal(sha(f),h,f);
 assert.equal(initialTags.length,19);assert.equal(fixedTags.length,13);assert.equal(extraTags.length,7);assert.equal(gateTags.length,42);
 for(const t of gateTags)gate(t,t==='power-rebased-gates-v68'?1:t==='power-rebased-clippy-v68'?101:0);
 const captures=[outer('power-rebased-gates-v68',initialTags,1),outer('power-rebased-fixed-gates-v68',fixedTags,0),outer('power-rebased-extra-v68',extraTags,0)];
 assert(Date.parse(captures[1].started)>Date.parse(captures[0].finished));assert(Date.parse(captures[2].started)>Date.parse(captures[1].finished));
 const base=json(output('power-rebased-baseline-metadata-v68')),candidate=json(output('power-rebased-candidate-metadata-v68'));
 const normalize=s=>s.replaceAll(resolve('power-rebased-candidate-app-v68'),resolve('power-rebased-baseline-app-v68'))
  .replaceAll(resolve('power-rebased-v68/hypersolve'),resolve('point-demand-candidate/hypersolve'));
 assert.deepEqual(JSON.parse(normalize(JSON.stringify(candidate))),base);assert.equal(base.packages.length,33);assert.equal(base.resolve.nodes.length,33);
 for(const name of ['hyperreal','hyperlattice','hyperlimit','hypersolve'])assert.equal(base.packages.filter(p=>p.name===name).length,1);
 assert.equal(sha('power-rebased-baseline-app-v68/Cargo.lock'),sha('power-rebased-candidate-app-v68/Cargo.lock'));
 const testCounts={};for(const profile of ['debug','release']){
  const old=tests('point-retained-'+profile+'-v67'),first=tests('power-rebased-tests-'+profile+'-v68'),now=tests('power-rebased-fixed-tests-'+profile+'-v68');
  assert.deepEqual(first,now);assert.equal(now.length,814);assert.equal(old.length,811);
  const added=now.filter(n=>n.startsWith('algebraic_binary::power_sums::tests::'));assert.equal(added.length,3);
  assert.deepEqual(now.filter(n=>!n.startsWith('algebraic_binary::power_sums::tests::')),old);testCounts[profile]=now.length;
 }
 const kernel=checkPowerSumsKernel(output('power-rebased-fixed-kernel-v68'));assert.equal(kernel.status,'pass');
 assert.deepEqual(kernel,json(output('power-rebased-fixed-kernel-check-v68')));assert.deepEqual(kernel,json(output('power-rebased-kernel-check-v68')));
 assert.deepEqual(kernel.counts,{rows:4840,baseline:4840,candidate:4840,direct:4704,fallback:136,zeroResultant:32});
 const publicChecks={};for(const mode of ['public','approx']){
  const b=output('power-rebased-baseline-'+mode+'-v68'),f=output('power-rebased-fixed-'+mode+'-v68');
  assert.equal(sha(b),sha('results/point-demand-'+mode+'.stdout'));assert.equal(sha(f),sha(b));
  assert.equal(sha(output('power-rebased-candidate-'+mode+'-v68')),sha(f));
  const result=checkPowerSumsPublic(b,f);assert.equal(result.status,'pass');assert.equal(result.totalChecks,48059);
  assert.deepEqual(result,json(output('power-rebased-fixed-'+mode+'-check-v68')));publicChecks[mode]=result;
 }
 const extended={};fieldSelfTest();
 for(const variant of ['baseline','candidate']){
  const tag=variant==='candidate'?'power-rebased-fixed-extended-v68':'power-rebased-baseline-extended-v68',p=output(tag),rows=lines(p);
  assert.equal(sha(p),sha('results/point-demand-extended.stdout'));assert.equal(rows.length,385);
  assert.deepEqual(rows.at(-1),{type:'terminal',cases:48,histories:4,policies:2,rows:384});
  const checks={},statuses={};let i=0;for(let which=0;which<48;which++)for(let policy=0;policy<2;policy++)for(let history=0;history<4;history++){
   const r=rows[i++];assert.equal(r.type,'query');assert.equal(r.case,which);assert.equal(r.policy,policy);assert.equal(r.history,history);
   const result=validateExtendedObservation(r);assert.deepEqual(result.failures,[]);
   for(const[k,n]of Object.entries(result.checks))checks[k]=(checks[k]??0)+n;statuses[r.report.status]=(statuses[r.report.status]??0)+1;
  }
  extended[variant]={rows:384,checks,statuses,totalChecks:Object.values(checks).reduce((a,b)=>a+b,0)};
  assert.equal(extended[variant].totalChecks,11248);
 }assert.deepEqual(extended.baseline,extended.candidate);
 assert.equal(sha(output('power-rebased-candidate-extended-v68')),sha(output('power-rebased-fixed-extended-v68')));
 const binaries=json('power-rebased-fixed-binaries-v68.json'),initial=json('power-rebased-binaries-v68.json');
 assert.equal(binaries.bindingSha256,sha(binding));assert.equal(initial.bindingSha256,sha('power-rebased-binding-v68.json'));
 const unique=new Map();for(const b of [initial,binaries])for(const a of b.artifacts){assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);unique.set(a.path,a);}
 assert.equal(unique.size,12);const memoryResults={};
 for(const variant of ['baseline','candidate'])for(const mode of ['approx','extended']){
  const tag='power-rebased-memory-'+variant+'-'+mode+'-v68',a=binaries.artifacts.find(a=>a.variant===variant&&a.mode===mode),g=gate(tag);
  assert.equal(g.command,'valgrind');assert.deepEqual(g.args,['--leak-check=full','--show-leak-kinds=all',
   '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',a.path]);
  const reference=output(variant==='candidate'?'power-rebased-fixed-'+mode+'-v68':'power-rebased-baseline-'+mode+'-v68');
  assert.equal(sha(output(tag)),sha(reference));memoryResults[variant+'-'+mode]=memory(tag);
 }
 const env=json(output('power-rebased-fixed-environment-v68'));
 assert.deepEqual(fixedSources(),source);
 return {checkpoint:68,status:'isolated-numerical-qualified',sourceBindingSha256:sha(binding),currentLiveFiles:956,
  gates:gateTags,captures,successfulCaptures:40,preservedFailedCaptures:2,dependencyGraph:{packages:33,nodes:33,pathOnlyNormalization:true,locksEqual:true},
  tests:testCounts,suitesPerProfile:8,additionalTests:3,kernel,publicChecks,extended,memory:memoryResults,
  binaries:{files:unique.size,bytes:[...unique.values()].reduce((n,a)=>n+a.bytes,0),dir:binaries.dir},environment:env,
  limits:'Current retained baseline already recovers point witnesses. All authored full results are unchanged; this is not a new completeness gain. Initial five-borrow Clippy failure preserved, fixed source rerun. Approximate-policy rational records use the same exact certificates, despite the inherited checker label. Memory totals include collector/setup/output; no peak/RSS/bounded-reachability or timing claim. WASM compile only; broad coefficients, matched costs, consumer/size and retention remain open. No live/donor edit or new donor-line credit.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(rebasedEvidence()));
