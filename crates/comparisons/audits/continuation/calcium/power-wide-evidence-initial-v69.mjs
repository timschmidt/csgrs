import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {fixedSources,sha,json} from './power-rebased-fixed-sources-v68.mjs';
import {checkWide} from './check-power-wide-v69.mjs';
import {zeroDeflationProbe} from './probe-zero-deflation-v69.mjs';
export const nested=['baseline','candidate'].flatMap(v=>['lock','metadata','build','run','clippy'].map(k=>'power-wide-'+v+'-'+k+'-v69'));
export const gates=[...nested,'power-wide-gates-v69','power-wide-check-v69','power-zero-deflation-v69','power-wide-environment-v69'];
export function wideEvidence(){
 const o=json('power-wide-origin-v69.json'),previous=json('power-rebased-v68-manifest.json');
 assert.equal(o.previousSha256,sha('power-rebased-v68-manifest.json'));
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 assert.deepEqual(fixedSources(),o.source);assert.deepEqual(previous.source,o.source);
 for(const[p,h]of Object.entries(o.files))assert.equal(sha(p),h,p);
 for(const t of gates){const g=json('results/'+t+'.json');assert.equal(g.tag,t);assert.equal(g.code,0);assert.equal(g.signal,null);assert(Date.parse(g.finished)>=Date.parse(g.started));}
 const outer=json('results/power-wide-gates-v69.json'),rows=readFileSync('results/power-wide-gates-v69.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,21);assert.equal(rows.at(-1).checkpoint,69);assert.equal(readFileSync('results/power-wide-gates-v69.stderr').length,0);
 let last=Date.parse(outer.started);for(let i=0;i<nested.length;i++){
  const g=json('results/'+nested[i]+'.json'),begin=rows[2*i];assert.deepEqual(rows[2*i+1],g);
  assert.deepEqual({...begin,pid:undefined},{tag:g.tag,started:g.started,cwd:g.cwd,command:g.command,args:g.args,pid:undefined});
  assert(Number.isInteger(begin.pid)&&begin.pid>0);assert(Date.parse(g.started)>=last);last=Date.parse(g.finished);
 }assert(Date.parse(outer.finished)>=last);
 for(const t of ['power-wide-check-v69','power-zero-deflation-v69','power-wide-environment-v69']){
  const g=json('results/'+t+'.json');assert(Date.parse(g.started)>Date.parse(outer.finished));assert.equal(readFileSync('results/'+t+'.stderr').length,0);
 }
 const baseline=json('results/power-wide-baseline-metadata-v69.stdout'),candidate=json('results/power-wide-candidate-metadata-v69.stdout');
 const normalized=JSON.parse(JSON.stringify(candidate).replaceAll(resolve('power-wide-candidate-app-v69'),resolve('power-wide-baseline-app-v69'))
  .replaceAll(resolve('power-rebased-v68/hypersolve'),resolve('point-demand-candidate/hypersolve')));
 assert.deepEqual(normalized,baseline);assert.equal(baseline.packages.length,33);assert.equal(baseline.resolve.nodes.length,33);
 for(const name of ['hyperreal','hyperlimit','hyperlattice','hypersolve'])assert.equal(baseline.packages.filter(p=>p.name===name).length,1);
 assert.equal(sha('power-wide-baseline-app-v69/Cargo.lock'),sha('power-wide-candidate-app-v69/Cargo.lock'));
 const wide=checkWide();assert.equal(wide.status,'pass');assert.deepEqual(wide,json('results/power-wide-check-v69.stdout'));
 assert.equal(wide.totalChecks,81610);assert.equal(wide.independentDeterminants,1791);
 assert.deepEqual(wide.statuses,{baseline:{Transformed:3540,DenominatorMayContainZero:80,Undecided:60},candidate:{Transformed:3540,DenominatorMayContainZero:80,Undecided:60}});
 assert.deepEqual([wide.maxInputNumeratorBits,wide.maxInputDenominatorBits,wide.maxPrimitiveCoefficientBits,wide.maxResultCoefficientBits],[2577,258,2320,4639]);
 assert.equal(sha('results/power-wide-baseline-run-v69.stdout'),sha('results/power-wide-candidate-run-v69.stdout'));
 const original=readFileSync('check-power-sums-public.mjs','utf8'),cut=original.indexOf('const key=r=>r.type');assert(cut>0);
 assert.equal(readFileSync('power-rational-oracle-v69.mjs','utf8'),original.slice(0,cut)+'\nselfTest();\nexport {parse,rootCount,image,root,q,qc,evaluate,integers};\n');
 const opportunity=zeroDeflationProbe();assert.deepEqual(opportunity,json('results/power-zero-deflation-v69.stdout'));
 assert.equal(opportunity.records,70);assert.equal(opportunity.totalChecks,810);assert.equal(opportunity.distinctNormalizedCarrierPairs,40);
 const b=json('power-wide-binaries-v69.json');assert.equal(b.originSha256,sha('power-wide-origin-v69.json'));assert.equal(b.binaries.length,2);
 for(const a of b.binaries){assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);
  const g=json('results/power-wide-'+a.variant+'-run-v69.json');assert.equal(g.command,a.path);assert.deepEqual(g.args,[resolve('power-wide-input-v69.json')]);}
 const environment=json('results/power-wide-environment-v69.stdout');
 assert.deepEqual(environment.versions,previous.evidence.environment.versions);
 assert.equal(environment.node,previous.evidence.environment.node);assert.equal(environment.v8,previous.evidence.environment.v8);
 assert.deepEqual(fixedSources(),o.source);
 return{checkpoint:69,status:'wide-numerical-qualified-deflation-opportunity',sourceSha256:sha('power-rebased-fixed-binding-v68.json'),
  currentLiveFiles:956,candidateFiles:176,gates,wide,opportunity,
  dependencyGraph:{packages:33,nodes:33,locksEqual:true,pathOnlyNormalization:true},
  artifacts:{binaries:b.binaries.length,binaryBytes:b.binaries.reduce((n,a)=>n+a.bytes,0),
   inputBytes:statSync('power-wide-input-v69.json').size,pairedOutputBytes:statSync('results/power-wide-baseline-run-v69.stdout').size*2},environment,
  next:'Investigate certified divisor zero-factor removal as a separate completeness transfer before timing. Preserve strict nonzero evidence, source validation, interval/polynomial replay, signed primitive orientation and correct post-transformation carrier sharing/degree admission. Power-sum benchmarks/consumer/size and the whole ecosystem audit remain open.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(wideEvidence()));
