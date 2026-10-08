import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {checkPowerSumsKernel} from './check-power-sums-kernel.mjs';
import {checkPowerSumsPublic} from './check-power-sums-public.mjs';
export const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
export const json=p=>JSON.parse(readFileSync(p,'utf8'));
export function powerSumsEvidence(){
 const origin=json('power-sums-origin.json'),live=json(origin.sourceManifest),workspace=resolve('../../../..');
 assert.equal(sha(origin.sourceManifest),origin.sourceManifestSha256);
 assert.deepEqual(origin.liveSources,live.liveSources);assert.equal(Object.keys(live.liveSources).length,956);
 assert.equal(origin.copiedFiles,956);assert.equal(origin.copiedBytes,45450206);
 const candidateSources={},changed=[];
 for(const[p,h]of Object.entries(live.liveSources)){
  assert.equal(sha(resolve(workspace,p)),h,p);assert.equal(sha(origin.baseline+'/'+p),h,p);
  const ch=sha(origin.candidate+'/'+p);candidateSources[p]=ch;if(ch!==h)changed.push(p);
 }
 assert.deepEqual(changed,['hypersolve/src/algebraic_binary.rs']);
 const added='hypersolve/src/algebraic_binary/power_sums.rs';candidateSources[added]=sha(origin.candidate+'/'+added);
 // Verify that the original public pipeline and sampled fallback tail are unchanged.
 const old=readFileSync(origin.baseline+'/'+changed[0],'utf8'),now=readFileSync(origin.candidate+'/'+changed[0],'utf8');
 const boundary='fn resultant_polynomial_for_binary_image(';
 assert.equal(now.slice(0,now.indexOf(boundary)).replace('mod power_sums;\n\n',''),old.slice(0,old.indexOf(boundary)));
 const tail='    let mut samples = Vec::with_capacity(resultant_degree + 1);';
 assert.equal(now.slice(now.indexOf(tail)),old.slice(old.indexOf(tail)));
 const original=readFileSync('check-power-sums-kernel.mjs','utf8');
 assert.equal(sha('check-power-sums-kernel.mjs'),'23d5961387f4691d886f7a561fb5d7991f66cee18a594264f4df9b4f7a3de674');
 const cut=original.indexOf('export function checkPowerSumsKernel(path)');assert(cut>0);
 assert.equal(readFileSync('power-sums-polynomial-oracle.mjs','utf8'),original.slice(0,cut)+'\nselfTest();\nexport {oracle};\n');
 const gates=['candidate-test-build','kernel-debug','kernel-check','candidate-debug','baseline-cost-build',
  'candidate-cost-build','public-baseline','public-candidate','public-check','public-memcheck'].map(t=>'power-sums-'+t);
 for(const tag of gates){const r=json('results/'+tag+'.json');assert.equal(r.code,tag==='power-sums-public-check'?1:0);assert.equal(r.signal,null);}
 const kernel=checkPowerSumsKernel('results/power-sums-kernel-debug.stdout');
 assert.deepEqual(kernel,json('results/power-sums-kernel-check.stdout'));assert.equal(kernel.status,'pass');
 assert.deepEqual(kernel.counts,{rows:4840,baseline:4840,candidate:4840,direct:4704,fallback:136,zeroResultant:32});
 assert.equal(kernel.independentDeterminants,1210);
 const baseline='results/power-sums-public-baseline.stdout',candidate='results/power-sums-public-candidate.stdout';
 const output=readFileSync(baseline);assert.equal(output.length,4332578);
 assert(output.equals(readFileSync(candidate)));assert(output.equals(readFileSync('results/power-sums-public-memcheck.stdout')));
 const publicCheck=checkPowerSumsPublic(baseline,candidate);assert.deepEqual(publicCheck,json('results/power-sums-public-check.stdout'));
 assert.equal(publicCheck.status,'mathematical-fail');assert.equal(publicCheck.totalChecks,43934);assert.equal(publicCheck.failures.length,825);
 assert.equal(publicCheck.independentResultants,1178);
 assert.deepEqual(publicCheck.counts,{InvalidTransformedEvidence:825,DenominatorMayContainZero:241,Transformed:4301,
  Undecided:40,NonIsolatingImageInterval:65,UnsupportedDegree:968});
 const rows=output.toString().trim().split('\n').map(s=>JSON.parse(s)),byKey=new Map(rows.map(r=>[
  r.type==='public'?[r.type,r.i,r.j,r.op,r.scale].join(':'):r.type==='cost-case'?[r.type,r.case].join(':'):r.type,r]));
 for(const f of publicCheck.failures){
  assert.equal(f.name,'nonisolating-control');assert.deepEqual(f.detail,{imageRoots:1,status:'InvalidTransformedEvidence'});
  const row=byKey.get(f.key);assert(row);assert.equal(row.report.message,'a collapsed refinement interval requires an exact witness');
  assert(row.left.lower===row.left.upper||row.right.lower===row.right.upper);
 }
 const test=readFileSync('results/power-sums-candidate-debug.stdout','utf8');
 const results=[...test.matchAll(/test result: ok\. (\d+) passed; 0 failed; 0 ignored; 0 measured; 0 filtered out;/g)].map(m=>+m[1]);
 assert.deepEqual(results,[441,9,7,2,4,131,211]);
 assert(readFileSync('results/power-sums-kernel-debug.stdout','utf8').includes('test result: ok. 19 passed; 0 failed; 0 ignored; 0 measured; 422 filtered out;'));
 const memory=readFileSync('results/power-sums-public-memcheck.stderr','utf8');
 for(const s of ['2,634,556 allocs, 2,622,951 frees, 156,378,772 bytes allocated',
  'definitely lost: 0 bytes in 0 blocks','indirectly lost: 0 bytes in 0 blocks','possibly lost: 0 bytes in 0 blocks',
  'still reachable: 1,364,464 bytes in 11,605 blocks','ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'])assert(memory.includes(s),s);
 const cost=json('power-sums-cost-binaries.json'),binaries=Object.values(cost);
 for(const b of binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
 const path='/tmp/calcium-power-sums.2CGTky/candidate-tests-debug';
 binaries.push({path,sha256:sha(path),bytes:statSync(path).size});
 assert.equal(binaries.reduce((n,b)=>n+b.bytes,0),34189432);
 return{candidateSources,changed,added,gates,kernel,publicCheck,binaries,debugDefaultTests:805,
  outputBytesPerVariant:output.length,outputSha256:sha(baseline),
  memory:{errors:0,definitelyLost:0,indirectlyLost:0,possiblyLost:0,reachableBytes:1364464,reachableBlocks:11605,
   allocations:2634556,frees:2622951,cumulativeRequestedBytes:156378772}};
}
