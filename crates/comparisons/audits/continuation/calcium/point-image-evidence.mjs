import {readFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,checkPointImageSources} from './point-image-sources.mjs';
import {checkPointImage} from './check-point-image.mjs';
import {checkPointImageCosts} from './check-point-image-costs.mjs';
export const pointImageGates=[
 'baseline-debug','baseline-memcheck','baseline-release','candidate-clippy-v2','candidate-clippy','candidate-cost-build',
 'candidate-debug-v2','candidate-debug','candidate-doc','candidate-fmt-v2','candidate-memcheck','candidate-release-v2',
 'candidate-release','consumer-debug','cost-allocation','cost-check','cost-cpu','cost-environment','focused-debug',
 'guard-clippy','guard-controls','guard-cost-build','guard-debug','guard-fmt','guard-focused','guard-memcheck',
 'guard-public-check','guard-public','guard-red','guard-release','public-candidate','public-check','bind-draft'].map(t=>'point-image-'+t);
function tests(tag){
 const matches=[...readFileSync('results/point-image-'+tag+'.stdout','utf8').matchAll(/test result: ok\. (\d+) passed; 0 failed; (\d+) ignored; 0 measured; (\d+) filtered out;/g)];
 assert(matches.length>0,tag);return{passed:matches.reduce((n,r)=>n+ +r[1],0),ignored:matches.reduce((n,r)=>n+ +r[2],0),
  filtered:matches.reduce((n,r)=>n+ +r[3],0),suites:matches.length};
}
function memory(tag){
 const s=readFileSync('results/point-image-'+tag+'.stderr','utf8');
 for(const k of ['definitely','indirectly','possibly'])assert(new RegExp(k+' lost: 0 bytes in 0 blocks').test(s));
 assert(s.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
 const reach=s.match(/still reachable: ([\d,]+) bytes in ([\d,]+) blocks/),heap=s.match(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/);
 assert(reach&&heap);const n=s=>+s.replaceAll(',','');
 return{errors:0,definitelyLost:0,indirectlyLost:0,possiblyLost:0,reachableBytes:n(reach[1]),reachableBlocks:n(reach[2]),allocations:n(heap[1]),frees:n(heap[2]),cumulativeRequestedBytes:n(heap[3])};
}
export function pointImageEvidence(){
 const v2Sources=checkPointImageSources(),v1=json('point-image-binaries.json'),guard=json('point-image-guard-binaries.json'),origin=json('point-image-guard-origin.json');
 assert.deepEqual(guard.dependencySources,v2Sources);assert.deepEqual(origin.sourceHashes,v2Sources);
 assert.equal(origin.copiedFiles,175);assert.equal(origin.copiedBytes,6088204);
 for(const[p,h]of Object.entries(guard.guardSources)){
  assert.equal(sha('point-image-guard/'+p),h,p);
  if(!['hypersolve/Cargo.toml','hypersolve/src/algebraic_binary.rs'].includes(p))assert.equal(h,v2Sources[p],p);
 }
 let manifest=readFileSync('point-image-candidate/hypersolve/Cargo.toml','utf8');
 for(const name of ['hyperreal','hyperlattice','hyperlimit'])manifest=manifest.replace('path = "../'+name+'"','path = "../../point-image-candidate/'+name+'"');
 assert.equal(readFileSync('point-image-guard/hypersolve/Cargo.toml','utf8'),manifest);
 const old=readFileSync('e-plan-qualified-candidate/hypersolve/src/algebraic_binary.rs','utf8').split('#[cfg(test)]')[0];
 const current=readFileSync('point-image-guard/hypersolve/src/algebraic_binary.rs','utf8').split('#[cfg(test)]')[0];
 const start=current.indexOf('    // Only certified equality supplies a singleton witness.'),end=current.indexOf('    Some(IsolatedRootInterval {',start);
 assert(start>0&&end>start);
 assert.equal((current.slice(0,start)+current.slice(end)).replace('        exact_root,\n','        exact_root: None,\n'),old);
 assert.equal(current.split('\n').length-old.split('\n').length,20);
 assert.equal(pointImageGates.length,33);
 for(const tag of pointImageGates){const r=json('results/'+tag+'.json');
  assert.equal(r.code,tag==='point-image-bind-draft'?1:['point-image-candidate-clippy','point-image-guard-red'].includes(tag)?101:0,tag);assert.equal(r.signal,null,tag);
  assert(Date.parse(r.finished)>=Date.parse(r.started),tag);
 }
 assert(readFileSync('results/point-image-candidate-clippy.stderr','utf8').includes('casting to the same type is unnecessary'));
 assert(readFileSync('results/point-image-guard-red.stdout','utf8').includes('left: None\n right: Some(Equal)'));
 const testResults={};for(const [tag,passed]of Object.entries({'baseline-debug':803,'baseline-release':803,
  'candidate-debug':809,'candidate-release':809,'candidate-debug-v2':809,'candidate-release-v2':809,
  'guard-debug':811,'guard-release':811,'candidate-doc':0,'consumer-debug':1764,'focused-debug':22,'guard-controls':8,'guard-focused':24})){
  testResults[tag]=tests(tag);assert.equal(testResults[tag].passed,passed,tag);assert.equal(testResults[tag].ignored,tag==='consumer-debug'?9:0,tag);
 }
 const baseline='results/power-sums-public-baseline.stdout',candidate='results/point-image-public-candidate.stdout',final='results/point-image-guard-public.stdout';
 const mathematical=checkPointImage(baseline,final);assert.deepEqual(mathematical,json('results/point-image-guard-public-check.stdout'));
 assert.deepEqual(mathematical,checkPointImage(baseline,candidate));assert.deepEqual(mathematical,json('results/point-image-public-check.stdout'));
 assert.equal(mathematical.mathematical.totalChecks,48059);assert.equal(mathematical.improved,825);assert.equal(mathematical.unchanged,5616);
 assert.equal(mathematical.mathematical.counts.Transformed,5126);assert.equal(mathematical.mathematical.checks['exact-root'],1794);
 const output=readFileSync(final);assert.equal(output.length,4428994);
 for(const p of [candidate,'results/point-image-candidate-memcheck.stdout','results/point-image-guard-memcheck.stdout'])assert(output.equals(readFileSync(p)));
 assert(readFileSync(baseline).equals(readFileSync('results/point-image-baseline-memcheck.stdout')));
 const memories={baseline:memory('baseline-memcheck'),candidate:memory('candidate-memcheck'),guard:memory('guard-memcheck')};
 assert.equal(memories.candidate.cumulativeRequestedBytes,215408740);
 assert.equal(memories.guard.cumulativeRequestedBytes,215408746);
 assert.deepEqual({...memories.candidate,cumulativeRequestedBytes:memories.guard.cumulativeRequestedBytes},memories.guard);
 assert.equal(memories.baseline.reachableBytes,1839816);assert.equal(memories.guard.reachableBytes,1840544);
 assert.equal(memories.baseline.reachableBlocks,15311);assert.equal(memories.guard.reachableBlocks,15318);
 const costs=checkPointImageCosts();assert.deepEqual(costs,json('results/point-image-cost-check.stdout'));assert.equal(costs.status,'pass');
 const binaries=[...Object.values(v1.binaries),...Object.values(guard.binaries)];
 for(const b of binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
 assert.equal(binaries.reduce((n,b)=>n+b.bytes,0),56943680);
 return{v2Sources,guardSources:guard.guardSources,algorithmNetLines:20,tests:testResults,mathematical,memories,costs,binaries,
  gates:pointImageGates,outputBytes:output.length,outputSha256:sha(final),dedicatedExecutableBytes:56943680};
}
