import './verify-arf-fused.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json,sources,here} from './complex-product-sources.mjs';
import {checkComplexProduct} from './check-complex-product.mjs';
const m=json('complex-product-experiment.json'),previous=json('arf-fused-experiment.json');
assert.equal(Object.keys(m.files).length,71);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.sourceMap,sources());assert.deepEqual(m.liveSources,previous.liveSources);
assert.equal(Object.keys(m.liveSources).length,955);assert.deepEqual(m.sourceMap.baseline,m.liveSources);
assert.deepEqual(m.coverageAtBinding,previous.coverageAtBinding);assert.equal(m.donorReread.newCoverageLines,0);
assert.equal(sha(m.donorReread.path),m.donorReread.sha256);assert.deepEqual(m.donorReread.ranges,[[495,535]]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 const content=readFileSync(resolve(here,'../../../..',p),'utf8'),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(m.liveSources[p]);assert.equal(sha(resolve(here,'../../../..',p)),m.liveSources[p]);
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.equal(m.gates.length,16);assert.equal(new Set(m.gates.map(g=>g.tag)).size,16);
for(const g of m.gates) {
 const r=json('results/'+g.tag+'.json');assert.equal(r.tag,g.tag);assert.equal(r.code,0);assert.equal(r.signal,null);
 assert.equal(g.code,r.code);assert.equal(g.signal,r.signal);assert(Date.parse(r.finished)>=Date.parse(r.started));
}
const binaries=[...Object.values(json('complex-product-binaries.json').binaries),...Object.values(json('complex-product-trace-binaries.json'))];
assert.equal(binaries.length,8);let bytes=0;
for(const b of binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);bytes+=b.bytes;}
assert.equal(bytes,m.binaryBytes);assert.equal(bytes,16697200);assert.equal(m.binaryFiles,8);
assert.deepEqual(m.checks,checkComplexProduct());
assert.deepEqual(m.checks,json('results/complex-product-output-check.stdout'));
for(const v of ['baseline','candidate']) {
 const lock=readFileSync(resolve(here,'complex-product-app-'+v+'/Cargo.lock'),'utf8');
 assert.equal((lock.match(/name = "hyperreal"/g)||[]).length,1);assert.equal((lock.match(/name = "hyperlattice"/g)||[]).length,1);
}
console.log(JSON.stringify({checkpoint:'Cold common-scale rational complex-product v1',files:71,gates:16,liveFiles:955,newDonorLines:0,
 binaryFiles:8,binaryBytes:m.binaryBytes,checks:m.checks,status:m.status,production:m.production,findings:m.findings,limits:m.limits,followup:m.followup}));
