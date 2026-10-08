import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkArfFusedControls} from './check-arf-fused-controls.mjs';
const tags=['compile','native','memcheck','linked-libraries','bigint-oracle','output-check','inventory'].map(t=>'arf-fused-'+t);
const files=['flint-arf-fused-controls.c','flint-arf-rounding-controls.c','arf-fused-bigint-oracle.mjs','check-arf-fused-controls.mjs',
 'record-arf-fused-reads.mjs','arf-fused-read-selection.json','bind-arf-fused.mjs','verify-arf-fused.mjs',
 'arf-rounding-experiment-v2.json','capture.mjs'];
for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);}
const previous=json('arf-rounding-experiment-v2.json'),live=previous.liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
const readRecords=json('arf-fused-read-selection.json').map(s=>{
 const r=coverage.find(r=>r.repo===s.repo&&r.path===s.path&&JSON.stringify(r.ranges)===JSON.stringify(s.newRanges));assert(r);return r;
});
const libraryPaths=[...readFileSync('results/arf-fused-linked-libraries.stdout','utf8').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]);
const libraries=Object.fromEntries(libraryPaths.map(p=>[p,sha(p)]));assert.deepEqual(libraries,previous.libraries);
for(const[p,h]of Object.entries(previous.configurationFiles))assert.equal(sha(p),h,p);
const hyperReadRanges={'hyperreal/AGENTS.md':[[1,14]],'hyperreal/src/rational/arithmetic/aggregate_products.rs':[[2780,2920]],
 'hyperreal/src/real/arithmetic/facts.rs':[[475,555]],'hyperlattice/src/complex.rs':[[271,320]]};
for(const[p,ranges]of Object.entries(hyperReadRanges)){
 const content=readFileSync(resolve('../../../..',p),'utf8'),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(live[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
const binary='/tmp/calcium-arf-fused.rUeaCU/arf-fused-controls';
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:tags.map(tag=>({tag,code:0,signal:null})),liveSources:live,libraries,configurationFiles:previous.configurationFiles,
 binary:{path:binary,bytes:statSync(binary).size,sha256:sha(binary)},readRecords,coverageAtBinding:effectiveSummary(),hyperReadRanges,
 checks:checkArfFusedControls(),
 status:'Finite fused, complex, addition, sum and exact-dot controls pass native and Memcheck with independent complete-integer expression/rounding checks. Separate BigInt reconstruction matches every result group fingerprint and decision count. Source and mathematical qualification remain distinct; the full ecosystem audit is incomplete.',
 production:'No new production/donor edits or fifth continuation transfer; all 955 live hashes match checkpoint 34. Existing library/build caches, snapshots, failed gates and core evidence preserved.',
 limits:'829440 scalar-component/value/flag comparisons over 576 fixtures: 12 base limb lengths 1..251, 12 families, four selected sign masks (not all 16), 12 precision positions and five modes, with 24 result routes. Complex operations contribute two component results each, not two independent calls. 19584 before/after input checks include original scalars, initial, sum arrays and dot vectors. Families include real/imaginary exact or near cancellation, finite-zero controls, 63/64/65/129/257-bit exponent gaps and two/three-limb component imbalance. Inputs are bounded exact dyadics with ordinary exponents; exact precision remains small enough for memory. Rounding oracle is the unchanged C/GMP helper from checkpoint 34, comparing every full decoded value and exactness flag; the previous scalar main is compiled but never run by this harness. BigInt independently reconstructs 345600 expression roundings and 13824 compact group fingerprints, not collision-free certificates. Its 468 zero and 600 unit-magnitude integer reference positions include repetition and zero-input controls; they are not distinct nontrivial cancellation identities or necessarily real values of magnitude one after dyadic scaling. No approximate-dot/high-complex accuracy, raw invalid shape/overlap, infinity/NaN sweep, huge exponent, all-stride/length/alias, arbitrary thread, direct fmpzi helper, 32-bit/ARM/FFT or full stack qualification. The finite memory gate includes all setup/oracle/product-vector costs; not peak RSS or donor-only allocations. No matched Hyper benchmark has yet been run for the new candidate idea, so no performance, allocation, code/binary-size improvement is claimed.',
 findings:'Exact fused/complex paths preserve complete products before final rounding; exact sums merge near blocks and use the sign of well-separated remainder, whereas approximate dot intentionally discards intermediate error information. ARF addition uses separate bounded-size TLS scratch, and mantissa free-list caching is explicitly disabled. Exact complex limb kernels normalize where multiplication cost warrants it, accumulate into caller outputs to reduce scratch/copies and choose three products using operand shape; transformed reuse is read but disabled here. Hyper already has cache-evidence-gated Real three-product multiplication and word-sized/fused rational reducers. The current cold wide rational complex fallback still forms four products, so a shape-gated, common-scale three-product candidate deserves an isolated matched benchmark with small/unbalanced/cancellation controls; source arithmetic counts alone do not justify retention.',
 followup:'Investigate an isolated wide balanced exact-rational complex-product candidate, preserve word-sized and unbalanced paths and compare exact results, cache/trace/state behavior, matched CPU, separate allocation and representative size costs before retention. Continue remaining ARF/Arb-dot, generic matrix/field/algebraic support and every original reference; no scope narrowing or completion claim.'};
writeFileSync('arf-fused-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'ARF fused/complex cancellation binding',files:files.length,gates:tags.length,liveFiles:Object.keys(live).length,
 readRecords:readRecords.length,readLines:4131,coverage:m.coverageAtBinding,binaryBytes:m.binary.bytes,summary:m.checks.summary,status:m.status}));
