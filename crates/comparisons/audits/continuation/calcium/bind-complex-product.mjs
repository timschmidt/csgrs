import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json,sources,here} from './complex-product-sources.mjs';
import {checkComplexProduct} from './check-complex-product.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
const previous=json('arf-fused-experiment.json'),sourceMap=sources(),frozen=json('complex-product-binaries.json');
assert.deepEqual(sourceMap,frozen.sourceMap);assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const gates=['build-baseline','build-candidate','freeze','check-baseline','check-candidate',
 'trace-build-candidate','memcheck-baseline','memcheck-candidate','trace-build-baseline',
 'trace-baseline','trace-candidate','allocation','toolchain','machine','cpu','output-check'].map(s=>'complex-product-'+s);
const names=['prepare-complex-product.mjs','complex-product-origin.json','complex-product-sources.mjs',
 'run-complex-product-costs.mjs','freeze-complex-product-traces.mjs','complex-product-binaries.json',
 'complex-product-trace-binaries.json','complex-product-corpus.rs','complex-product-cpu.rs',
 'complex-product-allocation.rs','complex-product-check.rs','complex-product-trace.rs',
 'check-complex-product.mjs','bind-complex-product.mjs','verify-complex-product.mjs',
 'complex-product-app-baseline/Cargo.toml','complex-product-app-baseline/Cargo.lock',
 'complex-product-app-candidate/Cargo.toml','complex-product-app-candidate/Cargo.lock',
 'complex-product-cpu-summary.json','complex-product-allocation-summary.json',
 'results/complex-product-cpu.jsonl','results/complex-product-allocation.jsonl'];
for(const tag of gates)for(const ext of ['json','stdout','stderr'])names.push('results/'+tag+'.'+ext);
assert.equal(new Set(names).size,71);
const hyperReadRanges={
 'hyperreal/AGENTS.md':[[1,14]],
 'hyperreal/Cargo.toml':[[1,84]],'hyperlattice/Cargo.toml':[[1,49]],
 'hyperreal/src/rational/arithmetic/construction.rs':[[1,120],[280,830]],
 'hyperreal/src/rational/arithmetic/representation.rs':[[1,160]],
 'hyperreal/src/rational/arithmetic/aggregate_products.rs':[[2790,2930],[3541,3591],[3845,3893],[4180,4470],[5038,5213]],
 'hyperreal/src/rational/arithmetic/ops.rs':[[925,977]],
 'hyperreal/src/rational/arithmetic/tests.rs':[[4340,4490],[4715,4855]],
 'hyperreal/src/rational/arithmetic/toom4_multiplication.rs':[[1,138]],
 'hyperreal/src/rational/arithmetic/queries_conversion.rs':[[180,225]],
 'hyperreal/src/real/arithmetic/facts.rs':[[45,105],[148,181],[475,550]],
 'hyperreal/src/dispatch_trace.rs':[[1,120]],
 'hyperlattice/src/complex.rs':[[1,125],[270,325]],
};
for(const[p,ranges]of Object.entries(hyperReadRanges)) {
 const content=readFileSync(resolve(here,'../../../..',p),'utf8'),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(sourceMap.baseline[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
const donorPath=resolve(here,'../../../../exact-real-references/flint/src/mpn_extras/mul_complex.c');
const binaryBytes=[...Object.values(frozen.binaries),...Object.values(json('complex-product-trace-binaries.json'))].reduce((n,b)=>{assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);return n+b.bytes;},0);
const m={schema:1,recorded:new Date().toISOString(),status:'Isolated v1 experiment qualified for the recorded finite corpus and costs; not selected as implemented.',
 files:Object.fromEntries(names.map(p=>[p,sha(p)])),sourceMap,liveSources:previous.liveSources,hyperReadRanges,
 donorReread:{path:donorPath,sha256:sha(donorPath),ranges:[[495,535]],newCoverageLines:0},
 coverageAtBinding:effectiveSummary(),checks:checkComplexProduct(),binaryFiles:8,binaryBytes,
 gates:gates.map(tag=>{const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);return{tag,code:g.code,signal:g.signal};}),
 production:'All 955 retained live hashes unchanged; four previous continuation transfers remain. No donor or production edit, cleanup, deletion, commit or push.',
 findings:[
  'Shared per-operand rational denominators allow an exact three-product complex numerator without an LCM. Final ordinary reduction handles signs, cancellation and common factors; both ordinary and conjugate products are exercised.',
  'The existing word path precedes the candidate. The shape estimate is only a heuristic: nine selected 192-bit workload groups regress, and bypass/cached controls also have measured costs.',
  '75 selected groups: 59 per-group intervals below parity, nine above, seven overlapping. Across all 192 groups: 64 below, 43 above, 85 overlapping. No multiplicity adjustment or universal gain claim.',
  'Request demand falls but peak demand rises in 66 groups. The helper retains signed input sums through its scope; shortening their lifetime is a concrete next hypothesis, not a measured v2 benefit.',
  'The public lattice first-observation and repeated-use dispatch labels agree between variants. The scalar selector is reached 2520 times in the trace corpus; repeated public multiplication keeps the existing reuse schedule.',
 ],
 limits:'Release finite GMP oracle through 2048 bits; separate trace/exact num-rational comparison through 65536 bits shares the Rust integer backend. No new full crate suites, debug-profile corpus, serialization/cancellation/concurrency histories, internal unreduced-storage campaign, quotient CPU/allocation campaign, representative application size, WASM/other architectures, huge backend cutoffs or universal memory/error proof. CPU/allocation exclude input construction and pool lifetime. Earlier failed gates remain preserved by the preceding chain.',
 followup:'Keep v1 isolated. Investigate a measured crossover and lower-overhead out-of-line dispatch plus shorter temporary lifetimes in a separate v2; repeat matched costs and exactness gates before broader state/downstream qualification or retention. Continue all remaining original reference and supporting-source work.',
};
writeFileSync(resolve(here,'complex-product-experiment.json'),JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({files:names.length,gates:gates.length,liveSources:955,binaryFiles:8,binaryBytes,checks:m.checks,status:m.status}));
