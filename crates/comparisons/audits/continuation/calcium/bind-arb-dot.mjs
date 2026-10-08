import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkArbDotControls} from './check-arb-dot-controls.mjs';
const tags=['compile','native','memcheck','linked-libraries','output-check','inventory'].map(t=>'arb-dot-'+t);
const files=['flint-arb-dot-controls.c','check-arb-dot-controls.mjs','record-arb-dot-reads.mjs','arb-dot-read-selection.json',
 'bind-arb-dot.mjs','verify-arb-dot.mjs','complex-product-v2-experiment.json','arf-fused-experiment.json','capture.mjs'];
for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);}
const previous=json('complex-product-v2-experiment.json'),nativePrevious=json('arf-fused-experiment.json');
const live=previous.liveSources;for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
const readRecords=json('arb-dot-read-selection.json').map(s=>{
 const r=coverage.find(r=>r.repo===s.repo&&r.path===s.path&&JSON.stringify(r.ranges)===JSON.stringify(s.newRanges));assert(r);return r;
});
const paths=[...readFileSync('results/arb-dot-linked-libraries.stdout','utf8').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]);
const libraries=Object.fromEntries(paths.map(p=>[p,sha(p)]));assert.deepEqual(libraries,nativePrevious.libraries);
for(const[p,h]of Object.entries(nativePrevious.configurationFiles))assert.equal(sha(p),h,p);
const hyperReadRanges={'hyperlattice/src/vector.rs':[[240,278]],
 'hyperreal/src/rational/arithmetic/aggregate_products.rs':[[3880,4035],[4285,4365]]};
for(const[p,ranges]of Object.entries(hyperReadRanges)){
 const content=readFileSync(resolve('../../../..',p),'utf8'),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(live[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
const binary='/tmp/calcium-arb-dot.zsWvwv/arb-dot-controls';
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:tags.map(tag=>({tag,code:0,signal:null})),liveSources:live,libraries,configurationFiles:nativePrevious.configurationFiles,
 binary:{path:binary,bytes:statSync(binary).size,sha256:sha(binary)},readRecords,newDonorLines:3327,
 coverageAtBinding:effectiveSummary(),hyperReadRanges,checks:checkArbDotControls(),
 status:'Bounded finite public Arb dot and all five integer wrappers pass complete rational endpoint containment and focused Memcheck. Source coverage, mathematical qualification and transfer selection remain distinct; full ecosystem audit incomplete.',
 production:'No production or donor changes; all 955 live source hashes match checkpoint 37. Four retained continuation transfers remain unchanged. No new performance or size claim and no fifth transfer.',
 corpus:'648 fixture groups, 1224 operation groups, 198288 results and 396576 lower/upper endpoint comparisons. Three ball routines each have 46656 calls; each of five integer wrappers has 11664. Twelve ball limb widths [1,2,3,11,12,13,24,25,26,331,332,333], six exact/dense/narrow-radius/wide-radius/cancellation/sparse families, lengths [0,1,2,5], three layouts [(1,1),(-1,1),(2,-2)], both subtract values, absent/separate/output-aliased initial, nine precision positions [2,63,64,65,127,128,129,64*w,128*w+1]. Typed wrappers use x widths 1,3,26; fmpz coefficient sizes reach 258 bits. Zero radius and cancellation counters are repeated cases, not distinct identities. Precision positions can coincide. Inputs touch source-relevant width/shift boundaries, but no instrumented branch-coverage or universal crossover claim.',
 findings:'Fixed-point midpoint accumulation can avoid repeated full product vectors, provided term truncation, propagated input uncertainty and final rounding remain separately certified. Radius/actual-product-bottom precision caps avoid resolving below the useful input uncertainty. Integer adapters build shallow normalized exact views, with one batched shift buffer for unnormalized big coefficients. The precise reference materializes products and uses upward radius accumulation; it is not a separate arithmetic backend or a tightest rectangle enclosure. Small/lagom and generic/nonfinite fused support are read with clear execution limits. MPFR high-product helper symbols are externally linked; their source is not newly audited or conflated with the previously failing FLINT high-product bound assertions.',
 comparison:'Hyperlattice shared-scale dot already carries certified exact factors into Hyperreal without repeated proof, and Hyperreal dyadic product-sum plans account for scale/headroom with word, stack and big-integer schedules plus delayed reduction. These exact-value paths cannot accept truncated products as exact. No specific nonredundant transfer with measured benefit was established; retaining existing architecture is the disposition, not a benchmark claim that it is globally faster.',
 limits:checkArbDotControls().limits+' Direct source-only exceptional branches remain unqualified. No additional Rust full-suite/debug/state-history/serialization/concurrency/application or WASM tests were run. The one 27768-byte audit executable reuses existing native libraries; no whole FLINT build or destructive cleanup.',
 followup:'Continue remaining Arb/magnitude and scalar support, then generic field/matrix/algebraic support and every original reference. MPFR high-helper source remains an explicit supporting dependency gap. Reconcile the complete original inventory before any completion claim; preserve all prior failures, snapshots, binaries and crash evidence.'};
assert.equal(files.length,27);assert.equal(m.binary.bytes,27768);
writeFileSync('arb-dot-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'Arb dot enclosure binding',files:files.length,gates:tags.length,liveFiles:Object.keys(live).length,
 readRecords:readRecords.length,readLines:m.newDonorLines,coverage:m.coverageAtBinding,binaryBytes:m.binary.bytes,checks:m.checks}));
