import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkMagnitudeControls} from './check-magnitude-controls.mjs';
const tags=['compile','compile-formatted','native','memcheck','linked-libraries','read-count-initial','output-check','inventory'].map(t=>'magnitude-'+t);
const failed=new Set(['magnitude-compile','magnitude-read-count-initial']);
const files=['flint-magnitude-controls.c','flint-magnitude-controls-initial.c','flint-arb-dot-controls.c',
 'check-magnitude-controls.mjs','record-magnitude-reads.mjs','record-magnitude-reads-initial.mjs','magnitude-read-selection.json',
 'bind-magnitude.mjs','verify-magnitude.mjs','arb-dot-experiment.json','capture.mjs'];
for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,failed.has(tag)?1:0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);}
const previous=json('arb-dot-experiment.json'),live=previous.liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
const readRecords=json('magnitude-read-selection.json').map(s=>{
 const r=coverage.find(r=>r.repo===s.repo&&r.path===s.path&&JSON.stringify(r.ranges)===JSON.stringify(s.newRanges));assert(r);return r;
});
const paths=[...readFileSync('results/magnitude-linked-libraries.stdout','utf8').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]);
const libraries=Object.fromEntries(paths.map(p=>[p,sha(p)]));assert.deepEqual(libraries,previous.libraries);
for(const[p,h]of Object.entries(previous.configurationFiles))assert.equal(sha(p),h,p);
const hyperReadRanges={'hyperreal/src/computable/approximation/arithmetic_kernels.rs':[[1,205]],
 'hyperreal/src/computable/node/bounds.rs':[[1,155]],'hyperreal/src/computable/approximation.rs':[[1,62]]};
for(const[p,ranges]of Object.entries(hyperReadRanges)){
 const content=readFileSync(resolve('../../../..',p),'utf8'),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(live[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
const binary='/tmp/calcium-magnitude.ipqBZw/magnitude-controls';
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:tags.map(tag=>({tag,code:failed.has(tag)?1:0,signal:null})),liveSources:live,libraries,
 configurationFiles:previous.configurationFiles,binary:{path:binary,bytes:statSync(binary).size,sha256:sha(binary)},
 readRecords,newDonorLines:3978,newFullFileReads:36,completedExistingPaths:['src/mag.h'],coverageAtBinding:effectiveSummary(),hyperReadRanges,
 reusedDonorFiles:['src/arb/fma.c','src/arb/addmul.c','src/arb/add_error.c'],checks:checkMagnitudeControls(),
 status:'Finite magnitude bounds, comparisons, constructors, FMA and radius inflation pass independent exact rational checks and focused Memcheck. Two corrected audit-harness/bookkeeping failures remain preserved. Full ecosystem audit incomplete.',
 production:'No production/donor edits or fifth continuation transfer; all 955 live hashes match checkpoint 38. The one audit executable reuses native libraries. Existing snapshots, gates, binaries and crash evidence preserved.',
 corpus:'7344 magnitude pairs from 12 normalized-or-zero mantissas, three base scales and 17 signed exponent gaps through 1025; 28 arithmetic routes with separate/valid whole-object aliases, excluding zero denominators and zero negative powers. Small unsigned powers [0,1,2,3,7], signed powers [-7,-1,0,1,2,3,8]. 680 constructor fixtures cover 17 integer-width positions through 513 bits, four patterns, both signs and five ordinary scales; 16 conversion/composed-integer routes. Ball controls use six widths [1,2,3,25,26,331], six families, four sign masks and eight precision positions: 5760 FMA/addmul results plus 720 error inflations, 12960 endpoint comparisons. All five inflation APIs also preserve the exact midpoint. Counts include repeated precisions, related operations and aliases, not distinct mathematical identities.',
 findings:'Upper and lower magnitude objects have different rounding contracts, not best-rounding guarantees. Decreasing operands such as denominators must be bounded in the opposite direction. Shifted additions/products carry explicit one/two-unit slack; deep-cancellation lower subtraction falls back to ARF. Binary64 root estimates are padded outward and normalized, and finite correctness is checked by exact squared inequalities. Bound constructors can be inexact even for exactly representable inputs. Fast operations require finite inline exponents in inputs and destinations; they are not universal drop-in replacements. Raw limb-window helpers are reached only through valid nonzero magnitude constructors. Hypot, promoted exponents, large power loops and remaining transcendental/tail functions are not newly numerically qualified.',
 comparison:'Hyper already separates structural Zero/Unknown/NonZero facts and marks approximate MSD planning estimates explicitly. Its demand-sized multiplication and reciprocal kernels use guard bits and refine rounded-zero divisors instead of treating a planning estimate as a proof. Importing fixed 30-bit magnitude values would change the representation/error contract without an established exactness, completeness or workload benefit. Keep the certification boundary and current scalar architecture; no nonredundant candidate is selected or benchmark advantage asserted.',
 corrections:'Initial compile failed only -Werror=misleading-indentation; frozen initial and formatted C have identical whitespace-stripped content. Initial read-ledger assertion expected 4078, but actual verified disjoint ranges sum to 3978. The one-constant correction was made before applying any new read records. Both failed gate outputs and initial sources are bound, not erased or described as passing numerical tests.',
 limits:checkMagnitudeControls().limits+' Supplemental relative quality uses 1023/1024..1025/1024 around the exact target, applied after squaring for root routes; this is not a general ulp theorem. Bound-quality and direction checks each cover 505048 outputs. No new Rust full-suite/debug/serialization/state-history/concurrency/WASM/application-size gates. Audit binary includes the unused frozen previous main, so its 47216 bytes are not a donor or Hyper size measurement.',
 followup:'Continue remaining magnitude transcendental/tail/combinatorial/IO support, unread ARF conversion ranges and other scalar support, including the explicit external MPFR high-helper source gap. Then continue generic field/matrix/algebraic and every original reference; full inventory reconciliation and unresolved historical transfers remain open.'};
assert.equal(files.length,35);assert.equal(m.binary.bytes,47216);
writeFileSync('magnitude-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'Magnitude bounds/error inflation binding',files:files.length,gates:tags.length,
 successfulGates:tags.length-failed.size,preservedFailedGates:failed.size,liveFiles:Object.keys(live).length,readRecords:readRecords.length,
 readLines:m.newDonorLines,coverage:m.coverageAtBinding,binaryBytes:m.binary.bytes,checks:m.checks}));
