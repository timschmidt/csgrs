import {readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const coverage=JSON.parse(readFileSync(resolve(here,'coverage.json')));
const inventory=JSON.parse(readFileSync(resolve(here,'inventory.json'))).sources.find(s=>s.repo==='flint');
const specifications=[
 ['src/nfloat/ctx.c',238,'Full method-table and context setup. Immutable word-granularity fixed precision, inline element size, approximate rather than exact field predicates, static method table lazily initialized with a plain flag, full scalar/vector/matrix registrations including commented omissions. No concurrency proof or direct replacement for Hyper exact scalar representation.'],
 ['src/nfloat/mat.c',171,'Full precision-dependent cutoffs for real/complex triangular solves, LU and real LQ; classical, recursive and Gram-Schmidt routes vary with limb cost and matrix dimension. Benchmark-calibrated dispatch is a useful principle, but these donor thresholds are not evidence for Hyper small fixed-dimension matrices or exact scalars.'],
 ['src/nfloat/test/t-nfloat_directed.c',364,'Full tests: floor/ceil at word precisions through 320, status-conditional ARF directional inequalities for scalar operations and short random forward/reverse dots with initial/subtract toggles, conversions, tolerance-based Arb comparisons and reduced high-precision/special-function sample counts. Comparisons certify enclosure direction, not equal correctly rounded results; addmul/submul accumulate from previously computed approximate states. Vector TODO and small directed-dot length leave support scope open. Failure-only printing passes the nfloat pointer to arf_printd; no failure-path reproduction performed.'],
 ['src/nfloat.h',570,'Full layout, flags, bounds, inline constructors/normalization, API declarations and real/complex/fixed-point dispatch interfaces. Fixed precision up to 4224 bits on both limb widths enables bounded local storage, unlike general lazy exact Hyper values. Directed rounding is explicitly experimental. NFLOAT_HANDLE_OVERFLOW text repeats the underflow condition and handler; public route and impact not validated here. This and unsoundly treating default truncation as an enclosure prohibit wholesale backend adoption.'],
 ['src/nfloat/nfloat.c',310,'Read 1..310 only: MPFR/ARF modes default to truncation toward zero, sign-dependent outward rounding selection, I/O, random conversion, copy/vector initialization, structural equality, units and special-value flags, underflow/overflow statuses, exact machine-integer constructors and start of limb conversion. Notable unreachable context clear after nfloat_write return is recorded as source observation, not a measured resource leak. Remaining arithmetic/conversions and called nfloat kernels remain unread/open.']
];
const entries=specifications.map(([path,last,note])=>{
 assert(!coverage.some(e=>e.repo==='flint'&&e.path===path),path);
 const f=inventory.files.find(f=>f.path===path);assert(f?.text);assert(last<=f.lines);
 return {repo:'flint',path,ranges:[[1,last]],note};
});
console.log('*** Begin Patch\n*** Update File: '+here+'/coverage.json\n@@\n [\n'+
 entries.map(e=>'+'+JSON.stringify(e)+',').join('\n')+
 '\n*** Add File: '+here+'/derivative-qualification-read-selection.json\n'+
 JSON.stringify(entries.map(e=>({repo:e.repo,path:e.path,newRanges:e.ranges})),null,2).split('\n').map(l=>'+'+l).join('\n')+'\n*** End Patch');
