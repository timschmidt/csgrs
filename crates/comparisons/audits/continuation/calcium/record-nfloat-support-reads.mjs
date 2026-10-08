import {readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha} from './derivative-demand-sources.mjs';
const here=dirname(fileURLToPath(import.meta.url));
const json=p=>JSON.parse(readFileSync(resolve(here,p)));
const coverage=json('coverage.json'),extensions=json('coverage-extensions.json');
const inventory=json('inventory.json').sources.find(s=>s.repo==='flint');
const specs=[
 ['src/nfloat/nfloat.c',311,3795,'Full remainder 311..3795, extending the frozen checkpoint-27 first 310 lines. Normalized limb imports and sign-dependent directed carry, fmpz/fmpq/ARF imports, cross-precision conversion, default arbitrary/truncating arithmetic, specialised one-to-four-limb add/sub and vector paths, high-product multiply/square, directed full-product fallbacks, scalar/vector multiply-accumulate, MPFR-backed inverse/divide/root operations, special values/status propagation and ARF-backed transcendental adapters. String parsing and generic transcendental adapters ignore directed rounding, but the documentation explicitly excludes those operations. nfloat_set_arf ignores the limb-converter return status and the overflow macro repeats underflow handling; boundary impact not numerically qualified. No failed-output inspection, invalid-representation or assertion reproduction. Fixed precision is not an exact-real tower replacement.'],
 ['src/nfloat/test/t-nfloat.c',1,271,'Whole arithmetic test file. Word-precision loop, status-based conversion checks, tolerance/Arb overlap arithmetic and dot tests, much smaller high-precision sample counts, special-function tests capped at 256 bits and small-limb conversion helpers compared with the same generic backend. Does not independently establish directed enclosure or general numerical correctness.'],
 ['doc/source/nfloat.rst',1,563,'Whole manual. Packed fixed-word precision up to 4224 bits; typical 1-2 ULP arithmetic, arbitrary default direction and platform-dependent rounding; errors can leave garbage outputs. Experimental directed subset explicitly includes simple conversions, scalar/vector arithmetic, dots and matrix multiply, excludes parsing, most generic mixed operations, transcendendentals and complex operations, and warns about underflow flushing. Fixed-point (-1,1) kernels require caller-proved intermediate/error bounds and lack overflow handling. Donor semantics prohibit using ordinary outputs as exact or outward-certified Hyper values.'],
 ['src/nfloat/dot.c',1,1187,'Whole real and complex dot implementation. Exponent lookahead, logarithmic length padding, fixed-width signed accumulators, one/two-limb high-product specialisations, skipped tiny terms and conservative directed error padding (not correctly rounded equality). Forward/reverse, initial, subtraction and empty inputs; 32-bit huge-length error-count overflow TODO remains unqualified. Complex separate accumulators and magnitude-dependent three-product path are outside the directed contract; associated proof TODO remains. Bounded numerical controls use padded empty-vector storage, not invalid-pointer tests.'],
 ['src/nfloat/mat_mul.c',1,1852,'Whole real/complex fixed-point, exponent-block and public dispatch. Precision/dimension tables, fixed-point intermediate/error bounds, exact fmpz block products with entrywise conversion, block memory estimates, scalar-dot fallback, whole-matrix alias temporaries and complex three-product layout. Public real directed mode always chooses classical multiplication; approximate fixed/block speed paths cannot be credited to the certified directed path. Called nfixed/fmpz kernels and general matrix/field-relation support remain open.'],
 ['src/gr_mat/mul_classical.c',1,115,'Whole generic classical matrix multiply. Shape rejection, zero inner dimension, whole-output A/B alias temporary, scalar product for inner dimension one and shallow-transposed B feeding vector dots otherwise. This is the public nfloat directed-matrix route. No arbitrary partially overlapping matrix-window contract inferred.'],
 ['src/gr_mat.h',1,95,'Read only 1..95: matrix entry layout/accessors, method signatures and dispatch macros, init/clear/check-resize, whole-struct swap, row/column permutations and beginning of window construction assertions. Remaining header declarations and window behavior remain open.']
];
const records=specs.map(([path,first,last,note])=>{
 const f=inventory.files.find(f=>f.path===path);assert(f?.text&&last<=f.lines);
 assert.equal(sha(resolve(here,'../../../../exact-real-references/flint',path)),f.sha256,path);
 if(first===1)assert(!coverage.some(e=>e.repo==='flint'&&e.path===path),path);
 else {assert.deepEqual(coverage.find(e=>e.repo==='flint'&&e.path===path).ranges,[[1,310]]);
  assert(!extensions.some(e=>e.repo==='flint'&&e.path===path));}
 return {repo:'flint',path,ranges:[[first,last]],note};
});
let patch='*** Begin Patch\n*** Update File: '+here+'/coverage.json\n@@\n [\n'+
 records.slice(1).map(e=>'+'+JSON.stringify(e)+',').join('\n')+'\n';
patch+='*** Update File: '+here+'/coverage-extensions.json\n@@\n [\n+'+JSON.stringify(records[0])+',\n';
patch+='*** Add File: '+here+'/nfloat-support-read-selection.json\n'+
 JSON.stringify(records.map((e,i)=>({repo:e.repo,path:e.path,newRanges:e.ranges,extension:i===0})),null,2)
 .split('\n').map(s=>'+'+s).join('\n')+'\n*** End Patch';
console.log(patch);
