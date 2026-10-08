import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,sha,json} from './derivative-demand-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkComplexControls,checkFilterProbe} from './check-nfloat-complex-controls.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const specs=[['nfloat-complex-compile',1],['nfloat-complex-compile-v2',0],['nfloat-complex-native',0],
 ['nfloat-complex-memcheck',0],['nfloat-complex-hyper-debug',0],['nfloat-complex-hyper-release',0],
 ['nfloat-complex-linked-libraries',0],['nfloat-complex-inventory',0],['nfloat-complex-filter-probe-build',101],
 ['nfloat-complex-filter-probe-build-v2',0],['nfloat-complex-filter-probe-native',0],['nfloat-complex-output-check',0]];
const files=['bind-nfloat-complex.mjs','verify-nfloat-complex.mjs','check-nfloat-complex-controls.mjs',
 'record-nfloat-complex-reads.mjs','nfloat-complex-read-selection.json','flint-nfloat-complex-controls.c',
 'flint-nfloat-complex-controls-initial.c','nfixed-experiment.json','capture.mjs',
 'sign-filter-probe/Cargo.toml','sign-filter-probe/Cargo.lock','sign-filter-probe/src/main.rs','sign-filter-probe-initial.rs'];
for(const[tag,code]of specs) {
 const g=json('results/'+tag+'.json');assert.equal(g.code,code,tag);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
}
const candidateSources=sources(),coverage=json('coverage.json');
assert.deepEqual(candidateSources,json('nfixed-experiment.json').candidateSources);
for(const[p,h]of Object.entries(candidateSources.candidate))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
const paths=[...readFileSync(resolve(here,'results/nfloat-complex-linked-libraries.stdout'),'utf8').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]);
const libraries=Object.fromEntries(paths.map(p=>[p,sha(p)]));assert.deepEqual(libraries,json('nfixed-experiment.json').libraries);
const binary='/tmp/calcium-complex-controls.8yAaAz/nfloat-complex-controls',probe='/tmp/calcium-complex-controls.8yAaAz/sign-filter-probe';
const binaries=[binary,probe].map(path=>({path,sha256:sha(path),bytes:statSync(path).size}));
const manifest={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:specs.map(([tag,code])=>({tag,code,signal:null})),candidateSources,libraries,binaries,
 reads:json('nfloat-complex-read-selection.json').map(s=>coverage.find(e=>e.repo===s.repo&&e.path===s.path)),
 coverageAtBinding:effectiveSummary(),nfloatDirectory:effectiveCoverage().filter(e=>e.repo==='flint'&&e.path.startsWith('src/nfloat/')).map(e=>({path:e.path,ranges:e.ranges})),
 controls:checkComplexControls(),probe:checkFilterProbe(),hyperReadRanges:{
  'hyperlattice/src/complex.rs':[[1,555]],'hyperlattice/tests/complex.rs':[[1,89]],
  'hyperlattice/benches/mathbench/complex_ops.rs':[[1,509]],'hyperlattice/fuzz/fuzz_targets/complex_ops.rs':[[1,193]],
  'hyperlattice/Cargo.toml':[[1,49]],'hyperlattice/src/point.rs':[[250,280]],
  'hyperlimit/src/resolve.rs':[[1,180],[240,430]],'hyperlimit/src/predicates/ring.rs':[[275,367]],
  'hyperlimit/src/trace.rs':[[1,13]],'hyperlimit/src/geometry/point.rs':[[1,7]],
  'hyperlimit/src/lib.rs':[[195,215]],'hyperlimit/Cargo.toml':[[1,43]],
  'hyperreal/src/dispatch_trace.rs':[[1,22],[500,549],[701,738]]
 },
 production:'No new production or donor change. All 955 live source/support hashes remain unchanged; the four prior continuation transfers remain retained.',
 status:'All 26 tracked files under src/nfloat are read. This closes that directory only, not called support, full Calcium/FLINT inventories or the ecosystem audit.',
 limits:'Finite 64-bit ordinary-approximate complex controls only: all 66 word precisions, 32 deterministic patterns, 14 arithmetic operations, three whole-output placements and 2112 exact squared-norm comparisons. Zero reciprocal/division cases are skipped, never inspected as failed outputs. Accuracy tolerance is 64*2^-p*max(1,absolute expected components); abs/sqrt/rsqrt use exact rational squared residuals and principal-branch signs. This is not correct-rounding, directed complex enclosure, arbitrary componentwise relative error, all FENV/operand Cartesian combinations, arbitrary alias overlap, concurrent use, 32-bit, nonfinite, extreme exponent, matrix/dot or transcendental qualification. Native/Memcheck stdout is identical for this corpus; the earlier independent FENV discrepancy remains unresolved and is not generalized away. Memory counts include the exact oracle, not donor-only performance or peak RSS. Hyper integration tests are unchanged and use shared Real arithmetic in their expected values, not a new independent scalar oracle. No timing/size improvement is claimed.',
 followup:'Public strict ring-area fixture has twice-area pi-103993/33102, with 3/4/8/16/32 vertices obtained by subdividing a straight edge. Each of three same-object queries per size reaches the >4-term mixed-sign filter and then exact refinement; rational and well-separated pi controls bypass it. First local query is not process-cold. This is reachability only, not a replacement prototype or performance qualification. Next isolate a constant-state sign summary preserving Unknown short-circuit, zero/mixed trace order and public decision/state behavior, then measure matched CPU/allocations/retention/binary size before retaining anything. Remaining high-product/ARF/generic matrix/field and original reference work stays open.'};
writeFileSync(resolve(here,'nfloat-complex-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'nfloat complex binding',files:files.length,gates:specs.length,reads:manifest.reads.length,
 binaryBytes:binaries.reduce((s,b)=>s+b.bytes,0),binaries:binaries.length,sourceFiles:955,
 coverageAtBinding:manifest.coverageAtBinding,nfloatFiles:manifest.nfloatDirectory.length,controls:manifest.controls.summary,probe:manifest.probe}));
