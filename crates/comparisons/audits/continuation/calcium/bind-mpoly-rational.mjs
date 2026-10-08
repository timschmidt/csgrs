import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkMpolyRational} from './check-mpoly-rational.mjs';
import {checkMpolyRationalUpstream} from './check-mpoly-rational-upstream.mjs';
const tags=['compile','compile-fixed','native','linked-libraries','memcheck','output-check',
 'upstream-compile','upstream-native','upstream-linked-libraries','upstream-check'].map(s=>'mpoly-rational-'+s);
const files=['flint-mpoly-rational-controls.c','flint-mpoly-rational-controls-v2.c',
 'check-mpoly-rational.mjs','check-mpoly-rational-upstream.mjs','record-mpoly-rational-reads.mjs',
 'record-mpoly-rational-support.mjs','mpoly-rational-read-records.json','mpoly-rational-support-read-records.json',
 'mpoly-rational-findings.md','bind-mpoly-rational.mjs','verify-mpoly-rational.mjs','arf-conversion-experiment.json',
 'capture.mjs',...['json','stdout','stderr'].map(ext=>'results/arf-conversion-verify-full.'+ext)];
const gates=tags.map(tag=>{const g=json('results/'+tag+'.json');assert.equal(g.code,tag==='mpoly-rational-compile'?1:0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
 return{tag,code:g.code,signal:g.signal,cwd:g.cwd,command:g.command,args:g.args};});
const previous=json('arf-conversion-experiment.json'),live=previous.liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
for(const prefix of ['mpoly-rational','mpoly-rational-upstream']) {
 const linked=readFileSync('results/'+prefix+'-linked-libraries.stdout','utf8');
 const libraries=Object.fromEntries([...linked.matchAll(/=> (\/[^ ]+) \(/g)].map(m=>[m[1],sha(m[1])]));assert.deepEqual(libraries,previous.libraries);
}
for(const[p,h]of Object.entries(previous.configurationFiles))assert.equal(sha(p),h,p);
const binaries=['controls','upstream'].map(name=>{const path='/tmp/calcium-mpoly-rational.Hmk8Is/'+name;return{path,bytes:statSync(path).size,sha256:sha(path)};});
const checks=checkMpolyRational(),upstreamChecks=checkMpolyRationalUpstream();
assert.deepEqual(checks,json('results/mpoly-rational-output-check.stdout'));assert.deepEqual(upstreamChecks,json('results/mpoly-rational-upstream-check.stdout'));
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates,
 liveSources:live,libraries:previous.libraries,configurationFiles:previous.configurationFiles,binaries,
 readRecords:[...json('mpoly-rational-read-records.json'),...json('mpoly-rational-support-read-records.json')],
 newDonorLines:3955,newCompleteFiles:38,newPartialFiles:2,completeCurrentSliceFiles:36,completeCurrentSliceLines:3564,
 coverageAtBinding:effectiveSummary(),hyperReadRanges:{
  'hyperreal/src/rational/arithmetic/ops.rs':[[160,305],[550,586]],
  'hypersolve/src/algebraic_fiber.rs':[[1125,1160],[3060,3208],[3260,3415]],
  'hypersolve/src/algebraic_rational_image.rs':[[1935,2025]]},checks,upstreamChecks,
 status:'Current36-file multivariate rational-function implementation/test/header/manual slice is source-complete; bounded independent canonical arithmetic and all15 upstream tests pass. Archived code, recursive support and full ecosystem remain incomplete.',
 production:'No production/donor edit, new transfer, cleanup, deletion, commit, push or external report. All956 live hashes preserve the five retained continuation improvements. Existing native libraries reused; no Rust/whole-native rebuild or source copy.',
 findings:'Denominator-GCD staging, scalar-content cancellation and opposite-side pre-cancellation preserve canonical formal rational functions while controlling expansion. Upstream tests share GCD/polynomial references, use only lexicographic contexts and omit explicit scalar-wrapper in-place output cases. Independent BigInt full-polynomial and known primitive-linear-factor certificates cover23841 complete values,13797 aliases and three monomial orders across1/2/4 variables. No numerical defect observed in this corpus.',
 comparison:'Hyperreal already has rational cross-cancellation and restricted possible-divisor reduction. Hypersolve recognizes common denominators, allocation-free one and bounded-degree residue arithmetic. Its partial Real coefficients and selected-root nonzero obligations differ from a total Q[x] rational-function field. Existing rational-image cancellation is certificate-guarded; no unchecked formal cancellation or wholesale representation transfer is justified.',
 preservedFailure:'Initial audit compile exit1: omitted required GMP header. Original source/gate retained, no initial executable. Corrected source differs only by the standard-I/O/GMP prelude and trailing newline; independently re-derived. No donor correction or discarded numerical fixture.',
 limits:checks.limits+' '+upstreamChecks.limits+' Both finite corpus and upstream runs are native64; focused Memcheck applies to the independent corpus only. New source credit3955 lines/40 records;38 complete and two partial additions. Two executables total60840 /tmp bytes and paired numerical logs6174888 workspace bytes. All previous numerical/FENV/LLL/thread-memory failures remain preserved.',
 followup:'Read archived rational-function implementation/tests and current generic-ring/fexpr bridges, recursive polynomial/GCD and remaining scalar support. Continue every original formal/symbolic/historical reference, unresolved transfer experiment and full inventory reconciliation. Field-specific pre-cancellation needs separate nonzero proofs and matched measurements before any retention.'};
assert.equal(files.length,46);assert.equal(m.readRecords.length,40);assert.deepEqual(binaries.map(b=>b.bytes),[24280,36560]);
writeFileSync('mpoly-rational-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({files:46,gates:10,successfulGates:9,preservedFailedGates:1,liveFiles:956,readRecords:40,newDonorLines:3955,
 coverage:m.coverageAtBinding,binaryBytes:60840,checks,upstreamChecks}));
