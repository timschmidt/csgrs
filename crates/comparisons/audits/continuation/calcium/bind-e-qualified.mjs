import {writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {specifications} from './e-qualified-gates.mjs';
import {checkEQualified} from './check-e-qualified.mjs';
const draft=process.argv.includes('--draft'),sourceMap=sources(false),gates=specifications(!draft),files={};
const live=draft?sourceMap.baseline:sourceMap.candidate;for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
const selected=['e-plan-experiment.json','e-plan-binaries.json','e-plan-qualified-origin.json','prepare-e-plan-qualified.mjs',
 'e-plan-qualified-sources.mjs','run-e-qualified-gates.mjs','e-qualified-wasm.rs','prepare-e-qualified-wasm.mjs',
 'run-e-qualified-wasm.mjs','e-qualified-wasm-binaries.json','e-qualified-wasm-check-summary.json','results/e-qualified-wasm-check.jsonl',
 'run-e-qualified-wasm-costs.mjs','e-qualified-wasm-cpu-summary.json','results/e-qualified-wasm-cpu.jsonl',
 'run-e-qualified-controls.mjs','e-qualified-controls-summary.json','results/e-qualified-controls.jsonl',
 'measure-e-qualified-app-size.mjs','measure-e-qualified-app-size.initial.mjs','e-qualified-app-size-failure.md','e-qualified-app-size-summary.json',
 'record-e-qualified-reads.mjs','e-qualified-read-records.json','check-e-qualified.mjs','e-qualified-gates.mjs',
 'bind-e-qualified.mjs','verify-e-qualified.mjs','make-e-qualified-live-patch.mjs','run-e-qualified-live.mjs','capture.mjs',
 'bind-e-qualified.initial.mjs','verify-e-qualified.initial.mjs','make-e-qualified-live-patch.initial.mjs',
 'e-qualified-experiment-draft.json','e-qualified-draft-range-correction.md',
 ...(!draft?['e-qualified-experiment-draft-v2.json','results/e-qualified-verify-draft.json','results/e-qualified-verify-draft.stdout','results/e-qualified-verify-draft.stderr']:[]),
 ...['baseline','candidate'].flatMap(v=>['Cargo.toml','Cargo.lock','kernel.rs'].map(p=>'e-qualified-wasm-'+v+'/'+p))];
for(const p of selected)files[p]=sha(p);
for(const g of gates) {
 const recorded=json('results/'+g.tag+'.json');assert.equal(recorded.code,0,g.tag);assert.equal(recorded.signal,null);
 for(const k of ['cwd','command','args'])assert.deepEqual(recorded[k],g[k],g.tag+': '+k);
 for(const ext of ['json','stdout','stderr']){const p='results/'+g.tag+'.'+ext;files[p]=sha(p);}
}
assert.equal(gates.length,draft?50:55);
const binaries={};
for(const b of Object.values(json('e-qualified-wasm-binaries.json').modules))binaries[b.path]={sha256:b.sha256,bytes:b.bytes};
for(const b of json('e-qualified-app-size-summary.json').artifacts.flatMap(a=>a.files))binaries[b.path]={sha256:b.sha256,bytes:b.bytes};
let binaryBytes=0;for(const[p,b]of Object.entries(binaries)){assert.equal(sha(p),b.sha256);assert.equal(statSync(p).size,b.bytes);binaryBytes+=b.bytes;}
const m={schema:1,recorded:new Date().toISOString(),draft,files,gates,sourceMap,liveSources:live,binaries,binaryBytes,
 readRecords:json('e-qualified-read-records.json'),newDonorLines:1176,completedArfTopLevelCFiles:41,coverageAtBinding:effectiveSummary(),
 hyperReadRanges:{'hyperreal/AGENTS.md':[[1,14]],'hyperreal/Cargo.toml':[[1,84]],'hyperreal/fuzz/Cargo.toml':[[1,52]],
  'hyperlattice/Cargo.toml':[[1,49]],'hyperlimit/Cargo.toml':[[1,43]],'hypertri/Cargo.toml':[[1,91]],'hypersolve/Cargo.toml':[[1,63]],
  'hypercurve/Cargo.toml':[[1,149]],'hypercurve/.github/workflows/ci.yml':[[1,103]],'hypercurve/examples/basic.rs':[[1,29]],
  'hypercurve/examples/arrangement.rs':[[1,44]],'hyperreal/examples/readme_quickstart.rs':[[1,14]],
  'hyperreal/src/computable/approximation.rs':[[1,62]],'hyperreal/src/computable/approximation/constants.rs':[[1,150]],
  'hyperreal/tests/numerical_cross_reference.rs':[[1,100]],'hyperreal/src/real/arithmetic/representation.rs':[[320,440]],
  'hyperreal/src/rational/arithmetic/construction.rs':[[670,735]],'hyperreal/src/real/arithmetic/elementary_functions.rs':[[4170,4200]]},
 checks:checkEQualified(),
 preCaptureFailure:{source:'measure-e-qualified-app-size.initial.mjs',note:'e-qualified-app-size-failure.md',
  invalidTag:'e-qualified-app-baseline-hyperreal-readme_quickstart-strip',error:'Usage: capture.mjs TAG CWD COMMAND [ARG ...]',
  root:'/tmp/calcium-e-qualified-apps.AHiwfn',preservedSuccessfulGate:'e-qualified-app-baseline-hyperreal-build'},
 production:{retained:!draft,paths:[...sourceMap.changed,...sourceMap.added],algorithmNetLines:8,testModuleLines:112,testRegistrationLines:4,
  previousFiles:955,qualifiedFiles:956,priorSnapshot:'derivative-demand-candidate',qualifiedSnapshot:'e-plan-qualified-candidate',
  note:'Planning only. Exact series/splitting/rounding/cache/representation code unchanged; no runtime dependency added. Four prior retained continuation changes and pre-existing user changes preserved.'},
 status:draft?'Qualified lower-factorial e planner; production retention pending live source application and gates. Full ecosystem remains open.':
  'Retained lower-factorial e term planning in Hyperreal, the fifth continuation transfer. Source inventory and full ecosystem audit remain incomplete.',
 findings:'Four permanent regressions add exact threshold neighborhoods, directed e kernels, public refinement/coarsening and cancellation/serde recovery. Candidate756 default and859 all-feature tests pass in both profiles;24 doctests. Both consumers match exact test membership:803 Hypersolve and1764 Hypercurve release tests, nine prior ignored curve cases remain unrun. Strict scalar/consumer Clippy, default/fuzz checks and supported WASM libraries pass. WASM full-word outputs pass8302 exact plans and600 independent complete-tail enclosures, and all186 kernel outputs match native exactly. Initial app-size tag validation failed before capture/strip; original script and existing executable are preserved and resumed without another snapshot directory.',
 comparison:'Native40-block follow-up1600 observations confirms high-precision cold e gains, but262144-bit coarsening retains a1.027059 paired ratio (~59ns/query). WASM22 groups1056 observations:12 intervals below one,1 above,9 overlap; four cold public queries improve. Warm4096-bit e has a1.053223 interval (~2.9ns/query). These costs remain accepted/disclosed, not erased as noise. Native allocation gains from checkpoint42 are unchanged; public/kernel peak and live demand are not reduced. Stripped curve examples shrink944/928 bytes; scalar example is byte-identical. WASM oracle module shrinks507 bytes; test/oracle modules are not full application size. Reproducible cold/uncached speed and allocation benefits plus small code change justify retention, not a universal gain.',
 sourceFindings:'All41 inventoried top-level ARF C files are source-read after18 new complete files/1176 lines. Tests, public header ranges and recursive MPFR/GMP internals remain separate. ARF finite-dyadic exponent/valuation rounding and sticky quotient correction are representation-specific; Hyper already separates exact rational storage from certified partial rounding and total multivalued near-integer selection. ARF nearest-even differs from Hyper certified ties-away rounding; NaN comparison conventions and nonnegative root domains are not general exact-real decisions. MPFR wrapper state handling is source-only review, not newly reproduced. No additional backend/rounding transfer selected.',
 limits:'Not full ecosystem coverage, arbitrary precision/history proof, physical ARM/RISC-V execution, all-FENV, whole-Alumina timing/size, default-only downstream matrix, all ignored stress tests, full CI or peak RSS qualification. WASM consumer feature set excludes comparative benchmark dependencies; library compile gates do not run the browser demo. WASM tests run locally in Node22/V8, not an embedded device. New source-read ARF routines have no new independent numerical campaign here. Prior focused Memchecks are reused because production numerical code is identical to checkpoint42 except planner; test-only additions do not extend memory qualification. Earlier numerical failures, LLL assertion, thread-memory findings, and initial failed42 audit build remain preserved. Per-group bootstrap intervals are unadjusted; targeted controls supplement rather than replace the original71 groups. Dedicated snapshots total108188255 bytes plus the one45446168-byte source copy; shared build cache grows separately. No donor edit, cleanup/deletion, commit, push or external report.',
 followup:'Finish ARF tests/header and MPFR/GMP high-helper support with bounded independent numerical oracles as warranted, then remaining generic field/matrix/algebraic and all original formal/symbolic/historical references. Reconcile every requested inventory item and older open transfer before audit completion.'};
assert.equal(binaryBytes,108188255);
writeFileSync(draft?'e-qualified-experiment-draft-v2.json':'e-qualified-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({draft,files:Object.keys(files).length,gates:gates.length,liveFiles:Object.keys(live).length,binaries:Object.keys(binaries).length,binaryBytes,newDonorLines:1176}));
