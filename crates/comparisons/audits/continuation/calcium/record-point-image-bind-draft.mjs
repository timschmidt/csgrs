import {writeFileSync,existsSync} from 'node:fs';
import assert from 'node:assert/strict';
import {effectiveSummary} from './effective-coverage.mjs';
import {sha,json} from './point-image-sources.mjs';
import {pointImageEvidence} from './point-image-evidence.mjs';
assert(!existsSync('point-image-manifest.json'));
const evidence=pointImageEvidence(),previous=json('power-sums-manifest.json');
assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['point-image-findings.md','point-image-evidence.mjs','record-point-image.mjs','verify-point-image.mjs',
 'prepare-point-image.mjs','point-image-origin.json','check-point-image.mjs','point-image-binaries.json','freeze-point-image.mjs',
 'run-point-image-gates.mjs','prepare-point-image-test-cleanup.mjs','point-image-test-cleanup.json','point-image-v1-algebraic_binary.rs',
 'point-image-sources.mjs','finish-point-image-gates.mjs','prepare-point-image-guard.mjs','point-image-guard-origin.json',
 'point-image-v2-algebraic_binary.rs','preserve-point-image-guard-red.mjs','point-image-guard-red-source.rs','point-image-guard-red-source.json',
 'preserve-point-image-guard-controls.mjs','point-image-guard-unstrengthened.rs','point-image-guard-strengthening.json',
 'run-point-image-guard-gates.mjs','freeze-point-image-guard.mjs','point-image-guard-binaries.json','run-point-image-guard-public.mjs',
 'point-image-cost-statistics.mjs','run-point-image-costs.mjs','run-point-image-cost-campaign.mjs','point-image-cost-environment.mjs',
 'check-point-image-costs.mjs','point-image-cost-cpu-summary.json','point-image-cost-allocation-summary.json',
 'capture.mjs','effective-coverage.mjs','power-sums-manifest.json','mpoly-bridge-experiment.json',
 ...['point-image-cost-candidate','point-image-guard-cost'].flatMap(p=>['Cargo.toml','Cargo.lock'].map(f=>p+'/'+f)),
 ...['cpu','allocation'].flatMap(m=>['results/point-image-cost-'+m+'.jsonl','results/point-image-cost-'+m+'-pilots.jsonl']),
 ...['json','stdout','stderr'].map(e=>'results/power-sums-verify.'+e),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
const m={schema:1,checkpoint:53,recorded:new Date().toISOString(),previousManifest:'power-sums-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,coverageBefore:previous.coverageAtBinding,
 coverageAtBinding:effectiveSummary(),newDonorLines:0,liveSourceManifest:'mpoly-bridge-experiment.json',liveFiles:956,
 hyperReadRanges:{'hypersolve/src/algebraic_binary.rs':[[1,373],[390,1089]],'hypersolve/src/algebraic.rs':[[450,500],[2960,3110]],
  'hypersolve/src/root_isolation.rs':[[693,840]],'hypersolve/src/test_support.rs':[[1,23]],
  'hyperlimit/src/real.rs':[[1,118]],'hyperlimit/src/predicates/order.rs':[[1,230]],
  'hyperlimit/src/predicate.rs':[[330,370]],'hyperlimit/src/resolve.rs':[[130,360]]},
 qualification:'Guarded solver 811 all-feature tests per profile and independent public/cost oracles pass. V2 consumer 1764 pass/9 ignored is not guarded consumer qualification. Two code-101 failures are preserved as test-code issues, not mathematical defects. No new WASM, guarded consumer, nonrational/Unknown cost, representative size or full-CI qualification.',
 production:'No production/donor edit or retained transfer. All 956 live sources and five retained continuation transfers unchanged; point-image and power-sum candidates remain isolated.',
 next:'Qualify the guarded source in consumers and representative native/WASM applications, with nonrational/Unknown endpoint/state-history costs before deciding retention. Then resume the separate power-sum transfer.',
 scope:'No new donor coverage. Full original ecosystem inventory and pending transfer work remain incomplete.'};
writeFileSync('point-image-manifest.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:53,boundFiles:files.length,gates:evidence.gates.length,mathematicalStatus:evidence.mathematical.status,
 improved:825,solverTestsPerProfile:811,cpuObservations:3840,allocationObservations:480,coverage:m.coverageAtBinding,production:m.production,next:m.next}));
