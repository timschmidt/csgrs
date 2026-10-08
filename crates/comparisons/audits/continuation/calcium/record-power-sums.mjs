import {writeFileSync,existsSync} from 'node:fs';
import assert from 'node:assert/strict';
import {effectiveSummary} from './effective-coverage.mjs';
import {powerSumsEvidence,sha,json} from './power-sums-evidence.mjs';
assert(!existsSync('power-sums-manifest.json'));
const evidence=powerSumsEvidence(),previous=json('qqbar-arithmetic-manifest.json');
assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['power-sums-findings.md','power-sums-evidence.mjs','record-power-sums.mjs','verify-power-sums.mjs',
 'prepare-power-sums.mjs','power-sums-origin.json','prepare-power-sums-oracle.mjs','power-sums-polynomial-oracle.mjs',
 'check-power-sums-kernel.mjs','check-power-sums-public.mjs','power-sums-public.rs','power-sums-cpu.rs',
 'power-sums-allocation.rs','power-sums-cost-binaries.json','capture.mjs','effective-coverage.mjs',
 'mpoly-bridge-experiment.json','qqbar-arithmetic-manifest.json',
 ...['baseline','candidate'].flatMap(v=>['Cargo.toml','Cargo.lock'].map(p=>'power-sums-cost-'+v+'/'+p)),
 ...['json','stdout','stderr'].map(e=>'results/qqbar-arithmetic-verify.'+e),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
const m={schema:1,checkpoint:52,recorded:new Date().toISOString(),previousManifest:'qqbar-arithmetic-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,liveSourceManifest:'mpoly-bridge-experiment.json',liveFiles:956,
 newDonorLines:0,coverageBefore:previous.coverageAtBinding,coverageAtBinding:effectiveSummary(),
 hyperReadRanges:{'hypersolve/src/root_isolation.rs':[[693,840]],'hypersolve/src/algebraic_binary.rs':[[90,160],[270,373]],
  'hyperreal/src/rational/arithmetic/aggregate_products.rs':[[2020,2088]],
  'hyperreal/src/rational/arithmetic/queries_conversion.rs':[[160,203]],'hypersolve/benches/representations.rs':[[1,160]]},
 qualification:'Exact kernel passes; public gate remains exit 1 with 825 shared point-image completeness failures. All returned roots pass. Default debug 805 tests only; no new matched performance/allocation/representative-size, release/all-feature, WASM or full-CI qualification.',
 production:'No production or donor edit. Five retained continuation transfers unchanged. Separate point-image repair is next, off retained baseline; power sums remain isolated and unselected.',
 scope:'No new donor coverage. Full ecosystem inventory and further transfers remain open.'};
writeFileSync('power-sums-manifest.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:52,boundFiles:files.length,liveFiles:956,candidateFiles:Object.keys(evidence.candidateSources).length,
 kernel:evidence.kernel,publicStatus:evidence.publicCheck.status,publicChecks:43934,publicFailures:825,coverage:m.coverageAtBinding,binaries:evidence.binaries}));
