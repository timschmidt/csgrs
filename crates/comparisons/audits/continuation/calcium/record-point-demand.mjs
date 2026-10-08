import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointDemandEvidence} from './point-demand-evidence.mjs';
assert(!existsSync('point-demand-manifest.json'));
const evidence=await pointDemandEvidence(),previous=json('point-history-manifest.json');assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['point-demand-findings.md','point-demand-evidence.mjs','record-point-demand.mjs','verify-point-demand.mjs',
 'prepare-point-demand.mjs','point-demand-origin.json','point-demand-sources.mjs','record-point-demand-source.mjs','point-demand-source-binding.json',
 'point-demand-build-origin.json','build-point-demand.mjs','point-demand-binaries.json','point-demand-bindings.mjs','point-demand-extended.rs',
 'run-point-demand-qualification.mjs','check-point-demand-public.mjs','qualify-point-demand-history.mjs','point-demand-history.json',
 'point-demand-cost-protocol.mjs','run-point-demand-costs.mjs','run-point-demand-campaign.mjs','check-point-demand-costs.mjs',
 'point-demand-cost-cpu-summary.json','point-demand-cost-allocation-summary.json','point-demand-app/Cargo.toml','point-demand-app/Cargo.lock',
 'point-history-base.rs','point-history-work.rs','point-history-cpu.rs','point-history-allocation.rs','point-history-protocol.mjs',
 'point-history-binaries.json','point-history-manifest.json','point-image-cost-environment.mjs','point-extended-field.mjs','check-point-extended.mjs',
 'check-point-image.mjs','check-power-sums-public.mjs','power-sums-polynomial-oracle.mjs','power-sums-public.rs',
 'point-qualified-platform.rs','point-qualified-approx.rs','point-qualified-origin.json','point-qualified-sources.mjs','point-qualified-capture.mjs','capture.mjs',
 ...['json','stdout','stderr'].map(e=>'results/point-history-verify.'+e),
 ...['history','history-failures',...['cpu','allocation'].flatMap(m=>['cost-'+m,'cost-'+m+'-pilots','cost-'+m+'-failures'])].map(t=>'results/point-demand-'+t+'.jsonl'),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(new Set(files).size,files.length);
const result={schema:1,checkpoint:57,recorded:new Date().toISOString(),previousManifest:'point-history-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,coverageBefore:previous.coverageAtBinding,coverageAtBinding:effectiveSummary(),
 newDonorLines:0,liveFiles:956,
 production:'Isolated solver prototype only. No live production/donor edit, new retained transfer, cleanup/deletion, commit, push or external report. All five retained transfers and 956 live identities unchanged.',
 scope:'Not whole-audit completion. Native qualification does not close extended WASM, final consumer/size gates, separate power-sum work, remaining reference/support reads or inventory reconciliation.'};
writeFileSync('point-demand-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:57,status:evidence.costs.status,boundFiles:files.length,gates:evidence.gates.length,sourceFiles:175,
 solverTests:811,historyChecks:44992,cpuObservations:55296,cpuPilots:2304,allocationObservations:6912,dedicatedBytes:evidence.dedicatedBytes,
 lineDeltas:evidence.lineDeltas,production:result.production,next:result.next}));
