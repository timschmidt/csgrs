import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointHistoryEvidence} from './point-history-evidence.mjs';
assert(!existsSync('point-history-manifest.json'));
const evidence=await pointHistoryEvidence(),previous=json('point-extended-manifest.json');
assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['point-history-findings.md','point-history-evidence.mjs','record-point-history.mjs','verify-point-history.mjs',
 'prepare-point-history.mjs','build-point-history.mjs','point-history-base.rs','point-history-work.rs','point-history-cpu.rs','point-history-allocation.rs',
 'point-history-origin.json','point-history-binaries.json','point-history-protocol.mjs','qualify-point-history.mjs','point-history-qualification.json',
 'point-history-statistics.mjs','run-point-history-costs.mjs','run-point-history-campaign.mjs','check-point-history.mjs',
 'point-history-cost-cpu-summary.json','point-history-cost-allocation-summary.json','point-image-cost-environment.mjs',
 'point-extended-manifest.json','point-qualified-origin.json','point-qualified-sources.mjs','point-qualified-capture.mjs','capture.mjs',
 'point-extended.rs','point-extended-field.mjs','check-point-extended.mjs','power-sums-allocation.rs',
 ...['baseline','candidate'].flatMap(v=>['Cargo.toml','Cargo.lock'].map(f=>'point-history-'+v+'/'+f)),
 ...['json','stdout','stderr'].map(e=>'results/point-extended-verify.'+e),
 ...['qualification','qualification-failures',...['cpu','allocation'].flatMap(m=>['cost-'+m,'cost-'+m+'-pilots','cost-'+m+'-failures'])].map(t=>'results/point-history-'+t+'.jsonl'),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(new Set(files).size,files.length);
const result={schema:1,checkpoint:56,recorded:new Date().toISOString(),previousManifest:'point-extended-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,coverageBefore:previous.coverageAtBinding,
 coverageAtBinding:effectiveSummary(),newDonorLines:0,liveFiles:956,
 production:'No production/donor edit or retained transfer; five retained continuation transfers and all 956 live hashes remain unchanged.',
 scope:'Native cost qualification, not completion. No new donor coverage; all original references, supporting kernels, unresolved transfers and full inventory reconciliation remain open.'};
writeFileSync('point-history-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:56,boundFiles:files.length,gates:evidence.gates.length,status:evidence.costs.status,
 qualificationChecks:84864,cpuObservations:36864,cpuPilots:1536,allocationObservations:4608,dedicatedBytes:evidence.dedicatedBytes,
 production:result.production,next:result.next}));
