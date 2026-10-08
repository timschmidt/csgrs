import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointWasmEvidence} from './point-wasm-evidence.mjs';
import {variants} from './point-wasm-protocol.mjs';
assert(!existsSync('point-wasm-manifest.json'));
const evidence=await pointWasmEvidence(),previous=json('point-cold-manifest.json');assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['point-wasm-findings.md','point-wasm-evidence.mjs','record-point-wasm.mjs','verify-point-wasm.mjs',
 'prepare-point-wasm.mjs','point-wasm-work.rs','point-wasm-platform.rs','build-point-wasm.mjs','point-wasm-origin.json','point-wasm-binaries.json',
 'point-wasm-protocol.mjs','qualify-point-wasm.mjs','check-point-wasm-qualification.mjs','check-point-wasm-costs.mjs',
 'run-point-wasm-costs.mjs','run-point-wasm-campaign.mjs','point-wasm-qualification.json','point-wasm-cost-summary.json',
 'point-cold-manifest.json','point-demand-source-binding.json','point-demand-sources.mjs','point-cold-protocol.mjs',
 'point-history-base.rs','point-history-work.rs','point-history-protocol.mjs','point-demand-cost-protocol.mjs',
 'point-extended-field.mjs','check-point-extended.mjs','point-qualified-capture.mjs','capture.mjs','point-image-cost-environment.mjs',
 ...variants.flatMap(v=>['point-wasm-'+v+'/Cargo.toml','point-wasm-'+v+'/Cargo.lock']),
 'results/point-history-qualification.jsonl','results/point-demand-history.jsonl',
 ...['qualification','qualification-failures','cost','cost-pilots','cost-failures'].map(p=>'results/point-wasm-'+p+'.jsonl'),
 ...['json','stdout','stderr'].map(e=>'results/point-cold-verify.'+e),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(new Set(files).size,files.length);
const result={schema:1,checkpoint:59,recorded:new Date().toISOString(),previousManifest:'point-cold-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,coverageBefore:previous.coverageAtBinding,coverageAtBinding:effectiveSummary(),
 newDonorLines:0,liveFiles:956,
 production:'Unchanged isolated demand candidate; no live production/donor edit, new algorithm revision, retained transfer, source snapshot copy, cleanup/deletion, commit, push or external report. Five retained transfers and all 956 live identities unchanged.',
 scope:'Optimized-tier Node/V8 WASM qualification and matched costs, not a default-tier/browser/whole-application claim. No new allocator/Memcheck/final-consumer/representative-size qualification. Full ecosystem audit and separate power-sum work remain incomplete.'};
writeFileSync('point-wasm-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:59,status:evidence.costs.status,boundFiles:files.length,gates:evidence.gates.length,
 sourceFiles:175,liveFiles:956,qualificationObservations:4608,independentChecks:129856,cpuObservations:55296,cpuPilots:2304,
 dedicatedBytes:evidence.dedicatedBytes,rawBytes:evidence.rawBytes,next:evidence.next}));
