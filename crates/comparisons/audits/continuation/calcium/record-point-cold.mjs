import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointColdEvidence} from './point-cold-evidence.mjs';
import {variants,platforms,metadataPath} from './point-cold-protocol.mjs';
assert(!existsSync('point-cold-manifest.json'));
const evidence=await pointColdEvidence(),previous=json('point-demand-manifest.json');assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['point-cold-findings.md','point-cold-evidence.mjs','record-point-cold.mjs','verify-point-cold.mjs',
 'point-cold-common.rs','point-cold-native.rs','point-cold-platform.rs','point-cold-clippy-fix.patch',
 'prepare-point-cold.mjs','build-point-cold.mjs','resume-point-cold-build.mjs','point-cold-origin.json','point-cold-binaries.json',
 'point-cold-protocol.mjs','run-point-cold-native.mjs','run-point-cold-wasm.mjs','run-point-cold-campaign.mjs','check-point-cold.mjs',
 'point-demand-manifest.json','point-demand-source-binding.json','point-demand-bindings.mjs','point-demand-sources.mjs',
 'point-history-base.rs','point-extended-field.mjs','check-point-extended.mjs','point-qualified-capture.mjs','capture.mjs',
 ...variants.flatMap(v=>['point-cold-'+v+'/Cargo.toml','point-cold-'+v+'/Cargo.lock']),
 ...platforms.flatMap(p=>variants.map(v=>metadataPath(p,v))),
 ...['json','stdout','stderr'].map(e=>'results/point-demand-verify.'+e),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(new Set(files).size,files.length);
const result={schema:1,checkpoint:58,recorded:new Date().toISOString(),previousManifest:'point-demand-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,
 coverageBefore:previous.coverageAtBinding,coverageAtBinding:effectiveSummary(),newDonorLines:0,liveFiles:956,
 production:'No live production/donor edits, algorithm revisions, new retained transfers, source copies, cleanup/deletion, commits, pushes or external reports. Existing isolated demand candidate and all five retained transfers unchanged.',
 scope:'Cold-state semantic qualification only. No new CPU/allocation/Memcheck/consumer/representative-size result; full ecosystem audit and separate power-sum work incomplete.'};
writeFileSync('point-cold-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:58,status:evidence.semantic.status,boundFiles:files.length,gates:evidence.gates.length,
 successfulGates:evidence.successfulGates,preservedFailures:evidence.preservedFailures,queries:evidence.semantic.totalQueries,
 independentChecks:evidence.semantic.totalChecks,dedicatedBytes:evidence.dedicatedBytes,rawBytes:evidence.rawBytes,next:evidence.next}));
