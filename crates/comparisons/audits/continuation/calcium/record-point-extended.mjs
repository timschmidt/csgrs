import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointExtendedEvidence} from './point-extended-evidence.mjs';
assert(!existsSync('point-extended-manifest.json'));
const evidence=pointExtendedEvidence(),previous=json('point-qualified-manifest.json');
assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['point-extended-findings.md','point-extended-evidence.mjs','record-point-extended.mjs','verify-point-extended.mjs',
 'point-extended.rs','point-extended-cpu.rs','build-point-extended.mjs','point-extended-origin.json','point-extended-binaries.json',
 'run-point-extended.mjs','run-point-extended-memory.mjs','probe-point-extended-field.mjs',
 'point-extended-field-v1.mjs','point-extended-field.mjs','check-point-extended-v1.mjs','check-point-extended.mjs',
 'point-qualified-manifest.json','point-qualified-origin.json','point-qualified-sources.mjs','point-qualified-capture.mjs','capture.mjs',
 ...['baseline','candidate'].flatMap(v=>['Cargo.toml','Cargo.lock'].map(f=>'point-extended-'+v+'/'+f)),
 ...['json','stdout','stderr'].map(e=>'results/point-qualified-verify.'+e),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(new Set(files).size,files.length);
const result={schema:1,checkpoint:55,recorded:new Date().toISOString(),previousManifest:'point-qualified-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,coverageBefore:previous.coverageAtBinding,
 coverageAtBinding:effectiveSummary(),newDonorLines:0,liveFiles:956,
 qualification:'Extended serialized exact-value oracle passes 21216 checks: 384 queries per variant, two policies/four histories, 128 gains and 256 unchanged queries. Focused native/Memcheck outputs agree. Initial decoder coverage failure and compiler warning are preserved. No matched CPU/allocator, new full-suite or WASM qualification.',
 production:'No production/donor edit or retained transfer; five retained continuation transfers and all 956 live hashes remain unchanged.',
 next:'Finalize/qualify full-result and state-aware CPU/allocation harness, run matched endpoint/history costs and relevant WASM qualification, then decide guarded-repair retention. Separate power-sum performance work follows.',
 scope:'No donor coverage added; all original references, supporting kernels, unresolved transfers and full inventory reconciliation remain incomplete.'};
writeFileSync('point-extended-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:55,boundFiles:files.length,gates:evidence.gates.length,mathematical:evidence.mathematical.status,
 checks:21216,improved:128,unchangedQueries:256,dedicatedBytes:evidence.dedicatedBytes,production:result.production,next:result.next}));
