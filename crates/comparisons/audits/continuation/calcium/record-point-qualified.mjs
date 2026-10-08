import {writeFileSync,existsSync} from 'node:fs';
import assert from 'node:assert/strict';
import {effectiveSummary} from './effective-coverage.mjs';
import {sha,json} from './point-qualified-sources.mjs';
import {pointQualifiedEvidence} from './point-qualified-evidence.mjs';
assert(!existsSync('point-qualified-manifest.json'));
const evidence=pointQualifiedEvidence(),previous=json('point-image-manifest.json');
assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['point-qualified-findings.md','point-qualified-evidence.mjs','record-point-qualified.mjs','verify-point-qualified.mjs',
 'prepare-point-qualified.mjs','point-qualified-origin.json','point-qualified-sources.mjs','point-qualified-capture.mjs',
 'measure-point-qualified-apps.mjs','point-qualified-apps-origin.json','point-qualified-apps-summary.json',
 'point-qualified-environment.mjs','prepare-point-qualified-approx.mjs','capture.mjs','effective-coverage.mjs',
 'point-image-manifest.json','power-sums-public.rs','check-point-image.mjs','e-qualified-app-size-summary.json',
 ...['platform','approx'].flatMap(mode=>['point-qualified-'+mode+'.rs','build-point-qualified-'+mode+'.mjs',
  'run-point-qualified-'+mode+'.mjs','point-qualified-'+mode+'-origin.json','point-qualified-'+mode+'-binaries.json',
  ...['baseline','candidate'].flatMap(v=>['Cargo.toml','Cargo.lock'].map(f=>'point-qualified-'+mode+'-'+v+'/'+f))]),
 'run-point-qualified-wasm.mjs','run-point-qualified-approx-wasm.mjs','check-point-qualified-public.mjs','check-point-qualified-approx-public.mjs',
 ...['json','stdout','stderr'].map(e=>'results/point-image-verify.'+e),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(new Set(files).size,files.length);
const manifest={schema:1,checkpoint:54,recorded:new Date().toISOString(),previousManifest:'point-image-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,
 coverageBefore:previous.coverageAtBinding,coverageAtBinding:effectiveSummary(),newDonorLines:0,liveFiles:956,
 qualification:'Final guarded consumer release 1764 pass/9 ignored; Clippy and selected-feature WASM consumer build pass. Full rational public corpus executes on native and WASM under STRICT and APPROXIMATE_512, preserving 825 gains and 5616 unchanged records each. Prior unstrengthened debug-consumer evidence is not relabeled. No new CPU/allocator/Memcheck, guarded full-debug-consumer or full-CI claim.',
 production:'No production/donor edit or retained transfer; all 956 live sources and five retained continuation transfers unchanged.',
 next:'Nonrational/Unknown endpoint and state-history costs, relevant WASM measurements, then an explicit guarded-repair retention decision. Separate power-sum optimization remains unselected/untimed.',
 scope:'No donor coverage added. Whole original reference inventory, supporting kernels, pending transfer experiments and reconciliation remain incomplete.'};
writeFileSync('point-qualified-manifest.json',JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:54,boundFiles:files.length,gates:evidence.gates.length,consumer:evidence.consumer,
 strict:evidence.mathematical.status,approximate:evidence.approximateMathematical.status,dedicatedBytes:evidence.dedicatedBytes,
 production:manifest.production,next:manifest.next}));
