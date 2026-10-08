import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {statisticsEvidence} from './point-statistics-evidence-v60.mjs';
import {campaigns} from './reanalyse-point-statistics-v60.mjs';
assert(!existsSync('point-statistics-v60-manifest.json'));
const evidence=statisticsEvidence(),previous=json('point-wasm-manifest.json');assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=['point-statistics-v60-findings.md','paired-statistics-v60.mjs','test-paired-statistics-v60.mjs',
 'reanalyse-point-statistics-v60.mjs','check-point-statistics-v60.mjs','point-statistics-v60-analysis.json',
 'point-statistics-evidence-v60.mjs','record-point-statistics-v60.mjs','verify-point-statistics-v60.mjs',
 'point-wasm-manifest.json','point-demand-source-binding.json','point-demand-sources.mjs','point-wasm-protocol.mjs',
 'point-qualified-capture.mjs','capture.mjs',
 ...evidence.inventory.files.map(f=>f.path),...campaigns.flatMap(c=>[c.summary,c.raw]),
 ...['json','stdout','stderr'].map(e=>'results/point-wasm-verify.'+e),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(new Set(files).size,files.length);
const result={schema:1,checkpoint:60,recorded:new Date().toISOString(),previousManifest:'point-wasm-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,coverageBefore:previous.coverageAtBinding,coverageAtBinding:effectiveSummary(),
 newDonorLines:0,liveFiles:956,
 production:'No live production/donor edit, algorithm revision, new benchmark or binary, source-tree copy, cleanup/deletion, commit, push or external report. Raw observations and all five retained transfers unchanged; their historical inference claims require the disclosed statistical impact review.',
 scope:'Four point-family statistical campaigns corrected, not all historical confidence intervals or the original ecosystem audit. No new numerical regression/allocation/consumer/representative-size/retention qualification. The full objective remains incomplete.'};
writeFileSync('point-statistics-v60-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:60,status:'statistical-correction-recorded',boundFiles:files.length,gates:evidence.gates.length,
 rawRows:evidence.rawRows,comparisons:evidence.comparisons,inventoryMatches:evidence.inventory.count,newTemporaryBytes:0,next:evidence.next}));
