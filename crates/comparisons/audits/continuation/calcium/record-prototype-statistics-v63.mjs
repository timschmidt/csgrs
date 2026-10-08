import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {evidence} from './prototype-statistics-evidence-v63.mjs';
import {campaigns,paths,manifests,reviewedScripts,supportingScripts} from './reanalyse-prototype-statistics-v63.mjs';
assert(!existsSync('prototype-statistics-v63-manifest.json'));
const data=evidence(),prior=json('proof-facts-monic-v62-manifest.json');assert.deepEqual(effectiveSummary(),prior.coverageAtBinding);
const files=[...new Set(['prototype-statistics-v63-findings.md','prototype-statistics-v63-analysis.json',
 'reanalyse-prototype-statistics-v63.mjs','check-prototype-statistics-v63.mjs','prototype-statistics-evidence-v63.mjs',
 'record-prototype-statistics-v63.mjs','verify-prototype-statistics-v63.mjs','paired-statistics-v60.mjs',
 'prototype-statistics-v63-inventory.mjs','prototype-statistics-v63-inventory.json',
 'prototype-statistics-inventory-v63.mjs','prototype-statistics-inventory-failed-v63.mjs','prototype-statistics-v63-inventory-corrected.json',
 'proof-facts-monic-v62-manifest.json','reanalyse-proof-facts-monic-v62.mjs','point-demand-sources.mjs','capture.mjs',
 'matrix-solve-experiment.json',...manifests,...reviewedScripts,...supportingScripts,
 ...campaigns.flatMap(c=>['cpu','allocation'].flatMap(mode=>Object.values(paths(c,mode)))),
 ...['json','stdout','stderr'].map(e=>'results/proof-facts-monic-verify.'+e),
 ...data.captures.flatMap(({tag})=>['json','stdout','stderr'].map(e=>'results/'+tag+'.'+e))])];
const result={schema:1,checkpoint:63,recorded:new Date().toISOString(),previousManifest:'proof-facts-monic-v62-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...data,coverageBefore:prior.coverageAtBinding,coverageAtBinding:effectiveSummary(),newDonorLines:0,liveFiles:956,
 production:'No production/donor change, new retained transfer, source-tree copy, benchmark, binary, temporary build/file, cleanup/deletion, commit or push. Five previously rejected prototypes remain isolated; five existing retained continuation changes remain unchanged.',
 scope:'Five historical CPU campaigns corrected and allocation metrics rechecked. Historical 23-checkpoint rank chain plus four complex/sign checkers rerun; only the small exact BigInt sign oracle is newly executed. No fresh Rust/backend/Memcheck/consumer/size runs or full 47-chain. Initial incomplete/failed/output-missing inventory/oracle captures remain preserved. Historical estimator review and the original ecosystem audit are incomplete.'};
writeFileSync('prototype-statistics-v63-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:63,status:'recorded',boundFiles:files.length,captures:data.captures.length,qualifiedGates:data.qualifiedGates.length,
 rawRows:data.rawRows,comparisons:data.comparisons,allocationRows:data.allocationRows,aggregate:data.aggregate,newTemporaryBytes:0,next:data.next}));
