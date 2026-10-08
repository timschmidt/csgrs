import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {retainedStatisticsEvidence} from './retained-statistics-evidence-v61.mjs';
import {campaigns,historicalManifests,reviewedScripts} from './reanalyse-retained-statistics-v61.mjs';
assert(!existsSync('retained-statistics-v61-manifest.json'));
const evidence=retainedStatisticsEvidence(),previous=json('point-statistics-v60-manifest.json');
assert.deepEqual(effectiveSummary(),previous.coverageAtBinding);
const files=[...new Set(['retained-statistics-v61-findings.md','retained-statistics-v61-analysis.json',
 'reanalyse-retained-statistics-v61.mjs','check-retained-statistics-v61.mjs','retained-statistics-evidence-v61.mjs',
 'record-retained-statistics-v61.mjs','verify-retained-statistics-v61.mjs','paired-statistics-v60.mjs',
 'point-statistics-v60-manifest.json','point-demand-sources.mjs','e-plan-qualified-sources.mjs','derivative-demand-sources.mjs',
 'capture.mjs',...historicalManifests,...reviewedScripts,...campaigns.flatMap(c=>[c.summary,c.raw]),
 ...['json','stdout','stderr'].map(e=>'results/point-statistics-verify.'+e),
 ...evidence.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])];
const result={schema:1,checkpoint:61,recorded:new Date().toISOString(),previousManifest:'point-statistics-v60-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...evidence,coverageBefore:previous.coverageAtBinding,
 coverageAtBinding:effectiveSummary(),newDonorLines:0,liveFiles:956,
 production:'No live production/donor edit, new retained transfer, benchmark, binary, source-tree copy, temporary file, cleanup/deletion, commit or push. Five retained changes unchanged; intended-path planner/derivative benefits survive corrected individual intervals with known costs disclosed.',
 scope:'Five planner/derivative timing campaigns corrected and historical numerical/allocation/test outputs rechecked offline. No fresh Rust regression, benchmark, memory, consumer or size execution. Other retained/historical statistics and the complete original ecosystem audit remain incomplete.'};
writeFileSync('retained-statistics-v61-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:61,status:'recorded',boundFiles:files.length,gates:evidence.gates.length,
 rawRows:evidence.rawRows,comparisons:evidence.comparisons,totals:evidence.totals,newTemporaryBytes:0,next:evidence.next}));
