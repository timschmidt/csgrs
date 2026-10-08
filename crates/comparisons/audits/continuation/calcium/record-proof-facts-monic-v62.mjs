import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {evidence} from './proof-facts-monic-evidence-v62.mjs';
import {campaigns,paths,manifests,reviewedScripts} from './reanalyse-proof-facts-monic-v62.mjs';
assert(!existsSync('proof-facts-monic-v62-manifest.json'));
const data=evidence(),prior=json('retained-statistics-v61-manifest.json');assert.deepEqual(effectiveSummary(),prior.coverageAtBinding);
const files=[...new Set(['proof-facts-monic-v62-findings.md','proof-facts-monic-v62-analysis.json',
 'reanalyse-proof-facts-monic-v62.mjs','check-proof-facts-monic-v62.mjs','proof-facts-monic-evidence-v62.mjs',
 'record-proof-facts-monic-v62.mjs','verify-proof-facts-monic-v62.mjs','paired-statistics-v60.mjs',
 'retained-statistics-v61-manifest.json','reanalyse-retained-statistics-v61.mjs','point-demand-sources.mjs','capture.mjs',
 'run-polynomial-facts-allocation-bounded.mjs',...manifests,...reviewedScripts,
 ...campaigns.flatMap(c=>['cpu',...(c.rejected?[]:['alloc'])].flatMap(mode=>Object.values(paths(c,mode)))),
 ...['json','stdout','stderr'].map(e=>'results/retained-statistics-verify.'+e),
 ...data.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])];
const result={schema:1,checkpoint:62,recorded:new Date().toISOString(),previousManifest:'retained-statistics-v61-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...data,coverageBefore:prior.coverageAtBinding,coverageAtBinding:effectiveSummary(),newDonorLines:0,liveFiles:956,
 production:'No production/donor change, new retained transfer, source copy, benchmark, binary, temporary file, cleanup/deletion, commit or push. Existing completeness-first retentions unchanged with corrected scoped performance evidence and costs preserved.',
 scope:'Seven accepted CPU campaigns corrected plus one separately rejected historical run, and allocation metrics rechecked without promoting their instrumented clocks. Historical 18-checkpoint verifier rerun, not new Rust/numerical/memory/consumer/size executions or the full 47-chain. Remaining historical statistics and original ecosystem audit are incomplete.'};
writeFileSync('proof-facts-monic-v62-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:62,status:'recorded',boundFiles:files.length,gates:data.gates.length,eligible:data.eligible,
 rejectedHistoricalOnly:data.rejectedHistoricalOnly,allocationRows:data.allocationRows,newTemporaryBytes:0,next:data.next}));
