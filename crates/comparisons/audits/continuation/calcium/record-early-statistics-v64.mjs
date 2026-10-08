import {existsSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {evidence} from './early-statistics-evidence-v64.mjs';
import {campaigns,paths,manifests,reviewedScripts,supportingScripts} from './reanalyse-early-statistics-v64.mjs';
assert(!existsSync('early-statistics-v64-manifest.json'));
const data=evidence(),prior=json('prototype-statistics-v63-manifest.json');assert.deepEqual(effectiveSummary(),prior.coverageAtBinding);
const files=[...new Set(['early-statistics-v64-findings.md','early-statistics-v64-analysis.json',
 'reanalyse-early-statistics-v64.mjs','check-early-statistics-v64.mjs','early-statistics-evidence-v64.mjs',
 'record-early-statistics-v64.mjs','verify-early-statistics-v64.mjs','inspect-early-binaries-v64.mjs','early-statistics-v64-binary-inventory.json',
 'paired-statistics-v60.mjs','prototype-statistics-v63-manifest.json','reanalyse-prototype-statistics-v63.mjs','point-demand-sources.mjs','capture.mjs',
 'baseline-hyperreal.json','qualification-summary.json','polynomial-decision-baseline-sources.json',
 ...manifests,...reviewedScripts,...supportingScripts,
 ...campaigns.flatMap(c=>['cpu',...(c.noAlloc?[]:['alloc'])].flatMap(mode=>Object.values(paths(c,mode)))),
 ...['json','stdout','stderr'].map(e=>'results/prototype-statistics-verify.'+e),
 ...data.gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])];
const result={schema:1,checkpoint:64,recorded:new Date().toISOString(),previousManifest:'prototype-statistics-v63-manifest.json',
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),...data,coverageBefore:prior.coverageAtBinding,coverageAtBinding:effectiveSummary(),newDonorLines:0,liveFiles:956,
 production:'No production/donor change, new retained transfer, source-tree copy, build, benchmark, binary, /tmp file, cleanup/deletion, commit or push. Early unselected versions remain isolated; later retained cache/fact-aware replacements remain unchanged.',
 scope:'Nine early historical CPU datasets corrected with explicit source-limited and rejected strata, and six allocation datasets rechecked without timing inference. Fourteen-checkpoint archival evidence chain rerun; no fresh numerical/Rust/backend/Memcheck/consumer/size execution or full 47-chain. Surviving 45 known sampler scripts have addressed dataset scopes, not exhaustive semantic statistical or ecosystem closure.'};
writeFileSync('early-statistics-v64-manifest.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:64,status:'recorded',boundFiles:files.length,gates:data.gates.length,rawRows:data.rawRows,comparisons:data.comparisons,
 allocationRows:data.allocationRows,withdrawnAllocationIntervals:263,addressedKnownMatches:data.addressedKnownMatches.length,newTemporaryBytes:0,next:data.next}));
