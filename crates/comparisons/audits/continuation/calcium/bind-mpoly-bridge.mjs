import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkMpolyBridge} from './check-mpoly-bridge.mjs';
const tags=['compile','native','linked-libraries','memcheck','output-check'].map(s=>'mpoly-bridge-'+s);
const files=['flint-mpoly-bridge-controls.c','prepare-mpoly-bridge-helpers.mjs','mpoly-bridge-helpers.h',
 'mpoly-bridge-math.mjs','check-mpoly-bridge.mjs','record-mpoly-bridge-reads.mjs','mpoly-bridge-read-records.json',
 'mpoly-bridge-findings.md','bind-mpoly-bridge.mjs','verify-mpoly-bridge.mjs','mpoly-rational-experiment.json',
 'capture.mjs',...['json','stdout','stderr'].map(ext=>'results/mpoly-rational-verify-full.'+ext)];
const gates=tags.map(tag=>{const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 for(const ext of['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
 return{tag,code:g.code,signal:g.signal,cwd:g.cwd,command:g.command,args:g.args};});
const previous=json('mpoly-rational-experiment.json');
for(const[p,h]of Object.entries(previous.liveSources))assert.equal(sha(resolve('../../../..',p)),h,p);
for(const[p,h]of Object.entries({...previous.libraries,...previous.configurationFiles}))assert.equal(sha(p),h,p);
const linked=readFileSync('results/mpoly-bridge-linked-libraries.stdout','utf8');
assert.deepEqual(Object.fromEntries([...linked.matchAll(/=> (\/[^ ]+) \(/g)].map(x=>[x[1],sha(x[1])])),previous.libraries);
const path='/tmp/calcium-mpoly-bridge.7pQl1d/controls',binaries=[{path,bytes:statSync(path).size,sha256:sha(path)}];
const checks=checkMpolyBridge();assert.deepEqual(checks,json('results/mpoly-bridge-output-check.stdout'));
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates,
 liveSources:previous.liveSources,libraries:previous.libraries,configurationFiles:previous.configurationFiles,binaries,
 readRecords:json('mpoly-bridge-read-records.json'),newDonorLines:5029,newCompleteFiles:54,newPartialFiles:2,
 coverageAtBinding:effectiveSummary(),hyperReadRanges:{'hyperreal/src/rational/arithmetic/ops.rs':[[1019,1078]],
  'hypersolve/src/algebraic_fiber.rs':[[1138,1157],[3060,3118],[3260,3346]]},checks,
 status:'Archived45-file rational-function slice source-complete including the prior manual read. Both rational-expression bridges, current generic adapter/test and selected supporting fexpr code read. Full ecosystem and recursive support remain incomplete.',
 production:'No production/donor edit or new retained transfer. All956 live hashes unchanged; five prior continuation improvements preserved. Existing native libraries reused; one28488-byte /tmp executable, no whole-native/Rust rebuild or cleanup/deletion/commit/push/external report.',
 findings:'Archived staged denominator/content GCD cancellation evolves into current scalar fast paths. Formal expression normalization treats terminal expressions as independent and does not preserve authored domain evidence. Generic adapter distinguishes domain from inability outcomes; disabled factor/root methods are not implementations. Independent finite full-polynomial and canonicality certificates pass for7308 complete values including2142 generic aliases.',
 comparison:'Hyperreal already stages denominator GCD and restricts residual cancellation. Hypersolve already represents unit/shared denominators, bounded residues and explicit selected-root denominator obligations. No unchecked formal normalization transfer or new matched performance claim justified.',
 limits:checks.limits+' Source-only archived tests and current generic test are not newly executed. Total source credit5029 lines/56 records;54 new complete and two partial. Paired numerical logs1728768 workspace bytes. Original failures preserved.',
 followup:'Read remaining recursive polynomial/GCD, generic/default conversion, fexpr/algebraic paths; continue every original uncompleted reference and unresolved transfer; reconcile entire requested inventory before completion.'};
assert.equal(files.length,30);assert.equal(m.readRecords.length,56);assert.equal(binaries[0].bytes,28488);
writeFileSync('mpoly-bridge-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({files:30,gates:5,liveFiles:956,readRecords:56,newDonorLines:5029,coverage:m.coverageAtBinding,binaryBytes:28488,checks}));
