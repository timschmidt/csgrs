import './verify-fexpr-representation.mjs';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkFexprFormattingMetadata} from './check-fexpr-formatting-metadata.mjs';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const m=json('fexpr-formatting-manifest.json'),workspace=resolve('../../../..');
assert.equal(m.checkpoint,49);assert.equal(Object.keys(m.files).length,11);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.readRecords,json('fexpr-formatting-read-records.json'));assert.equal(m.readRecords.length,25);
const inv=json('inventory.json'),extension=json('coverage-extensions.json'),coverage=effectiveCoverage();
let lines=0,newFiles=0,completed=0;
for(const r of m.readRecords){
 assert(extension.some(c=>JSON.stringify(c)===JSON.stringify(r)));
 const f=inv.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);
 assert.equal(m.donorSources[r.repo+':'+r.path],f.sha256);
 assert.equal(sha(resolve(workspace,'exact-real-references',r.repo,r.path)),f.sha256);
 for(const[a,b]of r.ranges){assert(a>=1&&b>=a&&b<=f.lines);lines+=b-a+1;}
 if(r.repo==='calcium'&&r.path==='calcium.h'){
  assert.deepEqual(r.ranges,[[1,75],[109,183]]);completed++;
 }else{assert.deepEqual(r.ranges,[[1,f.lines]]);newFiles++;}
}
assert.equal(lines,13667);assert.equal(m.newDonorLines,lines);assert.equal(newFiles,24);assert.equal(m.newFiles,newFiles);
assert.equal(completed,1);assert.equal(m.completedPartialFiles,completed);
const closure={};
for(const s of inv.sources){let files=0,sourceLines=0;
 for(const f of s.files){
  if(!(/^(src\/)?fexpr(?:_builtin)?(?:\.h|\/)/.test(f.path)||
   /^doc\/source\/fexpr(?:_builtin)?\.rst$/.test(f.path)||/^(src\/)?calcium(?:\.h|\/)/.test(f.path)))continue;
  const c=coverage.find(r=>r.repo===s.repo&&r.path===f.path);assert(c,f.path);assert.deepEqual(c.ranges,[[1,f.lines]]);
  files++;sourceLines+=f.lines;
 }
 closure[s.repo]={files,sourceLines};
}
for(const[p,h]of Object.entries(m.auxiliarySources))assert.equal(sha(resolve(workspace,p)),h,p);
const live=json('mpoly-bridge-experiment.json');
for(const[p,ranges]of Object.entries({...m.hyperReadRanges,...m.auxiliaryReadRanges})){
 if(p.startsWith('hyperreal/'))assert(live.liveSources[p]);else assert(m.auxiliarySources[p]);
 const s=readFileSync(resolve(workspace,p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.deepEqual(m.checks,checkFexprFormattingMetadata());
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:479,complete:479,partial:0,readLines:52311},
 {repo:'flint',reviewed:810,complete:790,partial:20,readLines:115473}]);
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);
 assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
console.log(JSON.stringify({checkpoint:49,status:'static-metadata-and-source-evidence-integrity-pass',
 boundFiles:11,liveFiles:956,readRecords:25,newDonorLines:13667,closedSourceSlices:closure,
 coverage:m.coverageAtBinding,checks:m.checks,qualification:m.qualification,production:m.production,scope:m.scope}));
