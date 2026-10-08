import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const m=json('fexpr-representation-manifest.json'),workspace=resolve('../../../..');
assert.equal(m.checkpoint,48);assert.equal(Object.keys(m.files).length,10);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
const previous=json(m.previousManifest);assert.equal(Object.keys(previous.files).length,30);
for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
assert.equal(Object.keys(previous.liveSources).length,956);
for(const[p,h]of Object.entries(previous.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
for(const[p,h]of Object.entries({...previous.libraries,...previous.configurationFiles}))assert.equal(sha(p),h,p);
for(const b of previous.binaries){assert.equal(sha(b.path),b.sha256,b.path);assert.equal(statSync(b.path).size,b.bytes);}
const terminal=json('results/mpoly-bridge-verify-full.json');assert.equal(terminal.code,0);assert.equal(terminal.signal,null);
assert.equal(readFileSync('results/mpoly-bridge-verify-full.stderr').length,0);
const full=readFileSync('results/mpoly-bridge-verify-full.stdout'),prefix=readFileSync('results/mpoly-rational-verify-full.stdout');
assert.equal(full.length,139322);assert.equal(prefix.length,135764);assert(full.subarray(0,prefix.length).equals(prefix));
assert.deepEqual(m.readRecords,json('fexpr-representation-read-records.json'));assert.equal(m.readRecords.length,88);
const inv=json('inventory.json'),extension=json('coverage-extensions.json'),coverage=effectiveCoverage();
let lines=0,newFiles=0,completed=0;
for(const r of m.readRecords){
 assert(extension.some(c=>JSON.stringify(c)===JSON.stringify(r)));
 const f=inv.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);
 assert.equal(m.donorSources[r.repo+':'+r.path],f.sha256);
 assert.equal(sha(resolve(workspace,'exact-real-references',r.repo,r.path)),f.sha256);
 for(const[a,b]of r.ranges){assert(a>=1&&b>=a&&b<=f.lines);lines+=b-a+1;}
 if(r.repo==='flint'&&r.path==='doc/source/fexpr.rst'){
  assert.deepEqual(r.ranges,[[1,489],[586,620]]);completed++;
 }else{assert.deepEqual(r.ranges,[[1,f.lines]]);newFiles++;}
}
assert.equal(lines,8321);assert.equal(m.newDonorLines,lines);
assert.equal(newFiles,87);assert.equal(m.newFiles,newFiles);
assert.equal(completed,1);assert.equal(m.completedPartialFiles,completed);
let selected=0;
for(const s of inv.sources)for(const f of s.files){
 if(!(f.path==='doc/source/fexpr.rst'||/^(src\/)?fexpr\.h$/.test(f.path)||
  (/^(src\/)?fexpr\//.test(f.path)&&!f.path.endsWith('/write_latex.c'))))continue;
 const c=coverage.find(r=>r.repo===s.repo&&r.path===f.path);assert(c,f.path);
 assert.deepEqual(c.ranges,[[1,f.lines]]);selected++;
}
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 assert(previous.liveSources[p]);const s=readFileSync(resolve(workspace,p),'utf8');
 const n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
// Later append-only checkpoints may increase effective coverage; never rewrite
// this checkpoint's recorded coverage snapshot to make later work appear older.
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);
 assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
assert.equal(m.coverageAtBinding.reduce((n,r)=>n+r.complete,0),1244);
assert.equal(m.coverageAtBinding.reduce((n,r)=>n+r.partial,0),21);
assert.equal(m.coverageAtBinding.reduce((n,r)=>n+r.readLines,0),154117);
console.log(JSON.stringify({checkpoint:48,status:'source-and-recorded-evidence-integrity-pass',
 boundFiles:10,previousBoundFiles:30,liveFiles:956,readRecords:88,newDonorLines:8321,
 selectedSourceFilesComplete:selected,coverage:m.coverageAtBinding,
 qualification:m.qualification,production:m.production,scope:m.scope}));
