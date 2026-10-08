import {existsSync,statSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {campaigns} from './reanalyse-early-statistics-v64.mjs';
const inspected=new Date().toISOString(),rows=[];
for(const c of campaigns){
 const summary=c.stem+'-cpu-summary.json';
 for(const[variant,b]of Object.entries(json(summary).binaries)){
  assert(existsSync(b.path));const observedSha256=sha(b.path),observedBytes=statSync(b.path).size;
  rows.push({campaign:c.id,variant,summary,summarySha256:sha(summary),path:b.path,
   recordedSha256:b.sha256,recordedBytes:b.bytes,observedSha256,observedBytes,matches:observedSha256===b.sha256});
 }
}
assert.equal(rows.length,18);assert(rows.every(r=>r.matches===(r.campaign==='polynomial')));
const result={inspected,rows,matching:2,nonmatching:16,
 scope:'Point-in-time read-only check at the recorded paths, not a search for all alternate copies. Sixteen reusable CPU binary paths no longer contain their recorded executable; the two separately frozen polynomial binaries match. Later writes to reusable build paths do not retroactively change this observation. No replacement binary is benchmarked or passed off as the historical CPU executable.'};
writeFileSync('early-statistics-v64-binary-inventory.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify(result));
