// Historical checkpoint manifests can bind old partial records exactly.
// Keep those bytes intact and compute unique cumulative reads from extensions.
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
export function effectiveCoverage() {
  const records=json('coverage.json'),extensions=json('coverage-extensions.json');
  const map=new Map(records.map(e=>[`${e.repo}:${e.path}`,structuredClone(e)]));
  assert.equal(map.size,records.length);
  for(const e of extensions) {
    assert(e.note?.length);const key=`${e.repo}:${e.path}`,old=map.get(key);
    if(!old){map.set(key,structuredClone(e));continue;}
    const ranges=[...old.ranges,...e.ranges].sort((a,b)=>a[0]-b[0]),merged=[];
    for(const [a,b]of ranges) {
      assert(a>=1&&b>=a);const previous=merged.at(-1);
      if(previous&&a<=previous[1]+1)previous[1]=Math.max(previous[1],b);else merged.push([a,b]);
    }
    old.ranges=merged;old.note+=' '+e.note;
  }
  return [...map.values()];
}
export function effectiveSummary() {
  const records=effectiveCoverage(),inventory=json('inventory.json');
  return inventory.sources.map(s=>{
    let complete=0,readLines=0;const selected=records.filter(e=>e.repo===s.repo);
    for(const e of selected) {
      const f=s.files.find(f=>f.path===e.path);assert(f?.text);
      assert(e.ranges.every(([a,b],i)=>a>=1&&b>=a&&b<=f.lines&&(!i||a>e.ranges[i-1][1])));
      const n=e.ranges.reduce((n,[a,b])=>n+b-a+1,0);complete+=n===f.lines;readLines+=n;
    }
    return {repo:s.repo,reviewed:selected.length,complete,partial:selected.length-complete,readLines};
  });
}
if(process.argv.includes('--effective-summary'))console.log(JSON.stringify(effectiveSummary()));
