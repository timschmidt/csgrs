import {readFileSync,statSync,writeSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {demandBindings} from './point-demand-bindings.mjs';
import {sha,json} from './point-demand-sources.mjs';
export const variants=['baseline','eager','demand'];
export const platforms=['native','wasm'];
export const lifecycles=['retained','fresh','roundtrip'];
export const config={cases:48,policies:2,histories:4,lifecycles,calls:9,
 groups:1152,queriesPerVariantPlatform:10368,wasmGcEvery:8};
export const bytesSha=b=>createHash('sha256').update(b).digest('hex');
export function* groups(){
 for(let c=0;c<48;c++)for(let p=0;p<2;p++)for(let h=0;h<4;h++)for(const lifecycle of lifecycles)
  yield {case:c,policy:p,history:h,lifecycle};
}
export const groupKey=g=>[g.case,g.policy,g.history,g.lifecycle].join(':');
export const argsFor=g=>[g.case,g.policy,g.history,lifecycles.indexOf(g.lifecycle)];
export function writeAll(fd,bytes){let n=0;while(n<bytes.length)n+=writeSync(fd,bytes,n,bytes.length-n);}
export function parseGroup(bytes,g){
 const text=bytes.toString('utf8');assert(text.endsWith('\n'));const rows=text.slice(0,-1).split('\n').map(JSON.parse);
 assert.equal(rows.length,10);
 for(let call=0;call<9;call++){
  const r=rows[call];assert.equal(r.type,'query');assert.equal(r.call,call);
  for(const k of ['case','policy','history','lifecycle'])assert.equal(r[k],g[k]);
  assert(r.left&&r.right&&r.report);
 }
 assert.deepEqual(rows[9],{type:'terminal',...g,calls:9});return rows.slice(0,9);
}
export function coldBindings(){
 demandBindings();const origin=json('point-cold-origin.json'),b=json('point-cold-binaries.json');
 assert.equal(origin.predecessorSha256,sha('point-demand-manifest.json'));
 assert.equal(origin.sourceBindingSha256,sha('point-demand-source-binding.json'));
 assert.equal(b.originSha256,sha('point-cold-origin.json'));assert.equal(b.root,origin.root);
 assert.equal(b.artifacts.length,6);
 for(const[p,h]of Object.entries(b.harnessSources))assert.equal(sha(p),h,p);
 for(const variant of variants)for(const platform of platforms){
  const matches=b.artifacts.filter(a=>a.variant===variant&&a.platform===platform);assert.equal(matches.length,1);
  const a=matches[0];assert.equal(statSync(a.path).size,a.bytes);assert.equal(sha(a.path),a.sha256);
 }
 return b;
}
export const metadataPath=(platform,variant)=>'results/point-cold-'+platform+'-'+variant+'-groups.jsonl';
export const outputPath=(platform,variant)=>'results/point-cold-'+platform+'-'+variant+'.stdout';
export const jsonLines=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
