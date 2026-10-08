// Version historical source reads, not mathematical checks or recorded outcomes.
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';

export const digest=s=>createHash('sha256').update(s).digest('hex');
const read=p=>readFileSync(p,'utf8');
const fixes={};
for(const name of ['high-product','arf-rounding-v2','arf-fused','arb-dot',
 'magnitude','mag-transcendental','mag-series']) {
 fixes['verify-'+name+'.mjs']=[
  ['const content=read(resolve(workspace,p))',
   "const content=read(resolve('derivative-demand-candidate',p))",1],
  ['assert.equal(sha(resolve(workspace,p)),m.liveSources[p])',
   "assert.equal(sha(resolve('derivative-demand-candidate',p)),m.liveSources[p])",1]
 ];
}
for(const name of ['complex-product-sources','complex-product-v2-sources'])
 fixes[name+'.mjs']=[["resolve(here,'../../../..',p)",
  "resolve(here,'derivative-demand-candidate',p)",1]];
fixes['verify-complex-product.mjs']=[["resolve(here,'../../../..',p)",
 "resolve(here,'derivative-demand-candidate',p)",2]];
fixes['verify-complex-product-v2.mjs']=[["resolve(here,'../../../..',r.path)",
 "resolve(here,'derivative-demand-candidate',r.path)",2]];

// All modules stay beside their originals: relative artifacts and captured cwd
// assertions therefore retain their original meaning. Only affected imports fork.
export function plan() {
 const graph=new Map(),entry='verify-e-qualified.mjs';
 function visit(path) {
  if(graph.has(path))return;
  const source=read(path),imports=[...source.matchAll(/^(?:import|export)[^\n]*?['"](\.\/[^'"]+\.mjs)['"]/gm)]
   .map(m=>m[1].slice(2));
  graph.set(path,{source,imports});for(const p of imports)visit(p);
 }
 visit(entry);for(const p of Object.keys(fixes))assert(graph.has(p),p);
 const selected=new Set(Object.keys(fixes));
 let changed;
 do {changed=false;for(const[p,g]of graph)if(!selected.has(p)&&g.imports.some(x=>selected.has(x))) {
  selected.add(p);changed=true;
 }}while(changed);
 const modules=[];
 for(const original of [...selected].sort()) {
  const g=graph.get(original),edits=[];let source=g.source;
  for(const[before,after,count]of fixes[original]??[]) {
   assert.equal(source.split(before).length-1,count,original+': '+before);
   source=source.replaceAll(before,after);edits.push({kind:'historical-source',before,after,count});
  }
  for(const dependency of g.imports.filter(p=>selected.has(p))) {
   const before="'./"+dependency+"'",after="'./snapshot-v43-"+dependency+"'";
   assert.equal(source.split(before).length-1,1,original+': '+before);
   source=source.replace(before,after);edits.push({kind:'import',before,after,count:1});
  }
  assert.notEqual(source,g.source,original);
  modules.push({original,originalSha256:digest(g.source),path:'snapshot-v43-'+original,
   sha256:digest(source),bytes:Buffer.byteLength(source),edits,source});
 }
 return{entry:'snapshot-v43-'+entry,modules};
}
