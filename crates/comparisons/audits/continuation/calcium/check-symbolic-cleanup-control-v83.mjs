import {readFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {resolve}from 'node:path';
import {fileURLToPath}from 'node:url';
import {json,sha,workspace}from './zero-factor-retained-sources-v75.mjs';
export function cleanupEvidence(){
 const o=json('symbolic-cleanup-control-origin-v83.json');
 for(const[p,h]of Object.entries(o.files))assert.equal(sha(p),h,p);
 const donor=workspace+'/exact-real-references/flint/src/qqbar/set_fexpr.c',added='                        fmpz_clear(p);\n                        fmpz_clear(q);\n';
 assert.equal(sha(donor),o.donorSha256);
 assert.equal(readFileSync('flint-set-fexpr-cleanup-control-v83.c','utf8').replace(added,''),readFileSync(donor,'utf8'));
 const controls=[];
 for(const which of [0,1,2]){
  const native='symbolic-cleanup-control-'+which+'-v83',mem='symbolic-cleanup-control-mem-'+which+'-v83',original='symbolic-power-'+which+'-256-v83',
   raw=readFileSync('results/'+native+'.stdout');
  assert(raw.equals(readFileSync('results/'+mem+'.stdout')));assert(raw.equals(readFileSync('results/'+original+'.stdout')));
  for(const tag of [native,mem]){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);}
  const log=readFileSync('results/'+mem+'.stderr','utf8');
  assert(log.includes('in use at exit: 0 bytes in 0 blocks'));assert(log.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
  const total=log.match(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/);assert(total);
  const n=x=>Number(x.replaceAll(',',''));assert.equal(n(total[1]),n(total[2]));
  controls.push({which,records:257,identicalOutput:true,errors:0,liveBytes:0,allocations:n(total[1]),frees:n(total[2]),requestedBytes:n(total[3])});
 }
 return{checkpoint:83,status:'missing-pow-temporary-clears-causally-confirmed',controls,
  limits:'Two-line audit-only renamed translation unit, unchanged donor/library and Hyper. Shared fmpz pool loss is not a 69 KiB per-call allocation; only 16 retained payload bytes per rejected call scale in this corpus.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(cleanupEvidence()));
