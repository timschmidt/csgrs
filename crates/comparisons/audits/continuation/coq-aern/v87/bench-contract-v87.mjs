import {readFileSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import assert from 'node:assert/strict';
const root='/home/tim/Documents/GitHub/workspace/exact-real-references/coq-aern/extracted-examples/bench/';
const hs=readFileSync(root+'bench.hs','utf8'),sh=readFileSync(root+'runBench.sh','utf8');
const active=[...hs.matchAll(/^  bench "([^"]+)"/gm)].map(x=>x[1]);
const commented=[...hs.matchAll(/^  -- bench "([^"]+)"/gm)].map(x=>x[1]);
const enabled=sh.split('\n').filter(x=>/^csqrt\d+ExtractedOnly$/.test(x));
const selected=enabled.map(name=>{
 const start=sh.indexOf('function '+name+'\n'),end=sh.indexOf('\n}',start);assert(start>=0&&end>start);
 const body=sh.slice(start,end),bench=body.match(/bench="([^"]+)"/)[1],params=body.match(/step="([^"]+)"/)[1].split(' ');
 assert(body.includes('method_E_bparamss="$steps"'));assert(body.includes('{1..10}'));
 return{name,dispatcher:bench+'E',params,repetitions:10,hasActiveDispatcher:active.includes(bench+'E'),hasCommentedDispatcher:commented.includes(bench+'E')};
});
assert.deepEqual(enabled,['csqrt3ExtractedOnly','csqrt5ExtractedOnly']);assert.equal(active.length,17);
assert(selected.every(x=>!x.hasActiveDispatcher&&x.hasCommentedDispatcher));
assert(hs.includes('let eps = (0.5 :: t)^(n :: Integer)'));assert(hs.includes('if (split x eps eps)'));
assert(sh.includes('if [ $? != 0 ]; then rm $runlog; exit 1; fi'));
assert(sh.includes('${utime/0.00/0.01}'));assert(sh.includes('overwritelogs="true"'));
// Only syntax-check the donor runner. Do not execute its overwrite/remove paths.
const r=spawnSync('bash',['-n',root+'runBench.sh'],{encoding:'utf8'});
assert(!r.error,r.error?.message);assert.equal(r.status,0);assert.equal(r.signal,null);assert.equal(r.stdout,'');assert.equal(r.stderr,'');
console.log(JSON.stringify({status:'verified-static-benchmark-contract',activeDispatchers:active,commentedComplexDispatchers:commented,selected,
 plannedInvocationsIfAllSucceeded:selected.reduce((n,x)=>n+x.params.length*x.repetitions,0),
 syntaxCheck:'passed',runnerExecuted:false,nativeHaskellExecuted:false,failuresDeletedByRunner:true,overwritesRepeatedParameters:true,zeroCpuTimesReplaced:true}));
