import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {finalZeroSources,sha,json} from './zero-factor-final-sources-v70.mjs';
import {captured} from './point-qualified-capture.mjs';
const source=finalZeroSources(),g=json('results/zero-factor-final-gates-v70.json');assert.equal(g.code,0);assert.equal(g.signal,null);
const binaries=json('zero-factor-binaries-v70.json');
for(const [name,args]of [['public',['check']],['extended',['check']],['wide',[resolve('zero-factor-degree-input-v70.json')]]]){
 const a=binaries.binaries.find(b=>b.name===name);assert.equal(sha(a.path),a.sha256);
 await captured('zero-factor-memory-'+name+'-v70','.','valgrind',['--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',a.path,...args]);
}
await captured('zero-factor-environment-v70','.','node',['point-qualified-environment.mjs']);
assert.deepEqual(finalZeroSources(),source);
console.log(JSON.stringify({checkpoint:70,status:'extra-gates-terminal',limits:'Native Memcheck only; process totals are not matched allocation/performance measurements, peak/RSS or bounded-cache proofs.'}));
