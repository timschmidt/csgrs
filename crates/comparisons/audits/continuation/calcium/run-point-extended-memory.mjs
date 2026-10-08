import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
import {captured} from './point-qualified-capture.mjs';
const o=sources(),b=json('point-extended-binaries.json');
assert.equal(json('results/point-extended-mathematical.json').code,0);
for(const f of b.artifacts){
 assert.equal(sha(f.path),f.sha256);
 await captured('point-extended-memory-'+f.variant,'.','valgrind',[
  '--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',f.path,'check']);
 assert.equal(sha('results/point-extended-memory-'+f.variant+'.stdout'),sha('results/point-extended-public-'+f.variant+'.stdout'));
}
assert.deepEqual(sources(),o);console.log(JSON.stringify({status:'memory-execution-completed',outputs:'identical to native per variant',
 limits:'Read full Memcheck summaries before claiming memory properties. Not allocation benchmarking, peak-RSS qualification or general boundedness.'}));
