// Preserve the initial failed binding and every original bound file unchanged.
import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
const initial=json('arf-rounding-experiment.json');
for(const[p,h]of Object.entries(initial.files))assert.equal(sha(p),h,p);
assert.deepEqual(initial.hyperReadRanges['hyperlattice/src/kernels.rs'],[[110,255]]);
const failed=json('results/arf-rounding-verify-full.json');assert.equal(failed.code,1);assert.equal(failed.signal,null);
assert.match(readFileSync('results/arf-rounding-verify-full.stderr','utf8'),/AssertionError \[ERR_ASSERTION\]: hyperlattice\/src\/kernels.rs/);
const m=structuredClone(initial);m.recorded=new Date().toISOString();
m.hyperReadRanges['hyperlattice/src/kernels.rs']=[[110,250]];
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 const path=resolve('../../../..',p),content=readFileSync(path,'utf8'),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert.equal(sha(path),m.liveSources[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
for(const p of ['arf-rounding-experiment.json','bind-arf-rounding-v2.mjs','verify-arf-rounding-v2.mjs',
 'results/arf-rounding-verify-full.json','results/arf-rounding-verify-full.stdout','results/arf-rounding-verify-full.stderr'])m.files[p]=sha(p);
m.gates.push({tag:'arf-rounding-verify-full',code:1,signal:null});
m.correction={initialManifest:'arf-rounding-experiment.json',failedVerification:'arf-rounding-verify-full',
 path:'hyperlattice/src/kernels.rs',before:[[110,255]],after:[[110,250]],
 reason:'The read command requested through line 255 but this file has only 250 lines. Correct the Hyper read-range metadata to the actual returned lines. Preserve the original manifest/verifier and failed verification unchanged. No donor coverage, source, binary or numerical result changes.'};
writeFileSync('arf-rounding-experiment-v2.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'ARF rounding corrected binding',files:Object.keys(m.files).length,gates:m.gates.length,
 successfulGates:7,preservedMetadataFailure:1,correction:m.correction,summary:m.checks.summary}));
