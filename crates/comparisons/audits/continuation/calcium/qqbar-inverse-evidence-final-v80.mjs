import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {json,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {inverseEvidence,evidenceFiles as originalFiles} from './qqbar-inverse-evidence-v80.mjs';
export const evidenceFiles=[...originalFiles,'qqbar-inverse-evidence-final-v80.mjs'];
export function inverseFinalEvidence(){
 const original=inverseEvidence(),tag='qqbar-inverse-evidence-v80',g=json('results/'+tag+'.json');
 assert.equal(g.code,0);assert.equal(g.signal,null);assert.equal(g.cwd,resolve('.'));assert.equal(g.command,'node');
 assert.deepEqual(g.args,['qqbar-inverse-evidence-v80.mjs']);assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
 assert.deepEqual(original,json('results/'+tag+'.stdout'));
 const current=retainedSources(),live=json('zero-factor-retained-origin-v75.json').after,prior=json('twelfth-revision-v79-manifest.json');
 assert.deepEqual(live,prior.liveSources);assert.equal(Object.keys(live).length,957);assert.equal(current.liveFiles,957);
 assert.equal(original.liveFiles,Object.keys(current).length);assert.equal(original.liveFiles,14);
 return{...original,liveFiles:957,developmentGates:[tag],
  bookkeepingCorrection:'The original summary counted the 14 fields of retainedSources() instead of its verified 957-file live map. Source hashes, all mathematics and every raw observation are unchanged; preserve that original script and capture.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(inverseFinalEvidence()));
