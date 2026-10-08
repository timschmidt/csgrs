import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const records=[{repo:'flint',path:'src/test_helpers.h',ranges:[[145,210]],
 note:'Checkpoint46 partial driver read: TEST_MAIN with no test arguments iterates every registered test and aborts on a failing return. Also read adjacent failure formatting and named-test argument selection; no invalid argument branch exercised.'},
 {repo:'flint',path:'src/generic_files/test_helpers.c',ranges:[[1,37]],
 note:'Checkpoint46 complete multiplier helper: initialize once from FLINT_TEST_MULTIPLIER, default one, fallback on values outside the finite accepted comparison range. Upstream run explicitly supplies literal1, so the declared loop counts are not silently reduced. No environment parsing edge probes.'}];
const inv=json('inventory.json').sources.find(s=>s.repo==='flint'),prior=effectiveCoverage();
for(const r of records){assert(!prior.some(c=>c.repo===r.repo&&c.path===r.path));const f=inv.files.find(f=>f.path===r.path);
 assert(f?.text);assert.equal(sha(resolve('../../../../exact-real-references/flint',r.path)),f.sha256);
 for(const[a,b]of r.ranges)assert(a>=1&&b>=a&&b<=f.lines);}
writeFileSync('mpoly-rational-support-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
console.log(JSON.stringify({newCompleteFiles:1,newPartialFiles:1,newLines:103,coverage:effectiveSummary()}));
