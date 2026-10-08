import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const inv=json('inventory.json').sources.find(s=>s.repo==='flint'),prior=effectiveCoverage();
const records=[{repo:'flint',path:'src/ulong_extras.h',ranges:[[70,126]],
 note:'Checkpoint44 support read. Lines74..80 prove random upper limits1/4 use a bitmask and exclude the purported alias/nearest default branches. Also read adjacent exact-word divisibility/preinverse division contracts; no new numerical campaign or transfer for these helpers.'},
 {repo:'flint',path:'src/ulong_extras/randomisation.c',ranges:[[35,43]],
 note:'Checkpoint44 support read. Public n_randint/n_urandint forward to the header helper. Establishes the source-level unreachable branches without executing random exceptional/large-exponent tests.'}];
for(const r of records){assert(!prior.some(x=>x.repo===r.repo&&x.path===r.path));const f=inv.files.find(f=>f.path===r.path);
 assert(f?.text);assert.equal(sha(resolve('../../../../exact-real-references/flint',r.path)),f.sha256);}
writeFileSync('arf-contract-support-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
console.log(JSON.stringify({newPartialFiles:2,newLines:66,coverage:effectiveSummary()}));
