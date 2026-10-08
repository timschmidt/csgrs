import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const path='src/fmpz/set.c',inv=json('inventory.json').sources.find(s=>s.repo==='flint');
const f=inv.files.find(f=>f.path===path);assert(f?.text);assert.equal(f.lines,238);
assert.equal(sha(resolve('../../../../exact-real-references/flint',path)),f.sha256);
assert(!effectiveCoverage().some(r=>r.repo==='flint'&&r.path===path));
const records=[{repo:'flint',path,ranges:[[1,238]],
 note:'Checkpoint45 complete integer-import support read: small exact cast window, truncating binary64/scaled-binary64 import, MPF and GMP conversion with inline/heap promotion, signed modular limb and unsigned-array import under their explicit size/sign preconditions. Only bounded finite public scalar paths are numerically qualified here; raw import helpers are read, not probed. Hyper already has explicit exact/lossy/certified boundaries and distinct rounding contracts; no nonredundant transfer selected. fmpz/get.c and ARF conversion/rounding/decomposition/comparison sources were reread but add no coverage.'}];
writeFileSync('arf-conversion-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
console.log(JSON.stringify({newCompleteFiles:1,newLines:238,coverage:effectiveSummary()}));
