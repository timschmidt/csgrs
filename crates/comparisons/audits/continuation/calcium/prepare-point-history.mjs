import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
sources();const b=json('point-extended-binaries.json');assert.equal(sha('point-extended.rs'),b.harnessSha256);
const read=p=>readFileSync(p,'utf8'),original=read('point-extended.rs');
assert.equal(original.split('fn run(').length,2);
const prefix=original.slice(0,original.indexOf('fn run('));assert.equal(prefix.split('let mut visit =').length,2);
const base=prefix.replace('let mut visit =','let visit =').replace('use std::time::Instant;\n','');
const allocation=read('power-sums-allocation.rs').split('include!("power-sums-public.rs");')[0];
const files=[['point-history-base.rs',base],['point-history-allocation.rs',allocation+'\ninclude!("point-history-base.rs");\ninclude!("point-history-work.rs");\nfn main() { benchmark_main(Some(allocation::snapshot)); }\n']];
for(const variant of ['baseline','candidate']){
 const app='point-history-'+variant;
 let manifest=read('point-extended-'+variant+'/Cargo.toml').replaceAll('point-extended','point-history');
 manifest+='\n[[bin]]\nname = "'+app+'-allocation"\npath = "../point-history-allocation.rs"\n';
 files.push([app+'/Cargo.toml',manifest]);
}
console.log('*** Begin Patch\n'+files.map(([p,s])=>'*** Add File: '+resolve(p)+'\n'+s.trimEnd().split('\n').map(l=>'+'+l).join('\n')).join('\n')+'\n*** End Patch');
