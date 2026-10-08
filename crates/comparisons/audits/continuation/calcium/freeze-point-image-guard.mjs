import {readFileSync,writeFileSync,copyFileSync,constants,statSync} from 'node:fs';
import {checkPointImageSources,sha,json} from './point-image-sources.mjs';
import assert from 'node:assert/strict';
checkPointImageSources();
for(const tag of ['focused','debug','release','clippy','fmt','cost-build'])assert.equal(json('results/point-image-guard-'+tag+'.json').code,0);
const b=json('point-image-binaries.json'),origin=json('point-image-guard-origin.json');
const guardSources=Object.fromEntries(Object.keys(origin.sourceHashes).filter(p=>p.startsWith('hypersolve/')).map(p=>[p,sha('point-image-guard/'+p)]));
const target='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',dir='/tmp/calcium-point-image.4f6mja';
const binaries={};
const stderr=readFileSync('results/point-image-guard-focused.stderr','utf8');
const match=stderr.match(/Running unittests src\/lib.rs \(([^\n]+)\)/);assert(match);
for(const[name,from]of Object.entries({cpu:target+'/release/calcium-point-image-guard-cpu',
 allocation:target+'/release/calcium-point-image-guard-allocation','tests-debug-default':match[1]})){
 const path=dir+'/guard-'+name;copyFileSync(from,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(path),sha(from));binaries[name]={path,sha256:sha(path),bytes:statSync(path).size};
}
const result={checkpoint:53,variant:'guarded',binaries,guardSources,dependencySources:checkPointImageSources(),baselineBinaries:b.baselineBinaries,
 note:'The guarded source closes the approximate-extrema proof boundary and includes eight point-image tests. Old v1/v2 binaries, sources and gates are unchanged. Source guards are independent of current production.'};
writeFileSync('point-image-guard-binaries.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({binaries,guardFiles:Object.keys(guardSources).length,totalDedicatedBytes:Object.values(binaries).reduce((n,b)=>n+b.bytes,0)}));
