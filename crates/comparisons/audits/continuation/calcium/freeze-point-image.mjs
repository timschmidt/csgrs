import {readFileSync,writeFileSync,copyFileSync,statSync,constants} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const before=JSON.parse(readFileSync('power-sums-manifest.json'));
for(const[p,h]of Object.entries(before.candidateSources))assert.equal(sha('power-sums-candidate/'+p),h,p);
const target='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',dir='/tmp/calcium-point-image.4f6mja';
const sources={cpu:target+'/release/calcium-point-image-candidate-cpu',
 allocation:target+'/release/calcium-point-image-candidate-allocation',
 'tests-debug-default':target+'/debug/deps/hypersolve-9930d053377a5876'};
const binaries={};
for(const[name,from]of Object.entries(sources)){
 const path=dir+'/'+name;copyFileSync(from,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 assert.equal(sha(from),sha(path));binaries[name]={path,bytes:statSync(path).size,sha256:sha(path)};
}
const origin=JSON.parse(readFileSync('point-image-origin.json'));
const sourceHashes=Object.fromEntries(Object.keys(origin.liveSources).map(p=>[p,sha(origin.candidate+'/'+p)]));
assert.deepEqual(Object.keys(sourceHashes).filter(p=>sourceHashes[p]!==origin.liveSources[p]),['hypersolve/src/algebraic_binary.rs']);
const result={schema:1,checkpoint:53,binaries,sourceHashes,
 baselineBinaries:JSON.parse(readFileSync('power-sums-cost-binaries.json')),
 note:'Frozen candidate drivers and default-feature focused-test executable. Existing baseline executables reused; power-sum source unchanged.'};
writeFileSync('point-image-binaries.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({binaries,totalDedicatedBytes:Object.values(binaries).reduce((n,b)=>n+b.bytes,0),sourceFiles:956}));
