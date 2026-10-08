import {accessSync,constants,existsSync,readdirSync,readFileSync,statSync,statfsSync} from 'node:fs';
import {resolve} from 'node:path';
const candidates=(process.env.PATH??'').split(':');
const executable=name=>candidates.map(p=>resolve(p,name)).find(p=>{try{accessSync(p,constants.X_OK);return statSync(p).isFile();}catch{return false;}})??null;
const tools=Object.fromEntries(['coqc','rocq','opam','ghc','runghc','stack','cabal','cargo','rustc','valgrind','node'].map(n=>[n,executable(n)]));
const inventory=JSON.parse(readFileSync('inventory-v85.json'));
const symlinks=inventory.files.filter(f=>f.kind==='symlink').map(f=>({path:f.path,target:f.target,
 resolved:resolve(inventory.root,f.path,'..',f.target),exists:existsSync(resolve(inventory.root,f.path,'..',f.target))}));
const cache='/home/tim/.stack/programs/x86_64-linux',tmp=statfsSync('/tmp');
const binaries=['debug','release'].map(profile=>{const path='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/'+profile+'/coq-aern-hyper-v85';return{path,bytes:statSync(path).size};});
console.log(JSON.stringify({checked:new Date().toISOString(),node:process.version,platform:process.platform,arch:process.arch,tools,
 stackCompilerCache:{path:cache,entries:existsSync(cache)?readdirSync(cache):null},symlinks,binaries,
 auditBinaryBytes:binaries.reduce((s,x)=>s+x.bytes,0),tmpAvailableBytes:tmp.bavail*tmp.bsize,
 qualification:'PATH/cache inspection only; no native Coq/GHC compilation or toolchain installation, no performance or product-size comparison.'}));
