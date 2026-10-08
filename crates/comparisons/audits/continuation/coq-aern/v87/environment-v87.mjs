import {spawnSync} from 'node:child_process';
import {existsSync,statfsSync,readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const commands={};
for(const cmd of ['ghc','runghc','coqc','rocq','opam','stack','cabal','cargo','node','valgrind','bash']){
 const r=spawnSync('bash',['--noprofile','--norc','-c','command -v "$1"','tool-check',cmd],{encoding:'utf8'});
 assert(!r.error,r.error?.message);assert.equal(r.signal,null);assert([0,1].includes(r.status));assert.equal(r.stderr,'');
 if(r.status===0)assert(r.stdout.trim().length>0,'empty successful tool lookup');
 else assert.equal(r.stdout,'');
 commands[cmd]=r.status===0?r.stdout.trim():null;
}
const paths=['/tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc','/home/tim/.ghcup/bin/ghc','/home/tim/.stack/programs/x86_64-linux'];
const historic='/home/tim/Documents/GitHub/workspace/exact-real-references/constructible-qualification/boundary-README.md';
const s=statfsSync('/tmp');
console.log(JSON.stringify({status:'checked-known-native-tool-locations',commands,paths:paths.map(path=>({path,exists:existsSync(path)})),historicalBuildNote:{path:historic,sha256:createHash('sha256').update(readFileSync(historic)).digest('hex')},tmpAvailableBytes:s.bavail*s.bsize,installedOrDownloaded:false}));
