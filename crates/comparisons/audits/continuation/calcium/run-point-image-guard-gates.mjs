import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
import {checkPointImageSources} from './point-image-sources.mjs';
checkPointImageSources();
const env=['env','CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',
 'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'],gates=[];
for(const profile of ['debug','release'])gates.push([profile,...env,'cargo','test','--offline',
 '--manifest-path','point-image-guard/hypersolve/Cargo.toml',...(profile==='release'?['--release']:[]),'--all-features','--lib','--tests']);
gates.push(['clippy',...env,'cargo','clippy','--offline','--manifest-path','point-image-guard/hypersolve/Cargo.toml',
 '--all-features','--all-targets','--','-D','warnings']);
gates.push(['fmt','rustfmt','--edition','2024','--check','point-image-guard/hypersolve/src/algebraic_binary.rs']);
gates.push(['cost-build',...env,'cargo','build','--offline','--release','--manifest-path','point-image-guard-cost/Cargo.toml','--bins']);
for(const[tag,...args]of gates){
 const c=spawn(process.execPath,['capture.mjs','point-image-guard-'+tag,'.',...args],{stdio:'inherit'});
 const r=await new Promise((ok,fail)=>{c.on('error',fail);c.on('close',(code,signal)=>ok({code,signal}));});
 assert.equal(r.code,0,tag);assert.equal(r.signal,null,tag);
}
console.log(JSON.stringify({status:'pass',gates:gates.map(g=>'point-image-guard-'+g[0])}));
