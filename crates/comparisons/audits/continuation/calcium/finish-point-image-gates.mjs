import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
import {checkPointImageSources} from './point-image-sources.mjs';
checkPointImageSources();
const env=['env','CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',
 'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
const gates=[];
for(const profile of ['debug','release'])gates.push(['candidate-'+profile+'-v2',...env,'cargo','test','--offline',
 '--manifest-path','point-image-candidate/hypersolve/Cargo.toml',...(profile==='release'?['--release']:[]),'--all-features','--lib','--tests']);
gates.push(['candidate-clippy-v2',...env,'cargo','clippy','--offline','--manifest-path','point-image-candidate/hypersolve/Cargo.toml',
 '--all-features','--all-targets','--','-D','warnings']);
gates.push(['candidate-fmt-v2','rustfmt','--edition','2024','--check','point-image-candidate/hypersolve/src/algebraic_binary.rs']);
gates.push(['consumer-debug',...env,'cargo','test','--offline','--manifest-path','point-image-candidate/hypercurve/Cargo.toml','--all-features','--lib','--tests']);
for(const[tag,...args]of gates){
 const c=spawn(process.execPath,['capture.mjs','point-image-'+tag,'.',...args],{stdio:'inherit'});
 const r=await new Promise((ok,fail)=>{c.on('error',fail);c.on('close',(code,signal)=>ok({code,signal}));});
 assert.equal(r.code,0,tag);assert.equal(r.signal,null,tag);
}
console.log(JSON.stringify({status:'pass',gates:gates.map(g=>'point-image-'+g[0])}));
