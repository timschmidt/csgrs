import {execFileSync} from 'node:child_process';
import {statfsSync} from 'node:fs';
import os from 'node:os';
const commands=[['rustc',['-vV']],['cargo',['-vV']],['strip',['--version']],['size',['--version']]];
const volumes=['/tmp','/home/tim/Documents/GitHub/workspace'].map(path=>{
 const s=statfsSync(path);return{path,totalBytes:s.blocks*s.bsize,availableBytes:s.bavail*s.bsize};
});
console.log(JSON.stringify({recorded:new Date().toISOString(),node:process.version,v8:process.versions.v8,
 platform:process.platform,arch:process.arch,kernel:os.release(),
 versions:commands.map(([command,args])=>({command,args,stdout:execFileSync(command,args,{encoding:'utf8'})})),volumes,
 limits:'Build/runtime identity and a post-build filesystem-capacity snapshot. No timing, allocator, peak-memory, fixed-frequency or idle-machine claim.'}));
