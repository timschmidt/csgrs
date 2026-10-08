import {execFileSync} from 'node:child_process';
import {readFileSync,statfsSync} from 'node:fs';
import os from 'node:os';
const read=p=>{try{return readFileSync(p,'utf8').trim();}catch(e){return {unavailable:e.code};}};
const cpu='/sys/devices/system/cpu/cpu2/';
console.log(JSON.stringify({checkpoint:79,recorded:new Date().toISOString(),node:process.version,v8:process.versions.v8,kernel:os.release(),
 affinity:read('/proc/self/status').match(/^Cpus_allowed_list:\s*(.+)$/m)?.[1],cpu:2,
 cpuModel:execFileSync('lscpu',['-J'],{encoding:'utf8'}),sibling:read(cpu+'topology/thread_siblings_list'),
 governor:read(cpu+'cpufreq/scaling_governor'),frequency:read(cpu+'cpufreq/scaling_cur_freq'),load:read('/proc/loadavg'),cpuAccounting:read('/proc/stat'),
 processes:execFileSync('ps',['-eo','pid,etimes,pcpu,stat,comm'],{encoding:'utf8'}),
 versions:[['rustc',['-vV']],['cargo',['-vV']]].map(([command,args])=>({command,args,stdout:execFileSync(command,args,{encoding:'utf8'})})),
 volumes:['/tmp',process.cwd()].map(path=>{const s=statfsSync(path);return{path,totalBytes:s.blocks*s.bsize,availableBytes:s.bavail*s.bsize};}),
 limits:'Host snapshots, not continuous contention observation or a reserved/fixed-frequency CPU. No arguments/environment secrets are captured in the process list.'}));

