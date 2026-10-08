import {readFileSync} from 'node:fs';
import {cpus,release,platform,arch,totalmem,freemem,loadavg} from 'node:os';
const read=p=>{try{return readFileSync(p,'utf8').trim();}catch{return null;}};
console.log(JSON.stringify({recorded:new Date().toISOString(),platform:platform(),release:release(),arch:arch(),
 cpuCount:cpus().length,cpu6:cpus()[6],totalMemory:totalmem(),freeMemory:freemem(),loadAverage:loadavg(),
 node:process.versions,governor:read('/sys/devices/system/cpu/cpu6/cpufreq/scaling_governor'),
 siblingCPUs:read('/sys/devices/system/cpu/cpu6/topology/thread_siblings_list'),
 note:'Host snapshot only; not a guarantee of no external load or fixed frequency. No environment secrets collected.'}));
