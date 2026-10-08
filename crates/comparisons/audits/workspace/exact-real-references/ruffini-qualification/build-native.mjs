import {readFileSync, writeFileSync, readdirSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {spawnSync} from 'node:child_process';
const root=import.meta.dirname, workspace=resolve(root,'../..');
const build=resolve(workspace,'.audit-ruffini-build.LmZgYM');
const pins={
 'guava-32.0.0-jre.jar':'39f3550b0343d8d19dd4e83bd165b58ea3389d2ddb9f2148e63903f79ecdb114',
 'commons-math3-3.6.1.jar':'1e56d7b058d28b65abd256b8458e3885b674c1d588fa43cd7d1cbb9c7ef2b308',
 'junit-4.13.1.jar':'c30719db974d6452793fe191b3638a5777005485bae145924044530ffa5f6122',
 'hamcrest-core-1.3.jar':'66fdef91e9739348df7a096aa384a5685f4e875584cce89386a7a47251c4d8e9',
};
const digest=path=>createHash('sha256').update(readFileSync(path)).digest('hex');
for(const [name,hash] of Object.entries(pins)) if(digest(build+'/deps/'+name)!==hash) throw Error('dependency changed: '+name);
const files=readFileSync(resolve(root,'../RUFFINI_FILE_INVENTORY.tsv'),'utf8').trimEnd().split('\n').slice(1).map(row=>row.split('\t')[0]).filter(p=>/^(common|reals)\/src\/main\/java\/.*\.java$/.test(p));
const sources=files.map(p=>resolve(root,'../Ruffini',p));
sources.push(resolve(root,'RuffiniBoundary.java'));
const args=['--release','16','-encoding','UTF-8','-cp',build+'/deps/*','-d',build+'/classes',...sources];
const r=spawnSync('javac',args,{encoding:'utf8',timeout:60000,maxBuffer:8*1024*1024});
writeFileSync(root+'/native-build.log',(r.stdout??'')+(r.stderr??'')+'\n'+JSON.stringify({status:r.status,signal:r.signal,error:r.error?.message})+'\n');
if(r.error||r.status!==0) throw Error('javac failed; see native-build.log');
const classes=readdirSync(build+'/classes',{recursive:true}).filter(p=>p.endsWith('.class')).sort().map(p=>[p,digest(build+'/classes/'+p)]);
writeFileSync(root+'/native-build-manifest.json',JSON.stringify({compiler:spawnSync('javac',['-version'],{encoding:'utf8'}).stdout.trim(),runtime:spawnSync('java',['-version'],{encoding:'utf8'}).stderr.trim(),release:16,donorFiles:files.length,sources:sources.map(p=>[p,digest(p)]),dependencies:pins,classes},null,2)+'\n');
console.log(JSON.stringify({status:'compiled',donorFiles:files.length,classFiles:classes.length,build}));
