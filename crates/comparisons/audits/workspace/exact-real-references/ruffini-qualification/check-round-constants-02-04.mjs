import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,prefix='demos/src/main/java/demo/poseidon/poseidon_constants/';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const modulus=21888242871839275222246405745257275088548364400416034343698204186575808495617n;
const inventory=new Map(readFileSync(root+'/../RUFFINI_FILE_INVENTORY.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>{const r=s.split('\t');return[r[0],r];}));
const valid=s=>/^(0|[1-9][0-9]*)$/.test(s)&&BigInt(s)>=0n&&BigInt(s)<modulus;
for(const s of ['','00','01','-1','+1',' 1','1 ','1.0','1e0','0x1',modulus.toString(),(modulus+1n).toString()])assert.equal(valid(s),false);
for(const s of ['0','1',(modulus-1n).toString()])assert.equal(valid(s),true);
const files=[];
for(const[index,partialRounds]of [[2,56],[3,60],[4,60]]){
 const name='constants'+String(index).padStart(2,'0'),path=root+'/../Ruffini/'+prefix+name;
 const text=readFileSync(path,'utf8'),lines=text.split('\n');if(text.endsWith('\n'))lines.pop();
 const row=inventory.get(prefix+name);assert.equal(row[4],'READ');assert.equal(hash(path),row[3]);
 assert.equal(lines.length,+row[2]);assert.equal(lines.length,(index+2)*(8+partialRounds));
 assert(lines.every(valid));
 files.push({name,sha256:hash(path),physicalLines:lines.length,coordinates:index+2,fullRounds:8,partialRounds,finalNewline:text.endsWith('\n'),canonicalDecimalAndRange:true});
}
const result={sourceSha256:hash(import.meta.filename),files,totalEntries:files.reduce((s,f)=>s+f.physicalLines,0),predicateControls:15,scope:'only constants02, constants03 and constants04 explicitly read; dimension/format/range checks, not generating-procedure or cryptographic qualification and not unread-data credit'};
writeFileSync(root+'/round-constants-02-04-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
