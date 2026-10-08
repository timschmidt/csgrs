import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname, repo=root+'/../Ruffini/';
const hash=b=>createHash('sha256').update(b).digest('hex');
const inputs=process.argv.slice(2);assert(inputs.length>0 && inputs.length<=16);
assert(inputs.every(s=>/^(0|[1-9]|1[0-5])$/.test(s)));const indices=inputs.map(Number);
assert.deepEqual(indices,[...new Set(indices)].sort((a,b)=>a-b));
const rows=new Map(readFileSync(root+'/../RUFFINI_FILE_INVENTORY.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>{
    const row=s.split('\t');return[row[0],row];
}));
function pinned(path) {
    const bytes=readFileSync(repo+path),row=rows.get(path);assert(row);
    assert.equal(row[4],'READ','validation never creates read credit');
    assert.equal(hash(bytes),row[3]);assert.equal(bytes.length,+row[1]);
    const text=bytes.toString('utf8');assert(Buffer.from(text).equals(bytes));
    const lines=text.split('\n');if(text.endsWith('\n'))lines.pop();assert.equal(lines.length,+row[2]);
    return{bytes,text,lines,sha256:row[3]};
}
const modulusText='21888242871839275222246405745257275088548364400416034343698204186575808495617';
const modulus=BigInt(modulusText);
function numeric(s){const m=/^(0|[1-9][0-9]*)$/.exec(s);return m!==null && m[0]===s && BigInt(s)<modulus;}
function lexical(s){
    if(!s.length || (s.length>1 && s[0]==='0'))return false;
    for(const c of s)if(c<'0'||c>'9')return false;
    return s.length<modulusText.length || (s.length===modulusText.length && s<modulusText);
}
const controls=[['0',true],['1',true],[(modulus-1n).toString(),true],[modulusText,false],
    [(modulus+1n).toString(),false],['',false],['-1',false],['+1',false],['01',false],['00',false],
    ['1 ',false],[' 1',false],['1\n',false],['1\r',false],['1.0',false],['1e1',false],['0x10',false],['١',false]];
for(const[s,want]of controls){assert.equal(numeric(s),want,JSON.stringify(s));assert.equal(lexical(s),want,JSON.stringify(s));}
const loader=pinned('demos/src/main/java/demo/poseidon/Constants.java');
assert.equal(loader.sha256,'e8d4dcbf37d53c451b0244dfe136ab5c0e0b7632148a2d71c511c80f23b829c8');
assert(/getFullRounds\(int t\)\s*\{\s*return 8;/.test(loader.text));
const partial=/getPartialRounds\(int t\)\s*\{\s*return List\.of\(([^)]+)\)\.get\(t\);/.exec(loader.text);assert(partial);
const rounds=partial[1].split(',').map(s=>Number(s.trim()));
assert.deepEqual(rounds,[56,57,56,60,60,63,64,63,60,66,60,65,70,60,64,68]);
const files=[];
for(const index of indices){
    const name='constants'+String(index).padStart(2,'0');
    const {bytes,text,lines,sha256}=pinned('demos/src/main/java/demo/poseidon/poseidon_constants/'+name);
    assert.equal(lines.length,(index+2)*(8+rounds[index]));
    for(const s of lines){assert(numeric(s));assert(lexical(s));}
    files.push({name,sha256,bytes:bytes.length,physicalLines:lines.length,coordinates:index+2,
        fullRounds:8,partialRounds:rounds[index],finalNewline:text.endsWith('\n'),canonicalDecimalAndRange:true});
}
const result={scriptSHA256:hash(readFileSync(import.meta.filename)),loaderSHA256:loader.sha256,files,
    totalEntries:files.reduce((n,f)=>n+f.physicalLines,0),predicateControls:controls.length,independentPredicatesAgree:true,
    scope:'Only explicitly requested, already-read resources. Shape, canonical decimal and modulus range, not generating-procedure, cryptographic or performance qualification; no unread-file credit.'};
const out=root+'/read-round-constants-'+indices.map(i=>String(i).padStart(2,'0')).join('-')+'-analysis.json';
const encoded=JSON.stringify(result,null,2)+'\n';
if(existsSync(out))assert.equal(readFileSync(out,'utf8'),encoded,'preserve historical evidence');else writeFileSync(out,encoded);
console.log(encoded);
