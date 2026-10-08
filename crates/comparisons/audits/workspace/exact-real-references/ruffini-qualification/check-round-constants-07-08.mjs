import {readFileSync, writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root = import.meta.dirname, prefix = 'demos/src/main/java/demo/poseidon/poseidon_constants/';
const sha = b => createHash('sha256').update(b).digest('hex');
const modulusText = '21888242871839275222246405745257275088548364400416034343698204186575808495617';
const modulus = BigInt(modulusText);
const rows = new Map(readFileSync(root+'/../RUFFINI_FILE_INVENTORY.tsv','utf8').trimEnd().split('\n').slice(1).map(s => {
    const row=s.split('\t'); return [row[0], row];
}));
function checkPinned(path, status='READ') {
    const bytes = readFileSync(root+'/../Ruffini/'+path), row = rows.get(path); assert(row);
    assert.equal(row[4],status); assert.equal(sha(bytes),row[3]); assert.equal(bytes.length,+row[1]);
    const text=bytes.toString('utf8'); assert(Buffer.from(text).equals(bytes));
    const lines=text.split('\n'); if(text.endsWith('\n'))lines.pop(); assert.equal(lines.length,+row[2]);
    return {bytes, text, lines, hash:row[3]};
}
// These are independently implemented range predicates; neither silently trims input.
function validInteger(s) { return /^(0|[1-9][0-9]*)$/.test(s) && BigInt(s)<modulus; }
function validLexical(s) {
    if(s.length===0 || (s.length>1 && s[0]==='0'))return false;
    for(const c of s) if(c<'0'||c>'9')return false;
    return s.length<modulusText.length || (s.length===modulusText.length && s<modulusText);
}
const controls=[['0',true],['1',true],[(modulus-1n).toString(),true],
    [modulusText,false],[(modulus+1n).toString(),false],['',false],['-1',false],
    ['+1',false],['01',false],['00',false],['1 ',false],[' 1',false],['1\n',false],
    ['1\r',false],['1.0',false],['1e1',false],['0x10',false],['١',false]];
for(const[s,expected]of controls){assert.equal(validInteger(s),expected,JSON.stringify(s));assert.equal(validLexical(s),expected,JSON.stringify(s));}
const loader=checkPinned('demos/src/main/java/demo/poseidon/Constants.java');
assert(/getFullRounds\(int t\)\s*\{\s*return 8;/.test(loader.text));
const partial=/getPartialRounds\(int t\)\s*\{\s*return List\.of\(([^)]+)\)\.get\(t\);/.exec(loader.text); assert(partial);
const rounds=partial[1].split(',').map(s=>Number(s.trim()));
assert.deepEqual(rounds,[56,57,56,60,60,63,64,63,60,66,60,65,70,60,64,68]);
const files=[];
for(const index of [7,8]) {
    const name='constants'+String(index).padStart(2,'0'), {bytes,text,lines,hash}=checkPinned(prefix+name);
    assert.equal(lines.length,(index+2)*(8+rounds[index]));
    for(const s of lines){assert(validInteger(s));assert(validLexical(s));}
    files.push({name,sha256:hash,bytes:bytes.length,physicalLines:lines.length,coordinates:index+2,
        fullRounds:8,partialRounds:rounds[index],finalNewline:text.endsWith('\n'),canonicalDecimalAndRange:true});
}
const result={scriptSHA256:sha(readFileSync(import.meta.filename)),loaderSHA256:loader.hash,files,
    totalEntries:files.reduce((n,f)=>n+f.physicalLines,0),predicateControls:controls.length,
    independentPredicatesAgree:true,
    scope:'Already-read constants07 and constants08 only. Shape, canonical decimal syntax and modulus range; no generating-procedure, cryptographic, performance or unread-file qualification.'};
writeFileSync(root+'/round-constants-07-08-analysis.json',JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify(result,null,2));
