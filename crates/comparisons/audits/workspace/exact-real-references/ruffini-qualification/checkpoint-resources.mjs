import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const root=import.meta.dirname,ws=resolve(root,'../..');
const args=process.argv.slice(2);assert.equal(args.length,3);assert(args.every(s=>/^[1-9][0-9]*$/.test(s)));
const [expectedFiles,expectedLines,firstUnread]=args.map(Number);
assert(expectedFiles>=240 && expectedFiles<=244);assert(firstUnread>=12 && firstUnread<=16);
assert(expectedLines<=26222);
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const read=p=>readFileSync(p,'utf8');
const snapshot=JSON.parse(read(root+'/square-root-pilot/snapshot.json'));assert.equal(snapshot.files.length,397);
const currentDrift=snapshot.files.filter(([p,h])=>hash(ws+'/'+p)!==h).map(([p,h])=>({path:p,expected:h,actual:hash(ws+'/'+p)}));
assert.deepEqual(currentDrift,[]);
const coverage=read(root+'/../RUFFINI_READ_COVERAGE.tsv').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
const inventory=read(root+'/../RUFFINI_FILE_INVENTORY.tsv').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
assert.equal(inventory.length,245);assert.equal(new Set(inventory.map(r=>r[0])).size,245);
assert.equal(coverage.length,expectedFiles);assert.equal(new Set(coverage.map(r=>r[0])).size,expectedFiles);
assert.equal(coverage.reduce((n,r)=>n+(+r[1]),0),expectedLines);
for(const[p,n,h,range]of coverage){assert.equal(hash(root+'/../Ruffini/'+p),h,p);assert.equal(range,'1-'+n);}
const groups={READ:[],REVIEWED_ASSET:[],UNREAD:[]};
for(const row of inventory){
    assert(groups[row[4]]);const path=root+'/../Ruffini/'+row[0],bytes=readFileSync(path),text=bytes.toString('utf8');
    assert(Buffer.from(text).equals(bytes));assert.equal(bytes.length,+row[1]);assert.equal(hash(path),row[3]);
    assert.equal(text.split('\n').length-(text.endsWith('\n')?1:0),+row[2]);groups[row[4]].push(row);
}
assert.deepEqual(groups.READ.map(r=>r[0]),coverage.map(r=>r[0]));
assert.deepEqual(groups.REVIEWED_ASSET.map(r=>r[0]),['abstractions.svg']);assert.equal(+groups.REVIEWED_ASSET[0][2],12);
const expectedUnread=Array.from({length:16-firstUnread},(_,i)=>'constants'+String(firstUnread+i).padStart(2,'0'));
assert.deepEqual(groups.UNREAD.map(r=>r[0].split('/').at(-1)),expectedUnread);
const unreadLines=groups.UNREAD.reduce((n,r)=>n+(+r[2]),0);assert.equal(expectedLines+12+unreadLines,26234);
const artifacts=['inventory.mjs','../RUFFINI_FILE_INVENTORY.tsv','../RUFFINI_READ_COVERAGE.tsv',
    'check-read-round-constants.mjs','read-round-constants-09-10-analysis.json','read-round-constants-09-10-11-analysis.json'];
const result={status:'PROGRESS; full ecosystem audit OPEN',sourceSHA256:hash(import.meta.filename),
    sourceDataReadFiles:expectedFiles,sourceDataReadLines:expectedLines,separatelyReviewedAssets:1,
    unreadFiles:expectedUnread,unreadLines,currentHyperSnapshotFiles:397,currentDrift,
    artifactSHA256:Object.fromEntries(artifacts.map(p=>[p,hash(root+'/'+p)])),newProductionChanges:0,
    scope:'Current coverage/resource fingerprint checkpoint; not a rerun of historical native arithmetic or performance qualification. File reads are credited explicitly in the ledger, never inferred from this checker.'};
const encoded=JSON.stringify(result,null,2)+'\n',out=root+'/resource-checkpoint-'+expectedFiles+'-files.json';
if(existsSync(out))assert.equal(read(out),encoded,'preserve existing checkpoint');else writeFileSync(out,encoded);
console.log(encoded);
