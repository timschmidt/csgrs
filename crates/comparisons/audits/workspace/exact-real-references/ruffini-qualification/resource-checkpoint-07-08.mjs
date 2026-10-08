import {readFileSync, writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import './verify-abstractions.mjs';
import './check-round-constants-07-08.mjs';
const root=import.meta.dirname,ws=resolve(root,'../..');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const read=p=>readFileSync(p,'utf8');
const snapshot=JSON.parse(read(root+'/square-root-pilot/snapshot.json'));
assert.equal(snapshot.files.length,397);
const drift=snapshot.files.filter(([p,h])=>hash(ws+'/'+p)!==h).map(([p,h])=>({path:p,expected:h,actual:hash(ws+'/'+p)}));
assert.deepEqual(drift,[]);
const coverage=read(root+'/../RUFFINI_READ_COVERAGE.tsv').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
const inventory=read(root+'/../RUFFINI_FILE_INVENTORY.tsv').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
assert.equal(inventory.length,245);assert.equal(coverage.length,237);
assert.equal(new Set(inventory.map(r=>r[0])).size,245);assert.equal(new Set(coverage.map(r=>r[0])).size,237);
assert.equal(coverage.reduce((n,r)=>n+(+r[1]),0),19087);
for(const[p,n,h,range]of coverage){assert.equal(hash(root+'/../Ruffini/'+p),h,p);assert.equal(range,'1-'+n);}
const groups={READ:[],REVIEWED_ASSET:[],UNREAD:[]};
for(const row of inventory){assert(groups[row[4]]);assert.equal(hash(root+'/../Ruffini/'+row[0]),row[3]);groups[row[4]].push(row);}
assert.deepEqual(groups.READ.map(r=>r[0]),coverage.map(r=>r[0]));
assert.deepEqual(groups.REVIEWED_ASSET.map(r=>r[0]),['abstractions.svg']);
assert.equal(+groups.REVIEWED_ASSET[0][2],12);
assert.equal(groups.UNREAD.length,7);assert.equal(groups.UNREAD.reduce((n,r)=>n+(+r[2]),0),7135);
assert.deepEqual(groups.UNREAD.map(r=>r[0].split('/').at(-1)),['constants09','constants10','constants11','constants12','constants13','constants14','constants15']);
const artifacts=['inspect-abstractions-v2.mjs','verify-abstractions.mjs','abstractions-audit/metadata.json',
    'abstractions-audit/verification.json','check-round-constants-07-08.mjs','round-constants-07-08-analysis.json',
    'inventory.mjs','../RUFFINI_FILE_INVENTORY.tsv','../RUFFINI_READ_COVERAGE.tsv'];
const result={status:'PROGRESS; full ecosystem goal OPEN',sourceSHA256:hash(import.meta.filename),
    donorPin:'82d552fee22d92e493936183fab8672517694e56',sourceDataReadFiles:237,sourceDataReadLines:19087,
    separatelyReviewedAssets:1,unreadFiles:7,unreadLines:7135,currentHyperSnapshotFiles:397,currentDrift:drift,
    artifactSHA256:Object.fromEntries(artifacts.map(p=>[p,hash(root+'/'+p)])),
    newProductionChanges:0,scope:'Current resource/asset evidence and source-fingerprint checkpoint only; not a rerun of old native tests or performance experiments.'};
writeFileSync(root+'/resource-checkpoint-07-08.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
