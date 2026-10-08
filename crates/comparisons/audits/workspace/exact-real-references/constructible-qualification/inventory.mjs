import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname;
const rows=readFileSync(resolve(dir,'../CONSTRUCTIBLE_READ_COVERAGE.tsv'),'utf8').trim().split('\n').slice(1);
assert.equal(rows.length,4);
let lines=0;
for(const row of rows){
  const [path,count,hash,ranges]=row.split('\t');const body=readFileSync(resolve(dir,'../haskell-constructible',path));
  assert.equal(createHash('sha256').update(body).digest('hex'),hash,path);
  assert.equal(body.toString().split('\n').length-1,Number(count),path);
  let next=1;for(const range of ranges.split(';')){const [a,b]=range.split('-').map(Number);assert.equal(a,next,path);assert(b>=a);next=b+1;}
  assert.equal(next,Number(count)+1,path);lines+=Number(count);
}
assert.equal(lines,490);
console.log(JSON.stringify({pin:'46d760cbd2d21f955ec96c8fe2c13fdf3b2dd9d0',files:4,physicalLines:lines,sourceReading:'complete',nativeAndTransferQualification:'OPEN'}));
