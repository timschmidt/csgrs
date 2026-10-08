import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname;
const results=[];
for(const packageName of ['binary-search-0.0','complex-generic-0.1.1.1','integer-roots-1.0.4.0']){
  const tar=gunzipSync(readFileSync(resolve(dir,`${packageName}.tar.gz`)));
  let files=0,bytes=0;
  for(let offset=0;offset+512<=tar.length;){
    const header=tar.subarray(offset,offset+512);if(header.every(b=>b===0))break;
    const str=(start,len)=>header.subarray(start,start+len).toString().replace(/\0.*$/s,'');
    const checksum=parseInt(str(148,8).trim(),8);
    assert.equal(header.reduce((n,b,i)=>n+(i>=148&&i<156?32:b),0),checksum);
    const prefix=str(345,155),name=(prefix?prefix+'/':'')+str(0,100);
    assert(name.startsWith(packageName+'/')&&!name.split('/').includes('..'));
    const size=parseInt(str(124,12).trim(),8),type=str(156,1);
    assert(Number.isSafeInteger(size)&&size>=0);
    assert(['','0','5'].includes(type),`unexpected tar member type ${type}`);
    if(type!=='5'){
      const body=tar.subarray(offset+512,offset+512+size);
      assert.equal(body.length,size);assert.deepEqual(readFileSync(resolve(dir,name)),body,name);
      files++;bytes+=size;
    }
    offset+=512+Math.ceil(size/512)*512;
  }
  results.push({package:packageName,files,bytes,sourceBytesMatchArchive:true});
}
console.log(JSON.stringify(results,null,2));
