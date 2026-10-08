import {readFileSync,lstatSync,readlinkSync} from 'node:fs';
import {createHash} from 'node:crypto';
const decoder=new TextDecoder('utf-8',{fatal:true});
const entries=[];
for(const entry of readFileSync(0,'utf8').split('\0').filter(Boolean)) {
    const match=entry.match(/^(\d+) (\w+) ([a-f0-9]+)\s+(\d+|-)\t([\s\S]+)$/);
    if(!match)throw Error(`Unrecognized tree entry: ${entry}`);
    const [,mode,type,oid,size,path]=match;
    if(type!=='blob'){entries.push({mode,type,oid,path});continue;}
    const stat=lstatSync(path);
    const data=stat.isSymbolicLink()?Buffer.from(readlinkSync(path)):readFileSync(path);
    if(data.length!==+size)throw Error(`Size mismatch: ${path}`);
    let text;
    if(!data.includes(0))try{text=decoder.decode(data);}catch{}
    const lines=text===undefined?null:text.length===0?0:text.split('\n').length-(text.endsWith('\n')?1:0);
    entries.push({mode,type:stat.isSymbolicLink()?'symlink':text===undefined?'binary':'text',oid,path,bytes:data.length,lines,sha256:createHash('sha256').update(data).digest('hex')});
}
process.stdout.write(JSON.stringify(entries));
