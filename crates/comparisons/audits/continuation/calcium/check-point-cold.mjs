import {createReadStream,readFileSync} from 'node:fs';
import {createInterface} from 'node:readline';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {isDeepStrictEqual} from 'node:util';
import assert from 'node:assert/strict';
import {variants,platforms,config,groups,groupKey,argsFor,bytesSha,parseGroup,coldBindings,
 metadataPath,outputPath,jsonLines} from './point-cold-protocol.mjs';
import {sha} from './point-demand-sources.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {fieldSelfTest} from './point-extended-field.mjs';
const content=r=>({left:r.left,right:r.right,report:r.report});
const sig=r=>({status:r.report.status,witness:r.report.root?.exact!=null});
const increment=(o,k,n=1)=>{o[k]=(o[k]??0)+n;};
const rowKey=r=>groupKey(r)+':'+r.call;
async function* readGroups(path){
 const stream=createReadStream(path),rl=createInterface({input:stream,crlfDelay:Infinity});let lines=[];
 try{for await(const line of rl){lines.push(line);if(lines.length===10){yield Buffer.from(lines.join('\n')+'\n');lines=[];}}}
 finally{rl.close();stream.destroy();}
 assert.equal(lines.length,0,path+' trailing partial group');
}
export async function checkCold(){
 const binding=coldBindings();fieldSelfTest();
 const summaries={},comparisons={},stateChanges=[],firstLifecycleDifferences=[],failures=[];
 const negativeControls=[],crossPlatform=[];
 for(const platform of platforms){
  const iterators=Object.fromEntries(variants.map(v=>[v,readGroups(outputPath(platform,v))[Symbol.asyncIterator]()]));
  const metadata=Object.fromEntries(variants.map(v=>[v,jsonLines(metadataPath(platform,v))]));
  const summary=Object.fromEntries(variants.map(v=>[v,{groups:0,queries:0,statuses:{},witnesses:0,checks:{},totalChecks:0,
   outputBytes:0,firstQueryStatuses:{},firstQueryWitnesses:0,perCall:Array.from({length:9},()=>({statuses:{},witnesses:0})),
   stateChangedGroups:0,statusOrWitnessChangedGroups:0,rawSha256:sha(outputPath(platform,v))}]));
  for(const v of variants)assert.equal(metadata[v].length,config.groups);
  const pairs=[['baseline','eager'],['baseline','demand'],['eager','demand']];
  const compare=Object.fromEntries(pairs.map(([a,b])=>[a+'/'+b,{equal:0,different:0,
   gainedAnswer:0,lostAnswer:0,gainedWitness:0,lostWitness:0,otherChanges:0,differences:[]}]));
  let index=0,firstByVariant={};
  for(const g of groups()){
   const rowsByVariant={};
   for(const v of variants){
    const next=await iterators[v].next();assert(!next.done,platform+' '+v+' missing group '+index);
    const bytes=next.value,rows=parseGroup(bytes,g),meta=metadata[v][index],s=summary[v];rowsByVariant[v]=rows;
    assert.equal(meta.group,groupKey(g));assert.deepEqual(meta.args,argsFor(g).map(x=>platform==='native'?String(x):x));
    assert.equal(meta.outputBytes,bytes.length);assert.equal(meta.sha256,bytesSha(bytes));
    assert(Number.isFinite(Date.parse(meta.started))&&Date.parse(meta.finished)>=Date.parse(meta.started));
    if(platform==='native'){
     assert.equal(meta.code,0);assert.equal(meta.signal,null);assert.equal(meta.error,null);assert.equal(meta.stderrBytes,0);
     assert(Number.isSafeInteger(meta.pid)&&meta.pid>0);
    }else{assert(meta.initialMemory>0&&meta.memoryAfterCollection>=meta.initialMemory);}
    if(index>0)assert(Date.parse(meta.started)>=Date.parse(metadata[v][index-1].finished));
    s.groups++;s.outputBytes+=bytes.length;
    for(const r of rows){
     s.queries++;increment(s.statuses,r.report.status);increment(s.perCall[r.call].statuses,r.report.status);
     if(sig(r).witness){s.witnesses++;s.perCall[r.call].witnesses++;}
     if(r.call===0){increment(s.firstQueryStatuses,r.report.status);if(sig(r).witness)s.firstQueryWitnesses++;}
     try{
      const result=validateExtendedObservation(r);
      failures.push(...result.failures.map(f=>({platform,variant:v,key:rowKey(r),...f})));
      for(const[k,n]of Object.entries(result.checks)){increment(s.checks,k,n);s.totalChecks+=n;}
     }catch(error){failures.push({platform,variant:v,key:rowKey(r),error:String(error)});}
    }
    const changed=rows.slice(1).filter(r=>!isDeepStrictEqual(content(r),content(rows[0])));
    if(changed.length){
     const statusChanges=changed.filter(r=>!isDeepStrictEqual(sig(r),sig(rows[0])));s.stateChangedGroups++;
     if(statusChanges.length)s.statusOrWitnessChangedGroups++;
     stateChanges.push({platform,variant:v,group:groupKey(g),initial:sig(rows[0]),
      changedCalls:changed.map(r=>r.call),sequence:rows.map(sig),
      inputChangedCalls:rows.slice(1).filter(r=>!isDeepStrictEqual([r.left,r.right],[rows[0].left,rows[0].right])).map(r=>r.call)});
    }
    if(g.lifecycle==='retained')firstByVariant[v]=content(rows[0]);
    else if(!isDeepStrictEqual(firstByVariant[v],content(rows[0])))firstLifecycleDifferences.push({platform,variant:v,group:groupKey(g),initial:sig(rows[0])});
    if(platform==='native'&&v==='demand'&&index===0){
     for(const kind of ['witness','polynomial','endpoint']){
      const row=structuredClone(rows[0]);assert(row.report.root?.exact);
      if(kind==='witness')row.report.root.exact.rational.numerator=[3];
      if(kind==='polynomial')row.report.root.polynomial[0].rational={sign:1,numerator:[1],denominator:[1]};
      if(kind==='endpoint')row.report.root.lower.rational.numerator=[3];
      const checked=validateExtendedObservation(row);assert(checked.failures.length>0);
      negativeControls.push({kind,rejectedBy:checked.failures.map(f=>f.name)});
     }
    }
   }
   for(const[a,b]of pairs){
    const c=compare[a+'/'+b];
    for(let i=0;i<9;i++){
     const x=rowsByVariant[a][i],y=rowsByVariant[b][i];
     if(isDeepStrictEqual(x,y)){c.equal++;continue;}
     c.different++;const xs=sig(x),ys=sig(y);let classified=false;
     if(xs.status!=='Transformed'&&ys.status==='Transformed'){c.gainedAnswer++;classified=true;}
     if(xs.status==='Transformed'&&ys.status!=='Transformed'){c.lostAnswer++;classified=true;}
     if(xs.status==='Transformed'&&ys.status==='Transformed'){
      if(!xs.witness&&ys.witness){c.gainedWitness++;classified=true;}
      if(xs.witness&&!ys.witness){c.lostWitness++;classified=true;}
     }
     if(!classified)c.otherChanges++;
     // Full records remain in the bound raw captures; retain every differing key.
     c.differences.push({key:rowKey(x),from:xs,to:ys,
      inputEqual:isDeepStrictEqual([x.left,x.right],[y.left,y.right]),
      fromSha256:bytesSha(JSON.stringify(content(x))),toSha256:bytesSha(JSON.stringify(content(y)))});
    }
   }
   index++;
  }
  assert.equal(index,config.groups);
  for(const v of variants){
   assert((await iterators[v].next()).done);assert.equal(summary[v].queries,10368);
   const runtime=JSON.parse(readFileSync('results/point-cold-'+platform+'-'+v+'.stderr','utf8'));
   assert.equal(runtime.groups,1152);assert.equal(runtime.queries,10368);assert.equal(runtime.outputBytes,summary[v].outputBytes);
   assert.equal(runtime.variant,v);assert.equal(runtime.platform,platform);
   assert.equal(runtime.binarySha256,binding.artifacts.find(a=>a.variant===v&&a.platform===platform).sha256);
   if(platform==='native')assert.equal(runtime.freshProcesses,1152);
   else{
    assert.deepEqual(runtime.imports,[]);assert.equal(runtime.freshInstances,1152);assert.equal(runtime.gcEvery,8);assert.equal(runtime.gcs,144);
    assert.equal(runtime.minMemoryAfterCollection,Math.min(...metadata[v].map(m=>m.memoryAfterCollection)));
    assert.equal(runtime.maxMemoryAfterCollection,Math.max(...metadata[v].map(m=>m.memoryAfterCollection)));
   }
   summary[v].runtime=runtime;
  }
  summaries[platform]=summary;comparisons[platform]=compare;
 }
 for(const v of variants)crossPlatform.push({variant:v,byteIdentical:summaries.native[v].rawSha256===summaries.wasm[v].rawSha256});
 const sameResultRegressions=Object.values(comparisons).reduce((n,c)=>n+c['eager/demand'].lostAnswer+c['eager/demand'].lostWitness,0);
 const comparisonSummary=Object.fromEntries(Object.entries(comparisons).map(([p,cs])=>[p,Object.fromEntries(Object.entries(cs).map(([k,c])=>{
  // Repeated changed records count, but unique first-query families are explicit.
  const first=c.differences.filter(d=>d.key.endsWith(':0'));
  return[k,{...c,firstQueryDifferences:first.length,distinctChangedGroups:new Set(c.differences.map(d=>d.key.split(':').slice(0,4).join(':'))).size}];
 }))]));
 return{status:failures.length===0&&sameResultRegressions===0&&firstLifecycleDifferences.length===0&&crossPlatform.every(x=>x.byteIdentical)?'pass':'fail',
 config,summaries,comparisons:comparisonSummary,failures,negativeControls,stateChanges,firstLifecycleDifferences,crossPlatform,sameResultRegressions,
 totalQueries:Object.values(summaries).flatMap(Object.values).reduce((n,s)=>n+s.queries,0),
 totalChecks:Object.values(summaries).flatMap(Object.values).reduce((n,s)=>n+s.totalChecks,0),
 limits:'Authored 48-case corpus, two policies, four initial histories, three lifecycles, nine observed calls per fresh process/instance. Every full record independently checked with the previously qualified exact field/identity oracle. Initial constructors/refinements may seed caches. No CPU, allocator, RSS, lifetime-bound, arbitrary-expression, final-consumer, representative-size or retention claim.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=await checkCold();console.log(JSON.stringify(result));if(result.status!=='pass')process.exitCode=1;
}
