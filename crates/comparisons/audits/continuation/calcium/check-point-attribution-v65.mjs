import assert from 'node:assert/strict';
import {analyse,readRows,validateRows,paired} from './analyse-point-attribution-v65.mjs';
import {checkedPlan,paths,json} from './point-attribution-runtime-v65.mjs';
import {nativeExpected} from './point-wasm-protocol.mjs';
import {medianOrderInterval} from './paired-statistics-v60.mjs';

const saved=json('point-attribution-analysis-v65.json');assert.deepEqual(analyse(),saved);
const plan=checkedPlan().plan,expected=nativeExpected(),original=readRows(paths(0).raw),rejected=[];
const mutations={
 truncate:rs=>rs.pop(),duplicate:rs=>rs[1]=structuredClone(rs[0]),
 observation:rs=>rs[0].observation++,gc:rs=>rs[0].gcsBefore++,pass:rs=>rs[0].pass++,setting:rs=>rs[0].setting='single',
 phase:rs=>rs[0].phase='measurement',block:rs=>rs[0].block=0,position:rs=>rs[0].position=0,
 case:rs=>rs[0].case++,policy:rs=>rs[0].policy++,history:rs=>rs[0].history++,lifecycle:rs=>rs[0].lifecycle='fresh',
 variant:rs=>rs[0].variant='demand',iterations:rs=>rs[0].iterations++,checksum:rs=>rs[0].host_checksum++,
 counterArithmetic:rs=>rs[0].thread_cpu_ns++,negativeCounter:rs=>rs[0].processUserUs=-1,
 resource:rs=>delete rs[0].resources.minorPageFault,negativeResource:rs=>rs[0].resources.majorPageFault=-1,
 zeroWall:rs=>rs[0].elapsed_ns=0,clockBound:rs=>rs[0].elapsed_ns=1e12,
 timestamp:rs=>rs[0].finished='invalid',memory:rs=>rs[0].memoryAfterBatch=0,
 unchangedFlag:rs=>rs[0].input_records_unchanged=false,
 result:rs=>rs[0].actual.status='Unknown',input:rs=>rs[0].left.count++,
};
for(const[name,mutate]of Object.entries(mutations)){
 const rows=[...original];rows[0]=structuredClone(rows[0]);mutate(rows);
 assert.throws(()=>validateRows(rows,0,plan,expected),undefined,name);rejected.push(name);
}
// Zero CPU readings are legal; they are not removed from observations.
const zeros=[...original];zeros[0]=structuredClone(zeros[0]);
for(const k of ['thread_cpu_ns','threadUserUs','threadSystemUs','process_cpu_ns','processUserUs','processSystemUs'])zeros[0][k]=0;
validateRows(zeros,0,plan,expected);
const blocks=Array.from({length:36},()=>['baseline','eager','demand','demand','eager','baseline'].map(variant=>({variant,t:variant==='demand'?20:10})));
const constant=paired(blocks,'eager','t','v65-constant-control');assert(constant.available);
assert.equal(constant.pairedMedianRatio,2);assert.deepEqual(constant.bootstrap.interval,[2,2]);assert.deepEqual(constant.orderStatistic.interval,[2,2]);
for(const r of blocks[0])if(r.variant==='eager')r.t=0;
const unavailable=paired(blocks,'eager','t','v65-zero-control');assert.equal(unavailable.available,false);
assert.equal(unavailable.blockMedians.length,36);assert.equal(unavailable.blockMedians[0].eager,0);
assert(!('pairedMedianRatio'in unavailable));
// Independent Pascal convolution, using exact integers below 2^53, checks
// the n=36 order ranks and strongest coverage at or above 95 percent.
let binomial=[1];for(let n=0;n<36;n++){
 const next=Array(n+2).fill(0);for(let k=0;k<binomial.length;k++){next[k]+=binomial[k];next[k+1]+=binomial[k];}binomial=next;
}
assert.equal(binomial.reduce((n,x)=>n+x,0),2**36);
let best;for(let k=1;k<=18;k++){
 const inside=binomial.slice(k,37-k).reduce((n,x)=>n+x,0);
 if(inside/(2**36)>=.95)best={k,inside};
}
const order=medianOrderInterval(Array.from({length:36},(_,i)=>i+1));
assert.equal(order.lowerRank,best.k);assert.equal(order.upperRank,37-best.k);
assert.equal(order.coverageNumerator,String(best.inside));assert.equal(order.coverageDenominator,String(2**36));
assert.deepEqual(order.interval,[best.k,37-best.k]);
console.log(JSON.stringify({checkpoint:65,status:'pass',fullRecomputation:true,rejectedControls:rejected,
 zeroCountersPreserved:true,zeroDenominatorWholeMetricUnavailable:true,constantRatio:2,independentOrder36:order,
 measured:saved.measured,qualification:saved.qualification,emptyControls:saved.emptyControls}));
