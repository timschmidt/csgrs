import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const read=p=>readFileSync(resolve(dir,p),'utf8');
const json=p=>JSON.parse(read(p));
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
for(const file of ['boundary-bench-before.sha256','boundary-after.sha256']) {
  for(const line of read(file).trim().split('\n')) {
    const [,expected,p]=line.match(/^(\w+)\s+(.+)$/);
    // Before's old Hyper source hash is provenance, not the live candidate.
    if(file.includes('before')&&p==='hyperreal/src/computable/format.rs')continue;
    assert.equal(hash(p),expected,p);
  }
}
const inventory=read('../PLUME_FILE_INVENTORY.tsv').trim().split('\n');
const header=inventory.shift().split('\t');
const pi=header.findIndex(x=>/^(path|file)$/.test(x)),hi=header.findIndex(x=>/sha256/.test(x));
assert(pi>=0&&hi>=0,header.join(','));
for(const line of inventory) {const cols=line.split('\t');assert.equal(hash('exact-real-references/Plume/'+cols[pi]),cols[hi]);}
assert.equal(inventory.length,174);
for(const mode of ['grid','format','printed']) {
  for(const suffix of ['','-debug']) {
    const rows=json(`boundary-${mode}${suffix}-runs.json`);assert.equal(rows.length,1);
    assert.equal(rows[0].status,0);assert(!rows[0].error);assert.equal(rows[0].stdout,read(`boundary-${mode}${suffix}.log`));
  }
  assert.equal(read(`boundary-${mode}.log`),read(`boundary-${mode}-debug.log`));
}
assert(read('boundary-grid.log').includes('TOTAL (5128,17,0)'));
assert(read('boundary-printed.log').includes('PRINTED 51 32 34 32'));

const abs=x=>x<0n?-x:x;
const gcd=(a,b)=>{a=abs(a);b=abs(b);while(b){[a,b]=[b,a%b];}return a;};
const q=(a,b=1n)=>{const g=gcd(a,b);return [a/g,b/g];};
const mul=([a,b],[c,d])=>q(a*c,b*d);
const pow2=e=>e>=0?q(1n<<BigInt(e)):q(1n,1n<<BigInt(-e));
const within=([a,b],[c,d],[rn,rd])=>abs(a*d-b*c)*rd<=rn*b*d;
function decimal(s) {
  const m=s.match(/^(-?)(\d+)(?:\.(\d+))?$/);if(!m)return null;
  return q((m[1]? -1n:1n)*BigInt(m[2]+(m[3]??'')),10n**BigInt(m[3]?.length??0));
}
let formatChecks=0,formatBad=0;const formatFailures=[];const groups={};
const keys=new Set();
for(const line of read('boundary-format.log').split('\n').filter(x=>x.startsWith('ROW '))) {
  const m=line.match(/^ROW (-?\d+) (False|True) (-?\d+) (\d+) (".*") (-?\d+) (\d+)$/);assert(m,line);
  const [,a,rep,e,p,quoted,qn,qd]=m,places=Number(p),text=JSON.parse(quoted);
  const expected=mul(q(BigInt(a),32n),pow2(Number(e)));assert.deepEqual(expected,q(BigInt(qn),BigInt(qd)));
  const key=[a,rep,e,p].join(':');assert(!keys.has(key));keys.add(key);
  const output=decimal(text),pass=output&&within(output,expected,q(1n,10n**BigInt(p)));
  const group=groups[`exponent=${e} places=${p}`]??={checks:0,failures:0};group.checks++;formatChecks++;
  if(!pass){formatBad++;group.failures++;formatFailures.push({a:Number(a),rep,e:Number(e),places,text,expected:expected.map(String)});}
}
assert.equal(formatChecks,2080);assert.equal(formatBad,128);
for(let a=-32;a<=32;a++)for(const rep of ['False','True'])for(const e of [-4,0,1,4])for(const p of [0,1,3,6])assert(keys.has([a,rep,e,p].join(':')));
function prefix(text) {
  const m=text.trim().match(/^\((-?\d+),\[([-\d,]+)\]\)$/);assert(m,text);
  const e=Number(m[1]),digits=m[2].split(',').map(Number);assert.equal(digits.length,24);assert(digits.every(d=>[-1,0,1].includes(d)));
  let numerator=0n;for(const d of digits)numerator=2n*numerator+BigInt(d);
  return {e,center:mul(q(numerator,1n<<24n),pow2(e)),radius:pow2(e-24)};
}
const processes={};
for(const mode of ['productive','literal']) {
  const base=json(`boundary-${mode}-runs.json`),debug=json(`boundary-${mode}-debug-runs.json`);
  assert.equal(base.length,mode==='productive'?31:26);assert.equal(debug.length,base.length);
  let finite=0,timed=0,exceptions=0;
  for(let i=0;i<base.length;i++) {
    const row=base[i],other=debug[i];assert.deepEqual(row.args,other.args);assert.equal(row.status,other.status);assert.equal(row.error,other.error);
    assert.notEqual(row.error,'EPERM');assert.notEqual(other.error,'EPERM');
    if(row.status===0&&!row.error) {
      assert.equal(row.stdout,other.stdout);const got=prefix(row.stdout);finite++;
      let expected;
      if(mode==='productive') {
        const [,op,n]=row.args;expected=op==='mul-nonzero'?pow2(Number(n)-2):q(0n);
        if(['sb-max','sb-extra'].includes(op))assert.equal(BigInt(got.e),-BigInt(n));
      }else {
        const literal=row.args[1];const canonical=literal.replace(/^\+/, '').replace(/^(-?)\./,(_,sign)=>sign+'0.');
        expected=['','+','-'].includes(literal)?q(0n):decimal(canonical);assert(expected,literal);
      }
      assert(within(got.center,expected,got.radius));
    }else if(row.error==='ETIMEDOUT'){assert.equal(row.signal,'SIGKILL');timed++;}
    else {assert.equal(row.status,1);assert(row.stderr.length);exceptions++;}
  }
  assert.deepEqual([finite,timed,exceptions],mode==='productive'?[21,10,0]:[11,0,15]);
  processes[mode]={requests:base.length,finite,timed,exceptions};
}

const rows=json('boundary-bench-runs.json');assert.equal(rows.length,72);
const median=a=>{a=[...a].sort((a,b)=>a-b);return (a[(a.length-1)>>1]+a[a.length>>1])/2;};
const stats=a=>({median:median(a),min:Math.min(...a),max:Math.max(...a)});
// Use high bits: LCG low bits modulo8 cycle through all8 observations and
// falsely produce a zero-width bootstrap interval for an8-sample group.
let seed=0x6b0e;const rand=n=>{seed=(Math.imul(1664525,seed)+1013904223)>>>0;return Math.floor(seed/4294967296*n);};
const bootstrap=a=>{const samples=Array.from({length:10000},()=>median(a.map(()=>a[rand(a.length)]))).sort((a,b)=>a-b);return [samples[249],samples[9749]];};
assert(bootstrap([1,2,3,4,5,6,7,8])[0]<bootstrap([1,2,3,4,5,6,7,8])[1]);
seed=0x6b0e;
const parsed=rows.map(row=> {
  assert.equal(row.status,0);assert(!row.error);assert.deepEqual(row.args.slice(0,2),['-c','6']);
  const [family,n,cpu,wall,calls,bytes,checksum]=row.stdout.trim().split('\t');assert.equal(family,row.family);assert.equal(n,'512');
  return {...row,cpu:Number(cpu),wall:Number(wall),calls:Number(calls),bytes:Number(bytes),checksum};
});
const benchmarks={};
for(const family of ['rational','tiny-rational','tiny-radical','ordinary-radical']) {
  const all=parsed.filter(x=>x.family===family);assert.equal(all.length,18);
  for(let round=0;round<9;round++)assert.deepEqual(all.filter(x=>x.round===round).map(x=>x.variant),round%2?['after','before']:['before','after']);
  assert.equal(new Set(all.map(x=>x.checksum)).size,1);
  const before=all.filter(x=>x.round>0&&x.variant==='before'),after=all.filter(x=>x.round>0&&x.variant==='after');
  const ratios=before.map((x,i)=>x.cpu/after[i].cpu);
  benchmarks[family]={samples:8,before:stats(before.map(x=>x.cpu)),after:stats(after.map(x=>x.cpu)),
    medianCpuSpeedup:median(before.map(x=>x.cpu))/median(after.map(x=>x.cpu)),pairedCpuSpeedups:ratios,pairedMedianBootstrap95:bootstrap(ratios),
    beforeCalls:stats(before.map(x=>x.calls)),afterCalls:stats(after.map(x=>x.calls)),beforeBytes:stats(before.map(x=>x.bytes)),afterBytes:stats(after.map(x=>x.bytes))};
}
for(const suffix of ['release','debug'])assert(read(`boundary-hyper-${suffix}.log`).includes('decimal/history=2188, shifted-zero/nonzero precision/history=300'));
assert(read('boundary-hyper-memcheck.log').includes('ERROR SUMMARY: 0 errors'));
const testTotals={};
for(const name of ['hyper-full-debug','hyper-full-release-all-features','hyperlattice','hyperlimit','hypertri','hypersolve','hypercurve']) {
  const log=read(`boundary-${name}.log`),results=[...log.matchAll(/test result: ok\. (\d+) passed; 0 failed;/g)];
  assert(results.length && !log.includes('test result: FAILED'));
  testTotals[name]=results.reduce((n,m)=>n+Number(m[1]),0);
}
assert.equal(testTotals.hypercurve,891);
assert(read('boundary-hypercurve.log').includes('891 passed; 0 failed; 1 ignored;'));
const summary={originalHashes:174,gridChecks:5128,dyadicCapFailures:17,gridNumericExceptions:0,printed:{subtractionCases:51,rejected:32,residualCases:34,rejectedResidual:32},formatChecks,formatBad,groups,formatFailures,processes,benchmarks,
  testTotals,
  limitations:['Finite scoped tests, not universal correctness or productivity proofs.','Timeouts are observations, not benchmark samples.','Benchmark construction, validation, startup and final drop excluded; Rust cumulative requested allocation, not RSS or retained memory.','512 fresh values per process; CPU6 pinned, round0 warmups excluded, eight paired samples per family.','Scientific notation deliberately unchanged: a significant-digit request has a different contract from fixed decimal places.']};
writeFileSync(resolve(dir,'boundary-summary.json'),JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify({...summary,formatFailures:formatFailures.slice(0,4)},null,2));
