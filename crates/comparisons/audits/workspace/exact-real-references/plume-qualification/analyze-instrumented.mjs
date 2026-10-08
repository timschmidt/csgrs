import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname;
const read=n=>readFileSync(resolve(dir,n),'utf8');
const hash=x=>createHash('sha256').update(x).digest('hex');
const json=n=>JSON.parse(read(n));
const inventory=read('../PLUME_FILE_INVENTORY.tsv').trim().split('\n').slice(1).map(x=>x.split('\t'));
for(const [path,,bytes,,expected] of inventory) {
  const data=readFileSync(resolve(dir,'../Plume',path));
  assert.equal(data.length,+bytes,path);assert.equal(hash(data),expected,path);
}
const coverage=new Map(read('../PLUME_READ_COVERAGE.tsv').trim().split('\n').slice(1).map(x=>{const [p,...r]=x.split('\t');return [p,r]}));
const perform=inventory.filter(x=>x[0].startsWith('versioned/perform/'));
assert.equal(perform.length,36);
for(const [path,,,lines] of perform) assert.equal(coverage.get(path)[0],`1-${lines}`,path);
const sourceHash='af10ffbec882d11aa60a2290a3e803f187e0833b88cd2d53d101b756b3bdfd2b';
assert.equal(hash(read('InstrumentedProbe.hs')),sourceHash);
assert.equal(read('InstrumentedProbe.hs'),read('InstrumentedProbe.bench-final.hs'));
const gridRuns=json('instrumented-grid-runs.json');assert.equal(gridRuns.length,8);
for(const r of gridRuns) {
  assert.equal(r.status,0);assert.equal(r.error,undefined);assert.equal(r.signal,null);
  const data=read(r.output);assert.equal(hash(data),r.sha256);assert.equal(Buffer.byteLength(data),r.bytes);
}
for(const suffix of ['grid.log','extra.log','demand.tsv','examples.log']) {
  assert.equal(read(`instrumented-${suffix}`),read(`instrumented-debug-${suffix}`),`O2/O0 difference: ${suffix}`);
}
function grid(name,expected) {
  const text=read(name), total=text.match(/TOTAL \((\d+),(\d+),(\d+)\)/);
  assert(total);assert.deepEqual(total.slice(1).map(Number),expected);
  return {checks:expected[0],failures:expected[1],exceptions:expected[2]};
}
const grids={arithmetic:grid('instrumented-grid.log',[200017,3841,0]),extra:grid('instrumented-extra.log',[1498,266,0])};
const demand=read('instrumented-demand.tsv').trim().split('\n').slice(1).map(x=>x.split(' '));
assert.equal(demand.length,152700);assert.equal(new Set(demand.map(x=>x.slice(0,4).join('/'))).size,demand.length);
const demandCounts={};
for(const [op,a,b,n,reported,actual,label] of demand) {
  assert(+a>=0&&+a<81&&+b>=0&&+b<81);assert([1,3,8].includes(+n));assert(+actual<=128);
  assert.equal(label,+reported<+actual ? 'UNDER' : +reported===+actual ? 'EXACT' : 'OVER');
  const c=demandCounts[op]??={checks:0,under:0,over:0,excess_sum:0,max_excess:-Infinity};
  c.checks++;c.under+=label==='UNDER';c.over+=label==='OVER';
  c.excess_sum+=+actual-+n;c.max_excess=Math.max(c.max_excess,+actual-+n);
}
assert.equal(demandCounts.multiply.under,486);
assert.equal(Object.values(demandCounts).reduce((n,x)=>n+x.under,0),486);
assert.equal(Object.values(demandCounts).reduce((n,x)=>n+x.over,0),0);
for(const c of Object.values(demandCounts)) c.mean_excess=c.excess_sum/c.checks;
const elementary=json('instrumented-elementary-runs.json');assert.equal(elementary.length,108);
assert.equal(new Set(elementary.map(x=>[x.op,x.a,x.b,x.bits].join('/'))).size,108);
assert(!elementary.some(x=>x.error==='EPERM'));
const mainInputs=new Set(json('elementary-runs.json').map(x=>[x.op,x.a,x.b,x.bits].join('/')));
for(const r of elementary) assert(mainInputs.has([r.op,r.a,r.b,r.bits].join('/')));
const finite=elementary.filter(x=>x.status===0);
assert.equal(finite.length,100);assert(finite.every(x=>!x.error&&!x.signal));
assert.deepEqual(read('instrumented-elementary-results.tsv').trim().split('\n').map(x=>x.split(/\s+/).join(' ')),finite.map(x=>x.stdout.trim()));
assert.match(read('instrumented-mpfr.log'),/TOTAL\s+checks=100\s+failures=20/);
const boundaries=json('instrumented-boundary-runs.json');assert.equal(boundaries.length,3);
assert(boundaries.every(x=>!x.error&&!x.signal));
assert.equal(boundaries.filter(x=>x.status===251&&x.stderr.includes('Heap exhausted')).length,2);
assert.equal(boundaries.find(x=>x.name==='dyadic-norm-cap').stdout,'20\n');
const benchRuns=json('instrumented-bench-runs.json');assert.equal(benchRuns.length,216);
const benchRows=read('instrumented-bench-results.tsv').trim().split('\n').slice(1).map(x=>x.split('\t'));
assert.equal(benchRows.length,216);assert.equal(new Set(benchRows.map(r=>r.slice(0,4).join('/'))).size,216);
for(let i=0;i<216;i++) {
  const r=benchRuns[i], row=benchRows[i];
  assert.equal(r.status,0);assert.equal(r.error,undefined);assert.equal(r.signal,null);
  assert.deepEqual(r.args.slice(0,2),['-c','6']);
  assert.deepEqual([String(r.round),...r.stdout.trim().split(/\s+/)],row);
  assert.equal(row[4],'256');assert.equal(row[8],'PASS');assert(+row[5]>0&&+row[6]>0);
}
const median=a=>{const s=[...a].sort((x,y)=>x-y);return (s[3]+s[4])/2};
const stat=a=>({median:median(a),min:Math.min(...a),max:Math.max(...a),mean:a.reduce((s,x)=>s+x,0)/a.length});
const benchmark=[];
for(const family of ['dense','sparse','terminating']) for(const bits of [32,64,128,256]) {
  const groups=['average','carry-average'].map(op=>{
    const rows=benchRows.filter(r=>r[0]!=='0'&&r[1]===op&&r[2]===family&&+r[3]===bits);
    assert.equal(rows.length,8);
    return {op,rows,cpu_ms:stat(rows.map(r=>+r[5]/1e9)),allocated_bytes:stat(rows.map(r=>+r[6]))};
  });
  const pairRatios=[];
  for(let round=1;round<=8;round++) {
    const a=groups[0].rows.find(x=>+x[0]===round),b=groups[1].rows.find(x=>+x[0]===round);
    assert(a&&b);pairRatios.push(+a[5]/+b[5]);
    const first=benchRows.find(r=>+r[0]===round&&r[2]===family&&+r[3]===bits);
    assert.equal(first[1],round%2 ? 'carry-average' : 'average');
  }
  benchmark.push({family,bits,
    variants:groups.map(({rows,...s})=>s),paired_cpu_ratio_average_over_carry:stat(pairRatios),
    median_cpu_ratio_average_over_carry:groups[0].cpu_ms.median/groups[1].cpu_ms.median,
    allocation_ratio_average_over_carry:groups[0].allocated_bytes.median/groups[1].allocated_bytes.median});
}
for(const suffix of ['tests','release-tests']) {
  assert.match(read(`instrumented-hyper-linear-${suffix}.log`),/5 passed; 0 failed/);
  assert.match(read(`instrumented-hyper-format-${suffix}.log`),/23 passed; 0 failed/);
}
assert.match(read('instrumented-hyper-decimal.log'),/PASS exact decimal output controls=1170/);
assert.match(read('instrumented-memcheck-extra.log'),/ERROR SUMMARY: 0 errors from 0 contexts/);
const summary={source_files_verified:inventory.length,versioned_perform_files_read:36,
  versioned_perform_lines_read:perform.reduce((n,x)=>n+(+x[3]),0),probe_sha256:sourceHash,
  optimized_unoptimized_outputs_identical:true,grids,demand:{rows:demand.length,operations:demandCounts},
  elementary:{requests:108,finite:100,mpfr_failures:20,
    heap_limits:elementary.filter(x=>x.stderr?.includes('Heap exhausted')).length,
    timeouts:elementary.filter(x=>x.error==='ETIMEDOUT').length},
  benchmark_observations:216,warmup_observations:24,final_observations:192,benchmark,
  hyper:{head:'bd92d87f0e107fda0b31b93463cc8e0cc5e26d7c',decimal_checks:1170,debug_tests:28,release_tests:28,
    new_production_changes:0},
  limitations:['Finite demand grid measures numeric values, not list spines, execution time or a universal dependency bound.',
    'Averaging benchmarks use instrumented Haskell objects with pre-forced inputs; no Hyper speed or retained-RSS comparison.',
    'Submillisecond timings and variable host load limit conclusions; all min/max and paired ratios retained.',
    'Separate archive copies, CGI, remaining report and broader ecosystem audit are still open.']};
writeFileSync(resolve(dir,'instrumented-qualification-summary.json'),JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify(summary,null,2));
