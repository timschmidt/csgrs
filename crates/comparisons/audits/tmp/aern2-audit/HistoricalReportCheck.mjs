// Read-only reconciliation of stored benchmark reports; no donor JS execution.
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
const root = '/home/tim/Documents/GitHub/workspace/exact-real-references/aern2';
const read = f => fs.readFileSync(path.join(root, f), 'utf8');
const inventory = JSON.parse(fs.readFileSync('/tmp/aern2-artifact-inventory.json', 'utf8'));
const result = {criterion:[], csv:[], matrix:[], netlogs:[]};
for (const name of ['range', 'rootIsolation']) {
  const file = `aern2-fnreps/benchresults/${name}.html`, s = read(file);
  const reports = JSON.parse(s.match(/^  var reports = (.*);$/m)[1]);
  const names = JSON.parse(s.match(/^  var benches = (.*);$/m)[1].replace(/,\]$/, ']'));
  const means = JSON.parse(s.match(/^  var means = \$\.scaleTimes\((.*)\);$/m)[1].replace(/,\]$/, ']'));
  assert.equal(reports.length, names.length);
  const rows = reports.map((r, i) => {
    assert.equal(r.reportName, names[i]);
    assert.equal(r.reportAnalysis.anMean.estPoint, means[i]);
    const time = r.reportKeys.indexOf('time'), iters = r.reportKeys.indexOf('iters');
    assert(time >= 0 && iters >= 0);
    for (const sample of r.reportMeasured) {
      assert(Number.isFinite(sample[time]) && sample[time] >= 0);
      assert(Number.isSafeInteger(sample[iters]) && sample[iters] > 0);
    }
    const {anMean, anRegress, anOutlierVar} = r.reportAnalysis;
    return {name:r.reportName, samples:r.reportMeasured.length,
      mean:anMean.estPoint, meanCI:[anMean.estLowerBound,anMean.estUpperBound],
      ols:anRegress[0].regCoeffs.iters.estPoint,
      rSquared:anRegress[0].regRSquare.estPoint, outlierVariance:anOutlierVar};
  });
  result.criterion.push({file,rows});
}
for (const f of inventory.files.filter(f=>f.kind === 'csv')) {
  const lines = read(f.file).trimEnd().split('\n');
  const header = lines.shift().split(',');
  const rows = lines.map(l=> {
    const row = l.split(','); assert.equal(row.length, header.length);
    return Object.fromEntries(header.map((h,i)=>[h,row[i]]));
  });
  const metricHeaders = header.filter(h=>/Time\(s\)|Mem\(kB\)/.test(h));
  for (const r of rows) for (const h of metricHeaders) assert(Number.isFinite(Number(r[h])) && Number(r[h]) >= 0);
  const shortfalls = rows.filter(r=>r.Parameters !== undefined && /^\d+$/.test(r['Accuracy(bits)']) && +r['Accuracy(bits)'] < +r.Parameters);
  const summary = {file:f.file,rows:rows.length,shortfalls:shortfalls.length,
    worstShortfalls:shortfalls.sort((a,b)=>(+b.Parameters-+b['Accuracy(bits)'])-(+a.Parameters-+a['Accuracy(bits)'])).slice(0,6)};
  if (f.file === 'aern2-fnreps/benchresults/results.csv') {
    summary.logMatches=0; summary.missingLogs=[]; summary.mismatches=[];
    for (const r of rows) {
      const file=`aern2-fnreps/benchresults/${r.Fn}/run-${r.Op}-${r.Fn}-${r.FnRepr}-${r.Parameters}.log`;
      if (!fs.existsSync(path.join(root,file))) {summary.missingLogs.push(file);continue;}
      const log=read(file);
      const value = re => log.match(re)?.[1];
      const accuracy = value(/^accuracy:\s*(?:bits\s+)?(\S+)/im);
      const metrics = [value(/User time \(seconds\):\s*(\S+)/),value(/System time \(seconds\):\s*(\S+)/),value(/Maximum resident set size \(kbytes\):\s*(\S+)/)];
      const expected = [r['UTime(s)'],r['STime(s)'],r['Mem(kB)']];
      const same = accuracy?.toLowerCase() === r['Accuracy(bits)'].toLowerCase() && metrics.every((v,i)=>Number(i<2 && v==='0.00'?'0.01':v)===+expected[i]);
      if (same) summary.logMatches++; else summary.mismatches.push({file,accuracy,metrics,expected});
    }
  }
  result.csv.push(summary);
}
const matrix = JSON.parse(read('aern2-linear/bench/all.js').replace(/^const allData = /,'')
  .replace(/([{,]\s*)([a-z]+):/g,'$1"$2":').replace(/,\s*\]/g,']'));
for (const r of matrix) {
  const file=`aern2-linear/bench/${r.bench}/run-${r.bench}-${r.param}-${r.method}-${r.prec}.log`;
  const meta=inventory.logs.find(l=>l.file===file); assert(meta, file);
  const s=read(file), bits=s.match(/accuracy.*bits (\d+)/i)?.[1] ?? '-10000000';
  assert.equal(+bits,r.bits,file);
  assert.equal(+(meta.user==='0.00'?'0.01':meta.user),r.utime,file);
  assert.equal(+(meta.system==='0.00'?'0.01':meta.system),r.stime,file);
  assert.equal(+meta.maxrss,r.mem,file);
}
result.matrix={rows:matrix.length,logMatches:matrix.length};
for (const f of inventory.files.filter(f=>f.file.endsWith('/netlog.js'))) {
  const s=read(f.file); assert(s.startsWith("netlog='") && s.endsWith("'"));
  const events=JSON.parse(s.slice(8,-1));
  const known=new Set(), outstanding=new Map(), errors=[], types={};
  const counts={queries:0,answers:0}, cacheDescriptions={};
  for (const event of events) {
    assert.equal(Object.keys(event).length,1);
    const [type,v]=Object.entries(event)[0]; types[type]=(types[type]??0)+1;
    if(type==='QANetLogCreate') {
      if(known.has(v.qaLogCreate_newId)) errors.push('duplicate create');
      for(const id of v.qaLogCreate_sources) if(!known.has(id)) errors.push(`unknown source ${id}`);
      known.add(v.qaLogCreate_newId);
    } else if(type==='QANetLogQuery' || type==='QANetLogAnswer') {
      const query=type==='QANetLogQuery', prefix=query?'qaLogQuery_':'qaLogAnswer_';
      const client=v[prefix+'client'], provider=v[prefix+'provider'];
      if(!known.has(provider) || (client!==null && !known.has(client))) errors.push('unknown endpoint');
      const key=JSON.stringify([client,provider]);
      const pending=outstanding.get(key)??0;
      if(!query && pending===0) errors.push('answer without query');
      outstanding.set(key,pending+(query?1:-1));
      counts[query?'queries':'answers']++;
      if(!query) {const description=v.qaLogAnswer_cacheUseDescription??'';cacheDescriptions[description]=(cacheDescriptions[description]??0)+1;}
    } else errors.push(`unclassified event ${type}`);
  }
  result.netlogs.push({file:f.file,events:events.length,nodes:known.size,types,...counts,cacheDescriptions,
    outstanding:[...outstanding].filter(([,n])=>n!==0),errors});
}
console.log(JSON.stringify(result,null,2));
