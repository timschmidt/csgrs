import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname;
const read=p=>readFileSync(resolve(dir,p),'utf8');
const hash=p=>createHash('sha256').update(readFileSync(resolve(dir,p))).digest('hex');
const sourceHash='50348c07dde0dc2d1d9b1ecf95b9056bc8164bfcb1b64d2e13d9f35377d66c4f';
const reportHash='1e08410b19402f9baaebdac0e5e10005be4de58032f06e2fb4349de5ecb24e8e';
if(hash('report_fixture_controls.rs')!==sourceHash||hash('../Plume/report.txt')!==reportHash) throw Error('Source/report changed');
const expected=[
  'PASS cancellation: Real exact=200, Computable precision/history=1200, donor 30-place fixture=1; exact=-54767/66192',
  'PASS archived maximum prefix: critical point inside positive interval; exact squared-endpoint enclosure',
  'PASS exact report identity controls=6561, carry controls=189; rejected printed dyadic cross-term failures=6016, inequality failures=88'
];
for(const file of ['report-fixture-controls.log','report-fixture-controls-debug.log','report-fixture-memcheck.log']) {
  const log=read(file), lines=log.split('\n').filter(x=>x.startsWith('PASS '));
  if(JSON.stringify(lines)!==JSON.stringify(expected)||/panicked|error\[E\d+\]/.test(log)) throw Error(`Unexpected ${file}`);
}
if(!/ERROR SUMMARY: 0 errors/.test(read('report-fixture-memcheck.log'))) throw Error('Memcheck error');
if(!read('../PLUME_REPORT_COVERAGE.tsv').includes('1-4871 of4871')) throw Error('Report coverage incomplete');
const summary={sourceHash,reportHash,reportTextLinesRead:4871,visualReviewComplete:false,
  perBuild:{hyperRealExactChecks:200,hyperComputableChecks:1200,donorCancellationDecimal:1,
    archivedMaximumPrefix:true,correctDyadicIdentities:6561,correctCarryInvariants:189,
    rejectedPrintedCrossTermFailures:6016,rejectedPrintedInequalityFailures:88},
  debugReleasePassLinesIdentical:true,memcheckErrors:0,leakQualification:false,
  newHyperProductionChange:false,performanceClaim:false};
writeFileSync(resolve(dir,'report-fixture-summary.json'),JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify(summary,null,2));
