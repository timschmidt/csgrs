import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const local={
  'BoundaryProbe.hs':'373bc9aaecbf219e3ecc7f266e709da023a4c69985064ed5d37c0c578a5cec03',
  'hyper_boundary.rs':'b9959e008eb0b408b8eaf4375981fee540e12da6d3a56332198b37993c9d81a8',
  'binary-search-0.0.tar.gz':'a8a50515e504693597110034fa66abc5af954fcd8f80742ad6e0f4cb020abd4d',
  'complex-generic-0.1.1.1.tar.gz':'1f535c9ab52930cfae7665b659713214af81ab6ffdfddb42c540bad8522a8b0f',
  'integer-roots-1.0.4.0.tar.gz':'a50c8287fe5f84a66bc196864e23cfc4bb9ecd10c7d664383c0c00e8f1896526',
};
for(const [path,sha] of Object.entries(local))assert.equal(hash(resolve(dir,path)),sha);
const binaries={
  'boundary-before-O0':'c7b553011b21752028816c05cd6e66da30908b3f42e720892313db5da190850c',
  'boundary-before-O2':'58068f545126e8266557167be7934a58520bb621817494d4d6483f9be05cea8c',
  'hyper-boundary-debug':'a0e9fa70cd90628a2fc1575ff38844c9eda1848849e46fa9b43e562a5801d0c3',
  'hyper-boundary-release':'30740644b5194c18d88f1ab12001d8eb3abe8ef6e2c223881cbb19d0617d5836',
};
for(const [path,sha] of Object.entries(binaries))assert.equal(hash(resolve(root,'.audit-constructible-build.l4UDoe',path)),sha);
const rows=readFileSync(resolve(dir,'DEPENDENCY_READ_COVERAGE.tsv'),'utf8').trim().split('\n').slice(1);
assert.equal(rows.length,5);let total=0;
for(const row of rows){const [path,lines,sha,range]=row.split('\t');assert.equal(hash(resolve(dir,path)),sha);assert.equal(range,`1-${lines}`);assert.equal(readFileSync(resolve(dir,path),'utf8').split('\n').length-1,Number(lines));total+=Number(lines);}
assert.equal(total,466);
const failed=[...[2,3,5,7,10].map(n=>`FAIL\tfraction--1-${n}`),...[154,155,160,200,500].map(k=>`FAIL\tscaled-finite-${k}`)].sort();
for(const mode of ['O0','O2']){
  const lines=readFileSync(resolve(dir,`boundary-${mode}.log`),'utf8').trim().split('\n');
  assert.equal(lines.pop(),'SUMMARY\t99\t109');assert.equal(lines.length,109);assert.equal(new Set(lines).size,109);
  assert.deepEqual(lines.filter(s=>s.startsWith('FAIL\t')).sort(),failed);
  assert.equal(lines.filter(s=>s.startsWith('PASS\t')).length,99);
}
for(const mode of ['debug','release']){
  const lines=readFileSync(resolve(dir,`hyper-boundary-${mode}.log`),'utf8').trim().split('\n');
  assert.equal(lines.pop(),'SUMMARY\t31\t31');assert.equal(lines.length,31);assert.equal(new Set(lines).size,31);
  assert(lines.every(s=>s.startsWith('PASS\t')));
}
console.log(JSON.stringify({native:{perBuild:109,passed:99,failed:10,builds:['O0','O2']},hyper:{perBuild:31,passed:31,builds:['debug','release']},selectedDependencyReadLines:466,productionChanges:0,fullNativeBenchmarkAndTransferAudit:'OPEN'},null,2));
