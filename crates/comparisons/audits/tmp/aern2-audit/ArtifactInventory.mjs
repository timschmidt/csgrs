// Read-only artifact inventory/consistency checks. This does not credit human
// source coverage or reproduce any historical numerical benchmark.
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import zlib from 'node:zlib';
import vm from 'node:vm';
import {execFileSync} from 'node:child_process';

const repo = '/home/tim/Documents/GitHub/workspace/exact-real-references/aern2';
const entries = execFileSync('git', ['ls-files', '-s', '-z'], {cwd:repo, encoding:'utf8'})
  .split('\0').filter(Boolean).map(s => {
    const [header,file] = s.split('\t');
    return {file,mode:header.split(' ')[0],blob:header.split(' ')[1]};
  });
const count = (s,re) => [...s.matchAll(re)].length;
const validations = [], files = [], logs = [];
function checked(file, kind, action) {
  try { validations.push({file,kind,status:'pass',detail:action()}); }
  catch(e) { validations.push({file,kind,status:'fail',detail:String(e.message).slice(0,1000)}); }
}
for (const entry of entries) {
  const p = path.join(repo,entry.file);
  if (entry.mode === '120000') {
    const target = fs.readlinkSync(p);
    files.push({...entry,kind:'symlink',target,exists:fs.existsSync(p)});
    continue;
  }
  const data = fs.readFileSync(p), s = data.toString('utf8');
  const ext = path.extname(p).slice(1);
  const kind = ['log','pdf','png','svg','gz','tgz','csv','js','html','ct2'].includes(ext) ? ext : 'source-or-metadata';
  files.push({...entry,kind,bytes:data.length,newlines:count(s,/\n/g),
    sha256:crypto.createHash('sha256').update(data).digest('hex')});
  if (ext === 'log') {
    const match = re => s.match(re)?.[1] ?? null;
    const status = match(/Exit status:\s*(\S+)/);
    const accuracyLine = s.split('\n').find(l=>/^accuracies = /.test(l));
    const accuracies = accuracyLine ? [...accuracyLine.matchAll(/Bits\s+(-?\d+)/gi)].map(m=>Number(m[1])) : [];
    const exact = accuracyLine ? count(accuracyLine,/Exact/g) : 0;
    const requested = match(/using accuracy\s+(-?\d+)/);
    const iterations = match(/\((\d+) times\)/);
    const command = match(/Command being timed: "([^"]+)"/);
    logs.push({file:entry.file,status,command,
      user:match(/User time \(seconds\):\s*(\S+)/),
      system:match(/System time \(seconds\):\s*(\S+)/),
      maxrss:match(/Maximum resident set size \(kbytes\):\s*(\S+)/),
      requested,iterations,reportedValues:accuracies.length+exact,
      minAccuracy:accuracies.length ? Math.min(...accuracies) : null,
      maxAccuracy:accuracies.length ? Math.max(...accuracies) : null,
      exact,abnormalLines:s.split('\n').filter(l=>/timed out|terminated|error:|Exception|failed|CallStack|timeout/i.test(l)).map(l=>l.slice(0,1000))});
  }
  if (ext === 'js') checked(entry.file,'JavaScript syntax (not executed)',()=>{
    new vm.Script(s); return 'parsed';
  });
  if (ext === 'html') checked(entry.file,'inline JavaScript syntax (not executed)',()=>{
    let parsed=0;
    for (const m of s.matchAll(/<script\b([^>]*)>([\s\S]*?)<\/script>/gi)) {
      if (!m[2].trim() || /type=["']application\/json/.test(m[1])) continue;
      new vm.Script(m[2]); parsed++;
    }
    return {parsed};
  });
  if (ext === 'svg') checked(entry.file,'XML syntax',()=>{
    execFileSync('xmllint',['--nonet','--noout',p],{stdio:['ignore','pipe','pipe']}); return 'parsed';
  });
  if (ext === 'pdf') checked(entry.file,'PDF inventory',()=>{
    const info=execFileSync('pdfinfo',[p],{encoding:'utf8'});
    return {pages:Number(info.match(/^Pages:\s*(\d+)/m)?.[1]),
      javascript:info.match(/^JavaScript:\s*(\S+)/m)?.[1]};
  });
  if (ext === 'png') checked(entry.file,'PNG decode',()=>
    execFileSync('identify',['-format','%w x %h',p],{encoding:'utf8'}));
  if (ext === 'gz' || ext === 'tgz') checked(entry.file,'gzip CRC/decompression',()=>{
    const decoded=zlib.gunzipSync(data);
    return {bytes:decoded.length,newlines:count(decoded.toString('utf8'),/\n/g),
      sha256:crypto.createHash('sha256').update(decoded).digest('hex')};
  });
}
const counts = {};
for (const f of files) counts[f.kind]=(counts[f.kind]||0)+1;
const logSummary={};
for (const l of logs) {
  const key=l.file.split('/')[0]+'/'+(l.status ?? 'no-exit-status');
  logSummary[key]=(logSummary[key]||0)+1;
}
const scalarAccuracyIssues=logs.filter(l=>l.requested !== null && l.minAccuracy !== null && l.minAccuracy < Number(l.requested));
const failureList=validations.filter(v=>v.status==='fail');
console.log(JSON.stringify({snapshot:execFileSync('git',['rev-parse','HEAD'],{cwd:repo,encoding:'utf8'}).trim(),
  counts,logSummary,scalarAccuracyIssues,validationFailures:failureList,
  files,logs,validations},null,2));
console.error(JSON.stringify({counts,logSummary,scalarAccuracyIssues:scalarAccuracyIssues.length,
  validationCount:validations.length,validationFailures:failureList},null,2));
