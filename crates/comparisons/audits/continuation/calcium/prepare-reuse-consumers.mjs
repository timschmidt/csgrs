// A new verified snapshot; never repoint or modify the immutable v2 consumers.
import { readFileSync, writeFileSync, mkdirSync, copyFileSync, symlinkSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)), workspace=resolve(here,'../../../..');
const previous=JSON.parse(readFileSync(resolve(here,'sign-consumers.json')));
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
for(const {crate,files} of previous.crates) for(const f of files) {
  assert.equal(sha(resolve(here,'sign-consumers/baseline',crate,f.path)),f.sha256);
  assert.equal(sha(resolve(workspace,crate,f.path)),f.sha256,`Live file changed: ${crate}/${f.path}`);
}
const destination=resolve(here,'reuse-consumers');
mkdirSync(destination);
symlinkSync(resolve(here,'root-exp-reuse-trial-hyperreal'),resolve(destination,'hyperreal'),'dir');
for(const {crate,files} of previous.crates) for(const f of files) {
  const target=resolve(destination,crate,f.path);
  mkdirSync(dirname(target),{recursive:true});
  copyFileSync(resolve(here,'sign-consumers/baseline',crate,f.path),target,
    constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  assert.equal(sha(target),f.sha256);
}
const report={schema:1,recorded:new Date().toISOString(),crates:previous.crates,
  scalar:resolve(here,'root-exp-reuse-trial-hyperreal'),
  note:'Existing 773 consumer snapshot files also match current live bytes. New untracked-file inventory is not established by this hash comparison. Only scalar dependency differs from frozen baseline; no consumer edits.'};
writeFileSync(resolve(here,'reuse-consumers.json'),JSON.stringify(report,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({crates:report.crates.length,files:report.crates.flatMap(c=>c.files).length}));
