# Evidence harness path correction

The first approved `node evidence-v85.mjs --record` exited 1 before creating a
manifest. It interpreted the historical relative candidate root from the new
coq-aern working directory instead of its original Calcium directory:

```
Error: ENOENT: no such file or directory, open
'twelfth-revision-candidate-v79/hyperreal/.github/workflows/ci.yml'
at evidence-v85.mjs:18:74
Node.js v22.22.2
```

The exact incorrect expression was `sha(isolated.candidateRoot+'/'+p)`.
It is corrected to `sha(resolve(c,isolated.candidateRoot,p))`. This is an audit
harness failure, not a changed/missing production file or donor defect. The
failed attempt is recorded in the tool transcript; no successful manifest was
overwritten. The summary status field was also moved after the nested numerical
summary so the latter cannot overwrite `sealed` with `verified`.
