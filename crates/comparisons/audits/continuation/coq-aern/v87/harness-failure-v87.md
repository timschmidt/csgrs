# Initial seal bookkeeping failure

The first `node evidence-v87.mjs --record`, run with approved subprocess access
from v87, exited 1. It reached the reread-range assertion after revalidating
the preceding evidence, models and test captures. It did not write a manifest.
The direct exec session was 49794; its terminal output reported:

```text
AssertionError [ERR_ASSERTION]: read range hyperreal/AGENTS.md
    at file:///home/tim/Documents/GitHub/workspace/exactcore-hyper-comparison/audits/continuation/coq-aern/v87/evidence-v87.mjs:76:2
    at Array.map (<anonymous>)
    at file:///home/tim/Documents/GitHub/workspace/exactcore-hyper-comparison/audits/continuation/coq-aern/v87/evidence-v87.mjs:75:3
```

The script had recorded `['hyperreal/AGENTS.md',[[1,15]]]`; the file contains
14 physical lines and had been read completely. The corrected entry is
`['hyperreal/AGENTS.md',[[1,14]]]`. No donor read credit, numerical model,
native test, source code, earlier capture or previous manifest changed.
This is an audit bookkeeping error, not a numerical or environment failure.
The failed direct invocation did not use the capture wrapper, so no independent
JSON timestamps/stdout/stderr triplet is claimed for it. This note preserves the
command, terminal status, failing assertion and exact correction.
