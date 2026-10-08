# Preserved initial size-harness failure

The original `measure-e-qualified-app-size.initial.mjs` completed
`e-qualified-app-baseline-hyperreal-build` successfully at
2026-09-10T01:23:58.128Z, then copied the original executable to
`/tmp/calcium-e-qualified-apps.AHiwfn/baseline-hyperreal-readme_quickstart`.

Its next requested tag was
`e-qualified-app-baseline-hyperreal-readme_quickstart-strip`.
`capture.mjs` accepts only `[a-z0-9-]+`; its initial validation rejected the
underscore before opening output files or spawning `strip`. The observed error
was `Usage: capture.mjs TAG CWD COMMAND [ARG ...]`; the parent script exited with
code 1. There is therefore no fabricated per-command capture for this rejected
tag. The successful preceding build capture, original script and executable
are preserved.

The corrected script normalizes only capture tags and resumes this exact
directory, validating any reused successful gate's command and arguments.
It does not replace the compiled source, rebuild an already successful baseline
command, discard the failure or create another snapshot directory. Numerical
and performance evidence are unaffected by this orchestration failure.
