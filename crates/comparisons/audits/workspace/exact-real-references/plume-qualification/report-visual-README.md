# Primary Plume report visual closure and printed-table controls

Later closure checkpoint: the alternate report's complete4871-line text and11
selected PDF pages, including all5 extraction-difference pages, are now reviewed.
All147 generated HTML section links are inventoried. See ../PLUME_FILE_NOTES.md
and author-report-analysis.log for current scope and the explicit uncredited
generated-container limits; earlier open-status wording below is historical.

All133 primary report pages are independently visually read, separately from
the prior complete4871-line text read. Read ranges/status are in
../PLUME_REPORT_COVERAGE.tsv; report/source discrepancies and comparison to
Hyper are in ../PLUME_FILE_NOTES.md. Alternate author-report and unacquired
HTML containers remain open. No new Hyper production edit is selected here.

Primary PDF SHA-256:
3707551d0aba1e26f5729684d86ab4d9bb344b609553d6ae283c9d78df416952.
Primary extracted text:
1e08410b19402f9baaebdac0e5e10005be4de58032f06e2fb4349de5ecb24e8e.

## Printed logistic table

Physical19/printed11 reports ten six-decimal correct-result entries for the
logistic recurrence4*x*(1-x), input0.671875=43/64 (physical18/printed10).
They are transcribed without numerical alteration in report-logistic-table.tsv:
98c19aa01511f8708bb8cc6e186dcde0342383134d0fb895b6c4be3cf9f857eb.
The existing late_oracle TSV schema is reused; the arbitrary label `greedy`
does not imply that a donor stream was executed for these table rows.
Each printed value is the center of its six-decimal rounding cell, radius
1/2000000. All ten4096-bit MPFR outward-enclosure checks pass.

```sh
.audit-targets/ireal-derivative-18555/release/late_oracle logistic exact-real-references/plume-qualification/report-logistic-table.tsv
.audit-targets/ireal-derivative-18555/release/late_oracle hyper 43 64 60 32
node exact-real-references/plume-qualification/analyze-report-visuals.mjs
```

Outputs are report-logistic-table-oracle.log, report-logistic-hyper.log and
report-visual-analysis.log. The existing late_oracle source is unchanged:
e37beb789ee61d654f5d8f4b221afd1f5425a26ffcb8c82160c33719a7d9a10a.
Its reused release binary is
97e397f37897680b3ca9b0a996ecbe5841bea6535a2d094f0bb66efae9e6cce4.
That binary was built before the Display change: its four Hyper approximation/
history queries are explicitly historical-binary evidence, not a new current
production-build qualification. Current Display qualification is independently
recorded in boundary-README.md. No benchmark binary was rebuilt or altered.

analyze-report-visuals.mjs uses an independent Node BigInt implementation:
fixed dyadic outward enclosures at1024,2048 and4096 bits, dependency-safe
interval multiplication, no MPFR or Hyper arithmetic. All30 table checks pass;
the complete enclosure lies strictly within each rounding cell. Its enclosure
is also checked against an unrounded expanded rational recurrence for every
iteration0-12 at all three precisions:39 self-checks pass. Exact unrounded
rational growth doubles bit length, so it is deliberately not continued to60.
The printed single/double columns are not reproduced and these checks do not
qualify any historical timing or compiler/processor behavior.

Printed18's B-adic scale defect is checked using36 examples: three bases
2,3,10 andn=1..12. Taking c_n=B^n exactly satisfies the stated error condition
forr=1, while the printed c_n/(B^-n) equalsB^(2n), not a convergent sequence
toward1. The corrected expression c_n/B^n equals1. This is a report equation
error, not an inferred donor-source or Hyper bug.

## Linked image and validation boundaries

The web reader could not fetch the historical diagram; direct HTTPS retrieval
succeeded without altering the original image:

```sh
curl -sS --fail --max-time 30 https://www.dcs.ed.ac.uk/home/mhe/plume/img216.gif -o exact-real-references/Plume/img216.gif
```

585x256 GIF89a; SHA-256
12465a21095f3adfb0d64edb908c652f4c700cfe028e29112d3438e621186fe6.
It was independently viewed and agrees with primary figure6.1. No read credit
is inferred for other HTML links, navigation icons or the alternate report.

The numerical validator checks fixture hashes and ledger consistency; it
cannot mechanically prove that a person/model read the pages. The explicit
range ledger records actual visual review, including blank pages. Truncated
image calls were repeated without crediting the truncated calls.
