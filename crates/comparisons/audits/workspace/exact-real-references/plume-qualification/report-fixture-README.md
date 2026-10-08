# Plume report and archived-fixture controls

No Hyper production change is selected. Hyperreal remains bd92d87f0e107fda0b31b93463cc8e0cc5e26d7c.
All4871 report text lines were read; selected visual-page coverage is separate
in ../PLUME_REPORT_COVERAGE.tsv. Full source-container coverage does not close
remaining report visuals, alternate report containers or functional experiments.

The original solaris/test1.era input is fed only to the modern isolated main
calculator compatibility build, never to an archived SPARC executable:

```sh
/usr/bin/time -v timeout 15s .audit-plume.SZHOs8/plume-calc < exact-real-references/Plume/solaris/test1.era
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo run --offline --release --manifest-path exact-real-references/plume-qualification/Cargo.toml --bin report_fixture_controls
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo run --offline --manifest-path exact-real-references/plume-qualification/Cargo.toml --bin report_fixture_controls
```

The first command's combined output is solaris-cancellation-calc.log. It exits0
and produces a30-place decimal within10^-30 of -54767/66192. Its timing/RSS
are single-run diagnostics, not benchmark data. Maple is not executed.

The Rust oracle uses independent GMP rationals alongside Hyper Real and
Computable. For100 input pairs (a5x5 neighborhood and all x/y sign choices),
two coefficient decompositions pass200 exact Real comparisons and1200
Computable checks at bit demands0,8,32,128,512,8. Public rational folding is
allowed. This is not a claim that opaque Computable nodes or all downstream
consumers were tested. Release/debug PASS lines are identical. Initial audit
build errors involving Rug's incomplete expressions and an annotation are
preserved in report-fixture-controls-{initial,second}-build-error.log.

The test2.mws archived maximum-value prefix is checked without floating point:
squared rational endpoints prove1/sqrt(60) is in[0.129099,0.1291], and bound
30-2*sqrt(60) inside1857/128 +/-1/256. This does not validate Maple's encoded
plot or constitute a fresh execution of the archived function-maximum run.

The same harness checks6561 dyadic multiplication identities and189 legal
signed-digit/carry invariants. Printed page45's duplicated cross term fails6016
grid cases; page47's rearranged inequality fails88. These rejected report
transcriptions are distinguished from the correct identities and from already
qualified source kernels. Other visually confirmed report discrepancies are
described in ../PLUME_FILE_NOTES.md; no donor formula was silently repaired.

Frozen test-source SHA-256:
50348c07dde0dc2d1d9b1ecf95b9056bc8164bfcb1b64d2e13d9f35377d66c4f.
Read report text SHA-256:
1e08410b19402f9baaebdac0e5e10005be4de58032f06e2fb4349de5ecb24e8e.

```sh
valgrind --tool=memcheck --error-exitcode=77 --leak-check=no .audit-targets/ireal-derivative-18555/release/report_fixture_controls
node exact-real-references/plume-qualification/analyze-report-fixture.mjs
```

Memcheck is scoped to this fixture/identity oracle, not the full library or a
leak qualification. No new performance candidate emerged from these report
formulas, so no additional performance or binary-size claim is made.
