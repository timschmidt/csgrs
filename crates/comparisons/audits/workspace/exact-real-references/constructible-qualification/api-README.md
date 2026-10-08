# Constructible API qualification — 2026-09-06

Both native O0/O2 runs pass 8,113/8,113 checks, terminal0, no60s caps:

- 7,944 corpus checks: for both sides of all1,324 independently qualified field
  identities, read(show x)==x, read(show(-x))==-x, and recursive deconstruction.
  Every irrational triple must have b!=0,r>0,reconstruct x=a+b*sqrt(r), and
  recursively satisfy the same contract. A512-level harness depth cap was
  never reached; this finite corpus does not prove termination for all objects.
- 161 enumeration checks: succ/pred, six-value enumFrom/enumFromThen, inclusive
  enumFromTo, descending enumFromThenTo and a finite prefix of a zero-step range,
  across17 signed half-integers and six signed irrational roots.
- Seven parser precedence/trailing-garbage/list controls, and one typed
  unsupported logBase exception control supplementing ContractProbe.

The source build initially failed because a harness list-enumeration type
annotation was malformed. Corrected before execution; original failure log
retained. No donor source or dependency edits. ApiProbe imports the frozen
FieldProbe Main module and builds with -main-is ApiProbe using the direct GHC
recipe in boundary-README.md.

These self-consistency/API checks supplement, rather than replace, the integer
identity corpus and directed-MPFR floating oracle. They do not hide the prior
negative-irrational properFraction defect or the extreme-scale floating-view
limitation. No Hyper production change follows from this API slice.

Frozen ApiProbe.hs SHA256
986dd265e1ef2363b4c3654becdfd110e3737e39e8bc12dc693ccf99e8a385fc.
Binaries under `.audit-constructible-build.l4UDoe/`:

- api-before-O0:1a4f80a3ebabc3658449b3562296db1969c4a869e1ab2cbed878a60dfa9fb314
- api-before-O2:a897bbe22ea0344c98a2408e43c96430ebda5ddf6c8a41d29c36a44cf0883338

`analyze-api-float.mjs` checks all expected API labels, ordered coverage, source
and binary hashes, exact native floating output, Pell recurrences and all four
directed-oracle result logs. Whole-target/transfer status remains OPEN.
