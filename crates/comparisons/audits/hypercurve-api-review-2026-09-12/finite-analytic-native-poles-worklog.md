# Native discovery is optional for finite analytic pairs

Status: committed Hypercurve `8228394f174be2ae607694682defb167f030f540`; qualified within the recorded scope.

Parent Hypercurve is `b518e5ff82ce2e6dd80c426aac0cb53236047d05`. HyperBREP and the five other dependencies remain pinned to the same committed-source snapshots used in the preceding finite pair qualification. Hyperreal work owned by the other session remains untouched. No globally clean workspace claim is made.

## Executed baseline

The public fixture has rational source P(t)=(-t/(1-2t),-t/(1-2t)) on [0,1/4], with its excluded native pole at t=1/2. The equivalent whole-native chart P(u/4) uses controls (0,0),(-1/2,-1/2) and weights 1,1/2. For zero displacement, the horizontal y=-1/6 contact is independently (-1/6,-1/6) at t=1/8. For left displacement sqrt(2)/4, the source tangent points down-left and its normal adds (1/4,-1/4), so the horizontal y=-5/12 contact is (1/12,-5/12). The two charts describe exactly the same finite geometry.

The matrix covers both charts, zero/nonzero displacement, an ordinary rational or analytic horizontal operand, both reversals, both orders and both policies. On the committed parent it executes all 128 cases: 96 pass, while all 32 displaced cases on the pole-bearing source return incomplete Boundary. The compact equivalent charts all succeed. It is a completeness failure, not an observed wrong certified contact. Public point and location replay are checked in every successful case. The baseline record is `finite-analytic-native-poles-baseline.json`; source SHA256 is `ec10467bfcc46c94a0722084d1fbd30bb13be43bd643af4f9c7405a407a9f655`. The first compile-only fixture attempted private convenience constructors; its source and compiler output are retained separately in `finite-analytic-native-poles-compile-attempt1/`. No runtime claim is based on that attempt.

## Change

The shared Pair dispatcher uses complete native evidence when available. A native worker returning uncertainty or incomplete replay falls through to the existing finite-domain authority, for both parallel/rational and parallel/parallel pairs. Query containment in [0,1] no longer makes failure of whole-source discovery authoritative for a valid retained interval. Invalid-input errors still propagate. Native one-sided frame workers and complete supported fast paths remain; no new solver, representation or compatibility interface is introduced.

A unit regression exercises the independent public geometry and checks transversality, exact contact, original-chart location replay, absence of spurious components, both policies, both charts, both operand forms, reversal and order. Six focused tests also cover the preceding exterior-contact, overlap and constant-image evidence migrations. The existing 33 public matrices are retained along with the new 128-case matrix, for 34 total.

This is a bounded repair to optional native discovery. It does not complete one-sided source-cusp frames in the finite solver, selected-circle analytic domains, selected-point finite incidence, normalized public region construction, independent inverse replay, known failures/exclusions or the broader algebraic/computational architecture. Repeating a failed native solve before finite fallback has an unmeasured cost on formerly unsupported inputs; no general performance or memory claim is made.

## Qualification protocol

The working/isolated sources are frozen while any owned build/test/probe is live. Builds use `/tmp/hypercurve-native-poles-qualification` with one-file overlay on the committed parent and pinned dependencies. The final writer must bind the 396 working files, 1,005 isolated files, toolchain, libraries, test executables and public source/executable hashes. The broad scope is 49 targets with the five previously recorded failures, nine ignored tests and eight expensive exclusions identified explicitly. A successful qualification writer must not be rerun on changed sources.

## Focused assertion audit

The first candidate passed all four builds and all 34 public matrices, including the repaired 128-case probe, but failed one of six focused tests because the new assertion required every contact to retain an eager transverse certificate. The candidate and matching source manifests, libraries, libtest and logs are archived in `finite-analytic-native-poles-attempt1/`; no broad suite or successful qualification was attributed to it.

A separate public probe was compiled against both the archived parent and candidate libraries. The parent already reports transverse=false and no tangent cross sign for 32 zero-distance cases, and the candidate preserves exactly those 32 cases. Of the parent's 96 successful contacts, 40 have a transverse certificate without a cross sign and 24 have both. The candidate adds 32 repaired displaced contacts, all with a transverse certificate and signed cross product (16 each sign). This is recorded in `finite-analytic-native-poles-transverse-evidence.json` with independent source/executable/library hashes. The optional native evidence is not a certified tangency claim.

The final test therefore requires the transverse certificate for displaced contacts, checks every retained cross sign against the independently known orientation and traversal/order parity, and retains exact contact/location assertions for all zero-distance cases. Production code is unchanged from the first candidate. If rebuilding this test-only edit leaves the normal library byte-identical, the 34 already executed public matrices remain applicable to that artifact and will not be needlessly rerun; final qualification verifies each library, source and executable hash. Focused and broad tests run against the new libtest.

The test-only rebuild changed the normal library hash from `1009979d5810efa1b84886db7d49eff7a84d8c2de7860bca7d90557645fe4b3b` to `80e20958e5d3b5d992cd8a64349c236522867b1c89147c36fd889e417c5c85ba`. All 34 previous public executions, including matching sources, executable copies and hashes, are archived in `finite-analytic-native-poles-attempt1/public/`. The final 34-matrix run is therefore repeated against the new artifact; no reuse claim is made for the prior executable set.

## Final qualification

All 49 broad targets finish with **2,294 passes** (2,062 Hypercurve, 232 HyperBREP), five unchanged known failures, nine ignored tests, eight prior expensive exclusions, and no new failures or broad-run timeouts. All six focused tests and all 34 public probes/matrices pass on the final artifacts. Both repositories pass all-target no-default checks and all-feature release test builds. Formatting, git diff checking, 396 working-source hashes and 1,005 isolated-source hashes verify. The normal library SHA256 is `80e20958e5d3b5d992cd8a64349c236522867b1c89147c36fd889e417c5c85ba`; libtest SHA256 is `5d9907e5903b14ac2c3fc3116582ecaa60f2520e46a58c1d670ea316533f3c1e`. Matching artifacts are archived in `finite-analytic-native-poles-libraries/`.

The architecture goal remains active. The next bounded selected-circle finite-domain audit and independently derived public fillet fixture are in `finite-circle-analytic-domains-next.md`; those new circle fixtures have not yet been executed.
