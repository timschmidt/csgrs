# Fresh physical validation and public format failures

The new physical build completes 10 of 12 validation commands successfully.
Both original native timing gates still fail, so the 35 pending native files
remain unadmitted. No main production source changes are made in this phase.
The [decision](decision-v1.json) and [checkpoint](checkpoint-v1.json) retain
scope, source pins, commands and artifact expectations.

## Requalification scope

The source baseline is `3b69af324a9642d1f71b9cc38de58b59a236f740` plus the unchanged
35 pending native files. The metadata-order candidate is excluded. Cargo targets
start empty on direct physical tmpfs; no nested overlay or inherited build seed
is used. Eight path-identity reads, hard-link replacement/truncation and direct
executable loading passed in the bounded [storage check](storage-v1.json).
That check does not prove a filesystem correct for every operation.

The [initial protocol](validation-v1/protocol.json) pins 2,823 copied source inputs,
dependency locks, original archive hashes, rustc 1.98.1, Cargo, build settings and
the AMD Ryzen 7 5800X3D host. Commands use release builds, offline dependencies,
two build jobs, a 1,800-second external elapsed limit and an 8-GiB per-process
address-space limit. These are external containment, not internal work, heap-byte
or cancellation guarantees.

Before board extraction, `/tmp` quota had about 45 MiB available while the two
archives expand to about 49 MiB. The supervisor was stopped while its current
curve-all command continued uninterrupted. Once that command exited 0, its
original GNU time measurement was adopted and only the stopped supervisor was
terminated (exit 143). The [continuation](continuation-v1.md) uses dedicated
physical scratch space. No completed target or used runner was changed, no test
was restarted, and supervisor termination is not a test failure.

## Executed checks

[Combined command records](validation-v2/commands.json) locate each raw log and
retained executable. Counts overlap across configurations and are not unique
coverage totals. Existing ignored cases remain ignored; no new ignore or fixture
reduction was introduced.

| Command | Exit | Reported test results |
| --- | ---: | --- |
| format | 0 | command status only |
| root-all | 0 | 405 passed, 0 failed, 1 ignored |
| root-minimal | 0 | 132 passed, 0 failed, 0 ignored |
| curve-default | 0 | 2460 passed, 0 failed, 28 ignored |
| curve-all | 0 | 2497 passed, 0 failed, 33 ignored |
| strict-clippy | 0 | command status only |
| native-other-tests | 0 | 1643 passed, 0 failed, 0 ignored |
| docs | 0 | command status only |
| full-board | 101 | 0 passed, 1 failed, 0 ignored |
| copper-neck | 101 | 0 passed, 1 failed, 0 ignored |
| gerber | 0 | 1 passed, 0 failed, 0 ignored |
| native-cli | 0 | command status only |

The full-board fixture reports **191.224524234 seconds of worker CPU**, exceeding its
unchanged **120-second** limit. The copper-neck check body reports **86.843039677
seconds of wall time**, exceeding **60 seconds**. Whole-command elapsed times
also include input construction and must not replace those assertion metrics.
The Gerber gate passes. No cost comparison against the unqualified historical
overlay run or application speedup is claimed.

## Complete report replay

Two fresh CLI processes used the same pinned physical input paths as the previous
report capture. All complete bytes and parsed JSON agree without normalization;
exit 1 denotes active findings. This supplies current-source report artifacts,
not an independent geometry oracle or retroactive compiler-input qualification
for the historical run.

| Fixture | Bytes | Active findings | SHA-256 |
| --- | ---: | ---: | --- |
| cparti-fpga | 147297 | 12 | `164102f85d103de8930e871318107fe17e51f3f478aa29a34e6eb9a43ea1d2c2` |
| hvp109a | 128420 | 197 | `d73ffb1c463a28abf1ab32740213ca3feb5c991f7b34ac506dab5c4af424875b` |

The [replay protocol](report-replay-v1/protocol.json), [commands](report-replay-v1/commands.json)
and compressed full outputs preserve the comparison. Report execution was not
used as a matched performance measurement.

## Reproduced format defects

The unchanged [twelve-test public probe](format_probe_v1.rs) produced four passes
and eight failures in a separate initially empty physical target. It demonstrates:

- OBJ export accepts missing indices and `usize::MAX`; its stream wrapper writes
  malformed geometry instead of failing before writing (three failures).
- Fixed decimal formatting erases nonzero binary64-projected tiny coordinates
  in OBJ and ASCII STL (two failures).
- Valid binary32-representable cubes of side 2^-12 fail STL reimport in both
  formats because the imported mesh is subjected to an absolute epsilon area
  cutoff (two failures).
- Binary STL export succeeds after binary32 projection collapses faces of a
  translated unit cube (one failure).

Canonical empty and unit-cube controls pass. Their semantic checks distinguish
strict numerical certainty, closed PWN validity, manifoldness, box boundary
planes, exact oriented volume and subsequent Boolean composition. The tiny text
tests check preservation of binary64 projections; they do not certify exact
source intent or a general lossless interchange contract.

The supplemental [STL topology probe](stl-topology-v1/probe_v1.rs) records two more
failures: both import paths accept one duplicated directed cube face with three
unmatched directed edges. Reversed-face rejection and balanced doubled-winding
volume controls pass. Native topology validation rejects the malformed source
fixture; a format-specific edge map that overwrites repeated directed keys loses
the required multiplicity. These four tests are additional evidence, not
replacements for the original twelve or full-scale fixtures.

No format fix is applied here. The next coherent change is the reproduced OBJ
index correction through Hypermesh's existing resolver. The original format
probe and all remaining failures must remain visible during that qualification.

## Preservation and remaining work

The checkpoint hashes each new physical path independently and retains the
executable byte sequences used by the commands. It adds expectations for the
fresh build and separate STL reproduction namespaces to the parent manifest.
The 27,093 inherited missing paths and known nested-overlay identity failures
remain recorded. Historical artifacts are not re-audited here; there is no
whole-history completeness, zero-loss or zero-mismatch claim.

Only the `current_work` value in the status ledger changes. All bytes outside
that value and the other decoded keys remain bound to revision `9025de62`.
The complete pre-push workflow matrix, published-package consumer pipeline,
release, broad format provenance/resource contracts, coverage and formal work
remain open. No push or native-runtime admission is part of this checkpoint.
The full properties plan remains active and incomplete.
