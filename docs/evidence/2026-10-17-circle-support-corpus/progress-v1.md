# Continued curve and native cache qualification

This continuation made progress by executing the acceptance corpus and qualifying
the remaining five test-fixture storage corrections in two files. The index
remains isolated and unadmitted; production implementation is unchanged.
The exact command exits, terminal results and raw output are retained beside this
record. Updating this record does not itself establish implementation acceptance.

| Scope | Passed tests | Failed tests | Existing ignored tests | Filtered out | Failed commands |
| --- | ---: | ---: | ---: | ---: | --- |
| default-corpus-v1 | 1741 | 0 | 2 | 0 | none |
| native-build-v1 | 0 | 0 | 0 | 0 | none |
| native-corpus-v1 | 1651 | 5 | 0 | 0 | library |
| curve-all-v1 | 2485 | 0 | 33 | 0 | none |
| storage-validation-v1 | 40 | 0 | 0 | 3078 | none |
| native-build-v2 | 0 | 0 | 0 | 0 | none |
| native-corpus-v2 | 1559 | 0 | 0 | 0 | none |

The minimal-feature Hypercurve library executable is reused only after source,
feature and binary checks. Its build used --no-default-features; the manifest
declares an empty default feature set. The earlier default-feature shorthand is
clarified in feature-scope-v1.json, without editing frozen protocols. Both
existing ignored tests remain unchanged, including E38's diagnostic cost test.
A fresh runtime source mirror preserves any generated failure artifacts without
modifying the closed E38 source snapshot. No separate default-feature Cargo
invocation or integration/documentation pass is claimed.

The first native library test harness is compiled with the current, independently
admitted temporary-storage repair and the verified E38 candidate dependencies.
Production source, fixtures and policies are otherwise identical to that
candidate. All 1,559 native library tests are selected, including the original
full-board, copper-neck and Gerber gates. Unchanged integration executables and
their compiled CLI path are hash-checked, and documentation tests are executed.
The first run reports five additional hardcoded-/tmp fixture failures. Each
reproduces with the committed-main harness and writable selected storage, without
the index. The repair changes only five test path constructors and removes one
unused test import. Fixture bytes, assertions and production prefixes are
unchanged. All 39 app tests and the Arrow test pass; all-feature/all-target
HyperDRC Clippy is strict, and selected temporary storage is cleaned.

The repaired indexed harness replays the entire native library after the
all-feature curve run and Clippy finish, preserving all three workload gates.
Its unchanged consumer executables and documentation checks are reused from the
first run only after source, production-prefix, dependency and artifact checks.
The original failures remain in the table and raw logs. A successful replay
does not erase them or imply admission of the cache.

The all-feature Hypercurve build uses a fresh Cargo target in repository storage.
Its full library, integration and documentation run keeps the original fixtures,
policies and generated-case settings. Strict Clippy covers all features and
targets. The target is closed after these commands; it may subsequently be read
but never reopened for writes.

The independent all-feature build may overlap the minimal-feature functional corpus.
Both finish before native timing gates begin; the all-feature tests and Clippy
wait for the first native corpus to finish. The independent test-repair builds
may overlap curve functional tests. The second native run waits for all those
checks to become terminal. No speedup is inferred from these runs.
Each executed command has 1,800-second external containment and an 8-GiB process
address-space limit. Repository free space is monitored with a 768-MiB reserve.
These limits do not establish internal work, heap or cancellation bounds.

The original 35-file disposition and the E36 rejection remain unchanged.
The complete E39 CPArti and HVP109A report-byte comparison remains evidence for
the same production proposal, not a claim about the rejected exact-track model.
The inherited 27,093 missing physical paths remain disclosed; this checkpoint
individually pins its own new physical paths.

The cache still needs the remaining affected CSGRS/Hypercircuit configurations,
actual binding consumers and final matched-cost evidence. Default-feature
integration/doc coverage is not inferred from the library run. Any failures in
this checkpoint need investigation without weakening the contract. All other
property-plan and release obligations remain active.
