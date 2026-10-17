# Fresh full-library qualification within the remaining quota

The candidate executable and nine focused contracts passed. This additional
scope includes both entire library configurations, without new filtering:
no features, and the six features selected by the frozen Cargo all-feature
build. Each uses a new source mirror to preserve property-test persistence.
All-feature compilation reuses only verified, closed dependency artifacts;
no completed Cargo target is reopened. The complete source, compiler, flags,
features and dependency hashes remain recorded by the inherited helper.

Prior physical outputs consumed about 27.3 MB for the no-feature run and
75.4 MB for the all-feature binary plus mirror. The current quota headroom
allows those outputs alongside the preserved measurement matrix and allocation
traces while maintaining the already declared 256 MiB quota reserve. Run the
same quota-aware supervisor for every command; stop with retained diagnostics
if the reserve is breached. The root reserve stays 512 MiB, host available
memory 2 GiB, each command's address-space limit 8 GiB and wall limit 1,800 s.
No ordinary native workload limit or fixture changes.

The independent functional library runs may overlap after the uninstrumented
cost matrix completes. No latency measurements overlap compilation, library
tests or allocation instrumentation. Run the final shared-support comparison
only after both library jobs and all other qualification jobs are terminal.
Full library passes do not stand in for integration tests, documentation,
examples, Cargo Clippy combinations, downstream, native reports, bindings or
the clean external distribution path. These remain separate admission gates.
