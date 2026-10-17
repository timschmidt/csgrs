# Guarded circle-support prototype: local evidence complete, admission open

This continuation made progress: it reproduced the E41 rejection, developed an
isolated guard, passed both complete library configurations, and measured every
original and large-coefficient cost case. **The revised index is unadmitted and
has not been applied to main.** The original E38 index remains rejected, as do
the original 35-file native bundle and E36 exact-ingress integration. Previously
independently admitted improvements remain unchanged.

## Contract and implementation

The owning layer is Hypercurve's temporary intersection batch. Batches with at
most eight retained entries use the original ordered equality scan. Larger
batches use the E38 numeric index only when every exact rational coefficient's
existing scalar storage class is eligible. `VeryLarge` coefficients and symbolic
supports keep ordered equality lookup. The scalar hint reads stored bit lengths;
it does not canonicalize, approximate, refine or exclude a supported input.
Earlier unindexed entries keep their first-match priority.

The added semantic test crosses the small-batch threshold with interleaved small
and million-bit coordinates. Independently derived 3-4-5 intersection witnesses,
rebuilt value-equal owners, changed geometry and strict Certified receipts all
pass. The original seven cache contracts and all original fixtures remain.
The three-file proposal under `run/proposal-v1/source` preserves the qualified
production and semantic-test bytes, removing only the two ignored diagnostic
module hooks. It is formatted and retained for review, not installed in main.

## Executed correctness and lint checks

- Eight focused cache contracts pass in both baseline and guarded variants.
- Complete no-feature library: 1,742 passed, zero failed, three ignored.
- Complete all-feature library: 1,749 passed, zero failed, eight ignored.
- Strict direct Clippy (`-D warnings`) passes on both no-feature test harnesses
  and the guarded all-feature test harness. This is library scope, not Cargo's
  full feature/target/manifest matrix.
- The existing ignored cases remain: one long analytic-contact benchmark in
  both configurations and five additional manual drivers with all features.
  The two new diagnostic functions are ignored by ordinary corpus runs and
  explicitly executed by the cost checks.

The first prototype had a collapsible-if lint; a fresh snapshot fixes it. The
first full diagnostic run exposed source-path remapping that prevented proptest
from locating regression files. Its verified live process was stopped after
194 seconds; that incomplete output and the control record are preserved.
Identical source was rebuilt with relative crate paths. An existing generated
property test then passed without persistence warnings in both variants. Both
final complete suites also had no such warnings, no source mutations and no
new regression files. No assertion, fixture, policy or persistence feature was
weakened. The two final functional runs overlapped; timing measurements began
only after both suites and their builds/lints were terminal.

## Same-contract cost evidence

All fifteen one-sample preflight cases completed, followed by seven matched
repetitions: 210 fresh-process samples and 420 cold/warm observations. Every
result matched independently derived exact witnesses and retained a Certified
receipt. The ordinary cases include full ordered witness materialization. The
coefficient cases include the complete Disjoint relation for one cold query and
20,000 warm queries. Both variants use the same diagnostics, compiler flags and
verified dependencies. Input construction, printing and teardown are outside
phase timing and included in complete-process resource observations.

The table reports medians in seconds. Single-query cold timings and other
microsecond/sub-millisecond rows are diagnostic controls; their ratios are not
precise general speed factors. All raw samples, including unfavorable ones,
remain in the [matched record](run/cost-matched-v1/result.json).

| Case / parameter | Baseline cold (s) | Guarded cold (s) | Baseline warm (s) | Guarded warm (s) |
| --- | ---: | ---: | ---: | ---: |
| rational / 1 | 0.000060426 | 0.000082844 | 0.000005960 | 0.000005699 |
| rational / 16 | 0.000151859 | 0.000178978 | 0.000039487 | 0.000032988 |
| rational / 128 | 0.001147231 | 0.000702082 | 0.000792146 | 0.000260982 |
| rational / 512 | 0.011447502 | 0.002151803 | 0.009677744 | 0.000850512 |
| rational / 2048 | 0.160550892 | 0.008299029 | 0.152873200 | 0.003432894 |
| rational / 8192 | 2.448368057 | 0.034932119 | 2.419235670 | 0.014704510 |
| symbolic / 128 | 0.007776746 | 0.008075075 | 0.006501293 | 0.006584627 |
| symbolic / 512 | 0.085618854 | 0.084177193 | 0.080062986 | 0.079333176 |
| mixed / 128 | 0.009425862 | 0.009545704 | 0.008183907 | 0.008303430 |
| mixed / 512 | 0.121417273 | 0.119103085 | 0.117024607 | 0.116150244 |
| superrange / 128 | 0.002686955 | 0.001338368 | 0.001885941 | 0.000634536 |
| subrange / 128 | 0.011769311 | 0.005365041 | 0.007502554 | 0.001718662 |
| shared-coefficient / 64 | 0.000020328 | 0.000034267 | 0.000462968 | 0.000466558 |
| shared-coefficient / 65536 | 0.000023198 | 0.000027608 | 0.000492387 | 0.000476798 |
| shared-coefficient / 1048576 | 0.000034857 | 0.000049587 | 0.000448079 | 0.000479987 |

The million-bit warm median is 0.000479987 seconds versus 0.000448079 seconds for
the baseline. Both seven-sample ranges overlap. The prior roughly 0.78-second
penalty is absent in this matched case; a zero-overhead claim is not established.
The rational N=8192 warm median is 0.014704510 seconds versus 2.419235670 seconds.
These are private-kernel results, not distributed-binary or end-to-end gains.

Unfavorable medians are retained explicitly. Cold rational N=1 and N=16 cost
about 22 and 27 additional microseconds, respectively. Symbolic N=128 cold is
about 3.8% higher, and mixed N=128 cold/warm about 1.3–1.5% higher. Small cold
batches need an amortized completed-operation check before retention if this
cost is material to the affected workload. No representation or output contract
is relaxed to hide those observations.

## Resource and size evidence

Twenty whole-process heap observations cover the seven earlier allocation cases
and all three shared-coefficient cases. Every instrumented witness matches the
uninstrumented exact output. Rational N=2048 allocation calls decrease from
12,706,548 to 158,508, with reported peak heap decreasing from 9.96M to 9.59M.
The single-support and shared-coefficient cases each add one allocation. Mixed
N=128 and superrange N=128 increase reported peak heap by about 7.8K and 14.6K;
these unfavorable values remain in the [heap record](run/heap-v1/result.json).

The matched diagnostic executable changes from 46,946,288 to 46,903,064 bytes;
text from 33,448,816 to 33,432,660; data from 1,305,216 to 1,303,544; BSS from
2,744 to 4,192. This is not a distributed-package size claim. Allocation and
peak figures include input construction, both phases, validation, row encoding,
printing and cleanup. They do not isolate cache-owned heap, establish an internal
memory bound, or make process-end retention a cache-leak finding. Instrumented
time is excluded from latency evidence.

External guards retained 256 MiB on the runtime filesystem, 768 MiB on the
repository filesystem and 2 GiB of available host memory, with 1,800-second and
8-GiB per-command limits. No resource guard stopped the final runs. This does
not establish production total-work, cancellation, coefficient-growth or heap
bounds. Completed targets, sources, helpers, failures and raw output are frozen.
Large Git log copies are losslessly gzip-encoded and checked against the untouched
physical originals; `retention-v1.json` maps every copy to its raw bytes.

## Remaining acceptance boundary

Affected integration/doc/example/bench checks, Cargo's full Clippy combinations,
CSGRS, Hypercircuit and HyperDRC consumer configurations, original native workload
gates, exact full-report comparison and real C/C++/Python/WASM/JS/TS consumers
remain open for this revised implementation. E38/E40/E41 results for different
production bytes do not discharge them. The storage constraint still limits
fresh broad Cargo targets; existing artifacts have not been removed.

Only current_work in the ledger changes. The inherited 27,093 missing artifact
paths remain disclosed; this is not a whole-history preservation claim. The
complete properties plan, clean published-package path, release/MSRV/platform
validation, coverage, directed campaigns, formal verification, documentation,
paper and crate-specific future work remain active.
