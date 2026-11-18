# Exact scalar maps on finite dyadic intervals

Hyperinterval revision `f10679ece` implements the finite dyadic `scale` and
`affine` operations required by §3.5 of the interval plan. Both accept scalar
mantissas by reference with signed base-two exponents. Results are exact on
successful completion. Empty input, constants, cancellation, reversed endpoint
order and representability at both exponent limits have public regression tests.

## Endpoint construction

A dyadic scalar has a fixed sign. Multiplying the two source endpoints in the
order selected by that sign gives the exact bounds. If the product exponent
would exceed the stored maximum, the required left shift is reserved on the
factor before multiplication. If it would fall below the minimum, the source
exponent is negative. A normalized nonzero source then has at least one odd
endpoint, so the factor must supply the entire power of two needed to raise the
exponent. A failed divisibility check returns `Overflow`.

Translation consumes the product's owned mantissas. On different grids it
shifts the two endpoints or their single shared offset. The same caller budget
covers exponent repair, products, alignment, and addition. The map
`-3*x + 1` on `[-3,5]` returns `[-14,10]` in 60 work units with a five-bit
limit. A zero offset requires only 50 units. A 122-bit factor at the minimum
grid completes in 172 units because its 120 removable bits are accounted for
before multiplying. The preceding four-product path exhausts the 50-unit
budget on the ordinary scaling case.

## Qualification

The maintained suite compares 2,835 GMP endpoint cases and their compositions,
then exercises extreme exponents, wide cancellation, exhaustion and retry.
Four deliberate faults failed the tests. A standalone consumer separately
checked 32,768 generated images against signed 128-bit arithmetic on native
and in two fresh WASM instances with each of Rust 1.88 and 1.95.

The existing product/add client and the existing inverse-function client
rebuilt byte-for-byte on native and WASM. The native Hypersolve
`enclosed_program` example also matched its preceding executable. An earlier
implementation that reused the inverse-function point constructor introduced
4,056 bytes of RELRO padding in that example. Earlier placements also slowed
an existing zero-offset composition. Keeping the offset as a borrowed scalar
until the final addition removed the dependency on the inverse-function
constructor and the intermediate singleton allocation.

| Final standalone scalar client | Preceding composition | Direct affine method |
| --- | ---: | ---: |
| Native executable, bytes | 552,496 | 543,472 |
| WASM executable, bytes | 104,699 | 95,219 |
| Largest measured requested heap, bytes | 432 | 240 |
| Largest measured whole-call stack, bytes | 2,512 | 1,840 |

The native exception table and read-only data grow by 80 and 32 bytes
respectively in the direct client. Both charges are recorded separately.
The memory comparison comprises 224 fresh processes across seven cases, cold
and warm calls, two stack patterns and two repetitions for each of four
builds. The observer measures from input construction through checked
materialization and destruction on a guarded 2 MiB thread. All measured cases
returned to zero requested bytes at cleanup.

Across three CPUs and seven workloads the direct method took 14–47% of the
time for the equivalent materialized product/add composition, using the median
paired ratio for each workload and CPU. The release timing clients ran without
allocation instrumentation. The measured domains include single- and multiword
inputs. Larger operands, embedded timing and final firmware deployment have
separate qualification requirements.
[Raw measurements and reproduction scripts](../evidence-archive.tsv)
