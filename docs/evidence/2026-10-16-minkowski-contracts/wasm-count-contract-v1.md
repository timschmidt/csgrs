# JavaScript resource-count boundary

WASM owns conversion of JavaScript numbers into native collection limits
(P6/P20/P28/P30). The new `minkowskiSumWithLimits` interface declares counts:
each must be a finite integer from zero through 4,294,967,295. Zero is a valid
limit and may produce ResourceExhausted for a nonempty operation; an invalid
number must instead produce InvalidInput, including when both meshes are empty.
Converting a negative or fractional number to an unsigned integer before
validation can silently enlarge or change the requested resource contract.

Before changing this boundary, run the actual generated Node/WASM module with
negative, fractional, NaN, infinite and out-of-range limits, independently in all
three slots, on both cube and empty operands. Retain all results as a failing
acceptance test if the current wrapper admits or misclassifies them. The native
usize API and mathematical sum need no change: validation belongs before the
JavaScript-number-to-native-count conversion. Keep the representable endpoints,
typed exhaustion, certainty and recovery tests; selecting an approximate policy
does not relax count validity.

After any fix, rerun affected Rust/all-feature, Clippy, README/docs and actual
WASM/TypeScript consumers on the final source. Preserve the passing scalar and
native results when their source is unchanged. Correct the separately recorded
benchmark harness path in the same final qualification. No policy, workload,
fixture resolution or required certified outcome is weakened.
