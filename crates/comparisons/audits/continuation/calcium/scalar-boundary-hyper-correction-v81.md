# Probe API correction

The first audit-only Rust build failed: the available borrowed export is
`to_f64_lossy`, not `as_f64_lossy`, and there are no Real sec_pi/csc_pi methods.
Preserve the original source, origin, formatter/metadata results and failed
build/driver captures. Correct only those three expressions, then bind the
corrected source before rerunning every probe gate.

The forward reciprocal functions are explicitly constructed as 1/cos_pi and
1/sin_pi. The inverse functions remain the disclosed acos(1/x)/asin(1/x)
compositions. Thus both forward and inverse reciprocal layers are derived Hyper
expressions, not claims of absent APIs. No production code is modified and no
representation-independent performance comparison follows from this probe.
