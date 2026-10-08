# Hypertri size harness

This dependency-only binary keeps benchmark, fuzz, UI, and differential-oracle
dependencies out of native and WASM artifact measurements.

From the Hypertri repository root:

```sh
./benchmarks/size-harness/measure.sh core
./benchmarks/size-harness/measure.sh earcut
./benchmarks/size-harness/measure.sh cdt
./benchmarks/size-harness/measure.sh nd
./benchmarks/size-harness/measure.sh all
```

Each command reports speed-oriented and size-oriented native and
`wasm32-unknown-unknown` artifacts, including compressed sizes and
`wasm-opt -Oz` when the corresponding tools are installed. The executable
materializes one result from every selected algorithm family so dead-code
elimination cannot turn a feature row into a metadata-only measurement.
