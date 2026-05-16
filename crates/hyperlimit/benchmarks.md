# Benchmarks

This file is generated from Criterion output under `target/criterion`.

Generated at Unix timestamp `1778930716`.

## Commands

Run the default benchmark suite and update this file:

```sh
cargo bench --bench predicates
```

Run dispatch tracing separately and update `dispatch_trace.md`:

```sh
cargo bench --bench predicates --features dispatch-trace -- --write-dispatch-trace-md
```

Regenerate this file from existing Criterion output:

```sh
cargo run --example write_benchmarks_md
```

Open Criterion's detailed HTML report at `target/criterion/report/index.html`.

## Latest Results

No Criterion results were found. Run `cargo bench --bench predicates` first.
