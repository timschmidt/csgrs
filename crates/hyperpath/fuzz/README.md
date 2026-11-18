# Hyperpath fuzzing

The suite covers line, arc, conic, and Bezier arrangements; PCB geometry and
fabrication rules; Specctra import/export; CAM; routing; keepouts; and feed
schedules. `hyperreal_representations` crosses all eight public Hyperreal
structural kinds against each other through translated exact path topology.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```

The `schedule_equivalence` target exercises the public lookahead planner and
certifier on one to six connected line segments. It uses an axis-aligned or
3–4–5 basis, caller feed ceilings, corner radii, a global speed cap and a
constant acceleration limit. At each node it checks the minimum of every
local squared-speed ceiling plus twice the acceleration times the distance
from that ceiling. Taking the minimum over preceding nodes checks the forward
pass. A full-route minimum checks the final schedule. The certifier replays
the planned feeds and must return the same per-row proof statuses. It then
replays one edited feed against independently computed span and corner
residuals. The target repeats these checks on the reversed route.

From the Hyperpath crate directory, run the deterministic matrix and retained
seeds, or start a bounded fuzz campaign. The first corpus directory is
writable so libFuzzer keeps generated inputs outside the retained seeds. The
Linux qualification build uses linker compression to retain its DWARF debug
information within the test executable’s size budget.

```sh
cargo test --locked --test schedule_equivalence
mkdir -p fuzz/corpus/schedule_equivalence
RUSTFLAGS="-C link-arg=-Wl,--compress-debug-sections=zlib" \
  cargo +nightly fuzz run --fuzz-dir fuzz schedule_equivalence \
  fuzz/corpus/schedule_equivalence fuzz/seeds/schedule_equivalence -- -max_len=15 -runs=10000
```
