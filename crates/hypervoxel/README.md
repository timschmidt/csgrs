<h1 align="center">
  <img src="https://raw.githubusercontent.com/WildPixelGames/voxelis/master/docs/voxelis_logo.png"
       alt="Voxelis" width="480"><br>
  <sub>Tiny voxels. Huge worlds. Zero hassle.</sub>
</h1>

<p align="center">
  <a href="https://crates.io/crates/voxelis"><img src="https://img.shields.io/crates/v/voxelis.svg?style=for-the-badge&color=brightgreen"></a>
  <a href="https://docs.rs/voxelis"><img src="https://img.shields.io/badge/docs‑rs-online-blue.svg?style=for-the-badge"></a>
  <a href="LICENSE"><img src="https://img.shields.io/github/license/WildPixelGames/voxelis?style=for-the-badge"></a>
</p>

> **"Need voxels? Reach for Voxelis."**
> Powered by VoxTree — a deliciously crafted SVO DAG with batching. Drop it into your Rust, C++, Godot, or Bevy project and start carving worlds down to centimetre-level detail — while your memory bill stays shockingly low.

---

## 🚀 Why Voxelis?

* **Tiny voxels** (4 cm resolution) without melting your RAM.
* **Shared memory** — DAG compression at 99.999% ratio.
* **Batch edits** — mutate hundreds of thousands of voxels in a blink.
* **Zero garbage collection** — just deterministic reference counting with generations.
* **Built for gamedev** — chunk grids, paging hooks, multithread-ready.
* **Fearless Rust** — no UB, no data races, only pure, undiluted speed.

> This isn't just another voxel crate. It's a foundation for colossal, high-fidelity worlds.

---

## ✨ Benchmarks: "Speed so fast, it hurts."

| Operation                       | 32³ Voxels | Single Op   | Batch Op    | Notes |
|----------------------------------|-------------|-------------|-------------|-------|
| **fill()**                      | 32K         | **9 ns**    | **10.6 ns** | ⚡ Single leaf collapse |
| **set_uniform()**               | 32K         | **5.17 ms** | **23.1 μs** | 🚀 ~224× faster |
| **set_checkerboard()**          | 32K         | **2.52 ms** | **116.6 μs** | 🚀 ~22× faster |
| **set_sum() (high-entropy)**     | 32K         | **5.92 ms** | **194.1 μs** | 🌪️ Complex pattern, ~30× faster |
| **perlin dunes (high-entropy)**  | 32K         | -           | **~12 μs**  | 🌎 ~1380 chunks/frame (60 FPS) |

> Full raw results? Check [benches-raw.md](https://github.com/WildPixelGames/voxelis/blob/master/docs/benches-raw.md).
>
> Summary tables? See [benches-tables.md](https://github.com/WildPixelGames/voxelis/blob/master/docs/benches-tables.md).
>
> Full commentary and insights? Dive into [benches.md](https://github.com/WildPixelGames/voxelis/blob/master/docs/benches.md).

---

## 🔧 Quick Start

```rust
use voxelis::{VoxTree, VoxInterner};
use glam::IVec3;

let mut interner = VoxInterner::<u8>::with_memory_budget(256 * 1024 * 1024);
let mut tree = VoxTree::new(5); // 32³ voxels (chunk)

let mut batch = tree.create_batch();
batch.fill(&mut interner, 0); // air
batch.set(&mut interner, IVec3::new(3,0,4), 1); // stone

tree.apply_batch(&mut interner, &batch)?;
assert_eq!(tree.get(&interner, IVec3::new(3,0,4)), Some(1));
```

Add via Cargo:

```bash
cargo add voxelis # Requires Rust 1.86+, optionally use `wide` for SIMD meshing
```

---

## 🔍 Under the Hood

| Concept | Purpose |
|:--------|:--------|
| **VoxTree** | The SVO-DAG — compressed octree core. |
| **VoxInterner** | Shared memory for leaves/branches. Hash-consed. |
| **Batch** | Bottom-up batched editing — mutate at light speed. |
| **VoxOps** | Trait for per-voxel manipulation — set, get, fill, clear. |
| **BlockId** | 64-bit magic to encode voxel state compactly. |
| **Mesher** | SIMD greedy meshing (WIP) — turn voxels into worlds. |

More? Crack open **The Voxelis Bible** ([docs/The Voxelis Bible_ From Pixels to Worlds - An In-Depth Guide v2.3.pdf](https://github.com/WildPixelGames/voxelis/blob/master/docs/The%20Voxelis%20Bible_%20From%20Pixels%20to%20Worlds%20-%20An%20In-Depth%20Guide%20v2.3.pdf)) — 38 pages of dangerously concentrated nerdery.

---

## 🌏 Roadmap: Into the Voxelverse

* Multithreaded interner with Rayon.
* GreedyMesh v2 (distance-field LOD magic).
* GPU frustum-culling traversals.
* OBJ / glTF import/export.
* True out-of-core paging (MMAP + LRU).

> PRs welcome. Bonus points if your patch makes the CI bot 🐈‍🔄 purr.

---

## 🚡 Run Your Own Benches

```bash
cargo bench -p voxelis_bench
```

Hardware: Apple M3 Max, Rust 1.86 stable, `-C target-cpu=native`, final profile.

Want real numbers? We've got them — [benches-tables.md](https://github.com/WildPixelGames/voxelis/blob/master/docs/benches-tables.md) and [benches-raw.md](https://github.com/WildPixelGames/voxelis/blob/master/docs/benches-raw.md) await.

---

## 👍 Contributing

1. Fork and branch (`feat/my-magnificent-contribution`).
2. `cargo test && cargo bench`
3. Open PR, include new benchmark delta.
4. Bask in voxel-induced glory.

---

## 🌐 License

Dual licensed under MIT / Apache-2.0.
Pick your poison, build something massive.

---

## ⚠️ Warning

Voxelis may cause extreme enthusiasm, uncontrollable world-building, and compulsive Rust evangelism. Consult your GPU before operating heavy voxel engines. 😜
