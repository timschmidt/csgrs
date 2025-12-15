# boolmesh

[![License: MPL-2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Crates.io](https://img.shields.io/crates/v/boolmesh.svg)](https://crates.io/crates/boolmesh)
[![Docs.rs](https://img.shields.io/docsrs/boolmesh)](https://docs.rs/boolmesh)

![demo](https://raw.githubusercontent.com/komietty/boolmesh/main/examples/docs/demo.png)
Boolmesh is a pure Rust library for performing robust and efficient mesh boolean operations. It is a full-from-scratch Rust implementation inspired by [Elalish’s Manifold](https://manifoldcad.org/docs/html/classmanifold_1_1_manifold.html), which is well known for its robustness and is now part of OpenSCAD.

The codebase is clean and minimal, with dependencies only on `glam`—which provides SIMD acceleration—and optionally on `rayon` for multi-threading support. Besides being robust, Boolmesh is also very fast. For example, generating a Menger Sponge of depth 4 (the model shown on the right) takes only around 8 seconds on an Apple Silicon M4 using single-threading, and around 4 seconds with multi-threading enabled. Also, the library supports both 32-bit and 64-bit floating point arithmetic.

## Usage
The usage is intentionally simple, as the library exposes only one main function for end users. To perform a boolean operation, construct a mesh buffer structure (called a `Manifold`) from vertex positions and face indices, then call `compute_boolean()` to obtain the result.

Note: Input meshes must be manifold, meaning they must not contain boundaries or overlapping geometry.

``` rust  
let mfd_0 = Manifold::new(&positions_0, &indices_0).unwrap();    
let mfd_1 = Manifold::new(&positions_1, &indices_1).unwrap(); 
let result: Manifold = compute_boolean(&mfd_0, &mfd_1, OpType::Subtract).unwrap();
```

Examples such as a Menger Sponge generator and simple mesh boolean samples can be found in the examples folder.

```
 cargo run --package boolmesh --release --example menger_sponge --features=bevy,rayon,f32
```

## Roadmap
Planned upcoming implementations include:
- More multi-threading optimizations 
- A CSG tree structure for pre-computation optimization
- UV value and mesh ordering inheritance for output meshes

## LICENSE
Mozilla Public License Version 2.0 (MPL-2.0)