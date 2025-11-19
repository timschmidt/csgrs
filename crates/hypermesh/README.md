# boolmesh
![demo](https://raw.githubusercontent.com/komietty/boolmesh/main/examples/docs/demo.png)
Boolmesh is a pure Rust library for performing robust and efficient mesh boolean operations.
It is a full-from-scratch Rust implementation inspired by [Elalish’s Manifold](https://manifoldcad.org/docs/html/classmanifold_1_1_manifold.html), well known for its robustness and now part of OpenSCAD.

The codebase is clean and minimal, with dependencies only on `anyhow` and `nalgebra`.
Besides being robust, Boolmesh is also very fast — for example, generating a Menger Sponge of depth 4 (the model shown on the right) takes only around 8 seconds on an Apple Silicon M4, without multi-threading.

## Usage
The usage is intentionally simple, as the library exposes only one main function for end users.
To perform a boolean operation, construct a mesh buffer structure (called a `Manifold`) from vertex positions and face indices, then call `compute_boolean()` to obtain the result.

Note: Input meshes must be manifold, meaning they must not contain boundaries or overlapping geometry.

``` rust  
let mfd_0 = Manifold::new(&positions_0, &indices_0).unwrap();    
let mfd_1 = Manifold::new(&positions_1, &indices_1).unwrap(); 
let result: Manifold = compute_boolean(&mfd_0, &mfd_1, OpType::Subtruct).unwrap();
```

Examples such as a Menger Sponge generator and simple mesh boolean samples can be found in the examples folder.

## Roadmap
Planned upcoming implementations include:
- Multithreading support for improved performance
- A CSG tree structure for pre-computation optimization
- UV value and mesh ordering inheritance for output meshes

## LICENSE
Mozilla Public License Version 2.0 (MPL-2.0)