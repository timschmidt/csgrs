# Software Requirements Specification (SRS)
## IndexedMesh Module - Technical Specifications

### **1. System Overview**
IndexedMesh implements an indexed mesh representation optimizing 3D geometry processing through vertex deduplication and indexed connectivity while maintaining complete API compatibility with the regular Mesh module.

### **2. Functional Requirements**

#### **2.1 Core Data Structures**
- **IndexedMesh<S>**: Main container with vertices, polygons, bounding box, metadata
- **IndexedVertex**: Position + normal with GPU-ready memory layout
- **IndexedPolygon<S>**: Vertex indices + plane + metadata
- **IndexedBSP**: Binary space partitioning for CSG operations

#### **2.2 Shape Generation Functions**
**REQUIREMENT**: Complete parity with regular Mesh shape API

**Basic Primitives**:
- `cube(size, metadata)` ✓
- `cuboid(w, l, h, metadata)` ❌
- `sphere(radius, u_segs, v_segs, metadata)` ✓
- `cylinder(radius, height, segments, metadata)` ✓
- `frustum(r1, r2, height, segments, metadata)` ❌
- `torus(major_r, minor_r, maj_segs, min_segs, metadata)` ❌

**Advanced Shapes**:
- `polyhedron(points, faces, metadata)` ❌
- `octahedron(radius, metadata)` ❌
- `icosahedron(radius, metadata)` ❌
- `ellipsoid(rx, ry, rz, segments, stacks, metadata)` ❌
- `egg(width, length, rev_segs, out_segs, metadata)` ❌
- `teardrop(width, height, rev_segs, shape_segs, metadata)` ❌

**SDF-Based Shapes**:
- `metaballs(balls, resolution, iso_value, padding, metadata)` ✓
- `sdf<F>(sdf, resolution, min_pt, max_pt, iso_value, metadata)` ✓
- `gyroid(resolution, period, iso_value, metadata)` ✓
- `schwarz_p(resolution, period, iso_value, metadata)` ❌
- `schwarz_d(resolution, period, iso_value, metadata)` ❌

#### **2.3 CSG Boolean Operations**
**REQUIREMENT**: Direct indexed BSP operations without conversion

- `union(&other)` - Must use IndexedBSP directly
- `difference(&other)` - Must use IndexedBSP directly  
- `intersection(&other)` - Must use IndexedBSP directly
- `xor(&other)` - Must use IndexedBSP directly

**CRITICAL**: Current hybrid approach (convert→operate→convert) is FORBIDDEN

#### **2.4 Mesh Processing Operations**
- `triangulate()` - Convert to triangular mesh
- `subdivide_triangles(levels)` - Mesh refinement
- `vertices()` - Extract all vertices
- `renormalize()` - Recompute vertex normals
- `validate()` - Comprehensive mesh validation
- `is_manifold()` - Topology validation

#### **2.5 Import/Export Operations**
- `from_polygons(polygons, metadata)` - Create from polygon list
- `to_stl_ascii(name)` - ASCII STL export
- `to_stl_binary(name)` - Binary STL export
- `from_stl(data)` - STL import
- `to_bevy_mesh()` - Bevy integration
- `to_trimesh()` - Parry integration

### **3. Non-Functional Requirements**

#### **3.1 Performance Requirements**
- Memory usage ≤50% of equivalent regular Mesh
- CSG operations 2-3x faster than regular Mesh
- Zero-copy operations for vertex/index buffer generation
- Iterator-based processing for vectorization

#### **3.2 Quality Requirements**
- 100% test coverage for critical paths
- Comprehensive edge case handling
- Robust error recovery with Result types
- Thread-safe operations for parallel processing

#### **3.3 Design Requirements**
- SOLID design principles compliance
- Zero-cost abstractions preference
- Iterator combinators for performance
- Minimal memory allocations

### **4. Interface Requirements**

#### **4.1 CSG Trait Implementation**
```rust
impl<S: Clone + Send + Sync + Debug> CSG for IndexedMesh<S> {
    fn new() -> Self;
    fn union(&self, other: &Self) -> Self;
    fn difference(&self, other: &Self) -> Self;
    fn intersection(&self, other: &Self) -> Self;
    fn xor(&self, other: &Self) -> Self;
    fn transform(&self, matrix: &Matrix4<Real>) -> Self;
    fn inverse(&self) -> Self;
    fn bounding_box(&self) -> Aabb;
    fn invalidate_bounding_box(&mut self);
}
```

#### **4.2 API Compatibility Matrix**
| Method | Regular Mesh | IndexedMesh | Status |
|--------|-------------|-------------|---------|
| cube() | ✓ | ✓ | Complete |
| sphere() | ✓ | ✓ | Complete |
| cylinder() | ✓ | ✓ | Complete |
| torus() | ✓ | ❌ | Missing |
| union() | ✓ | ⚠️ | Hybrid approach |
| triangulate() | ✓ | ✓ | Complete |
| to_stl_ascii() | ✓ | ❌ | Missing |

### **5. Validation Requirements**

#### **5.1 Test Coverage Requirements**
- Unit tests for all shape generation functions
- Integration tests for CSG operations
- Performance benchmarks vs regular Mesh
- Memory usage validation tests
- Edge case and error condition tests

#### **5.2 Acceptance Criteria**
- All existing regular Mesh tests pass with IndexedMesh
- Performance targets met in benchmark tests
- Memory usage targets validated
- No breaking changes to existing API
- Comprehensive documentation coverage
