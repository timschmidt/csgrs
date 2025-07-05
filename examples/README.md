# CSG Examples

This directory contains examples demonstrating various CSG (Constructive Solid Geometry) operations using the csgrs library.

## Boolean Operations Between Cube and Sphere

These examples demonstrate all fundamental boolean operations between cube and sphere primitives, showcasing the power of CSG for creating complex geometries from simple shapes.

### cube_sphere_union.rs - Union Operation (A ∪ B)

Demonstrates **union** - combining two objects into one containing all space from both.

**What it creates:**
- 40×40×40mm cube centered at origin
- 25mm radius sphere offset to create partial overlap
- Combined geometry containing all space from both objects

**Key concepts:**
- Union is commutative: A ∪ B = B ∪ A
- Result encompasses both input objects
- Creates smooth blended geometry

### cube_sphere_difference.rs - Difference Operation (A - B)

Demonstrates **difference** - subtracting one object from another.

**What it creates:**
- 50×50×50mm cube with spherical cavity
- 20mm radius sphere positioned to intersect cube
- Cube with spherical "bite" taken out

**Key concepts:**
- Difference is NOT commutative: A - B ≠ B - A
- Creates cavities and cutouts
- Result bounded by first object

### sphere_cube_difference.rs - Reverse Difference (B - A)

Demonstrates **difference in reverse** - subtracting cube from sphere.

**What it creates:**
- 30mm radius sphere with cubic cavity
- 35mm cube positioned to intersect sphere  
- Spherical shell with cubic "bite" taken out

**Key concepts:**
- Shows non-commutative nature of difference
- Creates complex hollow geometries
- Useful for architectural/shell structures

### cube_sphere_intersection.rs - Intersection Operation (A ∩ B)

Demonstrates **intersection** - keeping only overlapping space.

**What it creates:**
- Only the space that exists in BOTH cube AND sphere
- Hybrid geometry with flat and curved faces
- Smaller than either input object

**Key concepts:**
- Intersection is commutative: A ∩ B = B ∩ A
- Result is bounded by both input objects
- Creates unique hybrid geometries

### cube_sphere_xor.rs - XOR/Symmetric Difference (A ⊕ B)

Demonstrates **XOR** - space in either object but NOT in both.

**What it creates:**
- "Donut" or "shell" effect geometry
- Hollow structure with cavity where objects overlapped
- Mathematically: (A ∪ B) - (A ∩ B)

**Key concepts:**
- XOR is commutative: A ⊕ B = B ⊕ A
- Creates hollow/shell structures
- Equivalent to union minus intersection

## File Format Export Examples

### multi_format_export.rs - Multi-Format Export

Demonstrates **multi-format file export** - converting CSG objects to widely-supported 3D file formats including OBJ, PLY, and AMF.

**What it creates:**
- 7 OBJ files, 7 PLY files, and 7 AMF files showcasing various CSG operations
- Basic primitives: cube, sphere, cylinder
- Complex operations: boolean combinations and drilling
- Triple-format output for maximum compatibility

**Key features:**
- **Triple-format support**: OBJ (universal), PLY (research), and AMF (3D printing) formats
- **Universal compatibility**: Files open in most 3D software and 3D printers
- **Mesh statistics**: Displays vertex, face, and triangle counts for all formats
- **Proper formatting**: Includes vertices, normals, face definitions, and XML structure
- **Metadata support**: Generated with proper headers, comments, and manufacturing info

**Supported software:**
- **3D Modeling**: Blender, Maya, 3ds Max, Cinema 4D
- **CAD Programs**: AutoCAD, SolidWorks, Fusion 360, FreeCAD
- **Analysis Tools**: MeshLab, CloudCompare, ParaView
- **Research Tools**: Open3D, PCL, VTK-based applications
- **Game Engines**: Unity, Unreal Engine, Godot
- **Online Viewers**: Many web-based 3D viewers

**Technical details:**
- **OBJ format**: ASCII, triangulated meshes, 1-indexed vertices, separate normals
- **PLY format**: ASCII, triangulated meshes, vertex+normal data, research-oriented
- **AMF format**: XML-based, triangulated meshes, metadata support, 3D printing optimized
- Vertex deduplication for optimized file size
- Normal vectors for proper shading and analysis
- Color/material support (AMF)
- Comprehensive format validation and testing

## Basic Primitive Example

### cube_with_hole.rs

This example demonstrates creating a rectangular cube with a cylindrical hole drilled through it using CSG difference operations.

**What it creates:**
- A rectangular cube with dimensions 127×85×44mm
- A cylindrical hole with 6mm diameter
- The hole travels through the entire 127mm length (X-axis)
- The hole is centered in the 85×44mm cross-section (Y=42.5mm, Z=22.0mm)

**Key CSG operations demonstrated:**
1. **`CSG::cuboid()`** - Creating a rectangular box primitive
2. **`CSG::cylinder()`** - Creating a cylindrical primitive
3. **`.rotate()`** - Rotating geometry (cylinder from Z-axis to X-axis)
4. **`.translate()`** - Positioning geometry in 3D space
5. **`.difference()`** - Boolean subtraction operation
6. **`.to_stl_binary()`** - Exporting results to STL format

## Running the Examples

### Individual examples:
```bash
# Basic cube with hole
cargo run --example cube_with_hole

# Boolean operations
cargo run --example cube_sphere_union
cargo run --example cube_sphere_difference  
cargo run --example sphere_cube_difference
cargo run --example cube_sphere_intersection
cargo run --example cube_sphere_xor

# File format export
cargo run --example multi_format_export
```

### Running tests:
```bash
# Test individual examples
cargo test --example cube_with_hole
cargo test --example cube_sphere_union
cargo test --example multi_format_export
# ... etc for other examples

# Test all examples
cargo test --examples
```

## Output Files

Each example creates output files demonstrating the operations:

**STL Files (3D printing format):**
- `cube_with_hole.stl` - Cube with cylindrical hole
- `cube_sphere_union.stl` - Combined cube and sphere
- `cube_sphere_difference.stl` - Cube with spherical cavity
- `sphere_cube_difference.stl` - Sphere with cubic cavity  
- `cube_sphere_intersection.stl` - Overlapping region only
- `cube_sphere_xor.stl` - Hollow shell structure

**OBJ Files (universal 3D format):**
- `cube.obj` - Basic cube primitive (8 vertices, 12 faces)
- `sphere.obj` - High-resolution sphere (482 vertices, 960 faces)
- `cylinder.obj` - Cylindrical primitive (50 vertices, 96 faces)
- `cube_with_cavity.obj` - Complex boolean difference (370 vertices, 574 faces)
- `cube_sphere_union.obj` - Union operation (219 vertices, 379 faces)
- `cube_sphere_intersection.obj` - Intersection operation (159 vertices, 314 faces)
- `cube_with_hole.obj` - Drilling operation (57 vertices, 78 faces)

**AMF Files (3D printing format):**
- `cube.amf` - Basic cube primitive (8 vertices, 12 triangles, 3.1KB XML)
- `sphere.amf` - High-detail spherical mesh (482 vertices, 960 triangles, 198KB XML)
- `cylinder.amf` - Cylindrical primitive (50 vertices, 96 triangles, 20KB XML)
- `cube_with_cavity.amf` - Boolean difference (370 vertices, 574 triangles, 133KB XML)
- `cube_sphere_union.amf` - Union operation (219 vertices, 379 triangles, 83KB XML)
- `cube_sphere_intersection.amf` - Intersection operation (159 vertices, 314 triangles, 65KB XML)
- `cube_with_hole.amf` - Complex drilling operation (57 vertices, 78 triangles, 19KB XML)

All files can be opened in 3D modeling software, CAD programs, 3D printing slicers, or online viewers.

## Mathematical Relationships

The examples also demonstrate important boolean algebra relationships:

- **Commutative**: Union and Intersection are commutative
- **Non-commutative**: Difference is not commutative  
- **Identity**: XOR = (A ∪ B) - (A ∩ B) = (A - B) ∪ (B - A)
- **Verification**: Examples include tests validating these mathematical properties

## Technical Implementation Details

- **Sphere Parameters**: All spheres use `(radius, segments, stacks)` format
- **Surface Quality**: Examples use 32 segments for smooth surfaces in main code, 16/8 in tests for speed
- **Positioning**: Strategic offsets create meaningful overlaps for demonstration
- **Testing**: Comprehensive unit tests validate geometric properties and mathematical relationships
- **Multi-format Export**: STL (binary), OBJ (ASCII), PLY (research), and AMF (3D printing) formats 
- **File Statistics**: Examples display mesh complexity (vertex/face/triangle counts) for analysis
- **3D Printing Ready**: AMF format includes manufacturing metadata and material support 
