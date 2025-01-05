# csg.rs

Constructive Solid Geometry (CSG) is a modeling technique that uses Boolean operations like union and intersection to combine 3D solids. This library implements CSG operations on meshes elegantly and concisely using BSP trees, and is meant to serve as an easily understandable implementation of the algorithm. All edge cases involving overlapping coplanar polygons in both solids are correctly handled.

![Example CSG output](docs/csg.png)

Construct a shape:

    let cube = csgrs::cube(None);
    let sphere = csgrs::sphere(None);
    let cylinder = csgrs::cylinder(None);

Combine shapes with:

    let union_result = cube.union(&sphere);
    let subtraction_result = cube.subtract(&sphere);
    let intersection_result = cylinder.intersect(&sphere);

Extract polygons:

    let polygons = union_result.to_polygons();
    println!("Polygon count = {}", polygons.len());

Export an STL:

    let stl_data = union_result.to_stl("cube_minus_sphere");
    let filename = "output.stl";
    let mut file = File::create(filename).expect("Failed to create file");
    file.write_all(stl_data.as_bytes()).expect("Failed to write STL");

# Implementation Details

All CSG operations are implemented in terms of two functions, `clipTo()` and `invert()`, which remove parts of a BSP tree inside another BSP tree and swap solid and empty space, respectively. To find the union of `a` and `b`, we want to remove everything in `a` inside `b` and everything in `b` inside `a`, then combine polygons from `a` and `b` into one solid:

    a.clipTo(b);
    b.clipTo(a);
    a.build(b.allPolygons());

The only tricky part is handling overlapping coplanar polygons in both trees. The code above keeps both copies, but we need to keep them in one tree and remove them in the other tree. To remove them from `b` we can clip the inverse of `b` against `a`. The code for union now looks like this:

    a.clipTo(b);
    b.clipTo(a);
    b.invert();
    b.clipTo(a);
    b.invert();
    a.build(b.allPolygons());

Subtraction and intersection naturally follow from set operations. If union is `A | B`, subtraction is `A - B = ~(~A | B)` and intersection is `A & B = ~(~A | ~B)` where `~` is the complement operator.

# License

Copyright (c) 2025 Timothy Schmidt, based on a translation of CSG.js Copyright (c) 2011 Evan Wallace (http://madebyevan.com/), under the [MIT license](http://www.opensource.org/licenses/mit-license.php).
