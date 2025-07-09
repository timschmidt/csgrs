use csgrs::mesh::Mesh;
use csgrs::traits::CSG;

fn main() {
    println!("Testing complex CSG operations for stack overflow...");
    
    // Test 1: Large sphere with many subdivisions
    println!("Test 1: Large complex sphere...");
    let sphere = Mesh::<()>::sphere(50.0, 64, 32, None);
    println!("  Created sphere with {} polygons", sphere.polygons.len());
    
    // Test 2: Multiple boolean operations creating deep BSP tree
    println!("Test 2: Multiple boolean operations...");
    let cube1 = Mesh::<()>::cube(10.0, None);
    let cube2 = Mesh::<()>::cube(8.0, None).translate(2.0, 0.0, 0.0);
    let cube3 = Mesh::<()>::cube(6.0, None).translate(0.0, 2.0, 0.0);
    let cube4 = Mesh::<()>::cube(4.0, None).translate(0.0, 0.0, 2.0);
    
    let result = cube1.union(&cube2).difference(&cube3).union(&cube4);
    println!("  Complex boolean result has {} polygons", result.polygons.len());
    
    // Test 3: Many small objects unioned together
    println!("Test 3: Many objects union...");
    let mut union_result = Mesh::<()>::cube(1.0, None);
    for i in 0..20 {
        let small_sphere = Mesh::<()>::sphere(0.5, 8, 4, None)
            .translate(i as f64 * 0.5, 0.0, 0.0);
        union_result = union_result.union(&small_sphere);
    }
    println!("  Union of many objects has {} polygons", union_result.polygons.len());
    
    // Test 4: Deep difference operations
    println!("Test 4: Deep difference operations...");
    let base = Mesh::<()>::cube(20.0, None);
    let mut result = base.clone();
    for i in 0..10 {
        let hole = Mesh::<()>::sphere(1.0, 16, 8, None)
            .translate(i as f64 * 2.0 - 9.0, 0.0, 0.0);
        result = result.difference(&hole);
    }
    println!("  Result with many holes has {} polygons", result.polygons.len());
    
    println!("\nAll tests completed successfully - no stack overflow!");
} 