use csgrs::IndexedMesh::IndexedMesh;
use std::collections::HashMap;

#[test]
fn test_perfect_manifold_validation() {
    println!("=== PERFECT MANIFOLD VALIDATION ===");
    println!("Testing the CSGRS_PERFECT_MANIFOLD environment variable mode");
    
    // Create test shapes
    let cube = IndexedMesh::<String>::cube(2.0, Some("cube".to_string()));
    let sphere = IndexedMesh::<String>::sphere(1.0, 6, 6, Some("sphere".to_string()));
    
    println!("Test shapes:");
    println!("  Cube: {} vertices, {} polygons", cube.vertices.len(), cube.polygons.len());
    println!("  Sphere: {} vertices, {} polygons", sphere.vertices.len(), sphere.polygons.len());
    
    // Test 1: Standard Mode (Baseline)
    println!("\n--- Test 1: Standard Mode (Baseline) ---");
    
    let union_standard = cube.union_indexed(&sphere);
    let intersection_standard = cube.intersection_indexed(&sphere);
    let difference_standard = cube.difference_indexed(&sphere);
    
    let union_boundary_standard = count_boundary_edges(&union_standard);
    let intersection_boundary_standard = count_boundary_edges(&intersection_standard);
    let difference_boundary_standard = count_boundary_edges(&difference_standard);
    
    println!("Standard mode results:");
    println!("  Union: {} boundary edges", union_boundary_standard);
    println!("  Intersection: {} boundary edges", intersection_boundary_standard);
    println!("  Difference: {} boundary edges", difference_boundary_standard);
    
    let total_standard = union_boundary_standard + intersection_boundary_standard + difference_boundary_standard;
    println!("  Total boundary edges: {}", total_standard);
    
    // Test 2: Perfect Manifold Mode
    println!("\n--- Test 2: Perfect Manifold Mode ---");
    
    // Set environment variable for perfect manifold mode
    unsafe {
        std::env::set_var("CSGRS_PERFECT_MANIFOLD", "1");
    }
    
    let start_time = std::time::Instant::now();
    
    let union_perfect = cube.union_indexed(&sphere);
    let intersection_perfect = cube.intersection_indexed(&sphere);
    let difference_perfect = cube.difference_indexed(&sphere);
    
    let perfect_time = start_time.elapsed();
    
    // Clear environment variable
    unsafe {
        std::env::remove_var("CSGRS_PERFECT_MANIFOLD");
    }
    
    let union_boundary_perfect = count_boundary_edges(&union_perfect);
    let intersection_boundary_perfect = count_boundary_edges(&intersection_perfect);
    let difference_boundary_perfect = count_boundary_edges(&difference_perfect);
    
    println!("Perfect manifold mode results:");
    println!("  Union: {} boundary edges", union_boundary_perfect);
    println!("  Intersection: {} boundary edges", intersection_boundary_perfect);
    println!("  Difference: {} boundary edges", difference_boundary_perfect);
    
    let total_perfect = union_boundary_perfect + intersection_boundary_perfect + difference_boundary_perfect;
    println!("  Total boundary edges: {}", total_perfect);
    
    // Test 3: Performance Comparison
    println!("\n--- Test 3: Performance Comparison ---");
    
    let start_time = std::time::Instant::now();
    let _union_standard_perf = cube.union_indexed(&sphere);
    let standard_time = start_time.elapsed();
    
    let performance_ratio = perfect_time.as_secs_f64() / standard_time.as_secs_f64();
    
    println!("Performance comparison:");
    println!("  Standard mode: {:.2}ms", standard_time.as_secs_f64() * 1000.0);
    println!("  Perfect manifold mode: {:.2}ms", perfect_time.as_secs_f64() * 1000.0);
    println!("  Performance ratio: {:.1}x slower", performance_ratio);
    
    // Test 4: Memory Efficiency Analysis
    println!("\n--- Test 4: Memory Efficiency Analysis ---");
    
    let standard_vertices = union_standard.vertices.len();
    let standard_polygons = union_standard.polygons.len();
    let perfect_vertices = union_perfect.vertices.len();
    let perfect_polygons = union_perfect.polygons.len();
    
    let vertex_sharing_standard = calculate_vertex_sharing(&union_standard);
    let vertex_sharing_perfect = calculate_vertex_sharing(&union_perfect);
    
    println!("Memory efficiency comparison:");
    println!("  Standard: {} vertices, {} polygons, {:.2}x vertex sharing", 
             standard_vertices, standard_polygons, vertex_sharing_standard);
    println!("  Perfect: {} vertices, {} polygons, {:.2}x vertex sharing", 
             perfect_vertices, perfect_polygons, vertex_sharing_perfect);
    
    let vertex_overhead = perfect_vertices as f64 / standard_vertices as f64;
    let polygon_overhead = perfect_polygons as f64 / standard_polygons as f64;
    
    println!("  Vertex overhead: {:.2}x", vertex_overhead);
    println!("  Polygon overhead: {:.2}x", polygon_overhead);
    
    // Test 5: Manifold Topology Validation
    println!("\n--- Test 5: Manifold Topology Validation ---");
    
    let perfect_operations = [union_boundary_perfect, intersection_boundary_perfect, difference_boundary_perfect]
        .iter().filter(|&&edges| edges == 0).count();
    
    let standard_operations = [union_boundary_standard, intersection_boundary_standard, difference_boundary_standard]
        .iter().filter(|&&edges| edges == 0).count();
    
    println!("Manifold topology validation:");
    println!("  Standard mode perfect operations: {}/3", standard_operations);
    println!("  Perfect mode perfect operations: {}/3", perfect_operations);
    
    let improvement = total_standard as i32 - total_perfect as i32;
    let improvement_percentage = if total_standard > 0 {
        improvement as f64 / total_standard as f64 * 100.0
    } else {
        0.0
    };
    
    println!("  Boundary edge improvement: {} ({:.1}%)", improvement, improvement_percentage);
    
    // Test 6: Watertight Validation
    println!("\n--- Test 6: Watertight Validation ---");
    
    let union_watertight = is_watertight(&union_perfect);
    let intersection_watertight = is_watertight(&intersection_perfect);
    let difference_watertight = is_watertight(&difference_perfect);
    
    println!("Watertight validation:");
    println!("  Union is watertight: {}", union_watertight);
    println!("  Intersection is watertight: {}", intersection_watertight);
    println!("  Difference is watertight: {}", difference_watertight);
    
    let watertight_operations = [union_watertight, intersection_watertight, difference_watertight]
        .iter().filter(|&&watertight| watertight).count();
    
    println!("  Watertight operations: {}/3", watertight_operations);
    
    // Test 7: Quality Assessment
    println!("\n--- Test 7: Quality Assessment ---");
    
    let quality_score = calculate_quality_score(
        perfect_operations,
        watertight_operations,
        improvement,
        performance_ratio,
        vertex_overhead,
        polygon_overhead,
    );
    
    println!("Quality assessment:");
    println!("  Perfect operations: {}/3", perfect_operations);
    println!("  Watertight operations: {}/3", watertight_operations);
    println!("  Boundary edge improvement: {}", improvement);
    println!("  Performance cost: {:.1}x", performance_ratio);
    println!("  Memory overhead: {:.1}x vertices, {:.1}x polygons", vertex_overhead, polygon_overhead);
    println!("  Overall quality score: {:.1}%", quality_score);
    
    // Final Assessment
    println!("\n--- Final Assessment ---");
    
    if perfect_operations == 3 && watertight_operations == 3 {
        println!("🎉 PERFECT SUCCESS: All operations achieve perfect manifold topology!");
        
        if performance_ratio <= 2.0 {
            println!("   ✅ Excellent performance cost");
        } else if performance_ratio <= 5.0 {
            println!("   ✅ Acceptable performance cost");
        } else {
            println!("   ⚠️  High performance cost but may be acceptable for quality");
        }
        
        if vertex_overhead <= 1.5 && polygon_overhead <= 2.0 {
            println!("   ✅ Excellent memory efficiency");
        } else {
            println!("   ⚠️  Some memory overhead but reasonable");
        }
        
        println!("   RECOMMENDATION: Deploy perfect manifold mode for high-quality applications");
        
    } else if perfect_operations >= 2 || improvement >= 20 {
        println!("✅ EXCELLENT IMPROVEMENT: Significant quality enhancement achieved");
        println!("   {} operations achieve perfect topology", perfect_operations);
        println!("   {} boundary edges eliminated", improvement);
        
        if performance_ratio <= 5.0 {
            println!("   ✅ Performance cost is acceptable");
            println!("   RECOMMENDATION: Deploy as enhanced quality mode");
        } else {
            println!("   ⚠️  Performance cost is high");
            println!("   RECOMMENDATION: Offer as optional high-quality mode");
        }
        
    } else if improvement > 0 {
        println!("🟡 SOME IMPROVEMENT: Partial quality enhancement");
        println!("   {} boundary edges eliminated", improvement);
        println!("   Continue algorithm development for better results");
        
    } else {
        println!("❌ NO IMPROVEMENT: Perfect manifold mode ineffective");
        println!("   Need different algorithmic approaches");
    }
    
    // Test 8: Deployment Recommendations
    println!("\n--- Test 8: Deployment Recommendations ---");
    
    if quality_score >= 80.0 {
        println!("DEPLOYMENT STATUS: ✅ PRODUCTION READY");
        println!("  Perfect manifold mode is ready for production deployment");
        println!("  Recommended for: CAD applications, 3D printing, high-quality rendering");
        println!("  Configuration: Set CSGRS_PERFECT_MANIFOLD=1 environment variable");
    } else if quality_score >= 60.0 {
        println!("DEPLOYMENT STATUS: 🟡 BETA READY");
        println!("  Perfect manifold mode shows significant improvement");
        println!("  Recommended for: Testing, quality-critical applications");
        println!("  Configuration: Optional high-quality mode");
    } else {
        println!("DEPLOYMENT STATUS: ⚠️  DEVELOPMENT NEEDED");
        println!("  Continue algorithm development and optimization");
        println!("  Focus on: Performance optimization, memory efficiency");
    }
    
    println!("=== PERFECT MANIFOLD VALIDATION COMPLETE ===");
    
    // Assertions for automated testing
    assert!(perfect_operations >= standard_operations, 
            "Perfect manifold mode should not decrease quality");
    assert!(total_perfect <= total_standard, 
            "Perfect manifold mode should not increase boundary edges");
    assert!(performance_ratio <= 10.0, 
            "Performance cost should be reasonable");
}

fn count_boundary_edges(mesh: &IndexedMesh<String>) -> usize {
    let mut edge_count: HashMap<(usize, usize), usize> = HashMap::new();
    
    for polygon in &mesh.polygons {
        let indices = &polygon.indices;
        for i in 0..indices.len() {
            let v1 = indices[i];
            let v2 = indices[(i + 1) % indices.len()];
            let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
            *edge_count.entry(edge).or_insert(0) += 1;
        }
    }
    
    edge_count.values().filter(|&&count| count == 1).count()
}

fn calculate_vertex_sharing(mesh: &IndexedMesh<String>) -> f64 {
    let total_vertex_references: usize = mesh.polygons.iter()
        .map(|poly| poly.indices.len()).sum();
    
    if mesh.vertices.is_empty() {
        return 0.0;
    }
    
    total_vertex_references as f64 / mesh.vertices.len() as f64
}

fn is_watertight(mesh: &IndexedMesh<String>) -> bool {
    count_boundary_edges(mesh) == 0
}

fn calculate_quality_score(
    perfect_operations: usize,
    watertight_operations: usize,
    improvement: i32,
    performance_ratio: f64,
    vertex_overhead: f64,
    polygon_overhead: f64,
) -> f64 {
    let topology_score = (perfect_operations as f64 / 3.0) * 40.0;
    let watertight_score = (watertight_operations as f64 / 3.0) * 30.0;
    let improvement_score = (improvement.min(50) as f64 / 50.0) * 20.0;
    let performance_penalty = if performance_ratio <= 2.0 { 0.0 } else { (performance_ratio - 2.0).min(8.0) };
    let memory_penalty = if vertex_overhead <= 1.5 && polygon_overhead <= 2.0 { 0.0 } else { 5.0 };
    
    (topology_score + watertight_score + improvement_score - performance_penalty - memory_penalty).max(0.0)
}
