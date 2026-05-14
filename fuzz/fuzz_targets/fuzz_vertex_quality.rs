#![no_main]

use csgrs::float_types::Real;
use csgrs::vertex::{Vertex, VertexCluster};
use hashbrown::HashMap;
use libfuzzer_sys::fuzz_target;
use nalgebra::{Point3, Vector3};

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-100.0, 100.0) as Real
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0usize;
    let count = (bytes[idx % bytes.len()] as usize % 12) + 1;
    idx += 1;
    let mut vertices = Vec::with_capacity(count);
    for _ in 0..count {
        vertices.push(Vertex::new(
            Point3::new(
                decode_real(bytes, &mut idx),
                decode_real(bytes, &mut idx),
                decode_real(bytes, &mut idx),
            ),
            Vector3::new(
                decode_real(bytes, &mut idx),
                decode_real(bytes, &mut idx),
                decode_real(bytes, &mut idx),
            ),
        ));
    }
    let weighted = vertices
        .iter()
        .copied()
        .map(|v| (v, decode_real(bytes, &mut idx)))
        .collect::<Vec<_>>();
    if let Some(vertex) = Vertex::weighted_average(&weighted) {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
    }
    if vertices.len() >= 3 {
        let v = Vertex::barycentric_interpolate(
            &vertices[0],
            &vertices[1],
            &vertices[2],
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        );
        if v.position.coords.iter().all(|c| c.is_finite()) {
            assert!(v.normal.x.is_finite());
        }
    }
    let mut adjacency = HashMap::new();
    adjacency.insert(0usize, (1..vertices.len()).collect::<Vec<_>>());
    let mut positions = HashMap::new();
    let mut normals = HashMap::new();
    for (i, v) in vertices.iter().enumerate() {
        positions.insert(i, v.position);
        normals.insert(i, v.normal);
    }
    let _ = vertices[0].comprehensive_quality_analysis(0, &adjacency, &positions, &normals);
    if let Some(cluster) = VertexCluster::from_vertices(&vertices) {
        assert!(cluster.position.x.is_finite());
        assert!(cluster.radius.is_finite());
    }
});
