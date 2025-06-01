//! This example shows using optional metadata for storing color and label.\
//! Other use cases include storing speed, ID, or layer info.

use csgrs::{Vertex, polygon::Polygon};
use nalgebra::{Point3, Vector3};

/// Demo metadata type
#[derive(Clone)]
struct MyMetadata {
    /// RGB-8 color of the `Polygon`
    color: (u8, u8, u8),
    /// Name of the `Polygon`
    label: String,
}

const PATH: &str = "stl/metadata";

fn main() {
    // For a single polygon:
    let mut poly_w_metadata = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        Some(MyMetadata {
            color: (255, 0, 0),
            label: "Triangle".into(),
        }),
    );

    // Retrieve metadata
    if let Some(data) = poly_w_metadata.metadata() {
        println!("This polygon is labeled {}", data.label);
    }

    // Mutate metadata
    if let Some(data_mut) = poly_w_metadata.metadata_mut() {
        data_mut.label.push_str("_extended");
    }
}
