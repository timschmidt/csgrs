//! Internal test modules for the crate.

mod support;

mod bug_tests;
#[cfg(feature = "cavalier")]
mod cavalier_tests;
mod csg_tests;
mod extrude_slice_tests;
mod flatten_slice_tests;
mod mesh_quality_tests;
mod metadata_tests;
mod misc_tests;
mod node_tests;
mod offset_tests;
mod plane_tests;
mod polygon_tests;
mod vertex_tests;
