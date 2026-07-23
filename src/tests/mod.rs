//! Internal test modules for the crate.

mod support;

mod bug_tests;
mod csg_tests;
#[cfg(feature = "dispatch-trace")]
mod dispatch_trace_tests;
mod extrude_slice_tests;
mod flatten_slice_tests;
mod metadata_tests;
mod misc_tests;
mod offset_tests;
mod ownership_tests;
mod parts_tests;
mod plane_tests;
mod polygon_tests;
mod sketch_region_tests;
mod vertex_tests;
