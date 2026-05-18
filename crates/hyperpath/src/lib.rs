//! Exact path planning primitives for the hyper geometry stack.
//!
//! `hyperpath` owns path-domain carriers and scheduling facts for CAM and PCB
//! routing. It deliberately delegates scalar arithmetic to `hyperreal` and
//! topology predicates to `hyperlimit`. This is the object-layer split
//! advocated by Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997): path search may generate candidates, but exact
//! predicates certify the topology before the candidate becomes output.

pub mod pcb;
pub mod provenance;
pub mod routing;
pub mod segment;
pub mod swept;

pub use pcb::{
    ClearanceStatus, NetId, PcbCircularPad, PcbPadFacts, PcbRectPad, PcbTrace, PcbTraceFacts,
    PcbViaStack, TraceClearanceReport, TraceLayer, TraceWidthClass, ViaAnnularRingReport,
    check_trace_clearance, check_trace_pad_clearance, check_trace_rect_pad_clearance,
    check_trace_via_clearance,
};
pub use provenance::{PathProvenance, PathSourceFormat, SourceGrid};
pub use routing::{LengthMatchProblem, build_length_match_problem, certify_length_extension};
pub use segment::{Axis, LinePathSegment, LinePathSegmentFacts, SegmentParameterOrder};
pub use swept::{SweptLineSegment, SweptLineSegmentFacts};
