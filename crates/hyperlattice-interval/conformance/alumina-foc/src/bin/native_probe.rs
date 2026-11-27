//! Native firmware provider linked to the shared digest workload for executable comparison.
//! The included main owns argument parsing, wrapping seed progression and output formatting.
use interval_foc_conformance::samples::native as provider;
include!("probe_body.inc");
