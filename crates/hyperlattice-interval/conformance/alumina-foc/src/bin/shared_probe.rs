//! Shared-kernel provider linked to the identical digest workload for executable comparison.
//! The included main owns argument parsing, wrapping seed progression and output formatting.
use interval_foc_conformance::samples::shared as provider;
include!("probe_body.inc");
