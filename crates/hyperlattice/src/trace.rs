//! Feature-gated forwarding of lattice dispatch labels to Hyperreal's trace recorder.
//!
//! The enabled route evaluates label expressions and calls the recorder; the disabled
//! route erases them. Recording is thread-local and opt-in in Hyperreal, so compiling
//! the feature alone does not guarantee that counters are collected.
/// Forward three static string labels to hyperreal::dispatch_trace::record.
///
/// Arguments are evaluated even when the recorder is currently inactive; this macro
/// does not open a recording scope or synchronize traces across threads.
#[cfg(feature = "hyperreal-dispatch-trace")]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {
        ::hyperreal::dispatch_trace::record($layer, $operation, $path);
    };
}

/// Expand to no statements when tracing is disabled, without evaluating any label expression.
#[cfg(not(feature = "hyperreal-dispatch-trace"))]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {};
}

pub(crate) use trace_dispatch;
