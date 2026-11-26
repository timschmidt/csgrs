//! Optional dispatch instrumentation shared by predicate and scalar execution paths.

#[cfg(feature = "dispatch-trace")]
/// Record one layer/operation/path event in Hyperreal's shared dispatch collector.
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {
        ::hyperreal::dispatch_trace::record($layer, $operation, $path);
    };
}

#[cfg(not(feature = "dispatch-trace"))]
/// Erase the instrumentation call without evaluating any of its argument expressions.
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {};
}

pub(crate) use trace_dispatch;
