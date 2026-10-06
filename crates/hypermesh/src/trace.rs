#[cfg(feature = "dispatch-trace")]
macro_rules! trace_dispatch {
    ($operation:expr, $path:expr) => {
        ::hyperreal::dispatch_trace::record("hypermesh", $operation, $path);
    };
}

#[cfg(not(feature = "dispatch-trace"))]
macro_rules! trace_dispatch {
    ($operation:expr, $path:expr) => {};
}

pub(crate) use trace_dispatch;
