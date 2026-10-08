#[cfg(feature = "dispatch-trace")]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {
        $crate::dispatch_trace::record($layer, $operation, $path);
    };
}

#[cfg(not(feature = "dispatch-trace"))]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {};
}

pub(crate) use trace_dispatch;
