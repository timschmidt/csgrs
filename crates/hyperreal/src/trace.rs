//! Feature-selected dispatch instrumentation for scalar execution paths.
//!
//! With dispatch-trace enabled, the local macro forwards three labels to the
//! shared collector; collection still depends on the current thread's recording
//! scope. Without that feature, the same invocation emits no code and evaluates
//! none of its argument expressions. Instrumentation arguments must not contain
//! side effects needed for the underlying arithmetic to be correct.
//!
//! The macro records a chosen label, not a correctness certificate, policy receipt
//! or timing measurement. It does not choose or reorder arithmetic paths. Trace
//! storage, aggregation and recording scope belong to dispatch_trace; this shim
//! adds no independent resource budget or failure classification.
//!
//! Make the selected lexical macro available to crate-local path-based invocations.
//!
//! The crate root re-exports this alias for crate::trace_dispatch! callers.
//! Visibility remains crate-local; forwarding keeps the macro's $crate resolution
//! and feature-specific argument semantics, without a public exported macro.

/// Forward one dispatch event to this crate's shared collector when tracing is compiled.
///
/// Accepts exactly three expressions for layer, operation and path, without an
/// optional trailing comma. Each is evaluated once, left to right, before calling
/// record, even when recording is currently disabled. The callee requires static
/// string labels and only retains the event within the current thread's recording
/// scope. This macro neither starts a scope nor clears existing counts.
///
/// Uses $crate to resolve the collector independently of the invocation's module.
/// Expansion is a semicolon-terminated call for statement use. Collection can
/// allocate and has the collector's ordinary counter/borrow behavior; the macro
/// does not introduce typed errors, bounds or a certainty claim.
#[cfg(feature = "dispatch-trace")]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {
        $crate::dispatch_trace::record($layer, $operation, $path);
    };
}

/// Erase one dispatch invocation when the tracing feature is absent.
///
/// Matches exactly three Rust expressions, without an optional trailing comma,
/// and emits no tokens. Arguments must parse as expressions but their names and
/// types are not resolved and they are not evaluated after erasure. No collector,
/// allocation or argument side effect is reached through this expansion.
///
/// The invocation is intended for statement contexts; empty expansion is not a
/// unit-valued expression. This definition and the enabled definition are mutually
/// exclusive. Enabling tracing therefore changes argument evaluation, not merely
/// whether the collector retains an event.
#[cfg(not(feature = "dispatch-trace"))]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {};
}

/// Make the selected lexical macro available to crate-local path-based invocations.
///
/// The crate root re-exports this alias for crate::trace_dispatch! callers.
/// Visibility remains crate-local; forwarding keeps the macro's $crate resolution
/// and feature-specific argument semantics, without a public exported macro.
pub(crate) use trace_dispatch;
