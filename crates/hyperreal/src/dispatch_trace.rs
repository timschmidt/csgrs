//! Thread-local dispatch counts and rational arithmetic diagnostics.
//!
//! Available with the dispatch-trace feature. Instrumented scalar, predicate and
//! linear-algebra paths supply static string labels; recording scopes enable
//! collection on the current thread. Counts accumulate across scopes until reset
//! or taken. Live keys distinguish string addresses and lengths, while snapshots
//! merge equal text and sort it deterministically. Labels describe reported paths,
//! not exactness, topology, provenance or a proof that an operation completed.
//!
//! Reports are ordinary mutable values. Their label-based correlation buckets may
//! overlap and may misclassify arbitrary labels; they are not certainty receipts.
//! Rational statistics count explicit recorder calls rather than independently
//! observing all allocations or arithmetic. Cloning reports does not clone the
//! thread's collector or create a new recording scope.
//!
//! Storage and snapshot aggregation allocate without a total-work, memory or
//! cancellation budget. Counters use ordinary u64 addition: overflow follows the
//! build's overflow-check settings, not saturation or a typed exhausted outcome.
//! RefCell borrow conflicts or inaccessible thread-local state can panic. Recording
//! guards rely on same-thread, reverse-order destruction; their type does not
//! enforce either rule. See RecordingGuard before managing scopes manually.

use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::hash::{BuildHasherDefault, Hash, Hasher};

use num::{BigUint, One, Zero};

/// One content-keyed dispatch count in an owned report.
///
/// Recorder snapshots merge all live keys with equal label text into this row.
/// Public fields also allow fabricated, duplicate or zero-count rows; construction
/// does not validate labels or evidence. Derived equality compares all field values,
/// including string contents. Clone copies the labels and count without affecting
/// collector state; Debug formats this report, not the underlying arithmetic.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DispatchCount {
    /// Caller-supplied layer label; snapshots compare its string contents.
    pub layer: &'static str,
    /// Caller-supplied operation label within the layer; empty labels are allowed.
    pub operation: &'static str,
    /// Caller-supplied selected-path label; no semantic validation is performed.
    pub path: &'static str,
    /// Recorded multiplicity, or any count assigned by a report author.
    pub count: u64,
}

/// Sum of reported path counts for one layer/operation pair.
///
/// TraceSnapshot::operation_summaries produces one row per distinct text pair in
/// lexicographic order. Public construction need not come from a collector or
/// represent completed work. Clone and equality operate on the three fields;
/// this reporting unit carries no policy or numerical-certainty claim.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OperationSummary {
    /// Layer text shared by the rows grouped into this summary.
    pub layer: &'static str,
    /// Operation text shared by those rows, independent of their path labels.
    pub operation: &'static str,
    /// Sum of path counts for this text pair; not a distinct-operation count.
    pub count: u64,
}

/// Sum of reported operation/path counts for one layer.
///
/// TraceSnapshot::layer_summaries groups by label contents, returning sorted rows.
/// This public carrier may also be built or edited directly. Clone, Debug and
/// equality describe its fields; a count is instrumentation data, not a measured
/// duration or a proof that all operations in the layer were observed.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LayerSummary {
    /// Layer text used as the grouping key.
    pub layer: &'static str,
    /// Sum of all row counts for this layer, including duplicate source rows.
    pub count: u64,
}

/// Overlapping label-heuristic totals and copied rational recorder counts.
///
/// TraceSnapshot::correlation_summary weights each matching category by its row's
/// count. Matching is case-sensitive and mostly substring based; a row may enter
/// several categories, or none beyond dispatch_events. Arbitrary labels can produce
/// false positives and false negatives, so these categories do not establish the
/// mathematical work, certainty or costs their names suggest.
///
/// Public fields are editable. Default zeroes every counter; Clone/Copy duplicate
/// these scalar values, while Debug and equality inspect them. Neither construction
/// nor comparison validates a recording window or relates counters to geometry.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TraceCorrelationSummary {
    /// Sum of every dispatch row count, regardless of category matches.
    pub dispatch_events: u64,
    /// Weighted rows whose layer is hyperlimit or contains predicate.
    pub predicate_events: u64,
    /// Weighted rows whose layer starts with hyperlattice.
    pub linear_algebra_events: u64,
    /// Weighted non-real rows matching the object-fact substring rules.
    pub object_fact_events: u64,
    /// Weighted real-layer rows matching scalar-fact substrings.
    pub scalar_fact_events: u64,
    /// Weighted rows matching detailed or exact-set/structural/geometry-fact labels.
    pub detailed_fact_events: u64,
    /// Weighted rows matching unknown, uncertain or missing-fact substrings.
    pub unknown_fact_events: u64,
    /// Weighted rows matching rational-kind, exact-set, dyadic or shared-denominator labels.
    pub exact_rational_kind_events: u64,
    /// Weighted rows matching sign, zero, compare, ordering or domain substrings.
    pub sign_or_zero_query_events: u64,
    /// Weighted rows matching the exact-reducer substring rules, not verified reductions.
    pub exact_reducer_events: u64,
    /// Weighted rows matching approx, lossy, f64 or float substrings.
    pub approximation_events: u64,
    /// Weighted approximation rows matching start rules and excluding cache rules.
    pub approximation_start_events: u64,
    /// Weighted approximation rows also matching cache, cached or hit substrings.
    pub approximation_cache_events: u64,
    /// Weighted rows matching refine, refinement or certified-sign substrings.
    pub refinement_events: u64,
    /// Weighted predicate-layer rows also matching a decision-stage substring.
    pub predicate_decision_stage_events: u64,
    /// Weighted rows matching cache, cached or prepared substrings.
    pub cache_events: u64,
    /// Weighted rows matching abort, fallback, generic, unknown or error/rejection labels.
    pub fallback_or_abort_events: u64,
    /// Copy of RationalTraceStats::temporary_rationals, independent of row classification.
    pub rational_temporaries: u64,
    /// Copy of RationalTraceStats::reductions, independent of row classification.
    pub rational_reductions: u64,
    /// Copy of RationalTraceStats::gcds, independent of row classification.
    pub rational_gcds: u64,
}

/// Owned dispatch rows and rational statistics captured from one thread.
///
/// snapshot_trace and take_trace produce text-merged, lexicographically sorted
/// rows. These public fields are mutable, so arbitrary reports can contain unsorted
/// or duplicate rows and unrelated statistics. Query methods operate on that supplied
/// data without validating it. In particular path_count returns the first matching
/// row, whereas grouped totals include every matching row.
///
/// Clone duplicates the row vector and statistics but leaves live counters alone;
/// derived equality compares vector order and every field. This report owns no
/// geometry cache and is neither immutable nor a certification receipt.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceSnapshot {
    /// Owned rows; collector snapshots sort and merge them, but public mutation need not.
    pub dispatch: Vec<DispatchCount>,
    /// Copied rational recorder statistics; public mutation need not match the rows.
    pub rational: RationalTraceStats,
}

/// Query and aggregate supplied report data without changing the live collector.
///
/// Grouping allocates temporary maps/vectors; counter sums use ordinary u64 arithmetic.
/// Publicly edited reports are accepted as-is, including duplicates and zero counts.
impl TraceSnapshot {
    /// Return the first row matching all three labels by content, or zero if absent.
    ///
    /// The linear search neither sums duplicates nor requires sorted input. Collector
    /// snapshots contain unique text keys, but publicly edited reports may not. Empty
    /// labels are valid; no live collector state or rational statistic is consulted.
    pub fn path_count(&self, layer: &str, operation: &str, path: &str) -> u64 {
        self.dispatch
            .iter()
            .find(|entry| {
                entry.layer == layer && entry.operation == operation && entry.path == path
            })
            .map_or(0, |entry| entry.count)
    }

    /// Sum every row whose layer text equals the requested label.
    ///
    /// Includes duplicate rows and uses ordinary u64 addition, with build-dependent
    /// overflow behavior. Returns zero when no row matches; does not read live state.
    pub fn layer_count(&self, layer: &str) -> u64 {
        self.dispatch
            .iter()
            .filter(|entry| entry.layer == layer)
            .map(|entry| entry.count)
            .sum()
    }

    /// Sum all rows matching the layer and operation text, including duplicates.
    ///
    /// Path labels do not affect this total. Empty matches give zero; ordinary u64 sum
    /// overflow is not converted into a typed error or a saturated report.
    pub fn operation_count(&self, layer: &str, operation: &str) -> u64 {
        self.dispatch
            .iter()
            .filter(|entry| entry.layer == layer && entry.operation == operation)
            .map(|entry| entry.count)
            .sum()
    }

    /// Group every supplied row by layer text and return lexicographically sorted totals.
    ///
    /// Allocates a BTreeMap and result vector, summing duplicates and retaining groups
    /// whose counts are zero. Arithmetic overflow follows the build configuration;
    /// the input snapshot and live collector are unchanged.
    pub fn layer_summaries(&self) -> Vec<LayerSummary> {
        let mut grouped = BTreeMap::<&'static str, u64>::new();
        for entry in &self.dispatch {
            *grouped.entry(entry.layer).or_insert(0) += entry.count;
        }
        grouped
            .into_iter()
            .map(|(layer, count)| LayerSummary { layer, count })
            .collect()
    }

    /// Group rows by the text pair (layer,operation), summing all paths and duplicates.
    ///
    /// Returns lexicographically sorted pairs using an allocated BTreeMap and vector.
    /// Zero-count groups remain present. Ordinary u64 addition can overflow; this
    /// method neither validates the labels nor modifies the original report.
    pub fn operation_summaries(&self) -> Vec<OperationSummary> {
        let mut grouped = BTreeMap::<(&'static str, &'static str), u64>::new();
        for entry in &self.dispatch {
            *grouped.entry((entry.layer, entry.operation)).or_insert(0) += entry.count;
        }
        grouped
            .into_iter()
            .map(|((layer, operation), count)| OperationSummary {
                layer,
                operation,
                count,
            })
            .collect()
    }

    /// Classify each supplied row by label heuristics and copy rational recorder totals.
    ///
    /// Adds each row's count to dispatch_events and to every matching category, in the
    /// fixed predicate order below. Categories overlap; they are not a partition or
    /// semantic validation. The helper predicates define the exact case-sensitive label
    /// rules, which can match incidental substrings. Duplicate rows contribute again.
    ///
    /// Copies temporary/reduction/GCD statistics independently of the dispatch rows.
    /// No live counters are read, and no allocation is required by this method itself.
    /// All sums use ordinary u64 arithmetic, without a typed overflow result.
    pub fn correlation_summary(&self) -> TraceCorrelationSummary {
        let mut summary = TraceCorrelationSummary {
            rational_temporaries: self.rational.temporary_rationals,
            rational_reductions: self.rational.reductions,
            rational_gcds: self.rational.gcds,
            ..TraceCorrelationSummary::default()
        };

        for entry in &self.dispatch {
            summary.dispatch_events += entry.count;
            if is_predicate_layer(entry.layer) {
                summary.predicate_events += entry.count;
            }
            if is_linear_algebra_layer(entry.layer) {
                summary.linear_algebra_events += entry.count;
            }
            if is_object_fact_event(entry) {
                summary.object_fact_events += entry.count;
            }
            if is_scalar_fact_event(entry) {
                summary.scalar_fact_events += entry.count;
            }
            if is_detailed_fact_event(entry) {
                summary.detailed_fact_events += entry.count;
            }
            if is_unknown_fact_event(entry) {
                summary.unknown_fact_events += entry.count;
            }
            if is_exact_rational_kind_event(entry) {
                summary.exact_rational_kind_events += entry.count;
            }
            if is_sign_or_zero_query_event(entry) {
                summary.sign_or_zero_query_events += entry.count;
            }
            if is_exact_reducer_event(entry) {
                summary.exact_reducer_events += entry.count;
            }
            if is_approximation_event(entry) {
                summary.approximation_events += entry.count;
            }
            if is_approximation_start_event(entry) {
                summary.approximation_start_events += entry.count;
            }
            if is_approximation_cache_event(entry) {
                summary.approximation_cache_events += entry.count;
            }
            if is_refinement_event(entry) {
                summary.refinement_events += entry.count;
            }
            if is_predicate_decision_stage_event(entry) {
                summary.predicate_decision_stage_events += entry.count;
            }
            if is_cache_event(entry) {
                summary.cache_events += entry.count;
            }
            if is_fallback_or_abort_event(entry) {
                summary.fallback_or_abort_events += entry.count;
            }
        }

        summary
    }
}

/// Test a case-sensitive substring without allocation or label normalization.
///
/// Uses str::contains, so an empty needle matches every string. This is a lexical
/// reporting helper; a match does not prove that the named arithmetic occurred.
fn contains_label_part(value: &str, needle: &str) -> bool {
    value.contains(needle)
}

/// Match a substring in layer, operation or path, in that short-circuit order.
///
/// The row count is ignored. Matching is case-sensitive and can cross no label
/// boundary; it is not tokenization or a validated event taxonomy.
fn entry_contains(entry: &DispatchCount, needle: &str) -> bool {
    contains_label_part(entry.layer, needle)
        || contains_label_part(entry.operation, needle)
        || contains_label_part(entry.path, needle)
}

/// Recognize predicate reporting layers by exact hyperlimit or substring predicate.
///
/// Other spellings or capitalization do not match unless they satisfy that literal
/// rule. This is a reporting classification, not proof of predicate execution.
fn is_predicate_layer(layer: &str) -> bool {
    layer == "hyperlimit" || layer.contains("predicate")
}

/// Recognize any layer beginning with hyperlattice, including arbitrary suffixes.
///
/// The case-sensitive prefix test alone assigns the linear-algebra category;
/// it neither parses a namespace nor verifies a matrix/vector operation.
fn is_linear_algebra_layer(layer: &str) -> bool {
    layer.starts_with("hyperlattice")
}

/// Classify a non-real layer row as object-fact work by label substrings.
///
/// Requires layer != real and any label containing facts, structural, shared-scale,
/// schedule or prepared. Other categories may match the same row. The heuristic
/// does not inspect cached facts or prove that object-level evidence was reused.
fn is_object_fact_event(entry: &DispatchCount) -> bool {
    entry.layer != "real"
        && (entry_contains(entry, "facts")
            || entry_contains(entry, "structural")
            || entry_contains(entry, "shared-scale")
            || entry_contains(entry, "schedule")
            || entry_contains(entry, "prepared"))
}

/// Classify a real-layer row as scalar-fact work by label substrings.
///
/// Requires layer == real and facts, exact-set, zero, domain or sign in any label.
/// This matches text rather than scalar certificates and can overlap other buckets.
fn is_scalar_fact_event(entry: &DispatchCount) -> bool {
    entry.layer == "real"
        && (entry_contains(entry, "facts")
            || entry_contains(entry, "exact-set")
            || entry_contains(entry, "zero")
            || entry_contains(entry, "domain")
            || entry_contains(entry, "sign"))
}

/// Classify a row as detailed fact reporting by literal label substrings.
///
/// Matches any of `detailed`, `exact_set_facts`, `exact-set`, `structural-facts`,
/// `geometry_facts` in layer, operation or path, using case-sensitive substring
/// tests. The row count is ignored. This heuristic can overlap other categories
/// and is not semantic evidence.
fn is_detailed_fact_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "detailed")
        || entry_contains(entry, "exact_set_facts")
        || entry_contains(entry, "exact-set")
        || entry_contains(entry, "structural-facts")
        || entry_contains(entry, "geometry_facts")
}

/// Classify a row as unknown or missing fact reporting by literal label substrings.
///
/// Matches any of `unknown`, `uncertain`, `missing-sign`, `nonzero-no-sign`,
/// `unavailable` in layer, operation or path, using case-sensitive substring
/// tests. The row count is ignored. This heuristic can overlap other categories
/// and is not semantic evidence.
fn is_unknown_fact_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "unknown")
        || entry_contains(entry, "uncertain")
        || entry_contains(entry, "missing-sign")
        || entry_contains(entry, "nonzero-no-sign")
        || entry_contains(entry, "unavailable")
}

/// Classify a row as exact rational kind reporting by literal label substrings.
///
/// Matches any of `exact-rational`, `rational-det`, `rational-kind`, `exact-set`,
/// `exact_set`, `dyadic`, `shared-denominator`, `common-denominator` in layer,
/// operation or path, using case-sensitive substring tests. The row count is
/// ignored. This heuristic can overlap other categories and is not semantic
/// evidence.
fn is_exact_rational_kind_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "exact-rational")
        || entry_contains(entry, "rational-det")
        || entry_contains(entry, "rational-kind")
        || entry_contains(entry, "exact-set")
        || entry_contains(entry, "exact_set")
        || entry_contains(entry, "dyadic")
        || entry_contains(entry, "shared-denominator")
        || entry_contains(entry, "common-denominator")
}

/// Classify a row as sign or zero query reporting by literal label substrings.
///
/// Matches any of `sign`, `zero`, `compare`, `ordering`, `domain` in layer,
/// operation or path, using case-sensitive substring tests. The row count is
/// ignored. This heuristic can overlap other categories and is not semantic
/// evidence.
fn is_sign_or_zero_query_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "sign")
        || entry_contains(entry, "zero")
        || entry_contains(entry, "compare")
        || entry_contains(entry, "ordering")
        || entry_contains(entry, "domain")
}

/// Classify a row as exact reducer reporting by literal label substrings.
///
/// Matches any of `exact`, `rational`, `determinant`, `product-sum`, `signed-
/// product-sum`, `kernel`, `dyadic`, `shared-denominator` in layer, operation or
/// path, using case-sensitive substring tests. The row count is ignored. This
/// heuristic can overlap other categories and is not semantic evidence.
fn is_exact_reducer_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "exact")
        || entry_contains(entry, "rational")
        || entry_contains(entry, "determinant")
        || entry_contains(entry, "product-sum")
        || entry_contains(entry, "signed-product-sum")
        || entry_contains(entry, "kernel")
        || entry_contains(entry, "dyadic")
        || entry_contains(entry, "shared-denominator")
}

/// Classify a row as approximation reporting by literal label substrings.
///
/// Matches any of `approx`, `lossy`, `f64`, `float` in layer, operation or path,
/// using case-sensitive substring tests. The row count is ignored. This heuristic
/// can overlap other categories and is not semantic evidence.
fn is_approximation_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "approx")
        || entry_contains(entry, "lossy")
        || entry_contains(entry, "f64")
        || entry_contains(entry, "float")
}

/// Recognize an approximation row as a start/export event while excluding cache matches.
///
/// Requires is_approximation_event, rejects is_approximation_cache_event, then
/// matches start, export, lossy, generic or approx in any label. The ordering and
/// exclusion distinguish this bucket from approximation_cache_events; other
/// report categories can still overlap. No execution stage is independently checked.
fn is_approximation_start_event(entry: &DispatchCount) -> bool {
    is_approximation_event(entry)
        && !is_approximation_cache_event(entry)
        && (entry_contains(entry, "start")
            || entry_contains(entry, "export")
            || entry_contains(entry, "lossy")
            || entry_contains(entry, "generic")
            || entry_contains(entry, "approx"))
}

/// Recognize an approximation row also containing cache, cached or hit in any label.
///
/// Checks is_approximation_event first. These case-sensitive substrings need not
/// describe an actual cache access; the predicate only controls report grouping.
fn is_approximation_cache_event(entry: &DispatchCount) -> bool {
    is_approximation_event(entry)
        && (entry_contains(entry, "cache")
            || entry_contains(entry, "cached")
            || entry_contains(entry, "hit"))
}

/// Classify a row as refinement reporting by literal label substrings.
///
/// Matches any of `refine`, `refinement`, `certified-sign` in layer, operation or
/// path, using case-sensitive substring tests. The row count is ignored. This
/// heuristic can overlap other categories and is not semantic evidence.
fn is_refinement_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "refine")
        || entry_contains(entry, "refinement")
        || entry_contains(entry, "certified-sign")
}

/// Recognize a predicate-layer row with a decision-stage substring.
///
/// First requires is_predicate_layer. Then matches resolve, decide, filter, exact,
/// refine, real-determinant, det, decided, positive, negative, zero, unknown or
/// uncertain in any label. Substring matches can be incidental; this classification
/// does not decide a predicate or infer numerical certainty from a label.
fn is_predicate_decision_stage_event(entry: &DispatchCount) -> bool {
    is_predicate_layer(entry.layer)
        && (entry_contains(entry, "resolve")
            || entry_contains(entry, "decide")
            || entry_contains(entry, "filter")
            || entry_contains(entry, "exact")
            || entry_contains(entry, "refine")
            || entry_contains(entry, "real-determinant")
            || entry_contains(entry, "det")
            || entry_contains(entry, "decided")
            || entry_contains(entry, "positive")
            || entry_contains(entry, "negative")
            || entry_contains(entry, "zero")
            || entry_contains(entry, "unknown")
            || entry_contains(entry, "uncertain"))
}

/// Classify a row as cache reporting by literal label substrings.
///
/// Matches any of `cache`, `cached`, `prepared` in layer, operation or path,
/// using case-sensitive substring tests. The row count is ignored. This heuristic
/// can overlap other categories and is not semantic evidence.
fn is_cache_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "cache")
        || entry_contains(entry, "cached")
        || entry_contains(entry, "prepared")
}

/// Classify a row as fallback or abort reporting by literal label substrings.
///
/// Matches any of `abort`, `fallback`, `generic`, `unknown`, `rejected`, `domain-
/// error`, `div-by-zero` in layer, operation or path, using case-sensitive
/// substring tests. The row count is ignored. This heuristic can overlap other
/// categories and is not semantic evidence.
fn is_fallback_or_abort_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "abort")
        || entry_contains(entry, "fallback")
        || entry_contains(entry, "generic")
        || entry_contains(entry, "unknown")
        || entry_contains(entry, "rejected")
        || entry_contains(entry, "domain-error")
        || entry_contains(entry, "div-by-zero")
}

/// Counts of explicitly reported common factors, grouped by value shape and size.
///
/// record_common_factor checks zero/one before powers of two, then bit-width bins.
/// The metadata-only power-of-two recorder also updates the first two bins. Counts
/// therefore need not sum to gcds. Default is all zeros; Copy/Clone and equality
/// operate on these public fields without validating the reported factors.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CommonFactorBuckets {
    /// Calls reporting zero or one, plus zero-shift common-factor metadata.
    pub none: u64,
    /// Calls reporting a power of two greater than one, plus any positive shift metadata.
    pub power_of_two: u64,
    /// Non-power-of-two factors with at most eight bits, after zero/one exclusion.
    pub small: u64,
    /// Non-power-of-two factors with nine through sixty-four bits.
    pub medium: u64,
    /// Non-power-of-two factors with more than sixty-four bits.
    pub large: u64,
}

/// Operand-width bins for explicitly recorded GCD calls.
///
/// The maximum and minimum operand bit lengths choose one bin per successful
/// record_rational_gcd call. Zero has bit length zero and fits the native bins.
/// Default zeroes the counters; public editing, Clone/Copy, Debug and equality
/// neither compute a GCD nor validate the claimed divisor.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct GcdOperandWidthBuckets {
    /// Both reported operand bit lengths are at most 64, including zero.
    pub both_u64: u64,
    /// Maximum operand width is 65..=128 bits and the minimum is at most 64.
    pub mixed_u64_u128: u64,
    /// Both reported operands need 65..=128 bits.
    pub both_u128: u64,
    /// At least one reported operand needs more than 128 bits.
    pub arbitrary_precision: u64,
}

/// Scalar recorder counters and maximum reported operand size for one thread.
///
/// These count explicit instrumentation calls, not all allocations, arithmetic or
/// GCD implementations independently. Default zeroes every field and bucket.
/// Copy/Clone duplicates the report values; Debug and equality inspect all fields.
/// Public construction and mutation need not preserve relations between counters.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RationalTraceStats {
    /// Number of calls to record_rational_temporary while recording, not bytes allocated.
    pub temporary_rationals: u64,
    /// Number of recorded reduction calls, regardless of whether input parts were reduced.
    pub reductions: u64,
    /// Number of recorded GCD calls; the recorder trusts the supplied divisor.
    pub gcds: u64,
    /// Reported common-factor categories, also affected by the metadata-only recorder.
    pub common_factors: CommonFactorBuckets,
    /// One operand-width bin per recorded GCD call, absent overflow or public mutation.
    pub gcd_operand_widths: GcdOperandWidthBuckets,
    /// Maximum BigUint bit length supplied to reduction/GCD recorders; zero if none.
    ///
    /// Includes supplied GCD divisors but not shift-only metadata, temporary bytes or
    /// the retained memory of the collector.
    pub peak_operand_bits: u64,
}

/// Content-ordered label triple used to merge and sort snapshot rows.
///
/// Derived equality and ordering compare layer, operation, then path as string
/// contents. Clone/Copy retain the static references. Unlike ActiveDispatchKey,
/// this key intentionally merges distinct allocations containing equal text.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct DispatchKey {
    /// Layer label reference; the owning key determines content versus pointer identity.
    layer: &'static str,
    /// Operation label reference; neither key validates its meaning.
    operation: &'static str,
    /// Path label reference; empty strings are permitted.
    path: &'static str,
}

/// Address-and-length identity for a live label triple.
///
/// Labels have static lifetimes, but callers may supply different static storage
/// with identical text, even at the same call site. Equality and hashing use each
/// string pointer and length rather than its contents. Snapshots later merge by
/// text. Clone/Copy retain the same identities; this private key is not a portable
/// serialized identity, geometric identity or externally stable event identifier.
#[derive(Clone, Copy)]
struct ActiveDispatchKey {
    /// Layer label reference; the owning key determines content versus pointer identity.
    layer: &'static str,
    /// Operation label reference; neither key validates its meaning.
    operation: &'static str,
    /// Path label reference; empty strings are permitted.
    path: &'static str,
}

/// Compare live keys by the complete pointer identity of each static string slice.
impl PartialEq for ActiveDispatchKey {
    /// Require equal string addresses and lengths for layer, operation and path.
    ///
    /// Short-circuits in that order. Equal text in distinct storage remains a distinct
    /// live key until snapshot aggregation; the count is not part of this key.
    fn eq(&self, other: &Self) -> bool {
        std::ptr::eq(self.layer, other.layer)
            && std::ptr::eq(self.operation, other.operation)
            && std::ptr::eq(self.path, other.path)
    }
}

/// Mark pointer-and-length equality as a total equivalence relation for live keys.
impl Eq for ActiveDispatchKey {}

/// Hash the same three pointer/length pairs used by live-key equality.
impl Hash for ActiveDispatchKey {
    /// Feed layer, operation and path address then length to the supplied hasher.
    ///
    /// Equal live keys therefore hash equally. Values depend on process addresses and
    /// the hasher, so they are not stable checksums of event label contents.
    fn hash<H: Hasher>(&self, state: &mut H) {
        state.write_usize(self.layer.as_ptr() as usize);
        state.write_usize(self.layer.len());
        state.write_usize(self.operation.as_ptr() as usize);
        state.write_usize(self.operation.len());
        state.write_usize(self.path.as_ptr() as usize);
        state.write_usize(self.path.len());
    }
}

/// Deterministic unseeded mixer for process-local dispatch-key lookup.
///
/// Default starts at zero. Pointer-sized writes use integer mixing and rotation;
/// byte writes use xor/multiply recurrence. This is not cryptographic, adversarially
/// seeded or a persistent fingerprint. Different write methods need not produce
/// the same hash for equivalent byte representations.
#[derive(Default)]
struct DispatchHasher(/** Current wrapping mixer state; Default initializes it to zero. */ u64);

/// Implement the byte and pointer-sized mixing used by the live HashMap builder.
impl Hasher for DispatchHasher {
    /// Return the current 64-bit mixer state without finalization or mutation.
    fn finish(&self) -> u64 {
        self.0
    }

    /// Fold bytes in order by xor then wrapping multiplication by 0x100000001b3.
    ///
    /// Empty input is unchanged; splitting the same byte stream across calls preserves
    /// this recurrence. Starting state is supplied by the hasher, normally zero.
    fn write(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.0 ^= u64::from(*byte);
            self.0 = self.0.wrapping_mul(0x100_0000_01b3);
        }
    }

    /// Mix a usize into 64 bits, then combine it with a 21-bit rotation of the state.
    ///
    /// Uses the explicit cast, xor shifts and wrapping multiplications below. It is
    /// specialized for pointer/length inputs and does not promise equivalence with
    /// write on the native byte encoding or cross-process stability.
    fn write_usize(&mut self, value: usize) {
        let mut mixed = value as u64;
        mixed ^= mixed >> 30;
        mixed = mixed.wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed ^= mixed >> 27;
        mixed = mixed.wrapping_mul(0x94d0_49bb_1331_11eb);
        mixed ^= mixed >> 31;
        self.0 = self.0.rotate_left(21) ^ mixed;
    }
}

/// Live identity-keyed event counts using a fresh zero-initialized DispatchHasher.
///
/// Distinct equal-text labels may occupy multiple entries. The HashMap allocates
/// and retains capacity when cleared; snapshot conversion owns text-based merging.
type ActiveCounts = HashMap<ActiveDispatchKey, u64, BuildHasherDefault<DispatchHasher>>;

/// Mutable recording flag and counters belonging to one thread.
///
/// Default disables recording, creates an empty live map and zeroes statistics.
/// A RefCell mediates access; reset methods preserve the flag and map capacity.
#[derive(Default)]
struct TraceState {
    /// Whether recorder calls on this thread currently retain events/statistics.
    recording: bool,
    /// Live pointer-identity counts, merged by text only when reported.
    counts: ActiveCounts,
    /// Rational recorder statistics accumulated independently of the live label map.
    rational: RationalTraceStats,
}

thread_local! {
    /// Lazily initialized per-thread collector, initially disabled with empty counters.
    ///
    /// The thread_local expansion supplies access/initialization machinery. RefCell
    /// borrowing checks remain active; this is not a cross-thread shared collector or
    /// a synchronization primitive for moving RecordingGuard between threads.
    static TRACE_STATE: RefCell<TraceState> = RefCell::new(TraceState::default());
}

/// Saved recording flag restored on destruction, intended for nested same-thread use.
///
/// recording_scope enables collection and stores the previous flag here. Drop
/// writes that flag to the thread-local collector of the thread executing Drop;
/// it does not clear counts. Use guards in reverse creation order on their origin
/// thread, including during unwinding, or prefer with_recording.
///
/// The type contains only a bool, so its auto traits do not prevent moving it to
/// another thread. Such a drop changes that other thread and leaves the origin
/// enabled. Out-of-order drops can disable a live inner scope or leave recording
/// enabled after all guards are gone. Forgetting a guard likewise skips restoration.
/// These usages are not checked or converted into typed errors.
pub struct RecordingGuard {
    /// Recording flag captured at scope entry; no thread identity or nesting token is stored.
    previous: bool,
}

/// Restore the saved flag on the thread that executes destruction.
impl Drop for RecordingGuard {
    /// Write previous to the current thread-local flag, leaving all counts intact.
    ///
    /// No originating-thread or nesting-order check is made. RefCell borrowing and
    /// thread-local access retain their panic behavior, including during unwinding.
    fn drop(&mut self) {
        TRACE_STATE.with(|state| state.borrow_mut().recording = self.previous);
    }
}

/// Clear both counter groups on the current thread without changing recording state.
///
/// Clears live entries while retaining HashMap capacity and replaces rational
/// statistics with zeroed defaults. Existing owned snapshots are unaffected; this
/// does not end a scope or guarantee that allocated collector storage is released.
pub fn reset() {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        state.counts.clear();
        state.rational = RationalTraceStats::default();
    });
}

/// Enable recording on the current thread and return a guard saving its prior flag.
///
/// Existing counters remain intact, so nested or successive scopes accumulate.
/// The guard must be dropped on this thread in reverse creation order; its type
/// does not enforce this. An immediately discarded guard restores the flag at once.
/// The call is not a reset, snapshot or resource-budget reservation.
pub fn recording_scope() -> RecordingGuard {
    let previous = TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        let previous = state.recording;
        state.recording = true;
        previous
    });
    RecordingGuard { previous }
}

/// Run a closure with recording enabled on this thread, returning its value unchanged.
///
/// A local guard restores the prior flag on normal return or unwinding; panics are
/// not caught. Counts are neither cleared before the closure nor rolled back after
/// a panic. Spawned threads have independent collectors and are not enabled by
/// this scope. Manual guard misuse inside the closure is not prevented.
pub fn with_recording<T>(f: impl FnOnce() -> T) -> T {
    let _guard = recording_scope();
    f()
}

/// Record one static label triple when this thread is recording.
///
/// Accepts arbitrary static strings, including empty labels and equal text at
/// different addresses. Borrows thread-local state even when disabled; active
/// calls increment the identity-keyed count, possibly allocating a map entry.
/// Argument expressions have already run before this function tests the flag.
/// Ordinary count overflow and RefCell/TLS failures are not typed diagnostics.
#[inline]
pub fn record(layer: &'static str, operation: &'static str, path: &'static str) {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if state.recording {
            record_active(&mut state.counts, layer, operation, path);
        }
    });
}

/// Increment one live identity-keyed label count in the supplied mutable map.
///
/// Does not check a recording flag; record owns that gate. A missing entry starts
/// at zero before ordinary u64 increment. Equal-text labels at different addresses
/// can allocate distinct entries; overflow is neither checked into a Result nor
/// saturated. This helper is cold/non-inlined but makes no measured cost guarantee.
#[cold]
#[inline(never)]
fn record_active(
    counts: &mut ActiveCounts,
    layer: &'static str,
    operation: &'static str,
    path: &'static str,
) {
    let key = ActiveDispatchKey {
        layer,
        operation,
        path,
    };
    *counts.entry(key).or_insert(0) += 1;
}

/// Record one sign-refinement precision bucket under the computable layer.
///
/// The caller supplies the operation label. Nonnegative precisions map to p>=0;
/// negative ranges have boundaries -32, -64, -128, -256, -512, -1024 and -2048,
/// with the lower endpoint included and upper endpoint excluded. Smaller values
/// map to p<-2048. Every i32 is classified; this does not run or certify refinement.
/// Delegates scope gating and storage to record without updating rational counters.
pub(crate) fn record_sign_refinement_precision(operation: &'static str, precision: i32) {
    let bucket = match precision {
        0.. => "p>=0",
        -32..=-1 => "-32<=p<0",
        -64..=-33 => "-64<=p<-32",
        -128..=-65 => "-128<=p<-64",
        -256..=-129 => "-256<=p<-128",
        -512..=-257 => "-512<=p<-256",
        -1024..=-513 => "-1024<=p<-512",
        -2048..=-1025 => "-2048<=p<-1024",
        _ => "p<-2048",
    };
    record("computable", operation, bucket);
}

/// Raise peak_operand_bits to the maximum of its old value and this BigUint bit length.
///
/// Zero contributes zero. This private report update has no recording gate and
/// measures integer magnitude width, not bytes allocated or a resource limit.
fn update_peak(stats: &mut RationalTraceStats, value: &BigUint) {
    stats.peak_operand_bits = stats.peak_operand_bits.max(value.bits());
}

/// Increment exactly one common-factor bucket for a supplied nonnegative value.
///
/// Zero and one use none; other powers of two use power_of_two regardless of size.
/// Remaining values use <=8, 9..=64 or >64 bit bins. The helper trusts its argument
/// as a reported divisor, has no scope gate, and does not change gcds or peak width.
/// Bucket increments use ordinary u64 arithmetic.
fn record_common_factor(stats: &mut RationalTraceStats, divisor: &BigUint) {
    if divisor.is_zero() || divisor.is_one() {
        stats.common_factors.none += 1;
    } else if divisor.trailing_zeros() == Some(divisor.bits() - 1) {
        stats.common_factors.power_of_two += 1;
    } else {
        match divisor.bits() {
            0..=8 => stats.common_factors.small += 1,
            9..=64 => stats.common_factors.medium += 1,
            _ => stats.common_factors.large += 1,
        }
    }
}

/// Increment the current thread's temporary-rational counter while recording.
///
/// This counts explicit instrumentation calls, not all allocations or live objects.
/// Does not update peak width, labels or reduction/GCD counters; overflow uses the
/// ordinary build-dependent u64 behavior.
pub fn record_rational_temporary() {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if state.recording {
            state.rational.temporary_rationals += 1;
        }
    });
}

/// Count a reported rational reduction and observe both part widths while recording.
///
/// Accepts any BigUint pair, including a zero denominator; it neither validates a
/// fraction nor performs reduction. Increments reductions then raises peak width.
/// No common-factor or dispatch-label counter is changed by this call.
pub fn record_rational_reduction(numerator: &BigUint, denominator: &BigUint) {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if state.recording {
            state.rational.reductions += 1;
            update_peak(&mut state.rational, numerator);
            update_peak(&mut state.rational, denominator);
        }
    });
}

/// Count a reported GCD, bin operand widths and classify the supplied divisor.
///
/// While recording, increments gcds, selects one width bin, updates the peak with
/// both operands and divisor, then increments a common-factor bucket. Any BigUint
/// values, including zeros or an incorrect divisor, are accepted without checking
/// the GCD relation. This is instrumentation, not another arithmetic authority.
/// Ordinary overflow can interrupt these sequential updates without rollback.
pub fn record_rational_gcd(left: &BigUint, right: &BigUint, divisor: &BigUint) {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if !state.recording {
            return;
        }
        let stats = &mut state.rational;
        stats.gcds += 1;
        let left_bits = left.bits();
        let right_bits = right.bits();
        let widest = left_bits.max(right_bits);
        let narrowest = left_bits.min(right_bits);
        if widest <= 64 {
            stats.gcd_operand_widths.both_u64 += 1;
        } else if widest <= 128 && narrowest <= 64 {
            stats.gcd_operand_widths.mixed_u64_u128 += 1;
        } else if widest <= 128 {
            stats.gcd_operand_widths.both_u128 += 1;
        } else {
            stats.gcd_operand_widths.arbitrary_precision += 1;
        }
        update_peak(stats, left);
        update_peak(stats, right);
        update_peak(stats, divisor);
        record_common_factor(stats, divisor);
    });
}

/// Count a known power-of-two common factor from exponent metadata while recording.
///
/// Shift zero increments none; every positive u64 shift increments power_of_two.
/// No power is materialized, so u64::MAX is accepted. This call changes neither
/// gcds nor operand-width/peak statistics and does not prove that a factor exists.
/// It shares the common-factor counters but not a one-to-one relation with GCD calls.
pub fn record_rational_power_of_two_common_factor(shift: u64) {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if !state.recording {
            return;
        }
        let stats = &mut state.rational;
        if shift == 0 {
            stats.common_factors.none += 1;
        } else {
            stats.common_factors.power_of_two += 1;
        }
    });
}

/// Zero only rational statistics on the current thread.
///
/// Preserves dispatch entries, map capacity and recording state. Existing copied
/// reports remain unchanged; no arithmetic or trace scope is reset.
pub fn reset_rational_stats() {
    TRACE_STATE.with(|state| state.borrow_mut().rational = RationalTraceStats::default());
}

/// Copy the current thread's rational statistics without clearing them.
///
/// Returns an independent fixed-size value regardless of whether recording is
/// enabled. Dispatch entries and recording state are untouched.
pub fn snapshot_rational_stats() -> RationalTraceStats {
    TRACE_STATE.with(|state| state.borrow().rational)
}

/// Return the current rational statistics and replace them with zeroed defaults.
///
/// Does not clear dispatch entries or disable recording. The returned value owns
/// no references to mutable collector state and remains valid after later updates.
pub fn take_rational_stats() -> RationalTraceStats {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        std::mem::take(&mut state.rational)
    })
}

/// Merge live identity-keyed counts by label text and return sorted report rows.
///
/// Consumes the supplied iterator, grouping into a BTreeMap ordered by layer,
/// operation and path contents before allocating the output vector. Equal text
/// from different addresses sums into one row; zero-count groups are retained.
/// Ordinary u64 aggregation may overflow. Whether input counters are preserved
/// or drained on failure depends on the caller's iterator, not this helper.
fn collect_dispatch_counts(
    counts: impl IntoIterator<Item = (ActiveDispatchKey, u64)>,
) -> Vec<DispatchCount> {
    let mut grouped = BTreeMap::<DispatchKey, u64>::new();
    for (key, count) in counts {
        *grouped
            .entry(DispatchKey {
                layer: key.layer,
                operation: key.operation,
                path: key.path,
            })
            .or_insert(0) += count;
    }
    grouped
        .into_iter()
        .map(|(key, count)| DispatchCount {
            layer: key.layer,
            operation: key.operation,
            path: key.path,
            count,
        })
        .collect()
}

/// Copy dispatch counts into text-merged, lexicographically sorted report rows.
///
/// Borrows the current thread's live map without clearing it, including when
/// recording is disabled. Temporary grouping and output allocate. Aggregation
/// overflow can panic with checks enabled but leaves live entries intact; rational
/// statistics and recording state are not changed. The result is independent storage.
pub fn snapshot() -> Vec<DispatchCount> {
    TRACE_STATE.with(|state| {
        collect_dispatch_counts(
            state
                .borrow()
                .counts
                .iter()
                .map(|(key, count)| (*key, *count)),
        )
    })
}

/// Drain dispatch entries into a text-merged, lexicographically sorted report.
///
/// The map retains capacity, while rational statistics and recording state remain
/// unchanged. HashMap::drain removes remaining entries when its iterator drops,
/// including on aggregation unwind, so failure can clear the live map without
/// returning a report. No transactional rollback or typed overflow is provided.
pub fn take() -> Vec<DispatchCount> {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        collect_dispatch_counts(state.counts.drain())
    })
}

/// Copy dispatch rows and rational statistics without clearing either group.
///
/// Reads dispatch first, then rational statistics from this thread. It is not a
/// process-wide or cross-thread atomic snapshot. Dispatch grouping allocates and
/// can fail before the statistics are copied; recording state is unchanged.
pub fn snapshot_trace() -> TraceSnapshot {
    TraceSnapshot {
        dispatch: snapshot(),
        rational: snapshot_rational_stats(),
    }
}

/// Take dispatch rows, then rational statistics, clearing both on successful return.
///
/// Preserves recording state and map capacity. The operations are sequential: if
/// dispatch aggregation unwinds, its drain clears entries but rational statistics
/// have not yet been taken. This is not an atomic all-or-nothing report operation.
pub fn take_trace() -> TraceSnapshot {
    TraceSnapshot {
        dispatch: take(),
        rational: take_rational_stats(),
    }
}

/// Regression fixtures for text merging, scope state, counters and report grouping.
///
/// Tests use isolated thread-local collectors and retain exact count assertions.
/// The debug overflow fixture seeds counters directly rather than producing an
/// unbounded event stream. These finite checks do not prove coverage of all
/// instrumented paths, report labels, resource costs or arbitrary scope misuse.
#[cfg(test)]
mod tests {
    use super::*;

    /// Check disabled recording, repeated-path aggregation and draining after one scope.
    ///
    /// Requires two report rows and count two for the repeated real/sin path, then an
    /// empty snapshot. The second row's exact label/count is not independently asserted.
    #[test]
    fn dispatch_trace_records_only_inside_scope() {
        reset();
        record("real", "sin", "ignored");
        assert!(snapshot().is_empty());

        with_recording(|| {
            record("real", "sin", "path");
            record("real", "sin", "path");
            record("computable", "sin", "other");
        });

        let counts = take();
        assert_eq!(counts.len(), 2);
        assert!(counts.iter().any(|entry| {
            entry.layer == "real"
                && entry.operation == "sin"
                && entry.path == "path"
                && entry.count == 2
        }));
        assert!(snapshot().is_empty());
    }

    /// Require equal text from distinct static addresses to merge into one count-two row.
    ///
    /// Separately allocated label slices prevent pointer identity from serving as text
    /// equality. The fixture checks the complete returned vector, not just its length.
    #[test]
    fn dispatch_snapshot_merges_equal_labels_from_distinct_static_storage() {
        /// Six distinct static byte slices with identical UTF-8 text, used as two label triples.
        static LABELS: [[u8; 4]; 6] = [*b"same"; 6];
        let label = |index: usize| std::str::from_utf8(&LABELS[index]).unwrap();
        let first = (label(0), label(1), label(2));
        let second = (label(3), label(4), label(5));
        assert!(!std::ptr::eq(first.0, second.0));

        reset();
        with_recording(|| {
            record(first.0, first.1, first.2);
            record(second.0, second.1, second.2);
        });

        assert_eq!(
            take(),
            vec![DispatchCount {
                layer: "same",
                operation: "same",
                path: "same",
                count: 2,
            }]
        );
    }

    /// Pin lexicographic output, equal-text merging, repeatable snapshots and destructive take.
    ///
    /// The expected four rows include empty labels and a merged count nine. Reuses the
    /// collector for a fresh event and checks that an earlier owned snapshot is unchanged.
    #[test]
    fn dispatch_snapshot_preserves_order_and_counts_until_taken() {
        /// Two distinct static slices with equal text for content-merge and ordering checks.
        static LABELS: [[u8; 4]; 2] = [*b"same"; 2];
        let first = std::str::from_utf8(&LABELS[0]).unwrap();
        let second = std::str::from_utf8(&LABELS[1]).unwrap();
        assert!(!std::ptr::eq(first, second));
        reset();
        assert!(snapshot().is_empty());
        with_recording(|| {
            for (layer, operation, path, count) in [
                ("z", "", "", 5),
                ("a", "z", "", 3),
                ("a", "", second, 7),
                ("a", "", first, 2),
                ("a", "", "", 11),
            ] {
                for _ in 0..count {
                    record(layer, operation, path);
                }
            }
        });
        let expected: Vec<_> = [
            ("a", "", "", 11),
            ("a", "", "same", 9),
            ("a", "z", "", 3),
            ("z", "", "", 5),
        ]
        .into_iter()
        .map(|(layer, operation, path, count)| DispatchCount {
            layer,
            operation,
            path,
            count,
        })
        .collect();
        let retained = snapshot();
        assert_eq!(retained, expected);
        assert_eq!(snapshot(), expected);
        assert_eq!(take(), expected);
        assert!(take().is_empty());
        with_recording(|| record("new", "", ""));
        assert_eq!(take()[0].count, 1);
        assert_eq!(retained, expected);
        assert!(snapshot().is_empty());
    }

    /// Check debug aggregation-overflow unwind drains entries and restores scope state.
    ///
    /// Seeds two distinct live identities with equal text at u64::MAX and one. Taking
    /// them must panic with overflow checks, leave no dispatch entries, ignore a later
    /// out-of-scope event and accept a fresh scoped event. This debug-only fixture does
    /// not test release wrapping, transactional rational statistics or counter exhaustion.
    #[cfg(debug_assertions)]
    #[test]
    fn take_clears_counts_and_restores_scope_after_aggregation_unwinds() {
        /// Two equal-text identities whose seeded counts overflow during debug aggregation.
        static LABELS: [[u8; 4]; 2] = [*b"same"; 2];
        let first = std::str::from_utf8(&LABELS[0]).unwrap();
        let second = std::str::from_utf8(&LABELS[1]).unwrap();
        assert!(!std::ptr::eq(first, second));
        reset();

        let result = std::panic::catch_unwind(|| {
            with_recording(|| {
                // Reach aggregation overflow without an unbounded event stream.
                TRACE_STATE.with(|state| {
                    let mut state = state.borrow_mut();
                    for (path, count) in [(first, u64::MAX), (second, 1)] {
                        state.counts.insert(
                            ActiveDispatchKey {
                                layer: "test",
                                operation: "unwind",
                                path,
                            },
                            count,
                        );
                    }
                    assert_eq!(state.counts.len(), 2);
                });
                drop(take());
            });
        });
        assert!(result.is_err());
        assert!(snapshot().is_empty());
        record("test", "unwind", "outside");
        assert!(snapshot().is_empty());
        with_recording(|| record("test", "unwind", "fresh"));
        assert_eq!(
            take(),
            vec![DispatchCount {
                layer: "test",
                operation: "unwind",
                path: "fresh",
                count: 1,
            }]
        );
        assert!(take().is_empty());
    }

    /// Pin three refinement events to their operation and precision labels.
    ///
    /// Records nonnegative attempt precision and negative attempt/decision precision
    /// in the first negative bucket, requiring each path count one. It does not
    /// exercise every boundary or establish that refinement actually occurred.
    #[test]
    fn sign_refinement_trace_records_attempt_and_decision_precision_buckets() {
        reset();
        with_recording(|| {
            record_sign_refinement_precision("sign_until_attempt_precision", 0);
            record_sign_refinement_precision("sign_until_attempt_precision", -16);
            record_sign_refinement_precision("sign_until_decision_precision", -16);
        });

        let snapshot = take_trace();
        assert_eq!(
            snapshot.path_count("computable", "sign_until_attempt_precision", "p>=0"),
            1
        );
        assert_eq!(
            snapshot.path_count("computable", "sign_until_attempt_precision", "-32<=p<0"),
            1
        );
        assert_eq!(
            snapshot.path_count("computable", "sign_until_decision_precision", "-32<=p<0"),
            1
        );
    }

    /// Exercise rational construction/addition and require nonzero recorder statistics.
    ///
    /// Checks positive temporary, reduction, GCD and peak counters, plus equality of
    /// GCD count and the sum of width bins. This fixture does not pin each operation
    /// count or independently measure allocations; it relies on instrumented arithmetic.
    #[test]
    fn rational_trace_records_reductions_and_gcds() {
        use crate::Rational;

        reset();
        with_recording(|| {
            let left = Rational::fraction(6, 8).unwrap();
            let right = Rational::fraction(9, 10).unwrap();
            let _ = left + right;
        });

        let stats = take_rational_stats();
        assert!(stats.temporary_rationals > 0);
        assert!(stats.reductions > 0);
        assert!(stats.gcds > 0);
        assert_eq!(
            stats.gcds,
            stats.gcd_operand_widths.both_u64
                + stats.gcd_operand_widths.mixed_u64_u128
                + stats.gcd_operand_widths.both_u128
                + stats.gcd_operand_widths.arbitrary_precision
        );
        assert!(stats.peak_operand_bits > 0);
    }

    /// Pin dispatch counts for selected native and wide known-dyadic quotient operations.
    ///
    /// Builds finite rational and affine fixtures before recording, then requires
    /// word-cross-cancel count two, wide-cross-cancel count one and affine-factor
    /// count two. It checks routing labels, not numerical results or timing.
    #[test]
    fn known_dyadic_quotient_trace_distinguishes_word_and_wide_results() {
        use crate::Rational;

        let numerator = Rational::fraction(5, 32).unwrap();
        let denominator = Rational::fraction(11, 64).unwrap();
        let large_scale_denominator = Rational::from_bigint_fraction(
            num::BigInt::from(3_u8),
            num::BigUint::from(1_u8) << 200,
        )
        .unwrap();
        let wide_origin = Rational::from_bigint(num::BigInt::from(1_u8) << 220_usize);
        let delta = Rational::fraction(35, 64).unwrap();
        let affine_numerator = Rational::fraction(55, 32).unwrap();
        let affine_denominator = Rational::new(45);

        reset();
        with_recording(|| {
            Rational::quotient_known_dyadic(&numerator, &denominator).unwrap();
            Rational::quotient_known_dyadic(&Rational::one(), &large_scale_denominator).unwrap();
            Rational::parameterized_point2_known_dyadic(
                [&wide_origin, &wide_origin],
                [&delta, &delta],
                &affine_numerator,
                &affine_denominator,
            )
            .unwrap();
        });

        let snapshot = take_trace();
        assert_eq!(
            snapshot.path_count("rational", "div", "known-dyadic-word-cross-cancel"),
            2
        );
        assert_eq!(
            snapshot.path_count("rational", "div", "known-dyadic-cross-cancel"),
            1
        );
        assert_eq!(
            snapshot.path_count("rational", "div", "known-dyadic-affine-factor-cross-cancel"),
            2
        );
    }

    /// Check a unified report's grouped counts and independent rational counter.
    ///
    /// Records scalar, predicate and matrix labels, verifies path/operation/layer
    /// queries and selected summary rows, then takes the report and requires both
    /// counter groups empty. Label examples do not validate those downstream kernels.
    #[test]
    fn unified_trace_snapshot_groups_cross_stack_counts() {
        reset();
        with_recording(|| {
            record("hyperlimit", "resolve_real_sign", "structural-real-facts");
            record("hyperlimit", "resolve_real_sign", "exact-predicate");
            record("hyperlattice_matrix", "query", "matrix4-structural-facts");
            record("hyperlattice_matrix", "query", "matrix4-structural-facts");
            record("real", "detailed_facts", "pi-like");
            record_rational_temporary();
        });

        let snapshot = snapshot_trace();
        assert_eq!(
            snapshot.path_count("hyperlattice_matrix", "query", "matrix4-structural-facts"),
            2
        );
        assert_eq!(
            snapshot.operation_count("hyperlimit", "resolve_real_sign"),
            2
        );
        assert_eq!(snapshot.layer_count("hyperlattice_matrix"), 2);
        assert_eq!(snapshot.rational.temporary_rationals, 1);

        let operations = snapshot.operation_summaries();
        assert!(operations.iter().any(|entry| {
            entry.layer == "hyperlimit"
                && entry.operation == "resolve_real_sign"
                && entry.count == 2
        }));

        let layers = snapshot.layer_summaries();
        assert!(
            layers
                .iter()
                .any(|entry| { entry.layer == "real" && entry.count == 1 })
        );

        let taken = take_trace();
        assert_eq!(taken.layer_count("hyperlimit"), 2);
        assert!(snapshot_trace().dispatch.is_empty());
        assert_eq!(snapshot_trace().rational, RationalTraceStats::default());
    }

    /// Pin the heuristic correlation totals for eight cross-stack label examples.
    ///
    /// The assertions document overlaps among reporting categories and copied rational
    /// counters, including zero reduction/GCD totals. They are a finite classifier
    /// fixture, not semantic certification that the labels describe actual operations.
    #[test]
    fn correlation_summary_groups_exact_geometry_ladder_events() {
        reset();
        with_recording(|| {
            record("hyperlimit", "orient2d", "exact-rational-kernel");
            record("hyperlattice_matrix", "query", "matrix4-structural-facts");
            record("real", "exact_set_facts", "scan");
            record("real", "approximation", "cached-f64-hit");
            record("real", "domain_facts", "sqrt-domain-positive");
            record("real", "to_f64_lossy", "lossy-export");
            record("real", "certified_sign", "bounded-refinement");
            record("hyperlimit", "resolve_real_sign", "unknown-fallback");
            record_rational_temporary();
            record_rational_power_of_two_common_factor(3);
        });

        let summary = snapshot_trace().correlation_summary();
        assert_eq!(summary.dispatch_events, 8);
        assert_eq!(summary.predicate_events, 2);
        assert_eq!(summary.linear_algebra_events, 1);
        assert_eq!(summary.object_fact_events, 1);
        assert_eq!(summary.scalar_fact_events, 3);
        assert_eq!(summary.detailed_fact_events, 2);
        assert_eq!(summary.unknown_fact_events, 1);
        assert_eq!(summary.exact_rational_kind_events, 2);
        assert_eq!(summary.sign_or_zero_query_events, 3);
        assert_eq!(summary.exact_reducer_events, 2);
        assert_eq!(summary.approximation_events, 2);
        assert_eq!(summary.approximation_start_events, 1);
        assert_eq!(summary.approximation_cache_events, 1);
        assert_eq!(summary.refinement_events, 1);
        assert_eq!(summary.predicate_decision_stage_events, 2);
        assert_eq!(summary.fallback_or_abort_events, 1);
        assert_eq!(summary.rational_temporaries, 1);
        assert_eq!(summary.rational_reductions, 0);
        assert_eq!(summary.rational_gcds, 0);
    }

    /// Check independent recorder counts on eight threads and join their owned reports.
    ///
    /// Thread i emits i+1 dispatch events and temporary-rational calls inside its own
    /// scope. Each report must retain those counts. Guards stay on their creating
    /// threads; this does not test or authorize moving a RecordingGuard between threads.
    #[test]
    fn concurrent_recording_scopes_keep_independent_snapshots() {
        let threads = (0..8)
            .map(|index| {
                std::thread::spawn(move || {
                    reset();
                    with_recording(|| {
                        for _ in 0..=index {
                            record("real", "parallel", "thread-local");
                            record_rational_temporary();
                        }
                    });
                    take_trace()
                })
            })
            .collect::<Vec<_>>();

        for (index, thread) in threads.into_iter().enumerate() {
            let snapshot = thread.join().expect("recording thread panicked");
            assert_eq!(snapshot.layer_count("real"), index as u64 + 1);
            assert_eq!(snapshot.rational.temporary_rationals, index as u64 + 1);
        }
    }
}
