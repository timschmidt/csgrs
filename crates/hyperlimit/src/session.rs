//! Session-level exact geometry preparation.
//!
//! `ExactGeometrySession` is the first shared object boundary for construction
//! graph work. It carries predicate policy and an explicit construction version
//! while continuing to return borrowed prepared predicate objects. Higher crates
//! keep topology storage, approximate views, and invalidation rules; this module
//! gives them a small exact-kernel session to thread through repeated predicate
//! preparation.

use crate::classify::{
    Aabb2Intersection, Aabb2PointLocation, LineSide, PlaneSide, PointSegmentLocation,
    SegmentIntersection, TriangleLocation,
};
use crate::geometry::Point2;
use crate::predicate::{
    PredicateApiSemantics, PredicateCertificate, PredicateOutcome, PredicatePolicy,
    PredicateReport,
};
use crate::predicates::aabb::PreparedAabb2;
use crate::predicates::segment::PreparedSegment2;
use crate::predicates::triangle::PreparedTriangle2;
use crate::{
    Plane3, Point3, PreparedIncircle2, PreparedInsphere3, PreparedLine2, PreparedOrientedPlane3,
    PreparedPlane3, Sign,
};

/// Monotone construction version carried by an [`ExactGeometrySession`].
///
/// The value is intentionally opaque. Domain crates can copy it into cached
/// approximate views, broad-phase bins, or construction certificates and compare
/// later versions before reusing those caches. This follows Yap's object-level
/// exact-geometric-computation model: cached facts are advisory and conservative,
/// and validity belongs to the object/session that produced them; see Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[derive(Clone, Copy, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ConstructionVersion(u64);

impl ConstructionVersion {
    /// The initial construction version.
    pub const ZERO: Self = Self(0);

    /// Return the raw version number for serialization or diagnostics.
    pub const fn get(self) -> u64 {
        self.0
    }

    /// Return the next version, saturating at `u64::MAX`.
    pub const fn next(self) -> Self {
        Self(self.0.saturating_add(1))
    }
}

/// Freshness of session-bound cached construction data.
///
/// This enum is diagnostic metadata only. A stale cache is not a topology
/// decision; it tells callers to recompute an approximate view, conservative
/// fact package, or retained predicate report before using it for scheduling.
/// The split mirrors Yap's exact-geometric-computation discipline: cache
/// validity and approximate views are managed outside exact predicates, while
/// predicates remain the source of combinatorial truth.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConstructionFreshness {
    /// The cache was produced for the session's current construction version.
    Current,
    /// The cache's source version does not match the session's current version.
    StaleSource {
        /// Version stored in the cache.
        cached: ConstructionVersion,
        /// Version currently held by the session.
        current: ConstructionVersion,
    },
    /// A predicate certificate attached to the cache was produced under a
    /// different construction version.
    StaleCertificate {
        /// Version stored in the certificate.
        cached: ConstructionVersion,
        /// Version currently held by the session.
        current: ConstructionVersion,
    },
    /// A dependency version is newer than the session version.
    ///
    /// This generally indicates invalid caller metadata: a session cannot
    /// validate a cache that depends on a construction state it has not reached.
    FutureDependency {
        /// Future dependency version found in the dependency list.
        dependency: ConstructionVersion,
        /// Version currently held by the session.
        current: ConstructionVersion,
    },
}

impl ConstructionFreshness {
    /// Return whether the cache is current.
    pub const fn is_current(self) -> bool {
        matches!(self, Self::Current)
    }
}

/// Expected payoff for retaining a prepared fact or predicate cache.
///
/// The values are abstract work units, not wall-clock timings. Higher crates can
/// map them to their own operation counters, arena costs, or hot-loop weights
/// while `hyperlimit` stays independent of topology storage and allocation
/// policy. This follows Yap's exact-geometric-computation split: geometric
/// objects should preserve structural information that can avoid repeated exact
/// refinement, but arithmetic truth still comes from exact predicates. See Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CachePayoff {
    warm_up_work: u32,
    expected_reuse_count: u32,
    saved_work_per_reuse: u32,
}

impl CachePayoff {
    /// Build cache payoff metadata.
    ///
    /// Returns `None` when no reuse is expected or no per-reuse work is saved.
    /// Such a cache is still legal for callers to create, but it should not be
    /// advertised as a profitable structural fact package.
    pub const fn new(
        warm_up_work: u32,
        expected_reuse_count: u32,
        saved_work_per_reuse: u32,
    ) -> Option<Self> {
        if expected_reuse_count == 0 || saved_work_per_reuse == 0 {
            return None;
        }
        Some(Self {
            warm_up_work,
            expected_reuse_count,
            saved_work_per_reuse,
        })
    }

    /// Return the construction or preparation work needed before reuse.
    pub const fn warm_up_work(self) -> u32 {
        self.warm_up_work
    }

    /// Return the expected number of future reuses.
    pub const fn expected_reuse_count(self) -> u32 {
        self.expected_reuse_count
    }

    /// Return the abstract work expected to be saved by each reuse.
    pub const fn saved_work_per_reuse(self) -> u32 {
        self.saved_work_per_reuse
    }

    /// Return the total expected work saved before subtracting warm-up work.
    pub const fn gross_saved_work(self) -> u32 {
        self.expected_reuse_count
            .saturating_mul(self.saved_work_per_reuse)
    }

    /// Return the net expected work saved after warm-up work.
    pub const fn net_saved_work(self) -> i64 {
        self.gross_saved_work() as i64 - self.warm_up_work as i64
    }

    /// Return whether the cache is expected to repay its warm-up cost.
    pub const fn is_profitable(self) -> bool {
        self.gross_saved_work() >= self.warm_up_work
    }
}

/// Versioned approximate view owned by an edge adapter or domain crate.
///
/// Approximate views are deliberately separate from exact predicate inputs.
/// They can cache rendering, IO, or broad-phase coordinates together with a
/// source [`ConstructionVersion`], precision, and an absolute error budget, but
/// they do not certify topology. This is the Yap EGC boundary in data form:
/// approximate numerical views are useful accelerators and presentation data,
/// while exact predicates remain responsible for combinatorial decisions; see
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CachedApproximateView<T> {
    value: T,
    source_version: ConstructionVersion,
    precision_bits: u32,
    max_abs_error: f64,
}

impl<T> CachedApproximateView<T> {
    /// Build a cached approximate view.
    ///
    /// Returns `None` if the precision is zero or if the error is negative,
    /// infinite, or NaN. This keeps primitive-float policy at the edge adapter
    /// instead of letting invalid approximate metadata leak into exact kernels.
    pub fn new(
        value: T,
        source_version: ConstructionVersion,
        precision_bits: u32,
        max_abs_error: f64,
    ) -> Option<Self> {
        if precision_bits == 0 || !max_abs_error.is_finite() || max_abs_error < 0.0 {
            return None;
        }
        Some(Self {
            value,
            source_version,
            precision_bits,
            max_abs_error,
        })
    }

    /// Return the approximate value.
    pub const fn value(&self) -> &T {
        &self.value
    }

    /// Return the source construction version.
    pub const fn source_version(&self) -> ConstructionVersion {
        self.source_version
    }

    /// Return the advertised binary precision of this approximate view.
    pub const fn precision_bits(&self) -> u32 {
        self.precision_bits
    }

    /// Return the maximum absolute error claimed by the edge adapter.
    pub const fn max_abs_error(&self) -> f64 {
        self.max_abs_error
    }

    /// Return whether this view was produced from the session's current version.
    pub fn is_current_for(&self, session: ExactGeometrySession) -> bool {
        self.freshness_for(session).is_current()
    }

    /// Return the API semantic class for approximate views.
    ///
    /// Approximate views are explicit edge artifacts for rendering, IO, and
    /// broad-phase adapters. They are therefore approximation-forcing metadata,
    /// not predicate certificates.
    pub const fn api_semantics(&self) -> PredicateApiSemantics {
        PredicateApiSemantics::ApproximationForcing
    }

    /// Return freshness diagnostics for this approximate view.
    pub fn freshness_for(&self, session: ExactGeometrySession) -> ConstructionFreshness {
        freshness_from_source(self.source_version, session.version())
    }
}

/// Versioned construction certificate for a predicate-derived fact.
///
/// The certificate binds a predicate route to the construction version under
/// which it was produced. It intentionally does not store geometry ownership or
/// approximate coordinates; product crates can attach it to their own graph
/// nodes while `hyperlimit` remains the exact predicate layer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConstructionCertificate {
    version: ConstructionVersion,
    predicate: PredicateCertificate,
}

impl ConstructionCertificate {
    /// Build a certificate for a versioned predicate route.
    pub const fn new(version: ConstructionVersion, predicate: PredicateCertificate) -> Self {
        Self { version, predicate }
    }

    /// Build a certificate from a predicate report and source version.
    pub const fn from_report<T>(version: ConstructionVersion, report: &PredicateReport<T>) -> Self {
        Self {
            version,
            predicate: report.certificate,
        }
    }

    /// Return the construction version under which the certificate was created.
    pub const fn version(self) -> ConstructionVersion {
        self.version
    }

    /// Return the predicate route certified for this construction version.
    pub const fn predicate(self) -> PredicateCertificate {
        self.predicate
    }

    /// Return the API semantic class implied by the stored predicate route.
    pub const fn api_semantics(self) -> PredicateApiSemantics {
        self.predicate.api_semantics()
    }

    /// Return whether this certificate was produced by the current session.
    pub fn is_current_for(self, session: ExactGeometrySession) -> bool {
        self.freshness_for(session).is_current()
    }

    /// Return freshness diagnostics for this certificate.
    pub fn freshness_for(self, session: ExactGeometrySession) -> ConstructionFreshness {
        match freshness_from_source(self.version, session.version()) {
            ConstructionFreshness::Current => ConstructionFreshness::Current,
            ConstructionFreshness::StaleSource { cached, current } => {
                ConstructionFreshness::StaleCertificate { cached, current }
            }
            other => other,
        }
    }
}

/// Construction dependency versions retained beside cached facts.
///
/// The dependency list is intentionally only a list of versions, not object
/// handles. Domain crates own object identity, graph edges, and invalidation
/// policy; `hyperlimit` only provides a compact exact-kernel data shape they
/// can attach to those objects. This follows Yap's package split in exact
/// geometric computation: geometry packages should carry enough provenance to
/// reuse exact facts, while arithmetic packages remain responsible for exact
/// number semantics.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ConstructionDependencies {
    versions: Vec<ConstructionVersion>,
}

impl ConstructionDependencies {
    /// Build an empty dependency list.
    pub const fn new() -> Self {
        Self {
            versions: Vec::new(),
        }
    }

    /// Build dependencies from an iterator of construction versions.
    pub fn from_versions<I>(versions: I) -> Self
    where
        I: IntoIterator<Item = ConstructionVersion>,
    {
        Self {
            versions: versions.into_iter().collect(),
        }
    }

    /// Append one dependency version.
    pub fn push(&mut self, version: ConstructionVersion) {
        self.versions.push(version);
    }

    /// Return the number of dependency versions.
    pub fn len(&self) -> usize {
        self.versions.len()
    }

    /// Return whether no dependencies are recorded.
    pub fn is_empty(&self) -> bool {
        self.versions.is_empty()
    }

    /// Return all dependency versions.
    pub fn versions(&self) -> &[ConstructionVersion] {
        &self.versions
    }

    /// Return the first dependency version newer than `current`, if any.
    pub fn first_future_version(
        &self,
        current: ConstructionVersion,
    ) -> Option<ConstructionVersion> {
        self.versions
            .iter()
            .copied()
            .find(|version| *version > current)
    }
}

/// Conservative facts with construction provenance.
///
/// `VersionedFacts` is a small carrier for structural facts that were computed
/// at construction, import, preparation, or kernel-selection time. It does not
/// interpret those facts and does not make them authoritative for topology.
/// A stale or absent fact must only cost performance; exact predicates still
/// decide geometry, following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VersionedFacts<F> {
    facts: F,
    source_version: ConstructionVersion,
    dependencies: ConstructionDependencies,
    certificate: Option<ConstructionCertificate>,
    payoff: Option<CachePayoff>,
}

/// A predicate report bound to a construction certificate.
///
/// This type is the session-level carrier for exact sign decisions and their
/// provenance. The report still contains the predicate outcome, including
/// explicit uncertainty. The certificate records the construction version and
/// semantic predicate route. Keeping these together follows Yap's exact
/// geometric computation guidance: applications can audit and cache certified
/// decisions without depending on `Real` internals or approximate coordinates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct VersionedPredicateReport<T> {
    report: PredicateReport<T>,
    certificate: ConstructionCertificate,
}

impl<T> VersionedPredicateReport<T> {
    /// Build a versioned predicate report.
    pub const fn new(report: PredicateReport<T>, certificate: ConstructionCertificate) -> Self {
        Self {
            report,
            certificate,
        }
    }

    /// Return the underlying predicate report.
    pub const fn report(&self) -> &PredicateReport<T> {
        &self.report
    }

    /// Return the construction certificate.
    pub const fn certificate(&self) -> ConstructionCertificate {
        self.certificate
    }

    /// Return the decided value, or `None` when the predicate was undecided.
    pub fn value(&self) -> Option<T>
    where
        T: Copy,
    {
        self.report.value()
    }

    /// Return whether the report's certificate matches the current session.
    pub fn is_current_for(&self, session: ExactGeometrySession) -> bool {
        self.freshness_for(session).is_current()
    }

    /// Return freshness diagnostics for this report.
    pub fn freshness_for(&self, session: ExactGeometrySession) -> ConstructionFreshness {
        self.certificate.freshness_for(session)
    }

    /// Return the API semantic class implied by the underlying report.
    pub const fn api_semantics(&self) -> PredicateApiSemantics {
        self.report.api_semantics()
    }
}

impl<F> VersionedFacts<F> {
    /// Build versioned facts without a predicate certificate.
    pub fn new(facts: F, source_version: ConstructionVersion) -> Self {
        Self {
            facts,
            source_version,
            dependencies: ConstructionDependencies::new(),
            certificate: None,
            payoff: None,
        }
    }

    /// Build versioned facts with dependencies and an optional certificate.
    pub fn from_parts(
        facts: F,
        source_version: ConstructionVersion,
        dependencies: ConstructionDependencies,
        certificate: Option<ConstructionCertificate>,
    ) -> Self {
        Self::from_parts_with_payoff(facts, source_version, dependencies, certificate, None)
    }

    /// Build versioned facts with dependencies, certificate, and cache payoff.
    pub fn from_parts_with_payoff(
        facts: F,
        source_version: ConstructionVersion,
        dependencies: ConstructionDependencies,
        certificate: Option<ConstructionCertificate>,
        payoff: Option<CachePayoff>,
    ) -> Self {
        Self {
            facts,
            source_version,
            dependencies,
            certificate,
            payoff,
        }
    }

    /// Attach cache payoff metadata to these facts.
    ///
    /// Payoff is advisory scheduling information. It must not be used as a
    /// correctness certificate; exact predicates still own topology decisions.
    pub fn with_payoff(mut self, payoff: CachePayoff) -> Self {
        self.payoff = Some(payoff);
        self
    }

    /// Return the stored conservative facts.
    pub const fn facts(&self) -> &F {
        &self.facts
    }

    /// Return the source construction version.
    pub const fn source_version(&self) -> ConstructionVersion {
        self.source_version
    }

    /// Return dependency versions carried with these facts.
    pub fn dependencies(&self) -> &ConstructionDependencies {
        &self.dependencies
    }

    /// Return the optional construction certificate.
    pub const fn certificate(&self) -> Option<ConstructionCertificate> {
        self.certificate
    }

    /// Return cache payoff metadata, if the producer supplied it.
    pub const fn payoff(&self) -> Option<CachePayoff> {
        self.payoff
    }

    /// Return the API semantic class for versioned fact carriers.
    ///
    /// Versioned facts populate reusable structural metadata. They can guide
    /// faster exact schedules, but they are not topology proofs unless an exact
    /// predicate report later certifies a decision.
    pub const fn api_semantics(&self) -> PredicateApiSemantics {
        PredicateApiSemantics::CachePopulating
    }

    /// Return whether these facts were computed for the session's current version.
    pub fn is_current_for(&self, session: ExactGeometrySession) -> bool {
        self.freshness_for(session).is_current()
    }

    /// Return freshness diagnostics for these facts.
    pub fn freshness_for(&self, session: ExactGeometrySession) -> ConstructionFreshness {
        let current = session.version();
        if self.source_version != current {
            return ConstructionFreshness::StaleSource {
                cached: self.source_version,
                current,
            };
        }
        if let Some(dependency) = self.dependencies.first_future_version(current) {
            return ConstructionFreshness::FutureDependency {
                dependency,
                current,
            };
        }
        self.certificate
            .map(|certificate| certificate.freshness_for(session))
            .unwrap_or(ConstructionFreshness::Current)
    }
}

fn freshness_from_source(
    cached: ConstructionVersion,
    current: ConstructionVersion,
) -> ConstructionFreshness {
    if cached == current {
        ConstructionFreshness::Current
    } else {
        ConstructionFreshness::StaleSource { cached, current }
    }
}

/// Shared exact predicate preparation session.
///
/// The session does not own points, rings, triangulations, or approximate
/// coordinates. Instead, it centralizes policy and versioning for prepared
/// predicate objects that borrow immutable inputs. This keeps `hyperlimit` at
/// the semantic boundary between exact Real predicates and product-crate
/// topology storage, matching Yap's separation between geometric object facts
/// and arithmetic packages.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactGeometrySession {
    policy: PredicatePolicy,
    version: ConstructionVersion,
}

impl ExactGeometrySession {
    /// Create a session using the strict default predicate policy.
    pub const fn new() -> Self {
        Self::with_policy(PredicatePolicy::STRICT)
    }

    /// Create a session with an explicit predicate policy.
    pub const fn with_policy(policy: PredicatePolicy) -> Self {
        Self {
            policy,
            version: ConstructionVersion::ZERO,
        }
    }

    /// Return the predicate policy used by this session.
    pub const fn policy(self) -> PredicatePolicy {
        self.policy
    }

    /// Return the semantic class for session predicate APIs.
    ///
    /// A session is policy-dependent: it carries the caller's exact/refinement
    /// policy into prepared predicates and versioned reports. Individual
    /// reports still expose their proof-producing or approximation-deferring
    /// certificate semantics.
    pub const fn api_semantics(self) -> PredicateApiSemantics {
        self.policy.api_semantics()
    }

    /// Return the current construction version.
    pub const fn version(self) -> ConstructionVersion {
        self.version
    }

    /// Advance the construction version.
    ///
    /// Higher crates should call this when they mutate owned construction
    /// graphs or invalidate approximate views. Existing prepared objects borrow
    /// immutable inputs and remain Rust-lifetime-safe, but their external
    /// broad-phase or approximation caches may be stale under the new version.
    pub fn advance_version(&mut self) -> ConstructionVersion {
        self.version = self.version.next();
        self.version
    }

    /// Build a versioned approximate view for the current session version.
    ///
    /// This helper exists for rendering, IO, and broad-phase adapters that need
    /// lossy coordinates with explicit validity metadata. Exact predicate
    /// methods never consume this type for topology decisions.
    pub fn approximate_view<T>(
        &self,
        value: T,
        precision_bits: u32,
        max_abs_error: f64,
    ) -> Option<CachedApproximateView<T>> {
        CachedApproximateView::new(value, self.version, precision_bits, max_abs_error)
    }

    /// Bind a predicate certificate to the current construction version.
    pub const fn construction_certificate(
        self,
        predicate: PredicateCertificate,
    ) -> ConstructionCertificate {
        ConstructionCertificate::new(self.version, predicate)
    }

    /// Bind a predicate report's certificate to the current construction version.
    pub const fn certificate_from_report<T>(
        self,
        report: &PredicateReport<T>,
    ) -> ConstructionCertificate {
        ConstructionCertificate::from_report(self.version, report)
    }

    /// Bind a predicate report to this session's construction version.
    pub fn versioned_report<T>(&self, report: PredicateReport<T>) -> VersionedPredicateReport<T> {
        let certificate = ConstructionCertificate::from_report(self.version, &report);
        VersionedPredicateReport::new(report, certificate)
    }

    /// Evaluate 2D orientation and bind the report to this session.
    pub fn orient2d_report(
        &self,
        a: &Point2,
        b: &Point2,
        c: &Point2,
    ) -> VersionedPredicateReport<Sign> {
        self.versioned_report(crate::orient2d_report_with_policy(a, b, c, self.policy))
    }

    /// Evaluate 3D orientation and bind the report to this session.
    pub fn orient3d_report(
        &self,
        a: &Point3,
        b: &Point3,
        c: &Point3,
        d: &Point3,
    ) -> VersionedPredicateReport<Sign> {
        self.versioned_report(crate::orient3d_report_with_policy(a, b, c, d, self.policy))
    }

    /// Evaluate the 2D in-circle predicate and bind the report to this session.
    pub fn incircle2d_report(
        &self,
        a: &Point2,
        b: &Point2,
        c: &Point2,
        d: &Point2,
    ) -> VersionedPredicateReport<Sign> {
        self.versioned_report(crate::incircle2d_report_with_policy(
            a,
            b,
            c,
            d,
            self.policy,
        ))
    }

    /// Evaluate the 3D in-sphere predicate and bind the report to this session.
    pub fn insphere3d_report(
        &self,
        a: &Point3,
        b: &Point3,
        c: &Point3,
        d: &Point3,
        e: &Point3,
    ) -> VersionedPredicateReport<Sign> {
        self.versioned_report(crate::insphere3d_report_with_policy(
            a,
            b,
            c,
            d,
            e,
            self.policy,
        ))
    }

    /// Package conservative facts with this session's current version.
    ///
    /// This is the exact predicate layer's neutral carrier for facts collected
    /// by higher-level construction graphs. The facts remain advisory; callers
    /// must route topology through exact predicates even when a fact is present.
    pub fn versioned_facts<F>(&self, facts: F) -> VersionedFacts<F> {
        VersionedFacts::new(facts, self.version)
    }

    /// Package conservative facts with dependencies and a construction certificate.
    pub fn versioned_facts_with_certificate<F>(
        &self,
        facts: F,
        dependencies: ConstructionDependencies,
        certificate: ConstructionCertificate,
    ) -> VersionedFacts<F> {
        VersionedFacts::from_parts(facts, self.version, dependencies, Some(certificate))
    }

    /// Package conservative facts with dependencies but no predicate certificate.
    pub fn versioned_facts_with_dependencies<F>(
        &self,
        facts: F,
        dependencies: ConstructionDependencies,
    ) -> VersionedFacts<F> {
        VersionedFacts::from_parts(facts, self.version, dependencies, None)
    }

    /// Package conservative facts with dependencies, certificate, and payoff.
    pub fn versioned_facts_with_payoff<F>(
        &self,
        facts: F,
        dependencies: ConstructionDependencies,
        certificate: Option<ConstructionCertificate>,
        payoff: CachePayoff,
    ) -> VersionedFacts<F> {
        VersionedFacts::from_parts_with_payoff(
            facts,
            self.version,
            dependencies,
            certificate,
            Some(payoff),
        )
    }

    /// Prepare a borrowed oriented line predicate.
    pub fn prepare_line2<'a>(&self, from: &'a Point2, to: &'a Point2) -> PreparedLine2<'a> {
        PreparedLine2::new(from, to)
    }

    /// Prepare a borrowed closed segment predicate.
    pub fn prepare_segment2<'a>(&self, start: &'a Point2, end: &'a Point2) -> PreparedSegment2<'a> {
        PreparedSegment2::new(start, end)
    }

    /// Prepare a borrowed triangle predicate using this session's policy.
    pub fn prepare_triangle2<'a>(
        &self,
        a: &'a Point2,
        b: &'a Point2,
        c: &'a Point2,
    ) -> PreparedTriangle2<'a> {
        PreparedTriangle2::with_policy(a, b, c, self.policy)
    }

    /// Prepare a borrowed axis-aligned box predicate.
    pub fn prepare_aabb2<'a>(&self, min: &'a Point2, max: &'a Point2) -> PreparedAabb2<'a> {
        PreparedAabb2::new(min, max)
    }

    /// Prepare a borrowed in-circle predicate.
    ///
    /// The prepared object owns no topology and borrows only the fixed sites.
    /// It caches lifted-circle coefficients plus exact structural facts, giving
    /// repeated triangulation and mesh-quality queries one session-level entry
    /// point for Yap-style prepared arithmetic dispatch.
    pub fn prepare_incircle2<'a>(
        &self,
        a: &'a Point2,
        b: &'a Point2,
        c: &'a Point2,
    ) -> PreparedIncircle2<'a> {
        PreparedIncircle2::new(a, b, c)
    }

    /// Prepare a borrowed in-sphere predicate.
    ///
    /// This is the 3D lifted-sphere companion to [`Self::prepare_incircle2`].
    /// Higher crates keep tetrahedral topology and cache invalidation policy;
    /// `hyperlimit` keeps exact predicate policy and reusable coefficient facts.
    pub fn prepare_insphere3<'a>(
        &self,
        a: &'a Point3,
        b: &'a Point3,
        c: &'a Point3,
        d: &'a Point3,
    ) -> PreparedInsphere3<'a> {
        PreparedInsphere3::new(a, b, c, d)
    }

    /// Prepare a borrowed explicit 3D plane predicate.
    ///
    /// This is the session-level entry point for reusing [`Plane3Facts`] and
    /// explicit plane coefficients across batches. Keeping the borrowed
    /// prepared object in `hyperlimit` preserves the abstraction boundary:
    /// higher crates own meshes, CSG cells, or curve topology, while this
    /// kernel owns exact predicate policy and structural scheduling facts.
    ///
    /// [`Plane3Facts`]: crate::Plane3Facts
    pub fn prepare_plane3<'a>(&self, plane: &'a Plane3) -> PreparedPlane3<'a> {
        PreparedPlane3::new(plane)
    }

    /// Prepare the oriented 3D plane through three points.
    ///
    /// The three-point plane is reduced once to an explicit [`Plane3`], then
    /// point queries reuse that exact object. This follows Yap's exact
    /// geometric computation boundary between geometric objects and arithmetic
    /// predicates; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn prepare_oriented_plane3(
        &self,
        a: &Point3,
        b: &Point3,
        c: &Point3,
    ) -> PreparedOrientedPlane3 {
        PreparedOrientedPlane3::new(a, b, c)
    }

    /// Classify a point against a prepared line using this session's policy.
    pub fn classify_prepared_line2(
        &self,
        line: &PreparedLine2<'_>,
        point: &Point2,
    ) -> PredicateOutcome<LineSide> {
        line.classify_point_with_policy(point, self.policy)
    }

    /// Classify a point against a prepared segment using this session's policy.
    pub fn classify_prepared_segment2_point(
        &self,
        segment: &PreparedSegment2<'_>,
        point: &Point2,
    ) -> PredicateOutcome<PointSegmentLocation> {
        segment.classify_point_with_policy(point, self.policy)
    }

    /// Classify two prepared segments using this session's policy.
    pub fn classify_prepared_segment2_intersection(
        &self,
        first: &PreparedSegment2<'_>,
        second: &PreparedSegment2<'_>,
    ) -> PredicateOutcome<SegmentIntersection> {
        first.classify_intersection_with_policy(second, self.policy)
    }

    /// Classify a point against a prepared triangle using this session's policy.
    pub fn classify_prepared_triangle2_point(
        &self,
        triangle: &PreparedTriangle2<'_>,
        point: &Point2,
    ) -> PredicateOutcome<TriangleLocation> {
        triangle.classify_point_with_policy(point, self.policy)
    }

    /// Return the cached orientation sign for a prepared triangle.
    pub fn prepared_triangle2_orientation(
        &self,
        triangle: &PreparedTriangle2<'_>,
    ) -> PredicateOutcome<Sign> {
        triangle.orientation()
    }

    /// Classify a point against a prepared AABB using this session's policy.
    pub fn classify_prepared_aabb2_point(
        &self,
        aabb: &PreparedAabb2<'_>,
        point: &Point2,
    ) -> PredicateOutcome<Aabb2PointLocation> {
        aabb.classify_point_with_policy(point, self.policy)
    }

    /// Classify two prepared AABBs using this session's policy.
    pub fn classify_prepared_aabb2_intersection(
        &self,
        first: &PreparedAabb2<'_>,
        second: &PreparedAabb2<'_>,
    ) -> PredicateOutcome<Aabb2Intersection> {
        first.classify_intersection_with_policy(second, self.policy)
    }

    /// Test a query point against a prepared in-circle predicate.
    pub fn test_prepared_incircle2(
        &self,
        incircle: &PreparedIncircle2<'_>,
        point: &Point2,
    ) -> PredicateOutcome<Sign> {
        incircle.test_point_with_policy(point, self.policy)
    }

    /// Test a query point against a prepared in-sphere predicate.
    pub fn test_prepared_insphere3(
        &self,
        insphere: &PreparedInsphere3<'_>,
        point: &Point3,
    ) -> PredicateOutcome<Sign> {
        insphere.test_point_with_policy(point, self.policy)
    }

    /// Classify a point against a prepared explicit plane using this session's policy.
    pub fn classify_prepared_plane3_point(
        &self,
        plane: &PreparedPlane3<'_>,
        point: &Point3,
    ) -> PredicateOutcome<PlaneSide> {
        plane.classify_point_with_policy(point, self.policy)
    }

    /// Classify a point against a prepared oriented plane using this session's policy.
    pub fn classify_prepared_oriented_plane3_point(
        &self,
        plane: &PreparedOrientedPlane3,
        point: &Point3,
    ) -> PredicateOutcome<PlaneSide> {
        plane.classify_point_with_policy(point, self.policy)
    }
}

impl Default for ExactGeometrySession {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::classify::{LineSide, PointSegmentLocation};

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(hyperreal::Real::from(x), hyperreal::Real::from(y))
    }

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(
            hyperreal::Real::from(x),
            hyperreal::Real::from(y),
            hyperreal::Real::from(z),
        )
    }

    #[test]
    fn session_prepares_predicates_with_shared_policy_and_version() {
        let mut session = ExactGeometrySession::default();
        assert_eq!(session.policy(), PredicatePolicy::STRICT);
        assert_eq!(session.version(), ConstructionVersion::ZERO);
        assert_eq!(session.advance_version().get(), 1);

        let origin = p2(0, 0);
        let x_axis = p2(4, 0);
        let y_axis = p2(0, 3);
        let inside = p2(1, 1);

        let line = session.prepare_line2(&origin, &x_axis);
        assert_eq!(
            session.classify_prepared_line2(&line, &y_axis).value(),
            Some(LineSide::Left)
        );

        let segment = session.prepare_segment2(&origin, &x_axis);
        assert!(segment.facts().has_sparse_support());
        assert_eq!(
            session
                .classify_prepared_segment2_point(&segment, &p2(2, 0))
                .value(),
            Some(PointSegmentLocation::OnSegment)
        );

        let triangle = session.prepare_triangle2(&origin, &x_axis, &y_axis);
        assert_eq!(
            session.prepared_triangle2_orientation(&triangle).value(),
            Some(Sign::Positive)
        );
        assert_eq!(
            session
                .classify_prepared_triangle2_point(&triangle, &inside)
                .value(),
            Some(TriangleLocation::Inside)
        );
    }

    #[test]
    fn session_prepared_aabb_preserves_zero_area_contact_semantics() {
        let session = ExactGeometrySession::default();
        let point_min = p2(2, 2);
        let point_max = p2(2, 2);
        let area_min = p2(0, 0);
        let area_max = p2(4, 4);
        let point = session.prepare_aabb2(&point_min, &point_max);
        let area = session.prepare_aabb2(&area_min, &area_max);

        assert!(point.facts().known_point());
        assert_eq!(
            session
                .classify_prepared_aabb2_point(&area, point.min())
                .value(),
            Some(Aabb2PointLocation::Inside)
        );
        assert_eq!(
            session
                .classify_prepared_aabb2_intersection(&point, &area)
                .value(),
            Some(Aabb2Intersection::Touching)
        );
    }

    #[test]
    fn session_prepares_planes_with_cached_structural_facts() {
        let session = ExactGeometrySession::default();
        let plane = Plane3::new(
            Point3::new(
                hyperreal::Real::from(0),
                hyperreal::Real::from(0),
                hyperreal::Real::from(1),
            ),
            hyperreal::Real::from(-2),
        );
        let prepared = session.prepare_plane3(&plane);

        assert_eq!(prepared.plane(), &plane);
        assert_eq!(prepared.facts(), plane.structural_facts());
        assert!(prepared.facts().normal_has_sparse_support());
        assert_eq!(prepared.facts().coefficient_zero_mask, 0b0011);
        assert_eq!(
            session
                .classify_prepared_plane3_point(&prepared, &p3(0, 0, 3))
                .value(),
            Some(PlaneSide::Above)
        );
        assert_eq!(
            session
                .classify_prepared_plane3_point(&prepared, &p3(0, 0, 1))
                .value(),
            Some(PlaneSide::Below)
        );
    }

    #[test]
    fn session_prepares_lifted_circle_and_sphere_predicates() {
        let session = ExactGeometrySession::default();
        let a = p2(1, 0);
        let b = p2(0, 1);
        let c = p2(-1, 0);
        let inside = p2(0, 0);
        let outside = p2(2, 0);
        let incircle = session.prepare_incircle2(&a, &b, &c);

        assert!(incircle.facts().fixed_coordinates_exact_rational);
        assert!(
            incircle
                .coefficient_facts()
                .coefficient_exact
                .all_exact_rational
        );
        assert_eq!(
            session.test_prepared_incircle2(&incircle, &inside).value(),
            crate::incircle2d(&a, &b, &c, &inside).value()
        );
        assert_eq!(
            session.test_prepared_incircle2(&incircle, &outside).value(),
            crate::incircle2d(&a, &b, &c, &outside).value()
        );

        let p = p3(0, 0, 0);
        let q = p3(1, 0, 0);
        let r = p3(0, 1, 0);
        let s = p3(0, 0, 1);
        let sphere_query = Point3::new(
            hyperreal::Real::from(hyperreal::Rational::fraction(1, 4).unwrap()),
            hyperreal::Real::from(hyperreal::Rational::fraction(1, 4).unwrap()),
            hyperreal::Real::from(hyperreal::Rational::fraction(1, 4).unwrap()),
        );
        let insphere = session.prepare_insphere3(&p, &q, &r, &s);

        assert!(insphere.facts().fixed_coordinates_exact_rational);
        assert!(
            insphere
                .coefficient_facts()
                .coefficient_exact
                .all_exact_rational
        );
        assert_eq!(
            session
                .test_prepared_insphere3(&insphere, &sphere_query)
                .value(),
            crate::insphere3d(&p, &q, &r, &s, &sphere_query).value()
        );
    }

    #[test]
    fn session_prepared_oriented_plane_matches_orient3d_boundary() {
        let session = ExactGeometrySession::default();
        let a = p3(0, 0, 0);
        let b = p3(1, 0, 0);
        let c = p3(0, 1, 0);
        let prepared = session.prepare_oriented_plane3(&a, &b, &c);
        let above = p3(0, 0, 1);
        let below = p3(0, 0, -1);
        let on_plane = p3(1, 1, 0);

        assert_eq!(prepared.facts(), prepared.plane().structural_facts());
        assert!(prepared.facts().normal_has_sparse_support());
        assert_eq!(
            session
                .classify_prepared_oriented_plane3_point(&prepared, &above)
                .value(),
            crate::classify_point_oriented_plane(&a, &b, &c, &above).value()
        );
        assert_eq!(
            session
                .classify_prepared_oriented_plane3_point(&prepared, &below)
                .value(),
            crate::classify_point_oriented_plane(&a, &b, &c, &below).value()
        );
        assert_eq!(
            session
                .classify_prepared_oriented_plane3_point(&prepared, &on_plane)
                .value(),
            Some(PlaneSide::On)
        );
    }

    #[test]
    fn session_versioned_approximate_views_are_edge_metadata_only() {
        let mut session = ExactGeometrySession::default();
        let view = session
            .approximate_view([1.0_f64, 2.0], 53, 0.25)
            .expect("valid finite approximation metadata");

        assert_eq!(view.value(), &[1.0, 2.0]);
        assert_eq!(view.source_version(), ConstructionVersion::ZERO);
        assert_eq!(view.precision_bits(), 53);
        assert_eq!(view.max_abs_error(), 0.25);
        assert_eq!(
            view.api_semantics(),
            PredicateApiSemantics::ApproximationForcing
        );
        assert!(view.is_current_for(session));
        assert_eq!(view.freshness_for(session), ConstructionFreshness::Current);

        session.advance_version();
        assert!(!view.is_current_for(session));
        assert_eq!(
            view.freshness_for(session),
            ConstructionFreshness::StaleSource {
                cached: ConstructionVersion::ZERO,
                current: session.version()
            }
        );
        assert!(session.approximate_view([0.0_f64, 0.0], 0, 0.0).is_none());
        assert!(
            session
                .approximate_view([0.0_f64, 0.0], 53, f64::NAN)
                .is_none()
        );
        assert!(session.approximate_view([0.0_f64, 0.0], 53, -1.0).is_none());
    }

    #[test]
    fn session_construction_certificates_track_predicate_route_and_version() {
        let mut session = ExactGeometrySession::default();
        let origin = p2(0, 0);
        let x_axis = p2(1, 0);
        let y_axis = p2(0, 1);
        let report =
            crate::orient2d_report_with_policy(&origin, &x_axis, &y_axis, session.policy());
        let certificate = session.certificate_from_report(&report);

        assert_eq!(certificate.version(), ConstructionVersion::ZERO);
        assert_eq!(certificate.predicate(), report.certificate);
        assert_eq!(certificate.api_semantics(), report.certificate.api_semantics());
        assert!(certificate.is_current_for(session));
        assert_eq!(
            certificate.freshness_for(session),
            ConstructionFreshness::Current
        );

        session.advance_version();
        assert!(!certificate.is_current_for(session));
        assert!(matches!(
            certificate.freshness_for(session),
            ConstructionFreshness::StaleCertificate { .. }
        ));
    }

    #[test]
    fn session_versioned_facts_carry_dependencies_and_certificates() {
        let mut session = ExactGeometrySession::default();
        let origin = p2(0, 0);
        let x_axis = p2(1, 0);
        let y_axis = p2(0, 1);
        let report =
            crate::orient2d_report_with_policy(&origin, &x_axis, &y_axis, session.policy());
        let certificate = session.certificate_from_report(&report);

        let mut dependencies = ConstructionDependencies::new();
        dependencies.push(ConstructionVersion::ZERO);
        dependencies.push(session.version());
        let versioned = session.versioned_facts_with_certificate(
            crate::geometry::triangle2_facts(&origin, &x_axis, &y_axis),
            dependencies,
            certificate,
        );

        assert!(versioned.is_current_for(session));
        assert_eq!(versioned.source_version(), ConstructionVersion::ZERO);
        assert_eq!(versioned.dependencies().len(), 2);
        assert_eq!(versioned.certificate(), Some(certificate));
        assert_eq!(versioned.facts().known_non_degenerate(), Some(true));
        assert_eq!(
            versioned.api_semantics(),
            PredicateApiSemantics::CachePopulating
        );
        assert_eq!(
            versioned.freshness_for(session),
            ConstructionFreshness::Current
        );

        session.advance_version();
        assert!(!versioned.is_current_for(session));
        assert!(matches!(
            versioned.freshness_for(session),
            ConstructionFreshness::StaleSource { .. }
        ));

        let dependency_list =
            ConstructionDependencies::from_versions([ConstructionVersion::ZERO, session.version()]);
        let facts = session.versioned_facts_with_dependencies((), dependency_list);
        assert!(facts.is_current_for(session));
        assert_eq!(facts.dependencies().versions().len(), 2);

        let future_dependency = ConstructionDependencies::from_versions([session.version().next()]);
        let invalid_facts = session.versioned_facts_with_dependencies((), future_dependency);
        assert_eq!(
            invalid_facts.freshness_for(session),
            ConstructionFreshness::FutureDependency {
                dependency: session.version().next(),
                current: session.version()
            }
        );
    }

    #[test]
    fn cache_payoff_documents_reuse_expectations_without_certifying_facts() {
        assert!(CachePayoff::new(3, 0, 4).is_none());
        assert!(CachePayoff::new(3, 4, 0).is_none());

        let marginal = CachePayoff::new(12, 3, 4).expect("equal payoff is profitable");
        assert_eq!(marginal.warm_up_work(), 12);
        assert_eq!(marginal.expected_reuse_count(), 3);
        assert_eq!(marginal.saved_work_per_reuse(), 4);
        assert_eq!(marginal.gross_saved_work(), 12);
        assert_eq!(marginal.net_saved_work(), 0);
        assert!(marginal.is_profitable());

        let unprofitable = CachePayoff::new(13, 3, 4).expect("nonzero reuse is valid metadata");
        assert_eq!(unprofitable.net_saved_work(), -1);
        assert!(!unprofitable.is_profitable());

        let saturated = CachePayoff::new(0, u32::MAX, u32::MAX)
            .expect("large abstract work counts saturate instead of overflowing");
        assert_eq!(saturated.gross_saved_work(), u32::MAX);
        assert_eq!(saturated.net_saved_work(), u32::MAX as i64);
    }

    #[test]
    fn versioned_facts_payoff_survives_freshness_diagnostics() {
        let mut session = ExactGeometrySession::default();
        let payoff = CachePayoff::new(5, 2, 8).expect("prepared triangle facts should repay");
        let facts = session.versioned_facts_with_payoff(
            "known-nondegenerate",
            ConstructionDependencies::from_versions([session.version()]),
            None,
            payoff,
        );

        assert_eq!(facts.payoff(), Some(payoff));
        assert_eq!(facts.freshness_for(session), ConstructionFreshness::Current);

        session.advance_version();
        assert_eq!(facts.payoff(), Some(payoff));
        assert!(matches!(
            facts.freshness_for(session),
            ConstructionFreshness::StaleSource { .. }
        ));

        let attached = session.versioned_facts("shared-scale").with_payoff(payoff);
        assert_eq!(attached.payoff(), Some(payoff));
        assert!(attached.is_current_for(session));
    }

    #[test]
    fn session_versioned_predicate_reports_bind_sign_decisions_to_versions() {
        let mut session = ExactGeometrySession::default();
        let origin = p2(0, 0);
        let x_axis = p2(1, 0);
        let y_axis = p2(0, 1);
        let report = session.orient2d_report(&origin, &x_axis, &y_axis);

        assert_eq!(report.value(), Some(Sign::Positive));
        assert_eq!(report.certificate().version(), ConstructionVersion::ZERO);
        assert_eq!(
            session.api_semantics(),
            PredicateApiSemantics::PolicyDependent
        );
        assert_eq!(
            report.api_semantics(),
            report.report().certificate.api_semantics()
        );
        assert_eq!(
            report.certificate().predicate(),
            report.report().certificate
        );
        assert!(report.is_current_for(session));
        assert_eq!(
            report.freshness_for(session),
            ConstructionFreshness::Current
        );

        session.advance_version();
        assert!(!report.is_current_for(session));
        assert!(matches!(
            report.freshness_for(session),
            ConstructionFreshness::StaleCertificate { .. }
        ));
    }
}
