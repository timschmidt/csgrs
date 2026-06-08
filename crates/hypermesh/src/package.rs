//! Bundled downstream handoff packages for exact mesh artifacts.
//!
//! [`ExactMeshHandoffPackage`] is a convenience envelope over independently
//! validated reports: retained-state audit, consumer readiness, exact surface
//! handoff, exact solid handoff, and lossy display/export view. It does not
//! systems may cache and route exact geometric artifacts, but cached packages
//! must replay against exact source objects instead of becoming unexamined
//! topology authority.

use super::{
    ApproximateMeshF64View, ExactMesh, ExactMeshAuditReport, ExactMeshConsumerReadinessReport,
    ExactSolidHandoffReport, ExactSurfaceHandoffReport, approximate_mesh_f64_view,
    audit_exact_mesh, exact_mesh_consumer_readiness, exact_solid_handoff, exact_surface_handoff,
};

/// Bundled report-bearing handoff package for common downstream mesh consumers.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactMeshHandoffPackage {
    /// Retained-state audit replayed before packaging.
    pub audit: ExactMeshAuditReport,
    /// Compact routing summary derived from the same source mesh.
    pub readiness: ExactMeshConsumerReadinessReport,
    /// Exact surface handoff when the mesh currently satisfies surface rules.
    pub surface: Option<ExactSurfaceHandoffReport>,
    /// Exact closed-solid handoff when the mesh currently satisfies solid rules.
    pub solid: Option<ExactSolidHandoffReport>,
    /// Lossy primitive-float view when display/export lowering currently replays.
    pub approximate_f64_view: Option<ApproximateMeshF64View>,
}

/// Consumer domain requested from an exact mesh handoff package.
///
/// Domain selection is explicit because surface evidence, solid evidence, and
/// approximate display/export evidence are not interchangeable. This follows
/// of implicit control-flow conventions such as "field is present enough."
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshConsumerDomain {
    /// Exact surface geometry, accepting boundary-allowed open surfaces.
    Surface,
    /// Exact closed-solid geometry for volumetric consumers.
    Solid,
    /// Lossy primitive-float display/export view.
    ApproximateF64View,
}

/// Summary of consumer domains carried by a handoff package.
///
/// This is a scheduler-friendly view over package availability. It keeps exact
/// geometry domains, closed-volume readiness, and lossy adapter domains
/// useful artifacts, but exact object evidence should remain distinguishable
/// in every consumer-facing boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactMeshDomainSummary {
    /// All domains present in stable package order.
    pub available_domains: Vec<ExactMeshConsumerDomain>,
    /// Exact geometry domains present in stable package order.
    pub exact_geometry_domains: Vec<ExactMeshConsumerDomain>,
    /// Lossy adapter domains present in stable package order.
    pub lossy_adapter_domains: Vec<ExactMeshConsumerDomain>,
    /// Number of exact geometry domains present.
    pub exact_geometry_count: usize,
    /// Number of lossy adapter domains present.
    pub lossy_adapter_count: usize,
    /// Whether a closed-volume domain is present.
    pub closed_volume_ready: bool,
}

/// Error returned when a retained domain summary no longer matches a package.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactMeshDomainSummaryError {
    /// The handoff package failed replay against the source mesh.
    Package(ExactMeshHandoffPackageError),
    /// A summary field disagrees with the package-derived value.
    SummaryMismatch {
        /// Name of the mismatched field.
        field: &'static str,
    },
    /// The requested consumer domain is not listed in this summary.
    MissingDomain {
        /// Domain requested by the caller.
        domain: ExactMeshConsumerDomain,
    },
    /// The summary has no exact geometry domain.
    MissingExactGeometry,
    /// The summary has no lossy adapter domain.
    MissingLossyAdapter,
    /// The summary has no closed-volume domain.
    MissingClosedVolume,
}

/// Freshness status for a retained domain summary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshDomainSummaryFreshness {
    /// The summary still matches the package.
    Current,
    /// The package failed replay against the source mesh.
    InvalidPackage,
    /// The summary disagrees with the package.
    StaleSummary,
}

impl ExactMeshDomainSummary {
    /// Return whether `domain` is listed in this summary.
    pub fn has_domain(&self, domain: ExactMeshConsumerDomain) -> bool {
        self.available_domains.contains(&domain)
    }

    /// Return whether at least one exact-geometry domain is listed.
    pub const fn has_exact_geometry(&self) -> bool {
        self.exact_geometry_count > 0
    }

    /// Return whether at least one lossy adapter domain is listed.
    pub const fn has_lossy_adapter(&self) -> bool {
        self.lossy_adapter_count > 0
    }

    /// Return the strongest exact-geometry domain listed in this summary.
    ///
    /// Closed-volume evidence is preferred over surface evidence because it
    /// carries stricter downstream semantics. Lossy adapter domains are never
    /// returned by this helper. This keeps scheduler preference aligned with
    /// while approximate representatives remain named adapter outputs.
    pub fn preferred_exact_geometry_domain(&self) -> Option<ExactMeshConsumerDomain> {
        if self.has_domain(ExactMeshConsumerDomain::Solid) {
            Some(ExactMeshConsumerDomain::Solid)
        } else if self.has_domain(ExactMeshConsumerDomain::Surface) {
            Some(ExactMeshConsumerDomain::Surface)
        } else {
            None
        }
    }

    /// Require and return the strongest exact-geometry domain in this summary.
    pub fn require_preferred_exact_geometry_domain(
        &self,
    ) -> Result<ExactMeshConsumerDomain, ExactMeshDomainSummaryError> {
        self.preferred_exact_geometry_domain()
            .ok_or(ExactMeshDomainSummaryError::MissingExactGeometry)
    }

    /// Validate this summary against `package`, then require the strongest exact domain.
    ///
    /// Use this when copied scheduler metadata may have crossed a queue or
    /// serialization boundary but the source mesh is not currently loaded. The
    /// summary must first replay from the package before the preferred exact
    /// routing facts are usable only while they replay from exact object
    /// evidence, and lossy adapter evidence is never a substitute for exact
    /// geometry.
    pub fn require_preferred_exact_geometry_domain_against_package(
        &self,
        package: &ExactMeshHandoffPackage,
    ) -> Result<ExactMeshConsumerDomain, ExactMeshDomainSummaryError> {
        self.validate_against_package(package)?;
        self.require_preferred_exact_geometry_domain()
    }

    /// Validate this summary through `package` against `mesh`, then require the strongest exact domain.
    ///
    /// This is the source-replayed counterpart to
    /// [`ExactMeshDomainSummary::require_preferred_exact_geometry_domain`].
    /// The package must replay from the exact mesh, the summary must replay
    /// from the package, and then closed-solid evidence is preferred over
    /// boundary-surface evidence.
    pub fn require_preferred_exact_geometry_domain_against_mesh(
        &self,
        package: &ExactMeshHandoffPackage,
        mesh: &ExactMesh,
    ) -> Result<ExactMeshConsumerDomain, ExactMeshDomainSummaryError> {
        self.validate_against_mesh(package, mesh)?;
        self.require_preferred_exact_geometry_domain()
    }

    /// Validate this summary against `package`, then return the preferred exact report.
    ///
    /// This combines copied-summary replay with typed package extraction. It
    /// prevents downstream schedulers from selecting a preferred exact domain
    /// and then manually unwrapping a different optional report. The guard is
    /// deliberately exact-only: lossy display/export views are never returned
    /// cached routing metadata to replay before it can authorize consumption
    /// of exact geometric evidence.
    pub fn preferred_exact_geometry_report_against_package<'a>(
        &self,
        package: &'a ExactMeshHandoffPackage,
    ) -> Result<ExactMeshDomainReportRef<'a>, ExactMeshDomainSummaryError> {
        let domain = self.require_preferred_exact_geometry_domain_against_package(package)?;
        package
            .domain_report(domain)
            .map_err(ExactMeshDomainSummaryError::Package)
    }

    /// Validate this summary through `package` against `mesh`, then return the preferred exact report.
    ///
    /// This is the strongest copied-summary extraction path. The package must
    /// replay from the exact mesh, the summary must replay from the package,
    /// and only then is the solid-over-surface preferred exact report exposed.
    pub fn preferred_exact_geometry_report_against_mesh<'a>(
        &self,
        package: &'a ExactMeshHandoffPackage,
        mesh: &ExactMesh,
    ) -> Result<ExactMeshDomainReportRef<'a>, ExactMeshDomainSummaryError> {
        let domain = self.require_preferred_exact_geometry_domain_against_mesh(package, mesh)?;
        package
            .domain_report(domain)
            .map_err(ExactMeshDomainSummaryError::Package)
    }

    /// Require a specific consumer domain in this summary.
    ///
    /// This is the copied-metadata counterpart to
    /// [`ExactMeshHandoffPackage::require_domain`]. It lets schedulers reject
    /// missing capabilities without reinterpreting raw vectors. Exact geometry
    pub fn require_domain(
        &self,
        domain: ExactMeshConsumerDomain,
    ) -> Result<(), ExactMeshDomainSummaryError> {
        if self.has_domain(domain) {
            Ok(())
        } else {
            Err(ExactMeshDomainSummaryError::MissingDomain { domain })
        }
    }

    /// Validate this summary against `package`, then require `domain`.
    ///
    /// This is useful when copied scheduler metadata is stored beside a
    /// retained handoff package but the source mesh is not currently loaded.
    /// It rejects stale summary metadata before accepting the requested
    /// cached facts to replay from the artifact they summarize.
    pub fn require_domain_against_package(
        &self,
        package: &ExactMeshHandoffPackage,
        domain: ExactMeshConsumerDomain,
    ) -> Result<(), ExactMeshDomainSummaryError> {
        self.validate_against_package(package)?;
        self.require_domain(domain)
    }

    /// Validate this summary through `package` against `mesh`, then require `domain`.
    ///
    /// This is the strongest copied-summary capability check: the package must
    /// replay from the exact mesh, the summary must replay from the package,
    /// and then the requested domain must be present.
    pub fn require_domain_against_mesh(
        &self,
        package: &ExactMeshHandoffPackage,
        mesh: &ExactMesh,
        domain: ExactMeshConsumerDomain,
    ) -> Result<(), ExactMeshDomainSummaryError> {
        self.validate_against_mesh(package, mesh)?;
        self.require_domain(domain)
    }

    /// Require at least one exact-geometry domain.
    pub fn require_exact_geometry(&self) -> Result<(), ExactMeshDomainSummaryError> {
        if self.has_exact_geometry() {
            Ok(())
        } else {
            Err(ExactMeshDomainSummaryError::MissingExactGeometry)
        }
    }

    /// Require at least one lossy adapter domain.
    pub fn require_lossy_adapter(&self) -> Result<(), ExactMeshDomainSummaryError> {
        if self.has_lossy_adapter() {
            Ok(())
        } else {
            Err(ExactMeshDomainSummaryError::MissingLossyAdapter)
        }
    }

    /// Require closed-volume readiness.
    pub fn require_closed_volume(&self) -> Result<(), ExactMeshDomainSummaryError> {
        if self.closed_volume_ready {
            Ok(())
        } else {
            Err(ExactMeshDomainSummaryError::MissingClosedVolume)
        }
    }

    /// Validate this retained summary against a handoff package.
    ///
    /// This is a local replay check for copied scheduler metadata. It does not
    /// prove that the package itself is fresh for a mesh; callers should pair
    /// this with [`ExactMeshHandoffPackage::validate_against_mesh`] at source
    /// metadata as reusable only while it replays from the exact artifact it
    /// summarizes.
    pub fn validate_against_package(
        &self,
        package: &ExactMeshHandoffPackage,
    ) -> Result<(), ExactMeshDomainSummaryError> {
        self.validate()?;
        let replay = package.domain_summary();
        if self.available_domains != replay.available_domains {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "available_domains",
            });
        }
        if self.exact_geometry_domains != replay.exact_geometry_domains {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "exact_geometry_domains",
            });
        }
        if self.lossy_adapter_domains != replay.lossy_adapter_domains {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "lossy_adapter_domains",
            });
        }
        if self.exact_geometry_count != replay.exact_geometry_count {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "exact_geometry_count",
            });
        }
        if self.lossy_adapter_count != replay.lossy_adapter_count {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "lossy_adapter_count",
            });
        }
        if self.closed_volume_ready != replay.closed_volume_ready {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "closed_volume_ready",
            });
        }
        Ok(())
    }

    /// Validate summary-internal domain consistency without access to a package.
    pub fn validate(&self) -> Result<(), ExactMeshDomainSummaryError> {
        if has_duplicate_domains(&self.available_domains) {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "available_domains",
            });
        }
        let exact_geometry_domains = self
            .available_domains
            .iter()
            .copied()
            .filter(|domain| domain.is_exact_geometry())
            .collect::<Vec<_>>();
        if self.exact_geometry_domains != exact_geometry_domains {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "exact_geometry_domains",
            });
        }
        let lossy_adapter_domains = self
            .available_domains
            .iter()
            .copied()
            .filter(|domain| domain.is_lossy_adapter())
            .collect::<Vec<_>>();
        if self.lossy_adapter_domains != lossy_adapter_domains {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "lossy_adapter_domains",
            });
        }
        if self.exact_geometry_count != self.exact_geometry_domains.len() {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "exact_geometry_count",
            });
        }
        if self.lossy_adapter_count != self.lossy_adapter_domains.len() {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "lossy_adapter_count",
            });
        }
        if self.closed_volume_ready != self.has_domain(ExactMeshConsumerDomain::Solid) {
            return Err(ExactMeshDomainSummaryError::SummaryMismatch {
                field: "closed_volume_ready",
            });
        }
        Ok(())
    }

    /// Validate this retained summary against `package` and `mesh`.
    ///
    /// This is the strongest summary replay boundary: the package must replay
    /// from the exact mesh, then the summary must replay from that package. It
    /// is intended for downstream schedulers that receive copied summary
    /// separation by treating cached facts as reusable only when they replay
    /// from the exact object whose semantics they summarize.
    pub fn validate_against_mesh(
        &self,
        package: &ExactMeshHandoffPackage,
        mesh: &ExactMesh,
    ) -> Result<(), ExactMeshDomainSummaryError> {
        package
            .validate_against_mesh(mesh)
            .map_err(ExactMeshDomainSummaryError::Package)?;
        self.validate_against_package(package)
    }

    /// Classify whether this retained summary still matches `package`.
    pub fn freshness_against_package(
        &self,
        package: &ExactMeshHandoffPackage,
    ) -> ExactMeshDomainSummaryFreshness {
        match self.validate_against_package(package) {
            Ok(()) => ExactMeshDomainSummaryFreshness::Current,
            Err(ExactMeshDomainSummaryError::Package(_)) => {
                ExactMeshDomainSummaryFreshness::InvalidPackage
            }
            Err(ExactMeshDomainSummaryError::SummaryMismatch { .. })
            | Err(ExactMeshDomainSummaryError::MissingDomain { .. })
            | Err(ExactMeshDomainSummaryError::MissingExactGeometry)
            | Err(ExactMeshDomainSummaryError::MissingLossyAdapter)
            | Err(ExactMeshDomainSummaryError::MissingClosedVolume) => {
                ExactMeshDomainSummaryFreshness::StaleSummary
            }
        }
    }

    /// Classify whether this retained summary still matches `package` and `mesh`.
    pub fn freshness_against_mesh(
        &self,
        package: &ExactMeshHandoffPackage,
        mesh: &ExactMesh,
    ) -> ExactMeshDomainSummaryFreshness {
        match self.validate_against_mesh(package, mesh) {
            Ok(()) => ExactMeshDomainSummaryFreshness::Current,
            Err(ExactMeshDomainSummaryError::Package(_)) => {
                ExactMeshDomainSummaryFreshness::InvalidPackage
            }
            Err(ExactMeshDomainSummaryError::SummaryMismatch { .. })
            | Err(ExactMeshDomainSummaryError::MissingDomain { .. })
            | Err(ExactMeshDomainSummaryError::MissingExactGeometry)
            | Err(ExactMeshDomainSummaryError::MissingLossyAdapter)
            | Err(ExactMeshDomainSummaryError::MissingClosedVolume) => {
                ExactMeshDomainSummaryFreshness::StaleSummary
            }
        }
    }
}

impl ExactMeshConsumerDomain {
    /// Return whether this domain carries exact geometric evidence.
    ///
    /// Surface and solid domains are exact geometry handoffs. The
    /// approximate `f64` view is an adapter artifact for display/export and is
    /// object evidence separate from approximate representatives.
    pub const fn is_exact_geometry(self) -> bool {
        matches!(self, Self::Surface | Self::Solid)
    }

    /// Return whether this domain carries closed-volume semantics.
    ///
    /// Only [`ExactMeshConsumerDomain::Solid`] is a closed-volume handoff.
    /// Boundary-allowed exact surfaces and lossy views must remain outside
    /// volumetric consumers unless a separate exact construction certifies
    /// solid semantics.
    pub const fn is_closed_volume(self) -> bool {
        matches!(self, Self::Solid)
    }

    /// Return whether this domain is explicitly lossy adapter evidence.
    pub const fn is_lossy_adapter(self) -> bool {
        matches!(self, Self::ApproximateF64View)
    }
}

/// Borrowed report selected from an exact mesh handoff package.
///
/// This enum keeps consumer extraction typed. A caller that asks for solid
/// evidence receives a solid handoff report, while a caller that asks for a
/// lossy view receives the view artifact and cannot accidentally reinterpret
/// object evidence and approximate adapter evidence distinct at API
/// boundaries.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ExactMeshDomainReportRef<'a> {
    /// Exact surface handoff report.
    Surface(&'a ExactSurfaceHandoffReport),
    /// Exact closed-solid handoff report.
    Solid(&'a ExactSolidHandoffReport),
    /// Lossy primitive-float display/export view.
    ApproximateF64View(&'a ApproximateMeshF64View),
}

impl<'a> ExactMeshDomainReportRef<'a> {
    /// Return the consumer domain represented by this borrowed report.
    pub const fn domain(&self) -> ExactMeshConsumerDomain {
        match self {
            Self::Surface(_) => ExactMeshConsumerDomain::Surface,
            Self::Solid(_) => ExactMeshConsumerDomain::Solid,
            Self::ApproximateF64View(_) => ExactMeshConsumerDomain::ApproximateF64View,
        }
    }

    /// Return the exact mesh audit shared by the selected report.
    ///
    /// This lets downstream code compare source identity without flattening
    /// object evidence and domain-specific consumers by exposing only the
    /// common retained-state audit at this layer.
    pub const fn audit(&self) -> &'a ExactMeshAuditReport {
        match self {
            Self::Surface(report) => &report.audit,
            Self::Solid(report) => &report.audit,
            Self::ApproximateF64View(report) => &report.audit,
        }
    }
}

/// Error returned when a retained handoff package is invalid or stale.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactMeshHandoffPackageError {
    /// The source mesh failed retained-state audit.
    Audit(super::ExactMeshValidationError),
    /// A downstream handoff package no longer matches replayed source evidence.
    PackageMismatch {
        /// Name of the mismatched field.
        field: &'static str,
    },
    /// The requested consumer domain is not present in this package.
    MissingDomain {
        /// Domain requested by the caller.
        domain: ExactMeshConsumerDomain,
    },
    /// The package contains contradictory internal evidence.
    InternalMismatch {
        /// Name of the mismatched field.
        field: &'static str,
    },
}

/// Freshness status for a retained exact mesh handoff package.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshHandoffPackageFreshness {
    /// The package replays exactly against the current mesh.
    Current,
    /// The mesh failed retained-state audit.
    InvalidMeshState,
    /// The retained package differs from replayed source evidence.
    StalePackage,
}

impl ExactMeshHandoffPackage {
    /// Build a handoff package after retained-state replay.
    ///
    /// Optional members are intentionally `Option`s. An open boundary mesh can
    /// package surface evidence and a lossy view while correctly omitting
    /// closed-solid evidence. Consumers that require a specific domain should
    /// still validate the corresponding inner report before using it.
    pub fn from_mesh(mesh: &ExactMesh) -> Result<Self, ExactMeshHandoffPackageError> {
        let audit = audit_exact_mesh(mesh).map_err(ExactMeshHandoffPackageError::Audit)?;
        let readiness = exact_mesh_consumer_readiness(mesh).map_err(|error| match error {
            super::ExactMeshConsumerReadinessError::Audit(error) => {
                ExactMeshHandoffPackageError::Audit(error)
            }
            super::ExactMeshConsumerReadinessError::ReportMismatch { .. } => {
                ExactMeshHandoffPackageError::PackageMismatch {
                    field: "exact_mesh_consumer_readiness",
                }
            }
        })?;
        Ok(Self {
            audit,
            readiness,
            surface: exact_surface_handoff(mesh).ok(),
            solid: exact_solid_handoff(mesh).ok(),
            approximate_f64_view: approximate_mesh_f64_view(mesh).ok(),
        })
    }

    /// Validate that this package still replays against `mesh`.
    pub fn validate_against_mesh(
        &self,
        mesh: &ExactMesh,
    ) -> Result<(), ExactMeshHandoffPackageError> {
        self.validate_internal()?;
        let replay = Self::from_mesh(mesh)?;
        if self != &replay {
            return Err(ExactMeshHandoffPackageError::PackageMismatch {
                field: "exact_mesh_handoff_package",
            });
        }
        Ok(())
    }

    /// Validate package-internal consistency without access to the source mesh.
    ///
    /// This is a serialization and queue-boundary guard. It cannot prove that
    /// the package is fresh for a current mesh; only
    /// [`ExactMeshHandoffPackage::validate_against_mesh`] can do that. It does
    /// reject self-contradictory packages where readiness flags, inner audits,
    /// and optional member reports disagree rather than trusting them because
    /// they are well-typed.
    pub fn validate_internal(&self) -> Result<(), ExactMeshHandoffPackageError> {
        if self.readiness.audit != self.audit {
            return Err(ExactMeshHandoffPackageError::InternalMismatch {
                field: "readiness.audit",
            });
        }
        if self.readiness.validate().is_err() {
            return Err(ExactMeshHandoffPackageError::InternalMismatch { field: "readiness" });
        }
        if self.readiness.surface_handoff_ready != self.surface.is_some() {
            return Err(ExactMeshHandoffPackageError::InternalMismatch {
                field: "readiness.surface_handoff_ready",
            });
        }
        if self.readiness.solid_handoff_ready != self.solid.is_some() {
            return Err(ExactMeshHandoffPackageError::InternalMismatch {
                field: "readiness.solid_handoff_ready",
            });
        }
        if self.readiness.approximate_f64_view_ready != self.approximate_f64_view.is_some() {
            return Err(ExactMeshHandoffPackageError::InternalMismatch {
                field: "readiness.approximate_f64_view_ready",
            });
        }
        if let Some(surface) = &self.surface
            && surface.audit != self.audit
        {
            return Err(ExactMeshHandoffPackageError::InternalMismatch {
                field: "surface.audit",
            });
        }
        if let Some(surface) = &self.surface
            && surface.validate().is_err()
        {
            return Err(ExactMeshHandoffPackageError::InternalMismatch { field: "surface" });
        }
        if let Some(solid) = &self.solid
            && solid.audit != self.audit
        {
            return Err(ExactMeshHandoffPackageError::InternalMismatch {
                field: "solid.audit",
            });
        }
        if let Some(solid) = &self.solid
            && solid.validate().is_err()
        {
            return Err(ExactMeshHandoffPackageError::InternalMismatch { field: "solid" });
        }
        if let Some(view) = &self.approximate_f64_view
            && view.audit != self.audit
        {
            return Err(ExactMeshHandoffPackageError::InternalMismatch {
                field: "approximate_f64_view.audit",
            });
        }
        if let Some(view) = &self.approximate_f64_view
            && view.validate().is_err()
        {
            return Err(ExactMeshHandoffPackageError::InternalMismatch {
                field: "approximate_f64_view",
            });
        }
        Ok(())
    }

    /// Return the consumer domains currently present in this package.
    ///
    /// The order is stable: surface, solid, then lossy display/export view.
    /// This helper keeps downstream routing from duplicating optional-field
    /// inspection and preserves the package as the single authority for domain
    /// adapter artifacts rather than implicit substitutes for exact topology.
    pub fn available_domains(&self) -> Vec<ExactMeshConsumerDomain> {
        let mut domains = Vec::with_capacity(3);
        if self.surface.is_some() {
            domains.push(ExactMeshConsumerDomain::Surface);
        }
        if self.solid.is_some() {
            domains.push(ExactMeshConsumerDomain::Solid);
        }
        if self.approximate_f64_view.is_some() {
            domains.push(ExactMeshConsumerDomain::ApproximateF64View);
        }
        domains
    }

    /// Return only exact-geometry domains currently present in this package.
    ///
    /// This filters out lossy adapter evidence. Downstream topology, voxel, and
    /// physics consumers should prefer this helper over
    /// [`ExactMeshHandoffPackage::available_domains`] when they need exact
    /// geometric evidence and not display/export artifacts.
    pub fn exact_geometry_domains(&self) -> Vec<ExactMeshConsumerDomain> {
        self.available_domains()
            .into_iter()
            .filter(|domain| domain.is_exact_geometry())
            .collect()
    }

    /// Return only lossy adapter domains currently present in this package.
    ///
    /// approximate representatives should remain explicitly labeled and should
    /// not be confused with exact object evidence.
    pub fn lossy_adapter_domains(&self) -> Vec<ExactMeshConsumerDomain> {
        self.available_domains()
            .into_iter()
            .filter(|domain| domain.is_lossy_adapter())
            .collect()
    }

    /// Return a compact summary of domains currently present in this package.
    pub fn domain_summary(&self) -> ExactMeshDomainSummary {
        let available_domains = self.available_domains();
        let exact_geometry_domains = available_domains
            .iter()
            .copied()
            .filter(|domain| domain.is_exact_geometry())
            .collect::<Vec<_>>();
        let lossy_adapter_domains = available_domains
            .iter()
            .copied()
            .filter(|domain| domain.is_lossy_adapter())
            .collect::<Vec<_>>();
        let closed_volume_ready = available_domains
            .iter()
            .copied()
            .any(ExactMeshConsumerDomain::is_closed_volume);
        ExactMeshDomainSummary {
            exact_geometry_count: exact_geometry_domains.len(),
            lossy_adapter_count: lossy_adapter_domains.len(),
            available_domains,
            exact_geometry_domains,
            lossy_adapter_domains,
            closed_volume_ready,
        }
    }

    /// Return the strongest exact-geometry domain currently available.
    pub fn preferred_exact_geometry_domain(&self) -> Option<ExactMeshConsumerDomain> {
        self.domain_summary().preferred_exact_geometry_domain()
    }

    /// Require and return the strongest exact-geometry domain in this package.
    ///
    /// Closed-solid evidence is preferred over surface evidence because it has
    /// stricter volumetric semantics. Approximate display/export views are
    /// exact object evidence, not lossy representatives, drives geometric
    /// decisions.
    pub fn require_preferred_exact_geometry_domain(
        &self,
    ) -> Result<ExactMeshConsumerDomain, ExactMeshHandoffPackageError> {
        self.preferred_exact_geometry_domain()
            .ok_or(ExactMeshHandoffPackageError::MissingDomain {
                domain: ExactMeshConsumerDomain::Surface,
            })
    }

    /// Validate package freshness against `mesh`, then require the strongest exact domain.
    ///
    /// Use this at downstream crate boundaries when a retained package may be
    /// stale. It rejects stale packages before selecting solid-over-surface
    /// exact evidence, preventing old optional fields from authorizing current
    /// mesh consumption.
    pub fn require_preferred_exact_geometry_domain_against_mesh(
        &self,
        mesh: &ExactMesh,
    ) -> Result<ExactMeshConsumerDomain, ExactMeshHandoffPackageError> {
        self.validate_against_mesh(mesh)?;
        self.require_preferred_exact_geometry_domain()
    }

    /// Return the preferred exact-geometry report without replaying against a mesh.
    ///
    /// Closed-solid reports are preferred over surface reports. Approximate
    /// primitive-float views are excluded even when present. This packages the
    /// selected exact domain and borrowed report together so consumers do not
    /// split scheduler preference from report extraction. The design follows
    /// retained, replayable artifacts rather than implicit optional-field
    /// conventions.
    pub fn preferred_exact_geometry_report(
        &self,
    ) -> Result<ExactMeshDomainReportRef<'_>, ExactMeshHandoffPackageError> {
        let domain = self.require_preferred_exact_geometry_domain()?;
        self.domain_report(domain)
    }

    /// Validate package freshness against `mesh`, then return the preferred exact report.
    ///
    /// Use this for downstream voxel, physics, and topology consumers that
    /// receive retained packages from queues or caches. Stale packages are
    /// rejected before any surface or solid report is exposed.
    pub fn preferred_exact_geometry_report_against_mesh(
        &self,
        mesh: &ExactMesh,
    ) -> Result<ExactMeshDomainReportRef<'_>, ExactMeshHandoffPackageError> {
        self.validate_against_mesh(mesh)?;
        self.preferred_exact_geometry_report()
    }

    /// Return whether `domain` is present in this package.
    pub fn has_domain(&self, domain: ExactMeshConsumerDomain) -> bool {
        match domain {
            ExactMeshConsumerDomain::Surface => self.surface.is_some(),
            ExactMeshConsumerDomain::Solid => self.solid.is_some(),
            ExactMeshConsumerDomain::ApproximateF64View => self.approximate_f64_view.is_some(),
        }
    }

    /// Validate that this package contains evidence for `domain`.
    ///
    /// This is intentionally a package-level guard, not a domain proof. A
    /// caller that receives `Ok(())` should still consume the corresponding
    /// inner report. The guard prevents open-surface packages from being used
    /// as solids and prevents lossy display packages from being treated as
    /// exact geometry merely because they share a source audit.
    pub fn require_domain(
        &self,
        domain: ExactMeshConsumerDomain,
    ) -> Result<(), ExactMeshHandoffPackageError> {
        if self.has_domain(domain) {
            Ok(())
        } else {
            Err(ExactMeshHandoffPackageError::MissingDomain { domain })
        }
    }

    /// Return the report for `domain` without replaying against a mesh.
    ///
    /// This is a local package extraction helper. Use
    /// [`ExactMeshHandoffPackage::domain_report_against_mesh`] when the
    /// package may be stale for a current source mesh.
    pub fn domain_report(
        &self,
        domain: ExactMeshConsumerDomain,
    ) -> Result<ExactMeshDomainReportRef<'_>, ExactMeshHandoffPackageError> {
        match domain {
            ExactMeshConsumerDomain::Surface => self
                .surface
                .as_ref()
                .map(ExactMeshDomainReportRef::Surface)
                .ok_or(ExactMeshHandoffPackageError::MissingDomain { domain }),
            ExactMeshConsumerDomain::Solid => self
                .solid
                .as_ref()
                .map(ExactMeshDomainReportRef::Solid)
                .ok_or(ExactMeshHandoffPackageError::MissingDomain { domain }),
            ExactMeshConsumerDomain::ApproximateF64View => self
                .approximate_f64_view
                .as_ref()
                .map(ExactMeshDomainReportRef::ApproximateF64View)
                .ok_or(ExactMeshHandoffPackageError::MissingDomain { domain }),
        }
    }

    /// Validate package freshness against `mesh`, then require `domain`.
    ///
    /// Use this at external crate boundaries where a retained package may have
    /// crossed caches, serialization, or scheduling queues. It rejects stale
    /// packages before checking domain presence, so cached artifacts are used
    /// only while they replay from the exact object whose semantics they
    /// summarize.
    pub fn require_domain_against_mesh(
        &self,
        mesh: &ExactMesh,
        domain: ExactMeshConsumerDomain,
    ) -> Result<(), ExactMeshHandoffPackageError> {
        self.validate_against_mesh(mesh)?;
        self.require_domain(domain)
    }

    /// Validate package freshness against `mesh`, then return the domain report.
    ///
    /// This combines source replay and typed extraction. It is the preferred
    /// boundary API for downstream crates that consume retained packages from
    /// queues, caches, or serialized stores.
    pub fn domain_report_against_mesh(
        &self,
        mesh: &ExactMesh,
        domain: ExactMeshConsumerDomain,
    ) -> Result<ExactMeshDomainReportRef<'_>, ExactMeshHandoffPackageError> {
        self.validate_against_mesh(mesh)?;
        self.domain_report(domain)
    }

    /// Classify whether this retained package is fresh for `mesh`.
    pub fn freshness_against_mesh(&self, mesh: &ExactMesh) -> ExactMeshHandoffPackageFreshness {
        match self.validate_against_mesh(mesh) {
            Ok(()) => ExactMeshHandoffPackageFreshness::Current,
            Err(ExactMeshHandoffPackageError::Audit(_)) => {
                ExactMeshHandoffPackageFreshness::InvalidMeshState
            }
            Err(ExactMeshHandoffPackageError::PackageMismatch { .. }) => {
                ExactMeshHandoffPackageFreshness::StalePackage
            }
            Err(ExactMeshHandoffPackageError::MissingDomain { .. }) => {
                ExactMeshHandoffPackageFreshness::StalePackage
            }
            Err(ExactMeshHandoffPackageError::InternalMismatch { .. }) => {
                ExactMeshHandoffPackageFreshness::StalePackage
            }
        }
    }
}

/// Build a bundled downstream handoff package for an exact mesh.
pub fn exact_mesh_handoff_package(
    mesh: &ExactMesh,
) -> Result<ExactMeshHandoffPackage, ExactMeshHandoffPackageError> {
    ExactMeshHandoffPackage::from_mesh(mesh)
}

fn has_duplicate_domains(domains: &[ExactMeshConsumerDomain]) -> bool {
    domains
        .iter()
        .enumerate()
        .any(|(index, domain)| domains[index + 1..].contains(domain))
}
