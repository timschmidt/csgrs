//! Typed failures from CSG grammar and native geometry operations.
//!
//! [`ValidationError`] retains operation-specific payloads; [`ValidationError::kind`]
//! groups them for callers and bindings. The category is deliberately coarser than
//! the native diagnostic and is not a substitute for inspecting the variant.
//! Successful empty geometry is represented by the operation's value, not an error.

use hypercurve::{CurveError, ExactCurveError};

/// Error carrier for native curve-region Boolean failures.
///
/// Keeps either the lower-level curve diagnostic or the contextual exact-operation
/// error. These transparent wrappers delegate display and source-chain behavior to
/// their payloads; match the variants to inspect those payloads directly.
#[derive(Clone, Debug, thiserror::Error, PartialEq)]
pub enum CurveBooleanError {
    /// Hypercurve rejected the topology operation.
    #[error(transparent)]
    Curve(
        /// Native curve invariant or construction diagnostic.
        #[from]
        CurveError,
    ),
    /// A higher-order exact curve operation failed with retained context.
    #[error(transparent)]
    ExactCurve(
        /// Native exact-operation failure, including blocker context when present.
        #[from]
        ExactCurveError,
    ),
}

/// Failure in a CSG construction or modeling operation.
///
/// Also exported as [`crate::Error`]. Native diagnostics remain in typed payloads
/// except for the text-only [`Self::Geometry`] fallback. Derived `From`
/// implementations wrap the variants marked `#[from]`; their classification is
/// computed by [`Self::kind`] rather than stored separately.
///
/// Source-chain exposure follows each variant's error attributes. In particular,
/// [`Self::Scalar`] transparently forwards the scalar's own source chain, while
/// [`Self::PolynomialTopology`] retains a payload without exposing it through
/// [`std::error::Error::source`]. Match the variant when that detail is required.
#[derive(Debug, Clone, thiserror::Error, PartialEq)]
#[non_exhaustive]
pub enum ValidationError {
    /// Lazy expression evaluation failed at the specified path from its root.
    /// An empty path identifies the root evaluation, including arrangement
    /// failures that cannot be attributed to an individual operand.
    #[error("CSG expression node {node_path:?}: {source}")]
    Expression {
        /// Route from the expression root to the failing node; empty means the root.
        node_path: Vec<crate::solid_expr::SolidExprChild>,
        /// Underlying failure; classification recursively follows this error.
        #[source]
        source: Box<ValidationError>,
    },
    /// A length or radius required to be nonnegative was classified negative.
    #[error("{name} must be nonnegative")]
    NegativeParameter {
        /// Name of the dimension classified negative.
        name: &'static str,
    },
    /// A scale or period that must be strictly positive is zero or negative.
    #[error("{name} must be positive")]
    NonPositiveParameter {
        /// Name of the parameter requiring a strictly positive value.
        name: &'static str,
    },
    /// An operation requires a nonzero component or parameter.
    #[error("{name} must be nonzero")]
    ZeroParameter {
        /// Name of the parameter required to be nonzero.
        name: &'static str,
    },
    /// Extraction encountered a degenerate output triangle and published no mesh.
    #[error("output triangle {triangle_index} is degenerate")]
    DegenerateOutputTriangle {
        /// Index of the rejected output triangle.
        triangle_index: usize,
    },
    /// A tessellation count does not meet the constructor's minimum.
    #[error("{name} must be at least {min}, got {actual}")]
    InvalidResolution {
        /// Name of the rejected tessellation count.
        name: &'static str,
        /// Smallest accepted count.
        min: usize,
        /// Supplied tessellation count.
        actual: usize,
    },
    /// The requested analytic primitive is outside the supported representation.
    #[error("unsupported primitive: {reason}")]
    UnsupportedPrimitive {
        /// Unsupported representation or construction requirement.
        reason: &'static str,
    },
    /// A valid transform cannot be represented by the target geometry carrier.
    #[error("unsupported transform: {reason}")]
    UnsupportedTransform {
        /// Restriction that prevents representing the transformed geometry.
        reason: &'static str,
    },
    /// A transform has no finite image under the admitted homogeneous map.
    #[error("unbounded transform: {reason}")]
    UnboundedTransform {
        /// Reason the homogeneous image cannot be represented as finite geometry.
        reason: &'static str,
    },
    /// A necessary predicate could not be decided under the selected policy.
    #[error("{predicate} is undecided under {policy:?}")]
    PredicateUndecided {
        /// Name of the required decision that remained unresolved.
        predicate: &'static str,
        /// Predicate policy under which the decision was attempted.
        policy: hyperlimit::PredicatePolicy,
    },
    /// A geometry buffer length or byte size cannot be represented on this platform.
    #[error("geometry buffer capacity exceeds platform limits")]
    CapacityOverflow,
    /// Reserving a geometry buffer failed before a complete result was published.
    #[error("could not allocate geometry buffer for {elements} elements")]
    AllocationFailed {
        /// Requested element count, not a byte count or configured work budget.
        elements: usize,
    },
    /// A checked scalar or lattice operation failed, retaining its native cause.
    #[error(transparent)]
    Scalar(
        /// Original scalar failure; inspect it to distinguish domain and decision errors.
        #[from]
        hyperreal::Problem,
    ),
    /// A native mesh operation failed, retaining its typed cause.
    #[error("{0}")]
    Mesh(
        /// Original mesh validation, arrangement, or operation diagnostic.
        #[from]
        hypermesh::HypermeshError,
    ),
    /// Native graphics-buffer construction failed, retaining its typed cause.
    #[error("{0}")]
    Gpu(
        /// Original graphics export or normal-construction diagnostic.
        #[from]
        hypermesh::GpuMeshError,
    ),
    /// A native exact curve operation failed, retaining its context and cause.
    #[error("{0}")]
    ExactCurve(
        /// Native exact-operation failure, including blocker context when present.
        #[from]
        ExactCurveError,
    ),
    /// Polynomial topology reconstruction failed with its original solver cause.
    #[error("polynomial topology failed: {0:?}")]
    PolynomialTopology(
        /// Original topology solver diagnostic, retained outside the error source chain.
        hypercurve::BivariateTopologyError,
    ),
    /// Native contour construction failed, retaining its original diagnostic.
    #[error("{0}")]
    Curve(
        /// Native curve invariant or construction diagnostic.
        #[from]
        CurveError,
    ),
    /// Native sampled-field extraction failed, retaining its typed diagnostic.
    #[error("{0}")]
    SurfaceNets(
        /// Original sampled 3D extraction diagnostic.
        #[from]
        hypermesh::SurfaceNetsError,
    ),
    /// Native planar sampled-field extraction failed, retaining its typed cause.
    #[error("{0}")]
    MarchingSquares(
        /// Original sampled planar extraction diagnostic.
        #[from]
        hypercurve::MarchingSquaresError,
    ),
    /// The font face could not be parsed; retains the parser's diagnostic.
    #[cfg(feature = "truetype-text")]
    #[error("invalid font face: {0}")]
    FontFace(
        /// Font parser failure for the requested face.
        #[source]
        ttf_parser::FaceParsingError,
    ),
    /// A required font table is absent or malformed.
    #[cfg(feature = "truetype-text")]
    #[error("invalid or missing font table {table}")]
    InvalidFontTable {
        /// Name of the missing or malformed table.
        table: &'static str,
    },
    /// The font's character map has no glyph for the requested character.
    #[cfg(feature = "truetype-text")]
    #[error("font has no glyph for {character:?}")]
    MissingFontGlyph {
        /// Character absent from the selected font map.
        character: char,
    },
    /// The selected glyph did not supply a complete valid outline.
    #[cfg(feature = "truetype-text")]
    #[error("invalid outline for font glyph {glyph_id}")]
    InvalidFontOutline {
        /// Font-local glyph identifier whose outline was rejected.
        glyph_id: u16,
    },
    /// A parser-emitted outline coordinate could not be imported.
    #[cfg(feature = "truetype-text")]
    #[error("invalid coordinate in font glyph {glyph_id}: {source}")]
    FontCoordinate {
        /// Font-local glyph identifier supplying the invalid coordinate.
        glyph_id: u16,
        /// Original coordinate import failure.
        #[source]
        source: hyperreal::Problem,
    },
    /// Native Hershey text construction failed without publishing partial text.
    #[cfg(feature = "hershey-text")]
    #[error("{0}")]
    HersheyText(
        /// Original stroke-font construction diagnostic.
        #[from]
        hypercurve::hershey::TextError,
    ),
    /// Native triangulation failed before a modeling surface was published.
    #[error("{0}")]
    Triangulation(
        /// Original triangulation diagnostic.
        #[from]
        hypertri::Error,
    ),
    /// A retained implicit-field construction failed before grid sampling.
    #[cfg(any(feature = "sdf", feature = "metaballs"))]
    #[error("{0}")]
    SdfEvaluation(
        /// Original retained-field construction or evaluation diagnostic.
        #[from]
        hypersdf::SdfEvaluationError,
    ),
    /// A retained expression could not be evaluated at this grid sample.
    #[cfg(any(feature = "sdf", feature = "metaballs"))]
    #[error("SDF sample {sample:?}: {source}")]
    SdfSample {
        /// XYZ grid indices of the failed sample, not world-space coordinates.
        sample: [usize; 3],
        /// Original evaluation failure at the named grid sample.
        #[source]
        source: hypersdf::SdfEvaluationError,
    },
    /// Operation requires corresponding loops with the same number of vertices.
    #[error("operation requires loops with the same number of vertices, {left} != {right}")]
    MismatchedVertexCount {
        /// Vertex count of the first loop.
        left: usize,
        /// Vertex count of the corresponding second loop.
        right: usize,
    },
    /// A required index is outside a collection.
    #[error("index {index} is out of range for length {len}")]
    IndexOutOfRangeWithLen {
        /// Attempted index.
        index: usize,
        /// Collection length; valid indices are strictly smaller.
        len: usize,
    },
    /// Feature arguments do not describe supported geometry.
    #[error("invalid arguments")]
    InvalidArguments,
    /// A named integer field is below the supported minimum.
    #[error("{name} must not be less than {min}")]
    FieldLessThan {
        /// Name of the rejected integer field.
        name: &'static str,
        /// Smallest accepted value; this variant does not store the supplied value.
        min: i32,
    },
    /// A native geometry dependency rejected the requested construction.
    #[error("{0}")]
    Geometry(
        /// Textual diagnostic; no typed error source is retained.
        String,
    ),
    /// An input number has no exact real value, such as a NaN, an infinity, a
    /// zero denominator or malformed decimal text.
    #[error("number has no exact real value: {0}")]
    InvalidNumber(
        /// Original scalar failure; inspect it to distinguish domain and decision errors.
        #[source]
        hyperreal::Problem,
    ),
}

/// Stable failure categories exposed by modeling operations and bindings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ValidationErrorKind {
    /// Input violates an admitted precondition or invariant.
    InvalidInput,
    /// A required representation, operation, or font glyph is unavailable.
    Unsupported,
    /// A required decision or unambiguous construction was not established.
    Undecided,
    /// Resource, capacity, or cancellation failure grouped by the current mapping.
    ///
    /// Includes some allocation failures and representational overflow, not only
    /// exhaustion of configured work budgets. Inspect the native payload to
    /// distinguish these cases; this category alone is not a budget receipt.
    ResourceExhausted,
    /// A curve uncertainty reason explicitly reports unavailable storage.
    ///
    /// Other allocation failures currently map to [`Self::ResourceExhausted`];
    /// this category is not an exhaustive classification of allocation failure.
    ResourceUnavailable,
    /// The requested result or homogeneous image cannot be represented as finite.
    Unbounded,
    /// Geometry or scalar processing failed outside the more specific categories.
    Geometry,
}

impl ValidationErrorKind {
    /// Machine-readable category used as the JavaScript geometry exception code.
    ///
    /// These labels identify this coarse category, not the native error variant
    /// or its payload. `Geometry` maps to `"geometry_failure"`.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidInput => "invalid_input",
            Self::Unsupported => "unsupported",
            Self::Undecided => "undecided",
            Self::ResourceExhausted => "resource_exhausted",
            Self::ResourceUnavailable => "resource_unavailable",
            Self::Unbounded => "unbounded",
            Self::Geometry => "geometry_failure",
        }
    }
}

/// Map a curve uncertainty reason without conflating storage failure and work limits.
/// Boundary ambiguity is classified as invalid input; unresolved sign/order
/// predicates remain undecided. This helper does not perform a new predicate.
const fn uncertainty_reason_kind(
    reason: hypercurve::UncertaintyReason,
) -> ValidationErrorKind {
    use hypercurve::UncertaintyReason;
    match reason {
        UncertaintyReason::RealSign
        | UncertaintyReason::Predicate
        | UncertaintyReason::Ordering => ValidationErrorKind::Undecided,
        UncertaintyReason::Boundary => ValidationErrorKind::InvalidInput,
        UncertaintyReason::Unsupported => ValidationErrorKind::Unsupported,
        UncertaintyReason::BudgetExhausted => ValidationErrorKind::ResourceExhausted,
        UncertaintyReason::StorageUnavailable => ValidationErrorKind::ResourceUnavailable,
    }
}

/// Classify a lower-level curve diagnostic for the modeling boundary.
/// Projection blockers keep their reason category; ambiguous connection/bulge and
/// policy incompatibility are undecided. Topology/scalar wrappers use the generic
/// geometry category; unmatched variants are treated as invalid input.
const fn curve_error_kind(source: &CurveError) -> ValidationErrorKind {
    match source {
        CurveError::UncertainFiniteCurveProjection { reason } => {
            uncertainty_reason_kind(*reason)
        },
        CurveError::UnsupportedFiniteCurveProjection { .. } => {
            ValidationErrorKind::Unsupported
        },
        CurveError::IncompatiblePredicatePolicy
        | CurveError::AmbiguousBulge
        | CurveError::AmbiguousCurveStringConnection => ValidationErrorKind::Undecided,
        CurveError::UnsupportedNurbsInterpolationDivision { .. } => {
            ValidationErrorKind::Unsupported
        },
        CurveError::Topology(_) | CurveError::Real(_) => ValidationErrorKind::Geometry,
        _ => ValidationErrorKind::InvalidInput,
    }
}

impl ValidationError {
    /// Return the binding-facing category by inspecting the retained diagnostic.
    ///
    /// Expression wrappers recurse into their source. Other wrappers use explicit
    /// variant mappings and fallbacks; no geometric validation or retry occurs.
    /// Classification is coarser than the payload: scalar and topology failures
    /// can share a category, and resource exhaustion includes capacity overflow,
    /// cancellation and some allocation failures. Preserve the error itself when
    /// deciding whether to retry, change inputs, or report missing support.
    ///
    /// This method does not certify that every resource failure reached a declared
    /// budget, nor that every allocation failure uses `ResourceUnavailable`.
    pub const fn kind(&self) -> ValidationErrorKind {
        match self {
            Self::Expression { source, .. } => source.kind(),
            Self::Gpu(error) => match error {
                hypermesh::GpuMeshError::TriangleNormalUndecided { .. }
                | hypermesh::GpuMeshError::TriangleNormalFailed {
                    source:
                        hyperreal::Problem::UnknownZero | hyperreal::Problem::UnknownCertificate,
                    ..
                } => ValidationErrorKind::Undecided,
                hypermesh::GpuMeshError::VertexCountExceededU32
                | hypermesh::GpuMeshError::TriangleNormalFailed {
                    source: hyperreal::Problem::Exhausted,
                    ..
                } => ValidationErrorKind::ResourceExhausted,
                hypermesh::GpuMeshError::TriangleNormalFailed { .. }
                | hypermesh::GpuMeshError::AttributeApproximationFailed { .. } => {
                    ValidationErrorKind::Geometry
                },
                _ => ValidationErrorKind::InvalidInput,
            },
            Self::Curve(source) => curve_error_kind(source),
            #[cfg(feature = "truetype-text")]
            Self::MissingFontGlyph { .. } => ValidationErrorKind::Unsupported,
            #[cfg(feature = "hershey-text")]
            Self::HersheyText(source) => match source {
                hypercurve::hershey::TextError::UndecidedSize { .. } => {
                    ValidationErrorKind::Undecided
                },
                hypercurve::hershey::TextError::ZeroSize => ValidationErrorKind::InvalidInput,
                hypercurve::hershey::TextError::Curve(source) => curve_error_kind(source),
                hypercurve::hershey::TextError::Font(source) => match source {
                    hypercurve::hershey::HersheyError::NoSuchGlyph(_)
                    | hypercurve::hershey::HersheyError::NoSuchGlyphIndex(_) => {
                        ValidationErrorKind::Unsupported
                    },
                    hypercurve::hershey::HersheyError::InvalidGlyph(_) => {
                        ValidationErrorKind::InvalidInput
                    },
                    hypercurve::hershey::HersheyError::CapacityOverflow
                    | hypercurve::hershey::HersheyError::AllocationFailed { .. } => {
                        ValidationErrorKind::ResourceExhausted
                    },
                },
            },
            Self::Triangulation(source) => match source {
                hypertri::Error::PredicateUndecided { .. } => ValidationErrorKind::Undecided,
                hypertri::Error::InvalidInput { .. } => ValidationErrorKind::InvalidInput,
                hypertri::Error::UnsupportedFeature { .. } => ValidationErrorKind::Unsupported,
                hypertri::Error::NoEarFound => ValidationErrorKind::Geometry,
            },
            Self::MarchingSquares(source) => match source {
                hypercurve::MarchingSquaresError::GridTooSmall { .. }
                | hypercurve::MarchingSquaresError::SampleCountMismatch { .. }
                | hypercurve::MarchingSquaresError::AxisNotIncreasing { .. }
                | hypercurve::MarchingSquaresError::BoundaryNotExterior { .. } => {
                    ValidationErrorKind::InvalidInput
                },
                hypercurve::MarchingSquaresError::CapacityOverflow
                | hypercurve::MarchingSquaresError::AllocationFailed { .. }
                | hypercurve::MarchingSquaresError::Scalar(hyperreal::Problem::Exhausted) => {
                    ValidationErrorKind::ResourceExhausted
                },
                hypercurve::MarchingSquaresError::PredicateUndecided { .. }
                | hypercurve::MarchingSquaresError::Scalar(
                    hyperreal::Problem::UnknownZero | hyperreal::Problem::UnknownCertificate,
                ) => ValidationErrorKind::Undecided,
                hypercurve::MarchingSquaresError::Region(ExactCurveError::Blocked(
                    blocker,
                )) => uncertainty_reason_kind(blocker.reason()),
                _ => ValidationErrorKind::Geometry,
            },
            #[cfg(any(feature = "sdf", feature = "metaballs"))]
            Self::SdfSample { source, .. } | Self::SdfEvaluation(source) => match source {
                hypersdf::SdfEvaluationError::InvalidDomain { .. } => {
                    ValidationErrorKind::InvalidInput
                },
                hypersdf::SdfEvaluationError::Undecided { .. }
                | hypersdf::SdfEvaluationError::Scalar {
                    source:
                        hyperreal::Problem::UnknownZero | hyperreal::Problem::UnknownCertificate,
                    ..
                } => ValidationErrorKind::Undecided,
                hypersdf::SdfEvaluationError::Scalar {
                    source: hyperreal::Problem::Exhausted,
                    ..
                } => ValidationErrorKind::ResourceExhausted,
                hypersdf::SdfEvaluationError::Scalar {
                    source:
                        hyperreal::Problem::SqrtNegative
                        | hyperreal::Problem::DivideByZero
                        | hyperreal::Problem::NotANumber,
                    ..
                } => ValidationErrorKind::InvalidInput,
                _ => ValidationErrorKind::Geometry,
            },
            Self::PolynomialTopology(error) => match error {
                hypercurve::BivariateTopologyError::InvalidDomain
                | hypercurve::BivariateTopologyError::ParameterOutsideDomain
                | hypercurve::BivariateTopologyError::ZeroPolynomial => {
                    ValidationErrorKind::InvalidInput
                },
                hypercurve::BivariateTopologyError::UnsupportedCoefficient
                | hypercurve::BivariateTopologyError::VerticalComponent
                | hypercurve::BivariateTopologyError::RepeatedComponent
                | hypercurve::BivariateTopologyError::BoundaryComponent => {
                    ValidationErrorKind::Unsupported
                },
                hypercurve::BivariateTopologyError::DegreeLimit
                | hypercurve::BivariateTopologyError::RefinementLimit
                | hypercurve::BivariateTopologyError::BudgetExhausted => {
                    ValidationErrorKind::ResourceExhausted
                },
                hypercurve::BivariateTopologyError::Undecided => {
                    ValidationErrorKind::Undecided
                },
            },
            Self::ExactCurve(ExactCurveError::Blocked(blocker)) => {
                uncertainty_reason_kind(blocker.reason())
            },
            Self::ExactCurve(ExactCurveError::Invalid { .. }) => {
                ValidationErrorKind::InvalidInput
            },
            Self::SurfaceNets(error) => match error {
                hypermesh::SurfaceNetsError::UnknownSampleSign { .. }
                | hypermesh::SurfaceNetsError::PredicateUndecided { .. } => {
                    ValidationErrorKind::Undecided
                },
                hypermesh::SurfaceNetsError::SampleCountOverflow { .. } => {
                    ValidationErrorKind::ResourceExhausted
                },
                hypermesh::SurfaceNetsError::GridTooSmall { .. }
                | hypermesh::SurfaceNetsError::SampleCountMismatch { .. } => {
                    ValidationErrorKind::InvalidInput
                },
                _ => ValidationErrorKind::Geometry,
            },
            Self::Mesh(error) => match error {
                hypermesh::HypermeshError::PredicateUndecided { .. }
                | hypermesh::HypermeshError::ConvexHullPredicate { .. }
                | hypermesh::HypermeshError::UnknownClassification => {
                    ValidationErrorKind::Undecided
                },
                hypermesh::HypermeshError::CapacityOverflow { .. }
                | hypermesh::HypermeshError::ResourceLimitExceeded { .. }
                | hypermesh::HypermeshError::OperationCancelled { .. }
                | hypermesh::HypermeshError::WindingOverflow => {
                    ValidationErrorKind::ResourceExhausted
                },
                hypermesh::HypermeshError::PointAtInfinity
                | hypermesh::HypermeshError::UnboundedProjectiveTriangle { .. }
                | hypermesh::HypermeshError::UnboundedBooleanOutput { .. } => {
                    ValidationErrorKind::Unbounded
                },
                hypermesh::HypermeshError::OpenOutput { .. }
                | hypermesh::HypermeshError::SurfaceArrangementFailed { .. } => {
                    ValidationErrorKind::Geometry
                },
                _ => ValidationErrorKind::InvalidInput,
            },
            Self::UnsupportedPrimitive { .. } | Self::UnsupportedTransform { .. } => {
                ValidationErrorKind::Unsupported
            },
            Self::UnboundedTransform { .. } => ValidationErrorKind::Unbounded,
            Self::InvalidNumber(_) => ValidationErrorKind::InvalidInput,
            Self::PredicateUndecided { .. }
            | Self::Scalar(
                hyperreal::Problem::UnknownZero | hyperreal::Problem::UnknownCertificate,
            ) => ValidationErrorKind::Undecided,
            Self::CapacityOverflow
            | Self::AllocationFailed { .. }
            | Self::Scalar(hyperreal::Problem::Exhausted) => {
                ValidationErrorKind::ResourceExhausted
            },
            Self::Geometry(_) | Self::Scalar(_) | Self::DegenerateOutputTriangle { .. } => {
                ValidationErrorKind::Geometry
            },
            _ => ValidationErrorKind::InvalidInput,
        }
    }
}
