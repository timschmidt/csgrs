//! Errors produced by CSG grammar and native-curve operations.

use hypercurve::{CurveError, ExactCurveError};

/// Failure to produce a certified native curve-region Boolean.
#[derive(Clone, Debug, thiserror::Error, PartialEq)]
pub enum CurveBooleanError {
    /// Hypercurve rejected the topology operation.
    #[error(transparent)]
    Curve(#[from] CurveError),
    /// A higher-order exact curve operation failed with retained context.
    #[error(transparent)]
    ExactCurve(#[from] ExactCurveError),
}

/// Failure in a CSG construction or modeling operation.
#[derive(Debug, Clone, thiserror::Error, PartialEq)]
#[non_exhaustive]
pub enum ValidationError {
    /// A length or radius that must be nonnegative was proved negative.
    #[error("{name} must be nonnegative")]
    NegativeParameter { name: &'static str },
    /// A scale or period that must be strictly positive is zero or negative.
    #[error("{name} must be positive")]
    NonPositiveParameter { name: &'static str },
    /// An operation requires a nonzero component or parameter.
    #[error("{name} must be nonzero")]
    ZeroParameter { name: &'static str },
    /// Extraction encountered a degenerate output triangle and published no mesh.
    #[error("output triangle {triangle_index} is degenerate")]
    DegenerateOutputTriangle { triangle_index: usize },
    /// A tessellation count does not meet the constructor's minimum.
    #[error("{name} must be at least {min}, got {actual}")]
    InvalidResolution {
        name: &'static str,
        min: usize,
        actual: usize,
    },
    /// The requested analytic primitive is outside the supported representation.
    #[error("unsupported primitive: {reason}")]
    UnsupportedPrimitive { reason: &'static str },
    /// A valid transform cannot be represented by the target geometry carrier.
    #[error("unsupported transform: {reason}")]
    UnsupportedTransform { reason: &'static str },
    /// A transform has no finite image under the admitted homogeneous map.
    #[error("unbounded transform: {reason}")]
    UnboundedTransform { reason: &'static str },
    /// A necessary predicate has no admissible proof under the selected policy.
    #[error("{predicate} is undecided under {policy:?}")]
    PredicateUndecided {
        predicate: &'static str,
        policy: hyperlimit::PredicatePolicy,
    },
    /// A geometry buffer length or byte size cannot be represented on this platform.
    #[error("geometry buffer capacity exceeds platform limits")]
    CapacityOverflow,
    /// Reserving a geometry buffer failed before a complete result was published.
    #[error("could not allocate geometry buffer for {elements} elements")]
    AllocationFailed { elements: usize },
    /// A checked scalar or lattice operation failed, retaining its native cause.
    #[error(transparent)]
    Scalar(#[from] hyperreal::Problem),
    /// A native mesh operation failed, retaining its typed cause.
    #[error("{0}")]
    Mesh(#[from] hypermesh::HypermeshError),
    /// Native graphics-buffer construction failed, retaining its typed cause.
    #[error("{0}")]
    Gpu(#[from] hypermesh::GpuMeshError),
    /// A native exact curve operation failed, retaining its context and cause.
    #[error("{0}")]
    ExactCurve(#[from] ExactCurveError),
    /// Native contour construction failed, retaining its original diagnostic.
    #[error("{0}")]
    Curve(#[from] CurveError),
    /// Native sampled-field extraction failed, retaining its typed diagnostic.
    #[error("{0}")]
    SurfaceNets(#[from] hypermesh::SurfaceNetsError),
    /// Native planar sampled-field extraction failed, retaining its typed cause.
    #[error("{0}")]
    MarchingSquares(#[from] hypercurve::MarchingSquaresError),
    /// The font face could not be parsed; retains the parser's diagnostic.
    #[cfg(feature = "truetype-text")]
    #[error("invalid font face: {0}")]
    FontFace(#[source] ttf_parser::FaceParsingError),
    /// A required font table is absent or malformed.
    #[cfg(feature = "truetype-text")]
    #[error("invalid or missing font table {table}")]
    InvalidFontTable { table: &'static str },
    /// The font's character map has no glyph for the requested character.
    #[cfg(feature = "truetype-text")]
    #[error("font has no glyph for {character:?}")]
    MissingFontGlyph { character: char },
    /// The selected glyph did not supply a complete valid outline.
    #[cfg(feature = "truetype-text")]
    #[error("invalid outline for font glyph {glyph_id}")]
    InvalidFontOutline { glyph_id: u16 },
    /// A parser-emitted outline coordinate could not be imported.
    #[cfg(feature = "truetype-text")]
    #[error("invalid coordinate in font glyph {glyph_id}: {source}")]
    FontCoordinate {
        glyph_id: u16,
        #[source]
        source: hyperreal::Problem,
    },
    /// Native Hershey text construction failed without publishing partial text.
    #[cfg(feature = "hershey-text")]
    #[error("{0}")]
    HersheyText(#[from] hypercurve::hershey::TextError),
    /// Native triangulation failed before a modeling surface was published.
    #[error("{0}")]
    Triangulation(#[from] hypertri::Error),
    /// A retained implicit-field construction failed before grid sampling.
    #[cfg(any(feature = "sdf", feature = "metaballs"))]
    #[error("{0}")]
    SdfEvaluation(#[from] hypersdf::SdfEvaluationError),
    /// A retained expression could not be evaluated at this grid sample.
    #[cfg(any(feature = "sdf", feature = "metaballs"))]
    #[error("SDF sample {sample:?}: {source}")]
    SdfSample {
        sample: [usize; 3],
        #[source]
        source: hypersdf::SdfEvaluationError,
    },
    /// Operation requires corresponding loops with the same number of vertices.
    #[error("operation requires loops with the same number of vertices, {left} != {right}")]
    MismatchedVertexCount { left: usize, right: usize },
    /// A required index is outside a collection.
    #[error("index {index} is out of range for length {len}")]
    IndexOutOfRangeWithLen { index: usize, len: usize },
    /// Feature arguments do not describe supported geometry.
    #[error("invalid arguments")]
    InvalidArguments,
    /// A named integer field is below the supported minimum.
    #[error("{name} must not be less than {min}")]
    FieldLessThan { name: &'static str, min: i32 },
    /// A native geometry dependency rejected the requested construction.
    #[error("{0}")]
    Geometry(String),
}

/// Stable failure categories exposed by modeling operations and bindings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ValidationErrorKind {
    InvalidInput,
    Unsupported,
    Undecided,
    ResourceExhausted,
    Unbounded,
    Geometry,
}

impl ValidationErrorKind {
    /// Machine-readable category used by JavaScript exceptions.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidInput => "invalid_input",
            Self::Unsupported => "unsupported",
            Self::Undecided => "undecided",
            Self::ResourceExhausted => "resource_exhausted",
            Self::Unbounded => "unbounded",
            Self::Geometry => "geometry_failure",
        }
    }
}

const fn curve_error_kind(source: &CurveError) -> ValidationErrorKind {
    match source {
        CurveError::AmbiguousBulge | CurveError::AmbiguousCurveStringConnection => {
            ValidationErrorKind::Undecided
        },
        CurveError::UnsupportedNurbsInterpolationDivision { .. } => {
            ValidationErrorKind::Unsupported
        },
        CurveError::Topology(_) | CurveError::Real(_) => ValidationErrorKind::Geometry,
        _ => ValidationErrorKind::InvalidInput,
    }
}

impl ValidationError {
    pub const fn kind(&self) -> ValidationErrorKind {
        match self {
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
                )) => match blocker.reason() {
                    hypercurve::UncertaintyReason::Unsupported => {
                        ValidationErrorKind::Unsupported
                    },
                    hypercurve::UncertaintyReason::BudgetExhausted => {
                        ValidationErrorKind::ResourceExhausted
                    },
                    hypercurve::UncertaintyReason::Boundary => {
                        ValidationErrorKind::InvalidInput
                    },
                    _ => ValidationErrorKind::Undecided,
                },
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
            Self::ExactCurve(ExactCurveError::Blocked(blocker)) => match blocker.reason() {
                hypercurve::UncertaintyReason::Unsupported => ValidationErrorKind::Unsupported,
                hypercurve::UncertaintyReason::BudgetExhausted => {
                    ValidationErrorKind::ResourceExhausted
                },
                hypercurve::UncertaintyReason::Boundary => ValidationErrorKind::InvalidInput,
                _ => ValidationErrorKind::Undecided,
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
