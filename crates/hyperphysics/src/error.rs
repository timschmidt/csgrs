//! Error types for exact physics construction and reports.

/// Result alias used by `hyperphysics`.
pub type PhysicsResult<T> = Result<T, PhysicsError>;

/// Errors surfaced by exact-aware physics carriers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PhysicsError {
    /// A caller supplied an empty identifier for a material, body, or fixture.
    EmptyIdentifier,
    /// A triangle mesh had no triangles.
    EmptyTriangleMesh,
    /// A requested density was not certified strictly positive.
    NonPositiveDensity,
    /// The oriented mesh volume was certified as zero.
    ZeroVolume,
    /// The signed-volume certification budget did not prove an orientation.
    UnknownSignedVolume,
    /// A time step was not certified strictly positive.
    NonPositiveTimeStep,
    /// A mass value was not certified strictly positive.
    NonPositiveMass,
    /// A replay candidate had an unexpected vector length.
    CandidateLengthMismatch,
    /// A replay diagnostic could not certify a sign or equality.
    UnknownDiagnostic,
    /// A thermal conductivity was not certified strictly positive.
    NonPositiveThermalConductivity,
    /// A slab thickness was not certified strictly positive.
    NonPositiveThickness,
    /// A contact area was not certified strictly positive.
    NonPositiveArea,
    /// A resistance was not certified non-negative.
    NegativeResistance,
    /// A thermal denominator was zero or undecidable.
    InvalidThermalDenominator,
    /// A refractive index was not certified strictly positive.
    NonPositiveRefractiveIndex,
    /// An absorption coefficient was certified negative.
    NegativeAbsorptionCoefficient,
    /// A ray/interface classification could not certify a sign.
    UnknownOpticalClassification,
    /// A box bound had `min > max` or could not certify `min <= max`.
    InvalidAxisAlignedBox,
    /// A shape query could not certify a sign.
    UnknownShapeQuery,
    /// An exposure value was not certified strictly positive.
    NonPositiveExposure,
    /// A penetration depth was not certified strictly positive.
    NonPositivePenetrationDepth,
    /// A layer thickness was not certified strictly positive.
    NonPositiveLayerThickness,
    /// A concentration was certified negative.
    NegativeConcentration,
    /// A photochemical comparison could not certify an ordering.
    UnknownPhotochemicalDecision,
    /// A property interval had `lower > upper` or could not certify `lower <= upper`.
    InvalidPropertyInterval,
    /// An elastic constant was not certified inside its supported domain.
    InvalidElasticConstant,
    /// A permittivity value was not certified strictly positive.
    NonPositivePermittivity,
    /// A permeability value was not certified strictly positive.
    NonPositivePermeability,
    /// An electrical conductivity value was certified negative.
    NegativeElectricalConductivity,
    /// A photochemical conversion/fraction was outside `[0, 1]` or undecidable.
    InvalidFraction,
    /// A diffusion coefficient was certified negative.
    NegativeDiffusionCoefficient,
    /// A grid spacing was not certified strictly positive.
    NonPositiveGridSpacing,
    /// A thermal capacitance/heat capacity was not certified strictly positive.
    NonPositiveThermalCapacitance,
    /// A viscosity value was certified negative.
    NegativeViscosity,
    /// A particle mass was not certified strictly positive.
    NonPositiveParticleMass,
    /// An SPH smoothing length was not certified strictly positive.
    NonPositiveSmoothingLength,
    /// A friction coefficient was certified negative.
    NegativeFrictionCoefficient,
    /// A restitution coefficient was outside `[0, 1]` or undecidable.
    InvalidRestitutionCoefficient,
}
