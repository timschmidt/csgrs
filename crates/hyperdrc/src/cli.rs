use std::path::PathBuf;

use clap::{Parser, ValueEnum};

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
/// Public enumeration for `Check`.
pub enum Check {
    /// Variant `MaskIslandKeepout`.
    MaskIslandKeepout,
    /// Variant `CopperOverlap`.
    CopperOverlap,
    /// Variant `BoardEdgeClearance`.
    BoardEdgeClearance,
    /// Variant `BoardOutlineCutoutClearance`.
    BoardOutlineCutoutClearance,
    /// Variant `BoardOutlineSanity`.
    BoardOutlineSanity,
    /// Variant `BoardOutlineFragments`.
    BoardOutlineFragments,
    /// Variant `BoardOutlineSelfIntersectionReadiness`.
    BoardOutlineSelfIntersectionReadiness,
    /// Variant `BoardOutlineNotchReadiness`.
    BoardOutlineNotchReadiness,
    /// Variant `BoardOutlineDuplicateReadiness`.
    BoardOutlineDuplicateReadiness,
    /// Variant `BoardOutlineNestingReadiness`.
    BoardOutlineNestingReadiness,
    /// Variant `PasteOverhang`.
    PasteOverhang,
    /// Variant `PasteApertureCoverage`.
    PasteApertureCoverage,
    /// Variant `PasteApertureRatio`.
    PasteApertureRatio,
    /// Variant `ThermalPadPasteWindowpaneReadiness`.
    ThermalPadPasteWindowpaneReadiness,
    /// Variant `StencilAreaRatioReadiness`.
    StencilAreaRatioReadiness,
    /// Variant `PasteApertureAspectRatioReadiness`.
    PasteApertureAspectRatioReadiness,
    /// Variant `TombstonePasteImbalanceReadiness`.
    TombstonePasteImbalanceReadiness,
    /// Variant `PasteViaExposureReadiness`.
    PasteViaExposureReadiness,
    /// Variant `MinimumPasteAperture`.
    MinimumPasteAperture,
    /// Variant `PasteApertureSpacing`.
    PasteApertureSpacing,
    /// Variant `PasteMaskAlignment`.
    PasteMaskAlignment,
    /// Variant `ExposedCopper`.
    ExposedCopper,
    /// Variant `SolderMaskOpeningCoverage`.
    SolderMaskOpeningCoverage,
    /// Variant `SolderMaskExpansion`.
    SolderMaskExpansion,
    /// Variant `SolderMaskOverlapClearance`.
    SolderMaskOverlapClearance,
    /// Variant `SolderMaskBoardEdgeClearance`.
    SolderMaskBoardEdgeClearance,
    /// Variant `SilkscreenOverlap`.
    SilkscreenOverlap,
    /// Variant `SilkscreenClearance`.
    SilkscreenClearance,
    /// Variant `SilkscreenBoardEdgeClearance`.
    SilkscreenBoardEdgeClearance,
    /// Variant `SilkscreenMinWidth`.
    SilkscreenMinWidth,
    /// Variant `MinCopperNeck`.
    MinCopperNeck,
    /// Variant `AcidTrap`.
    AcidTrap,
    /// Variant `AcidTrapTraceJunction`.
    AcidTrapTraceJunction,
    /// Variant `LayerSanity`.
    LayerSanity,
    /// Variant `CopperBalance`.
    CopperBalance,
    /// Variant `LocalCopperDensityReadiness`.
    LocalCopperDensityReadiness,
    /// Variant `MechanicalLayerGeometry`.
    MechanicalLayerGeometry,
    /// Variant `SolderMaskSliver`.
    SolderMaskSliver,
    /// Variant `MinimumMaskOpening`.
    MinimumMaskOpening,
    /// Variant `SolderMaskOpeningSpacing`.
    SolderMaskOpeningSpacing,
    /// Variant `AnnularRing`.
    AnnularRing,
    /// Variant `AnnularRingTolerance`.
    AnnularRingTolerance,
    /// Variant `PlatingIntent`.
    PlatingIntent,
    /// Variant `RoutedSlotReadiness`.
    RoutedSlotReadiness,
    /// Variant `CastellationIntent`.
    CastellationIntent,
    /// Variant `CastellationHoleReadiness`.
    CastellationHoleReadiness,
    /// Variant `ViaInPadReadiness`.
    ViaInPadReadiness,
    /// Variant `DrillCopperClearance`.
    DrillCopperClearance,
    /// Variant `DrillToCopperClearance`.
    DrillToCopperClearance,
    /// Variant `BoardOutlineDrillClearance`.
    BoardOutlineDrillClearance,
    /// Variant `DrillSpacing`.
    DrillSpacing,
    /// Variant `DrillAspectRatio`.
    DrillAspectRatio,
    /// Variant `DrillTableConsistency`.
    DrillTableConsistency,
    /// Variant `CopperWidthReadiness`.
    CopperWidthReadiness,
    /// Variant `CopperNetIntent`.
    CopperNetIntent,
    /// Variant `TeardropReadiness`.
    TeardropReadiness,
    /// Variant `ThermalReliefReadiness`.
    ThermalReliefReadiness,
    /// Variant `PlaneClearanceReadiness`.
    PlaneClearanceReadiness,
    /// Variant `BoardEdgeExposure`.
    BoardEdgeExposure,
    /// Variant `HighSpeedEdgeReadiness`.
    HighSpeedEdgeReadiness,
    /// Variant `EdgeCopperPullbackReadiness`.
    EdgeCopperPullbackReadiness,
    /// Variant `HighVoltageEdgeReadiness`.
    HighVoltageEdgeReadiness,
    /// Variant `ControlledImpedanceReadiness`.
    ControlledImpedanceReadiness,
    /// Variant `DifferentialPairReadiness`.
    DifferentialPairReadiness,
    /// Variant `DifferentialPairSpacingReadiness`.
    DifferentialPairSpacingReadiness,
    /// Variant `DifferentialPairWidthReadiness`.
    DifferentialPairWidthReadiness,
    /// Variant `DifferentialPairNeckdownReadiness`.
    DifferentialPairNeckdownReadiness,
    /// Variant `DifferentialPairSkewReadiness`.
    DifferentialPairSkewReadiness,
    /// Variant `DifferentialPairToPairSpacingReadiness`.
    DifferentialPairToPairSpacingReadiness,
    /// Variant `DifferentialPairViaProximityReadiness`.
    DifferentialPairViaProximityReadiness,
    /// Variant `DifferentialPairViaReturnReadiness`.
    DifferentialPairViaReturnReadiness,
    /// Variant `DifferentialPairViaSymmetryReadiness`.
    DifferentialPairViaSymmetryReadiness,
    /// Variant `DifferentialPairReturnReadiness`.
    DifferentialPairReturnReadiness,
    /// Variant `ReferencePlaneReadiness`.
    ReferencePlaneReadiness,
    /// Variant `ReferencePlaneVoidReadiness`.
    ReferencePlaneVoidReadiness,
    /// Variant `SplitPlaneCrossingReadiness`.
    SplitPlaneCrossingReadiness,
    /// Variant `ReturnPathProximityReadiness`.
    ReturnPathProximityReadiness,
    /// Variant `OrphanedZoneReadiness`.
    OrphanedZoneReadiness,
    /// Variant `SameNetIslandReadiness`.
    SameNetIslandReadiness,
    /// Variant `SameNetDrillBreakReadiness`.
    SameNetDrillBreakReadiness,
    /// Variant `DifferentNetShortReadiness`.
    DifferentNetShortReadiness,
    /// Variant `ReturnPathReadiness`.
    ReturnPathReadiness,
    /// Variant `HighCurrentReadiness`.
    HighCurrentReadiness,
    /// Variant `PowerViaArrayReadiness`.
    PowerViaArrayReadiness,
    /// Variant `PowerViaReturnReadiness`.
    PowerViaReturnReadiness,
    /// Variant `ThermalViaReadiness`.
    ThermalViaReadiness,
    /// Variant `ThermalViaDistributionReadiness`.
    ThermalViaDistributionReadiness,
    /// Variant `PowerPlaneReadiness`.
    PowerPlaneReadiness,
    /// Variant `HighCurrentNeckReadiness`.
    HighCurrentNeckReadiness,
    /// Variant `PowerPadEntryReadiness`.
    PowerPadEntryReadiness,
    /// Variant `VoltageClearanceReadiness`.
    VoltageClearanceReadiness,
    /// Variant `ProtectiveEarthSpacingReadiness`.
    ProtectiveEarthSpacingReadiness,
    /// Variant `SurgeProtectionKeepoutReadiness`.
    SurgeProtectionKeepoutReadiness,
    /// Variant `SensitiveNetSpacingReadiness`.
    SensitiveNetSpacingReadiness,
    /// Variant `SensitiveReturnReadiness`.
    SensitiveReturnReadiness,
    /// Variant `MixedSignalPartitionReadiness`.
    MixedSignalPartitionReadiness,
    /// Variant `RfKeepoutReadiness`.
    RfKeepoutReadiness,
    /// Variant `AntennaCopperKeepoutReadiness`.
    AntennaCopperKeepoutReadiness,
    /// Variant `RfViaFenceReadiness`.
    RfViaFenceReadiness,
    /// Variant `ChassisStitchingReadiness`.
    ChassisStitchingReadiness,
    /// Variant `EdgeStitchingReadiness`.
    EdgeStitchingReadiness,
    /// Variant `GoldFingerReadiness`.
    GoldFingerReadiness,
    /// Variant `GoldFingerEdgeReadiness`.
    GoldFingerEdgeReadiness,
    /// Variant `GoldFingerSpacingReadiness`.
    GoldFingerSpacingReadiness,
    /// Variant `GoldFingerDrillKeepoutReadiness`.
    GoldFingerDrillKeepoutReadiness,
    /// Variant `ComponentEdgeClearanceReadiness`.
    ComponentEdgeClearanceReadiness,
    /// Variant `ComponentHoleClearanceReadiness`.
    ComponentHoleClearanceReadiness,
    /// Variant `ComponentSpacingReadiness`.
    ComponentSpacingReadiness,
    /// Variant `ConnectorReworkClearanceReadiness`.
    ConnectorReworkClearanceReadiness,
    /// Variant `PadPairAsymmetryReadiness`.
    PadPairAsymmetryReadiness,
    /// Variant `ConnectorReturnPathReadiness`.
    ConnectorReturnPathReadiness,
    /// Variant `DecouplingProximityReadiness`.
    DecouplingProximityReadiness,
    /// Variant `EsdProtectionReadiness`.
    EsdProtectionReadiness,
    /// Variant `EsdReturnPathReadiness`.
    EsdReturnPathReadiness,
    /// Variant `SwitchNodeKeepoutReadiness`.
    SwitchNodeKeepoutReadiness,
    /// Variant `InductorCopperKeepoutReadiness`.
    InductorCopperKeepoutReadiness,
    /// Variant `TestpointCoverageReadiness`.
    TestpointCoverageReadiness,
    /// Variant `TestpointAccessibilityReadiness`.
    TestpointAccessibilityReadiness,
    /// Variant `TestpointCopperClearanceReadiness`.
    TestpointCopperClearanceReadiness,
    /// Variant `ToolingHoleReadiness`.
    ToolingHoleReadiness,
    /// Variant `MouseBiteReadiness`.
    MouseBiteReadiness,
    /// Variant `FiducialReadiness`.
    FiducialReadiness,
    /// Variant `LocalFiducialReadiness`.
    LocalFiducialReadiness,
    /// Variant `FiducialKeepoutReadiness`.
    FiducialKeepoutReadiness,
    /// Variant `DensePadEscapeReadiness`.
    DensePadEscapeReadiness,
    /// Variant `DensePadViaSpacingReadiness`.
    DensePadViaSpacingReadiness,
    /// Variant `DensePadMaskBridgeReadiness`.
    DensePadMaskBridgeReadiness,
    /// Variant `SelectiveWaveSolderKeepoutReadiness`.
    SelectiveWaveSolderKeepoutReadiness,
    /// Variant `PressFitKeepoutReadiness`.
    PressFitKeepoutReadiness,
    /// Variant `ConformalCoatingKeepoutReadiness`.
    ConformalCoatingKeepoutReadiness,
    /// Variant `ThermalPadViaReadiness`.
    ThermalPadViaReadiness,
    /// Variant `ThermalCopperAreaReadiness`.
    ThermalCopperAreaReadiness,
    /// Variant `HotComponentSpacingReadiness`.
    HotComponentSpacingReadiness,
    /// Variant `ThermalMechanicalKeepoutReadiness`.
    ThermalMechanicalKeepoutReadiness,
    /// Variant `MountingHoleGroundingReadiness`.
    MountingHoleGroundingReadiness,
    /// Variant `MountingHoleCopperKeepoutReadiness`.
    MountingHoleCopperKeepoutReadiness,
    /// Variant `MountingHoleEdgeClearanceReadiness`.
    MountingHoleEdgeClearanceReadiness,
    /// Variant `MountingHolePlatingIntentReadiness`.
    MountingHolePlatingIntentReadiness,
    /// Variant `MountingHoleDistributionReadiness`.
    MountingHoleDistributionReadiness,
    /// Variant `MountingHoleSpacingReadiness`.
    MountingHoleSpacingReadiness,
    /// Variant `PanelFeatureOutlineReadiness`.
    PanelFeatureOutlineReadiness,
    /// Variant `EdgePlatingIntentReadiness`.
    EdgePlatingIntentReadiness,
    /// Variant `CastellationPitchReadiness`.
    CastellationPitchReadiness,
    /// Variant `NetSpacing`.
    NetSpacing,
    /// Variant `DifferentNetSpacing`.
    DifferentNetSpacing,
    /// Variant `RegistrationTolerance`.
    RegistrationTolerance,
    /// Variant `LayerRegistrationTolerance`.
    LayerRegistrationTolerance,
    /// Variant `PanelizationClearance`.
    PanelizationClearance,
    /// Variant `Ipc356Coverage`.
    Ipc356Coverage,
    /// Variant `Ipc356DrillDiameter`.
    Ipc356DrillDiameter,
    /// Variant `ExcellonReadiness`.
    ExcellonReadiness,
    /// Variant `FileManifestReadiness`.
    FileManifestReadiness,
    /// Variant `ProductionArtifactReadiness`.
    ProductionArtifactReadiness,
    /// Variant `StackupReadiness`.
    StackupReadiness,
    /// Variant `NetConstraintReadiness`.
    NetConstraintReadiness,
    /// Variant `WaiverGovernance`.
    WaiverGovernance,
}

pub const DEFAULT_CHECKS: &[Check] = &[
    Check::MaskIslandKeepout,
    Check::CopperOverlap,
    Check::BoardEdgeClearance,
    Check::BoardOutlineCutoutClearance,
    Check::BoardOutlineSanity,
    Check::BoardOutlineFragments,
    Check::BoardOutlineSelfIntersectionReadiness,
    Check::BoardOutlineNotchReadiness,
    Check::BoardOutlineDuplicateReadiness,
    Check::BoardOutlineNestingReadiness,
    Check::PasteOverhang,
    Check::PasteApertureCoverage,
    Check::PasteApertureRatio,
    Check::ThermalPadPasteWindowpaneReadiness,
    Check::StencilAreaRatioReadiness,
    Check::PasteApertureAspectRatioReadiness,
    Check::TombstonePasteImbalanceReadiness,
    Check::PasteViaExposureReadiness,
    Check::MinimumPasteAperture,
    Check::PasteApertureSpacing,
    Check::PasteMaskAlignment,
    Check::ExposedCopper,
    Check::SolderMaskOpeningCoverage,
    Check::SolderMaskExpansion,
    Check::SolderMaskOverlapClearance,
    Check::SolderMaskBoardEdgeClearance,
    Check::SilkscreenOverlap,
    Check::SilkscreenClearance,
    Check::SilkscreenBoardEdgeClearance,
    Check::SilkscreenMinWidth,
    Check::MinCopperNeck,
    Check::AcidTrap,
    Check::AcidTrapTraceJunction,
    Check::LayerSanity,
    Check::CopperBalance,
    Check::LocalCopperDensityReadiness,
    Check::MechanicalLayerGeometry,
    Check::SolderMaskSliver,
    Check::MinimumMaskOpening,
    Check::SolderMaskOpeningSpacing,
    Check::AnnularRing,
    Check::AnnularRingTolerance,
    Check::PlatingIntent,
    Check::RoutedSlotReadiness,
    Check::CastellationIntent,
    Check::CastellationHoleReadiness,
    Check::ViaInPadReadiness,
    Check::DrillToCopperClearance,
    Check::BoardOutlineDrillClearance,
    Check::DrillSpacing,
    Check::DrillAspectRatio,
    Check::DrillTableConsistency,
    Check::CopperWidthReadiness,
    Check::CopperNetIntent,
    Check::TeardropReadiness,
    Check::ThermalReliefReadiness,
    Check::PlaneClearanceReadiness,
    Check::BoardEdgeExposure,
    Check::HighSpeedEdgeReadiness,
    Check::EdgeCopperPullbackReadiness,
    Check::HighVoltageEdgeReadiness,
    Check::ControlledImpedanceReadiness,
    Check::DifferentialPairReadiness,
    Check::DifferentialPairSpacingReadiness,
    Check::DifferentialPairWidthReadiness,
    Check::DifferentialPairNeckdownReadiness,
    Check::DifferentialPairSkewReadiness,
    Check::DifferentialPairToPairSpacingReadiness,
    Check::DifferentialPairViaProximityReadiness,
    Check::DifferentialPairViaReturnReadiness,
    Check::DifferentialPairViaSymmetryReadiness,
    Check::DifferentialPairReturnReadiness,
    Check::ReferencePlaneReadiness,
    Check::ReferencePlaneVoidReadiness,
    Check::SplitPlaneCrossingReadiness,
    Check::ReturnPathProximityReadiness,
    Check::OrphanedZoneReadiness,
    Check::SameNetIslandReadiness,
    Check::SameNetDrillBreakReadiness,
    Check::DifferentNetShortReadiness,
    Check::ReturnPathReadiness,
    Check::HighCurrentReadiness,
    Check::PowerViaArrayReadiness,
    Check::PowerViaReturnReadiness,
    Check::ThermalViaReadiness,
    Check::ThermalViaDistributionReadiness,
    Check::PowerPlaneReadiness,
    Check::HighCurrentNeckReadiness,
    Check::PowerPadEntryReadiness,
    Check::VoltageClearanceReadiness,
    Check::ProtectiveEarthSpacingReadiness,
    Check::SurgeProtectionKeepoutReadiness,
    Check::SensitiveNetSpacingReadiness,
    Check::SensitiveReturnReadiness,
    Check::MixedSignalPartitionReadiness,
    Check::RfKeepoutReadiness,
    Check::AntennaCopperKeepoutReadiness,
    Check::RfViaFenceReadiness,
    Check::ChassisStitchingReadiness,
    Check::EdgeStitchingReadiness,
    Check::GoldFingerReadiness,
    Check::GoldFingerEdgeReadiness,
    Check::GoldFingerSpacingReadiness,
    Check::GoldFingerDrillKeepoutReadiness,
    Check::ComponentEdgeClearanceReadiness,
    Check::ComponentHoleClearanceReadiness,
    Check::ComponentSpacingReadiness,
    Check::ConnectorReworkClearanceReadiness,
    Check::PadPairAsymmetryReadiness,
    Check::ConnectorReturnPathReadiness,
    Check::DecouplingProximityReadiness,
    Check::EsdProtectionReadiness,
    Check::EsdReturnPathReadiness,
    Check::SwitchNodeKeepoutReadiness,
    Check::InductorCopperKeepoutReadiness,
    Check::TestpointCoverageReadiness,
    Check::TestpointAccessibilityReadiness,
    Check::TestpointCopperClearanceReadiness,
    Check::ToolingHoleReadiness,
    Check::MouseBiteReadiness,
    Check::FiducialReadiness,
    Check::LocalFiducialReadiness,
    Check::FiducialKeepoutReadiness,
    Check::DensePadEscapeReadiness,
    Check::DensePadViaSpacingReadiness,
    Check::DensePadMaskBridgeReadiness,
    Check::SelectiveWaveSolderKeepoutReadiness,
    Check::PressFitKeepoutReadiness,
    Check::ConformalCoatingKeepoutReadiness,
    Check::ThermalPadViaReadiness,
    Check::ThermalCopperAreaReadiness,
    Check::HotComponentSpacingReadiness,
    Check::ThermalMechanicalKeepoutReadiness,
    Check::MountingHoleGroundingReadiness,
    Check::MountingHoleCopperKeepoutReadiness,
    Check::MountingHoleEdgeClearanceReadiness,
    Check::MountingHolePlatingIntentReadiness,
    Check::MountingHoleDistributionReadiness,
    Check::MountingHoleSpacingReadiness,
    Check::PanelFeatureOutlineReadiness,
    Check::EdgePlatingIntentReadiness,
    Check::CastellationPitchReadiness,
    Check::DifferentNetSpacing,
    Check::LayerRegistrationTolerance,
    Check::PanelizationClearance,
    Check::Ipc356Coverage,
    Check::Ipc356DrillDiameter,
    Check::ExcellonReadiness,
    Check::FileManifestReadiness,
    Check::ProductionArtifactReadiness,
    Check::StackupReadiness,
    Check::NetConstraintReadiness,
    Check::WaiverGovernance,
];

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
/// Public enumeration for `OutputFormat`.
pub enum OutputFormat {
    /// Variant `Text`.
    Text,
    /// Variant `Json`.
    Json,
    /// Variant `Jsonl`.
    Jsonl,
    /// Variant `Geojson`.
    Geojson,
    /// Variant `Sarif`.
    Sarif,
    /// Variant `GithubAnnotations`.
    GithubAnnotations,
    /// Variant `Html`.
    Html,
    /// Variant `Junit`.
    Junit,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
/// Public enumeration for `ConversionBackend`.
pub enum ConversionBackend {
    /// Variant `Transjlc`.
    Transjlc,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
/// Public enumeration for `SourceEda`.
pub enum SourceEda {
    /// Variant `Auto`.
    Auto,
    /// Variant `Kicad`.
    Kicad,
    /// Variant `Jlc`.
    Jlc,
    /// Variant `Protel`.
    Protel,
}

impl SourceEda {
    /// Run or compute `as_transjlc_arg`.
    pub fn as_transjlc_arg(self) -> &'static str {
        match self {
            SourceEda::Auto => "auto",
            SourceEda::Kicad => "kicad",
            SourceEda::Jlc => "jlc",
            SourceEda::Protel => "protel",
        }
    }
}

#[derive(Parser, Debug)]
#[command(author, version, about)]
/// Public data model for `Cli`.
pub struct Cli {
    /// JSON rule configuration file.
    #[arg(long = "config")]
    /// Field `config`.
    pub config: Option<PathBuf>,

    /// Gerber files to load as separate layers.
    pub files: Vec<PathBuf>,

    /// Directory containing Gerber files to load as layers. Repeat to merge multiple packages.
    #[arg(long = "gerber-dir")]
    /// Field `gerber_dirs`.
    pub gerber_dirs: Vec<PathBuf>,

    /// Gerber directory to convert before loading. Repeat for multiple input packages.
    #[arg(long = "convert-input")]
    /// Field `conversion_inputs`.
    pub conversion_inputs: Vec<PathBuf>,

    /// Converter backend for --convert-input packages.
    #[arg(long = "converter", value_enum, default_value_t = ConversionBackend::Transjlc)]
    /// Field `converter`.
    pub converter: ConversionBackend,

    /// Base directory for converted Gerber output.
    #[arg(long = "conversion-output-dir", default_value = "hyperdrc-converted")]
    /// Field `conversion_output_dir`.
    pub conversion_output_dir: PathBuf,

    /// Source EDA passed to the converter.
    #[arg(long = "source-eda", value_enum, default_value_t = SourceEda::Auto)]
    /// Field `source_eda`.
    pub source_eda: SourceEda,

    /// Path to the TransJLC executable used by --converter transjlc.
    #[arg(long = "transjlc-bin", default_value = "TransJLC")]
    /// Field `transjlc_bin`.
    pub transjlc_bin: PathBuf,

    /// Ask the converter to create a zip archive when supported.
    #[arg(long = "conversion-zip")]
    /// Field `conversion_zip`.
    pub conversion_zip: bool,

    /// Zip file base name passed to converters that support zipped output.
    #[arg(long = "conversion-zip-name", default_value = "Gerber")]
    /// Field `conversion_zip_name`.
    pub conversion_zip_name: String,

    /// Optional top colorful silkscreen image passed through to TransJLC.
    #[arg(long = "top-color-image")]
    /// Field `top_color_image`.
    pub top_color_image: Option<PathBuf>,

    /// Optional bottom colorful silkscreen image passed through to TransJLC.
    #[arg(long = "bottom-color-image")]
    /// Field `bottom_color_image`.
    pub bottom_color_image: Option<PathBuf>,

    /// Extra command-line arguments passed to the selected converter backend.
    #[arg(long = "conversion-arg", value_name = "ARG")]
    /// Field `conversion_args`.
    pub conversion_args: Vec<String>,

    /// KiCad .kicad_pcb file. Repeat to check multiple boards.
    #[arg(long = "kicad-pcb")]
    /// Field `kicad_pcbs`.
    pub kicad_pcbs: Vec<PathBuf>,

    /// Excellon drill file. Repeat for plated, non-plated, or panel drill files.
    #[arg(long = "excellon")]
    /// Field `excellon_files`.
    pub excellon_files: Vec<PathBuf>,

    /// IPC-D-356 netlist file. Repeat to merge multiple electrical test netlists.
    #[arg(long = "ipc356")]
    /// Field `ipc356_files`.
    pub ipc356_files: Vec<PathBuf>,

    /// Bill of materials file.
    #[arg(long = "bom")]
    /// Field `bom_files`.
    pub bom_files: Vec<PathBuf>,

    /// Placement / centroid file.
    #[arg(long = "centroid")]
    /// Field `centroid_files`.
    pub centroid_files: Vec<PathBuf>,

    /// Netlist source for pre-production validation and manifest completeness.
    #[arg(long = "netlist")]
    /// Field `netlist_files`.
    pub netlist_files: Vec<PathBuf>,

    /// Mechanical fabricator drawing file.
    #[arg(long = "fab-drawing")]
    /// Field `fab_drawing_files`.
    pub fab_drawing_files: Vec<PathBuf>,

    /// Assembly drawing, instruction, or fixture file.
    #[arg(long = "assembly-drawing")]
    /// Field `assembly_drawing_files`.
    pub assembly_drawing_files: Vec<PathBuf>,

    /// Readme or release-notes file describing the package.
    #[arg(long = "readme")]
    /// Field `readme_files`.
    pub readme_files: Vec<PathBuf>,

    /// Route, V-score, or tooling drawing for panelization review.
    #[arg(long = "rout-drawing")]
    /// Field `rout_drawing_files`.
    pub rout_drawing_files: Vec<PathBuf>,

    /// JSON waiver file. Repeat to combine waiver sets.
    #[arg(long = "waiver")]
    /// Field `waiver_files`.
    pub waiver_files: Vec<PathBuf>,

    /// Check(s) to run. Repeat the flag to run a sequence.
    #[arg(short = 'c', long = "check", value_enum)]
    /// Field `checks`.
    pub checks: Vec<Check>,

    /// Keepout distance for mask island isolation checks, in Gerber units.
    #[arg(long)]
    /// Field `keepout`.
    pub keepout: Option<f64>,

    /// Board outline layer index for board-edge clearance checks.
    #[arg(long)]
    /// Field `board_outline`.
    pub board_outline: Option<usize>,

    /// Copper layer index. Repeat to restrict copper-related checks.
    #[arg(long = "copper-layer")]
    /// Field `copper_layers`.
    pub copper_layers: Vec<usize>,

    /// KiCad copper layer name. Repeat to restrict KiCad copper checks, for example F.Cu.
    #[arg(long = "kicad-copper-layer")]
    /// Field `kicad_copper_layers`.
    pub kicad_copper_layers: Vec<String>,

    /// Layer pairs for copper overlap checks, written as zero-based indexes like 0:1.
    /// If omitted, all unique file pairs are checked.
    #[arg(long = "pair")]
    /// Field `pairs`.
    pub pairs: Vec<String>,

    /// Paste-to-copper layer pairs for paste overhang checks, written as PASTE:COPPER.
    #[arg(long = "paste-pair")]
    /// Field `paste_pairs`.
    pub paste_pairs: Vec<String>,

    /// Copper-to-mask-opening layer pairs for exposed copper checks, written as COPPER:MASK.
    #[arg(long = "mask-pair")]
    /// Field `mask_pairs`.
    pub mask_pairs: Vec<String>,

    /// Solder mask layer index. Repeat to run mask sliver checks on Gerber layers.
    #[arg(long = "mask-layer")]
    /// Field `mask_layers`.
    pub mask_layers: Vec<usize>,

    /// Silkscreen-to-blocker layer pairs for silkscreen overlap checks, written as SILK:BLOCKER.
    #[arg(long = "silk-pair")]
    /// Field `silk_pairs`.
    pub silk_pairs: Vec<String>,

    /// Silkscreen layer index. Repeat to run silkscreen width checks on Gerber layers.
    #[arg(long = "silk-layer")]
    /// Field `silk_layers`.
    pub silk_layers: Vec<usize>,

    /// Clearance distance for board-edge checks, in Gerber units.
    #[arg(long)]
    /// Field `clearance`.
    pub clearance: Option<f64>,

    /// Allowed paste overhang beyond copper, in Gerber units.
    #[arg(long)]
    /// Field `paste_tolerance`.
    pub paste_tolerance: Option<f64>,

    /// Minimum paste-to-copper area ratio for each paired copper island.
    #[arg(long)]
    /// Field `min_paste_area_ratio`.
    pub min_paste_area_ratio: Option<f64>,

    /// Maximum paste-to-copper area ratio for each paired copper island.
    #[arg(long)]
    /// Field `max_paste_area_ratio`.
    pub max_paste_area_ratio: Option<f64>,

    /// Stencil foil thickness used by stencil area-ratio readiness checks.
    #[arg(long)]
    /// Field `stencil_thickness`.
    pub stencil_thickness: Option<f64>,

    /// Minimum acceptable stencil aperture area ratio.
    #[arg(long)]
    /// Field `min_stencil_area_ratio`.
    pub min_stencil_area_ratio: Option<f64>,

    /// Minimum copper width used by the neck-width morphology check.
    #[arg(long)]
    /// Field `min_width`.
    pub min_width: Option<f64>,

    /// Minimum solder mask web width used by the sliver morphology check.
    #[arg(long)]
    /// Field `min_mask_width`.
    pub min_mask_width: Option<f64>,

    /// Maximum interior angle to report as an acid-trap candidate.
    #[arg(long)]
    /// Field `acid_trap_angle`.
    pub acid_trap_angle: Option<f64>,

    /// Warn when the largest selected copper layer area exceeds the smallest by this ratio.
    #[arg(long)]
    /// Field `max_copper_imbalance_ratio`.
    pub max_copper_imbalance_ratio: Option<f64>,

    /// Minimum acceptable annular ring around plated drills, in KiCad units.
    #[arg(long)]
    /// Field `annular_ring`.
    pub annular_ring: Option<f64>,

    /// Drill-to-copper clearance, in KiCad or Excellon units.
    #[arg(long)]
    /// Field `drill_clearance`.
    pub drill_clearance: Option<f64>,

    /// Finished board thickness used by drill aspect-ratio readiness checks.
    #[arg(long)]
    /// Field `board_thickness`.
    pub board_thickness: Option<f64>,

    /// Maximum allowed board-thickness-to-drill-diameter ratio.
    #[arg(long)]
    /// Field `max_drill_aspect_ratio`.
    pub max_drill_aspect_ratio: Option<f64>,

    /// Different-net copper spacing for KiCad net-aware checks.
    #[arg(long)]
    /// Field `net_clearance`.
    pub net_clearance: Option<f64>,

    /// Layer registration tolerance for cross-layer KiCad copper proximity checks.
    #[arg(long)]
    /// Field `registration_tolerance`.
    pub registration_tolerance: Option<f64>,

    /// Clearance from copper to panel features, NPTH drills, or Excellon panel drills.
    #[arg(long)]
    /// Field `panel_clearance`.
    pub panel_clearance: Option<f64>,

    /// Coordinate tolerance for matching IPC-D-356 records to parsed board geometry.
    #[arg(long)]
    /// Field `ipc356_tolerance`.
    pub ipc356_tolerance: Option<f64>,

    /// Print detected KiCad copper layers to stderr before running checks.
    #[arg(long)]
    /// Field `list_kicad_layers`.
    pub list_kicad_layers: bool,

    /// Ignore violation shapes whose area is at or below this threshold.
    #[arg(long)]
    /// Field `min_area`.
    pub min_area: Option<f64>,

    /// Warn when a parsed layer's total polygon area exceeds this value.
    #[arg(long)]
    /// Field `max_layer_area`.
    pub max_layer_area: Option<f64>,

    /// Maximum allowed age for generated-date filename tags before manifest freshness warnings.
    #[arg(long = "generated-date-stale-days")]
    /// Field `generated_date_stale_days`.
    pub generated_date_stale_days: Option<usize>,

    /// Output format.
    #[arg(long, value_enum, default_value_t = OutputFormat::Text)]
    /// Field `format`.
    pub format: OutputFormat,

    /// Write a compact JSON CI summary to this path.
    #[arg(long = "summary-file")]
    /// Field `summary_file`.
    pub summary_file: Option<PathBuf>,

    /// Return exit code 0 even when active findings remain.
    #[arg(long = "allow-findings")]
    /// Field `allow_findings`.
    pub allow_findings: bool,

    /// Declared total copper layer count from order metadata.
    #[arg(long = "declared-copper-layer-count")]
    /// Field `declared_copper_layer_count`.
    pub declared_copper_layer_count: Option<usize>,

    /// Write an SVG overlay of active violations to this path.
    #[arg(long = "svg-overlay")]
    /// Field `svg_overlay`.
    pub svg_overlay: Option<PathBuf>,

    /// Write proposed waiver stubs for active findings to this JSON path.
    #[arg(long = "waiver-stubs")]
    /// Field `waiver_stubs`.
    pub waiver_stubs: Option<PathBuf>,

    /// Write an active-finding baseline to this JSON path.
    #[arg(long = "baseline-file")]
    /// Field `baseline_file`.
    pub baseline_file: Option<PathBuf>,

    /// Existing active-finding baseline used to classify new, resolved, and unchanged findings.
    #[arg(long = "baseline-reference")]
    /// Field `baseline_reference`.
    pub baseline_reference: Option<PathBuf>,

    /// Write baseline comparison results to this JSON path.
    #[arg(long = "baseline-diff-file")]
    /// Field `baseline_diff_file`.
    pub baseline_diff_file: Option<PathBuf>,
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use clap::Parser;

    use super::{Check, Cli, ConversionBackend, OutputFormat, SourceEda};

    #[test]
    fn parses_multiple_checks_and_inputs() {
        let cli = Cli::parse_from([
            "hyperdrc",
            "--allow-findings",
            "--check",
            "copper-overlap",
            "--check",
            "acid-trap",
            "--check",
            "acid-trap-trace-junction",
            "--check",
            "drill-spacing",
            "--check",
            "board-outline-drill-clearance",
            "--check",
            "drill-aspect-ratio",
            "--check",
            "annular-ring-tolerance",
            "--check",
            "plating-intent",
            "--check",
            "routed-slot-readiness",
            "--check",
            "castellation-intent",
            "--check",
            "castellation-hole-readiness",
            "--check",
            "via-in-pad-readiness",
            "--check",
            "drill-to-copper-clearance",
            "--check",
            "drill-table-consistency",
            "--check",
            "copper-width-readiness",
            "--check",
            "copper-net-intent",
            "--check",
            "teardrop-readiness",
            "--check",
            "thermal-relief-readiness",
            "--check",
            "plane-clearance-readiness",
            "--check",
            "board-edge-exposure",
            "--check",
            "high-speed-edge-readiness",
            "--check",
            "edge-copper-pullback-readiness",
            "--check",
            "high-voltage-edge-readiness",
            "--check",
            "differential-pair-via-symmetry-readiness",
            "--check",
            "controlled-impedance-readiness",
            "--check",
            "differential-pair-readiness",
            "--check",
            "differential-pair-spacing-readiness",
            "--check",
            "differential-pair-width-readiness",
            "--check",
            "differential-pair-neckdown-readiness",
            "--check",
            "differential-pair-skew-readiness",
            "--check",
            "differential-pair-to-pair-spacing-readiness",
            "--check",
            "differential-pair-via-proximity-readiness",
            "--check",
            "differential-pair-via-return-readiness",
            "--check",
            "differential-pair-return-readiness",
            "--check",
            "edge-stitching-readiness",
            "--check",
            "reference-plane-readiness",
            "--check",
            "reference-plane-void-readiness",
            "--check",
            "split-plane-crossing-readiness",
            "--check",
            "return-path-proximity-readiness",
            "--check",
            "orphaned-zone-readiness",
            "--check",
            "same-net-island-readiness",
            "--check",
            "same-net-drill-break-readiness",
            "--check",
            "different-net-short-readiness",
            "--check",
            "return-path-readiness",
            "--check",
            "high-current-readiness",
            "--check",
            "power-via-array-readiness",
            "--check",
            "power-via-return-readiness",
            "--check",
            "thermal-via-readiness",
            "--check",
            "thermal-via-distribution-readiness",
            "--check",
            "power-plane-readiness",
            "--check",
            "high-current-neck-readiness",
            "--check",
            "power-pad-entry-readiness",
            "--check",
            "voltage-clearance-readiness",
            "--check",
            "protective-earth-spacing-readiness",
            "--check",
            "surge-protection-keepout-readiness",
            "--check",
            "sensitive-net-spacing-readiness",
            "--check",
            "sensitive-return-readiness",
            "--check",
            "mixed-signal-partition-readiness",
            "--check",
            "rf-keepout-readiness",
            "--check",
            "antenna-copper-keepout-readiness",
            "--check",
            "rf-via-fence-readiness",
            "--check",
            "chassis-stitching-readiness",
            "--check",
            "gold-finger-readiness",
            "--check",
            "gold-finger-edge-readiness",
            "--check",
            "gold-finger-spacing-readiness",
            "--check",
            "gold-finger-drill-keepout-readiness",
            "--check",
            "component-edge-clearance-readiness",
            "--check",
            "component-hole-clearance-readiness",
            "--check",
            "component-spacing-readiness",
            "--check",
            "connector-rework-clearance-readiness",
            "--check",
            "pad-pair-asymmetry-readiness",
            "--check",
            "connector-return-path-readiness",
            "--check",
            "decoupling-proximity-readiness",
            "--check",
            "esd-protection-readiness",
            "--check",
            "esd-return-path-readiness",
            "--check",
            "switch-node-keepout-readiness",
            "--check",
            "inductor-copper-keepout-readiness",
            "--check",
            "testpoint-coverage-readiness",
            "--check",
            "testpoint-accessibility-readiness",
            "--check",
            "testpoint-copper-clearance-readiness",
            "--check",
            "tooling-hole-readiness",
            "--check",
            "mouse-bite-readiness",
            "--check",
            "fiducial-readiness",
            "--check",
            "local-fiducial-readiness",
            "--check",
            "fiducial-keepout-readiness",
            "--check",
            "dense-pad-escape-readiness",
            "--check",
            "dense-pad-via-spacing-readiness",
            "--check",
            "dense-pad-mask-bridge-readiness",
            "--check",
            "selective-wave-solder-keepout-readiness",
            "--check",
            "press-fit-keepout-readiness",
            "--check",
            "conformal-coating-keepout-readiness",
            "--check",
            "thermal-pad-via-readiness",
            "--check",
            "thermal-copper-area-readiness",
            "--check",
            "hot-component-spacing-readiness",
            "--check",
            "thermal-mechanical-keepout-readiness",
            "--check",
            "mounting-hole-grounding-readiness",
            "--check",
            "mounting-hole-copper-keepout-readiness",
            "--check",
            "mounting-hole-edge-clearance-readiness",
            "--check",
            "mounting-hole-plating-intent-readiness",
            "--check",
            "mounting-hole-distribution-readiness",
            "--check",
            "mounting-hole-spacing-readiness",
            "--check",
            "panel-feature-outline-readiness",
            "--check",
            "edge-plating-intent-readiness",
            "--check",
            "castellation-pitch-readiness",
            "--check",
            "different-net-spacing",
            "--check",
            "layer-registration-tolerance",
            "--check",
            "solder-mask-opening-coverage",
            "--check",
            "solder-mask-expansion",
            "--check",
            "solder-mask-overlap-clearance",
            "--check",
            "silkscreen-clearance",
            "--check",
            "silkscreen-board-edge-clearance",
            "--check",
            "solder-mask-board-edge-clearance",
            "--check",
            "copper-balance",
            "--check",
            "local-copper-density-readiness",
            "--check",
            "paste-aperture-coverage",
            "--check",
            "paste-aperture-ratio",
            "--check",
            "thermal-pad-paste-windowpane-readiness",
            "--check",
            "stencil-area-ratio-readiness",
            "--check",
            "paste-aperture-aspect-ratio-readiness",
            "--check",
            "tombstone-paste-imbalance-readiness",
            "--check",
            "paste-via-exposure-readiness",
            "--check",
            "minimum-paste-aperture",
            "--check",
            "paste-aperture-spacing",
            "--check",
            "paste-mask-alignment",
            "--check",
            "minimum-mask-opening",
            "--check",
            "solder-mask-opening-spacing",
            "--check",
            "ipc356-drill-diameter",
            "--format",
            "json",
            "--check",
            "stackup-readiness",
            "--check",
            "net-constraint-readiness",
            "--check",
            "file-manifest-readiness",
            "--check",
            "production-artifact-readiness",
            "--check",
            "mechanical-layer-geometry",
            "--check",
            "board-outline-sanity",
            "--check",
            "board-outline-self-intersection-readiness",
            "--check",
            "board-outline-notch-readiness",
            "--check",
            "board-outline-duplicate-readiness",
            "--check",
            "board-outline-nesting-readiness",
            "--check",
            "board-outline-cutout-clearance",
            "--check",
            "board-outline-fragments",
            "--check",
            "waiver-governance",
            "--silk-layer",
            "1",
            "top.gbr",
            "bottom.gbr",
        ]);

        assert_eq!(
            cli.checks,
            vec![
                Check::CopperOverlap,
                Check::AcidTrap,
                Check::AcidTrapTraceJunction,
                Check::DrillSpacing,
                Check::BoardOutlineDrillClearance,
                Check::DrillAspectRatio,
                Check::AnnularRingTolerance,
                Check::PlatingIntent,
                Check::RoutedSlotReadiness,
                Check::CastellationIntent,
                Check::CastellationHoleReadiness,
                Check::ViaInPadReadiness,
                Check::DrillToCopperClearance,
                Check::DrillTableConsistency,
                Check::CopperWidthReadiness,
                Check::CopperNetIntent,
                Check::TeardropReadiness,
                Check::ThermalReliefReadiness,
                Check::PlaneClearanceReadiness,
                Check::BoardEdgeExposure,
                Check::HighSpeedEdgeReadiness,
                Check::EdgeCopperPullbackReadiness,
                Check::HighVoltageEdgeReadiness,
                Check::DifferentialPairViaSymmetryReadiness,
                Check::ControlledImpedanceReadiness,
                Check::DifferentialPairReadiness,
                Check::DifferentialPairSpacingReadiness,
                Check::DifferentialPairWidthReadiness,
                Check::DifferentialPairNeckdownReadiness,
                Check::DifferentialPairSkewReadiness,
                Check::DifferentialPairToPairSpacingReadiness,
                Check::DifferentialPairViaProximityReadiness,
                Check::DifferentialPairViaReturnReadiness,
                Check::DifferentialPairReturnReadiness,
                Check::EdgeStitchingReadiness,
                Check::ReferencePlaneReadiness,
                Check::ReferencePlaneVoidReadiness,
                Check::SplitPlaneCrossingReadiness,
                Check::ReturnPathProximityReadiness,
                Check::OrphanedZoneReadiness,
                Check::SameNetIslandReadiness,
                Check::SameNetDrillBreakReadiness,
                Check::DifferentNetShortReadiness,
                Check::ReturnPathReadiness,
                Check::HighCurrentReadiness,
                Check::PowerViaArrayReadiness,
                Check::PowerViaReturnReadiness,
                Check::ThermalViaReadiness,
                Check::ThermalViaDistributionReadiness,
                Check::PowerPlaneReadiness,
                Check::HighCurrentNeckReadiness,
                Check::PowerPadEntryReadiness,
                Check::VoltageClearanceReadiness,
                Check::ProtectiveEarthSpacingReadiness,
                Check::SurgeProtectionKeepoutReadiness,
                Check::SensitiveNetSpacingReadiness,
                Check::SensitiveReturnReadiness,
                Check::MixedSignalPartitionReadiness,
                Check::RfKeepoutReadiness,
                Check::AntennaCopperKeepoutReadiness,
                Check::RfViaFenceReadiness,
                Check::ChassisStitchingReadiness,
                Check::GoldFingerReadiness,
                Check::GoldFingerEdgeReadiness,
                Check::GoldFingerSpacingReadiness,
                Check::GoldFingerDrillKeepoutReadiness,
                Check::ComponentEdgeClearanceReadiness,
                Check::ComponentHoleClearanceReadiness,
                Check::ComponentSpacingReadiness,
                Check::ConnectorReworkClearanceReadiness,
                Check::PadPairAsymmetryReadiness,
                Check::ConnectorReturnPathReadiness,
                Check::DecouplingProximityReadiness,
                Check::EsdProtectionReadiness,
                Check::EsdReturnPathReadiness,
                Check::SwitchNodeKeepoutReadiness,
                Check::InductorCopperKeepoutReadiness,
                Check::TestpointCoverageReadiness,
                Check::TestpointAccessibilityReadiness,
                Check::TestpointCopperClearanceReadiness,
                Check::ToolingHoleReadiness,
                Check::MouseBiteReadiness,
                Check::FiducialReadiness,
                Check::LocalFiducialReadiness,
                Check::FiducialKeepoutReadiness,
                Check::DensePadEscapeReadiness,
                Check::DensePadViaSpacingReadiness,
                Check::DensePadMaskBridgeReadiness,
                Check::SelectiveWaveSolderKeepoutReadiness,
                Check::PressFitKeepoutReadiness,
                Check::ConformalCoatingKeepoutReadiness,
                Check::ThermalPadViaReadiness,
                Check::ThermalCopperAreaReadiness,
                Check::HotComponentSpacingReadiness,
                Check::ThermalMechanicalKeepoutReadiness,
                Check::MountingHoleGroundingReadiness,
                Check::MountingHoleCopperKeepoutReadiness,
                Check::MountingHoleEdgeClearanceReadiness,
                Check::MountingHolePlatingIntentReadiness,
                Check::MountingHoleDistributionReadiness,
                Check::MountingHoleSpacingReadiness,
                Check::PanelFeatureOutlineReadiness,
                Check::EdgePlatingIntentReadiness,
                Check::CastellationPitchReadiness,
                Check::DifferentNetSpacing,
                Check::LayerRegistrationTolerance,
                Check::SolderMaskOpeningCoverage,
                Check::SolderMaskExpansion,
                Check::SolderMaskOverlapClearance,
                Check::SilkscreenClearance,
                Check::SilkscreenBoardEdgeClearance,
                Check::SolderMaskBoardEdgeClearance,
                Check::CopperBalance,
                Check::LocalCopperDensityReadiness,
                Check::PasteApertureCoverage,
                Check::PasteApertureRatio,
                Check::ThermalPadPasteWindowpaneReadiness,
                Check::StencilAreaRatioReadiness,
                Check::PasteApertureAspectRatioReadiness,
                Check::TombstonePasteImbalanceReadiness,
                Check::PasteViaExposureReadiness,
                Check::MinimumPasteAperture,
                Check::PasteApertureSpacing,
                Check::PasteMaskAlignment,
                Check::MinimumMaskOpening,
                Check::SolderMaskOpeningSpacing,
                Check::Ipc356DrillDiameter,
                Check::StackupReadiness,
                Check::NetConstraintReadiness,
                Check::FileManifestReadiness,
                Check::ProductionArtifactReadiness,
                Check::MechanicalLayerGeometry,
                Check::BoardOutlineSanity,
                Check::BoardOutlineSelfIntersectionReadiness,
                Check::BoardOutlineNotchReadiness,
                Check::BoardOutlineDuplicateReadiness,
                Check::BoardOutlineNestingReadiness,
                Check::BoardOutlineCutoutClearance,
                Check::BoardOutlineFragments,
                Check::WaiverGovernance
            ]
        );
        assert_eq!(cli.silk_layers, vec![1]);
        assert_eq!(cli.format, OutputFormat::Json);
        assert!(cli.allow_findings);
        assert_eq!(cli.files.len(), 2);
    }

    #[test]
    fn parses_gerber_directories_and_conversion_options() {
        let cli = Cli::parse_from([
            "hyperdrc",
            "--gerber-dir",
            "gerbers",
            "--convert-input",
            "incoming",
            "--conversion-output-dir",
            "converted",
            "--source-eda",
            "kicad",
            "--transjlc-bin",
            "transjlc",
            "--conversion-zip",
            "--conversion-zip-name",
            "upload",
            "--conversion-arg=--foo",
            "--conversion-arg=--bar=baz",
        ]);

        assert_eq!(cli.gerber_dirs, vec![PathBuf::from("gerbers")]);
        assert_eq!(cli.conversion_inputs, vec![PathBuf::from("incoming")]);
        assert_eq!(cli.conversion_output_dir, PathBuf::from("converted"));
        assert_eq!(cli.converter, ConversionBackend::Transjlc);
        assert_eq!(cli.source_eda, SourceEda::Kicad);
        assert_eq!(cli.transjlc_bin, PathBuf::from("transjlc"));
        assert!(cli.conversion_zip);
        assert_eq!(cli.conversion_zip_name, "upload");
        assert_eq!(cli.conversion_args, vec!["--foo", "--bar=baz"]);
    }

    #[test]
    fn parses_manufacturing_readiness_sources() {
        let cli = Cli::parse_from([
            "hyperdrc",
            "--bom",
            "parts.json",
            "--centroid",
            "centroid.txt",
            "--netlist",
            "netlist.csv",
            "--fab-drawing",
            "fab.pdf",
            "--assembly-drawing",
            "assembly.dxf",
            "--readme",
            "README.md",
            "--rout-drawing",
            "panel.dxf",
            "--waiver-stubs",
            "waiver-stubs.json",
            "--baseline-file",
            "baseline.json",
            "--baseline-reference",
            "previous-baseline.json",
            "--baseline-diff-file",
            "baseline-diff.json",
            "--generated-date-stale-days",
            "14",
            "--declared-copper-layer-count",
            "4",
            "top.gbr",
        ]);

        assert_eq!(cli.bom_files, vec![PathBuf::from("parts.json")]);
        assert_eq!(cli.centroid_files, vec![PathBuf::from("centroid.txt")]);
        assert_eq!(cli.netlist_files, vec![PathBuf::from("netlist.csv")]);
        assert_eq!(cli.fab_drawing_files, vec![PathBuf::from("fab.pdf")]);
        assert_eq!(
            cli.assembly_drawing_files,
            vec![PathBuf::from("assembly.dxf")]
        );
        assert_eq!(cli.readme_files, vec![PathBuf::from("README.md")]);
        assert_eq!(cli.rout_drawing_files, vec![PathBuf::from("panel.dxf")]);
        assert_eq!(cli.waiver_stubs, Some(PathBuf::from("waiver-stubs.json")));
        assert_eq!(cli.baseline_file, Some(PathBuf::from("baseline.json")));
        assert_eq!(
            cli.baseline_reference,
            Some(PathBuf::from("previous-baseline.json"))
        );
        assert_eq!(
            cli.baseline_diff_file,
            Some(PathBuf::from("baseline-diff.json"))
        );
        assert_eq!(cli.generated_date_stale_days, Some(14));
        assert_eq!(cli.declared_copper_layer_count, Some(4));
    }

    #[test]
    fn parses_sarif_output_format() {
        let cli = Cli::parse_from(["hyperdrc", "--format", "sarif", "top.gbr"]);

        assert_eq!(cli.format, OutputFormat::Sarif);
    }

    #[test]
    fn parses_streaming_and_ci_output_formats() {
        let jsonl = Cli::parse_from(["hyperdrc", "--format", "jsonl", "top.gbr"]);
        let github = Cli::parse_from(["hyperdrc", "--format", "github-annotations", "top.gbr"]);
        let html = Cli::parse_from(["hyperdrc", "--format", "html", "top.gbr"]);
        let junit = Cli::parse_from(["hyperdrc", "--format", "junit", "top.gbr"]);

        assert_eq!(jsonl.format, OutputFormat::Jsonl);
        assert_eq!(github.format, OutputFormat::GithubAnnotations);
        assert_eq!(html.format, OutputFormat::Html);
        assert_eq!(junit.format, OutputFormat::Junit);
    }

    #[test]
    fn rejects_unknown_check_name() {
        let result = Cli::try_parse_from(["hyperdrc", "--check", "not-a-check", "top.gbr"]);

        assert!(result.is_err());
    }

    #[test]
    fn parses_legacy_check_names_for_plan_named_checks() {
        let cli = Cli::parse_from([
            "hyperdrc",
            "--check",
            "drill-copper-clearance",
            "--check",
            "net-spacing",
            "--check",
            "registration-tolerance",
            "top.gbr",
        ]);

        assert_eq!(
            cli.checks,
            vec![
                Check::DrillCopperClearance,
                Check::NetSpacing,
                Check::RegistrationTolerance
            ]
        );
    }
}
