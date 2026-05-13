use std::path::PathBuf;

use clap::{Parser, ValueEnum};

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum Check {
    MaskIslandKeepout,
    CopperOverlap,
    BoardEdgeClearance,
    BoardOutlineCutoutClearance,
    BoardOutlineSanity,
    BoardOutlineFragments,
    BoardOutlineSelfIntersectionReadiness,
    BoardOutlineNotchReadiness,
    BoardOutlineDuplicateReadiness,
    BoardOutlineNestingReadiness,
    PasteOverhang,
    PasteApertureCoverage,
    PasteApertureRatio,
    MinimumPasteAperture,
    PasteApertureSpacing,
    PasteMaskAlignment,
    ExposedCopper,
    SolderMaskOpeningCoverage,
    SolderMaskExpansion,
    SolderMaskOverlapClearance,
    SolderMaskBoardEdgeClearance,
    SilkscreenOverlap,
    SilkscreenClearance,
    SilkscreenBoardEdgeClearance,
    SilkscreenMinWidth,
    MinCopperNeck,
    AcidTrap,
    LayerSanity,
    CopperBalance,
    MechanicalLayerGeometry,
    SolderMaskSliver,
    MinimumMaskOpening,
    SolderMaskOpeningSpacing,
    AnnularRing,
    AnnularRingTolerance,
    PlatingIntent,
    RoutedSlotReadiness,
    CastellationIntent,
    CastellationHoleReadiness,
    ViaInPadReadiness,
    DrillCopperClearance,
    BoardOutlineDrillClearance,
    DrillSpacing,
    DrillAspectRatio,
    DrillTableConsistency,
    CopperWidthReadiness,
    CopperNetIntent,
    TeardropReadiness,
    ThermalReliefReadiness,
    PlaneClearanceReadiness,
    BoardEdgeExposure,
    HighSpeedEdgeReadiness,
    EdgeCopperPullbackReadiness,
    HighVoltageEdgeReadiness,
    ControlledImpedanceReadiness,
    DifferentialPairReadiness,
    DifferentialPairSpacingReadiness,
    DifferentialPairViaSymmetryReadiness,
    ReferencePlaneReadiness,
    ReferencePlaneVoidReadiness,
    OrphanedZoneReadiness,
    SameNetIslandReadiness,
    ReturnPathReadiness,
    HighCurrentReadiness,
    PowerViaArrayReadiness,
    ThermalViaReadiness,
    PowerPlaneReadiness,
    HighCurrentNeckReadiness,
    VoltageClearanceReadiness,
    SensitiveNetSpacingReadiness,
    SensitiveReturnReadiness,
    RfKeepoutReadiness,
    ChassisStitchingReadiness,
    EdgeStitchingReadiness,
    GoldFingerReadiness,
    TestpointCoverageReadiness,
    FiducialReadiness,
    DensePadEscapeReadiness,
    NetSpacing,
    RegistrationTolerance,
    PanelizationClearance,
    Ipc356Coverage,
    Ipc356DrillDiameter,
    ExcellonReadiness,
    FileManifestReadiness,
    ProductionArtifactReadiness,
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
    Check::LayerSanity,
    Check::CopperBalance,
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
    Check::DrillCopperClearance,
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
    Check::DifferentialPairViaSymmetryReadiness,
    Check::ReferencePlaneReadiness,
    Check::ReferencePlaneVoidReadiness,
    Check::OrphanedZoneReadiness,
    Check::SameNetIslandReadiness,
    Check::ReturnPathReadiness,
    Check::HighCurrentReadiness,
    Check::PowerViaArrayReadiness,
    Check::ThermalViaReadiness,
    Check::PowerPlaneReadiness,
    Check::HighCurrentNeckReadiness,
    Check::VoltageClearanceReadiness,
    Check::SensitiveNetSpacingReadiness,
    Check::SensitiveReturnReadiness,
    Check::RfKeepoutReadiness,
    Check::ChassisStitchingReadiness,
    Check::EdgeStitchingReadiness,
    Check::GoldFingerReadiness,
    Check::TestpointCoverageReadiness,
    Check::FiducialReadiness,
    Check::DensePadEscapeReadiness,
    Check::NetSpacing,
    Check::RegistrationTolerance,
    Check::PanelizationClearance,
    Check::Ipc356Coverage,
    Check::Ipc356DrillDiameter,
    Check::ExcellonReadiness,
    Check::FileManifestReadiness,
    Check::ProductionArtifactReadiness,
];

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum OutputFormat {
    Text,
    Json,
    Geojson,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum ConversionBackend {
    Transjlc,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum SourceEda {
    Auto,
    Kicad,
    Jlc,
    Protel,
}

impl SourceEda {
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
pub struct Cli {
    /// JSON rule configuration file.
    #[arg(long = "config")]
    pub config: Option<PathBuf>,

    /// Gerber files to load as separate layers.
    pub files: Vec<PathBuf>,

    /// Directory containing Gerber files to load as layers. Repeat to merge multiple packages.
    #[arg(long = "gerber-dir")]
    pub gerber_dirs: Vec<PathBuf>,

    /// Gerber directory to convert before loading. Repeat for multiple input packages.
    #[arg(long = "convert-input")]
    pub conversion_inputs: Vec<PathBuf>,

    /// Converter backend for --convert-input packages.
    #[arg(long = "converter", value_enum, default_value_t = ConversionBackend::Transjlc)]
    pub converter: ConversionBackend,

    /// Base directory for converted Gerber output.
    #[arg(long = "conversion-output-dir", default_value = "hyperdrc-converted")]
    pub conversion_output_dir: PathBuf,

    /// Source EDA passed to the converter.
    #[arg(long = "source-eda", value_enum, default_value_t = SourceEda::Auto)]
    pub source_eda: SourceEda,

    /// Path to the TransJLC executable used by --converter transjlc.
    #[arg(long = "transjlc-bin", default_value = "TransJLC")]
    pub transjlc_bin: PathBuf,

    /// Ask the converter to create a zip archive when supported.
    #[arg(long = "conversion-zip")]
    pub conversion_zip: bool,

    /// Zip file base name passed to converters that support zipped output.
    #[arg(long = "conversion-zip-name", default_value = "Gerber")]
    pub conversion_zip_name: String,

    /// Optional top colorful silkscreen image passed through to TransJLC.
    #[arg(long = "top-color-image")]
    pub top_color_image: Option<PathBuf>,

    /// Optional bottom colorful silkscreen image passed through to TransJLC.
    #[arg(long = "bottom-color-image")]
    pub bottom_color_image: Option<PathBuf>,

    /// Extra command-line arguments passed to the selected converter backend.
    #[arg(long = "conversion-arg", value_name = "ARG")]
    pub conversion_args: Vec<String>,

    /// KiCad .kicad_pcb file. Repeat to check multiple boards.
    #[arg(long = "kicad-pcb")]
    pub kicad_pcbs: Vec<PathBuf>,

    /// Excellon drill file. Repeat for plated, non-plated, or panel drill files.
    #[arg(long = "excellon")]
    pub excellon_files: Vec<PathBuf>,

    /// IPC-D-356 netlist file. Repeat to merge multiple electrical test netlists.
    #[arg(long = "ipc356")]
    pub ipc356_files: Vec<PathBuf>,

    /// Bill of materials file.
    #[arg(long = "bom")]
    pub bom_files: Vec<PathBuf>,

    /// Placement / centroid file.
    #[arg(long = "centroid")]
    pub centroid_files: Vec<PathBuf>,

    /// Netlist source for pre-production validation and manifest completeness.
    #[arg(long = "netlist")]
    pub netlist_files: Vec<PathBuf>,

    /// Mechanical fabricator drawing file.
    #[arg(long = "fab-drawing")]
    pub fab_drawing_files: Vec<PathBuf>,

    /// Assembly drawing, instruction, or fixture file.
    #[arg(long = "assembly-drawing")]
    pub assembly_drawing_files: Vec<PathBuf>,

    /// Readme or release-notes file describing the package.
    #[arg(long = "readme")]
    pub readme_files: Vec<PathBuf>,

    /// Route, V-score, or tooling drawing for panelization review.
    #[arg(long = "rout-drawing")]
    pub rout_drawing_files: Vec<PathBuf>,

    /// JSON waiver file. Repeat to combine waiver sets.
    #[arg(long = "waiver")]
    pub waiver_files: Vec<PathBuf>,

    /// Check(s) to run. Repeat the flag to run a sequence.
    #[arg(short = 'c', long = "check", value_enum)]
    pub checks: Vec<Check>,

    /// Keepout distance for mask island isolation checks, in Gerber units.
    #[arg(long)]
    pub keepout: Option<f64>,

    /// Board outline layer index for board-edge clearance checks.
    #[arg(long)]
    pub board_outline: Option<usize>,

    /// Copper layer index. Repeat to restrict copper-related checks.
    #[arg(long = "copper-layer")]
    pub copper_layers: Vec<usize>,

    /// KiCad copper layer name. Repeat to restrict KiCad copper checks, for example F.Cu.
    #[arg(long = "kicad-copper-layer")]
    pub kicad_copper_layers: Vec<String>,

    /// Layer pairs for copper overlap checks, written as zero-based indexes like 0:1.
    /// If omitted, all unique file pairs are checked.
    #[arg(long = "pair")]
    pub pairs: Vec<String>,

    /// Paste-to-copper layer pairs for paste overhang checks, written as PASTE:COPPER.
    #[arg(long = "paste-pair")]
    pub paste_pairs: Vec<String>,

    /// Copper-to-mask-opening layer pairs for exposed copper checks, written as COPPER:MASK.
    #[arg(long = "mask-pair")]
    pub mask_pairs: Vec<String>,

    /// Solder mask layer index. Repeat to run mask sliver checks on Gerber layers.
    #[arg(long = "mask-layer")]
    pub mask_layers: Vec<usize>,

    /// Silkscreen-to-blocker layer pairs for silkscreen overlap checks, written as SILK:BLOCKER.
    #[arg(long = "silk-pair")]
    pub silk_pairs: Vec<String>,

    /// Silkscreen layer index. Repeat to run silkscreen width checks on Gerber layers.
    #[arg(long = "silk-layer")]
    pub silk_layers: Vec<usize>,

    /// Clearance distance for board-edge checks, in Gerber units.
    #[arg(long)]
    pub clearance: Option<f64>,

    /// Allowed paste overhang beyond copper, in Gerber units.
    #[arg(long)]
    pub paste_tolerance: Option<f64>,

    /// Minimum paste-to-copper area ratio for each paired copper island.
    #[arg(long)]
    pub min_paste_area_ratio: Option<f64>,

    /// Maximum paste-to-copper area ratio for each paired copper island.
    #[arg(long)]
    pub max_paste_area_ratio: Option<f64>,

    /// Minimum copper width used by the neck-width morphology check.
    #[arg(long)]
    pub min_width: Option<f64>,

    /// Minimum solder mask web width used by the sliver morphology check.
    #[arg(long)]
    pub min_mask_width: Option<f64>,

    /// Maximum interior angle to report as an acid-trap candidate.
    #[arg(long)]
    pub acid_trap_angle: Option<f64>,

    /// Warn when the largest selected copper layer area exceeds the smallest by this ratio.
    #[arg(long)]
    pub max_copper_imbalance_ratio: Option<f64>,

    /// Minimum acceptable annular ring around plated drills, in KiCad units.
    #[arg(long)]
    pub annular_ring: Option<f64>,

    /// Drill-to-copper clearance, in KiCad or Excellon units.
    #[arg(long)]
    pub drill_clearance: Option<f64>,

    /// Finished board thickness used by drill aspect-ratio readiness checks.
    #[arg(long)]
    pub board_thickness: Option<f64>,

    /// Maximum allowed board-thickness-to-drill-diameter ratio.
    #[arg(long)]
    pub max_drill_aspect_ratio: Option<f64>,

    /// Different-net copper spacing for KiCad net-aware checks.
    #[arg(long)]
    pub net_clearance: Option<f64>,

    /// Layer registration tolerance for cross-layer KiCad copper proximity checks.
    #[arg(long)]
    pub registration_tolerance: Option<f64>,

    /// Clearance from copper to panel features, NPTH drills, or Excellon panel drills.
    #[arg(long)]
    pub panel_clearance: Option<f64>,

    /// Coordinate tolerance for matching IPC-D-356 records to parsed board geometry.
    #[arg(long)]
    pub ipc356_tolerance: Option<f64>,

    /// Print detected KiCad copper layers to stderr before running checks.
    #[arg(long)]
    pub list_kicad_layers: bool,

    /// Ignore violation shapes whose area is at or below this threshold.
    #[arg(long)]
    pub min_area: Option<f64>,

    /// Warn when a parsed layer's total polygon area exceeds this value.
    #[arg(long)]
    pub max_layer_area: Option<f64>,

    /// Output format.
    #[arg(long, value_enum, default_value_t = OutputFormat::Text)]
    pub format: OutputFormat,

    /// Write a compact JSON CI summary to this path.
    #[arg(long = "summary-file")]
    pub summary_file: Option<PathBuf>,

    /// Declared total copper layer count from order metadata.
    #[arg(long = "declared-copper-layer-count")]
    pub declared_copper_layer_count: Option<usize>,

    /// Write an SVG overlay of active violations to this path.
    #[arg(long = "svg-overlay")]
    pub svg_overlay: Option<PathBuf>,
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
            "--check",
            "copper-overlap",
            "--check",
            "acid-trap",
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
            "edge-stitching-readiness",
            "--check",
            "reference-plane-readiness",
            "--check",
            "reference-plane-void-readiness",
            "--check",
            "orphaned-zone-readiness",
            "--check",
            "same-net-island-readiness",
            "--check",
            "return-path-readiness",
            "--check",
            "high-current-readiness",
            "--check",
            "power-via-array-readiness",
            "--check",
            "thermal-via-readiness",
            "--check",
            "power-plane-readiness",
            "--check",
            "high-current-neck-readiness",
            "--check",
            "voltage-clearance-readiness",
            "--check",
            "sensitive-net-spacing-readiness",
            "--check",
            "sensitive-return-readiness",
            "--check",
            "rf-keepout-readiness",
            "--check",
            "chassis-stitching-readiness",
            "--check",
            "gold-finger-readiness",
            "--check",
            "testpoint-coverage-readiness",
            "--check",
            "fiducial-readiness",
            "--check",
            "dense-pad-escape-readiness",
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
            "paste-aperture-coverage",
            "--check",
            "paste-aperture-ratio",
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
                Check::DrillSpacing,
                Check::BoardOutlineDrillClearance,
                Check::DrillAspectRatio,
                Check::AnnularRingTolerance,
                Check::PlatingIntent,
                Check::RoutedSlotReadiness,
                Check::CastellationIntent,
                Check::CastellationHoleReadiness,
                Check::ViaInPadReadiness,
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
                Check::EdgeStitchingReadiness,
                Check::ReferencePlaneReadiness,
                Check::ReferencePlaneVoidReadiness,
                Check::OrphanedZoneReadiness,
                Check::SameNetIslandReadiness,
                Check::ReturnPathReadiness,
                Check::HighCurrentReadiness,
                Check::PowerViaArrayReadiness,
                Check::ThermalViaReadiness,
                Check::PowerPlaneReadiness,
                Check::HighCurrentNeckReadiness,
                Check::VoltageClearanceReadiness,
                Check::SensitiveNetSpacingReadiness,
                Check::SensitiveReturnReadiness,
                Check::RfKeepoutReadiness,
                Check::ChassisStitchingReadiness,
                Check::GoldFingerReadiness,
                Check::TestpointCoverageReadiness,
                Check::FiducialReadiness,
                Check::DensePadEscapeReadiness,
                Check::SolderMaskOpeningCoverage,
                Check::SolderMaskExpansion,
                Check::SolderMaskOverlapClearance,
                Check::SilkscreenClearance,
                Check::SilkscreenBoardEdgeClearance,
                Check::SolderMaskBoardEdgeClearance,
                Check::CopperBalance,
                Check::PasteApertureCoverage,
                Check::PasteApertureRatio,
                Check::MinimumPasteAperture,
                Check::PasteApertureSpacing,
                Check::PasteMaskAlignment,
                Check::MinimumMaskOpening,
                Check::SolderMaskOpeningSpacing,
                Check::Ipc356DrillDiameter,
                Check::FileManifestReadiness,
                Check::ProductionArtifactReadiness,
                Check::MechanicalLayerGeometry,
                Check::BoardOutlineSanity,
                Check::BoardOutlineSelfIntersectionReadiness,
                Check::BoardOutlineNotchReadiness,
                Check::BoardOutlineDuplicateReadiness,
                Check::BoardOutlineNestingReadiness,
                Check::BoardOutlineCutoutClearance,
                Check::BoardOutlineFragments
            ]
        );
        assert_eq!(cli.silk_layers, vec![1]);
        assert_eq!(cli.format, OutputFormat::Json);
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
        assert_eq!(cli.declared_copper_layer_count, Some(4));
    }

    #[test]
    fn rejects_unknown_check_name() {
        let result = Cli::try_parse_from(["hyperdrc", "--check", "not-a-check", "top.gbr"]);

        assert!(result.is_err());
    }
}
