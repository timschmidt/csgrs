use std::path::PathBuf;

use clap::{Parser, ValueEnum};

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum Check {
    MaskIslandKeepout,
    CopperOverlap,
    BoardEdgeClearance,
    BoardOutlineSanity,
    PasteOverhang,
    PasteApertureCoverage,
    ExposedCopper,
    SolderMaskOpeningCoverage,
    SilkscreenOverlap,
    SilkscreenMinWidth,
    MinCopperNeck,
    AcidTrap,
    LayerSanity,
    MechanicalLayerGeometry,
    SolderMaskSliver,
    AnnularRing,
    DrillCopperClearance,
    DrillSpacing,
    NetSpacing,
    RegistrationTolerance,
    PanelizationClearance,
    Ipc356Coverage,
    Ipc356DrillDiameter,
}

pub const DEFAULT_CHECKS: &[Check] = &[
    Check::MaskIslandKeepout,
    Check::CopperOverlap,
    Check::BoardEdgeClearance,
    Check::BoardOutlineSanity,
    Check::PasteOverhang,
    Check::PasteApertureCoverage,
    Check::ExposedCopper,
    Check::SolderMaskOpeningCoverage,
    Check::SilkscreenOverlap,
    Check::SilkscreenMinWidth,
    Check::MinCopperNeck,
    Check::AcidTrap,
    Check::LayerSanity,
    Check::MechanicalLayerGeometry,
    Check::SolderMaskSliver,
    Check::AnnularRing,
    Check::DrillCopperClearance,
    Check::DrillSpacing,
    Check::NetSpacing,
    Check::RegistrationTolerance,
    Check::PanelizationClearance,
    Check::Ipc356Coverage,
    Check::Ipc356DrillDiameter,
];

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum OutputFormat {
    Text,
    Json,
    Geojson,
}

#[derive(Parser, Debug)]
#[command(author, version, about)]
pub struct Cli {
    /// JSON rule configuration file.
    #[arg(long = "config")]
    pub config: Option<PathBuf>,

    /// Gerber files to load as separate layers.
    pub files: Vec<PathBuf>,

    /// KiCad .kicad_pcb file. Repeat to check multiple boards.
    #[arg(long = "kicad-pcb")]
    pub kicad_pcbs: Vec<PathBuf>,

    /// Excellon drill file. Repeat for plated, non-plated, or panel drill files.
    #[arg(long = "excellon")]
    pub excellon_files: Vec<PathBuf>,

    /// IPC-D-356 netlist file. Repeat to merge multiple electrical test netlists.
    #[arg(long = "ipc356")]
    pub ipc356_files: Vec<PathBuf>,

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

    /// Minimum copper width used by the neck-width morphology check.
    #[arg(long)]
    pub min_width: Option<f64>,

    /// Minimum solder mask web width used by the sliver morphology check.
    #[arg(long)]
    pub min_mask_width: Option<f64>,

    /// Maximum interior angle to report as an acid-trap candidate.
    #[arg(long)]
    pub acid_trap_angle: Option<f64>,

    /// Minimum acceptable annular ring around plated drills, in KiCad units.
    #[arg(long)]
    pub annular_ring: Option<f64>,

    /// Drill-to-copper clearance, in KiCad or Excellon units.
    #[arg(long)]
    pub drill_clearance: Option<f64>,

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

    /// Write an SVG overlay of active violations to this path.
    #[arg(long = "svg-overlay")]
    pub svg_overlay: Option<PathBuf>,
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::{Check, Cli, OutputFormat};

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
            "solder-mask-opening-coverage",
            "--check",
            "paste-aperture-coverage",
            "--check",
            "ipc356-drill-diameter",
            "--format",
            "json",
            "--check",
            "mechanical-layer-geometry",
            "--check",
            "board-outline-sanity",
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
                Check::SolderMaskOpeningCoverage,
                Check::PasteApertureCoverage,
                Check::Ipc356DrillDiameter,
                Check::MechanicalLayerGeometry,
                Check::BoardOutlineSanity
            ]
        );
        assert_eq!(cli.silk_layers, vec![1]);
        assert_eq!(cli.format, OutputFormat::Json);
        assert_eq!(cli.files.len(), 2);
    }

    #[test]
    fn rejects_unknown_check_name() {
        let result = Cli::try_parse_from(["hyperdrc", "--check", "not-a-check", "top.gbr"]);

        assert!(result.is_err());
    }
}
