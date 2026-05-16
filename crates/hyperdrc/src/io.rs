//! Input/output discovery and provenance.
//!
//! The DRC engine should not need to know whether a layer came from a direct
//! Gerber file, a Gerber package directory, or a converter. This module keeps
//! that discovery logic and source metadata in one place so future adapters can
//! implement the same shape.
//!
//! Reliability note: file-role discovery is filename heuristic driven. Suspect
//! package manifests, duplicate roles, or missing layers should be verified
//! against the supplier's naming convention and fabrication README.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::Serialize;

/// Input adapter that supplied a source file.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "kebab-case")]
/// Public enumeration for `IoAdapter`.
pub enum IoAdapter {
    /// A file passed directly on the command line.
    DirectFile,
    /// A file discovered inside a Gerber/package directory.
    GerberDirectory,
    /// A file produced by a conversion backend.
    Conversion,
    /// A file extracted from a manufacturing package archive.
    PackageArchive,
    /// A KiCad board source file.
    KiCad,
    /// An Excellon drill source file.
    Excellon,
    /// An IPC-D-356 netlist source file.
    Ipc356,
    /// A JSON waiver file.
    Waiver,
}

/// Functional role of a source file.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "kebab-case")]
/// Public enumeration for `IoRole`.
pub enum IoRole {
    /// Gerber or Gerber-like layer geometry.
    GerberLayer,
    /// Native KiCad board input.
    KiCadBoard,
    /// Drill sidecar such as Excellon.
    DrillSidecar,
    /// Electrical test netlist sidecar such as IPC-D-356.
    NetlistSidecar,
    /// BOM-style netlist or connectivity export.
    NetlistFile,
    /// Routing, tab, or mouse-bite drawing.
    RoutDrawingFile,
    /// Manufacturing exchange or handoff package such as IPC-2581 or ODB++.
    ManufacturingHandoff,
    /// Bill of materials.
    BomFile,
    /// Component placement or centroid file.
    CentroidFile,
    /// Fabrication drawing.
    FabDrawing,
    /// Assembly drawing.
    AssemblyDrawing,
    /// Manufacturing package README or notes file.
    ReadmeFile,
    /// Waiver policy input.
    Waiver,
}

/// Provenance record serialized into reports.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
/// Public data model for `SourceRecord`.
pub struct SourceRecord {
    /// Adapter that supplied the file.
    pub adapter: IoAdapter,
    /// Role assigned to the source.
    pub role: IoRole,
    /// Display path for the source.
    pub path: String,
    /// Optional parent package, converter input, or discovery origin.
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Field `origin`.
    pub origin: Option<String>,
    /// Optional units declared by the source file before internal normalization.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_units: Option<String>,
    /// Optional normalized unit basis used by HyperDRC for distance reporting.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub normalized_units: Option<String>,
    /// Optional deterministic hash of the converter input file or tree.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_hash: Option<String>,
    /// Optional command history that transformed the origin into this source.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub transformation_history: Vec<String>,
}

impl SourceRecord {
    /// Build a source record from path-like values.
    pub fn new(
        adapter: IoAdapter,
        role: IoRole,
        path: impl AsRef<Path>,
        origin: Option<impl AsRef<Path>>,
    ) -> Self {
        Self {
            adapter,
            role,
            path: path.as_ref().display().to_string(),
            origin: origin.map(|path| path.as_ref().display().to_string()),
            source_units: None,
            normalized_units: None,
            source_hash: None,
            transformation_history: Vec::new(),
        }
    }

    /// Attach source and normalized unit context to a provenance record.
    pub fn with_unit_context(
        mut self,
        source_units: Option<String>,
        normalized_units: Option<String>,
    ) -> Self {
        self.source_units = source_units;
        self.normalized_units = normalized_units;
        self
    }

    /// Attach converter provenance that produced this source.
    pub fn with_transformation_context(
        mut self,
        source_hash: Option<String>,
        transformation_history: Vec<String>,
    ) -> Self {
        self.source_hash = source_hash;
        self.transformation_history = transformation_history;
        self
    }
}

/// File discovered with its source provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
/// Public data model for `DiscoveredFile`.
pub struct DiscoveredFile {
    /// Filesystem path to the discovered source.
    pub path: PathBuf,
    /// Provenance record for reports and manifests.
    pub source: SourceRecord,
}

/// Sidecar files discovered next to a Gerber package.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
/// Public data model for `PackageSidecars`.
pub struct PackageSidecars {
    /// Excellon drill files.
    pub excellon_files: Vec<DiscoveredFile>,
    /// IPC-D-356 netlist files.
    pub ipc356_files: Vec<DiscoveredFile>,
    /// Bills of materials.
    pub bom_files: Vec<DiscoveredFile>,
    /// Component placement files.
    pub centroid_files: Vec<DiscoveredFile>,
    /// General netlist files.
    pub netlist_files: Vec<DiscoveredFile>,
    /// Fabrication drawings.
    pub fab_drawing_files: Vec<DiscoveredFile>,
    /// Assembly drawings.
    pub assembly_drawing_files: Vec<DiscoveredFile>,
    /// Package README or manufacturing notes.
    pub readme_files: Vec<DiscoveredFile>,
    /// Routing/tab drawings.
    pub rout_drawing_files: Vec<DiscoveredFile>,
    /// Manufacturing exchange or handoff files.
    pub manufacturing_handoff_files: Vec<DiscoveredFile>,
}

/// Wrap a direct Gerber file path as a discovered input.
pub fn direct_gerber_file(path: PathBuf) -> DiscoveredFile {
    DiscoveredFile {
        source: SourceRecord::new(
            IoAdapter::DirectFile,
            IoRole::GerberLayer,
            &path,
            Option::<&Path>::None,
        ),
        path,
    }
}

/// Wrap a converter-produced Gerber file with its original source.
pub fn converted_gerber_file(path: PathBuf, origin: &Path) -> DiscoveredFile {
    DiscoveredFile {
        source: SourceRecord::new(
            IoAdapter::Conversion,
            IoRole::GerberLayer,
            &path,
            Some(origin),
        ),
        path,
    }
}

/// Discover Gerber-like layer files in a directory.
pub fn discover_gerber_dir(directory: &Path) -> Result<Vec<DiscoveredFile>> {
    let mut files = Vec::new();
    for entry in std::fs::read_dir(directory)
        .with_context(|| format!("failed to read Gerber directory {}", directory.display()))?
    {
        let entry =
            entry.with_context(|| format!("failed to read entry in {}", directory.display()))?;
        let path = entry.path();
        if path.is_file() && is_gerber_path(&path) {
            files.push(DiscoveredFile {
                source: SourceRecord::new(
                    IoAdapter::GerberDirectory,
                    IoRole::GerberLayer,
                    &path,
                    Some(directory),
                ),
                path,
            });
        }
    }
    files.sort_by(|left, right| left.path.cmp(&right.path));
    Ok(files)
}

/// Recursively discover Gerber-like layer files in an extracted package tree.
pub fn discover_gerber_tree_from_archive(
    directory: &Path,
    archive: &Path,
) -> Result<Vec<DiscoveredFile>> {
    let mut files = Vec::new();
    discover_gerber_tree_inner(directory, archive, &mut files)?;
    files.sort_by(|left, right| left.path.cmp(&right.path));
    Ok(files)
}

fn discover_gerber_tree_inner(
    directory: &Path,
    archive: &Path,
    files: &mut Vec<DiscoveredFile>,
) -> Result<()> {
    for entry in std::fs::read_dir(directory).with_context(|| {
        format!(
            "failed to read extracted package directory {}",
            directory.display()
        )
    })? {
        let entry =
            entry.with_context(|| format!("failed to read entry in {}", directory.display()))?;
        let path = entry.path();
        if path.is_dir() {
            discover_gerber_tree_inner(&path, archive, files)?;
        } else if path.is_file() && is_gerber_path(&path) {
            files.push(DiscoveredFile {
                source: SourceRecord::new(
                    IoAdapter::PackageArchive,
                    IoRole::GerberLayer,
                    &path,
                    Some(archive),
                ),
                path,
            });
        }
    }
    Ok(())
}

/// Discover manufacturing package sidecars in the provided directories.
pub fn discover_package_sidecars(directories: &[PathBuf]) -> Result<PackageSidecars> {
    let mut sidecars = PackageSidecars::default();
    for directory in directories {
        for entry in std::fs::read_dir(directory).with_context(|| {
            format!(
                "failed to read package sidecar directory {}",
                directory.display()
            )
        })? {
            let entry = entry
                .with_context(|| format!("failed to read entry in {}", directory.display()))?;
            let path = entry.path();
            if !path.is_file() {
                continue;
            }

            let Some(role) = classify_package_sidecar(&path) else {
                continue;
            };
            let discovered = DiscoveredFile {
                source: SourceRecord::new(
                    IoAdapter::GerberDirectory,
                    role.clone(),
                    &path,
                    Some(directory),
                ),
                path,
            };
            match role {
                IoRole::DrillSidecar => sidecars.excellon_files.push(discovered),
                IoRole::NetlistSidecar => sidecars.ipc356_files.push(discovered),
                IoRole::BomFile => sidecars.bom_files.push(discovered),
                IoRole::CentroidFile => sidecars.centroid_files.push(discovered),
                IoRole::NetlistFile => sidecars.netlist_files.push(discovered),
                IoRole::FabDrawing => sidecars.fab_drawing_files.push(discovered),
                IoRole::AssemblyDrawing => sidecars.assembly_drawing_files.push(discovered),
                IoRole::ReadmeFile => sidecars.readme_files.push(discovered),
                IoRole::RoutDrawingFile => sidecars.rout_drawing_files.push(discovered),
                IoRole::ManufacturingHandoff => {
                    sidecars.manufacturing_handoff_files.push(discovered)
                }
                IoRole::GerberLayer | IoRole::KiCadBoard | IoRole::Waiver => {}
            }
        }
    }
    sidecars.sort();
    Ok(sidecars)
}

/// Recursively discover manufacturing package sidecars in extracted archives.
pub fn discover_package_sidecars_from_archives(
    packages: &[(PathBuf, PathBuf)],
) -> Result<PackageSidecars> {
    let mut sidecars = PackageSidecars::default();
    for (archive, directory) in packages {
        discover_package_sidecars_tree_inner(
            directory,
            archive,
            IoAdapter::PackageArchive,
            &mut sidecars,
        )?;
    }
    sidecars.sort();
    Ok(sidecars)
}

/// Recursively discover manufacturing sidecars in converter output directories.
pub fn discover_package_sidecars_from_conversions(
    outputs: &[(PathBuf, PathBuf)],
) -> Result<PackageSidecars> {
    let mut sidecars = PackageSidecars::default();
    for (source, directory) in outputs {
        discover_package_sidecars_tree_inner(
            directory,
            source,
            IoAdapter::Conversion,
            &mut sidecars,
        )?;
    }
    sidecars.sort();
    Ok(sidecars)
}

fn discover_package_sidecars_tree_inner(
    directory: &Path,
    origin: &Path,
    adapter: IoAdapter,
    sidecars: &mut PackageSidecars,
) -> Result<()> {
    for entry in std::fs::read_dir(directory).with_context(|| {
        format!(
            "failed to read extracted package sidecar directory {}",
            directory.display()
        )
    })? {
        let entry =
            entry.with_context(|| format!("failed to read entry in {}", directory.display()))?;
        let path = entry.path();
        if path.is_dir() {
            discover_package_sidecars_tree_inner(&path, origin, adapter.clone(), sidecars)?;
            continue;
        }
        if !path.is_file() {
            continue;
        }
        let Some(role) = classify_package_sidecar(&path) else {
            continue;
        };
        let discovered = DiscoveredFile {
            source: SourceRecord::new(adapter.clone(), role.clone(), &path, Some(origin)),
            path,
        };
        match role {
            IoRole::DrillSidecar => sidecars.excellon_files.push(discovered),
            IoRole::NetlistSidecar => sidecars.ipc356_files.push(discovered),
            IoRole::BomFile => sidecars.bom_files.push(discovered),
            IoRole::CentroidFile => sidecars.centroid_files.push(discovered),
            IoRole::NetlistFile => sidecars.netlist_files.push(discovered),
            IoRole::FabDrawing => sidecars.fab_drawing_files.push(discovered),
            IoRole::AssemblyDrawing => sidecars.assembly_drawing_files.push(discovered),
            IoRole::ReadmeFile => sidecars.readme_files.push(discovered),
            IoRole::RoutDrawingFile => sidecars.rout_drawing_files.push(discovered),
            IoRole::ManufacturingHandoff => sidecars.manufacturing_handoff_files.push(discovered),
            IoRole::GerberLayer | IoRole::KiCadBoard | IoRole::Waiver => {}
        }
    }
    Ok(())
}

/// Return true when a path looks like a Gerber layer export.
pub fn is_gerber_path(path: &Path) -> bool {
    let Some(name) = path.file_name().and_then(|name| name.to_str()) else {
        return false;
    };
    let lower = name.to_ascii_lowercase();
    matches!(
        path.extension()
            .and_then(|extension| extension.to_str())
            .map(str::to_ascii_lowercase)
            .as_deref(),
        Some(
            "gbr"
                | "ger"
                | "gtl"
                | "gbl"
                | "gts"
                | "gbs"
                | "gto"
                | "gbo"
                | "gko"
                | "gm1"
                | "gm2"
                | "gml"
                | "gpb"
                | "gpt"
        )
    ) || lower.starts_with("gerber_")
        || lower.contains("copper")
        || lower.contains("silkscreen")
        || lower.contains("soldermask")
        || lower.contains("solderpaste")
        || lower.contains("outline")
}

fn classify_package_sidecar(path: &Path) -> Option<IoRole> {
    if is_gerber_path(path) {
        return None;
    }

    let name = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();

    if matches!(extension.as_str(), "drl" | "xln" | "exc" | "drill")
        || has_any(&name, &["excellon", "pth", "npth"])
            && matches!(extension.as_str(), "txt" | "tap")
    {
        return Some(IoRole::DrillSidecar);
    }
    if matches!(extension.as_str(), "356" | "ipc")
        || has_any(&name, &["ipc356", "ipc-356", "d-356", "d356"])
    {
        return Some(IoRole::NetlistSidecar);
    }
    if matches!(extension.as_str(), "xml" | "ipc2581" | "cvg")
        && has_any(&name, &["ipc2581", "ipc-2581", "dpmx", "cvg"])
    {
        return Some(IoRole::ManufacturingHandoff);
    }
    if matches!(extension.as_str(), "gencad" | "gcd" | "cad")
        && has_any(&name, &["gencad", "gen-cad", "test", "fixture"])
    {
        return Some(IoRole::ManufacturingHandoff);
    }
    if matches!(
        extension.as_str(),
        "csv" | "tsv" | "txt" | "log" | "xml" | "json" | "brd" | "bsdl" | "svf"
    ) && has_any(
        &name,
        &[
            "boundary-scan",
            "boundary_scan",
            "jtag",
            "flying-probe",
            "flying_probe",
            "aoi",
            "bed-of-nails",
            "bed_of_nails",
            "bon",
            "fixture",
            "ict",
            "testpoint",
            "test-point",
        ],
    ) {
        return Some(IoRole::ManufacturingHandoff);
    }
    if matches!(extension.as_str(), "zip" | "tgz" | "tar" | "gz")
        && has_any(&name, &["odb", "odb++", "odbpp"])
    {
        return Some(IoRole::ManufacturingHandoff);
    }
    if extension == "json" && has_any(&name, &["statistics", "stats", "kicad-stats"]) {
        return Some(IoRole::ManufacturingHandoff);
    }
    if matches!(
        extension.as_str(),
        "png" | "jpg" | "jpeg" | "tif" | "tiff" | "bmp" | "webp"
    ) && has_any(
        &name,
        &[
            "render",
            "rendering",
            "preview",
            "screenshot",
            "viewer",
            "visual",
            "raster",
            "image",
        ],
    ) {
        return Some(IoRole::ManufacturingHandoff);
    }
    if matches!(
        extension.as_str(),
        "step" | "stp" | "stepz" | "stl" | "obj" | "ply" | "glb" | "gltf" | "u3d"
    ) {
        return Some(IoRole::ManufacturingHandoff);
    }
    if has_any(&name, &["bom", "bill-of-materials", "bill_of_materials"])
        && matches!(
            extension.as_str(),
            "csv" | "tsv" | "txt" | "xlsx" | "xls" | "xlsm" | "xlsb" | "ods" | "json"
        )
    {
        return Some(IoRole::BomFile);
    }
    if has_any(
        &name,
        &[
            "centroid",
            "placement",
            "positions",
            "pick-place",
            "pick_place",
            "pnp",
            "cpl",
        ],
    ) && matches!(
        extension.as_str(),
        "csv" | "tsv" | "txt" | "pos" | "xlsx" | "xls" | "xlsm" | "xlsb" | "ods" | "json"
    ) {
        return Some(IoRole::CentroidFile);
    }
    if has_any(&name, &["readme", "release", "notes", "fabrication-notes"])
        && matches!(extension.as_str(), "md" | "markdown" | "txt")
    {
        return Some(IoRole::ReadmeFile);
    }
    if has_any(&name, &["netlist", "nets"])
        && matches!(
            extension.as_str(),
            "csv" | "tsv" | "txt" | "net" | "xlsx" | "xls" | "xlsm" | "xlsb" | "ods" | "json"
        )
    {
        return Some(IoRole::NetlistFile);
    }
    if has_any(
        &name,
        &[
            "rout", "route", "routing", "vscore", "v-score", "panel", "tool",
        ],
    ) && matches!(
        extension.as_str(),
        "pdf"
            | "dxf"
            | "dxb"
            | "svg"
            | "dwg"
            | "sat"
            | "sab"
            | "acis"
            | "png"
            | "jpg"
            | "jpeg"
            | "ps"
            | "eps"
            | "plt"
            | "hpgl"
            | "hpg"
    ) {
        return Some(IoRole::RoutDrawingFile);
    }
    if has_any(&name, &["fab", "fabrication", "fabricator"])
        && matches!(
            extension.as_str(),
            "pdf"
                | "dxf"
                | "dxb"
                | "svg"
                | "dwg"
                | "sat"
                | "sab"
                | "acis"
                | "ps"
                | "eps"
                | "plt"
                | "hpgl"
                | "hpg"
        )
    {
        return Some(IoRole::FabDrawing);
    }
    if has_any(&name, &["assy", "assembly", "placement"])
        && matches!(
            extension.as_str(),
            "pdf"
                | "dxf"
                | "dxb"
                | "svg"
                | "dwg"
                | "sat"
                | "sab"
                | "acis"
                | "png"
                | "jpg"
                | "jpeg"
                | "ps"
                | "eps"
                | "plt"
                | "hpgl"
                | "hpg"
        )
    {
        return Some(IoRole::AssemblyDrawing);
    }
    None
}

impl PackageSidecars {
    pub(crate) fn sort(&mut self) {
        self.excellon_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.ipc356_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.bom_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.centroid_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.netlist_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.fab_drawing_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.assembly_drawing_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.readme_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.rout_drawing_files
            .sort_by(|left, right| left.path.cmp(&right.path));
        self.manufacturing_handoff_files
            .sort_by(|left, right| left.path.cmp(&right.path));
    }
}

fn has_any(haystack: &str, needles: &[&str]) -> bool {
    needles.iter().any(|needle| haystack.contains(needle))
}

#[cfg(test)]
mod tests {
    use std::fs::{create_dir_all, remove_dir_all, write};
    use std::path::PathBuf;
    use std::process::id;

    use super::{
        IoAdapter, IoRole, SourceRecord, discover_gerber_dir, discover_package_sidecars,
        discover_package_sidecars_from_conversions, is_gerber_path,
    };

    #[test]
    fn gerber_path_detection_covers_extensions_and_jlc_style_names() {
        assert!(is_gerber_path(&PathBuf::from("board.gbr")));
        assert!(is_gerber_path(&PathBuf::from("Gerber_TopCopperLayer.GTL")));
        assert!(is_gerber_path(&PathBuf::from("Fabrication_Outline.GKO")));
        assert!(!is_gerber_path(&PathBuf::from("board.drl")));
        assert!(!is_gerber_path(&PathBuf::from("readme.txt")));
    }

    #[test]
    fn gerber_directory_discovery_is_sorted_and_records_origin() {
        let dir = std::env::temp_dir().join(format!("hyperdrc-io-dir-{}", id()));
        let _ = remove_dir_all(&dir);
        create_dir_all(&dir).unwrap();
        write(dir.join("z-bottom.gbl"), "%").unwrap();
        write(dir.join("a-top.gtl"), "%").unwrap();
        write(dir.join("notes.txt"), "not gerber").unwrap();

        let files = discover_gerber_dir(&dir).unwrap();

        assert_eq!(files[0].path, dir.join("a-top.gtl"));
        assert_eq!(files[1].path, dir.join("z-bottom.gbl"));
        assert_eq!(files[0].source.adapter, IoAdapter::GerberDirectory);
        assert_eq!(files[0].source.role, IoRole::GerberLayer);
        assert_eq!(
            files[0].source.origin.as_deref(),
            Some(dir.to_str().unwrap())
        );
        let _ = remove_dir_all(&dir);
    }

    #[test]
    fn discover_gerber_dir_skips_directory_nodes_and_non_matching_files() {
        let dir = std::env::temp_dir().join(format!("hyperdrc-io-dir-skip-{}", id()));
        let _ = remove_dir_all(&dir);
        create_dir_all(&dir).unwrap();
        write(dir.join("top-layer.gtl"), "%").unwrap();
        write(dir.join("README.md"), "notes").unwrap();
        create_dir_all(dir.join("subdir")).unwrap();
        write(dir.join("subdir").join("inner.gbr"), "nope").unwrap();

        let files = discover_gerber_dir(&dir).unwrap();

        assert_eq!(files.len(), 1);
        assert_eq!(files[0].path, dir.join("top-layer.gtl"));
        assert_eq!(files[0].source.role, IoRole::GerberLayer);
        let _ = remove_dir_all(&dir);
    }

    #[test]
    fn discover_gerber_dir_reports_read_error_when_directory_is_missing() {
        let dir = std::env::temp_dir().join(format!("hyperdrc-io-dir-missing-{}", id()));
        let _ = remove_dir_all(&dir);

        let err = discover_gerber_dir(&dir).unwrap_err().to_string();

        assert!(err.contains("failed to read Gerber directory"));
        assert!(err.contains(dir.to_string_lossy().as_ref()));
    }

    #[test]
    fn package_sidecar_discovery_classifies_common_release_files() {
        let dir = std::env::temp_dir().join(format!("hyperdrc-sidecars-{}", id()));
        let _ = remove_dir_all(&dir);
        create_dir_all(&dir).unwrap();
        for name in [
            "board-top.gtl",
            "board.drl",
            "ipc356.ipc",
            "project_bom.csv",
            "project_cpl.csv",
            "netlist.net",
            "fab_drawing.pdf",
            "fabrication_enclosure.sat",
            "assembly_drawing.pdf",
            "assembly_fixture.dwg",
            "fabrication_plot.ps",
            "assembly_plot.eps",
            "README.md",
            "panel_route.dxf",
            "panel_route.hpgl",
            "panel_route.sab",
            "ipc2581.xml",
            "odb_output.zip",
            "fixture_gencad.cad",
            "flying_probe_report.csv",
            "boundary_scan.svf",
            "board.step",
            "board.glb",
            "board.ply",
            "statistics.json",
            "top_render.png",
            "bottom_preview.tiff",
            "unrelated.log",
        ] {
            write(dir.join(name), "x").unwrap();
        }

        let sidecars = discover_package_sidecars(std::slice::from_ref(&dir)).unwrap();

        assert_eq!(sidecars.excellon_files.len(), 1);
        assert_eq!(sidecars.ipc356_files.len(), 1);
        assert_eq!(sidecars.bom_files.len(), 1);
        assert_eq!(sidecars.centroid_files.len(), 1);
        assert_eq!(sidecars.netlist_files.len(), 1);
        assert_eq!(sidecars.fab_drawing_files.len(), 3);
        assert_eq!(sidecars.assembly_drawing_files.len(), 3);
        assert_eq!(sidecars.readme_files.len(), 1);
        assert_eq!(sidecars.rout_drawing_files.len(), 3);
        assert_eq!(sidecars.manufacturing_handoff_files.len(), 11);
        assert_eq!(
            sidecars.bom_files[0].source.adapter,
            IoAdapter::GerberDirectory
        );
        assert_eq!(sidecars.bom_files[0].source.role, IoRole::BomFile);
        assert_eq!(
            sidecars.bom_files[0].source.origin.as_deref(),
            Some(dir.to_str().unwrap())
        );
        let _ = remove_dir_all(dir);
    }

    #[test]
    fn package_sidecar_discovery_classifies_structured_table_files() {
        let dir = std::env::temp_dir().join(format!("hyperdrc-structured-sidecars-{}", id()));
        let _ = remove_dir_all(&dir);
        create_dir_all(&dir).unwrap();
        write(dir.join("project_bom.json"), "[]").unwrap();
        write(dir.join("project_positions.xlsx"), "x").unwrap();
        write(dir.join("netlist.ods"), "x").unwrap();

        let sidecars = discover_package_sidecars(std::slice::from_ref(&dir)).unwrap();

        assert_eq!(sidecars.bom_files.len(), 1);
        assert_eq!(sidecars.centroid_files.len(), 1);
        assert_eq!(sidecars.netlist_files.len(), 1);
        let _ = remove_dir_all(dir);
    }

    #[test]
    fn converted_sidecar_discovery_recurses_and_marks_conversion_origin() {
        let source = std::env::temp_dir().join(format!("hyperdrc-conv-source-{}", id()));
        let dir = std::env::temp_dir().join(format!("hyperdrc-conv-sidecars-{}", id()));
        let _ = remove_dir_all(&source);
        let _ = remove_dir_all(&dir);
        create_dir_all(&source).unwrap();
        create_dir_all(dir.join("nested")).unwrap();
        write(dir.join("nested").join("board.drl"), "M48\nMETRIC\nM30\n").unwrap();

        let sidecars =
            discover_package_sidecars_from_conversions(&[(source.clone(), dir.clone())]).unwrap();

        assert_eq!(sidecars.excellon_files.len(), 1);
        assert_eq!(
            sidecars.excellon_files[0].source.adapter,
            IoAdapter::Conversion
        );
        assert_eq!(
            sidecars.excellon_files[0].source.origin.as_deref(),
            Some(source.to_str().unwrap())
        );
        let _ = remove_dir_all(source);
        let _ = remove_dir_all(dir);
    }

    #[test]
    fn gerber_path_detection_handles_keyword_only_matches_and_rejects_obvious_false_positives() {
        assert!(is_gerber_path(&PathBuf::from("TopCopper_layer")));
        assert!(is_gerber_path(&PathBuf::from("top-copper")));
        assert!(is_gerber_path(&PathBuf::from("outline_bottom")));
        assert!(is_gerber_path(&PathBuf::from("soldermask-top")));
        assert!(is_gerber_path(&PathBuf::from("solderpaste-bottom")));
        assert!(is_gerber_path(&PathBuf::from("silkscreen_top")));
        assert!(!is_gerber_path(&PathBuf::from("readme.gbr.backup")));
        assert!(!is_gerber_path(&PathBuf::from("readme.txt")));
        assert!(!is_gerber_path(&PathBuf::from(".gbr")));
    }

    #[test]
    fn source_record_serializes_paths_as_display_strings() {
        let source = SourceRecord::new(
            IoAdapter::Conversion,
            IoRole::GerberLayer,
            PathBuf::from("top.gbr"),
            Some(PathBuf::from("board.kicad_pcb")),
        )
        .with_transformation_context(
            Some("hyperdrc-input-v1:abcd".to_string()),
            vec!["kicad-cli pcb export gerbers board.kicad_pcb".to_string()],
        );

        assert_eq!(source.path, "top.gbr");
        assert_eq!(source.origin.as_deref(), Some("board.kicad_pcb"));
        assert_eq!(
            source.source_hash.as_deref(),
            Some("hyperdrc-input-v1:abcd")
        );
        assert_eq!(source.transformation_history.len(), 1);
    }
}
