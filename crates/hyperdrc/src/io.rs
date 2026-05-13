//! Input/output discovery and provenance.
//!
//! The DRC engine should not need to know whether a layer came from a direct
//! Gerber file, a Gerber package directory, or a converter. This module keeps
//! that discovery logic and source metadata in one place so future adapters can
//! implement the same shape.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::Serialize;

#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum IoAdapter {
    DirectFile,
    GerberDirectory,
    Conversion,
    KiCad,
    Excellon,
    Ipc356,
    Waiver,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum IoRole {
    GerberLayer,
    KiCadBoard,
    DrillSidecar,
    NetlistSidecar,
    Waiver,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct SourceRecord {
    pub adapter: IoAdapter,
    pub role: IoRole,
    pub path: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub origin: Option<String>,
}

impl SourceRecord {
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
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DiscoveredFile {
    pub path: PathBuf,
    pub source: SourceRecord,
}

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

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::{IoAdapter, IoRole, SourceRecord, discover_gerber_dir, is_gerber_path};

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
        let dir = std::env::temp_dir().join(format!("hyperdrc-io-dir-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("z-bottom.gbl"), "%").unwrap();
        std::fs::write(dir.join("a-top.gtl"), "%").unwrap();
        std::fs::write(dir.join("notes.txt"), "not gerber").unwrap();

        let files = discover_gerber_dir(&dir).unwrap();

        assert_eq!(files[0].path, dir.join("a-top.gtl"));
        assert_eq!(files[1].path, dir.join("z-bottom.gbl"));
        assert_eq!(files[0].source.adapter, IoAdapter::GerberDirectory);
        assert_eq!(files[0].source.role, IoRole::GerberLayer);
        assert_eq!(
            files[0].source.origin.as_deref(),
            Some(dir.to_str().unwrap())
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn source_record_serializes_paths_as_display_strings() {
        let source = SourceRecord::new(
            IoAdapter::DirectFile,
            IoRole::GerberLayer,
            PathBuf::from("top.gbr"),
            Option::<PathBuf>::None,
        );

        assert_eq!(source.path, "top.gbr");
        assert!(source.origin.is_none());
    }
}
