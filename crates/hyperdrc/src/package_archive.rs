//! Manufacturing package archive extraction.
//!
//! Archives are expanded into a run-scoped temporary workspace and then fed
//! through the same directory discovery path as ordinary Gerber packages.

use std::fs::{self, File};
use std::io;
use std::path::{Component, Path, PathBuf};

use anyhow::{Context, Result, anyhow};

/// Extracted archive package directories kept alive for the duration of a run.
#[derive(Debug)]
pub struct ExtractedPackages {
    root: PathBuf,
    packages: Vec<ExtractedPackage>,
}

/// One extracted package directory and the source archive it came from.
#[derive(Clone, Debug)]
pub struct ExtractedPackage {
    /// Original archive path.
    pub archive: PathBuf,
    /// Directory containing extracted package contents.
    pub directory: PathBuf,
}

impl ExtractedPackages {
    /// Extract all package archives into a temporary run workspace.
    pub fn extract(archives: &[PathBuf]) -> Result<Self> {
        let root = temp_root()?;
        fs::create_dir_all(&root)
            .with_context(|| format!("failed to create archive workspace {}", root.display()))?;
        let mut packages = Vec::new();
        for (index, archive) in archives.iter().enumerate() {
            let directory = root.join(format!("package-{index}"));
            fs::create_dir_all(&directory).with_context(|| {
                format!(
                    "failed to create archive output directory {}",
                    directory.display()
                )
            })?;
            extract_archive(archive, &directory)?;
            packages.push(ExtractedPackage {
                archive: archive.clone(),
                directory,
            });
        }
        Ok(Self { root, packages })
    }

    /// Extracted package records.
    pub fn packages(&self) -> &[ExtractedPackage] {
        &self.packages
    }

    /// Extracted package directories.
    pub fn directories(&self) -> Vec<PathBuf> {
        self.packages
            .iter()
            .map(|package| package.directory.clone())
            .collect()
    }
}

impl Drop for ExtractedPackages {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.root);
    }
}

fn extract_archive(archive: &Path, output_dir: &Path) -> Result<()> {
    let lower = archive
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    if lower.ends_with(".zip") {
        extract_zip(archive, output_dir)
    } else if lower.ends_with(".tar.gz") || lower.ends_with(".tgz") {
        extract_tar_gz(archive, output_dir)
    } else if lower.ends_with(".tar") {
        extract_tar(archive, output_dir)
    } else {
        Err(anyhow!(
            "unsupported package archive {}; expected .zip, .tar, .tar.gz, or .tgz",
            archive.display()
        ))
    }
}

fn extract_zip(archive: &Path, output_dir: &Path) -> Result<()> {
    let file =
        File::open(archive).with_context(|| format!("failed to open {}", archive.display()))?;
    let mut zip = zip::ZipArchive::new(file)
        .with_context(|| format!("failed to read zip archive {}", archive.display()))?;
    for index in 0..zip.len() {
        let mut entry = zip.by_index(index).with_context(|| {
            format!(
                "failed to read entry {index} from zip archive {}",
                archive.display()
            )
        })?;
        let Some(relative) = safe_archive_name(entry.name()) else {
            continue;
        };
        let output_path = output_dir.join(relative);
        if entry.is_dir() {
            fs::create_dir_all(&output_path).with_context(|| {
                format!(
                    "failed to create extracted directory {}",
                    output_path.display()
                )
            })?;
            continue;
        }
        if let Some(parent) = output_path.parent() {
            fs::create_dir_all(parent).with_context(|| {
                format!("failed to create extracted directory {}", parent.display())
            })?;
        }
        let mut output = File::create(&output_path).with_context(|| {
            format!("failed to create extracted file {}", output_path.display())
        })?;
        io::copy(&mut entry, &mut output)
            .with_context(|| format!("failed to extract {}", output_path.display()))?;
    }
    Ok(())
}

fn extract_tar_gz(archive: &Path, output_dir: &Path) -> Result<()> {
    let file =
        File::open(archive).with_context(|| format!("failed to open {}", archive.display()))?;
    let decoder = flate2::read::GzDecoder::new(file);
    extract_tar_reader(decoder, archive, output_dir)
}

fn extract_tar(archive: &Path, output_dir: &Path) -> Result<()> {
    let file =
        File::open(archive).with_context(|| format!("failed to open {}", archive.display()))?;
    extract_tar_reader(file, archive, output_dir)
}

fn extract_tar_reader<R: io::Read>(reader: R, archive: &Path, output_dir: &Path) -> Result<()> {
    let mut tar = tar::Archive::new(reader);
    for entry in tar
        .entries()
        .with_context(|| format!("failed to read tar archive {}", archive.display()))?
    {
        let mut entry =
            entry.with_context(|| format!("failed to read entry from {}", archive.display()))?;
        let path = entry
            .path()
            .with_context(|| format!("failed to read tar entry path from {}", archive.display()))?;
        let Some(relative) = safe_relative_path(&path) else {
            continue;
        };
        let output_path = output_dir.join(relative);
        let entry_type = entry.header().entry_type();
        if entry_type.is_dir() {
            fs::create_dir_all(&output_path).with_context(|| {
                format!(
                    "failed to create extracted directory {}",
                    output_path.display()
                )
            })?;
            continue;
        }
        if !entry_type.is_file() {
            continue;
        }
        if let Some(parent) = output_path.parent() {
            fs::create_dir_all(parent).with_context(|| {
                format!("failed to create extracted directory {}", parent.display())
            })?;
        }
        entry
            .unpack(&output_path)
            .with_context(|| format!("failed to extract {}", output_path.display()))?;
    }
    Ok(())
}

fn safe_relative_path(path: &Path) -> Option<PathBuf> {
    let mut safe = PathBuf::new();
    for component in path.components() {
        match component {
            Component::Normal(part) => safe.push(part),
            Component::CurDir => {}
            Component::RootDir | Component::Prefix(_) | Component::ParentDir => return None,
        }
    }
    (!safe.as_os_str().is_empty()).then_some(safe)
}

fn safe_archive_name(name: &str) -> Option<PathBuf> {
    if name.contains('\0') {
        return None;
    }
    let normalized = name.replace('\\', "/");
    safe_relative_path(Path::new(&normalized))
}

fn temp_root() -> Result<PathBuf> {
    let mut attempts = 0u32;
    loop {
        let candidate = std::env::temp_dir().join(format!(
            "hyperdrc-archives-{}-{}",
            std::process::id(),
            attempts
        ));
        if !candidate.exists() {
            return Ok(candidate);
        }
        attempts += 1;
        if attempts > 1000 {
            return Err(anyhow!("failed to allocate a temporary archive workspace"));
        }
    }
}

#[cfg(test)]
mod tests {
    use std::fs::{File, create_dir_all, read_to_string, remove_dir_all, write};
    use std::io::Write;

    use super::{ExtractedPackages, safe_archive_name, safe_relative_path};

    fn id() -> String {
        format!(
            "{}-{:?}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        )
    }

    #[test]
    fn safe_relative_paths_reject_traversal() {
        assert!(safe_relative_path(std::path::Path::new("gerbers/top.gtl")).is_some());
        assert!(safe_relative_path(std::path::Path::new("../top.gtl")).is_none());
        assert!(safe_relative_path(std::path::Path::new("/tmp/top.gtl")).is_none());
        assert!(safe_archive_name("gerbers\\top.gtl").is_some());
        assert!(safe_archive_name("..\\top.gtl").is_none());
        assert!(safe_archive_name("gerbers/\0/top.gtl").is_none());
    }

    #[test]
    fn extracts_zip_packages() {
        let dir = std::env::temp_dir().join(format!("hyperdrc-zip-source-{}", id()));
        let _ = remove_dir_all(&dir);
        create_dir_all(&dir).unwrap();
        let archive_path = dir.join("package.zip");
        {
            let file = File::create(&archive_path).unwrap();
            let mut writer = zip::ZipWriter::new(file);
            let options = zip::write::SimpleFileOptions::default();
            writer.start_file("gerbers/top.gtl", options).unwrap();
            writer.write_all(b"%MOMM*%\nM02*\n").unwrap();
            writer.start_file("../evil.gtl", options).unwrap();
            writer.write_all(b"bad").unwrap();
            writer.finish().unwrap();
        }

        let extracted = ExtractedPackages::extract(std::slice::from_ref(&archive_path)).unwrap();
        let package = &extracted.packages()[0];

        assert!(package.directory.join("gerbers/top.gtl").exists());
        assert!(!package.directory.join("evil.gtl").exists());
        assert_eq!(
            read_to_string(package.directory.join("gerbers/top.gtl")).unwrap(),
            "%MOMM*%\nM02*\n"
        );
        let _ = remove_dir_all(dir);
    }

    #[test]
    fn extracts_tar_packages() {
        let dir = std::env::temp_dir().join(format!("hyperdrc-tar-source-{}", id()));
        let _ = remove_dir_all(&dir);
        create_dir_all(dir.join("input")).unwrap();
        write(dir.join("input/bottom.gbl"), "%MOMM*%\nM02*\n").unwrap();
        let archive_path = dir.join("package.tar");
        {
            let file = File::create(&archive_path).unwrap();
            let mut builder = tar::Builder::new(file);
            builder
                .append_path_with_name(dir.join("input/bottom.gbl"), "gerbers/bottom.gbl")
                .unwrap();
            builder.finish().unwrap();
        }

        let extracted = ExtractedPackages::extract(std::slice::from_ref(&archive_path)).unwrap();

        assert!(
            extracted.packages()[0]
                .directory
                .join("gerbers/bottom.gbl")
                .exists()
        );
        let _ = remove_dir_all(dir);
    }
}
