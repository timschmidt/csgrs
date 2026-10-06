//! Licensing-safe intelligent PCB exchange and external-adapter contracts.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use quick_xml::Reader;
use quick_xml::events::Event;
use serde::{Deserialize, Serialize};

use crate::{PackageDigest, PortableArtifactPath};

/// Stable external-adapter wire protocol.
pub const INTELLIGENT_EXCHANGE_SCHEMA: &str = "hypercircuit.intelligent-pcb-exchange";
/// Current external-adapter wire revision.
pub const INTELLIGENT_EXCHANGE_VERSION: u32 = 1;

/// Intelligent manufacturing exchange family selected by a release.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum IntelligentPcbExchangeFormat {
    /// IPC-2581 revision C, validated only against caller-supplied official schemas.
    Ipc2581C,
    /// Siemens ODB++Design through a separately licensed adapter.
    OdbDesign,
    /// Deterministic non-production format used by adapter contract tests.
    ContractTest,
}

/// Versioned external adapter identity.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct IntelligentPcbExchangeAdapterIdentity {
    pub name: String,
    pub version: String,
    pub format: IntelligentPcbExchangeFormat,
}

/// One content-addressed file inside an intelligent exchange package.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct IntelligentPcbExchangeFile {
    pub path: PortableArtifactPath,
    pub media_type: String,
    pub byte_len: u64,
    pub digest: PackageDigest,
}

/// Canonical manifest for an adapter-produced package.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct IntelligentPcbExchangeManifest {
    pub schema: String,
    pub version: u32,
    pub format: IntelligentPcbExchangeFormat,
    pub adapter: IntelligentPcbExchangeAdapterIdentity,
    pub source_release_digest: PackageDigest,
    pub files: Vec<IntelligentPcbExchangeFile>,
    pub evidence: Vec<String>,
}

/// Exact adapter package bytes plus their canonical content-addressed manifest.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct IntelligentPcbExchangePackage {
    pub manifest: IntelligentPcbExchangeManifest,
    pub files: BTreeMap<String, Vec<u8>>,
}

impl IntelligentPcbExchangePackage {
    /// Builds and verifies a deterministic package from exact file bytes.
    pub fn new(
        format: IntelligentPcbExchangeFormat,
        adapter: IntelligentPcbExchangeAdapterIdentity,
        source_release_digest: PackageDigest,
        files: impl IntoIterator<Item = (String, String, Vec<u8>)>,
        evidence: Vec<String>,
    ) -> Result<Self, IntelligentPcbExchangeError> {
        if adapter.format != format {
            return Err(IntelligentPcbExchangeError::FormatMismatch);
        }
        let mut bytes_by_path = BTreeMap::new();
        let mut descriptors = Vec::new();
        for (path, media_type, bytes) in files {
            let path = PortableArtifactPath::new(path)?;
            if media_type.trim().is_empty() {
                return Err(IntelligentPcbExchangeError::InvalidManifest(
                    "empty media type".into(),
                ));
            }
            descriptors.push(IntelligentPcbExchangeFile {
                path: path.clone(),
                media_type,
                byte_len: bytes.len() as u64,
                digest: PackageDigest::sha256(&bytes),
            });
            if bytes_by_path.insert(path.as_str().into(), bytes).is_some() {
                return Err(IntelligentPcbExchangeError::DuplicateFile(
                    path.as_str().into(),
                ));
            }
        }
        descriptors.sort_by(|left, right| left.path.cmp(&right.path));
        let package = Self {
            manifest: IntelligentPcbExchangeManifest {
                schema: INTELLIGENT_EXCHANGE_SCHEMA.into(),
                version: INTELLIGENT_EXCHANGE_VERSION,
                format,
                adapter,
                source_release_digest,
                files: descriptors,
                evidence,
            },
            files: bytes_by_path,
        };
        package.verify()?;
        Ok(package)
    }

    /// Verifies schema, adapter identity, file set, lengths, and exact digests.
    pub fn verify(&self) -> Result<(), IntelligentPcbExchangeError> {
        if self.manifest.schema != INTELLIGENT_EXCHANGE_SCHEMA
            || self.manifest.version != INTELLIGENT_EXCHANGE_VERSION
        {
            return Err(IntelligentPcbExchangeError::ProtocolVersion);
        }
        if self.manifest.adapter.format != self.manifest.format {
            return Err(IntelligentPcbExchangeError::FormatMismatch);
        }
        let mut expected = BTreeSet::new();
        for descriptor in &self.manifest.files {
            if !expected.insert(descriptor.path.as_str()) {
                return Err(IntelligentPcbExchangeError::DuplicateFile(
                    descriptor.path.as_str().into(),
                ));
            }
            let bytes = self.files.get(descriptor.path.as_str()).ok_or_else(|| {
                IntelligentPcbExchangeError::MissingFile(descriptor.path.as_str().into())
            })?;
            if bytes.len() as u64 != descriptor.byte_len
                || PackageDigest::sha256(bytes) != descriptor.digest
            {
                return Err(IntelligentPcbExchangeError::DigestMismatch(
                    descriptor.path.as_str().into(),
                ));
            }
        }
        if let Some(path) = self
            .files
            .keys()
            .find(|path| !expected.contains(path.as_str()))
        {
            return Err(IntelligentPcbExchangeError::UnexpectedFile(path.clone()));
        }
        Ok(())
    }

    /// Digest of the canonical package manifest; file digests make it transitive.
    pub fn digest(&self) -> PackageDigest {
        PackageDigest::sha256(
            &serde_json::to_vec(&self.manifest)
                .expect("intelligent exchange manifest is infallibly serializable"),
        )
    }
}

/// Request sent to an intelligent PCB exchange adapter.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct IntelligentPcbExchangeRequest {
    pub schema: String,
    pub version: u32,
    pub requested_format: IntelligentPcbExchangeFormat,
    pub source: IntelligentPcbExchangePackage,
    pub options: BTreeMap<String, String>,
}

impl IntelligentPcbExchangeRequest {
    /// Creates a versioned request over an already verified source package.
    pub fn new(
        requested_format: IntelligentPcbExchangeFormat,
        source: IntelligentPcbExchangePackage,
    ) -> Result<Self, IntelligentPcbExchangeError> {
        source.verify()?;
        Ok(Self {
            schema: INTELLIGENT_EXCHANGE_SCHEMA.into(),
            version: INTELLIGENT_EXCHANGE_VERSION,
            requested_format,
            source,
            options: BTreeMap::new(),
        })
    }
}

/// Library-first adapter interface; proprietary implementations remain out of process.
pub trait IntelligentPcbExchangeAdapter {
    /// Versioned adapter identity.
    fn identity(&self) -> &IntelligentPcbExchangeAdapterIdentity;
    /// Produces a verified content-addressed exchange package.
    fn export(
        &self,
        request: &IntelligentPcbExchangeRequest,
    ) -> Result<IntelligentPcbExchangePackage, IntelligentPcbExchangeError>;
}

/// Sandboxed-process adapter with bounded response and timeout policy.
pub struct SubprocessIntelligentPcbExchangeAdapter {
    identity: IntelligentPcbExchangeAdapterIdentity,
    program: PathBuf,
    arguments: Vec<String>,
    timeout: Duration,
    maximum_response_bytes: u64,
}

impl SubprocessIntelligentPcbExchangeAdapter {
    /// Constructs an out-of-process adapter with conservative defaults.
    pub fn new(
        identity: IntelligentPcbExchangeAdapterIdentity,
        program: impl Into<PathBuf>,
    ) -> Self {
        Self {
            identity,
            program: program.into(),
            arguments: Vec::new(),
            timeout: Duration::from_secs(120),
            maximum_response_bytes: 512 * 1024 * 1024,
        }
    }

    /// Supplies fixed program arguments.
    pub fn with_arguments(mut self, arguments: Vec<String>) -> Self {
        self.arguments = arguments;
        self
    }

    /// Overrides the process deadline.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Overrides the maximum JSON response size.
    pub fn with_maximum_response_bytes(mut self, maximum: u64) -> Self {
        self.maximum_response_bytes = maximum;
        self
    }
}

impl IntelligentPcbExchangeAdapter for SubprocessIntelligentPcbExchangeAdapter {
    fn identity(&self) -> &IntelligentPcbExchangeAdapterIdentity {
        &self.identity
    }

    fn export(
        &self,
        request: &IntelligentPcbExchangeRequest,
    ) -> Result<IntelligentPcbExchangePackage, IntelligentPcbExchangeError> {
        validate_request(request, &self.identity)?;
        let request_bytes = serde_json::to_vec(request)
            .map_err(|error| IntelligentPcbExchangeError::Protocol(error.to_string()))?;
        let mut child = Command::new(&self.program)
            .args(&self.arguments)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|error| IntelligentPcbExchangeError::Process(error.to_string()))?;
        let mut stdin = child
            .stdin
            .take()
            .ok_or_else(|| IntelligentPcbExchangeError::Process("missing child stdin".into()))?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| IntelligentPcbExchangeError::Process("missing child stdout".into()))?;
        let stderr = child
            .stderr
            .take()
            .ok_or_else(|| IntelligentPcbExchangeError::Process("missing child stderr".into()))?;
        let writer = thread::spawn(move || stdin.write_all(&request_bytes));
        let maximum = self.maximum_response_bytes;
        let output_reader = thread::spawn(move || read_bounded(stdout, maximum));
        let error_reader = thread::spawn(move || read_bounded(stderr, 64 * 1024));
        let started = Instant::now();
        let status = loop {
            if let Some(status) = child
                .try_wait()
                .map_err(|error| IntelligentPcbExchangeError::Process(error.to_string()))?
            {
                break status;
            }
            if started.elapsed() >= self.timeout {
                let _ = child.kill();
                let _ = child.wait();
                return Err(IntelligentPcbExchangeError::Timeout);
            }
            thread::sleep(Duration::from_millis(5));
        };
        let writer_result = writer
            .join()
            .map_err(|_| IntelligentPcbExchangeError::Process("stdin writer panicked".into()))?;
        let output = output_reader
            .join()
            .map_err(|_| IntelligentPcbExchangeError::Process("stdout reader panicked".into()))??;
        let stderr = error_reader
            .join()
            .map_err(|_| IntelligentPcbExchangeError::Process("stderr reader panicked".into()))??;
        if output.len() as u64 > self.maximum_response_bytes {
            return Err(IntelligentPcbExchangeError::ResponseLimit);
        }
        if !status.success() {
            return Err(IntelligentPcbExchangeError::Crashed {
                status: status.code(),
                stderr: String::from_utf8_lossy(&stderr).into_owned(),
            });
        }
        writer_result.map_err(|error| IntelligentPcbExchangeError::Process(error.to_string()))?;
        let package: IntelligentPcbExchangePackage = serde_json::from_slice(&output)
            .map_err(|error| IntelligentPcbExchangeError::Protocol(error.to_string()))?;
        package.verify()?;
        if package.manifest.format != request.requested_format
            || package.manifest.adapter != self.identity
            || package.manifest.source_release_digest
                != request.source.manifest.source_release_digest
        {
            return Err(IntelligentPcbExchangeError::FormatMismatch);
        }
        Ok(package)
    }
}

fn validate_request(
    request: &IntelligentPcbExchangeRequest,
    identity: &IntelligentPcbExchangeAdapterIdentity,
) -> Result<(), IntelligentPcbExchangeError> {
    if request.schema != INTELLIGENT_EXCHANGE_SCHEMA
        || request.version != INTELLIGENT_EXCHANGE_VERSION
    {
        return Err(IntelligentPcbExchangeError::ProtocolVersion);
    }
    request.source.verify()?;
    if request.requested_format != identity.format {
        return Err(IntelligentPcbExchangeError::FormatMismatch);
    }
    Ok(())
}

fn read_bounded(
    mut reader: impl Read,
    maximum: u64,
) -> Result<Vec<u8>, IntelligentPcbExchangeError> {
    let mut bytes = Vec::new();
    reader
        .by_ref()
        .take(maximum.saturating_add(1))
        .read_to_end(&mut bytes)
        .map_err(|error| IntelligentPcbExchangeError::Process(error.to_string()))?;
    Ok(bytes)
}

/// Caller-supplied official IPC-2581 schema identity and root policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ipc2581XmlPolicy {
    pub schema_uri: String,
    pub schema_digest: PackageDigest,
    pub root_local_name: String,
    pub namespace_attribute: String,
    pub namespace: String,
    pub revision_attribute: String,
    pub revision: String,
    pub maximum_bytes: usize,
    pub maximum_depth: usize,
    pub maximum_events: usize,
}

/// Security-focused inspection evidence produced before XSD validation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ipc2581XmlInspection {
    pub schema_uri: String,
    pub root_local_name: String,
    pub namespace: String,
    pub revision: String,
    pub events: usize,
    pub maximum_depth: usize,
    pub schema_digest: PackageDigest,
}

/// Rejects entities/DTDs and enforces caller-supplied root, revision, and resource policy.
///
/// This is deliberately not a substitute for validation against the official
/// XSD named by `policy`.
pub fn inspect_ipc2581_xml(
    bytes: &[u8],
    policy: &Ipc2581XmlPolicy,
) -> Result<Ipc2581XmlInspection, IntelligentPcbExchangeError> {
    if bytes.len() > policy.maximum_bytes {
        return Err(IntelligentPcbExchangeError::XmlLimit(
            "document byte limit exceeded".into(),
        ));
    }
    let mut reader = Reader::from_reader(bytes);
    reader.config_mut().check_end_names = true;
    let mut depth = 0_usize;
    let mut maximum_depth = 0_usize;
    let mut events = 0_usize;
    let mut root = None;
    let mut root_closed = false;
    loop {
        let event = reader
            .read_event()
            .map_err(|error| IntelligentPcbExchangeError::Xml(error.to_string()))?;
        events += 1;
        if events > policy.maximum_events {
            return Err(IntelligentPcbExchangeError::XmlLimit(
                "XML event limit exceeded".into(),
            ));
        }
        let starts_container = matches!(&event, Event::Start(_));
        match event {
            Event::DocType(_) | Event::GeneralRef(_) => {
                return Err(IntelligentPcbExchangeError::Xml(
                    "DTD and entity references are forbidden".into(),
                ));
            }
            Event::Start(element) | Event::Empty(element) => {
                if depth == 0 && root.is_some() {
                    return Err(IntelligentPcbExchangeError::Xml(
                        "XML document has multiple root elements".into(),
                    ));
                }
                if root.is_none() {
                    let local = std::str::from_utf8(element.local_name().as_ref())
                        .map_err(|error| IntelligentPcbExchangeError::Xml(error.to_string()))?
                        .to_owned();
                    let mut namespace = None;
                    let mut revision = None;
                    for attribute in element.attributes().with_checks(true) {
                        let attribute = attribute
                            .map_err(|error| IntelligentPcbExchangeError::Xml(error.to_string()))?;
                        let name = std::str::from_utf8(attribute.key.as_ref())
                            .map_err(|error| IntelligentPcbExchangeError::Xml(error.to_string()))?;
                        let value = std::str::from_utf8(attribute.value.as_ref())
                            .map_err(|error| IntelligentPcbExchangeError::Xml(error.to_string()))?;
                        if name == policy.namespace_attribute {
                            namespace = Some(value.to_owned());
                        }
                        if name == policy.revision_attribute {
                            revision = Some(value.to_owned());
                        }
                    }
                    let namespace = namespace.ok_or_else(|| {
                        IntelligentPcbExchangeError::Xml(
                            "root namespace attribute is missing".into(),
                        )
                    })?;
                    let revision = revision.ok_or_else(|| {
                        IntelligentPcbExchangeError::Xml(
                            "root revision attribute is missing".into(),
                        )
                    })?;
                    if local != policy.root_local_name
                        || namespace != policy.namespace
                        || revision != policy.revision
                    {
                        return Err(IntelligentPcbExchangeError::Xml(
                            "root, namespace, or revision does not match supplied schema policy"
                                .into(),
                        ));
                    }
                    root = Some((local, namespace, revision));
                }
                if starts_container {
                    depth += 1;
                    maximum_depth = maximum_depth.max(depth);
                    if depth > policy.maximum_depth {
                        return Err(IntelligentPcbExchangeError::XmlLimit(
                            "XML nesting depth exceeded".into(),
                        ));
                    }
                } else {
                    maximum_depth = maximum_depth.max(depth.saturating_add(1));
                    if depth.saturating_add(1) > policy.maximum_depth {
                        return Err(IntelligentPcbExchangeError::XmlLimit(
                            "XML nesting depth exceeded".into(),
                        ));
                    }
                    if depth == 0 {
                        root_closed = true;
                    }
                }
            }
            Event::End(_) => {
                depth = depth.checked_sub(1).ok_or_else(|| {
                    IntelligentPcbExchangeError::Xml("unbalanced XML end tag".into())
                })?;
                if depth == 0 {
                    root_closed = true;
                }
            }
            Event::Text(text) => {
                let bytes: &[u8] = text.as_ref();
                if depth == 0 && bytes.iter().any(|byte| !byte.is_ascii_whitespace()) {
                    return Err(IntelligentPcbExchangeError::Xml(
                        "non-whitespace content outside the root element".into(),
                    ));
                }
            }
            Event::Eof => break,
            _ => {}
        }
    }
    if depth != 0 || !root_closed {
        return Err(IntelligentPcbExchangeError::Xml(
            "unbalanced XML document".into(),
        ));
    }
    let (root_local_name, namespace, revision) =
        root.ok_or_else(|| IntelligentPcbExchangeError::Xml("XML document has no root".into()))?;
    Ok(Ipc2581XmlInspection {
        schema_uri: policy.schema_uri.clone(),
        root_local_name,
        namespace,
        revision,
        events,
        maximum_depth,
        schema_digest: policy.schema_digest.clone(),
    })
}

/// Typed adapter, protocol, process, or secure-XML failure.
#[derive(Debug)]
pub enum IntelligentPcbExchangeError {
    ArtifactPath(crate::ArtifactCatalogError),
    ProtocolVersion,
    Protocol(String),
    InvalidManifest(String),
    FormatMismatch,
    DuplicateFile(String),
    MissingFile(String),
    UnexpectedFile(String),
    DigestMismatch(String),
    Process(String),
    Crashed { status: Option<i32>, stderr: String },
    Timeout,
    ResponseLimit,
    Xml(String),
    XmlLimit(String),
}

impl From<crate::ArtifactCatalogError> for IntelligentPcbExchangeError {
    fn from(value: crate::ArtifactCatalogError) -> Self {
        Self::ArtifactPath(value)
    }
}

impl Display for IntelligentPcbExchangeError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for IntelligentPcbExchangeError {}

/// Returns whether a configured adapter executable exists as a regular file.
pub fn intelligent_exchange_adapter_available(program: &Path) -> bool {
    program.is_file()
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn identity(format: IntelligentPcbExchangeFormat) -> IntelligentPcbExchangeAdapterIdentity {
        IntelligentPcbExchangeAdapterIdentity {
            name: "fixture-adapter".into(),
            version: "1".into(),
            format,
        }
    }

    fn source_package() -> IntelligentPcbExchangePackage {
        IntelligentPcbExchangePackage::new(
            IntelligentPcbExchangeFormat::ContractTest,
            identity(IntelligentPcbExchangeFormat::ContractTest),
            PackageDigest::sha256(b"release"),
            [(
                "source/release.json".into(),
                "application/json".into(),
                b"{}".to_vec(),
            )],
            Vec::new(),
        )
        .unwrap()
    }

    #[test]
    fn content_addressed_package_rejects_tampering_and_version_drift() {
        let mut package = IntelligentPcbExchangePackage::new(
            IntelligentPcbExchangeFormat::ContractTest,
            identity(IntelligentPcbExchangeFormat::ContractTest),
            PackageDigest::sha256(b"release"),
            [(
                "output/package.bin".into(),
                "application/octet-stream".into(),
                b"fixture".to_vec(),
            )],
            vec!["not a production exchange format".into()],
        )
        .unwrap();
        package.verify().unwrap();
        package.files.get_mut("output/package.bin").unwrap()[0] ^= 1;
        assert!(matches!(
            package.verify(),
            Err(IntelligentPcbExchangeError::DigestMismatch(_))
        ));
        package.files.get_mut("output/package.bin").unwrap()[0] ^= 1;
        package.manifest.version += 1;
        assert!(matches!(
            package.verify(),
            Err(IntelligentPcbExchangeError::ProtocolVersion)
        ));
    }

    #[test]
    fn ipc_xml_inspection_rejects_entities_and_depth_bombs() {
        let policy = Ipc2581XmlPolicy {
            schema_uri: "fixture://licensed-schema.xsd".into(),
            schema_digest: PackageDigest::sha256(b"schema"),
            root_local_name: "IPC-2581".into(),
            namespace_attribute: "xmlns".into(),
            namespace: "urn:fixture:ipc2581".into(),
            revision_attribute: "revision".into(),
            revision: "C".into(),
            maximum_bytes: 1024,
            maximum_depth: 2,
            maximum_events: 32,
        };
        let valid = br#"<IPC-2581 xmlns="urn:fixture:ipc2581" revision="C"><Content/></IPC-2581>"#;
        let inspection = inspect_ipc2581_xml(valid, &policy).unwrap();
        assert_eq!(inspection.revision, "C");
        assert_eq!(inspection.schema_uri, policy.schema_uri);
        assert!(inspect_ipc2581_xml(
            br#"<!DOCTYPE x [<!ENTITY e SYSTEM "file:///etc/passwd">]><IPC-2581 xmlns="urn:fixture:ipc2581" revision="C"/>"#,
            &policy
        )
        .is_err());
        assert!(
            inspect_ipc2581_xml(
                br#"<IPC-2581 xmlns="urn:fixture:ipc2581" revision="C"><a><b/></a></IPC-2581>"#,
                &policy
            )
            .is_err()
        );
        assert!(
            inspect_ipc2581_xml(
                br#"<IPC-2581 xmlns="urn:fixture:ipc2581" revision="C"/><second/>"#,
                &policy
            )
            .is_err()
        );
    }

    proptest! {
        #[test]
        fn bounded_arbitrary_ipc2581_xml_never_panics(
            bytes in proptest::collection::vec(any::<u8>(), 0..2048)
        ) {
            let policy = Ipc2581XmlPolicy {
                schema_uri: "fixture://licensed-schema.xsd".into(),
                schema_digest: PackageDigest::sha256(b"schema"),
                root_local_name: "IPC-2581".into(),
                namespace_attribute: "xmlns".into(),
                namespace: "urn:fixture:ipc2581".into(),
                revision_attribute: "revision".into(),
                revision: "C".into(),
                maximum_bytes: 2048,
                maximum_depth: 16,
                maximum_events: 256,
            };
            let _ = inspect_ipc2581_xml(&bytes, &policy);
        }
    }

    #[cfg(unix)]
    #[test]
    fn subprocess_boundary_reports_crash_timeout_malformed_output_and_version_drift() {
        let adapter_identity = identity(IntelligentPcbExchangeFormat::OdbDesign);
        let request = IntelligentPcbExchangeRequest::new(
            IntelligentPcbExchangeFormat::OdbDesign,
            source_package(),
        )
        .unwrap();
        let crashed =
            SubprocessIntelligentPcbExchangeAdapter::new(adapter_identity.clone(), "/bin/sh")
                .with_arguments(vec!["-c".into(), "exit 7".into()]);
        assert!(matches!(
            crashed.export(&request),
            Err(IntelligentPcbExchangeError::Crashed {
                status: Some(7),
                ..
            })
        ));

        let timed_out =
            SubprocessIntelligentPcbExchangeAdapter::new(adapter_identity.clone(), "/bin/sh")
                .with_arguments(vec!["-c".into(), "while :; do :; done".into()])
                .with_timeout(Duration::from_millis(20));
        assert!(matches!(
            timed_out.export(&request),
            Err(IntelligentPcbExchangeError::Timeout)
        ));

        let malformed =
            SubprocessIntelligentPcbExchangeAdapter::new(adapter_identity.clone(), "/bin/sh")
                .with_arguments(vec!["-c".into(), "printf not-json".into()]);
        assert!(matches!(
            malformed.export(&request),
            Err(IntelligentPcbExchangeError::Protocol(_))
        ));

        let oversized =
            SubprocessIntelligentPcbExchangeAdapter::new(adapter_identity.clone(), "/bin/sh")
                .with_arguments(vec!["-c".into(), "printf 123456".into()])
                .with_maximum_response_bytes(3);
        assert!(matches!(
            oversized.export(&request),
            Err(IntelligentPcbExchangeError::ResponseLimit)
        ));

        let mut stale = request;
        stale.version += 1;
        assert!(matches!(
            validate_request(&stale, &adapter_identity),
            Err(IntelligentPcbExchangeError::ProtocolVersion)
        ));
    }
}
