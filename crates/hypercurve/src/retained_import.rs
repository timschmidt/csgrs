//! Retained source records for lossy file-format curve imports.
//!
//! Import adapters sit on Yap's exact-computation boundary: finite file data
//! may be admitted, but its provenance and tolerance must remain visible until
//! certified predicates replace it.  The records here are deliberately small
//! evidence objects for STEP/DXF/application import layers. They do not parse
//! those formats, and they do not make imported finite samples native topology.
//! See Chee Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 3-23 (1997).

use crate::{CurveError, CurveResult, RetainedTopologyStatus};

/// Source family for a lossy retained curve import record.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedImportFormat2 {
    /// Plain finite `f64` polyline input with no external file handle.
    FinitePolyline,
    /// STEP entity evidence, typically keyed by an entity id.
    Step,
    /// DXF entity evidence, typically keyed by a handle table index.
    Dxf,
    /// Application-local import evidence.
    Application,
}

/// Absolute/relative tolerance carried from an import source.
///
/// These are evidence values only. They may explain why the source was lossy,
/// but exact topology must still be decided by certified predicates rather than
/// by accepting a tolerance band as geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RetainedSourceTolerance2 {
    absolute: f64,
    relative: f64,
}

/// Retained audit record for one imported finite curve carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedImportRecord2 {
    format: RetainedImportFormat2,
    source_index: u64,
    source_tolerance: Option<RetainedSourceTolerance2>,
    input_point_count: usize,
    emitted_segment_count: usize,
    discarded_duplicate_count: usize,
    topology_status: RetainedTopologyStatus,
}

impl RetainedSourceTolerance2 {
    /// Constructs finite nonnegative absolute/relative source tolerances.
    pub fn try_new(absolute: f64, relative: f64) -> CurveResult<Self> {
        if !absolute.is_finite() || !relative.is_finite() || absolute < 0.0 || relative < 0.0 {
            return Err(CurveError::InvalidImportRecord);
        }
        Ok(Self { absolute, relative })
    }

    /// Returns the absolute source tolerance.
    pub const fn absolute(self) -> f64 {
        self.absolute
    }

    /// Returns the relative source tolerance.
    pub const fn relative(self) -> f64 {
        self.relative
    }

    /// Returns true when the source declared an exact zero tolerance.
    pub const fn is_zero(self) -> bool {
        self.absolute == 0.0 && self.relative == 0.0
    }
}

impl RetainedImportRecord2 {
    /// Constructs a retained lossy-import audit record.
    ///
    /// `discarded_duplicate_count` records finite duplicate samples that were
    /// consumed as file/import metadata rather than emitted as zero-length
    /// topology. The topology status is always [`RetainedTopologyStatus::ImportedLossy`]
    /// because this record crosses a finite or external file-format boundary.
    pub fn try_new(
        format: RetainedImportFormat2,
        source_index: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
        input_point_count: usize,
        emitted_segment_count: usize,
        discarded_duplicate_count: usize,
    ) -> CurveResult<Self> {
        if input_point_count < 2
            || emitted_segment_count == 0
            || emitted_segment_count > input_point_count
            || discarded_duplicate_count > input_point_count
            || emitted_segment_count
                .checked_add(discarded_duplicate_count)
                .is_none_or(|count| count > input_point_count)
        {
            return Err(CurveError::InvalidImportRecord);
        }
        Ok(Self {
            format,
            source_index,
            source_tolerance,
            input_point_count,
            emitted_segment_count,
            discarded_duplicate_count,
            topology_status: RetainedTopologyStatus::ImportedLossy,
        })
    }

    /// Constructs a STEP import record.
    pub fn step(
        entity_id: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
        input_point_count: usize,
        emitted_segment_count: usize,
        discarded_duplicate_count: usize,
    ) -> CurveResult<Self> {
        Self::try_new(
            RetainedImportFormat2::Step,
            entity_id,
            source_tolerance,
            input_point_count,
            emitted_segment_count,
            discarded_duplicate_count,
        )
    }

    /// Constructs a DXF import record.
    pub fn dxf(
        handle_index: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
        input_point_count: usize,
        emitted_segment_count: usize,
        discarded_duplicate_count: usize,
    ) -> CurveResult<Self> {
        Self::try_new(
            RetainedImportFormat2::Dxf,
            handle_index,
            source_tolerance,
            input_point_count,
            emitted_segment_count,
            discarded_duplicate_count,
        )
    }

    /// Returns the retained import format.
    pub const fn format(&self) -> RetainedImportFormat2 {
        self.format
    }

    /// Returns the opaque source index.
    pub const fn source_index(&self) -> u64 {
        self.source_index
    }

    /// Returns source tolerance evidence, if supplied by the importer.
    pub const fn source_tolerance(&self) -> Option<RetainedSourceTolerance2> {
        self.source_tolerance
    }

    /// Returns the number of finite input points.
    pub const fn input_point_count(&self) -> usize {
        self.input_point_count
    }

    /// Returns the number of native segments emitted from the import.
    pub const fn emitted_segment_count(&self) -> usize {
        self.emitted_segment_count
    }

    /// Returns duplicate finite edges discarded before topology construction.
    pub const fn discarded_duplicate_count(&self) -> usize {
        self.discarded_duplicate_count
    }

    /// Returns the topology readiness status for this retained import.
    pub const fn topology_status(&self) -> RetainedTopologyStatus {
        self.topology_status
    }
}
