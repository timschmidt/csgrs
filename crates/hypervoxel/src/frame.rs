//! Exact voxel grid frames.
//!
//! A `GridFrame` keeps the grid origin, axis pitches, source units, and source
//! provenance together instead of rediscovering them from scalar coordinates
//! or approximated chunk sizes.

use hyperreal::{Real, RealExactSetFacts, RealSign};

use crate::{ChunkShape, HypervoxelError, HypervoxelResult};

/// Maximum exact octree depth supported by the current `u64` address model.
pub const MAX_ADDRESS_DEPTH: u8 = 21;

/// Length units carried by an exact grid frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LengthUnit {
    /// Unitless model coordinates.
    Unitless,
    /// Meters.
    Meter,
    /// Millimeters.
    Millimeter,
    /// Micrometers.
    Micrometer,
    /// Nanometers.
    Nanometer,
}

/// Source/provenance for a grid frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GridSource {
    /// Stable source identifier, such as a mesh id, path id, or imported file id.
    pub id: String,
    /// Monotonic construction or import version.
    pub version: u64,
}

/// Orientation handedness of a grid-frame manifest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GridHandedness {
    /// Right-handed axis convention.
    RightHanded,
    /// Left-handed axis convention.
    LeftHanded,
    /// Handedness is not known.
    Unknown,
}

/// Source coordinate-system family for a grid-frame manifest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GridCoordinateSystem {
    /// Hyper-native exact grid coordinates.
    HyperGrid,
    /// Imported CAD/model coordinates.
    SourceModel,
    /// Image, medical, or volume index coordinates.
    ImageVolume,
    /// Display/render coordinates.
    Display,
    /// Coordinate system is not known.
    Unknown,
}

/// Basis metadata exposed by a grid-frame manifest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GridBasis {
    /// Axis-aligned basis with positive per-axis pitch.
    AxisAligned,
    /// Signed axis permutation over an axis-aligned source basis.
    SignedPermutation,
    /// General exact affine basis.
    ExactAffine,
    /// Basis is not known.
    Unknown,
}

impl GridSource {
    /// Creates a new source/version pair.
    pub fn new(id: impl Into<String>, version: u64) -> Self {
        Self {
            id: id.into(),
            version,
        }
    }
}

/// One exact axis pitch for an axis-aligned voxel grid.
#[derive(Clone, Debug, PartialEq)]
pub struct GridAxis {
    pitch: Real,
}

impl GridAxis {
    /// Creates a grid axis after proving the pitch is structurally positive.
    pub fn new(pitch: Real, axis: usize) -> HypervoxelResult<Self> {
        match pitch.structural_facts().sign {
            Some(RealSign::Positive) => Ok(Self { pitch }),
            Some(_) => Err(HypervoxelError::NonPositiveCellAxis { axis }),
            None => Err(HypervoxelError::UnknownCellAxisSign { axis }),
        }
    }

    /// Returns the exact cell pitch along this axis.
    pub fn pitch(&self) -> &Real {
        &self.pitch
    }
}

/// Exact axis-aligned grid frame.
#[derive(Clone, Debug, PartialEq)]
pub struct GridFrame {
    origin: [Real; 3],
    axes: [GridAxis; 3],
    depth: u8,
    units: LengthUnit,
    source: Option<GridSource>,
}

impl GridFrame {
    /// Creates an exact axis-aligned frame.
    pub fn new(
        origin: [Real; 3],
        pitch: [Real; 3],
        depth: u8,
        units: LengthUnit,
        source: Option<GridSource>,
    ) -> HypervoxelResult<Self> {
        if depth > MAX_ADDRESS_DEPTH {
            return Err(HypervoxelError::DepthTooLarge {
                depth,
                max_supported: MAX_ADDRESS_DEPTH,
            });
        }

        Ok(Self {
            origin,
            axes: [
                GridAxis::new(pitch[0].clone(), 0)?,
                GridAxis::new(pitch[1].clone(), 1)?,
                GridAxis::new(pitch[2].clone(), 2)?,
            ],
            depth,
            units,
            source,
        })
    }

    /// Starts a builder for an exact grid frame.
    pub fn builder() -> GridFrameBuilder {
        GridFrameBuilder::default()
    }

    /// Returns the exact frame origin.
    pub fn origin(&self) -> &[Real; 3] {
        &self.origin
    }

    /// Returns the exact axis pitches.
    pub fn axes(&self) -> &[GridAxis; 3] {
        &self.axes
    }

    /// Returns the exact pitch for one axis.
    pub fn pitch(&self, axis: usize) -> &Real {
        self.axes[axis].pitch()
    }

    /// Returns the maximum octree/grid depth.
    pub fn depth(&self) -> u8 {
        self.depth
    }

    /// Returns the source length unit.
    pub fn units(&self) -> LengthUnit {
        self.units
    }

    /// Returns the optional source version.
    pub fn source(&self) -> Option<&GridSource> {
        self.source.as_ref()
    }

    /// Returns `2^depth`, the number of finest cells per axis.
    pub fn cells_per_axis(&self) -> u64 {
        1_u64 << self.depth
    }

    /// Returns object-level exactness facts for the origin and pitch scalars.
    ///
    /// Downstream kernels can choose integer, dyadic, shared-denominator, or
    /// general exact-rational routes from this fact packet rather than peeking
    /// into scalar internals or lowering to floats.
    pub fn facts(&self) -> GridFrameFacts {
        let scalars = [
            &self.origin[0],
            &self.origin[1],
            &self.origin[2],
            self.pitch(0),
            self.pitch(1),
            self.pitch(2),
        ];
        let exact_scalars = Real::exact_set_facts(scalars);
        GridFrameFacts {
            depth: self.depth,
            cells_per_axis: self.cells_per_axis(),
            exact_scalars,
        }
    }
}

/// Coarse exactness and scheduling facts for a grid frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GridFrameFacts {
    /// Maximum address depth for the frame.
    pub depth: u8,
    /// Number of finest cells per axis.
    pub cells_per_axis: u64,
    /// Exact-set facts for origin and pitch scalars.
    pub exact_scalars: RealExactSetFacts,
}

/// Manifest for grid-frame metadata that adapters must make explicit.
///
/// `GridFrame` stores the exact axis-aligned model used by Hyper kernels. This
/// manifest records adapter-facing frame facts such as handedness, basis,
/// source coordinate system, and chunk shape without mutating the core frame.
/// Object structure and provenance remain available so downstream predicates
/// do not infer geometry from an approximate view or import convention.
#[derive(Clone, Debug, PartialEq)]
pub struct GridFrameManifest {
    /// Exact Hyper grid frame.
    pub frame: GridFrame,
    /// Basis family declared by the adapter or caller.
    pub basis: GridBasis,
    /// Handedness declared by the adapter or caller.
    pub handedness: GridHandedness,
    /// Source coordinate-system family.
    pub coordinate_system: GridCoordinateSystem,
    /// Optional chunk shape associated with the frame.
    pub chunk_shape: Option<ChunkShape>,
}

/// Report from a grid-frame manifest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GridFrameManifestReport {
    /// Frame exactness and scheduling facts.
    pub facts: GridFrameFacts,
    /// Basis family declared by the adapter or caller.
    pub basis: GridBasis,
    /// Handedness declared by the adapter or caller.
    pub handedness: GridHandedness,
    /// Source coordinate-system family.
    pub coordinate_system: GridCoordinateSystem,
    /// Optional chunk shape associated with the frame.
    pub chunk_shape: Option<ChunkShape>,
    /// Whether the manifest has all required structural metadata.
    pub complete: bool,
}

impl GridFrameManifest {
    /// Builds a report that keeps frame metadata separate from voxel samples.
    pub fn report(&self) -> GridFrameManifestReport {
        let complete = self.basis != GridBasis::Unknown
            && self.handedness != GridHandedness::Unknown
            && self.coordinate_system != GridCoordinateSystem::Unknown
            && self.chunk_shape.is_some();
        GridFrameManifestReport {
            facts: self.frame.facts(),
            basis: self.basis,
            handedness: self.handedness,
            coordinate_system: self.coordinate_system,
            chunk_shape: self.chunk_shape,
            complete,
        }
    }
}

impl GridFrameFacts {
    /// Returns whether all origin and pitch scalars are exact rationals.
    pub fn is_exact_rational_frame(&self) -> bool {
        self.exact_scalars.is_nonempty_exact_rational()
    }

    /// Returns whether the frame admits a dyadic exact schedule.
    pub fn has_dyadic_schedule(&self) -> bool {
        self.exact_scalars.has_dyadic_schedule()
    }

    /// Returns whether the frame admits a shared-denominator exact schedule.
    pub fn has_shared_denominator_schedule(&self) -> bool {
        self.exact_scalars.has_shared_denominator_schedule()
    }

    /// Returns whether the frame admits an integer-grid exact schedule.
    pub fn has_integer_grid_schedule(&self) -> bool {
        self.exact_scalars.has_integer_grid_schedule()
    }
}

/// Builder for [`GridFrame`].
#[derive(Clone, Debug)]
pub struct GridFrameBuilder {
    origin: [Real; 3],
    pitch: [Real; 3],
    depth: u8,
    units: LengthUnit,
    source: Option<GridSource>,
}

impl Default for GridFrameBuilder {
    fn default() -> Self {
        Self {
            origin: [0.into(), 0.into(), 0.into()],
            pitch: [1.into(), 1.into(), 1.into()],
            depth: 0,
            units: LengthUnit::Unitless,
            source: None,
        }
    }
}

impl GridFrameBuilder {
    /// Sets the exact origin.
    pub fn origin(mut self, origin: [Real; 3]) -> Self {
        self.origin = origin;
        self
    }

    /// Sets the exact per-axis cell pitch.
    pub fn pitch(mut self, pitch: [Real; 3]) -> Self {
        self.pitch = pitch;
        self
    }

    /// Sets the maximum grid depth.
    pub fn depth(mut self, depth: u8) -> Self {
        self.depth = depth;
        self
    }

    /// Sets the source units.
    pub fn units(mut self, units: LengthUnit) -> Self {
        self.units = units;
        self
    }

    /// Sets source provenance.
    pub fn source(mut self, source: GridSource) -> Self {
        self.source = Some(source);
        self
    }

    /// Builds the validated frame.
    pub fn build(self) -> HypervoxelResult<GridFrame> {
        GridFrame::new(self.origin, self.pitch, self.depth, self.units, self.source)
    }
}
