//! Exact voxel grid frames.
//!
//! A `GridFrame` keeps the grid origin, axis pitches, and source units together
//! instead of rediscovering them from scalar coordinates or approximated chunk
//! sizes.

use hyperreal::{Real, RealExactSetFacts, RealSign};

use crate::{HypervoxelError, HypervoxelResult};

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
}

impl GridFrame {
    /// Creates an exact axis-aligned frame.
    pub fn new(
        origin: [Real; 3],
        pitch: [Real; 3],
        depth: u8,
        units: LengthUnit,
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
}

impl Default for GridFrameBuilder {
    fn default() -> Self {
        Self {
            origin: [0.into(), 0.into(), 0.into()],
            pitch: [1.into(), 1.into(), 1.into()],
            depth: 0,
            units: LengthUnit::Unitless,
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

    /// Builds the validated frame.
    pub fn build(self) -> HypervoxelResult<GridFrame> {
        GridFrame::new(self.origin, self.pitch, self.depth, self.units)
    }
}
