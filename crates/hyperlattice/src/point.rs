//! Exact Cartesian points, borrowed denominator views, and structural fact snapshots.
//!
//! Point/vector conversions retain Real coordinate values; point subtraction yields
//! an oriented displacement and adding/subtracting a vector translates a point.
//! Interpolation and collection operations delegate to vector arithmetic. Arithmetic
//! and derived equality use Hyperreal semantics without a geometry policy, certainty
//! receipt, incidence record, or separate bound on nested scalar work and allocation.
//!
//! Fact packets are scheduling metadata, not topology or geometric identity proofs.
//! Their public fields can be fabricated or edited: queries trust stored values rather
//! than validating their agreement with coordinates. A point mutation requires fresh
//! facts. A borrowed view fixes the referenced coordinates but does not prevent edits
//! to its public fact fields or automatically refresh them after scalar cache changes.
//!
//! Here a shared scale means equal reduced rational denominators already recognized
//! by Hyperreal, not an arbitrary common-denominator rewrite. One-hot support means
//! one known nonzero coordinate, not necessarily one. Generic masks require N<=128;
//! the constructor rejects an empty set, and oversize eligible sets have no typed
//! rejection before the mask shift. Point2/Point3 use only two/three low mask bits.

use crate::{
    BlasResult, Real, RealExactSetFacts, SymbolicDependencyMask, Vector2, Vector3, ZeroKnowledge,
};
use std::ops::{Add, Sub};

/// Borrowed coordinate array with a snapshot of equal-denominator eligibility.
///
/// The constructor succeeds only for a nonempty set of recognized exact rationals
/// with equal reduced denominators. It retains references and facts, not a materialized
/// common factor or integer numerators. For example, (1/3,2/3) qualifies; (1/2,1/4)
/// and (0,1/3) do not, despite possessing mathematical common denominators.
///
/// The mask domain is 1<=N<=128. A larger eligible array can panic on its mask shift
/// in overflow-checked builds; there is no typed size error. Public fact fields may
/// be changed independently of the borrowed coordinates, so consumers must not treat
/// an arbitrary edited view as proof. Copying the view copies references and metadata.
#[derive(Clone, Copy, Debug)]
pub struct PointSharedScaleView<'a, const N: usize> {
    /// Original coordinate references in array order; copying the view does not clone or rescale the Reals.
    coordinates: [&'a Real; N],
    /// Snapshot from Real::exact_set_facts; this publicly editable metadata is not revalidated by packet queries.
    pub exact: RealExactSetFacts,
    /// Bit i records known-zero coordinate i in producer-created packets; public edits are not validated.
    pub known_zero_mask: u128,
    /// Bit i records known-nonzero coordinate i in producer-created packets, independently of its sign or magnitude.
    pub known_nonzero_mask: u128,
    /// Bit i records unavailable cheap zero knowledge, not mathematical zero or nonzero; public edits are not validated.
    pub unknown_zero_mask: u128,
}

impl<'a, const N: usize> PointSharedScaleView<'a, N> {
    /// Scan exact-set facts first and retain coordinates only if an equal-denominator schedule is available.
    ///
    /// Returns None for an empty set, any unrecognized/nonrational coordinate, or unequal
    /// reduced denominators; it does not compute a common denominator or refine a value
    /// into a rational representation. Then scans zero statuses in order to build masks.
    /// Requires N<=128 on the accepted path; oversize input is not rejected with a typed
    /// error before shifting. An unsuccessful eligibility query is not proof that a
    /// mathematical common-scale representation cannot exist.
    pub fn from_coordinates(coordinates: [&'a Real; N]) -> Option<Self> {
        crate::trace_dispatch!(
            "hyperlattice_point",
            "query",
            "point-shared-scale-view-from-coordinates"
        );
        let exact = Real::exact_set_facts(coordinates.iter().copied());
        if !exact.has_shared_denominator_schedule() {
            return None;
        }
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            coordinate_zero_status_masks(coordinates);
        Some(Self {
            coordinates,
            exact,
            known_zero_mask,
            known_nonzero_mask,
            unknown_zero_mask,
        })
    }

    /// Return the original references in array order; no scalar clone, normalization, or fact validation occurs.
    pub fn coordinates(self) -> [&'a Real; N] {
        self.coordinates
    }

    /// Return the array length N, independently of the editable exact.len field.
    pub const fn len(self) -> usize {
        N
    }

    /// Test N==0; the public from_coordinates constructor currently never returns an empty view.
    pub const fn is_empty(self) -> bool {
        N == 0
    }

    /// Copy the currently stored public metadata into a detached packet without rescanning coordinates or validating edits.
    pub fn facts(self) -> PointSharedScaleFacts<N> {
        PointSharedScaleFacts {
            exact: self.exact,
            known_zero_mask: self.known_zero_mask,
            known_nonzero_mask: self.known_nonzero_mask,
            unknown_zero_mask: self.unknown_zero_mask,
        }
    }

    /// Compare the stored zero mask with the low N-bit mask, without re-reading coordinates or checking other fields; requires N<=128.
    pub fn is_known_zero(self) -> bool {
        self.facts().is_known_zero()
    }

    /// Compare the stored nonzero mask with the low N-bit mask, without re-reading coordinates or checking other fields; requires N<=128.
    pub fn is_known_dense(self) -> bool {
        self.facts().is_known_dense()
    }

    /// Count every set bit of the stored known_zero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask.count_ones()
    }

    /// Count every set bit of the stored known_nonzero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask.count_ones()
    }

    /// Count every set bit of the stored unknown_zero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask.count_ones()
    }
}

/// Detached exact-set and zero-support metadata for N coordinates.
///
/// Fields are public and no constructor invariant is enforced. Producer-created
/// packets describe a point view at query time; fabricated, edited, or stale packets
/// need not agree with any coordinates. Equality compares metadata, not point images.
/// Mask comparisons require N<=128. A zero-dimensional packet with zero masks passes
/// both is_known_zero and is_known_dense, although the view constructor rejects N=0.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PointSharedScaleFacts<const N: usize> {
    /// Snapshot from Real::exact_set_facts; this publicly editable metadata is not revalidated by packet queries.
    pub exact: RealExactSetFacts,
    /// Bit i records known-zero coordinate i in producer-created packets; public edits are not validated.
    pub known_zero_mask: u128,
    /// Bit i records known-nonzero coordinate i in producer-created packets, independently of its sign or magnitude.
    pub known_nonzero_mask: u128,
    /// Bit i records unavailable cheap zero knowledge, not mathematical zero or nonzero; public edits are not validated.
    pub unknown_zero_mask: u128,
}

impl<const N: usize> PointSharedScaleFacts<N> {
    /// Compare the stored zero mask with the low N-bit mask, without re-reading coordinates or checking other fields; requires N<=128.
    pub fn is_known_zero(self) -> bool {
        self.known_zero_mask == coordinate_mask::<N>()
    }

    /// Compare the stored nonzero mask with the low N-bit mask, without re-reading coordinates or checking other fields; requires N<=128.
    pub fn is_known_dense(self) -> bool {
        self.known_nonzero_mask == coordinate_mask::<N>()
    }

    /// Count every set bit of the stored known_zero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask.count_ones()
    }

    /// Count every set bit of the stored known_nonzero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask.count_ones()
    }

    /// Count every set bit of the stored unknown_zero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask.count_ones()
    }
}

/// Snapshot of exact-set, symbolic-family, and coordinate-support facts for a Point2.
///
/// Produced masks use x/y bits 0/1. One-hot means exactly one known nonzero coordinate
/// and all others known zero; its value need not be +1. These are scheduling facts,
/// not topology, incidence, or a geometric-equality decision. Public fields may be
/// fabricated or edited, and methods do not cross-check their consistency. Recompute
/// after changing coordinates rather than transporting stale facts as evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point2Facts {
    /// Snapshot from Real::exact_set_facts; this publicly editable metadata is not revalidated by packet queries.
    pub exact: RealExactSetFacts,
    /// Union of scalar symbolic dependency-family flags; not a graph of shared coordinate/root identities.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Bit i records known-zero coordinate i in producer-created packets; public edits are not validated.
    pub known_zero_mask: u8,
    /// Bit i records known-nonzero coordinate i in producer-created packets, independently of its sign or magnitude.
    pub known_nonzero_mask: u8,
    /// Bit i records unavailable cheap zero knowledge, not mathematical zero or nonzero; public edits are not validated.
    pub unknown_zero_mask: u8,
    /// Bits for coordinates whose definitely_one query succeeds; separate from arbitrary-magnitude one-hot support.
    pub one_mask: u8,
    /// Index of the sole known-nonzero coordinate when all others are known zero and none are unknown; no unit-magnitude requirement.
    pub known_axis_index: Option<usize>,
    /// Producer sets this when every coordinate is known zero; methods trust this public flag without checking the masks.
    pub known_zero: bool,
}

impl Point2Facts {
    /// Count every set bit of the stored known_zero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask.count_ones()
    }

    /// Count every set bit of the stored known_nonzero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask.count_ones()
    }

    /// Count every set bit of the stored unknown_zero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask.count_ones()
    }

    /// Test whether the stored unknown mask is nonzero, including any out-of-dimension bits supplied by a caller.
    pub fn has_unknown_zero(self) -> bool {
        self.unknown_zero_mask != 0
    }

    /// Test only whether known_axis_index is Some; does not require coordinate value one, validate the index, or recheck the masks.
    pub fn is_one_hot(self) -> bool {
        self.known_axis_index.is_some()
    }

    /// Return the stored known_zero flag or is_one_hot; this recognizes zero/single-coordinate support, not every possible sparse pattern.
    pub fn has_sparse_support(self) -> bool {
        self.known_zero || self.is_one_hot()
    }
}

/// Snapshot of exact-set, symbolic-family, and coordinate-support facts for a Point3.
///
/// Produced masks use x/y/z bits 0/1/2. One-hot means exactly one known nonzero
/// coordinate, not a unit basis point. Two nonzero coordinates and one known zero do
/// not satisfy this packet's has_sparse_support shortcut. Public fields may be edited
/// or fabricated; methods trust them without coordinate, topology, or consistency
/// validation. A changed point requires fresh facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point3Facts {
    /// Snapshot from Real::exact_set_facts; this publicly editable metadata is not revalidated by packet queries.
    pub exact: RealExactSetFacts,
    /// Union of scalar symbolic dependency-family flags; not a graph of shared coordinate/root identities.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Bit i records known-zero coordinate i in producer-created packets; public edits are not validated.
    pub known_zero_mask: u8,
    /// Bit i records known-nonzero coordinate i in producer-created packets, independently of its sign or magnitude.
    pub known_nonzero_mask: u8,
    /// Bit i records unavailable cheap zero knowledge, not mathematical zero or nonzero; public edits are not validated.
    pub unknown_zero_mask: u8,
    /// Bits for coordinates whose definitely_one query succeeds; separate from arbitrary-magnitude one-hot support.
    pub one_mask: u8,
    /// Index of the sole known-nonzero coordinate when all others are known zero and none are unknown; no unit-magnitude requirement.
    pub known_axis_index: Option<usize>,
    /// Producer sets this when every coordinate is known zero; methods trust this public flag without checking the masks.
    pub known_zero: bool,
}

impl Point3Facts {
    /// Count every set bit of the stored known_zero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask.count_ones()
    }

    /// Count every set bit of the stored known_nonzero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask.count_ones()
    }

    /// Count every set bit of the stored unknown_zero_mask, without masking to the coordinate dimension or validating public edits.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask.count_ones()
    }

    /// Test whether the stored unknown mask is nonzero, including any out-of-dimension bits supplied by a caller.
    pub fn has_unknown_zero(self) -> bool {
        self.unknown_zero_mask != 0
    }

    /// Test only whether known_axis_index is Some; does not require coordinate value one, validate the index, or recheck the masks.
    pub fn is_one_hot(self) -> bool {
        self.known_axis_index.is_some()
    }

    /// Return the stored known_zero flag or is_one_hot; this recognizes zero/single-coordinate support, not every possible sparse pattern.
    pub fn has_sparse_support(self) -> bool {
        self.known_zero || self.is_one_hot()
    }
}

/// Cartesian 2D point storing exact Real coordinate expressions.
///
/// Construction does not assert incidence, uniqueness, or topology. Derived equality
/// uses coordinate Real equality rather than a separate certified predicate. Public
/// mutation invalidates previously detached facts. With serde enabled, derives scalar
/// field serialization/deserialization; no point-level provenance certificate is added.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Point2 {
    /// First Cartesian coordinate; publicly mutable without automatic invalidation of detached fact packets.
    pub x: Real,
    /// Second Cartesian coordinate in the same exact scalar carrier.
    pub y: Real,
}

impl Point2 {
    /// Store the supplied coordinates unchanged, without geometric validation or arithmetic.
    pub const fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }

    /// Construct exact zero coordinates; this is a point, not an empty-set sentinel.
    pub fn origin() -> Self {
        Self::new(Real::zero(), Real::zero())
    }

    /// Import each finite f64 as its exact binary value, in coordinate order.
    ///
    /// The first NaN returns Problem::NotANumber and either infinity returns
    /// Problem::Infinity; no partial point is published. This does not interpret decimal
    /// intent: 0.1_f64 is not 1/10. Use an exact parsed ratio such as "1/10" for that
    /// value. The scalar layer owns conversion and any retained primitive view cache.
    pub fn try_from_f64_array(values: [f64; 2]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_point", "constructor", "point2-from-f64-array");
        Ok(Self::new(
            Real::try_from(values[0])?,
            Real::try_from(values[1])?,
        ))
    }

    /// Import each finite f32 as its exact binary value, in coordinate order.
    ///
    /// The first NaN returns Problem::NotANumber and either infinity returns
    /// Problem::Infinity; no partial point is published. This does not interpret decimal
    /// intent: 0.1_f32 is not 1/10. Use an exact parsed ratio such as "1/10" for that
    /// value. The scalar layer owns conversion and any retained primitive view cache.
    pub fn try_from_f32_array(values: [f32; 2]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_point", "constructor", "point2-from-f32-array");
        Ok(Self::new(
            Real::try_from(values[0])?,
            Real::try_from(values[1])?,
        ))
    }

    /// Export coordinates in order through the scalar lossy conversion, requiring finite results.
    ///
    /// Returns None at the first missing or nonfinite scalar export; no partial array is
    /// returned. Rounding, underflow, and scalar approximation behavior remain possible.
    /// A finite zero does not prove an exact zero, and this API supplies no geometric
    /// error/topology guarantee or certainty receipt. Scalar caches may be read/updated.
    pub fn to_f64_array_lossy(&self) -> Option<[f64; 2]> {
        crate::trace_dispatch!("hyperlattice_point", "export", "point2-to-f64-array-lossy");
        Some([
            self.x.to_f64_lossy().filter(|value| value.is_finite())?,
            self.y.to_f64_lossy().filter(|value| value.is_finite())?,
        ])
    }

    /// Export coordinates in order through the scalar lossy conversion, requiring finite results.
    ///
    /// Returns None at the first missing or nonfinite scalar export; no partial array is
    /// returned. Rounding, underflow, and scalar approximation behavior remain possible.
    /// A finite zero does not prove an exact zero, and this API supplies no geometric
    /// error/topology guarantee or certainty receipt. Scalar caches may be read/updated.
    pub fn to_f32_array_lossy(&self) -> Option<[f32; 2]> {
        crate::trace_dispatch!("hyperlattice_point", "export", "point2-to-f32-array-lossy");
        Some([
            self.x.to_f32_lossy().filter(|value| value.is_finite())?,
            self.y.to_f32_lossy().filter(|value| value.is_finite())?,
        ])
    }

    /// Clone each coordinate into the corresponding vector lane, retaining exact values without assigning an incidence or displacement proof.
    pub fn to_vector(&self) -> Vector2 {
        Vector2::new([self.x.clone(), self.y.clone()])
    }

    /// Move the coordinates into the corresponding vector lanes, preserving their values without arithmetic or a new geometric certificate.
    pub fn into_vector(self) -> Vector2 {
        Vector2::new([self.x, self.y])
    }

    /// Delegate the affine combination `self + t * (to - self)` to the vector implementation.
    ///
    /// Coordinates remain Real expressions. No check that t is in `[0,1]` or clamping occurs, so
    /// other parameters extrapolate. Clones points/vectors for the existing arithmetic
    /// path; this does not certify that the result lies on a separately selected curve.
    pub fn lerp(&self, to: &Self, t: &Real) -> Self {
        crate::trace_dispatch!("hyperlattice_point", "method", "point2-lerp");
        Vector2::from(self.clone())
            .lerp(&Vector2::from(to.clone()), t)
            .into()
    }

    /// Return the coordinate arithmetic mean via the vector mean operation.
    ///
    /// Clones all points into an allocated vector collection before delegation. Empty
    /// input returns None; the delegated API also maps count-conversion or scalar-division
    /// failure to None rather than exposing a typed reason. For nonempty admitted input,
    /// summation follows slice order and divides by its exact integer count. No weighted
    /// mass, region centroid, or incidence interpretation is implied.
    pub fn centroid(points: &[Self]) -> Option<Self> {
        crate::trace_dispatch!("hyperlattice_point", "method", "point2-centroid");
        let vectors = points
            .iter()
            .cloned()
            .map(Vector2::from)
            .collect::<Vec<_>>();
        Vector2::mean(&vectors).map(Self::from)
    }

    /// Return the ordered coordinate sum of `points[i]*weights[i]`, without normalizing weights.
    ///
    /// Clones all points into a vector collection before checking shape in the delegated
    /// operation. Empty input or unequal lengths returns None. Negative/zero weights and
    /// sums other than one are accepted; there is no convexity or barycentric constraint.
    /// No division by total weight occurs, including when the total is zero.
    pub fn weighted_sum(points: &[Self], weights: &[Real]) -> Option<Self> {
        crate::trace_dispatch!("hyperlattice_point", "method", "point2-weighted-sum");
        let vectors = points
            .iter()
            .cloned()
            .map(Vector2::from)
            .collect::<Vec<_>>();
        Vector2::weighted_sum(&vectors, weights).map(Self::from)
    }

    /// Borrow coordinates in axis order through PointSharedScaleView::from_coordinates; unequal reduced denominators or unrecognized rational structure return None.
    pub fn shared_scale_view(&self) -> Option<PointSharedScaleView<'_, 2>> {
        PointSharedScaleView::from_coordinates([&self.x, &self.y])
    }

    /// Compute a fresh packet from the current coordinates without a geometry decision.
    ///
    /// Queries zero statuses first, then exact-set facts, symbolic dependency families,
    /// and definitely_one masks, preserving this call order. Exactly one known-nonzero
    /// lane with all others known zero sets the axis index regardless of its magnitude.
    /// Unknowns remain explicit. Publicly retained packets are snapshots, not evidence
    /// bound to later point mutations; scalar cache state may strengthen later queries.
    pub fn structural_facts(&self) -> Point2Facts {
        crate::trace_dispatch!("hyperlattice_point", "query", "point2-structural-facts");
        let coordinates = [&self.x, &self.y];
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            coordinate_zero_status_masks(coordinates);
        Point2Facts {
            exact: Real::exact_set_facts(coordinates),
            symbolic_dependencies: coordinate_symbolic_dependency_mask(coordinates),
            known_zero_mask: known_zero_mask as u8,
            known_nonzero_mask: known_nonzero_mask as u8,
            unknown_zero_mask: unknown_zero_mask as u8,
            one_mask: coordinate_one_mask(coordinates) as u8,
            known_axis_index: if known_zero_mask.count_ones() == 1
                && known_nonzero_mask.count_ones() == 1
                && unknown_zero_mask == 0
            {
                Some(single_bit_index(known_nonzero_mask))
            } else {
                None
            },
            known_zero: known_zero_mask == coordinate_mask::<2>(),
        }
    }
}

impl From<Vector2> for Point2 {
    /// Consume the vector, clone its coordinate lanes into a point, and drop the original vector; values are unchanged.
    fn from(value: Vector2) -> Self {
        Self::new(value.0[0].clone(), value.0[1].clone())
    }
}

impl From<Point2> for Vector2 {
    /// Consume the point by delegating to into_vector, moving coordinates without arithmetic.
    fn from(value: Point2) -> Self {
        value.into_vector()
    }
}

/// Cartesian 3D point storing exact Real coordinate expressions.
///
/// Construction does not validate incidence, uniqueness, or topology. Derived equality
/// uses coordinate Real equality rather than a separate certified predicate. Public
/// mutation requires recomputing detached facts. Point/vector conversions carry the
/// coordinate values without a source-incidence record or certainty receipt.
#[derive(Clone, Debug, PartialEq)]
pub struct Point3 {
    /// First Cartesian coordinate; publicly mutable without automatic invalidation of detached fact packets.
    pub x: Real,
    /// Second Cartesian coordinate in the same exact scalar carrier.
    pub y: Real,
    /// Third Cartesian coordinate in the same exact scalar carrier.
    pub z: Real,
}

impl Point3 {
    /// Store the supplied coordinates unchanged, without geometric validation or arithmetic.
    pub const fn new(x: Real, y: Real, z: Real) -> Self {
        Self { x, y, z }
    }

    /// Construct exact zero coordinates; this is a point, not an empty-set sentinel.
    pub fn origin() -> Self {
        Self::new(Real::zero(), Real::zero(), Real::zero())
    }

    /// Import each finite f64 as its exact binary value, in coordinate order.
    ///
    /// The first NaN returns Problem::NotANumber and either infinity returns
    /// Problem::Infinity; no partial point is published. This does not interpret decimal
    /// intent: 0.1_f64 is not 1/10. Use an exact parsed ratio such as "1/10" for that
    /// value. The scalar layer owns conversion and any retained primitive view cache.
    pub fn try_from_f64_array(values: [f64; 3]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_point", "constructor", "point3-from-f64-array");
        Ok(Self::new(
            Real::try_from(values[0])?,
            Real::try_from(values[1])?,
            Real::try_from(values[2])?,
        ))
    }

    /// Import each finite f32 as its exact binary value, in coordinate order.
    ///
    /// The first NaN returns Problem::NotANumber and either infinity returns
    /// Problem::Infinity; no partial point is published. This does not interpret decimal
    /// intent: 0.1_f32 is not 1/10. Use an exact parsed ratio such as "1/10" for that
    /// value. The scalar layer owns conversion and any retained primitive view cache.
    pub fn try_from_f32_array(values: [f32; 3]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_point", "constructor", "point3-from-f32-array");
        Ok(Self::new(
            Real::try_from(values[0])?,
            Real::try_from(values[1])?,
            Real::try_from(values[2])?,
        ))
    }

    /// Export coordinates in order through the scalar lossy conversion, requiring finite results.
    ///
    /// Returns None at the first missing or nonfinite scalar export; no partial array is
    /// returned. Rounding, underflow, and scalar approximation behavior remain possible.
    /// A finite zero does not prove an exact zero, and this API supplies no geometric
    /// error/topology guarantee or certainty receipt. Scalar caches may be read/updated.
    pub fn to_f64_array_lossy(&self) -> Option<[f64; 3]> {
        crate::trace_dispatch!("hyperlattice_point", "export", "point3-to-f64-array-lossy");
        Some([
            self.x.to_f64_lossy().filter(|value| value.is_finite())?,
            self.y.to_f64_lossy().filter(|value| value.is_finite())?,
            self.z.to_f64_lossy().filter(|value| value.is_finite())?,
        ])
    }

    /// Export coordinates in order through the scalar lossy conversion, requiring finite results.
    ///
    /// Returns None at the first missing or nonfinite scalar export; no partial array is
    /// returned. Rounding, underflow, and scalar approximation behavior remain possible.
    /// A finite zero does not prove an exact zero, and this API supplies no geometric
    /// error/topology guarantee or certainty receipt. Scalar caches may be read/updated.
    pub fn to_f32_array_lossy(&self) -> Option<[f32; 3]> {
        crate::trace_dispatch!("hyperlattice_point", "export", "point3-to-f32-array-lossy");
        Some([
            self.x.to_f32_lossy().filter(|value| value.is_finite())?,
            self.y.to_f32_lossy().filter(|value| value.is_finite())?,
            self.z.to_f32_lossy().filter(|value| value.is_finite())?,
        ])
    }

    /// Clone each coordinate into the corresponding vector lane, retaining exact values without assigning an incidence or displacement proof.
    pub fn to_vector(&self) -> Vector3 {
        Vector3::new([self.x.clone(), self.y.clone(), self.z.clone()])
    }

    /// Move the coordinates into the corresponding vector lanes, preserving their values without arithmetic or a new geometric certificate.
    pub fn into_vector(self) -> Vector3 {
        Vector3::new([self.x, self.y, self.z])
    }

    /// Delegate the affine combination `self + t * (to - self)` to the vector implementation.
    ///
    /// Coordinates remain Real expressions. No check that t is in `[0,1]` or clamping occurs, so
    /// other parameters extrapolate. Clones points/vectors for the existing arithmetic
    /// path; this does not certify that the result lies on a separately selected curve.
    pub fn lerp(&self, to: &Self, t: &Real) -> Self {
        crate::trace_dispatch!("hyperlattice_point", "method", "point3-lerp");
        Vector3::from(self.clone())
            .lerp(&Vector3::from(to.clone()), t)
            .into()
    }

    /// Return the coordinate arithmetic mean via the vector mean operation.
    ///
    /// Clones all points into an allocated vector collection before delegation. Empty
    /// input returns None; the delegated API also maps count-conversion or scalar-division
    /// failure to None rather than exposing a typed reason. For nonempty admitted input,
    /// summation follows slice order and divides by its exact integer count. No weighted
    /// mass, region centroid, or incidence interpretation is implied.
    pub fn centroid(points: &[Self]) -> Option<Self> {
        crate::trace_dispatch!("hyperlattice_point", "method", "point3-centroid");
        let vectors = points
            .iter()
            .cloned()
            .map(Vector3::from)
            .collect::<Vec<_>>();
        Vector3::mean(&vectors).map(Self::from)
    }

    /// Return the ordered coordinate sum of `points[i]*weights[i]`, without normalizing weights.
    ///
    /// Clones all points into a vector collection before checking shape in the delegated
    /// operation. Empty input or unequal lengths returns None. Negative/zero weights and
    /// sums other than one are accepted; there is no convexity or barycentric constraint.
    /// No division by total weight occurs, including when the total is zero.
    pub fn weighted_sum(points: &[Self], weights: &[Real]) -> Option<Self> {
        crate::trace_dispatch!("hyperlattice_point", "method", "point3-weighted-sum");
        let vectors = points
            .iter()
            .cloned()
            .map(Vector3::from)
            .collect::<Vec<_>>();
        Vector3::weighted_sum(&vectors, weights).map(Self::from)
    }

    /// Borrow coordinates in axis order through PointSharedScaleView::from_coordinates; unequal reduced denominators or unrecognized rational structure return None.
    pub fn shared_scale_view(&self) -> Option<PointSharedScaleView<'_, 3>> {
        PointSharedScaleView::from_coordinates([&self.x, &self.y, &self.z])
    }

    /// Compute a fresh packet from the current coordinates without a geometry decision.
    ///
    /// Queries zero statuses first, then exact-set facts, symbolic dependency families,
    /// and definitely_one masks, preserving this call order. Exactly one known-nonzero
    /// lane with all others known zero sets the axis index regardless of its magnitude.
    /// Unknowns remain explicit. Publicly retained packets are snapshots, not evidence
    /// bound to later point mutations; scalar cache state may strengthen later queries.
    pub fn structural_facts(&self) -> Point3Facts {
        crate::trace_dispatch!("hyperlattice_point", "query", "point3-structural-facts");
        let coordinates = [&self.x, &self.y, &self.z];
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            coordinate_zero_status_masks(coordinates);
        Point3Facts {
            exact: Real::exact_set_facts(coordinates),
            symbolic_dependencies: coordinate_symbolic_dependency_mask(coordinates),
            known_zero_mask: known_zero_mask as u8,
            known_nonzero_mask: known_nonzero_mask as u8,
            unknown_zero_mask: unknown_zero_mask as u8,
            one_mask: coordinate_one_mask(coordinates) as u8,
            known_axis_index: if known_zero_mask.count_ones() == 2
                && known_nonzero_mask.count_ones() == 1
                && unknown_zero_mask == 0
            {
                Some(single_bit_index(known_nonzero_mask))
            } else {
                None
            },
            known_zero: known_zero_mask == coordinate_mask::<3>(),
        }
    }
}

impl From<Vector3> for Point3 {
    /// Consume the vector, clone its coordinate lanes into a point, and drop the original vector; values are unchanged.
    fn from(value: Vector3) -> Self {
        Self::new(value.0[0].clone(), value.0[1].clone(), value.0[2].clone())
    }
}

impl From<Point3> for Vector3 {
    /// Consume the point by delegating to into_vector, moving coordinates without arithmetic.
    fn from(value: Point3) -> Self {
        value.into_vector()
    }
}

impl Add<Vector2> for Point2 {
    /// Translated point in the same dimension.
    type Output = Self;

    /// Translate the owned point by adding corresponding vector coordinates in axis order; move owned vector lanes.
    fn add(self, rhs: Vector2) -> Self::Output {
        let [x, y] = rhs.0;
        Self::new(self.x + x, self.y + y)
    }
}

impl Add<&Vector2> for Point2 {
    /// Translated point in the same dimension.
    type Output = Self;

    /// Translate the owned point by adding corresponding vector coordinates in axis order; clone borrowed vector lanes.
    fn add(self, rhs: &Vector2) -> Self::Output {
        Self::new(self.x + rhs.0[0].clone(), self.y + rhs.0[1].clone())
    }
}

impl Sub<Vector2> for Point2 {
    /// Translated point in the same dimension.
    type Output = Self;

    /// Translate the owned point by subtracting corresponding vector coordinates in axis order; move owned vector lanes.
    fn sub(self, rhs: Vector2) -> Self::Output {
        let [x, y] = rhs.0;
        Self::new(self.x - x, self.y - y)
    }
}

impl Sub<&Vector2> for Point2 {
    /// Translated point in the same dimension.
    type Output = Self;

    /// Translate the owned point by subtracting corresponding vector coordinates in axis order; clone borrowed vector lanes.
    fn sub(self, rhs: &Vector2) -> Self::Output {
        Self::new(self.x - rhs.0[0].clone(), self.y - rhs.0[1].clone())
    }
}

impl Sub for Point2 {
    /// Displacement vector in the same dimension.
    type Output = Vector2;

    /// Return the oriented displacement self-rhs in axis order, using the scalar ownership/borrowing form of this implementation; no norm or geometric decision is computed.
    fn sub(self, rhs: Self) -> Self::Output {
        Vector2::new([self.x - rhs.x, self.y - rhs.y])
    }
}

impl Sub<&Point2> for Point2 {
    /// Displacement vector in the same dimension.
    type Output = Vector2;

    /// Return the oriented displacement self-rhs in axis order, using the scalar ownership/borrowing form of this implementation; no norm or geometric decision is computed.
    fn sub(self, rhs: &Point2) -> Self::Output {
        Vector2::new([self.x - rhs.x.clone(), self.y - rhs.y.clone()])
    }
}

impl Sub<Point2> for &Point2 {
    /// Displacement vector in the same dimension.
    type Output = Vector2;

    /// Return the oriented displacement self-rhs in axis order, using the scalar ownership/borrowing form of this implementation; no norm or geometric decision is computed.
    fn sub(self, rhs: Point2) -> Self::Output {
        Vector2::new([self.x.clone() - rhs.x, self.y.clone() - rhs.y])
    }
}

impl Sub<&Point2> for &Point2 {
    /// Displacement vector in the same dimension.
    type Output = Vector2;

    /// Return the oriented displacement self-rhs in axis order, using the scalar ownership/borrowing form of this implementation; no norm or geometric decision is computed.
    fn sub(self, rhs: &Point2) -> Self::Output {
        Vector2::new([&self.x - &rhs.x, &self.y - &rhs.y])
    }
}

impl Add<Vector3> for Point3 {
    /// Translated point in the same dimension.
    type Output = Self;

    /// Translate the owned point by adding corresponding vector coordinates in axis order; move owned vector lanes.
    fn add(self, rhs: Vector3) -> Self::Output {
        let [x, y, z] = rhs.0;
        Self::new(self.x + x, self.y + y, self.z + z)
    }
}

impl Add<&Vector3> for Point3 {
    /// Translated point in the same dimension.
    type Output = Self;

    /// Translate the owned point by adding corresponding vector coordinates in axis order; clone borrowed vector lanes.
    fn add(self, rhs: &Vector3) -> Self::Output {
        Self::new(
            self.x + rhs.0[0].clone(),
            self.y + rhs.0[1].clone(),
            self.z + rhs.0[2].clone(),
        )
    }
}

impl Sub<Vector3> for Point3 {
    /// Translated point in the same dimension.
    type Output = Self;

    /// Translate the owned point by subtracting corresponding vector coordinates in axis order; move owned vector lanes.
    fn sub(self, rhs: Vector3) -> Self::Output {
        let [x, y, z] = rhs.0;
        Self::new(self.x - x, self.y - y, self.z - z)
    }
}

impl Sub<&Vector3> for Point3 {
    /// Translated point in the same dimension.
    type Output = Self;

    /// Translate the owned point by subtracting corresponding vector coordinates in axis order; clone borrowed vector lanes.
    fn sub(self, rhs: &Vector3) -> Self::Output {
        Self::new(
            self.x - rhs.0[0].clone(),
            self.y - rhs.0[1].clone(),
            self.z - rhs.0[2].clone(),
        )
    }
}

impl Sub for Point3 {
    /// Displacement vector in the same dimension.
    type Output = Vector3;

    /// Return the oriented displacement self-rhs in axis order, using the scalar ownership/borrowing form of this implementation; no norm or geometric decision is computed.
    fn sub(self, rhs: Self) -> Self::Output {
        Vector3::new([self.x - rhs.x, self.y - rhs.y, self.z - rhs.z])
    }
}

impl Sub<&Point3> for Point3 {
    /// Displacement vector in the same dimension.
    type Output = Vector3;

    /// Return the oriented displacement self-rhs in axis order, using the scalar ownership/borrowing form of this implementation; no norm or geometric decision is computed.
    fn sub(self, rhs: &Point3) -> Self::Output {
        Vector3::new([
            self.x - rhs.x.clone(),
            self.y - rhs.y.clone(),
            self.z - rhs.z.clone(),
        ])
    }
}

impl Sub<Point3> for &Point3 {
    /// Displacement vector in the same dimension.
    type Output = Vector3;

    /// Return the oriented displacement self-rhs in axis order, using the scalar ownership/borrowing form of this implementation; no norm or geometric decision is computed.
    fn sub(self, rhs: Point3) -> Self::Output {
        Vector3::new([
            self.x.clone() - rhs.x,
            self.y.clone() - rhs.y,
            self.z.clone() - rhs.z,
        ])
    }
}

impl Sub<&Point3> for &Point3 {
    /// Displacement vector in the same dimension.
    type Output = Vector3;

    /// Return the oriented displacement self-rhs in axis order, using the scalar ownership/borrowing form of this implementation; no norm or geometric decision is computed.
    fn sub(self, rhs: &Point3) -> Self::Output {
        Vector3::new([&self.x - &rhs.x, &self.y - &rhs.y, &self.z - &rhs.z])
    }
}

/// Return the low N bits set, using u128::MAX at N=128 and zero at N=0.
///
/// Requires N<=128, asserted only in debug builds. This is a bit-domain helper,
/// not a checked dimension validator or a scalar/geometric decision.
#[inline]
fn coordinate_mask<const N: usize>() -> u128 {
    debug_assert!(N <= u128::BITS as usize);
    if N == u128::BITS as usize {
        u128::MAX
    } else {
        (1_u128 << N) - 1
    }
}

/// Scan zero_status in coordinate order into disjoint zero/nonzero/unknown masks.
///
/// Requires N<=128 because each coordinate index is shifted into a u128. Does not
/// start sign refinement or validate the dimension before shifting. Unknown remains
/// a separate bit class; existing scalar cached knowledge may affect the snapshot.
#[inline]
fn coordinate_zero_status_masks<const N: usize>(coordinates: [&Real; N]) -> (u128, u128, u128) {
    let mut known_zero_mask = 0_u128;
    let mut known_nonzero_mask = 0_u128;
    let mut unknown_zero_mask = 0_u128;
    for (index, coordinate) in coordinates.into_iter().enumerate() {
        let bit = 1_u128 << index;
        match coordinate.zero_status() {
            ZeroKnowledge::Zero => known_zero_mask |= bit,
            ZeroKnowledge::NonZero => known_nonzero_mask |= bit,
            ZeroKnowledge::Unknown => unknown_zero_mask |= bit,
        }
    }
    (known_zero_mask, known_nonzero_mask, unknown_zero_mask)
}

/// Set bit i when coordinate i is definitely_one, in array order.
///
/// Requires N<=128 for shifts of recognized-one coordinates. False is not a proof
/// that a scalar differs from one. This mask is independent of one-hot support.
#[inline]
fn coordinate_one_mask<const N: usize>(coordinates: [&Real; N]) -> u128 {
    let mut mask = 0_u128;
    for (index, coordinate) in coordinates.into_iter().enumerate() {
        if coordinate.definitely_one() {
            mask |= 1_u128 << index;
        }
    }
    mask
}

/// Union detailed_facts symbolic dependency-family flags in coordinate order, starting from NONE.
///
/// This aggregates scalar family metadata, not correlations, shared-root identifiers,
/// or a proof that two coordinates are independent. An empty array returns NONE.
#[inline]
fn coordinate_symbolic_dependency_mask<const N: usize>(
    coordinates: [&Real; N],
) -> SymbolicDependencyMask {
    coordinates
        .into_iter()
        .fold(SymbolicDependencyMask::NONE, |mask, coordinate| {
            mask.union(coordinate.detailed_facts().symbolic.dependencies)
        })
}

/// Return trailing_zeros as the index of a required single set bit.
///
/// The one-bit precondition is debug-asserted only; current callers establish it by
/// counting the nonzero mask. This helper does not validate a general support mask.
#[inline]
fn single_bit_index(mask: u128) -> usize {
    debug_assert_eq!(mask.count_ones(), 1);
    mask.trailing_zeros() as usize
}

/// Boundary-array and collection regressions; lossy float assertions alone do not certify exact coordinate identities.
#[cfg(test)]
mod tests {
    use super::*;

    /// Lift a signed i32 test coefficient exactly into Real.
    fn real(value: i32) -> Real {
        Real::from(value)
    }

    /// Check 3D f64 and 2D f32 fixture round trips and reject a NaN in the 3D y coordinate; this does not cover every nonfinite value or lane.
    #[test]
    fn point_boundary_arrays_reject_nonfinite_values() {
        let point = Point3::try_from_f64_array([1.0, 2.0, 3.0]).unwrap();
        assert_eq!(point.to_f64_array_lossy(), Some([1.0, 2.0, 3.0]));
        assert!(Point3::try_from_f64_array([1.0, f64::NAN, 3.0]).is_err());

        let point = Point2::try_from_f32_array([1.5, 2.5]).unwrap();
        assert_eq!(point.to_f32_array_lossy(), Some([1.5, 2.5]));
    }

    /// Check one 3D midpoint/centroid and an unnormalized weighted sum against expected float exports.
    ///
    /// These assertions use to_f64_array_lossy, so they are not independent exact-value
    /// proofs or coverage of empty/mismatched collections and symbolic coordinates.
    #[test]
    fn point_centroid_weighted_sum_and_lerp_stay_in_real_coordinates() {
        let a = Point3::new(real(0), real(0), real(0));
        let b = Point3::new(real(4), real(8), real(12));
        let half = (Real::from(1_u32) / Real::from(2_u32)).unwrap();
        let mid = a.lerp(&b, &half);

        assert_eq!(mid.to_f64_array_lossy(), Some([2.0, 4.0, 6.0]));
        assert_eq!(
            Point3::centroid(&[a.clone(), b.clone()])
                .unwrap()
                .to_f64_array_lossy(),
            Some([2.0, 4.0, 6.0])
        );
        assert_eq!(
            Point3::weighted_sum(&[a, b], &[Real::from(1_u32), Real::from(2_u32)])
                .unwrap()
                .to_f64_array_lossy(),
            Some([8.0, 16.0, 24.0])
        );
    }
}
