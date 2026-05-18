//! Field-sample aggregate facts.
//!
//! `hypervoxel` stores compact field-sample IDs, not physical laws. This
//! module builds conservative exact/certified bounds over the side-table
//! records that those IDs reference. The separation follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997:
//! preserve object references and certified bounds at the voxel layer, while
//! leaving domain interpretation to `hyperphysics` or another owning crate.

use std::{cmp::Ordering, collections::BTreeSet};

use hyperreal::{CertifiedRealOrdering, Real};

use crate::{
    AggregateCertainty, FieldSampleId, HypervoxelError, HypervoxelResult, SparseVoxelGrid,
    VoxelCell, VoxelPayload, VoxelSideTables,
};

/// Certified scalar interval accumulated from field-sample records.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedFieldInterval {
    /// Certified lower bound.
    pub lower: Real,
    /// Certified upper bound.
    pub upper: Real,
}

/// Certified scalar ball derived from an interval.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedFieldBall {
    /// Exact ball center.
    pub center: Real,
    /// Exact non-negative radius.
    pub radius: Real,
}

/// Certified vector interval with per-component scalar bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedVectorInterval {
    /// Component intervals.
    pub components: Vec<CertifiedFieldInterval>,
}

/// Certified tensor interval with row-major component bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedTensorInterval {
    /// Number of tensor rows.
    pub rows: usize,
    /// Number of tensor columns.
    pub cols: usize,
    /// Row-major component intervals.
    pub components: Vec<CertifiedFieldInterval>,
}

impl CertifiedFieldInterval {
    /// Returns the exact midpoint/radius ball enclosing this interval.
    ///
    /// Interval-to-ball conversion is a certified enclosure transform: it does
    /// not interpret the scalar as a physical law, and it never samples the
    /// value as a primitive float.
    pub fn enclosing_ball(&self) -> CertifiedFieldBall {
        let half = Real::from(hyperreal::Rational::fraction(1, 2).unwrap());
        let center = (&self.lower + &self.upper) * &half;
        let radius = (&self.upper - &self.lower) * &half;
        CertifiedFieldBall { center, radius }
    }
}

/// Aggregate facts over field samples referenced by voxel cells.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldAggregateFacts {
    /// Number of cells that referenced field samples.
    pub sample_cell_count: usize,
    /// Whether at least one field-sample cell was observed.
    ///
    /// Zero field samples is a precise absence report, not certified field
    /// evidence. This follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997: an exact/certified object fact
    /// must be backed by object-level evidence rather than by an empty
    /// iterator convention.
    pub has_field_samples: bool,
    /// Distinct sample IDs observed.
    pub sample_ids: BTreeSet<FieldSampleId>,
    /// Union interval when every referenced sample has certified bounds.
    pub interval: Option<CertifiedFieldInterval>,
    /// Number of referenced sample IDs without side-table records.
    pub missing_records: usize,
    /// Number of records with absent lower or upper bounds.
    pub missing_bounds: usize,
    /// Certainty of the aggregate interval.
    pub certainty: AggregateCertainty,
    /// Whether certified field bounds are complete enough for downstream use.
    ///
    /// The voxel layer does not assign physics meaning to field samples. This
    /// flag only says that every referenced side-table record supplied a
    /// certified lower/upper scalar interval, matching Yap's separation of
    /// exact/certified facts from unknown object data.
    pub certified_field_bounds_ready: bool,
}

/// Aggregate facts over vector/tensor field envelopes.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldEnvelopeFacts {
    /// Number of vector envelopes observed.
    pub vector_count: usize,
    /// Number of tensor envelopes observed.
    pub tensor_count: usize,
    /// Total vector and tensor envelopes observed.
    pub envelope_count: usize,
    /// Whether at least one vector or tensor envelope was observed.
    ///
    /// An empty envelope list is an exact absence report, not evidence that a
    /// vector/tensor field has a certified envelope. Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997, treats
    /// exactness as a property of represented objects and predicates; this
    /// flag keeps an empty iterator from being promoted into object evidence.
    pub has_envelopes: bool,
    /// Union of vector component intervals when dimensions are consistent.
    pub vector_interval: Option<CertifiedVectorInterval>,
    /// Union of tensor component intervals when shapes are consistent.
    pub tensor_interval: Option<CertifiedTensorInterval>,
    /// Number of envelopes with incompatible vector dimensions or tensor shapes.
    pub incompatible_shapes: usize,
    /// Certainty of the envelope facts.
    pub certainty: AggregateCertainty,
    /// Whether every observed vector/tensor envelope contributed compatible
    /// certified component intervals and at least one envelope was present.
    ///
    /// In Yap's EGC terms, incompatible envelope shapes remain explicit
    /// unknowns rather than being coerced into a lossy common layout, while an
    /// empty set remains unknown for envelope-certification purposes.
    pub certified_envelope_ready: bool,
}

/// Field-sample references observed in a sparse grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FieldSampleQuery {
    /// Distinct field samples referenced by cells.
    pub referenced: BTreeSet<FieldSampleId>,
    /// Referenced field samples missing from the side table.
    pub missing_records: BTreeSet<FieldSampleId>,
    /// Referenced field samples without complete lower/upper bounds.
    pub missing_bounds: BTreeSet<FieldSampleId>,
}

impl FieldEnvelopeFacts {
    /// Builds aggregate facts over vector and tensor envelopes.
    pub fn from_envelopes<'a>(
        vectors: impl IntoIterator<Item = &'a CertifiedVectorInterval>,
        tensors: impl IntoIterator<Item = &'a CertifiedTensorInterval>,
    ) -> HypervoxelResult<Self> {
        let mut vector_count = 0_usize;
        let mut tensor_count = 0_usize;
        let mut incompatible_shapes = 0_usize;
        let mut vector_interval = None::<CertifiedVectorInterval>;
        let mut tensor_interval = None::<CertifiedTensorInterval>;

        for vector in vectors {
            vector_count += 1;
            match &mut vector_interval {
                Some(current) if current.components.len() == vector.components.len() => {
                    merge_components(&mut current.components, &vector.components)?;
                }
                Some(_) => incompatible_shapes += 1,
                None => vector_interval = Some(vector.clone()),
            }
        }

        for tensor in tensors {
            tensor_count += 1;
            match &mut tensor_interval {
                Some(current)
                    if current.rows == tensor.rows
                        && current.cols == tensor.cols
                        && current.components.len() == tensor.components.len() =>
                {
                    merge_components(&mut current.components, &tensor.components)?;
                }
                Some(_) => incompatible_shapes += 1,
                None => tensor_interval = Some(tensor.clone()),
            }
        }

        let envelope_count = vector_count + tensor_count;
        let has_envelopes = envelope_count > 0;
        let certified_envelope_ready = has_envelopes && incompatible_shapes == 0;

        Ok(Self {
            vector_count,
            tensor_count,
            envelope_count,
            has_envelopes,
            vector_interval,
            tensor_interval,
            incompatible_shapes,
            certainty: if certified_envelope_ready {
                AggregateCertainty::Certified
            } else {
                AggregateCertainty::Unknown
            },
            certified_envelope_ready,
        })
    }
}

impl FieldSampleQuery {
    /// Returns whether every referenced field sample has a complete side-table record.
    pub fn is_fully_resolved(&self) -> bool {
        self.missing_records.is_empty() && self.missing_bounds.is_empty()
    }
}

impl FieldAggregateFacts {
    /// Builds field aggregate facts from cells and side tables.
    pub fn from_cells<'a>(
        cells: impl IntoIterator<Item = &'a VoxelCell>,
        side_tables: &VoxelSideTables,
    ) -> HypervoxelResult<Self> {
        let mut sample_cell_count = 0_usize;
        let mut sample_ids = BTreeSet::new();
        let mut interval = None::<CertifiedFieldInterval>;
        let mut missing_records = 0_usize;
        let mut missing_bounds = 0_usize;

        for cell in cells {
            let VoxelPayload::FieldSample(sample_id) = cell.payload else {
                continue;
            };
            sample_cell_count += 1;
            sample_ids.insert(sample_id);
            let Some(record) = side_tables.field_sample(sample_id) else {
                missing_records += 1;
                continue;
            };
            let (Some(lower), Some(upper)) = (&record.lower, &record.upper) else {
                missing_bounds += 1;
                continue;
            };
            if certified_cmp(lower, upper, "field interval")? == Ordering::Greater {
                return Err(HypervoxelError::UnknownScalarOrdering {
                    field: "inverted field interval",
                });
            }

            match &mut interval {
                Some(current) => {
                    if certified_cmp(lower, &current.lower, "field lower")? == Ordering::Less {
                        current.lower = lower.clone();
                    }
                    if certified_cmp(upper, &current.upper, "field upper")? == Ordering::Greater {
                        current.upper = upper.clone();
                    }
                }
                None => {
                    interval = Some(CertifiedFieldInterval {
                        lower: lower.clone(),
                        upper: upper.clone(),
                    });
                }
            }
        }

        let has_field_samples = sample_cell_count > 0;
        let certified_field_bounds_ready =
            has_field_samples && missing_records == 0 && missing_bounds == 0;
        let certainty = if !certified_field_bounds_ready {
            AggregateCertainty::Unknown
        } else {
            AggregateCertainty::Certified
        };

        Ok(Self {
            sample_cell_count,
            has_field_samples,
            sample_ids,
            interval,
            missing_records,
            missing_bounds,
            certainty,
            certified_field_bounds_ready,
        })
    }

    /// Builds field aggregate facts over all explicitly stored cells in a sparse grid.
    pub fn from_grid(
        grid: &SparseVoxelGrid,
        side_tables: &VoxelSideTables,
    ) -> HypervoxelResult<Self> {
        Self::from_cells(grid.iter().map(|(_, cell)| cell), side_tables)
    }
}

/// Queries field-sample references over explicitly stored sparse cells.
pub fn query_field_samples(
    grid: &SparseVoxelGrid,
    side_tables: &VoxelSideTables,
) -> FieldSampleQuery {
    let mut referenced = BTreeSet::new();
    let mut missing_records = BTreeSet::new();
    let mut missing_bounds = BTreeSet::new();
    for (_, cell) in grid.iter() {
        let VoxelPayload::FieldSample(sample) = cell.payload else {
            continue;
        };
        referenced.insert(sample);
        match side_tables.field_sample(sample) {
            Some(record) if record.lower.is_some() && record.upper.is_some() => {}
            Some(_) => {
                missing_bounds.insert(sample);
            }
            None => {
                missing_records.insert(sample);
            }
        }
    }
    FieldSampleQuery {
        referenced,
        missing_records,
        missing_bounds,
    }
}

fn certified_cmp(left: &Real, right: &Real, field: &'static str) -> HypervoxelResult<Ordering> {
    match left.certified_cmp_until(right, -128) {
        CertifiedRealOrdering::Known { ordering, .. } => Ok(ordering),
        CertifiedRealOrdering::Unknown { .. } => {
            Err(HypervoxelError::UnknownScalarOrdering { field })
        }
    }
}

fn merge_components(
    current: &mut [CertifiedFieldInterval],
    next: &[CertifiedFieldInterval],
) -> HypervoxelResult<()> {
    for (current, next) in current.iter_mut().zip(next) {
        if certified_cmp(&next.lower, &current.lower, "vector/tensor lower")? == Ordering::Less {
            current.lower = next.lower.clone();
        }
        if certified_cmp(&next.upper, &current.upper, "vector/tensor upper")? == Ordering::Greater {
            current.upper = next.upper.clone();
        }
    }
    Ok(())
}
