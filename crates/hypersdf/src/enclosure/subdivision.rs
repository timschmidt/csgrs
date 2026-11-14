//! Bounded exact Cartesian covers with source-bound whole-cell evidence.
use super::{ExpressionBudget, ExpressionError, SdfCellEnclosure, SdfProgram, SdfProgramError};
use crate::SdfCellLocation;
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice::{BoxN, IntervalLatticeError};

/// Resolution request and separate retained-tree capacity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SdfSubdivisionOptions {
    /// Native enclosure grid spacing exponent; not a cell-width guarantee.
    pub precision: i32,
    /// Maximum number of bisections from root to leaf. Root depth is zero.
    pub max_depth: u32,
    /// Maximum total records, including internal nodes and the root.
    pub max_cells: usize,
}
impl Default for SdfSubdivisionOptions {
    fn default() -> Self {
        Self {
            precision: -48,
            max_depth: 8,
            max_cells: 1023,
        }
    }
}

/// Why a terminal nonempty cell still lacks a definite whole-cell sign.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfUnresolvedCell {
    /// The requested spatial depth was reached; no arithmetic limit is implied.
    DepthLimit,
    /// Every coordinate is a singleton, so spatial bisection cannot progress.
    Singleton,
}

/// An immutable visited cell and its original enclosure, including internal nodes.
#[derive(Clone, Debug)]
pub struct SdfSubdivisionCell {
    enclosure: SdfCellEnclosure,
    depth: u32,
    split: Option<(usize, [usize; 2])>,
    unresolved: Option<SdfUnresolvedCell>,
}
impl SdfSubdivisionCell {
    /// Original whole-cell range, source and qualification.
    pub fn enclosure(&self) -> &SdfCellEnclosure {
        &self.enclosure
    }
    /// Number of native bisections from the root.
    pub fn depth(&self) -> u32 {
        self.depth
    }
    /// Split coordinate and indices of the closed left/right children, if any.
    pub fn split(&self) -> Option<(usize, [usize; 2])> {
        self.split
    }
    /// Terminal unresolved reason. Internal and definitely classified cells have none.
    pub fn unresolved(&self) -> Option<SdfUnresolvedCell> {
        self.unresolved
    }
}

/// A complete closed cover of the query, which may contain unresolved leaves.
///
/// Root is record zero. Children union exactly to their parent and overlap on
/// the midpoint facet; this is not a disjoint-volume partition. Every record
/// retains its source-bound range. An empty query has one empty root record.
/// Resource/domain failure returns an error, never a provisional instance.
#[derive(Clone, Debug)]
pub struct SdfSubdivision {
    options: SdfSubdivisionOptions,
    cells: Vec<SdfSubdivisionCell>,
}
impl SdfSubdivision {
    /// The original resolution and capacity request.
    pub fn options(&self) -> SdfSubdivisionOptions {
        self.options
    }
    /// Immutable tree records, with root at zero and explicit child indices.
    pub fn cells(&self) -> &[SdfSubdivisionCell] {
        &self.cells
    }
    /// Terminal cells and their stable indices in this report.
    pub fn leaves(&self) -> impl Iterator<Item = (usize, &SdfSubdivisionCell)> {
        self.cells
            .iter()
            .enumerate()
            .filter(|(_, cell)| cell.split.is_none())
    }
    /// Whole-query classification from all covering leaves. Mixed or unresolved
    /// signs give Unknown. None means the query is empty. Boundary means the
    /// field is zero throughout the query, not a surface-topology certificate.
    pub fn location(&self) -> Option<SdfCellLocation> {
        let mut leaves = self.leaves();
        let first = leaves
            .next()
            .expect("a finite tree has a leaf")
            .1
            .enclosure
            .location();
        if leaves.all(|(_, cell)| cell.enclosure.location() == first) {
            first
        } else {
            Some(SdfCellLocation::Unknown)
        }
    }
}

impl SdfProgram {
    /// Refine only cells whose native natural image has an unresolved sign.
    ///
    /// Native exact widest-axis selection breaks ties by lowest index. Closed
    /// midpoint children are evaluated left then right; unresolved nodes are expanded
    /// left first, retaining their parent evidence.
    /// Depth-limited and singleton unresolved leaves remain explicit. Every
    /// native domain, arithmetic, capacity or cancellation error aborts the
    /// entire attempt; no partly evaluated cover is returned or cached.
    ///
    /// Work shares the supplied graph and arithmetic budgets across all cells.
    /// Graph size checks count the source definition, two node-sized evaluator
    /// arrays, tree records, pending indices and six fixed working records.
    /// Native interval arithmetic has its own size/work accounting. These counts
    /// are not heap-byte bounds; scalar refinement, payload and destruction keep
    /// the native evaluator's exclusions. Cancellation cannot interrupt those
    /// operations. max_cells independently bounds the retained tree length.
    pub fn subdivide(
        &self,
        cell: [DyadicInterval; 3],
        options: SdfSubdivisionOptions,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<SdfSubdivision, SdfProgramError> {
        graph.charge(0)?;
        capacity(options.max_cells, 1)?;
        self.subdivision_entries(1, 1, graph)?;
        graph.charge(2)?;
        let root = self.image(cell, options.precision, graph, arithmetic)?;
        let mut cells = vec![record(root, 0)];
        let mut pending = vec![0];
        while let Some(index) = pending.pop() {
            graph.charge(1)?;
            if cells[index].enclosure.location() != Some(SdfCellLocation::Unknown) {
                continue;
            }
            if cells[index]
                .enclosure
                .cell()
                .iter()
                .all(DyadicInterval::is_singleton)
            {
                cells[index].unresolved = Some(SdfUnresolvedCell::Singleton);
                continue;
            }
            let depth = cells[index].depth;
            if depth == options.max_depth {
                cells[index].unresolved = Some(SdfUnresolvedCell::DepthLimit);
                continue;
            }
            let count = cells
                .len()
                .checked_add(2)
                .ok_or(ExpressionError::SizeExhausted)?;
            capacity(options.max_cells, count)?;
            let pending_count = pending
                .len()
                .checked_add(2)
                .ok_or(ExpressionError::SizeExhausted)?;
            self.subdivision_entries(count, pending_count, graph)?;
            graph.charge(3)?;
            let [a, b, c] = cells[index].enclosure.cell();
            let domain = BoxN::new([
                a.checked_pow(1, arithmetic)
                    .map_err(IntervalLatticeError::from)?,
                b.checked_pow(1, arithmetic)
                    .map_err(IntervalLatticeError::from)?,
                c.checked_pow(1, arithmetic)
                    .map_err(IntervalLatticeError::from)?,
            ])?;
            graph.charge(3)?;
            let axis = domain
                .widest_axis(arithmetic)?
                .expect("unknown cell is nonempty");
            graph.charge(2)?;
            let children = domain
                .bisect_axis(axis, arithmetic)?
                .expect("widest lane is nonsingleton");
            let indices = [cells.len(), cells.len() + 1];
            for child in children {
                graph.charge(1)?;
                let enclosure = self.image(
                    child.into_lanes().expect("nonempty child"),
                    options.precision,
                    graph,
                    arithmetic,
                )?;
                cells.push(record(enclosure, depth + 1));
            }
            graph.charge(2)?;
            cells[index].split = Some((axis, indices));
            pending.push(indices[1]);
            pending.push(indices[0]);
        }
        graph.charge(1)?;
        let result = SdfSubdivision { options, cells };
        graph.charge(0)?;
        Ok(result)
    }

    fn subdivision_entries(
        &self,
        cells: usize,
        pending: usize,
        graph: &ExpressionBudget,
    ) -> Result<(), ExpressionError> {
        let definition = self.program().definition();
        graph.check_entries(&[
            definition.variables.len(),
            definition.nodes.len(),
            definition.outputs.len(),
            definition.nodes.len(),
            definition.nodes.len(),
            cells,
            pending,
            6,
        ])
    }
}
fn record(enclosure: SdfCellEnclosure, depth: u32) -> SdfSubdivisionCell {
    SdfSubdivisionCell {
        enclosure,
        depth,
        split: None,
        unresolved: None,
    }
}
fn capacity(limit: usize, required: usize) -> Result<(), SdfProgramError> {
    if required > limit {
        Err(SdfProgramError::SubdivisionCapacity { limit, required })
    } else {
        Ok(())
    }
}
