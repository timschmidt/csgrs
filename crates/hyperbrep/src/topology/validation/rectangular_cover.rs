//! Exact multiplicity-one coverage after scalar boundaries have been ordered.
//!
//! Sweeping rectangle events needs linear storage in the rectangles and x cuts;
//! it does not materialize the Cartesian product of the two coordinate grids.
use crate::{BuildError, GeometryError};
use hyperreal::Problem;

#[derive(Clone, Copy)]
struct Event {
    y: usize,
    start: usize,
    end: usize,
    delta: i64,
}
#[derive(Clone, Copy, Default)]
struct Node {
    minimum: i64,
    maximum: i64,
    offset: i64,
}
fn reserved<T>(count: usize) -> Result<Vec<T>, BuildError> {
    std::alloc::Layout::array::<T>(count)
        .map_err(|_| GeometryError::Numeric(Problem::OutOfRange))?;
    let mut values = Vec::new();
    values
        .try_reserve_exact(count)
        .map_err(|_| GeometryError::Numeric(Problem::Exhausted))?;
    Ok(values)
}
struct Coverage {
    nodes: Vec<Node>,
    width: usize,
}
impl Coverage {
    fn new(width: usize) -> Result<Self, BuildError> {
        let count = width
            .checked_mul(4)
            .ok_or(GeometryError::Numeric(Problem::OutOfRange))?;
        let mut nodes = reserved(count)?;
        nodes.resize(count, Node::default());
        Ok(Self { nodes, width })
    }
    fn update(&mut self, event: Event) {
        self.add(1, 0, self.width, event);
    }
    // Halving a usize interval bounds recursion by the machine word width.
    // Validated ranges cannot enter a disjoint branch or split a leaf.
    fn add(&mut self, node: usize, left: usize, right: usize, event: Event) {
        if event.start <= left && right <= event.end {
            let cell = &mut self.nodes[node];
            cell.minimum += event.delta;
            cell.maximum += event.delta;
            cell.offset += event.delta;
            return;
        }
        let middle = left + (right - left) / 2;
        if event.start < middle {
            self.add(2 * node, left, middle, event);
        }
        if event.end > middle {
            self.add(2 * node + 1, middle, right, event);
        }
        let minimum = self.nodes[2 * node]
            .minimum
            .min(self.nodes[2 * node + 1].minimum);
        let maximum = self.nodes[2 * node]
            .maximum
            .max(self.nodes[2 * node + 1].maximum);
        self.nodes[node].minimum = self.nodes[node].offset + minimum;
        self.nodes[node].maximum = self.nodes[node].offset + maximum;
    }
    fn uniformly(&self, count: i64) -> bool {
        self.nodes[1].minimum == count && self.nodes[1].maximum == count
    }
}

/// Half-open index rectangles `[x0, x1) × [y0, y1)` cover every cell exactly once.
/// Index ordering must already correspond to strictly ordered exact coordinates.
pub(super) fn exactly_covers(
    width: usize,
    height: usize,
    rectangles: &[[usize; 4]],
) -> Result<bool, BuildError> {
    if width == 0 || height == 0 || rectangles.is_empty() {
        return Ok(false);
    }
    // Every live multiplicity and tree offset is bounded by the rectangle count.
    i64::try_from(rectangles.len()).map_err(|_| GeometryError::Numeric(Problem::OutOfRange))?;
    let count = rectangles
        .len()
        .checked_mul(2)
        .ok_or(GeometryError::Numeric(Problem::OutOfRange))?;
    let mut events = reserved(count)?;
    for &[start, end, bottom, top] in rectangles {
        if start >= end || end > width || bottom >= top || top > height {
            return Ok(false);
        }
        events.push(Event {
            y: bottom,
            start,
            end,
            delta: 1,
        });
        events.push(Event {
            y: top,
            start,
            end,
            delta: -1,
        });
    }
    events.sort_unstable_by_key(|event| event.y);
    let mut coverage = Coverage::new(width)?;
    let mut previous_y = 0;
    let mut index = 0;
    while index < events.len() {
        let y = events[index].y;
        if y > previous_y && !coverage.uniformly(1) {
            return Ok(false);
        }
        while index < events.len() && events[index].y == y {
            coverage.update(events[index]);
            index += 1;
        }
        previous_y = y;
    }
    Ok(previous_y == height && coverage.uniformly(0))
}
