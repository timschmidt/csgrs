use cavalier_contours::polyline::{PlineCreation, PlineSource, PlineSourceMut, Polyline};

/// Cycles all the vertex index positions forward by `n`. E.g. index 0 becomes 1, last index becomes
/// 0, etc. (only applicable to closed polylines)
pub fn cycle_start_index_forward(input: &Polyline<f64>, n: usize) -> Polyline<f64> {
    assert!(n > 0, "cycling forward by 0 just returns the same polyline");
    assert!(
        n < input.vertex_count(),
        "cycling forward by more than the polyline length is unnecessary"
    );
    assert!(
        input.is_closed(),
        "cycling vertex index positions not possible with open polyline"
    );
    Polyline::from_iter(
        input
            .iter_vertexes()
            .cycle()
            .skip(n)
            .take(input.vertex_count()),
        input.is_closed(),
    )
}

#[derive(Debug, Clone, Copy)]
pub struct ModifiedPlineState {
    pub inverted_direction: bool,
    pub cycle_position: usize,
}

impl ModifiedPlineState {
    pub fn new(inverted_direction: bool, cycle_position: usize) -> Self {
        Self {
            inverted_direction,
            cycle_position,
        }
    }
}

pub trait ModifiedPlineSetVisitor {
    fn visit(&mut self, modified_pline: Polyline<f64>, pline_state: ModifiedPlineState);
}

#[derive(Debug, Clone)]
pub struct ModifiedPlineSet<'a> {
    pub input: &'a Polyline<f64>,
    pub invert_direction: bool,
    pub cycle_index_positions: bool,
}

impl<'a> ModifiedPlineSet<'a> {
    pub fn new(
        input: &'a Polyline<f64>,
        invert_direction: bool,
        cycle_index_positions: bool,
    ) -> Self {
        Self {
            input,
            invert_direction,
            cycle_index_positions,
        }
    }

    pub fn accept_closure<F>(&self, visitor: &mut F)
    where
        F: FnMut(Polyline<f64>, ModifiedPlineState),
    {
        visitor(self.input.clone(), ModifiedPlineState::new(false, 0));
        if self.invert_direction {
            let mut pl = self.input.clone();
            pl.invert_direction_mut();
            visitor(pl, ModifiedPlineState::new(true, 0));
        }

        if self.cycle_index_positions && self.input.is_closed() {
            for i in 1..self.input.vertex_count() {
                let cycled = cycle_start_index_forward(self.input, i);
                visitor(cycled, ModifiedPlineState::new(false, i));
            }

            if self.invert_direction {
                for i in 1..self.input.vertex_count() {
                    let mut inverted = self.input.clone();
                    inverted.invert_direction_mut();
                    let cycled = cycle_start_index_forward(&inverted, i);
                    visitor(cycled, ModifiedPlineState::new(true, i));
                }
            }
        }
    }

    pub fn accept<V>(&self, visitor: &mut V)
    where
        V: ModifiedPlineSetVisitor,
    {
        self.accept_closure(&mut |a, b| visitor.visit(a, b));
    }
}
