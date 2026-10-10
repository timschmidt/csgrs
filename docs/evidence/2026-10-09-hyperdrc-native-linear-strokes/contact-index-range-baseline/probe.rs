
// Adapter supplies valid conservative enclosures for known exact binary64
// rational points. The complete index implementation below is unmodified.
struct CurvePoint2 { exact: [f64; 2], bounds: [f64; 4] }
fn exact_contact_point_box(point: &CurvePoint2) -> Option<[f64; 4]> {
    let [lx, hx, ly, hy] = point.bounds;
    assert!(point.bounds.into_iter().chain(point.exact).all(f64::is_finite));
    assert!(lx <= point.exact[0] && point.exact[0] <= hx);
    assert!(ly <= point.exact[1] && point.exact[1] <= hy);
    Some(point.bounds)
}
#[derive(Default)]
struct ContactPointIndex {
    by_x_low: std::collections::BTreeMap<u64, Vec<usize>>,
    boxes: Vec<Option<[f64; 4]>>,
    unboxed: Vec<usize>,
    max_width: f64,
    vertex_boxes: Vec<Option<[f64; 4]>>,
    vertex_incidences: Vec<Vec<usize>>,
}

impl ContactPointIndex {
    /// Order-preserving key for finite f64 values.
    fn key(value: f64) -> u64 {
        let bits = value.to_bits();
        if bits >> 63 == 0 {
            bits | (1 << 63)
        } else {
            !bits
        }
    }

    fn ensure_vertex(&mut self, vertex: usize) {
        if self.vertex_boxes.len() <= vertex {
            self.vertex_boxes.resize(vertex + 1, None);
            self.vertex_incidences.resize_with(vertex + 1, Vec::new);
        }
    }

    fn push(&mut self, index: usize, point: Option<&CurvePoint2>, vertex: usize) {
        debug_assert_eq!(index, self.boxes.len());
        self.ensure_vertex(vertex);
        // A contact without its own point shares its vertex representative.
        let bounds = point
            .and_then(exact_contact_point_box)
            .or(self.vertex_boxes[vertex]);
        self.boxes.push(bounds);
        self.vertex_incidences[vertex].push(index);
        match bounds {
            Some(bounds) => {
                self.by_x_low
                    .entry(Self::key(bounds[0]))
                    .or_default()
                    .push(index);
                self.max_width = self.max_width.max(bounds[1] - bounds[0]);
                if self.vertex_boxes[vertex].is_none() {
                    self.vertex_boxes[vertex] = Some(bounds);
                }
            }
            None => self.unboxed.push(index),
        }
    }

    fn merge_vertex(&mut self, from: usize, to: usize) {
        self.ensure_vertex(from.max(to));
        let moved = std::mem::take(&mut self.vertex_incidences[from]);
        self.vertex_incidences[to].extend(moved);
        if self.vertex_boxes[to].is_none() {
            self.vertex_boxes[to] = self.vertex_boxes[from];
        }
    }

    fn incidences(&self, vertex: usize) -> &[usize] {
        self.vertex_incidences
            .get(vertex)
            .map_or(&[], Vec::as_slice)
    }

    /// Ascending candidate indices, or `None` when every contact must be
    /// examined because the new contact has no exact box.
    fn candidates(&self, point: Option<&CurvePoint2>) -> Option<Vec<usize>> {
        let bounds = point.and_then(exact_contact_point_box)?;
        let low = Self::key(bounds[0] - self.max_width);
        let high = Self::key(bounds[1]);
        let mut candidates = self.unboxed.clone();
        for indices in self.by_x_low.range(low..=high).map(|(_, indices)| indices) {
            for &index in indices {
                let other = self.boxes[index].expect("indexed contacts have boxes");
                if other[1] >= bounds[0]
                    && other[0] <= bounds[1]
                    && other[3] >= bounds[2]
                    && other[2] <= bounds[3]
                {
                    candidates.push(index);
                }
            }
        }
        candidates.sort_unstable();
        Some(candidates)
    }
}

fn main() {
    let mut failures = 0;
    let mut cases = 0;
    let mut check = |name: String, low: f64, high: f64, query: f64, overlaps: bool| {
        let stored = CurvePoint2 { exact: [high, 0.0], bounds: [low, high, 0.0, 0.0] };
        let query = CurvePoint2 { exact: [query, 0.0], bounds: [query, query, 0.0, 0.0] };
        let mut index = ContactPointIndex::default();
        index.push(0, Some(&stored), 0);
        index.push(1, None, 1);
        index.merge_vertex(1, 0);
        assert_eq!(index.incidences(0), &[0, 1]);
        let candidates = index.candidates(Some(&query)).unwrap();
        assert!(candidates.contains(&1), "unboxed contacts remain candidates");
        let retained = candidates.contains(&0);
        assert_eq!(overlaps, low <= query.exact[0] && query.exact[0] <= high);
        if overlaps { assert_eq!(stored.exact, query.exact); }
        failures += usize::from(retained != overlaps);
        cases += 1;
        println!("{{\"case\":\"{name}\",\"box_low_bits\":{},\"box_high_bits\":{},\"query_bits\":{},\"stored_width_bits\":{},\"search_low_bits\":{},\"expected_overlap\":{overlaps},\"retained\":{retained},\"candidates\":{candidates:?}}}", low.to_bits(), high.to_bits(), query.exact[0].to_bits(), index.max_width.to_bits(), (query.exact[0]-index.max_width).to_bits());
    };
    let fraction = f64::from_bits((1023_u64-54) << 52);
    for exponent in [-1000, -600, -53, 0, 53, 600, 1000] {
        let scale = f64::from_bits(((1023+exponent) as u64) << 52);
        check(format!("cross_zero_width_{exponent}"), -scale*fraction, scale, scale, true);
    }
    check("exact_point_control".into(), 1.0, 1.0, 1.0, true);
    check("wide_control".into(), -1.0, 1.0, 1.0, true);
    check("negative_endpoint_control".into(), -2.0, -1.0, -1.0, true);
    check("strictly_disjoint_control".into(), -fraction, 1.0, 1.0_f64.next_up(), false);
    assert_eq!(failures, 0, "{failures} of {cases} exact contact-box witnesses were contradicted");
}
