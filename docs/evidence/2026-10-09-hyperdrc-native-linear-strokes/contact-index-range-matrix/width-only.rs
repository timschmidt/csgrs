
struct CurvePoint2 { bounds: [f64; 4] }
fn exact_contact_point_box(point: &CurvePoint2) -> Option<[f64; 4]> {
    assert!(point.bounds.into_iter().all(f64::is_finite));
    assert!(point.bounds[0] <= point.bounds[1] && point.bounds[2] <= point.bounds[3]);
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
                self.max_width = self.max_width.max((bounds[1] - bounds[0]).next_up());
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
    let mut values = vec![-f64::MAX, -1.0, -0.0, 0.0, 1.0, f64::MAX];
    for exponent in [-1022, -1000, -600, -54, -53, 0, 53, 600, 1000, 1023] {
        let power = f64::from_bits(((1023+exponent) as u64) << 52);
        for value in [power.next_down(), power, power.next_up()] {
            values.extend([value, -value]);
        }
    }
    for bits in [1, 2, 3, (1_u64 << 52)-1] {
        let value = f64::from_bits(bits);
        values.extend([value, -value]);
    }
    values.sort_by(f64::total_cmp);
    values.dedup_by(|a,b| a.to_bits() == b.to_bits());
    let mut intervals = Vec::new();
    let mut index = ContactPointIndex::default();
    for &low in &values {
        for &high in &values {
            if low <= high {
                let bounds = [low, high, 0.0, 0.0];
                let i = intervals.len();
                index.push(i, Some(&CurvePoint2 { bounds }), i);
                intervals.push(bounds);
            }
        }
    }
    let unboxed = intervals.len();
    index.push(unboxed, None, unboxed);
    index.merge_vertex(unboxed, 0);
    assert_eq!(index.incidences(0), &[0, unboxed]);
    assert!(index.candidates(None).is_none());
    let mut cases = 0_u64;
    let mut omissions = 0_u64;
    let mut extras = 0_u64;
    // A saturated-width shared index checks upper keys and full overlap filtering.
    for &bounds in &intervals {
        let candidates = index.candidates(Some(&CurvePoint2 { bounds })).unwrap();
        assert!(candidates.contains(&unboxed));
        assert!(candidates.windows(2).all(|w| w[0] < w[1]));
        for (i, stored) in intervals.iter().enumerate() {
            let expected = stored[0] <= bounds[1] && stored[1] >= bounds[0];
            let actual = candidates.binary_search(&i).is_ok();
            cases += 1;
            omissions += u64::from(expected && !actual);
            extras += u64::from(!expected && actual);
        }
    }
    // Each isolated index exercises its own finite or overflowing rounded width.
    let mut isolated_omissions = 0_u64;
    for &stored in &intervals {
        let mut one = ContactPointIndex::default();
        one.push(0, Some(&CurvePoint2 { bounds: stored }), 0);
        for &query in &values {
            let bounds = [query, query, 0.0, 0.0];
            let expected = stored[0] <= query && stored[1] >= query;
            let actual = one.candidates(Some(&CurvePoint2 { bounds })).unwrap().contains(&0);
            cases += 1;
            isolated_omissions += u64::from(expected && !actual);
            extras += u64::from(!expected && actual);
        }
    }
    println!("{{\"values\":{},\"intervals\":{},\"overlap_comparisons\":{cases},\"shared_omissions\":{omissions},\"isolated_omissions\":{isolated_omissions},\"extras\":{extras}}}",values.len(),intervals.len());
    assert_eq!((omissions, isolated_omissions, extras), (0,0,0));
}
