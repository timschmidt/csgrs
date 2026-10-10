
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval, BezierParameterPolynomial,
    Curve2, CurveGeometry2, CurveParameterRange2, CurvePoint2, Point2, PredicatePolicy,
    RationalBezier2, Real, evaluate_under,
};
// The public STRICT bounds wrapper delegates to the production private
// bounds_with_policy path. The scalar enclosure helper is unchanged.
fn certified_f64_enclosure(value: &Real) -> Option<[f64; 2]> {
    if let Some(exact) = value.exact_rational_ref() { return exact.to_f64_enclosure(); }
    let [lower, upper] = value.certified_rational_interval(-64)?;
    Some([lower.to_f64_enclosure()?[0], upper.to_f64_enclosure()?[1]])
}
fn exact_contact_point_box(point: &CurvePoint2) -> Option<[f64; 4]> {
    if let Some(point) = point.coordinates() {
        let [x_low, x_high] = certified_f64_enclosure(point.x())?;
        let [y_low, y_high] = certified_f64_enclosure(point.y())?;
        return Some([x_low, x_high, y_low, y_high]);
    }
    let bounds = evaluate_under(PredicatePolicy::STRICT, || point.bounds()).certified()?.ok()?;
    Some([
        certified_f64_enclosure(bounds.min().x())?[0],
        certified_f64_enclosure(bounds.max().x())?[1],
        certified_f64_enclosure(bounds.min().y())?[0],
        certified_f64_enclosure(bounds.max().y())?[1],
    ])
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
    evaluate_under(PredicatePolicy::STRICT, || {
        let ulp = (Real::one()/Real::from(1_u64 << 52)).unwrap();
        let polynomial = BezierParameterPolynomial::try_new_power_basis(vec![
            (&ulp/Real::from(4)).unwrap()-Real::from(2), Real::one(), Real::zero(), Real::zero(), Real::zero(), Real::one(),
        ]).unwrap();
        let interval = BezierParameterInterval::try_new(-Real::one(), Real::one()).unwrap();
        let parameter = BezierAlgebraicParameter2::try_isolate(polynomial, interval).unwrap();
        let largest = Real::try_from(f64::MAX).unwrap();
        let half = (&largest/Real::from(2)).unwrap();
        let line = RationalBezier2::try_new(vec![Point2::from_values(0,0), Point2::new(half.clone(), Real::zero())], vec![Real::one();2]).unwrap();
        let range = CurveParameterRange2::try_new(BezierParameter2::Algebraic(parameter).into(), Real::from(2).into()).unwrap();
        let wide = Curve2::try_from_bezier_range(CurveGeometry2::RationalBezier(line), range).unwrap().start();
        let bounds = exact_contact_point_box(&wide).unwrap();
        assert_eq!(bounds, [-f64::MAX*0.5, f64::MAX*0.5, 0.0, 0.0]);
        let mut index = ContactPointIndex::default();
        for i in 0..64_u32 {
            let x = (-&largest * Real::from(65+i)/Real::from(128)).unwrap();
            let point = CurvePoint2::from(Point2::new(x, Real::zero()));
            index.push(usize::try_from(i).unwrap(), Some(&point), usize::try_from(i).unwrap());
        }
        index.push(64, Some(&wide), 64);
        index.merge_vertex(0, 0);
        assert_eq!(index.incidences(0), &[0]);
        let query = CurvePoint2::from(Point2::new(half, Real::zero()));
        let query_bounds = exact_contact_point_box(&query).unwrap();
        let low = ContactPointIndex::key(query_bounds[0]-index.max_width);
        let high = ContactPointIndex::key(query_bounds[1]);
        let visited: usize = index.by_x_low.range(low..=high).map(|(_,entries)| entries.len()).sum();
        let candidates = index.candidates(Some(&query)).unwrap();
        assert_eq!(candidates, vec![64]);
        println!("{{\"width_is_finite\":{},\"width_bits\":{},\"visited_indices\":{visited},\"candidates\":{candidates:?},\"expected_finite_width_bits\":{}}}",index.max_width.is_finite(),index.max_width.to_bits(),f64::MAX.to_bits());
        assert_eq!(index.max_width, f64::MAX, "the exactly representable symmetric width must not disable range pruning");
    }).certified().unwrap();
}
