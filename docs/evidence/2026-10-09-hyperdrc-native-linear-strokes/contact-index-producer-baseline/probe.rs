
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial,
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

fn selected_point(lower: Real, polynomial: &BezierParameterPolynomial) -> CurvePoint2 {
    let interval = BezierParameterInterval::try_new(lower, Real::one()).unwrap();
    let parameter = BezierAlgebraicParameter2::try_isolate(polynomial.clone(), interval).unwrap();
    let line = RationalBezier2::try_new(vec![Point2::from_values(0, 0), Point2::from_values(1, 0)], vec![Real::one(); 2]).unwrap();
    let range = CurveParameterRange2::try_new(parameter.into(), Real::from(2).into()).unwrap();
    Curve2::try_from_bezier_range(CurveGeometry2::RationalBezier(line), range).unwrap().start()
}
fn main() {
    evaluate_under(PredicatePolicy::STRICT, || {
        let ulp = (Real::one() / Real::from(1_u64 << 52)).unwrap();
        let delta = (&ulp / Real::from(4)).unwrap();
        // p(t)=t^5+t-(2-2^-54) is strictly increasing on R. Its unique
        // root is strictly between 1-2^-53 and 1, and both boxes enclose it.
        let polynomial = BezierParameterPolynomial::try_new_power_basis(vec![delta-Real::from(2), Real::one(), Real::zero(), Real::zero(), Real::zero(), Real::one()]).unwrap();
        let wide = selected_point(-Real::one()-&ulp, &polynomial);
        let narrow = selected_point(Real::one()-(&ulp/Real::from(2)).unwrap(), &polynomial);
        let wide_box = exact_contact_point_box(&wide).unwrap();
        let narrow_box = exact_contact_point_box(&narrow).unwrap();
        let equal = wide.coincides_with(&narrow).unwrap();
        println!("{{\"wide_box\":{wide_box:?},\"narrow_box\":{narrow_box:?},\"same_point_certified\":{equal}}}");
        assert!(equal);
        let mut index = ContactPointIndex::default();
        index.push(0, Some(&wide), 0);
        index.push(1, None, 1);
        index.merge_vertex(1, 0);
        assert_eq!(index.incidences(0), &[0, 1]);
        let candidates = index.candidates(Some(&narrow)).unwrap();
        println!("{{\"candidates\":{candidates:?},\"stored_width_bits\":{},\"search_low_bits\":{},\"same_point_retained\":{}}}", index.max_width.to_bits(), (narrow_box[0]-index.max_width).to_bits(), candidates.contains(&0));
        assert!(candidates.contains(&0), "a certified identical public selected point must remain a topology candidate");
    }).certified().unwrap();
}
