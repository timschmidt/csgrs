use hypercurve::*;
#[allow(dead_code)]
struct PriorLocation { span_index: usize, span_range: CurveSpanRange2, local_parameter: BezierParameter2 }
fn main() { println!("{{\"prior_location_bytes\":{},\"general_location_bytes\":{},\"bezier_parameter_bytes\":{},\"general_parameter_bytes\":{}}}",std::mem::size_of::<PriorLocation>(), std::mem::size_of::<CurveLocation2>(), std::mem::size_of::<BezierParameter2>(),std::mem::size_of::<CurveParameter2>()); }
