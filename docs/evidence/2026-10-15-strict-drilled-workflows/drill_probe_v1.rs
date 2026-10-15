use csgrs::{
    GeometryContext, Real,
    solid::{self, SolidExt},
};

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let segments: usize = args[1].parse().unwrap();
    let body = solid::cuboid(12.into(), 12.into(), 4.into()).unwrap();
    let drill = solid::cylinder(2.into(), 6.into(), segments)
        .unwrap()
        .translated(6.into(), 6.into(), (-1).into())
        .unwrap();
    let context = if args[2] == "strict" {
        GeometryContext::STRICT
    } else {
        assert_eq!(args[2], "approximate");
        GeometryContext::APPROXIMATE_512
    };
    println!(
        "segments={segments} context={context:?} body_faces={} drill_faces={}",
        body.triangles.len(),
        drill.triangles.len()
    );
    let result = body.try_difference_with_context(&drill, &context);
    match result {
        Err(error) => println!("outcome={error:?}"),
        Ok(outcome) => {
            println!(
                "outcome={:?} positions={} triangles={}",
                outcome.certainty,
                outcome.value.positions.len(),
                outcome.value.triangles.len()
            );
            println!(
                "topology={:?}",
                outcome
                    .value
                    .is_closed_manifold_geometry(&context.mesh_context())
            );
            let six_volume =
                outcome
                    .value
                    .triangles
                    .iter()
                    .fold(Real::zero(), |sum, triangle| {
                        let [a, b, c] = triangle
                            .indices()
                            .map(|i| outcome.value.positions[i].to_vector());
                        sum + a.dot(&b.cross(&c))
                    });
            let expected = Real::from(3456)
                - Real::from(768) * (Real::pi() / Real::from(8)).unwrap().sin();
            if segments == 16 {
                println!(
                    "six_volume_error_zero_status={:?}",
                    (six_volume - expected).zero_status()
                );
            }
        },
    }
}
