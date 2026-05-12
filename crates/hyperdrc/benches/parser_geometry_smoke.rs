use std::time::Instant;

use hyperdrc::LayerMetadata;
use hyperdrc::geometry::{circle_polygon, polygons_to_sketch, rect_polygon};
use hyperdrc::sexp;

fn main() {
    let sexp_input = r#"
        (kicad_pcb
          (net 1 "GND")
          (footprint "R" (at 10 20 0)
            (pad "1" smd rect (at 0 0) (size 1 2) (layers "F.Cu") (net 1 "GND"))))
    "#;

    let parse_elapsed = time("sexp_parse_10k", || {
        for _ in 0..10_000 {
            let _ = sexp::parse(sexp_input).expect("benchmark S-expression should parse");
        }
    });

    let geometry_elapsed = time("geometry_build_10k", || {
        for index in 0..10_000 {
            let x = index as f64 * 0.001;
            let _ = polygons_to_sketch(
                vec![
                    rect_polygon([x, x], [1.0, 2.0], 35.0),
                    circle_polygon([x + 2.0, x], 0.5, 32),
                ],
                Some(LayerMetadata {
                    name: "bench".to_string(),
                }),
            );
        }
    });

    println!(
        "parser_geometry_smoke total_ms={:.3}",
        (parse_elapsed + geometry_elapsed).as_secs_f64() * 1000.0
    );
}

fn time(name: &str, run: impl FnOnce()) -> std::time::Duration {
    let start = Instant::now();
    run();
    let elapsed = start.elapsed();
    println!("{name} ms={:.3}", elapsed.as_secs_f64() * 1000.0);
    elapsed
}
