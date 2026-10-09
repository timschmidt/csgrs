use clap::Parser;
fn main() {
 let path=std::env::args().nth(1).unwrap();
 let outcome=hyperdrc::run(hyperdrc::Cli::parse_from(["audit", "--kicad-pcb", &path, "--check", "layer-sanity", "--min-width", "0.0762", "--format", "text"])).unwrap();
 let rows=outcome.report.violations.iter().map(|f|serde_json::json!({"check":f.check,"layers":f.layers,"message":f.message,"island_index":f.island_index,"polygon_count":f.polygons.len(),"total_report_area":f.total_area,"locations":f.locations})).collect::<Vec<_>>();
 println!("{}",serde_json::to_string_pretty(&rows).unwrap());
}
