use hyperdrc::kicad::load_kicad_pcb;
use hyperreal::{Real, Rational};
fn scalar(token: &str) -> Real { Real::from(token.parse::<Rational>().unwrap()) }
fn main() {
    let board = load_kicad_pcb(std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/via.kicad_pcb"))).unwrap();
    let x = scalar("0.999");
    let y = scalar("0.03");
    let squared_radius = &x * &x + &y * &y;
    assert!(squared_radius < Real::one(), "independent exact circle membership");
    assert_eq!(board.copper.len(), 2);
    for copper in board.copper {
        let actual = copper.region.contains_xy(x.clone(), y.clone());
        println!("{}: source-circle point has squared radius 0.998901 < 1; imported region returns {:?}", copper.layer, actual);
        assert_eq!(actual, Some(true), "authored via circle must retain this interior point");
    }
}
