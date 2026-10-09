use hyperdrc::kicad::load_kicad_pcb;
fn main() {
    assert_eq!(hyperdrc::readiness::CHECK_IMPLEMENTATION_VERSION, 17);
    let path = std::path::Path::new("/tmp/drc-native-rectangles-underflow.kicad_pcb");
    for size in ["(size inf 1)", "(size 1e-400 1)", "(size 1 1e-400)"] {
        std::fs::write(path, format!("(kicad_pcb (footprint fixture (pad 1 smd rect (layers F.Cu) {size})))")).unwrap();
        let board = load_kicad_pcb(path).unwrap();
        println!("{size}: {} retained copper records", board.copper.len());
        assert_eq!(board.copper.len(), 1, "nonzero source size must not disappear through float underflow");
        assert!(board.copper[0].region.try_area().is_err());
    }
}
