use hyperdrc::geometry::polygon_to_profile;
use hyperdrc::kicad::load_kicad_pcb;
use hyperdrc::PcbRegionExt;
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let board = load_kicad_pcb(std::path::Path::new(&path)).unwrap();
    let mut entries = Vec::new();
    for (i, feature) in board.copper.iter().enumerate().filter(|(_,f)|f.layer == "F.Cu") {
        for polygon in feature.region.try_to_multipolygon().unwrap().0 {
            let area = polygon.try_area().unwrap_or_else(|e| panic!("Feature {i} area: {e}"));
            let bounds = polygon.try_bounds().unwrap().unwrap().map(|v|v.to_f64_lossy().unwrap());
            entries.push((i,polygon,area.to_f64_lossy().unwrap(),bounds));
        }
    }
    println!("Individual areas passed: {}",entries.len());
    for (li,(left,lp,la,lb)) in entries.iter().enumerate() {
        for (right,rp,ra,rb) in entries.iter().skip(li+1) {
            if lb[0] > rb[2] || rb[0] > lb[2] || lb[1] > rb[3] || rb[1] > lb[3] { continue; }
            if (la-ra).abs() > 1e-6 * la.abs().max(ra.abs()).max(1.) { continue; }
            let l = polygon_to_profile(lp.clone(),None);
            let r = polygon_to_profile(rp.clone(),None);
            let overlap=l.try_intersection(&r).unwrap_or_else(|e| panic!("Intersection {left}/{right}: {e}"));
            if let Err(e) = overlap.try_area() {
                println!("FAIL {left}/{right}: {e}; areas {la}/{ra}; bounds {lb:?}/{rb:?}");
                for i in [left,right] {
                    let f=&board.copper[*i];
                    println!("Feature {i}: {:?} {:?} {:?}",f.kind,f.net,f.location);
                    println!("Native: {:?}",f.region.clone().try_into_native().unwrap().native_contours_fast_path());
                }
                println!("Overlap: {:?}",overlap.try_into_native().unwrap().native_contours_fast_path());
                return;
            }
        }
    }
    println!("No unavailable area found");
}
