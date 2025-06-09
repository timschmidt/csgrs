//! This example demos converting a `CSG` to a bevy mesh

use csgrs::CSG;

const PATH: &str = "stl/bevymesh";

fn main() {
    #[cfg(not(feature = "bevymesh"))]
    panic!("This example requires the `bevymesh` feature to be enabled");

    #[cfg(feature = "bevymesh")]
    {
        let cube = CSG::<()>::cube(2.0, 2.0, 2.0, None);

        println!("{:#?}", cube.to_bevy_mesh());
    }
}
