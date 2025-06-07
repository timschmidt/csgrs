//! This example shows creating a `CSG` from an image

use csgrs::csg::CSG;
use image::{GrayImage, ImageBuffer};
use std::{fs, path::Path};

const PATH: &str = "stl/from_image";

fn main() {
    #[cfg(not(feature = "image"))]
    compile_error!("The 'image' feature is required for this example");

    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 15) from_image(img, threshold, closepaths, metadata) [requires "image" feature]
    // Make a simple 64x64 gray image with a circle in the center
    let mut img: GrayImage = ImageBuffer::new(64, 64);

    // Fill a small circle of "white" pixels in the middle
    let center = (32, 32);
    for y in 0..64 {
        for x in 0..64 {
            let dx = x as i32 - center.0;
            let dy = y as i32 - center.1;
            if dx * dx + dy * dy < 15 * 15 {
                img.put_pixel(x, y, image::Luma([255u8]));
            }
        }
    }
    let csg_img = CSG::from_image(&img, 128, true, None).center();
    write_example(&csg_img, "gray_scale_circle");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
