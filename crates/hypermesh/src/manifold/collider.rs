use nalgebra::Vector3;
use crate::bounds::BoundingBox;

fn spread_bits_3(v: u32) -> u32 {
    let mut v = v;
    v = 0xFF0000FFu32 & (v * 0x00010001u32);
    v = 0x0F00F00Fu32 & (v * 0x00000101u32);
    v = 0xC30C30C3u32 & (v * 0x00000011u32);
    v = 0x49249249u32 & (v * 0x00000005u32);
    v
}

//fn morton_code(pos: Vector3<f64>, bbox: BoundingBox) -> u32 {
//    let xyz = (pos - bounds.min) / (bbox.max - bbox.min);
//    xyz = la::min(vec3(1023.), la::max(vec3(0.), 1024. * xyz));
//    let x = spread_bits_3(xyz.x);
//    let y = spread_bits_3(xyz.y);
//    let z = spread_bits_3(xyz.z);
//    x * 4 + y * 2 + z
//}

struct Collider {

}