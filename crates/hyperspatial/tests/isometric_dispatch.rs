#[path = "../dev/inverse_isometric.rs"]
mod inverse_isometric;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[test]
fn inverse_scalar_feeds_earth_and_local_frames_across_all_families() {
    for family in 0..22 {
        for flags in [0, 3] {
            inverse_isometric::exercise([family, (family + 7) % 22, 5, 1, 3, flags]).unwrap();
        }
    }
}
