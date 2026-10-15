use hyperbrep::{Point3, Real, builder};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bodies = builder::cuboid(
        Point3::origin(),
        Point3::new(Real::from(2), Real::from(3), Real::from(5)),
    )?;
    let model = bodies.model();
    let volume = bodies.bodies().next().expect("one cuboid").volume()?;
    println!("faces: {}, exact volume: {volume}", model.counts().faces);
    assert_eq!(model.counts().faces, 6);
    Ok(())
}
