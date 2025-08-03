use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};

use glam::{DVec3, IVec3};

pub struct Obj {
    pub vertices: Vec<DVec3>,
    pub faces: Vec<IVec3>,
    pub aabb: (DVec3, DVec3),
    pub size: DVec3,
}

impl Obj {
    pub fn parse<P: AsRef<Path>>(path: &P) -> Self {
        println!("Parsing obj file: {}", path.as_ref().display());

        let file = File::open(path).unwrap();
        let reader = BufReader::new(file);

        let mut vertices = Vec::new();
        let mut faces = Vec::new();

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut min_z = f64::MAX;

        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut max_z = f64::MIN;

        for line in reader.lines() {
            let line = line.unwrap();
            let tokens: Vec<&str> = line.split_whitespace().collect();
            match tokens[0] {
                "v" => {
                    let x: f64 = tokens[1].parse().unwrap();
                    let y: f64 = tokens[2].parse().unwrap();
                    let z: f64 = tokens[3].parse().unwrap();

                    let vertex = DVec3::new(x, y, z);

                    min_x = min_x.min(x);
                    min_y = min_y.min(y);
                    min_z = min_z.min(z);
                    max_x = max_x.max(x);
                    max_y = max_y.max(y);
                    max_z = max_z.max(z);

                    vertices.push(vertex);
                }
                "f" => {
                    let v1: i32 = tokens[1].parse().unwrap();
                    let v2: i32 = tokens[2].parse().unwrap();
                    let v3: i32 = tokens[3].parse().unwrap();

                    let face = IVec3::new(v1, v2, v3);

                    faces.push(face);
                }
                _ => {}
            }
        }

        let aabb = (
            DVec3::new(min_x, min_y, min_z),
            DVec3::new(max_x, max_y, max_z),
        );
        let size = DVec3::new(max_x - min_x, max_y - min_y, max_z - min_z);

        println!("Parsed obj file: {}", path.as_ref().display());
        println!("Vertices: {}", vertices.len());
        println!("Faces: {}", faces.len());
        println!("Size: {size:?}");
        println!("AABB: {:?}, {:?}", aabb.0, aabb.1);

        Self {
            vertices,
            faces,
            aabb,
            size,
        }
    }
}
