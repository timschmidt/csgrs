use nalgebra::Vector3;

pub struct BoundingBox {
    pub min: Vector3<f64>,
    pub max: Vector3<f64>,
}

impl BoundingBox {
    pub fn new(p1: Vector3<f64>, p2: Vector3<f64>) -> Self {
        BoundingBox {
            min: Vector3::new(p1.x.min(p2.x), p1.y.min(p2.y), p1.z.min(p2.z)),
            max: Vector3::new(p1.x.max(p2.x), p1.y.max(p2.y), p1.z.max(p2.z)),
        }
    }
}



