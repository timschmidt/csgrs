use glam::Vec3;

pub const CUBE_VERTS: [Vec3; 8] = [
    Vec3::new(0.0, 1.0, 0.0),
    Vec3::new(1.0, 1.0, 0.0),
    Vec3::new(1.0, 1.0, 1.0),
    Vec3::new(0.0, 1.0, 1.0),
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 1.0),
    Vec3::new(0.0, 0.0, 1.0),
];

pub const VEC_RIGHT: Vec3 = Vec3::X;
pub const VEC_LEFT: Vec3 = Vec3::NEG_X;
pub const VEC_UP: Vec3 = Vec3::Y;
pub const VEC_DOWN: Vec3 = Vec3::NEG_Y;
pub const VEC_FORWARD: Vec3 = Vec3::NEG_Z;
pub const VEC_BACK: Vec3 = Vec3::Z;

#[derive(Default)]
pub struct MeshData {
    pub vertices: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub indices: Vec<u32>,
}

impl MeshData {
    pub fn clear(&mut self) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("MeshData::clear");

        self.vertices.clear();
        self.normals.clear();
        self.indices.clear();
    }
}

#[inline(always)]
pub fn add_quad(mesh_data: &mut MeshData, quad: [Vec3; 4], normal: &Vec3) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("add_quad");

    let index = mesh_data.vertices.len() as u32;

    mesh_data.vertices.extend(quad);
    mesh_data.normals.extend([normal, normal, normal, normal]);
    mesh_data
        .indices
        .extend([index + 2, index + 1, index, index + 3, index, index + 1]);
}
