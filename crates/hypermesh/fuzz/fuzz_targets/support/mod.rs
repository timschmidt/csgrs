pub mod coplanar {
    pub use hypermesh::coplanar::*;
}

pub mod error {
    pub use hypermesh::error::*;
}

pub mod mesh {
    pub use hypermesh::mesh::*;
}

pub mod narrow {
    pub use hypermesh::narrow::*;
}

pub mod orthogonal_surface {
    pub use hypermesh::orthogonal_surface::*;
}

pub mod provenance {
    pub use hypermesh::provenance::*;
}

pub mod validation {
    pub use hypermesh::validation::*;
}

#[allow(dead_code)]
#[path = "../../../tests/support/legacy_surface.rs"]
pub mod legacy_surface;
