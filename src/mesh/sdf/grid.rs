//! Grid shape definition for surface nets

/// The shape describing our discrete grid for Surface Nets:
#[derive(Clone, Copy)]
pub struct GridShape {
    pub nx: u32,
    pub ny: u32,
    pub nz: u32,
}

impl fast_surface_nets::ndshape::Shape<3> for GridShape {
    type Coord = u32;

    #[inline]
    fn as_array(&self) -> [Self::Coord; 3] {
        [self.nx, self.ny, self.nz]
    }

    fn size(&self) -> Self::Coord {
        self.nx * self.ny * self.nz
    }

    fn usize(&self) -> usize {
        (self.nx * self.ny * self.nz) as usize
    }

    fn linearize(&self, coords: [Self::Coord; 3]) -> u32 {
        let [x, y, z] = coords;
        (z * self.ny + y) * self.nx + x
    }

    fn delinearize(&self, i: u32) -> [Self::Coord; 3] {
        let x = i % self.nx;
        let yz = i / self.nx;
        let y = yz % self.ny;
        let z = yz / self.ny;
        [x, y, z]
    }
}
