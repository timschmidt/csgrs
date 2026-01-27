//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use crate::manifold::Manifold;

pub fn generate_cube() -> Result<Manifold, String> {
    let ps = [
        -0.5, -0.5, -0.5,
        -0.5, -0.5,  0.5,
        -0.5,  0.5, -0.5,
        -0.5,  0.5,  0.5,
        0.5, -0.5, -0.5,
        0.5, -0.5,  0.5,
        0.5,  0.5, -0.5,
        0.5,  0.5,  0.5
    ];

    let ts = [
        1, 0, 4, 2, 4, 0,
        1, 3, 0, 3, 1, 5,
        3, 2, 0, 3, 7, 2,
        5, 4, 6, 5, 1, 4,
        6, 4, 2, 7, 6, 2,
        7, 3, 5, 7, 5, 6
    ];
    Manifold::new(&ps, &ts)
}
