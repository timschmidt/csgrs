//! Deterministically generated serialized meshes shared by benchmark consumers.
//!
//! Small integer voxel boundaries are exported as OBJ under the runtime target
//! directory and reused when bytes match. The foam's diagonal edge contacts are
//! filled before boundary extraction; this changes the raw ternary-cell set.
//! Fixed counts and native geometric edge checks constrain the published fixtures,
//! separately from deterministic byte order or a general manifold/embedding proof.

use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::{Path, PathBuf},
};

use hyperlattice::{Point3, Real};
use hypermesh::{Triangle, TriangleMesh};

/// Nonnegative integer grid coordinate, used for occupied unit cells and lattice corners.
type Cell = [usize; 3];

/// Builds, checks and installs the slotted 31-by-31-by-6 labyrinth fixture.
///
/// Requires 4,012 shared positions, 8,020 triangles and native closed-manifold
/// geometry. Returns the retained OBJ path; failed checks/export/I/O panic.
pub fn concave_path() -> PathBuf {
    let mesh = voxel_boundary(concave_cells());
    assert_eq!(mesh.positions.len(), 4_012);
    assert_eq!(mesh.triangles.len(), 8_020);
    assert!(mesh.is_closed_manifold());
    install(
        "deterministic_concave_labyrinth_31x31x6.obj",
        &mesh,
        "deterministic_concave_labyrinth_31x31x6",
    )
}

/// Builds and installs the level-three ternary foam after filling diagonal edge contacts.
///
/// Requires 15,232 positions, 36,096 triangles and native closed-manifold geometry.
/// The installed fixture represents the modified cell set, not the raw fractal
/// selection. Returns its retained OBJ path; checks/export/I/O may panic.
pub fn sierpinski_foam_path() -> PathBuf {
    let mesh = voxel_boundary(manifoldize_voxels(sierpinski_foam_cells(3)));
    assert_eq!(mesh.positions.len(), 15_232);
    assert_eq!(mesh.triangles.len(), 36_096);
    assert!(mesh.is_closed_manifold());
    install("sierpinski_foam_level3.obj", &mesh, "sierpinski_foam_level3")
}

/// Selects a 31-by-31 footprint with seven alternating two-row side slots, six cells high.
/// Slots occur at Y=3+4*k; even slots remove X<24 and odd slots X>=7.
/// Ordered cells give stable boundary traversal without random seeds.
fn concave_cells() -> BTreeSet<Cell> {
    let mut cells = BTreeSet::new();
    for x in 0..31 {
        for y in 0..31 {
            let removed = (0..7).any(|slot| {
                let slot_y = 3 + slot * 4;
                (slot_y..slot_y + 2).contains(&y)
                    && if slot % 2 == 0 { x < 24 } else { x >= 7 }
            });
            if !removed {
                for z in 0..6 {
                    cells.insert([x, y, z]);
                }
            }
        }
    }
    cells
}

/// Fills diagonal-only occupancy patterns around grid edges to a deterministic fixed point.
///
/// Scans axes and quadrants in fixed order, collecting the first absent adjacent
/// cell for each two-opposite-quadrant contact, then applies additions together.
/// The finite original bounding cube is unchanged, so monotone additions terminate.
/// Empty input returns empty. This modifies the represented voxel set and removes
/// this edge pattern only; it does not prove all vertex links or embedding.
/// Intended for small fixture coordinates: extent arithmetic is unchecked and
/// no total-work or memory budget is imposed.
fn manifoldize_voxels(mut cells: BTreeSet<Cell>) -> BTreeSet<Cell> {
    let extent = cells
        .iter()
        .flat_map(|cell| cell.iter())
        .copied()
        .max()
        .map_or(0, |maximum| maximum + 1);
    loop {
        let mut additions = BTreeSet::new();
        for axis in 0..3 {
            let first = (axis + 1) % 3;
            let second = (axis + 2) % 3;
            for along in 0..extent {
                for first_grid in 0..=extent {
                    for second_grid in 0..=extent {
                        let quadrants =
                            [(false, false), (true, false), (true, true), (false, true)].map(
                                |(first_high, second_high)| {
                                    let first_coordinate = if first_high {
                                        Some(first_grid)
                                    } else {
                                        first_grid.checked_sub(1)
                                    };
                                    let second_coordinate = if second_high {
                                        Some(second_grid)
                                    } else {
                                        second_grid.checked_sub(1)
                                    };
                                    first_coordinate
                                        .zip(second_coordinate)
                                        .filter(|(a, b)| *a < extent && *b < extent)
                                        .map(|(a, b)| {
                                            let mut cell = [0; 3];
                                            cell[axis] = along;
                                            cell[first] = a;
                                            cell[second] = b;
                                            cell
                                        })
                                },
                            );
                        let occupied = quadrants
                            .map(|cell| cell.is_some_and(|cell| cells.contains(&cell)));
                        if (occupied == [true, false, true, false]
                            || occupied == [false, true, false, true])
                            && let Some(cell) = quadrants
                                .into_iter()
                                .flatten()
                                .find(|cell| !cells.contains(cell))
                        {
                            additions.insert(cell);
                        }
                    }
                }
            }
        }
        if additions.is_empty() {
            return cells;
        }
        cells.extend(additions);
    }
}

/// Selects cells in `[0,3^level)^3` by the ternary foam rule.
///
/// Discards a cell when any ternary digit position has at least two coordinates
/// equal to one. Level zero retains the unit cell. Does not repair diagonal edge
/// contacts; callers may explicitly modify the selection before meshing. Intended
/// for small levels: exponent arithmetic and cubic enumeration have no budget guard.
fn sierpinski_foam_cells(level: u32) -> BTreeSet<Cell> {
    let extent = 3_usize.pow(level);
    let mut cells = BTreeSet::new();
    for x in 0..extent {
        for y in 0..extent {
            for z in 0..extent {
                let mut digits = [x, y, z];
                let mut retained = true;
                for _ in 0..level {
                    let centered_axes = digits
                        .iter()
                        .filter(|coordinate| **coordinate % 3 == 1)
                        .count();
                    if centered_axes >= 2 {
                        retained = false;
                        break;
                    }
                    for coordinate in &mut digits {
                        *coordinate /= 3;
                    }
                }
                if retained {
                    cells.insert([x, y, z]);
                }
            }
        }
    }
    cells
}

/// Emits two outward triangles for each exposed unit-cell face in deterministic order.
///
/// Neighboring occupied cells suppress their shared face; integer corner keys
/// share one position row. The fixed face order and diagonal define serialization.
/// Requires corner addition to fit usize and every resulting coordinate to fit
/// u32 before the existing cast; the small fixtures satisfy this domain. Returns
/// empty buffers for no cells and does not itself validate topology or repair
/// contacts. Exactness is relative to the supplied voxel set.
fn voxel_boundary(cells: BTreeSet<Cell>) -> TriangleMesh {
    /// Neighbor offsets and outward counterclockwise corner loops for the six faces.
    /// Each loop is triangulated along its 0-to-2 diagonal; order is deterministic.
    const FACES: [([isize; 3], [[usize; 3]; 4]); 6] = [
        ([-1, 0, 0], [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0]]),
        ([1, 0, 0], [[1, 0, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1]]),
        ([0, -1, 0], [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]]),
        ([0, 1, 0], [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]]),
        ([0, 0, -1], [[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]]),
        ([0, 0, 1], [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]]),
    ];

    let mut position_indices = BTreeMap::<Cell, usize>::new();
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for cell in &cells {
        for (neighbor_offset, corners) in FACES {
            let neighbor = [
                cell[0].checked_add_signed(neighbor_offset[0]),
                cell[1].checked_add_signed(neighbor_offset[1]),
                cell[2].checked_add_signed(neighbor_offset[2]),
            ];
            if let [Some(x), Some(y), Some(z)] = neighbor
                && cells.contains(&[x, y, z])
            {
                continue;
            }
            let corners = corners.map(|offset| {
                let coordinate = [
                    cell[0] + offset[0],
                    cell[1] + offset[1],
                    cell[2] + offset[2],
                ];
                *position_indices.entry(coordinate).or_insert_with(|| {
                    let index = positions.len();
                    positions.push(Point3::new(
                        Real::from(coordinate[0] as u32),
                        Real::from(coordinate[1] as u32),
                        Real::from(coordinate[2] as u32),
                    ));
                    index
                })
            });
            triangles.push(Triangle::new(corners[0], corners[1], corners[2]));
            triangles.push(Triangle::new(corners[0], corners[2], corners[3]));
        }
    }
    TriangleMesh::new(positions, triangles)
}

/// Exports a fixture and installs it only when the existing UTF-8 bytes differ.
///
/// Creates the target fixture directory, writes a process-ID-suffixed temporary
/// file, then renames it to the destination. Returns the final path. Export or
/// write/rename failure panics; a rename failure can leave the temporary file.
/// There is no explicit synchronization, same-process concurrent-writer isolation,
/// cancellation cleanup or durability sync. Callers supply fixed local filenames.
fn install(filename: &str, mesh: &TriangleMesh, object_name: &str) -> PathBuf {
    let directory = fixture_directory();
    fs::create_dir_all(&directory).unwrap_or_else(|error| {
        panic!(
            "failed to create generated benchmark corpus {}: {error}",
            directory.display()
        )
    });
    let path = directory.join(filename);
    let serialized = csgrs::io::obj::to_obj(mesh, object_name)
        .expect("integer generated corpus is OBJ representable");
    if fs::read_to_string(&path).is_ok_and(|existing| existing == serialized) {
        return path;
    }
    let temporary = path.with_extension(format!("obj.part-{}", std::process::id()));
    fs::write(&temporary, serialized).unwrap_or_else(|error| {
        panic!(
            "failed to serialize generated benchmark corpus {}: {error}",
            temporary.display()
        )
    });
    fs::rename(&temporary, &path).unwrap_or_else(|error| {
        panic!(
            "failed to install generated benchmark corpus {}: {error}",
            path.display()
        )
    });
    path
}

/// Selects `benchmark-fixtures/generated` beneath the effective target directory.
/// This only computes the path; installation creates it and retains generated files.
fn fixture_directory() -> PathBuf {
    target_directory().join("benchmark-fixtures/generated")
}

/// Resolves runtime CARGO_TARGET_DIR, or defaults to the compiled manifest's target.
///
/// Absolute overrides are used directly; relative overrides are joined to the
/// runtime working directory, whose lookup may panic. Paths are not canonicalized.
fn target_directory() -> PathBuf {
    match env::var_os("CARGO_TARGET_DIR") {
        Some(directory) => {
            let directory = PathBuf::from(directory);
            if directory.is_absolute() {
                directory
            } else {
                env::current_dir()
                    .expect("benchmark working directory is available")
                    .join(directory)
            }
        },
        None => Path::new(env!("CARGO_MANIFEST_DIR")).join("target"),
    }
}
