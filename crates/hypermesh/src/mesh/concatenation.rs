//! Structural mesh composition with checked indices and buffer capacities.

use super::*;

fn add_capacity<T>(
    total: usize,
    additional: usize,
    operation: &'static str,
) -> HypermeshResult<usize> {
    total
        .checked_add(additional)
        .filter(|length| {
            length
                .checked_mul(std::mem::size_of::<T>())
                .is_some_and(|bytes| bytes <= isize::MAX as usize)
        })
        .ok_or(HypermeshError::CapacityOverflow { operation })
}

impl TriangleMesh {
    /// Concatenates complete position and triangle buffers in input order.
    ///
    /// Every source index is validated before rebasing. Open surfaces, unused
    /// vertices, duplicate triangles and winding multiplicity are preserved;
    /// this operation neither computes a Boolean union nor validates topology.
    /// Existing closed-PWN facts are transferred only when every nonempty
    /// input has one for its current buffers, preserving the weakest certainty.
    ///
    /// Count/byte overflow and vector reservation failure return
    /// [`HypermeshError::CapacityOverflow`]. These checks are not total-work or
    /// heap-byte limits; scalar clones and shared-buffer conversion can still
    /// allocate through their existing infallible allocation paths.
    pub fn try_concatenate(meshes: &[Self]) -> HypermeshResult<Self> {
        let mut position_count = 0;
        let mut triangle_count = 0;
        let mut pwn_certainty = Some(MeshCertainty::Certified);
        for mesh in meshes {
            mesh.validate_triangle_indices()?;
            position_count = add_capacity::<Point3>(
                position_count,
                mesh.positions.len(),
                "mesh concatenation positions",
            )?;
            triangle_count = add_capacity::<Triangle>(
                triangle_count,
                mesh.triangles.len(),
                "mesh concatenation triangles",
            )?;
            if mesh.positions.is_empty() && mesh.triangles.is_empty() {
                continue;
            }
            let source = mesh
                .geometry_facts()
                .and_then(|facts| facts.valid_pwn.certainty());
            pwn_certainty = match (pwn_certainty, source) {
                (Some(MeshCertainty::Certified), Some(MeshCertainty::Certified)) => {
                    Some(MeshCertainty::Certified)
                }
                (Some(_), Some(_)) => Some(MeshCertainty::Approximate512Consumed),
                _ => None,
            };
        }
        let mut positions = Vec::new();
        positions.try_reserve_exact(position_count).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "mesh concatenation position reservation",
            }
        })?;
        let mut triangles = Vec::new();
        triangles.try_reserve_exact(triangle_count).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "mesh concatenation triangle reservation",
            }
        })?;
        for mesh in meshes {
            let base = positions.len();
            positions.extend(mesh.positions.iter().cloned());
            // Each index is below this input's length and every accumulated
            // position count fits the platform byte limit, so rebasing fits.
            triangles.extend(mesh.triangles.iter().map(|triangle| {
                Triangle::new(base + triangle.v0, base + triangle.v1, base + triangle.v2)
            }));
        }
        let result = Self::new(positions, triangles);
        if !result.triangles.is_empty()
            && let Some(certainty) = pwn_certainty
        {
            result.facts.valid_pwn.record(certainty);
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open() -> TriangleMesh {
        TriangleMesh::new(
            vec![
                Point3::origin(),
                Point3::new(1.into(), 0.into(), 0.into()),
                Point3::new(0.into(), 1.into(), 0.into()),
            ],
            vec![Triangle::new(0, 1, 2)],
        )
    }

    fn tetrahedron() -> TriangleMesh {
        TriangleMesh::new(
            vec![
                Point3::origin(),
                Point3::new(1.into(), 0.into(), 0.into()),
                Point3::new(0.into(), 1.into(), 0.into()),
                Point3::new(0.into(), 0.into(), 1.into()),
            ],
            vec![
                Triangle::new(0, 2, 1),
                Triangle::new(0, 1, 3),
                Triangle::new(0, 3, 2),
                Triangle::new(1, 2, 3),
            ],
        )
    }

    #[test]
    fn open_buffers_and_unused_points_preserve_exact_order_and_multiplicity() {
        let source = open();
        let point = TriangleMesh::new(vec![Point3::new(Real::pi(), 0.into(), 0.into())], vec![]);
        let result =
            TriangleMesh::try_concatenate(&[source.clone(), point.clone(), source.clone()])
                .unwrap();
        assert_eq!(
            result.positions.as_ref(),
            [
                source.positions.as_ref(),
                point.positions.as_ref(),
                source.positions.as_ref()
            ]
            .concat()
        );
        assert_eq!(
            result.triangles.as_ref(),
            &[Triangle::new(0, 1, 2), Triangle::new(4, 5, 6)]
        );
        assert_eq!(result.facts.valid_pwn.certainty(), None);
        let empty = TriangleMesh::try_concatenate(&[]).unwrap();
        assert!(empty.positions.is_empty() && empty.triangles.is_empty());
    }

    #[test]
    fn invalid_source_indices_cannot_alias_another_input_or_wrap() {
        for index in [1, usize::MAX] {
            let malformed =
                TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, index, 0)]);
            for inputs in [[open(), malformed.clone()], [malformed, open()]] {
                assert_eq!(
                    TriangleMesh::try_concatenate(&inputs),
                    Err(HypermeshError::VertexIndexOutOfBounds {
                        index,
                        vertex_count: 1
                    })
                );
            }
        }
    }

    #[test]
    fn count_and_byte_overflow_are_typed_before_reservation() {
        for (total, additional) in [
            (usize::MAX, 1),
            (0, usize::MAX),
            (isize::MAX as usize / std::mem::size_of::<Point3>(), 1),
        ] {
            assert_eq!(
                add_capacity::<Point3>(total, additional, "test positions"),
                Err(HypermeshError::CapacityOverflow {
                    operation: "test positions"
                })
            );
        }
    }

    #[test]
    fn concatenation_preserves_weakest_pwn_fact_and_allows_new_strict_proof() {
        let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let strict = tetrahedron();
        strict.validate_closed_pwn(&strict_context).unwrap();
        let approximate =
            tetrahedron().with_valid_pwn_certainty(MeshCertainty::Approximate512Consumed);
        let result = TriangleMesh::try_concatenate(&[strict, approximate]).unwrap();
        assert_eq!(
            result.facts.valid_pwn.certainty(),
            Some(MeshCertainty::Approximate512Consumed)
        );
        let decisions = DecisionContext::new(&strict_context);
        assert!(!result.facts.valid_pwn.consume(&decisions));
        assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        let proof = polygon_soup(&strict_context, &[result.as_ref()]).unwrap();
        assert_eq!(proof.certainty, MeshCertainty::Certified);
        assert_eq!(
            result.facts.valid_pwn.certainty(),
            Some(MeshCertainty::Certified)
        );
    }

    #[test]
    fn edited_sources_do_not_transfer_stale_pwn_facts() {
        let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let original = tetrahedron();
        polygon_soup(&context, &[original.as_ref()]).unwrap();
        let mut edited = original.clone();
        Arc::make_mut(&mut edited.positions)[3] = Point3::origin();
        let result = TriangleMesh::try_concatenate(&[edited]).unwrap();
        assert_eq!(result.facts.valid_pwn.certainty(), None);
        assert!(matches!(
            polygon_soup(&context, &[result.as_ref()]),
            Err(HypermeshError::DegenerateTriangle { .. })
        ));
        let mut edited = original.clone();
        Arc::make_mut(&mut edited.triangles)[0].v0 = usize::MAX;
        assert!(matches!(
            TriangleMesh::try_concatenate(&[edited]),
            Err(HypermeshError::VertexIndexOutOfBounds { .. })
        ));
        let preserved =
            TriangleMesh::try_concatenate(&[TriangleMesh::new(vec![], vec![]), original]).unwrap();
        assert_eq!(
            preserved.facts.valid_pwn.certainty(),
            Some(MeshCertainty::Certified)
        );
    }
}
