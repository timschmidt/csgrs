"""Freeze a native nonzero-winding query using existing topology machinery."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent;ROOT=WORK.parents[2]
SOURCE=WORK/'candidate-source-v1';OUT=WORK/'candidate-preparation-v1'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert json.loads((WORK/'baseline-v3/result.json').read_text())['complete']
with gzip.open(WORK/'baseline-v3/current-input-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(ROOT/p)==h for p,h in inputs.items())
SOURCE.mkdir(exist_ok=False);OUT.mkdir(exist_ok=False)
for p,h in inputs.items():
 out=SOURCE/p;out.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/p,out);assert sha(out)==h
for p in ['tests/containment_winding.rs','tests/wasm/containment_winding.cjs']:
 shutil.copy2(WORK/'prototype-v2'/p,SOURCE/p)
# Move the existing signed-ray loop into its native owning module, preserving
# cancellation calls, direction order, checked sums and isolated receipts.
p=SOURCE/'crates/hypermesh/src/minkowski.rs';s=p.read_text()
start=s.index('fn winding_at_cell_point(');old=s[start:];assert old.rstrip().endswith('}')
body=old[old.index('    // Each face-parallel'):]
body=body.replace('The centroid is\n    // strictly inside an arrangement cell, hence on no source supporting plane.',
 'The caller excludes\n    // source triangles from the query point. Coplanar points outside a triangle\n    // cannot create a forward hit on it; parallel rays are retried.')
body=body.replace('options.check_cancelled()?','check_cancelled()?')
body=body.replace('Minkowski winding','mesh winding').replace('Minkowski cell witness','mesh winding query').replace('Minkowski cell winding','mesh point winding')
helper='''/// Winding of a validated closed PWN source away from its triangles.
///
/// Reuses exact oriented crossings without deduplicating multiplicity. A
/// complete direction supplies its own certainty; abandoned proof attempts
/// cannot qualify that independent completed proof. The finite direction count
/// is not a total-work, heap-byte or scalar-growth bound.
pub(crate) fn mesh_winding_number_decision(
    decisions: &DecisionContext,
    source: &TriangleMesh,
    point: &Point3,
    mut check_cancelled: impl FnMut() -> HypermeshResult<()>,
) -> HypermeshResult<i32> {
'''+body
s=s[:start].rstrip()+'\n'
s=s.replace('use crate::winding::{RayFacetRelation, ray_triangle_relation};','use crate::winding::mesh_winding_number_decision;')
s=s.replace('TriangleMesh, Vector3, boolean,','TriangleMesh, boolean,')
old='if winding_at_cell_point(decisions, &source, &centroid, options)? != 0 {'
new='''if mesh_winding_number_decision(decisions, &source, &centroid, || options.check_cancelled())? != 0 {'''
assert s.count(old)==1;s=s.replace(old,new);p.write_text(s)
p=SOURCE/'crates/hypermesh/src/winding.rs';s=p.read_text();s=s.replace('use crate::{Classification, Point3, Vector3};','use crate::{Classification, Point3, Real, TriangleMesh, Vector3};')
marker='#[cfg(test)]\nmod tests {';assert s.count(marker)==1;s=s.replace(marker,helper+'\n'+marker);p.write_text(s)
p=SOURCE/'crates/hypermesh/src/mesh.rs';s=p.read_text();s=s.replace('use crate::storage_hash::StorageHashMap;','use crate::storage_hash::StorageHashMap;\nuse crate::winding::mesh_winding_number_decision;')
s=s.replace('''    /// Tests strict solid containment using exact boundary predicates and ray
    /// parity. Points on a triangle, edge, or vertex are not inside.''','''    /// Tests strict interior of the regularized nonzero-winding solid.
    ///
    /// Oriented crossings retain winding multiplicity and cancellation. Points
    /// on the represented set's boundary are excluded; internal source faces,
    /// edges and vertices follow the represented set's interior. Source-face
    /// queries reuse the native Boolean arrangement to establish that boundary.''')
old='''    /// rules as explicit validation.
    pub fn contains_point('''
new='''    /// rules as explicit validation.
    ///
    /// Off-surface winding uses at most `8n + 1` candidate directions for `n`
    /// triangles. Boundary regularization uses the native arrangement. These
    /// algorithms have no total-work, heap-byte or cancellation limit; the ray
    /// count does not bound scalar expression growth or arrangement work.
    pub fn contains_point('''
assert s.count(old)==1;s=s.replace(old,new)
start=s.index('    pub(crate) fn contains_point_decision(')
query_start=s.index('        let query = hyperlimit::Point3::new',start)
direction_start=s.index('        let direction = Vector3::from_xyz(',query_start)
end=s.index('    /// Intersects consecutive polyline segments',direction_start)
surface=s[query_start:direction_start];assert surface.count('return Ok(false);')==1
surface=surface.replace('return Ok(false);','return Ok(true);')
replacement='''        if self.point_on_surface_decision(decisions, point)? {
            let regularized = crate::boolean(
                &MeshContext::new(decisions.policy()),
                &[self.as_ref()],
                crate::BooleanProgram::Operation(crate::BooleanOp::Union),
            )?;
            decisions.absorb(regularized.certainty);
            let normalized = regularized.value.into_triangle_meshes()?.pop().ok_or(
                HypermeshError::SurfaceArrangementFailed {
                    reason: "containment regularization produced no result",
                },
            )?;
            if normalized.point_on_surface_decision(decisions, point)? {
                return Ok(false);
            }
            return Ok(mesh_winding_number_decision(decisions, &normalized, point, || Ok(()))? != 0);
        }
        Ok(mesh_winding_number_decision(decisions, self, point, || Ok(()))? != 0)
    }

    fn point_on_surface_decision(
        &self,
        decisions: &DecisionContext,
        point: &Point3,
    ) -> HypermeshResult<bool> {
'''+surface+'''        Ok(false)
    }

'''
s=s[:query_start]+replacement+s[end:];p.write_text(s)
p=SOURCE/'src/solid.rs';s=p.read_text();s=s.replace('''/// Points on source triangles are excluded by the native strict containment
/// query. This convenience entry point selects the strict predicate policy.''','''/// This is strict interior of the regularized nonzero-winding solid: internal
/// source faces follow the represented set, and the external set boundary is
/// excluded. This convenience entry point selects the strict predicate policy.
/// See [`TriangleMesh::contains_point`] for the native resource contract.''');p.write_text(s)
p=SOURCE/'src/wasm/mesh_js.rs';s=p.read_text();old='''    /// Strict containment: canonical empty geometry returns false. Malformed
    /// closed-solid input throws GeometryError with code invalid_input, even''';new='''    /// Strict interior of the regularized nonzero-winding solid. Internal
    /// source faces follow represented-set membership; its boundary is excluded.
    /// Canonical empty geometry returns false. Malformed closed-solid input
    /// throws GeometryError with code invalid_input, even''';assert s.count(old)==1;s=s.replace(old,new);p.write_text(s)
p=SOURCE/'scripts/test-wasm-bindings.sh';s=p.read_text();old='node tests/wasm/mesh_errors.cjs "${wasm_bindings_dir}/csgrs.js"';assert s.count(old)==1;s=s.replace(old,old+'\nnode tests/wasm/containment_winding.cjs "${wasm_bindings_dir}/csgrs.js"');p.write_text(s)
changes=['crates/hypermesh/src/mesh.rs','crates/hypermesh/src/winding.rs','crates/hypermesh/src/minkowski.rs','src/solid.rs','src/wasm/mesh_js.rs','scripts/test-wasm-bindings.sh','tests/containment_winding.rs','tests/wasm/containment_winding.cjs']
subprocess.run(['cargo','fmt','--all'],cwd=SOURCE,check=True)
assert {p for p,h in inputs.items() if sha(SOURCE/p)!=h}<=set(changes)
for p in changes:inputs[p]=sha(SOURCE/p)
with gzip.open(OUT/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
dump(OUT/'result.json',{'complete':True,'base_commit':'d4eee9a675fa51d84eb0eb29503dd802bdcdbd19','runner_sha256':sha(__file__),'source':str(SOURCE),'source_inputs':len(inputs),'source_hashes_sha256':sha(OUT/'source-hashes.json.gz'),'changes':{p:inputs[p] for p in changes},'contract_sha256':sha(WORK/'prototype-v2/contract-v1.md'),'main_sources_modified':False,'diagnostic_migration':'Shared winding failures now use mesh winding labels instead of Minkowski-specific labels. Typed variants, checked overflow, direction schedule and Minkowski cancellation checks are preserved.'})
print('Frozen eight-file candidate with native shared winding and arrangement boundary queries',flush=True)
