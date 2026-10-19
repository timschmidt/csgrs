// The runner compiles and executes this beside the generated declarations.
import { MeshJs, CurveRegionJs, GeometryCertaintyJs, ConstructionEvidenceJs } from './csgrs';
function check(condition: boolean): void { if (!condition) throw new Error('construction evidence assertion failed'); }
function known(mesh: MeshJs): void {
  const evidence: ConstructionEvidenceJs = mesh.constructionEvidence;
  check(!evidence.sourceHistoryMissing);
  check(evidence.predicateCertainty === GeometryCertaintyJs.Certified);
  evidence.free();
}
const cube = MeshJs.cube(1);
known(cube);
const outcome = cube.convexHullWithContext(true);
check(outcome.certainty === GeometryCertaintyJs.Certified);
const extracted = outcome.intoMesh();
known(extracted);
const moved = extracted.translate(3, 5, 7);
const adjacent = cube.translate(4, 5, 7);
const union = moved.union(adjacent);
known(union);
const artifact = union.toStlBinaryArtifact();
const evidence = artifact.constructionEvidence;
check(evidence.coordinatesProjected && !evidence.sourceHistoryMissing);
check(evidence.predicateCertainty === GeometryCertaintyJs.Certified);
evidence.free();
const detached = artifact.bytes;
detached.fill(0); // This copy cannot change the bound artifact or its receipt.
const plain = MeshJs.fromSTL(artifact.bytes);
const missing = plain.constructionEvidence;
check(missing.sourceHistoryMissing && missing.predicateCertainty === undefined);
missing.free();
const restored = artifact.intoMesh();
known(restored);
const empty = restored.difference(restored);
known(empty);
const region = CurveRegionJs.square(1);
const extrusion = region.extrude(1, false).intoMesh();
const regionEvidence = extrusion.constructionEvidence;
check(regionEvidence.sourceHistoryMissing);
regionEvidence.free();
for (const mesh of [cube, extracted, moved, adjacent, union, plain, restored, empty, extrusion]) mesh.free();
region.free();
console.log('TypeScript bound artifacts, extraction, missing history and native evidence passed');
