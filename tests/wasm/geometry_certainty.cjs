// Run against wasm-bindgen's actual Node.js output, not a mocked native loader.
// Usage: node tests/wasm/geometry_certainty.cjs target/properties-wasm/csgrs.js
const assert = require("node:assert/strict");
const path = require("node:path");
const wasm = require(path.resolve(process.argv[2] || "target/properties-wasm/csgrs.js"));
const {
  CurveRegionJs, MeshJs, Point3Js, PlaneJs, Matrix4Js,
  GeometryCertaintyJs: Certainty, GeometryQueryStatusJs: Status,
} = wasm;

// Keep ownership explicit, including exception paths in these checks.
const owned = [];
const keep = (value) => { owned.push(value); return value; };
try {
  const empty = keep(new CurveRegionJs());
  const square = keep(CurveRegionJs.square(1));
  for (const [construct, code] of [
    [() => CurveRegionJs.square(-1), "invalid_input"],
    [() => CurveRegionJs.rectangle(0, -1), "invalid_input"],
    [() => CurveRegionJs.circle(0, 2), "invalid_input"],
    [() => CurveRegionJs.ellipse(0, -1, 4), "invalid_input"],
    [() => CurveRegionJs.rightTriangle(1, -1), "invalid_input"],
    [() => CurveRegionJs.regularNgon(2, 1), "invalid_input"],
    [() => CurveRegionJs.circle(1, 0xffffffff), "resource_exhausted"],
    [() => CurveRegionJs.square(NaN), "invalid_input"],
    [() => CurveRegionJs.ellipse(1, Infinity, 4), "invalid_input"],
  ]) {
    assert.throws(construct, (error) => error instanceof Error
      && error.name === "GeometryError" && error.code === code
      && error.message.length > 0);
  }
  for (const approximate of [false, true]) {
    assert.throws(() => CurveRegionJs.ring(1, -1, 4, approximate), (error) =>
      error instanceof Error && error.code === "invalid_input");
    assert.throws(() => CurveRegionJs.crescent(1, 2, 0, 6, approximate), (error) =>
      error instanceof Error && error.code === "invalid_input");
  }
  const zeroSquare = keep(CurveRegionJs.square(0));
  const recoveredCurve = keep(zeroSquare.union(square, false));
  assert.equal(recoveredCurve.certainty, Certainty.Certified);
  // intoRegion consumes the wrapper; unregister it before transferring ownership.
  owned.pop();
  const recoveredRegion = keep(recoveredCurve.intoRegion());
  assert.equal(recoveredRegion.containsXY(0.5, 0.5), true);
  assert.equal(recoveredRegion.containsXY(2, 0.5), false);
  for (const approximate of [false, true]) {
    for (const [x, y, value, status] of [
      [0.5, 0.5, true, Status.Decided],
      [-1, 0.5, false, Status.Decided],
      [0, 0.5, undefined, Status.Boundary],
    ]) {
      const result = keep(square.containsXYWithContext(x, y, approximate));
      assert.equal(result.status, status);
      assert.equal(result.value, value);
      assert.equal(result.certainty, Certainty.Certified);
      assert.equal(result.reason, undefined);
      assert.equal(result.approximate512Consumed, false);
    }
    const result = keep(empty.containsXYWithContext(0, 0, approximate));
    assert.equal(result.status, Status.Decided);
    assert.equal(result.value, false);
    assert.equal(result.certainty, Certainty.Certified);
  }
  assert.equal(empty.containsXY(0, 0), false);
  assert.equal(square.containsXY(0, 0.5), undefined);
  assert.throws(() => square.containsXYWithContext(NaN, 0, false));

  // Opposite rotations retain an exact identity that bounded predicates cannot
  // certify at this angle. A warm terminal answer must not satisfy STRICT.
  const rotatedSquare = keep(square.rotate(13));
  const restoredSquare = keep(rotatedSquare.rotate(-13));
  assert.throws(() => restoredSquare.containsXY(0.5, 0));
  for (const approximate of [false, true, false]) {
    const result = keep(restoredSquare.containsXYWithContext(0.5, 0, approximate));
    assert.equal(result.value, undefined);
    assert.equal(result.status, approximate ? Status.Boundary : Status.Uncertain);
    assert.equal(result.certainty, approximate ? Certainty.Approximate512Consumed : undefined);
    assert.equal(result.approximate512Consumed, approximate);
    assert.equal(typeof result.reason, approximate ? "undefined" : "string");
  }

  const union = keep(square.union(square, false));
  assert.equal(union.certainty, Certainty.Certified);
  const bounds = keep(square.boundingBoxWithContext(false));
  assert.equal(bounds.certainty, Certainty.Certified);
  assert.equal(bounds.isEmpty, false);
  assert.equal(keep(bounds.min).x, 0);
  assert.equal(keep(bounds.max).x, 1);
  const emptyCurveBounds = keep(empty.boundingBoxWithContext(false));
  assert.equal(emptyCurveBounds.isEmpty, true);
  assert.equal(emptyCurveBounds.min, undefined);
  assert.equal(emptyCurveBounds.max, undefined);
  assert.equal(emptyCurveBounds.certainty, Certainty.Certified);
  assert.equal(square.toRegionProfiles(false).certainty, Certainty.Certified);
  const arrays = square.toArrays(false);
  assert.equal(arrays.certainty, Certainty.Certified);
  assert.equal(arrays.indices.length, 6);

  // Derived native coordinates retain the unit width beyond binary64 spacing.
  // Display projection is separate; translate the solid back before inspecting it.
  const distantSquare = keep(square.translate(2 ** 53, 0, 0));
  for (const approximate of [false, true]) {
    for (const height of [NaN, Infinity, -Infinity]) {
      assert.throws(() => distantSquare.extrude(height, approximate), (error) =>
        error instanceof Error && error.name === "GeometryError"
        && error.code === "invalid_input" && error.message.includes("finite"));
    }
    assert.throws(() => distantSquare.extrude(0, approximate), (error) =>
      error instanceof Error && error.name === "GeometryError"
      && error.code === "invalid_input" && error.message.includes("nonzero"));
    const extrusion = keep(distantSquare.extrude(2, approximate));
    assert.equal(extrusion.certainty, Certainty.Certified);
    owned.pop();
    const distantSolid = keep(extrusion.intoMesh());
    const restoredSolid = keep(distantSolid.translate(-(2 ** 53), 0, 0));
    const exactBounds = restoredSolid.boundingBox();
    assert.equal(keep(exactBounds.min).x, 0);
    const maximum = keep(exactBounds.max);
    assert.equal(maximum.x, 1);
    assert.equal(maximum.z, 2);
    assert.equal(distantSquare.toArrays(approximate).certainty, Certainty.Certified);
  }

  const cube = keep(MeshJs.cube(1));
  const crossing = keep(cube.translate(-0.5, 0, 0));
  const projective = keep(new Matrix4Js(0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0));
  for (const [operation, code] of [
    [() => cube.scale(0, 1, 1), "invalid_input"],
    [() => cube.scaleWithContext(0, 1, 1, true), "invalid_input"],
    [() => cube.translate(NaN, 0, 0), "invalid_input"],
    [() => cube.rotateWithContext(0, Infinity, 0, false), "invalid_input"],
    [() => cube.transform(projective), "unbounded"],
    [() => crossing.transform(projective), "unbounded"],
    [() => crossing.transformWithContext(projective, false), "unbounded"],
    [() => crossing.transformWithContext(projective, true), "unbounded"],
  ]) {
    assert.throws(operation, (error) => error instanceof Error
      && error.name === "GeometryError" && error.code === code
      && error.message.length > 0);
  }
  const finiteProjective = keep(keep(cube.translate(1, 0, 0)).transform(projective));
  assert.equal(finiteProjective.vertexCount(), 8);
  for (const [construct, code] of [
    [() => MeshJs.cube(NaN), "invalid_input"],
    [() => MeshJs.sphere(Infinity, 8, 4), "invalid_input"],
    [() => MeshJs.cube(-1), "invalid_input"],
    [() => MeshJs.cuboid(0, -1, 1), "invalid_input"],
    [() => MeshJs.sphere(0, 2, 4), "invalid_input"],
    [() => MeshJs.sphere(1, 0xffffffff, 2), "resource_exhausted"],
    [() => MeshJs.torus(1, 1, 8, 4), "unsupported"],
  ]) {
    assert.throws(construct, (error) => error instanceof Error
      && error.name === "GeometryError" && error.code === code
      && error.message.length > 0);
  }
  const zeroCube = keep(MeshJs.cube(0));
  assert.equal(zeroCube.vertexCount(), 0);
  assert.equal(zeroCube.triangleCount(), 0);
  assert.equal(keep(zeroCube.union(cube)).vertexCount() > 0, true);
  const inside = keep(new Point3Js(0.5, 0.5, 0.5));
  const member = keep(cube.containsPointWithContext(inside, false));
  assert.equal(member.status, Status.Decided);
  assert.equal(member.value, true);
  assert.equal(member.certainty, Certainty.Certified);
  const emptyMesh = keep(new MeshJs());
  assert.equal(emptyMesh.boundingBox(), undefined);
  const emptyBounds = keep(emptyMesh.boundingBoxWithContext(false));
  assert.equal(emptyBounds.isEmpty, true);
  assert.equal(emptyBounds.min, undefined);
  assert.equal(emptyBounds.max, undefined);
  assert.equal(emptyBounds.certainty, Certainty.Certified);
  const placedEmpty = keep(keep(emptyMesh.center()).floatOnZ());
  assert.equal(placedEmpty.vertexCount(), 0);
  assert.equal(keep(placedEmpty.union(cube)).containsPoint(inside), true);
  const origin = keep(MeshJs.fromArrays(new Float64Array([0, 0, 0]), new Uint32Array()));
  const originBounds = keep(origin.boundingBoxWithContext(false));
  assert.equal(originBounds.isEmpty, false);
  assert.equal(keep(originBounds.min).x, 0);
  assert.equal(keep(originBounds.max).x, 0);
  const positioned = keep(keep(cube.center()).floatOnZ());
  const positionedBounds = positioned.boundingBox();
  assert.equal(keep(positionedBounds.min).x, -0.5);
  assert.equal(keep(positionedBounds.max).z, 1);
  const openMesh = keep(MeshJs.fromArrays(
    new Float64Array([0, 0, 0, 1, 0, 0, 0, 1, 0]),
    new Uint32Array([0, 1, 2]),
  ));
  for (const approximate of [false, true]) {
    for (const operation of [
      "unionWithContext", "intersectionWithContext",
      "differenceWithContext", "xorWithContext",
    ]) {
      for (const emptyOnLeft of [false, true]) {
        const [left, right] = emptyOnLeft ? [emptyMesh, cube] : [cube, emptyMesh];
        const result = keep(left[operation](right, approximate));
        assert.equal(result.certainty, Certainty.Certified);
        owned.pop();
        const mesh = keep(result.intoMesh());
        const isEmpty = operation === "intersectionWithContext"
          || (operation === "differenceWithContext" && emptyOnLeft);
        assert.equal(mesh.containsPoint(inside), !isEmpty);
        if (isEmpty) {
          assert.equal(mesh.vertexCount(), 0);
          assert.equal(mesh.triangleCount(), 0);
          assert.equal(keep(mesh.union(cube)).containsPoint(inside), true);
        } else {
          assert.equal(keep(mesh.difference(cube)).isEmpty(), true);
        }
        const [invalidLeft, invalidRight] = emptyOnLeft
          ? [emptyMesh, openMesh] : [openMesh, emptyMesh];
        assert.throws(() => invalidLeft[operation](invalidRight, approximate),
          (error) => String(error).includes(
            `input mesh ${emptyOnLeft ? 1 : 0} has 3 boundary edges`));
      }
    }
  }
  assert.equal(keep(cube.boundingBoxWithContext(false)).certainty, Certainty.Certified);
  const rotatedCube = keep(cube.rotate(0, 0, 13));
  const restoredCube = keep(rotatedCube.rotate(0, 0, -13));
  const surfacePoint = keep(new Point3Js(0.5, 0, 0.5));
  for (const approximate of [false, true, false]) {
    const result = keep(restoredCube.containsPointWithContext(surfacePoint, approximate));
    assert.equal(result.status, approximate ? Status.Decided : Status.Unknown);
    assert.equal(result.value, approximate ? false : undefined);
    assert.equal(result.certainty, approximate ? Certainty.Approximate512Consumed : undefined);
    assert.equal(typeof result.reason, approximate ? "undefined" : "string");
  }
  assert.throws(() => MeshJs.fromArrays(new Float64Array([0, 1]), new Uint32Array()));
  assert.throws(() => MeshJs.fromArrays(new Float64Array([0, 0, 0]), new Uint32Array([0, 0])));
  assert.throws(() => MeshJs.fromArrays(new Float64Array([Infinity, 0, 0]), new Uint32Array()));
  assert.throws(() => MeshJs.fromArrays(new Float64Array([0, 0, 0]), new Uint32Array([0, 1, 2])));

  const body = keep(MeshJs.cuboid(12, 12, 4));
  const cylinder = keep(MeshJs.cylinder(2, 6, 16));
  const drill = keep(cylinder.translate(6, 6, -1));
  assert.throws(() => body.differenceWithContext(drill, false));
  const difference = keep(body.differenceWithContext(drill, true));
  assert.equal(difference.certainty, Certainty.Approximate512Consumed);
  assert.equal(difference.approximate512Consumed, true);
  // intoMesh consumes its result wrapper, so release that wrapper from owned.
  owned.pop();
  const mesh = keep(difference.intoMesh());
  assert.equal(mesh.triangleCount(), 144);

  const planeResult = keep(PlaneJs.FromComponentsWithContext(0, 0, 0, 1, 0, 0, 0, 1, 0, false));
  assert.equal(planeResult.certainty, Certainty.Certified);
  owned.pop();
  const plane = keep(planeResult.intoPlane());
  assert.equal(keep(plane.reflectionMatrixWithContext(false)).certainty, Certainty.Certified);
  console.log("WASM certainty, query status, geometry, and invalid-input checks passed");
} finally {
  for (const value of owned.reverse()) value.free();
}
