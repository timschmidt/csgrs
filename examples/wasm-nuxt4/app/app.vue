<template>
  <div>
    <Viewer :meshes="meshes"></Viewer>
  </div>
</template>

<script setup lang="ts">

import { ref } from 'vue';

import Viewer from '../components/Viewer.vue';

import type { CsgRsMeshArrays, TresBufferGeometryAttributes } from '../types';

const meshes = ref<Array<CsgRsMeshArrays>>();

const loadWasm = async () => 
{
  const timeStartLoad = performance.now();
  const csgrs = (await import('../../../pkg/csgrs.js'));    
  console.log(`WASM load time: ${performance.now() - timeStartLoad} ms`);
  return csgrs;
}

// Ensure this code runs only on the client side
if (import.meta.client)
{
  await loadWasm().then((csgrs) => 
  {
    const timeStartGeneration = performance.now();
    // Create a triangle plane mesh from 3 vertices on XY plane
    // No need for decimal points
    const plane = new csgrs.PlaneJs(
      -50, 50, 0,
      50, 0, 0,
      0, 50, 0 
    );

    // Make a 2D Sketch
    const sketch = new csgrs.SketchJs();
    console.log(sketch.isEmpty()); // true
    
    /*
    sketch.polygon([
      -50, 50,
      50, 0,
      0, 50
    ]);
    */

    const circle = csgrs.SketchJs.circle(10, 32);
    // console.log(circle.center()); // Expect the center point, is center operation
    console.log(circle.boundingBox()); // { min: [-10,-10,0], max: [10,10,0] }
    const translatedCircle = circle.translate(100,0,0); // Expect in-place translation
    console.log(translatedCircle.boundingBox()); // { min: [90,-10,0], max: [110,10,0] }
    console.log(circle.toArrays().indices.length); // number of vertices for circle

    const extrudedCircle = circle.extrude(30);

    const sphere = csgrs.MeshJs.sphere(20, 64, 64).translate(20,0,0);
    const box = csgrs.MeshJs.cuboid(10,10,5).translate(0,0,25);
    const octahedron = csgrs.MeshJs.octahedron(10)
                    .rotate(30,0,0)
                    .translate(-10,5,20);
    
    const diffShape = extrudedCircle
                        .difference(sphere)
                        .difference(box)
                        .difference(octahedron);

    const meshData = diffShape.to_arrays() as CsgRsMeshArrays; // TODO: no TS typing from Rust

    console.log(`Geometry generation time: ${performance.now() - timeStartGeneration} ms`);
    
    meshes.value = [meshData];

  });
}
  

</script>