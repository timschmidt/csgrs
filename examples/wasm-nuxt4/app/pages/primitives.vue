<template>
  <div>
    <Viewer 
      :meshes="meshes" 
      :metrics="metrics"></Viewer>
  </div>
</template>

<script setup lang="ts">

import { ref } from 'vue';

import type { CsgRsMeshArrays } from '../../types';
import { useCsgrsJs } from '../../composables/useCsgrsJs';
const { loadWasm } = useCsgrsJs();

import Viewer from '../../components/Viewer.vue';

const meshes = ref<Array<CsgRsMeshArrays>>();
const metrics = ref<Record<string, number>>({});

// Ensure this code runs only on the client side
if (import.meta.client)
{
  const GRID_NUM_COLS = 3;
  const GRID_SIZE = 30;

  await loadWasm().then((csgrs) => 
  {
      const timeStartGeneration = performance.now();

      const primitiveMeshes = [ 
        csgrs.MeshJs.cuboid(15,20,25).translate(-7.5,-10,-12.5), // center at origin
        csgrs.MeshJs.sphere(15,32,32),
        csgrs.MeshJs.cylinder(10,10,30,32),
        csgrs.MeshJs.octahedron(15),
        csgrs.MeshJs.icosahedron(15),
        csgrs.MeshJs.egg(15,20, 16,16),
        csgrs.MeshJs.torus(10,3,16,16),
        csgrs.MeshJs.spur_gear_involute(2, 10, 20, 0, 0, 5, 5), // JS name!
        csgrs.MeshJs.teardrop(20,30,16,20).rotate(90,0,0),
        //csgrs.MeshJs.arrow([0,0,0], [0,0,30], 2, 5, 16),
        //csgrs.MeshJs.gyroid(10,3,16,16), // not bound yet
      ].map((p,i) => 
      {
          const row = Math.floor(i / GRID_NUM_COLS);
          const col = i % GRID_NUM_COLS;
          const bbox = p.boundingBox();
          const bboxWidth = bbox.max[0] - bbox.min[0];
          //console.log(row);
          console.log(col);
          console.log(bboxWidth);
          console.log(`col:${col}, row:${row}`);
          
          return p.translate((col) *GRID_SIZE, (row) *GRID_SIZE, 0)
                .to_arrays() as CsgRsMeshArrays;
      });
      
      meshes.value = primitiveMeshes;
      metrics.value['generation'] = performance.now() - timeStartGeneration;

  });
}
  

</script>