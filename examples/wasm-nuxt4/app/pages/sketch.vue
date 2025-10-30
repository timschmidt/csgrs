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
  await loadWasm().then((csgrs) => 
  {
      const MARGIN = 10;

      const timeStartGeneration = performance.now();
      
      let placeCursorX = 0;
      const sketches = [
        csgrs.SketchJs.polygon([[-20, -10],[20, -10],[15, 10],[-15, 20]]),
        csgrs.SketchJs.rectangle(40, 20),
        csgrs.SketchJs.circle(15, 32),
      ].map((sketch, i, arr) => 
      {
          const curBbox = sketch.boundingBox();
          const translatedSketchMesh = sketch.translate(placeCursorX - curBbox.min[0], 0,0)

          placeCursorX += curBbox.max[0] - curBbox.min[0] + MARGIN;
          return translatedSketchMesh;
      });

      // Extrude in multiple ways
      const extrudedShapes = sketches.map(
      (s,i) =>
      {
          if((i+1) % 2 === 0)
          {  
            // index = 0, 3, 6, ...
            return s.extrudeVector(10,-10,10);   
          }
          else if ((i+1) % 3 === 0)
          {
              //const pivot = s.center(); // centers - not center of mass
              const pivot = [(s.boundingBox().min[0] + s.boundingBox().max[0])/2,
                             (s.boundingBox().min[1] + s.boundingBox().max[1])/2,
                             0];
              // really could use exposed Vector3/Point3
              return s.sweep(
                        [[0,0,0],[0,0,10],[0,5,20],[-5,-5,30]] // TODO: check BUGS
                      );
          }
          else
          {
              return s.extrude(30);
          }
      }).map((ex) => ex.translate(0, 50, 0));

      meshes.value = [
        ...sketches.map((s) => s.toArrays()),
        ...extrudedShapes.map((s) => s.to_arrays()) // TODO: fix in lib
      ] as Array<CsgRsMeshArrays>; 
      
      metrics.value['generated'] = timeStartGeneration - performance.now();

  });
}
  

</script>