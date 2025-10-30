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

      const mainMesh = csgrs.MeshJs.cuboid(30, 30, 40).translate(-15,-15,-25);

      // Define animation
      const animationDuration = 5000; // 5 seconds in milliseconds
      const animationStart = performance.now();
      
      const animateFrame = (currentTime: number) => 
      {
        const timeStartGeneration = performance.now();

        const elapsed = currentTime - animationStart;
        const progress = (elapsed % animationDuration) / animationDuration; // 0 to 1, repeating
        const angle = progress * Math.PI * 2; // 0 to 2π
        
        // Calculate circular position (radius of 30 units)
        const radius = 15;
        const x = Math.cos(angle) * radius;
        const y = Math.sin(angle) * radius;
        const z = Math.sin(angle * 2) * 20; // Up and down motion
        
        // Create new box at animated position
        const movingSubBox = csgrs.MeshJs.cuboid(25,25,25)
                            .translate(x - 10, y - 10, z-10);

        const movingSubOcta = csgrs.MeshJs.octahedron(15)
                            .translate(x,0,0);

        const movingAddBox = csgrs.MeshJs.cuboid(15,15,15)
                            .translate(-x-10, -y-10, z-10);

        const result = mainMesh
                        .difference(movingSubBox)
                        .union(movingAddBox)
                        .difference(movingSubOcta);

        meshes.value = [result.to_arrays() as CsgRsMeshArrays];
        metrics.value['generation per frame'] = performance.now() - timeStartGeneration;
        
        requestAnimationFrame(animateFrame);

      };
      
      animateFrame(performance.now()); // Start the animation
      
  });
}
  

</script>