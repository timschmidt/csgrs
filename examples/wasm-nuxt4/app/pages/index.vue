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
      const timeStartGeneration = performance.now();
      
      const box = csgrs.MeshJs.cuboid(20,20,20).translate(-10,-10,-10); // left,front,bottom at origin
      const sphere = csgrs.MeshJs.sphere(12.5,16,16); // center at origin
      const booleanResult = box.difference(sphere);
      meshes.value = [booleanResult.to_arrays() as CsgRsMeshArrays] ; // TODO: fix in lib

      const timeEndGeneration = performance.now();
      metrics.value['generation'] = timeEndGeneration - timeStartGeneration;
  });
}
  

</script>